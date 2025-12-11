// Group members' names: Hoa Ly, Jonathan Tran, Kien Quoc Vu
//
// JTKJ Morse Communicator for Raspberry Pi Pico with TKJ HAT
//
// This application implements a Morse code communication device using a Raspberry Pi Pico
// and a custom TKJ HAT. It allows users to input Morse code through two different methods:
// 1.  An Inertial Measurement Unit (IMU) by tilting the device.
// 2.  A PDM microphone by making sounds of different durations.
//
// The application uses the FreeRTOS real-time operating system to manage multiple concurrent
// tasks, ensuring responsive handling of user input, serial communication, and feedback
// mechanisms like an OLED display, buzzer, and RGB LED. It also includes Wi-Fi
// connectivity to demonstrate network integration.
//




// --- Standard C Libraries ---
#include <stdio.h>      // Standard Input/Output operations (e.g., printf)
#include <string.h>     // String manipulation functions (e.g., strcmp, strlen)
#include <math.h>       // Mathematical functions (e.g., fabs for absolute float value)
#include <ctype.h>      // Character handling functions (e.g., toupper)
#include <stdlib.h>     // General utilities, including abs() for integer absolute value




// --- Pico SDK Headers ---
#include "pico/bootrom.h"   // For programmatic rebooting into BOOTSEL mode
#include "pico/stdlib.h"    // Core Pico functions and definitions
#include "pico/stdio.h"     // Standard I/O support for Pico (e.g., over USB)
#include "pico/cyw43_arch.h"// Architecture-specific functions for the CYW43 Wi-Fi/BT chip




// --- FreeRTOS Headers ---
#include <FreeRTOS.h>   // Core FreeRTOS definitions
#include <queue.h>      // FreeRTOS queue API for inter-task communication
#include <task.h>       // FreeRTOS task management API
#include <semphr.h>     // FreeRTOS semaphore and mutex API




// --- Custom Hardware Abstraction Layer ---
#include "tkjhat/sdk.h" // TKJHAT SDK for interacting with the HAT's components (display, sensors, etc.)




// =================================================================================
// --- Wi-Fi and TCP Networking (Collaboration with Nguyen Group) ---
// =================================================================================
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"




// --- Network Configuration ---
#define TEST_TCP_SERVER_IP "51.20.8.40" // Public IP of the TCP server
#define TCP_PORT 8080                   // Port for the TCP server
#define WIFI_SSID "panoulu"             // Wi-Fi network name
#define WIFI_PASS NULL                  // Wi-Fi password (NULL for open networks)
#define BUF_SIZE 502                    // TCP buffer size
#define POLL_TIME_S 5                   // TCP polling time in seconds




// Structure to hold the state of the TCP client
typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    uint8_t buffer[BUF_SIZE];
    int buffer_len;
    int sent_len;
    bool complete;
    int run_count;
    bool connected;
} TCP_CLIENT_T;




// Global state for the TCP client
TCP_CLIENT_T *clientState = NULL;
// =================================================================================
// --- End of Networking Section ---
// =================================================================================








// =================================================================================
// --- APPLICATION CONFIGURATION CONSTANTS ---
// =================================================================================




// --- Button and Input Mode Configuration ---
#define BTN_MODE            BUTTON1 // SW1 on the HAT: Toggles input mode (IMU/MIC), long press sends message
#define BTN_ACTION          BUTTON2 // SW2 on the HAT: Confirms a symbol in IMU mode, long press sends a space
#define BTN_LONG_PRESS_MS   800     // Duration (in ms) to qualify a button press as a "long press"
#define BTN_DEBOUNCE_MS     150     // Time (in ms) to ignore further button changes to prevent bouncing
#define MIC_AMPL_THRESHOLD  8000    // The average absolute amplitude required for the microphone to detect sound
#define MIC_DOT_MAX_MS      250     // A sound duration less than or equal to this is a DOT; otherwise, it's a DASH
#define INPUT_TASK_PERIOD_MS 50     // The polling rate (in ms) for the main input task loop




// --- FreeRTOS Queue Configuration ---
#define SYMBOL_QUEUE_LENGTH     64   // Max number of Morse symbols ('.', '-', ' ') that can be buffered internally
#define EVENT_QUEUE_LENGTH      8    // Max number of application events (e.g., message sent) to buffer




// =================================================================================
// --- TYPE DEFINITIONS ---
// =================================================================================




// Defines the possible input methods for Morse code
typedef enum {
    INPUT_MODE_IMU = 0, // Input via the Inertial Measurement Unit (tilting)
    INPUT_MODE_MIC = 1  // Input via the PDM microphone (sound)
} input_mode_t;




// Defines the states for the microphone input logic
typedef enum {
    MIC_STATE_IDLE = 0,     // The microphone is waiting for a sound loud enough to cross the threshold
    MIC_STATE_ACTIVE        // The microphone has detected a sound and is measuring its duration
} mic_state_t;




// Defines internal application events for inter-task signaling
typedef enum {
    APP_EVENT_MSG_SENT = 0 // An event indicating that a message has been successfully sent (e.g., by 3 spaces)
} app_event_t;




// =================================================================================
// --- GLOBAL VARIABLES ---
// =================================================================================
static input_mode_t g_inputMode = INPUT_MODE_IMU; // Global variable to track the current input mode, starts with IMU
static volatile int g_mic_samples_ready = 0;      // A flag (set by an ISR) to indicate that new microphone samples are ready
static int16_t      g_mic_buffer[MEMS_BUFFER_SIZE]; // Buffer to store the raw audio samples from the microphone




// --- FreeRTOS Handles ---
// These are handles to the queues and semaphores created in main()
static QueueHandle_t xSymbolQueue = NULL; // Internal queue to pass Morse symbols from the input task to the TX/playback tasks
static QueueHandle_t xEventQueue  = NULL; // Queue for high-level application events, like signaling the buzzer task
QueueHandle_t i2cMutex;             // A mutex to prevent concurrent access to the I2C bus (used by IMU and display)
QueueHandle_t xPlaybackQueue;       // Queue for symbols that need local feedback (display/sound)








// --- Microphone Interrupt Service Routine (ISR) Callback ---
// This function is called automatically by the PDM library when a buffer of samples is full.
static void on_pdm_samples_ready(void) {
    // Attempt to read the microphone samples into the global buffer
    int n = get_microphone_samples(g_mic_buffer, MEMS_BUFFER_SIZE);
    if (n > 0) {
        // If samples were read successfully, set the flag for the input task to process them
        g_mic_samples_ready = n;
    }
}




// --- Task Function Prototypes ---
// Forward declarations for the functions that will run as FreeRTOS tasks
static void input_task(void *pvParameters); // Handles all user input: buttons, IMU, and microphone
static void vBuzzerTask(void *pvParameters); // Plays sounds based on application events
static void serial_tx_task(void *pvParameters); // Task to send Morse symbols over serial and TCP
static void serial_rx_task(void *pvParameters); // Task to receive and process data from serial
static void playback_task(void *pvParameters);  // Task to provide local feedback for Morse symbols








// =================================================================================
// --- CONSTANTS AND DATA STRUCTURES ---
// =================================================================================




// --- Task Priority ---
#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 2) // Sets a priority level for all application tasks




// --- Queue Lengths ---
#define PLAYBACK_QUEUE_LENGTH 40  // Max number of symbols to buffer for local feedback (display, LED, buzzer)
#define SERIAL_RX_BUFFER_SIZE 128 // Max characters for the serial receive buffer




// --- IMU Thresholds ---
#define IMU_TILT_THRESHOLD              0.9f    // g-force value (approx 0.9g) for tilt input
#define IMU_ROTATION_ACTIVE_THRESHOLD   220.0f  // Gyro value for an active rotation (twist/nod)
#define IMU_ROTATION_IDLE_THRESHOLD     50.0f   // Gyro value to ensure other axes are relatively still
#define IMU_DEBOUNCE_MS                 300     // Debounce time for IMU actions




// --- IMU State Machine ---
// This state machine controls the logic for generating symbols from the IMU.
volatile enum AppState {
  STATE_IDLE,       // Waiting for the user to "arm" the IMU with a button press
  STATE_ARMED,      // IMU is armed. The next tilt will generate a symbol.
  STATE_COOLDOWN    // A symbol has just been generated. Waiting for the device to return to neutral.
} g_appState = STATE_IDLE;








// --- Morse Code to Alphabet Translation Table ---
// This array maps Morse code strings to their corresponding letters and numbers.
struct MorseAlphabet {
  char morseCode[7]; // String for the Morse code (e.g., ".-")
  char letter;       // The corresponding character (e.g., 'a')
};




// The full Morse code alphabet used for translation
struct MorseAlphabet morseCodes[40] = {
  {".-", 'a'}, {"-...", 'b'}, {"-.-.", 'c'}, {"-..", 'd'}, {".", 'e'},
  {"..-.", 'f'}, {"--.", 'g'}, {"....", 'h'}, {"..", 'i'}, {".---", 'j'},
  {"-.-", 'k'}, {".-..", 'l'}, {"--", 'm'}, {"-.", 'n'}, {"---", 'o'},
  {".--.", 'p'}, {"--.-", 'q'}, {".-.", 'r'}, {"...", 's'}, {"-", 't'},
  {"..-", 'u'}, {"...-", 'v'}, {".--", 'w'}, {"-..-", 'x'}, {"-.--", 'y'},
  {"--..", 'z'}, {"-----", '0'}, {".----", '1'}, {"..---", '2'},
  {"...--", '3'}, {"....-", '4'}, {".....", '5'}, {"-....", '6'},
  {"--...", '7'}, {"---..", '8'}, {"----.", '9'}, {".-.-.-", '.'},
  {"--..--", ','}, {"..--..", '?'}, {"-.-.--", '!'}, {"", ' '}
};








// =================================================================================
// --- FUNCTION PROTOTYPES (Continued) ---
// =================================================================================
char find_letter_from_morse_code(char *morseCode); // Utility to translate Morse string to a character
void process_received_line(char *line);            // Utility to process a complete line received via serial
void run_tcp_client_test(void);
void send_data_tcp(const char *msg);




// --- Helper Functions ---
// A simple wrapper to send a character symbol to the internal symbol queue.
static void send_symbol(char c) {
    xQueueSend(xSymbolQueue, &c, 0); // Use 0 as the timeout (don't block)
}




// A helper to send a space symbol, used for separating letters and words.
static void send_space(void) {
    char space = ' ';
    send_symbol(space);
}




// Plays a simple 3-note melody on the buzzer to indicate a message was sent.
static void play_message_sent_melody(void) {
    buzzer_play_tone(880, 150); // A5 note
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(660, 150); // E5 note
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(440, 300); // A4 note
}




// =================================================================================
// --- MAIN FUNCTION ---
// =================================================================================
int main() {
    // 1. Initialize Standard I/O
    stdio_init_all();




    // Add a small delay to wait for the USB serial connection to be established.
    int i = 0;
    while (!stdio_usb_connected() && i < 30) { // Wait max 3 seconds
        sleep_ms(100);
        i++;
    }
   
    printf("__JTKJ Morse Communicator - Starting...__\n");




    // 2. Initialize Wi-Fi and TCP Client (Collaboration with Nguyen Group)
    printf("Initializing Wi-Fi...\n");
    if (cyw43_arch_init()) {
        printf("WiFi init failed!\n");
    }




    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);
    if(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_OPEN, 30 * 1000)) {
        printf("Failed to connect to Wi-Fi\n");
    } else {
        printf("Connected to Wi-Fi\n");
    }




    printf("Starting TCP Client...\n");
    run_tcp_client_test();
    // --- End of Networking Init ---




    // 3. Initialize HAT Hardware
    init_hat_sdk();
    init_led();  
    init_rgb_led();
    init_buzzer();
    init_display();




    // 4. Initialize On-board Sensors (IMU and Microphone)
    if (init_ICM42670() == 0) { // Initialize the IMU
        printf("__IMU INIT OK__\n");
        if (ICM42670_start_with_default_values() != 0) { // Start it with default settings
            printf("__IMU START DEFAULT FAILED__\n");
        }
    } else {
        printf("__IMU INIT FAILED__\n");
    }




    if (init_pdm_microphone() == 0) { // Initialize the microphone
        pdm_microphone_set_callback(on_pdm_samples_ready); // Register the ISR callback
        if (init_microphone_sampling() == 0) { // Start sampling
            printf("__MIC INIT OK__\n");
        } else {
            printf("__MIC START FAILED__\n");
        }
    } else {
        printf("__MIC INIT FAILED__\n");
    }




    // 5. Display a startup message on the OLED
    clear_display();
    write_text_xy(0, 0, "IMU Mode Ready");
    write_text_xy(0, 10, "SW1=Mode, SW2=Action");
    write_text_xy(0, 30, "RX MSG:");




    // 6. Create FreeRTOS Queues and Mutexes
    xPlaybackQueue = xQueueCreate(PLAYBACK_QUEUE_LENGTH, sizeof(char));
    i2cMutex = xSemaphoreCreateMutex(); // To protect the I2C bus
    xSymbolQueue = xQueueCreate(SYMBOL_QUEUE_LENGTH, sizeof(char)); // Internal symbol queue
    xEventQueue  = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(app_event_t)); // App event queue




    // Check if all queues and the mutex were created successfully. If not, halt.
    if (xPlaybackQueue == NULL || i2cMutex == NULL || xSymbolQueue == NULL || xEventQueue == NULL) {
        printf("__CRITICAL ERROR: Could not create queues or mutex__\n");
        while (1) { blink_led(1); sleep_ms(100); } // Blink LED to indicate fatal error
    }




    // 7. Create FreeRTOS Tasks
    // Each task is a separate thread of execution.
    // Tasks are created with MAIN_TASK_PRIORITY.
    xTaskCreate(input_task, "InputTask", 4096, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_tx_task, "TxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_rx_task, "RxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(playback_task, "PlaybackTask", 4096, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(vBuzzerTask, "BuzzerTask", 1024, NULL, MAIN_TASK_PRIORITY, NULL);




    printf("__Initialization complete. Starting scheduler.__\n");




    // 8. Start the FreeRTOS Scheduler
    // This function starts the real-time operating system and begins executing the created tasks.
    // Code execution should never proceed past this point.
    vTaskStartScheduler();




    while (1); // This line should be unreachable.
    return 0;
}




// =================================================================================
// --- INPUT TASK ---
// This is the most complex task. It handles all user inputs:
// - Polling the two buttons for short and long presses.
// - Reading the IMU sensor to detect tilts.
// - Processing microphone data to detect sounds.
// - Switching between IMU and Microphone input modes.
// - Sending generated Morse symbols to the internal `xSymbolQueue`.
// =================================================================================
static void input_task(void *pvParameters) {
    (void)pvParameters; // Unused parameter




    // --- Local variables for IMU state ---
    float ax, ay, az, gx, gy, gz, t; // To store sensor readings
    bool read_ok_imu = false; // Flag to check if IMU read was successful




    // --- Local variables for Button state ---
    bool       sw1_prev = false, sw2_prev = false; // Previous state of the buttons
    TickType_t sw1_press_tick = 0, sw2_press_tick = 0; // Tick count when buttons were pressed
    TickType_t sw1_last_change = 0, sw2_last_change = 0; // Tick count of the last state change for debouncing




    // --- Local variables for Microphone state ---
    mic_state_t micState = MIC_STATE_IDLE; // Current state of the microphone logic
    TickType_t  micStartTick = 0; // Tick count when a sound was first detected




    // Initialize the GPIO for the buttons
    init_sw1();
    init_sw2();




    g_inputMode = INPUT_MODE_IMU; // Start in IMU mode by default
    g_appState = STATE_IDLE; // Start completely idle (not reading sensor)
    printf("__IMU MODE: IDLE__\n");




    // --- Main Task Loop ---
    while (1) {
        TickType_t now = xTaskGetTickCount(); // Get the current time in system ticks




        // Read the current state of the buttons
        bool sw1_now = gpio_get(BTN_MODE) ? true : false;
        bool sw2_now = gpio_get(BTN_ACTION) ? true : false;








        // --- SW1 (Mode Button) Logic with Debouncing ---
        if (sw1_now != sw1_prev && (now - sw1_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
            sw1_last_change = now; // Update the last change time
            if (sw1_now) { // Button was just PRESSED
                buzzer_play_tone(1000, 80); // Short beep for feedback
                sw1_press_tick = now; // Record the press time
            } else { // Button was just RELEASED
                TickType_t press_duration = now - sw1_press_tick; // Calculate duration
               
                if (press_duration >= pdMS_TO_TICKS(BTN_LONG_PRESS_MS)) {
                    // LONG PRESS on SW1: Send message (3 spaces)
                    printf("__MSG SEND VIA 3 SPACES__\n");
                    send_space(); send_space(); send_space();
                    // Send an event to the buzzer task to play a confirmation melody
                    app_event_t evt = APP_EVENT_MSG_SENT;
                    xQueueSend(xEventQueue, &evt, 0);
                } else {
                    // SHORT PRESS on SW1: Toggle input mode
                    if (g_inputMode == INPUT_MODE_IMU) {
                        g_inputMode = INPUT_MODE_MIC;
                        printf("__MIC MODE__\n");
                        micState = MIC_STATE_IDLE; // Reset mic state
                        g_appState = STATE_IDLE;   // Reset IMU state
                        set_led_status(false);     // Ensure LED is off
                        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            clear_display();
                            write_text_xy(0, 0, "MIC Mode Ready");
                            write_text_xy(0, 10, "SW1=Mode");
                            write_text_xy(0, 30, "RX MSG:");
                            xSemaphoreGive(i2cMutex);
                        }
                    } else {
                        g_inputMode = INPUT_MODE_IMU;
                        printf("__IMU MODE__\n");
                        g_appState = STATE_IDLE; // Reset IMU state
                        set_led_status(false); // Turn off LED
                        // Update Display
                        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            clear_display();
                            write_text_xy(0, 0, "IMU Mode Ready");
                            write_text_xy(0, 10, "SW1=Mode, SW2=Action");
                            write_text_xy(0, 30, "RX MSG:");
                            xSemaphoreGive(i2cMutex);
                        }
                    }
                }
            }
        }
        sw1_prev = sw1_now; // Update previous state for next loop








        // --- SW2 (Action Button) Logic with Debouncing ---
        if (sw2_now != sw2_prev && (now - sw2_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
            sw2_last_change = now;
            if (sw2_now) { // Button was just PRESSED
                buzzer_play_tone(700, 80); // Different beep for SW2
                sw2_press_tick = now;
            } else { // Button was just RELEASED
                TickType_t press_duration = now - sw2_press_tick;
                if (press_duration >= pdMS_TO_TICKS(BTN_LONG_PRESS_MS)) {
                    // LONG PRESS on SW2: Send a space (for separating letters/words)
                    send_space();
                    printf("__SPACE SENT__\n");
                    // Reset state just in case
                    g_appState = STATE_IDLE;
                    set_led_status(false); // Turn off LED
                }
                else {
                    // SHORT PRESS on SW2: Action depends on the current mode
                    if (g_inputMode == INPUT_MODE_IMU) {
                        if (g_appState == STATE_IDLE) {
                            // If idle, arm the IMU to prepare for a tilt gesture
                            g_appState = STATE_ARMED;
                            set_led_status(true); // Turn on LED to indicate armed state
                            printf("__IMU ARMED: Reading...__\n");
                        } else {
                            // If already armed and pressed again, cancel/reset
                            g_appState = STATE_IDLE;
                            set_led_status(false);
                            printf("__IMU CANCELLED -> IDLE__\n");
                        }
                    }
                }
            }
        }
        sw2_prev = sw2_now;




        // --- IMU State Machine Processing ---
        // --- Check for Rotational Gestures if ARMED ---
        if (g_inputMode == INPUT_MODE_IMU && g_appState == STATE_ARMED) {
            read_ok_imu = false;
           
            // Try to take the mutex for a short time to not block the loop
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                    read_ok_imu = true;
                }
                xSemaphoreGive(i2cMutex);
            }


            if (read_ok_imu) {
                char symbol_to_send = 0;


                    // Check for DOT gesture: A sharp "twist" around the Z-axis
                    if (fabs(gz) > IMU_ROTATION_ACTIVE_THRESHOLD && fabs(gx) < IMU_ROTATION_IDLE_THRESHOLD && fabs(gy) < IMU_ROTATION_IDLE_THRESHOLD) {
                        printf("__Twist Detected (Gz=%.1f) -> DOT__\n", gz);
                    symbol_to_send = '.';
                }
                    // Check for DASH gesture: A sharp "nod" around the Y-axis
                    else if (fabs(gy) > IMU_ROTATION_ACTIVE_THRESHOLD && fabs(gx) < IMU_ROTATION_IDLE_THRESHOLD && fabs(gz) < IMU_ROTATION_IDLE_THRESHOLD) {
                        printf("__Nod Detected (Gy=%.1f) -> DASH__\n", gy);
                    symbol_to_send = '-';
                }
                // Check for significant tilt along Z or Y axes
                else if (az > IMU_TILT_THRESHOLD) {
                    symbol_to_send = '.'; // Lay down -> DOT
                }
                else if (ay < -IMU_TILT_THRESHOLD) {
                    symbol_to_send = '-'; // Stand up -> DASH
                }


                    // If a gesture was detected, send the symbol and enter cooldown
                if (symbol_to_send != 0) {
                    send_symbol(symbol_to_send);
                   
                    // Stop reading sensor. Release Bus.
                    set_led_status(false);      // Turn off LED
                    g_appState = STATE_IDLE;
                    printf("__Sent '%c' -> Going IDLE__\n", symbol_to_send);
                   
                    // Add a small delay so it don't accidentally double-trigger
                    // if the sensor value lingers for 10ms
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
        }




        // --- Microphone Input Processing ---
        // This part runs when in MIC mode and the ISR has signaled that new samples are available.
        if (g_inputMode == INPUT_MODE_MIC && g_mic_samples_ready > 0) {
             int sample_count = g_mic_samples_ready;
            g_mic_samples_ready = 0; // Reset the flag immediately




            // Calculate the average absolute amplitude of the audio samples
            int64_t sumAbs = 0;
            for (int i = 0; i < sample_count; i++) {
                sumAbs += abs(g_mic_buffer[i]);
            }
            uint32_t avgAbs = (uint32_t)(sumAbs / sample_count);




            // Microphone State Machine
            switch (micState) {
                case MIC_STATE_IDLE:
                    // If idle and a loud sound is detected, switch to active state
                    if (avgAbs > MIC_AMPL_THRESHOLD) {
                        micState = MIC_STATE_ACTIVE;
                        micStartTick = now; // Record the start time of the sound
                        printf("__MIC ACTIVE (Threshold hit)__\n");
                    }
                    break;
                case MIC_STATE_ACTIVE:
                    // If active and the sound level drops, the sound has ended
                    if (avgAbs <= MIC_AMPL_THRESHOLD) {
                        uint32_t durationMs = (now - micStartTick) * portTICK_PERIOD_MS;
                        char symbol = (durationMs <= MIC_DOT_MAX_MS) ? '.' : '-'; // Determine if it was a dot or dash
                       
                        if(symbol == '.') printf("__MIC DOT (%lu ms)__\n", (unsigned long)durationMs);
                        else printf("__MIC DASH (%lu ms)__\n", (unsigned long)durationMs);
                        send_symbol(symbol);
                        micState = MIC_STATE_IDLE; // Return to idle state
                    }
                    break;
            }
        }
        // Delay to control the loop rate of this task
        vTaskDelay(pdMS_TO_TICKS(INPUT_TASK_PERIOD_MS));
    }
}




// =================================================================================
// --- SERIAL TX TASK (with TCP) ---
// (Collaboration with Nguyen Group)
// This task waits for Morse symbols on the `xSymbolQueue`, sends them to the
// USB serial port, and handles the logic for message termination (3 spaces).
// =================================================================================
static void serial_tx_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    int spaceCount = 0; // Counter for consecutive spaces
    char txBuffer[128] = {0};
    int txIndex = 0;



    while (1) {
        // Block indefinitely until a symbol is received from the input task
        if (xQueueReceive(xSymbolQueue, &symbol, portMAX_DELAY) == pdPASS) {
            if (symbol == ' ') {
                spaceCount++;
                if (txIndex < 127) txBuffer[txIndex++] = ' ';
                putchar(' ');  // Always output the space

                if (spaceCount == 3) {
                    // Three consecutive spaces signifies the end of a message
                    txBuffer[txIndex] = '\0';
                    send_data_tcp(txBuffer); // Send buffer over TCP
                    putchar('\n'); // Send a newline character
                    printf("\n__[Morse Send OK]__\n"); // Print confirmation
                    spaceCount = 0; // Reset counter
                   
                    // Send a newline to the playback task to clear its display buffer
                    char nl_char = '\n';
                    xQueueSend(xPlaybackQueue, &nl_char, 0);
                    // Trigger the buzzer melody for message sent
                    app_event_t evt = APP_EVENT_MSG_SENT;
                    xQueueSend(xEventQueue, &evt, 0);
                    txIndex = 0;
                    memset(txBuffer, 0, sizeof(txBuffer));
                }
            } else {
                // Any non-space symbol resets the space counter
                spaceCount = 0;
                if (txIndex < 127) txBuffer[txIndex++] = symbol;
                putchar(symbol);
                xQueueSend(xPlaybackQueue, &symbol, 0);
            }
            fflush(stdout); // Ensure the character is sent immediately
        }
    }
}




// =================================================================================
// --- SERIAL RX TASK ---
// This task continuously polls the USB serial for incoming characters, assembles
// them into lines, and passes complete lines to process_received_line.
// =================================================================================
//
//  AI-ASSISTED DEVELOPMENT NOTE (Author: Hoa Ly)
//            and call process_received_line().
//  How I modified and integrated this code:
//  - I chose my own buffer size constant SERIAL_RX_BUFFER_SIZE and index limit
//    checks (linePos < SERIAL_RX_BUFFER_SIZE - 1) to prevent overflow.
//  - I added the linePos = 0 reset in the overflow case so that a corrupted
//    line does not break the following lines.
//  - I integrated this function into our Morse project by calling our own
//    process_received_line(lineBuffer), which parses Morse symbols and commands.
//  - I kept the non-blocking pattern with getchar_timeout_us(0) but tuned the
//    vTaskDelay(pdMS_TO_TICKS(10)) value so it cooperates nicely with other
//    FreeRTOS tasks in the application.
//  In summary, the AI gave me a standard serial line reader pattern, and I
//  adapted it to use our constants, call our Morse-specific process_received_line(),
//  and fit into the timing constraints of the rest of the system.
// -----------------------------------------------------------------------------
static void serial_rx_task(void *pvParameters) {
    (void)pvParameters;
    char lineBuffer[SERIAL_RX_BUFFER_SIZE];
    int linePos = 0;
    while (1) {
        int c = getchar_timeout_us(0); // Non-blocking read
        if (c != PICO_ERROR_TIMEOUT) {
            // If a newline is received, the line is complete
            if (c == '\n' || c == '\r') {
                if (linePos > 0) {
                    lineBuffer[linePos] = '\0'; // Null-terminate the string
                    process_received_line(lineBuffer);
                    linePos = 0; // Reset for the next line
                }
            } else if (linePos < (SERIAL_RX_BUFFER_SIZE - 1)) {
                // Add the character to the buffer
                lineBuffer[linePos++] = (char)c;
            } else {
                // Buffer overflow, discard the line
                linePos = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield the CPU
    }
}




// =================================================================================
// --- PROCESS RECEIVED LINE ---
// This utility function processes a line of text received from the serial port.
// It handles special commands and extracts Morse code for playback.
// =================================================================================
void process_received_line(char *line) {
    // --- Handle special commands ---
    if (strncmp(line, ".clear", 6) == 0) { // Clear the serial console
        printf("\033[2J\033[H");
        fflush(stdout);
        return;
    }
    if (strncmp(line, ".boot", 5) == 0) { // Reboot into BOOTSEL mode for programming
        printf("__Rebooting to BOOTSEL mode...__\n");
        sleep_ms(100);
        reset_usb_boot(0, 0);
        return;
    }
    if (strncmp(line, ".exit", 5) == 0) { // Halt the processor
        printf("\n__--- .exit command received. Halting. ---__\n");
        fflush(stdout);
        while (1) { blink_led(1); sleep_ms(200); }
    }

    // --- Parse the line for Morse symbols, skipping debug messages in "__" ---
    bool in_debug_block = false;
    for (int i = 0; line[i] != '\0'; i++) {
        if (line[i] == '_' && line[i+1] == '_') {
            in_debug_block = !in_debug_block; // Toggle flag
            i++;
            continue;
        }
        // If not in a debug block, send valid Morse symbols to the playback queue
        if (!in_debug_block) {
            char symbol = line[i];
            if (symbol == '.' || symbol == '-' || symbol == ' ') {
                xQueueSend(xPlaybackQueue, &symbol, portMAX_DELAY);
            }
        }
    }
    // Send a newline to the playback task to finalize the received message
    char nl = '\n';
    xQueueSend(xPlaybackQueue, &nl, portMAX_DELAY);
}




// =================================================================================
// --- PLAYBACK TASK ---
// This task provides local feedback for Morse symbols (both sent and received).
// It controls the RGB LED, the buzzer, and updates the OLED display.
// =================================================================================
static void playback_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    char morseSymbolBuffer[10] = {0}; // Buffer to build a Morse code string for one letter
    int morseSymbolPos = 0;
    char textMessageBuffer[22] = {0}; // Buffer to build the translated text message
    int textMessagePos = 0;




    while (1) {
        // Block until a symbol is received on the playback queue
        if (xQueueReceive(xPlaybackQueue, &symbol, portMAX_DELAY) == pdPASS) {
            // --- Provide feedback based on the symbol ---
            if (symbol == '.') {
                rgb_led_write(255, 255, 0); // Yellow for dot
                buzzer_play_tone(880, 150); // High pitch tone
                vTaskDelay(pdMS_TO_TICKS(150));
                if (morseSymbolPos < 9) morseSymbolBuffer[morseSymbolPos++] = '.';
            }
            else if (symbol == '-') {
                rgb_led_write(255, 0, 255); // Magenta for dash
                buzzer_play_tone(660, 400); // Lower pitch tone
                vTaskDelay(pdMS_TO_TICKS(400));
                if (morseSymbolPos < 9) morseSymbolBuffer[morseSymbolPos++] = '-';
            }
            // --- Handle SPACE (end of a letter) ---
            else if (symbol == ' ') {
                rgb_led_write(0, 255, 255); // Cyan for space
                vTaskDelay(pdMS_TO_TICKS(200));
                morseSymbolBuffer[morseSymbolPos] = '\0'; // Null-terminate Morse string
                char letter = find_letter_from_morse_code(morseSymbolBuffer); // Translate
                if (textMessagePos < 20) {
                    textMessageBuffer[textMessagePos++] = letter; // Add to message
                }
                morseSymbolPos = 0; // Reset for next letter
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
            }
            // --- Handle NEWLINE (end of a message) ---
            else if (symbol == '\n') {
                // Translate the final letter of the message
                morseSymbolBuffer[morseSymbolPos] = '\0';
                if (morseSymbolPos > 0) {
                    char letter = find_letter_from_morse_code(morseSymbolBuffer);
                    if (textMessagePos < 20) {
                        textMessageBuffer[textMessagePos++] = letter;
                    }
                }
                textMessageBuffer[textMessagePos] = '\0'; // Null-terminate the final message
               
                // Update the OLED display with the received message
                // Use the I2C mutex to prevent conflicts with the IMU task
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    clear_display();
                    // Redraw the static UI elements
                    if(g_inputMode == INPUT_MODE_IMU) write_text_xy(0, 0, "IMU Mode Ready");
                    else write_text_xy(0, 0, "MIC Mode Ready");
                    write_text_xy(0, 10, "SW1=Mode, SW2=Action");
                    write_text_xy(0, 30, "RX MSG:");
                    // Write the new message
                    if (textMessagePos > 0) {
                        write_text_xy(0, 40, textMessageBuffer);
                    }
                    xSemaphoreGive(i2cMutex); // Release the mutex
                }




                // --- Send feedback to serial client ---
                if (textMessagePos > 0) {
                    printf("\n__RX OK: %s__\n", textMessageBuffer);
                    fflush(stdout);
                }




                // Reset all buffers for the next message
                textMessagePos = 0;
                morseSymbolPos = 0;
                memset(textMessageBuffer, 0, sizeof(textMessageBuffer));
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
            }
            rgb_led_write(255, 255, 255); // Set LED to white after symbol feedback
            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay between symbols
        }
    }
}




// =================================================================================
// --- BUZZER TASK ---
// A simple task that waits for application events and plays corresponding sounds.
// =================================================================================
static void vBuzzerTask(void *pvParameters) {
     (void)pvParameters;
     app_event_t evt;
     while (1) {
         // Block until an event is received
         if (xQueueReceive(xEventQueue, &evt, portMAX_DELAY) == pdTRUE) {
             if (evt == APP_EVENT_MSG_SENT) {
                 // Play the "message sent" melody
                 play_message_sent_melody();
             }
         }
     }
}




// =================================================================================
// --- Wi-Fi and TCP Networking (Collaboration with Nguyen Group) ---
// =================================================================================




static err_t tcp_client_close(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    err_t err = ERR_OK;
    if (state->tcp_pcb != NULL) {
        tcp_arg(state->tcp_pcb, NULL);
        tcp_poll(state->tcp_pcb, NULL, 0);
        tcp_sent(state->tcp_pcb, NULL);
        tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb);
        if (err != ERR_OK) {
            printf("close failed %d, calling abort\n", err);
            tcp_abort(state->tcp_pcb);
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL;
    }
    return err;
}




static err_t tcp_result(void *arg, int status) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (status == 0) {
        printf("TCP test success\n");
    } else {
        printf("TCP test failed %d\n", status);
    }
    state->complete = true;
    return tcp_client_close(arg);
}




static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    printf("tcp_client_sent %u\n", len);
    state->sent_len += len;
    return ERR_OK;
}




static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (err != ERR_OK) {
        printf("connect failed %d\n", err);
        return tcp_result(arg, err);
    }
    state->connected = true;
    const char *msg = "Client Connected\n";
    tcp_write(tpcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
    return ERR_OK;
}




static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
    return ERR_OK;
}




static void tcp_client_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        printf("tcp_client_err %d\n", err);
        tcp_result(arg, err);
    }
}




err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    if (!p) {
        return tcp_result(arg, -1);
    }
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        const uint16_t buffer_left = BUF_SIZE - state->buffer_len;
        state->buffer_len += pbuf_copy_partial(p, state->buffer + state->buffer_len,
                                               p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);
    if (state->buffer_len > 0) {
        if(state->buffer_len < BUF_SIZE) state->buffer[state->buffer_len] = '\0';
        else state->buffer[BUF_SIZE-1] = '\0';
        printf("TCP RX: %s\n", state->buffer);
        state->buffer_len = 0;
    }
    return ERR_OK;
}




static bool tcp_client_open(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T*)arg;
    printf("Connecting to %s port %u\n", ip4addr_ntoa(&state->remote_addr), TCP_PORT);
    state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
    if (!state->tcp_pcb) {
        printf("failed to create pcb\n");
        return false;
    }
    tcp_arg(state->tcp_pcb, state);
    tcp_poll(state->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2);
    tcp_sent(state->tcp_pcb, tcp_client_sent);
    tcp_recv(state->tcp_pcb, tcp_client_recv);
    tcp_err(state->tcp_pcb, tcp_client_err);
    state->buffer_len = 0;
    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();
    return err == ERR_OK;
}




static TCP_CLIENT_T* tcp_client_init(void) {
    clientState = calloc(1, sizeof(TCP_CLIENT_T));
    if (!clientState) {
        printf("failed to allocate state\n");
        return NULL;
    }
    ip4addr_aton(TEST_TCP_SERVER_IP, &clientState->remote_addr);
    return clientState;
}




void run_tcp_client_test(void) {
    TCP_CLIENT_T *state = tcp_client_init();
    if (!state) {
        return;
    }
    if (!tcp_client_open(state)) {
        tcp_result(state, -1);
        return;
    }
}




void send_data_tcp(const char *msg) {
    if (clientState == NULL || !clientState->connected) {
        printf("__TCP: Not connected. Cannot send data.__\n");
        return;
    }
    printf("__Sending TCP: %s__\n", msg);
    cyw43_arch_lwip_begin();
    err_t err = tcp_write(clientState->tcp_pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
    if(err == ERR_OK) {
        tcp_output(clientState->tcp_pcb);
    } else {
        printf("__TCP Write Error: %d__\n", err);
    }
    cyw43_arch_lwip_end();
}




// =================================================================================
// --- FIND LETTER FROM MORSE CODE ---
// A utility function to look up a Morse code string in the `morseCodes` table
// and return the corresponding character.
// =================================================================================
char find_letter_from_morse_code(char *morseCode) {
    if (strlen(morseCode) == 0) return ' '; // An empty Morse string is a space
    // Iterate through the translation table
    for (int i = 0; i < 40; i++){
        if (strcmp(morseCode, morseCodes[i].morseCode) == 0){
            return (char)toupper(morseCodes[i].letter); // Return the uppercase letter
        }
    }
    return '?'; // Return '?' if the code is not found
}





