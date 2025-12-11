#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include <tusb.h>

#include "usbSerialDebug/helper.h"

static SemaphoreHandle_t g_log_mtx;
static const TickType_t wait = pdMS_TO_TICKS(5);
static const TickType_t io_timeout = pdMS_TO_TICKS(10);

static inline bool cdc0_ready(void) {
    return tud_mounted() && tud_cdc_n_connected(0);
}

static SemaphoreHandle_t g_log_mtx;
bool usb_serial_init(void) {
    g_log_mtx = xSemaphoreCreateMutex();
    return g_log_mtx != NULL;
}


void usb_serial_flush(void) {
    if (!cdc0_ready()) return;

    // Try to take the mutex for not interfering with current writing
    if (g_log_mtx && xSemaphoreTake(g_log_mtx, 0) == pdTRUE) {
        tud_cdc_n_write_flush(0);
        xSemaphoreGive(g_log_mtx);
    } else {
        // If we do not get instantly we ask for a flush. 
        // TinyUSB handle it in a safe way (hopefully).
        tud_cdc_n_write_flush(0);
    }
}

bool usb_serial_connected(void){
    return cdc0_ready();
}

int usb_serial_print(const char *s) {
    if (!s) {
        return -1;
    }
    if ( !tud_mounted() || !tud_cdc_connected()) 
        return 0;
    
    if (xSemaphoreTake(g_log_mtx, wait) != pdTRUE) 
        return 0;

    size_t initial, n = strlen(s);

    TickType_t deadline = xTaskGetTickCount() + io_timeout;

    while (n) {
        uint32_t avail = tud_cdc_write_available();
        if (avail) {
            uint32_t chunk = (n < avail) ? (uint32_t)n : avail;
            tud_cdc_write(s, chunk);
            tud_cdc_write_flush();
            s += chunk; 
            n -= chunk;
        } 
        else {
            if (io_timeout == 0 || (int32_t)(xTaskGetTickCount() - deadline) >= 0) {
                xSemaphoreGive(g_log_mtx);
                return false; // give up to avoid blocking too long
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    xSemaphoreGive(g_log_mtx);
    return initial-n;
}