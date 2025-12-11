# TKJHAT SDK Overview

The TKJHAT is an educational add-on board for the Raspberry Pi Pico / Pico 2 to be used within in the course Computer Systems and Introduction to Computer Systems in University of Oulu.  
This SDK provides a minimal C API for working with the onboard sensors and actuators.

---

## Actuators

| Device        | Interface | Notes |
|---------------|-----------|-------|
| Red LED       | GPIO 14   | Onboard indicator LED (also referred to as “onboard LED”) |
| RGB LED       | GPIO 18:R, 19:G, 20:B | Common-anode LED, driven via PWM |
| Buzzer        | GPIO 17   | Digital output, simple square-wave control |
| OLED display (SSD1306) | I²C (0x3C) | 128×64 pixels, text & graphics via [SSD1306 datasheet](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf) |

---

## Sensors

| Sensor        | Interface | Address | Datasheet |
|---------------|-----------|---------|-----------|
| Ambient light sensor (VEML6030) | I²C | 0x10 | [Vishay VEML6030](https://www.vishay.com/docs/84366/veml6030.pdf) |
| Temperature & humidity (HDC2021) | I²C | 0x40 | [TI HDC2021](https://www.ti.com/lit/ds/symlink/hdc2021.pdf) |
| IMU (ICM-42670, accel+gyro) | I²C | 0x69 | [TDK ICM-42670](https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf) |
| PDM MEMS Microphone | GPIO 16 (DATA), GPIO 15 (CLK) | — | Uses PIO + [Arm Developer Pico microphone library](https://github.com/ArmDeveloperEcosystem/microphone-library-for-pico/tree/main) |

---

## Notes

- The default I²C bus uses SDA = GPIO 12 and SCL = GPIO 13.  
- The SDK is intended for teaching: APIs are simplified, and defaults (e.g. 100 Hz ODR, ±4 g accelerometer) are chosen to be practical.  

---

## Authors

- Raisul Islam  
- Iván Sánchez Milara

## Contributors
- Emil Kelhala - Help in initial testing. 

© 2025 — University of Oulu, released under the MIT License.
