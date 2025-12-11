/*
Version 0.8

MIT License

Copyright (c) 2025 Raisul Islam, Iván Sánchez Milara

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/* =========================
 *  Board / pin macros
 * ========================= */

#define DEFAULT_I2C_SDA_PIN                     12
#define DEFAULT_I2C_SCL_PIN                     13

#define DEFAULT_UART_0                          0
#define DEFAULT_UART_1                          1

#define SW1_PIN                                 02
#define SW2_PIN                                 22
#define BUTTON1                                 SW1_PIN
#define BUTTON2                                 SW2_PIN

#define RED_LED_PIN                             14
#define LED1                                    RED_LED_PIN

#define RGB_LED_R                               18
#define RGB_LED_G                               19
#define RGB_LED_B                               20

#define BUZZER_PIN                              17

#define PDM_DATA                                16
#define PDM_CLK                                 15

#define VEML6030_INTERRUPT                      9
#define HDC2021_INTERRUPT                       21
#define ICM42670_INT                            6
