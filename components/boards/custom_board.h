/**
 * Copyright (c) 2017, Zaowubang, Inc
 *
 * All rights reserved.
 *
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LEDs definitions for nRF52832-MDK
#define LEDS_NUMBER    3

#define LED_START      22
#define LED_1          22
#define LED_2          23
#define LED_3          24
#define LED_STOP       24

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3

#define BUTTONS_NUMBER 1
#define BUTTON_START   18
#define BUTTON_1       18
#define BUTTON_STOP    18
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1}

#define BSP_BUTTON_0   BUTTON_1


#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  20
#define HWFC           false

#define SER_CON_RX_PIN              19    // UART RX pin number.
#define SER_CON_TX_PIN              20    // UART TX pin number.
#define SER_APP_RX_PIN              20    // UART RX pin number.
#define SER_APP_TX_PIN              19    // UART TX pin number.

#define ARDUINO_SDA_PIN             11
#define ARDUINO_SCL_PIN             12

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
