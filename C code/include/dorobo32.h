/**
 * @file
 *
 * DoroboLib32 central header
 *
 * Initialization of the board and the most common modules
 * and control of the on-board LEDs.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#ifndef DOROBO32_INC_DOROBO32_H_
#define DOROBO32_INC_DOROBO32_H_

#include <stdint.h>

#include "digital.h"


#define DD_LED_ON DD_LEVEL_LOW
#define DD_LED_OFF DD_LEVEL_HIGH


#ifdef __cplusplus
 extern "C" {
#endif


/**
 * @brief   Board initialization and module initialization for digital i/o and uart.
 */
void dorobo_init(void);


/**
 * @brief Delay for a given duration by polling the SysTick timer.
 *
 * @note Reminder: When using an RTOS, prefer the functions provided by
 * the operating system.
 *
 * @param delay_ms Delay in ms
 */
void delay(uint32_t delay_ms);


/**
 * @brief   Set the green on-board led on or off.
 *
 * @param   level Level as defined in DD_PINLEVEL_E
 */
void led_green(enum DD_PINLEVEL_E level);

/**
 * @brief   Toggle the green on-board led.
 */
void led_green_toggle(void);

/**
 * @brief   Set the red on-board led on or off.
 *
 * @param   level Level as defined in DD_PINLEVEL_E
 */
void led_red(enum DD_PINLEVEL_E level);

/**
 * @brief   Toggle the red on-board led.
 */
void led_red_toggle(void);


#ifdef __cplusplus
}
#endif

#endif /* DOROBO32_INC_DOROBO32_H_ */
