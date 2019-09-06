/**
 * @file
 * DoroboLib32 Digital IO (DD)
 *
 * Functions to read and set digitial IO pins.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#ifndef DOROBO32_INC_DIGITAL_H_
#define DOROBO32_INC_DIGITAL_H_

#include <stdint.h>

/**
 * Dorobo32 pins available for digital i/o
 */
enum DD_PINS_E
{
    DD_PIN_PD14,  ///< digital i/o pin PD14
    DD_PIN_PD15,  ///< digital i/o pin PD15
    DD_PIN_PC8,   ///< digital i/o pin PC8
    DD_PIN_PC9,   ///< digital i/o pin PC9
    DD_PIN_PA8,   ///< digital i/o pin PA8
    DD_PIN_PC13,  ///< digital i/o pin PC13

    DD_PIN_PB11,  ///< digital i/o pin PB11
    DD_PIN_PB10,  ///< digital i/o pin PB10
    DD_PIN_PE14,  ///< digital i/o pin PE14
    DD_PIN_PE11,  ///< digital i/o pin PE11
    DD_PIN_PE9,   ///< digital i/o pin PE9
    DD_PIN_PB1,   ///< digital i/o pin PB1

	DD_PIN_PF10, ///< digital i/o pin PF10
	DD_PIN_PF9,  ///< digital i/o pin PF9
	DD_PIN_PE1,  ///< digital i/o pin PE1
	DD_PIN_PE0,  ///< digital i/o pin PE0
	DD_PIN_PB3,  ///< digital i/o pin PB3
	DD_PIN_PA15, ///< digital i/o pin PA15

	DD_PIN_NOPIN ///< no digital i/o pin, can be used as a marker
};
typedef enum DD_PINS_E DD_PINS_T; ///< @see DD_PINS_E

/**
 * Dorobo32 DIP switches
 */
enum DD_DIPS_E
{
    DD_DIP1,    ///< dip switch 1
    DD_DIP2,    ///< dip switch 2
    DD_DIP3,    ///< dip switch 3
    DD_DIP4     ///< dip switch 4
};
typedef enum DD_DIPS_E DD_DIPS_T; ///< @see DD_DIPS_E

enum DD_DIP_STATE_E
{
    DD_DIP_OFF, ///< dip switch in off position
    DD_DIP_ON   ///< dip switch in on position
};
typedef enum DD_DIP_STATE_E DD_DIP_STATE_T; ///< @see DD_DIP_STATE_E

/**
 * Pin signal levels
 */
enum DD_PINLEVEL_E
{
    DD_LEVEL_LOW,     ///< digital low
    DD_LEVEL_HIGH     ///< digital high
};
typedef enum DD_PINLEVEL_E DD_PINLEVEL_T; ///< @see DD_DIP_STATE_E

/**
 * Pin configuration
 */
enum DD_PINCONFIG_E
{
    DD_CFG_OUTPUT,         ///< pin configured as output
    DD_CFG_INPUT_PULLUP,   ///< pin configured as input with internal pullup enabled
    DD_CFG_INPUT_PULLDOWN, ///< pin configured as input with internal pulldown enabled
    DD_CFG_INPUT_NOPULL    ///< pin configured as input without internal pullup or pulldown resistor
};

#ifdef __cplusplus
 extern "C" {
#endif

 /**
 * @brief   Initialize the Dorobo32 Digital I/O (DD) module.
 */
void digital_init(void);

/**
 * @brief   Configure digital pin direction and pullup/pulldown resistors.
 *
 * @param   pin_no
 *          Pin to configure
 *
 * @param   direction
 *          Predefined configuration as defined in DD_PINCONFIG_E
 */
void digital_configure_pin(enum DD_PINS_E pin_no, enum DD_PINCONFIG_E direction);

/**
 * @brief   Set pin level of pin *pin_no*.
 *
 * @param   pin_no
 *          Digital pin that is to be set
 *
 * @param   level
 *          The desired pin level. Values can be DD_LEVEL_LOW or DD_LEVEL_HIGH
 */
void digital_set_pin(enum DD_PINS_E pin_no, enum DD_PINLEVEL_E level);

/**
 * @brief   Get signal level for a pin.
 *
 * @param   pin_no
 *          Pin to read from
 *
 * @return  Pin signal level as defined in DD_PINLEVEL_E
 */
enum DD_PINLEVEL_E digital_get_pin(enum DD_PINS_E pin_no);

/**
 * @brief   Get position of a dip switch.
 *
 * @param   dip_no
 *          Dip to read from
 *
 * @return  On/off state of the corresponding DIP switch
 */
DD_DIP_STATE_T digital_get_dip(DD_DIPS_T dip_no);

#ifdef __cplusplus
}
#endif

#endif /* DOROBO32_INC_DIGITAL_H_ */
