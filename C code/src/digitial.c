/**
 * DoroboLib32 Digital IO
 *
 * Funktionen für die Manipulation der digitalen IO Pins.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#include "stm32f0xx_hal.h"
#include "mxconstants.h"

#include "digital.h"


typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
} pin_t;


static void select_pin(enum DD_PINS_E pin_no, pin_t* currentPin);


void digital_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = DIP1_Pin;
    HAL_GPIO_Init(DIP1_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DIP2_Pin;
    HAL_GPIO_Init(DIP2_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DIP3_Pin;
    HAL_GPIO_Init(DIP3_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DIP4_Pin;
    HAL_GPIO_Init(DIP4_GPIO_Port, &GPIO_InitStruct);
}

void digital_configure_pin(enum DD_PINS_E pin_no, enum DD_PINCONFIG_E direction)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    pin_t pin;
    select_pin(pin_no, &pin);
    GPIO_InitStruct.Pin = pin.pin;
    switch(direction)
    {
        case DD_CFG_OUTPUT:
        {
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            HAL_GPIO_Init(pin.port, &GPIO_InitStruct);
            break;
        }
        case DD_CFG_INPUT_PULLUP:
        {
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            HAL_GPIO_Init(pin.port, &GPIO_InitStruct);
            break;
        }
        case DD_CFG_INPUT_PULLDOWN:
        {
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            HAL_GPIO_Init(pin.port, &GPIO_InitStruct);
            break;
        }
        case DD_CFG_INPUT_NOPULL:
        {
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(pin.port, &GPIO_InitStruct);
            break;
        }
    }
}

void digital_set_pin(enum DD_PINS_E pin_no, enum DD_PINLEVEL_E level)
{
    pin_t selectedPin;
    select_pin(pin_no, &selectedPin);
    HAL_GPIO_WritePin(selectedPin.port, selectedPin.pin, level);
}

enum DD_PINLEVEL_E digital_get_pin(enum DD_PINS_E pin_no)
{
    pin_t selectedPin;
    select_pin(pin_no, &selectedPin);
    return HAL_GPIO_ReadPin(selectedPin.port, selectedPin.pin);
}

DD_DIP_STATE_T digital_get_dip(DD_DIPS_T dip_no)
{
    enum DD_PINLEVEL_E level;
    switch (dip_no)
    {
    case DD_DIP1:
    {
        level = HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin);
        break;
    }
    case DD_DIP2:
    {
        level = HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin);
        break;
    }
    case DD_DIP3:
    {
        level = HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin);
        break;
    }
    case DD_DIP4:
    {
        level = HAL_GPIO_ReadPin(DIP4_GPIO_Port, DIP4_Pin);
        break;
    }
    }
    return ( level == DD_LEVEL_HIGH ? DD_DIP_OFF : DD_DIP_ON );
}


static void select_pin(enum DD_PINS_E pin_no, pin_t* currentPin)
{
    switch (pin_no)
    {
    case DD_PIN_PD14:
    {
        currentPin->port = DIGITAL0_GPIO_Port;
        currentPin->pin = DIGITAL0_Pin;
        break;
    }
    case DD_PIN_PD15:
    {
        currentPin->port = DIGITAL1_GPIO_Port;
        currentPin->pin = DIGITAL1_Pin;
        break;
    }
    case DD_PIN_PC8:
    {
        currentPin->port = DIGITAL2_GPIO_Port;
        currentPin->pin = DIGITAL2_Pin;
        break;
    }
    case DD_PIN_PC9:
    {
        currentPin->port = DIGITAL3_GPIO_Port;
        currentPin->pin = DIGITAL3_Pin;
        break;
    }
    case DD_PIN_PA8:
    {
        currentPin->port = DIGITAL4_GPIO_Port;
        currentPin->pin = DIGITAL4_Pin;
        break;
    }
    case DD_PIN_PC13:
    {
        currentPin->port = DIGITAL5_GPIO_Port;
        currentPin->pin = DIGITAL5_Pin;
        break;
    }
    case DD_PIN_PB11:
    {
        currentPin->port = SERVO0_GPIO_Port;
        currentPin->pin = SERVO0_Pin;
        break;
    }
    case DD_PIN_PB10:
    {
        currentPin->port = SERVO1_GPIO_Port;
        currentPin->pin = SERVO1_Pin;
        break;
    }
    case DD_PIN_PE14:
    {
        currentPin->port = SERVO2_GPIO_Port;
        currentPin->pin = SERVO2_Pin;
        break;
    }
    case DD_PIN_PE11:
    {
        currentPin->port = SERVO3_GPIO_Port;
        currentPin->pin = SERVO3_Pin;
        break;
    }
    case DD_PIN_PE9:
    {
        currentPin->port = SERVO4_GPIO_Port;
        currentPin->pin = SERVO4_Pin;
        break;
    }
    case DD_PIN_PB1:
    {
        currentPin->port = SERVO5_GPIO_Port;
        currentPin->pin = SERVO5_Pin;
        break;
    }
    case DD_PIN_PF10:
    {
        currentPin->port = PWM5_GPIO_Port;
        currentPin->pin = PWM5_Pin;
        break;
    }
    case DD_PIN_PF9:
	{
		currentPin->port = PWM4_GPIO_Port;
		currentPin->pin = PWM4_Pin;
		break;
	}
    case DD_PIN_PE1:
    {
    	currentPin->port = PWM3_GPIO_Port;
    	currentPin->pin = PWM3_Pin;
    	break;
    }
    case DD_PIN_PE0:
    {
    	currentPin->port = PWM2_GPIO_Port;
    	currentPin->pin = PWM2_Pin;
    	break;
    }
    case DD_PIN_PB3:
    {
    	currentPin->port = PWM1_GPIO_Port;
    	currentPin->pin = PWM1_Pin;
    	break;
    }
    case DD_PIN_PA15:
    {
    	currentPin->port = PWM0_GPIO_Port;
    	currentPin->pin = PWM0_Pin;
    	break;
    }
    default:
    {
        currentPin->port = LED_GREEN_GPIO_Port;
        currentPin->pin = LED_GREEN_Pin;
        break;
    }
        ;
    }
}
