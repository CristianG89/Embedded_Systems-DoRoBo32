/**
 * DoroboLib32 ADC (DA)
 *
 * Functions for reading analog voltages using the analog digital converter.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#include "stm32f0xx_hal.h"

#include "adc.h"

extern ADC_HandleTypeDef hadc;
ADC_ChannelConfTypeDef sConfig;


static uint32_t select_adc_channel(enum DA_ADC_CHANNEL_E adc_channels)
{
    switch (adc_channels)
    {
    case DA_ADC_CHANNEL0:
        return ADC_CHANNEL_0;
        break;
    case DA_ADC_CHANNEL1:
        return ADC_CHANNEL_1;
        break;
    case DA_ADC_CHANNEL2:
        return ADC_CHANNEL_2;
        break;
    case DA_ADC_CHANNEL3:
        return ADC_CHANNEL_3;
        break;
    case DA_ADC_CHANNEL4:
        return ADC_CHANNEL_4;
        break;
    case DA_ADC_CHANNEL5:
        return ADC_CHANNEL_5;
        break;
    case DA_ADC_CHANNEL6:
        return ADC_CHANNEL_6;
        break;
    case DA_ADC_CHANNEL7:
        return ADC_CHANNEL_7;
        break;
    case DA_ADC_CHANNEL8:
        return ADC_CHANNEL_14;
        break;
    case DA_ADC_CHANNEL9:
        return ADC_CHANNEL_15;
        break;
    default:
        return ADC_CHANNEL_0;
    }
}

static void activate_channel(uint32_t adc_channel)
{
    sConfig.Channel = adc_channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

static void deactivate_channel(uint32_t adc_channel)
{
    sConfig.Channel = adc_channel;
    sConfig.Rank = ADC_RANK_NONE;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

// **************************************************************************
// Public interface
// **************************************************************************

void adc_init()
{
    // no initialization
}


uint32_t adc_get_value(enum DA_ADC_CHANNEL_E adc_channel_no)
{
    uint32_t adc_channel = select_adc_channel(adc_channel_no);
    uint32_t adc_value = 0;
    uint32_t timeout = 10;
    activate_channel(adc_channel);
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, timeout);
    adc_value = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);
    deactivate_channel(adc_channel);
    return adc_value;
}


