/**
 * @file
 * DoroboLib32 ADC (DA)
 *
 * Functions for reading analog voltages using the analog digital converter.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#ifndef DOROBO32_INC_ADC_H_
#define DOROBO32_INC_ADC_H_

#include <stdint.h>

/**
 * Dorobo32 analog input channels
 */
enum DA_ADC_CHANNEL_E
{
    DA_ADC_CHANNEL0,  ///< analog channel 0
    DA_ADC_CHANNEL1,  ///< analog channel 1
    DA_ADC_CHANNEL2,  ///< analog channel 2
    DA_ADC_CHANNEL3,  ///< analog channel 3
    DA_ADC_CHANNEL4,  ///< analog channel 4
    DA_ADC_CHANNEL5,  ///< analog channel 5
    DA_ADC_CHANNEL6,  ///< analog channel 6
    DA_ADC_CHANNEL7,  ///< analog channel 7
    DA_ADC_CHANNEL8,  ///< analog channel 8
    DA_ADC_CHANNEL9   ///< analog channel 9
};

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief   Adc initialization.
 */
void adc_init(void);

/**
 * @brief   Get converted analog value from analog pin *adc_channel_no*.
 *
 * @param   adc_channel_no  The adc channel to be read.
 *
 * @return  Converted analog 12 bit value.
 */
uint32_t adc_get_value(enum DA_ADC_CHANNEL_E adc_channel_no);

#ifdef __cplusplus
}
#endif

#endif /* DOROBO32_INC_ADC_H_ */
