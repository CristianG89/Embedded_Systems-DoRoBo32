/**
 * DoroboLib32 FFT (DFFT)
 *
 * Simple implementation of a fast fourier transform (fft)
 * to detect 100 Hz and 125 Hz modulated signals.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#include <stm32f0xx_hal.h>
#include <stdlib.h>

#include <dorobo32.h>
#include <digital.h>

#include "fft.h"


// ***************************************************************************

extern TIM_HandleTypeDef htim6;

#define SAMPLES 40
static uint8_t ft_sample_index = 0;
static bool ft_samples_ready = false;
static enum DD_PINS_E sample_pin;

const static int8_t STEUERVEKTOR_RE8_[] =
{ 100, 71, 0, -71, -100, -71, 0, 71, 100, 71, 0, -71, -100, -71, 0, 71, 100, 71,
        0, -71, -100, -71, 0, 71, 100, 71, 0, -71, -100, -71, 0, 71, 100, 71, 0,
        -71, -100, -71, 0, 71 };
const static int8_t STEUERVEKTOR_IM8_[] =
{ 0, -71, -100, -71, 0, 71, 100, 71, 0, -71, -100, -71, 0, 71, 100, 71, 0, -71,
        -100, -71, 0, 71, 100, 71, 0, -71, -100, -71, 0, 71, 100, 71, 0, -71,
        -100, -71, 0, 71, 100, 71 };
const static int8_t STEUERVEKTOR_RE10_[] =
{ 100, 81, 31, -31, -81, -100, -81, -31, 31, 81, 100, 81, 31, -31, -81, -100,
        -81, -31, 31, 81, 100, 81, 31, -31, -81, -100, -81, -31, 31, 81, 100,
        81, 31, -31, -81, -100, -81, -31, 31, 81 };
const static int8_t STEUERVEKTOR_IM10_[] =
{ 0, -59, -95, -95, -59, 0, 59, 95, 95, 59, 0, -59, -95, -95, -59, 0, 59, 95,
        95, 59, 0, -59, -95, -95, -59, 0, 59, 95, 95, 59, 0, -59, -95, -95, -59,
        0, 59, 95, 95, 59 };

static uint8_t signal1_[SAMPLES];

static DFT_SamplingFinishedCallback_T sampling_finished = NULL;

// ***************************************************************************

void ft_init(void)
{
	// nothing to initialize
}


void ft_start_sampling(enum DD_PINS_E pin_no)
{
    sample_pin = pin_no;
    ft_samples_ready = false;
    HAL_TIM_Base_Start_IT(&htim6);
}


bool ft_is_sampling_finished(void)
{
    return ft_samples_ready;
}


uint16_t ft_get_transform(DFT_FREQ_T frequency)
{
    int16_t real = 0;
    int16_t imag = 0;
    const int8_t *steuervektor_re;
    const int8_t *steuervektor_im;

    // Steuervektor abhängig von der gewünschten Frequenzkomponente auswählen
    switch (frequency)
    {
    case DFT_FREQ100:
        steuervektor_re = STEUERVEKTOR_RE10_;
        steuervektor_im = STEUERVEKTOR_IM10_;
        break;
    case DFT_FREQ125:
        steuervektor_re = STEUERVEKTOR_RE8_;
        steuervektor_im = STEUERVEKTOR_IM8_;
        break;
    }

    // Fouriertransformierte für die gewünschte Frequenzkomponente bestimmen
    for (int i = 0; i < SAMPLES; i++)
    {
        if (signal1_[i])
        {
            real += steuervektor_re[i];
            imag += steuervektor_im[i];
        }
    }

    // TODO stimmt das?
    return abs(real) + abs(imag);
}

// ***************************************************************************

// TODO remove fft_simple and FreeRTOS dependency
//uint16_t fft_simple(enum DFFT_FFT_FREQ_E efft_freq, enum DD_PINS_E pin_no)
//{
//    fft_start_sampling(pin_no);
//    while (fft_samples_ready == 0)
//    {
//        taskYIELD();
//    }
//    return fft_get_transform(efft_freq);
//}

// ***************************************************************************

void ft_set_sampling_finished_handler(DFT_SamplingFinishedCallback_T sampling_finished_callback)
{
    sampling_finished = sampling_finished_callback;
}

// ***************************************************************************

void ft_get_samples()
{
    if (ft_sample_index == SAMPLES)
    {
        HAL_TIM_Base_Stop_IT(&htim6);
        ft_sample_index = 0;
        ft_samples_ready = true;
        if (sampling_finished != NULL)
            sampling_finished();
    }
    else
    {
        signal1_[ft_sample_index] = digital_get_pin(sample_pin);
        ft_sample_index++;
    }
}


/**
 * @brief This function handles TIM6 global and DAC channel underrun error interrupts.
 */
void TIM6_DAC_IRQHandler(void)
{
    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
            ft_get_samples();
        }
    }
}
