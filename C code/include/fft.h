/**
 * @file
 *
 * DoroboLib32 Fourier Transform (DFT)
 *
 * Simple implementation of a fourier transform (ft)
 * to detect 100 Hz and 125 Hz modulated signals.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#ifndef DOROBO32_INC_FFT_H_
#define DOROBO32_INC_FFT_H_

#include <stdint.h>
#include <stdbool.h>

#include <digital.h>


#ifdef __cplusplus
 extern "C" {
#endif


/**
 * The signals that can be detected by the function fft_simple()
 */
enum DFT_FREQ_E
{
    DFT_FREQ100,    ///< 100Hz
    DFT_FREQ125     ///< 125Hz
};
typedef enum DFT_FREQ_E DFT_FREQ_T;


/**
 * Callback function for an optional "sampling finished" notification.
 *
 * @see ft_set_sampling_finished_handler
 */
typedef void (*DFT_SamplingFinishedCallback_T)(void);


/**
 * @brief   FFT initialization.
 *
 * Call this routine once before using the FFT module.
 */
void ft_init(void);


/**
 * Start sampling digital data.
 *
 * Call this routine to start sampling data digital data from a GPIO pin,
 * which could e.g. be connected to a IR receiver. This routine starts
 * sampling the pin in a periodic interrupt routine in the background.
 * The periodic interrupts stop after taking the necessary number of samples.
 * Call @see ft_is_sampling_finished to check whether the required number of
 * samples has been taken.
 *
 * @param pin_no  GPIO input pin the IR sensor is connected to.
 */
void ft_start_sampling(enum DD_PINS_E pin_no);


/**
 * Set a handler function called after finishing sampling.
 *
 * The function pointer given as an argument is called by the library when
 * the required number of samples has been taken, i.e. a few 10 ms after
 * calling ft_start_sampling. It is called directly from an interrupt context,
 * thus all the respective constraints apply!
 *
 * @param DFT_SamplingFinishedCallback_T Function pointer to the callback
 * routine; NULL for deactivating the callback.
 */
void ft_set_sampling_finished_handler(DFT_SamplingFinishedCallback_T sampling_finished_callback);


/**
 * Check whether the number of samples required for the FFT has been taken.
 *
 * After calling @see fft_start_sampling, periodic sampling of a GPIO input
 * pin starts using interrupts in the background. Use this routine to check
 * whether sampling has finished (i.e. the required number of samples has
 * been captured).
 *
 * @return True if sampling has finished, false otherwise.
 */
bool ft_is_sampling_finished(void);


/**
 * Return the desired frequency component of the input signal previously sampled.
 *
 * After finishing sampling an input signal (@see ft_is_sampling_finished),
 * this routine determines the desired frequency component using a fourier
 * transform.
 *
 * @param efft_freq  Frequency component of the input signal to determine.
 * @return The desired frequency component of the input signal.
 */
uint16_t ft_get_transform(DFT_FREQ_T efft_freq);


/**
 * @brief   Start sampling from digital *pin_no* and perform fft over the samples.
 *
 * @deprecated
 * @note This function is deprecated and will be removed from the API - do not use it!
 *
 * @param   fft_freq
 *          The frequency that needs to be detected
 *
 * @param   pin_no
 *          Digital pin that is used for sampling
 *
 * @return  The compliance of the measured signal with the signal that is to be
 * detected. A value above 1,200 indicates compliance.
 */
//uint16_t fft_simple(enum DFFT_FFT_FREQ_E fft_freq, enum DD_PINS_E pin_no);


#ifdef __cplusplus
}
#endif

#endif /* DOROBO32_INC_FFT_H_ */
