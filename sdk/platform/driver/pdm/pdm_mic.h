/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup PDM
 * @{
 * @addtogroup PDM_MIC PDM MIC
 * @brief PDM MIC audio driver API
 * @{
 *
 * @file pdm_mic.h
 *
 * @brief PDM MIC audio interface driver.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#if !defined (__DA14531__)

#ifndef PDM_MIC_H_
#define PDM_MIC_H_

#include "pdm.h"

/**
 \brief PDM interface sample rates. Possible values: PDM_8000, PDM_11025, PDM_16000, PDM_22050,
 PDM_32000, PDM_44100, PDM_48000, PDM_88200, PDM_96000, PDM_176400 or PDM_192000.
 */
typedef enum {
        PDM_8000 = 8000,
        PDM_11025 = 11025,
        PDM_16000 = 16000,
        PDM_22050 = 22050,
        PDM_32000 = 32000,
        PDM_44100 = 44100,
        PDM_48000 = 48000,
        PDM_88200 = 88200,
        PDM_96000 = 96000,
        PDM_176400 = 176400,
        PDM_192000 = 192000
} SRC_SampleRate_t;

/// Callback type to be associated with PDM mic.
typedef uint32_t * (*pdm_mic_data_available_cb)(uint16_t length);

/// PDM mic configuration struct
typedef struct pdm_mic_setup_s {
    /// Buffer to write data to
    uint32_t *buffer;

    /// Buffer length
    uint16_t buffer_length;

    /// Buffer mode
    bool buffer_circular;

    /// Interrupt threshold
    uint16_t int_thresold;

    /// PDM clock port-pin
    pdm_gpio_t clk_gpio;

    /// PDM data port-pin
    pdm_gpio_t data_gpio;

    /// Sampling rate
    SRC_SampleRate_t sampling_rate;

    /// PDM mic callback
    pdm_mic_data_available_cb   callback;
} pdm_mic_setup_t;

/**
 ****************************************************************************************
 * \brief       Start MIC recording
 *
 * \param [in]  config MIC configuration
 ****************************************************************************************
 */
void pdm_mic_start(pdm_mic_setup_t *config);

/**
 ****************************************************************************************
 * \brief       Stop MIC recording
 ****************************************************************************************
 */
void pdm_mic_stop(void);

#endif /* PDM_MIC_H_ */

#endif // DA14585, DA14586

///@}
///@}
///@}
