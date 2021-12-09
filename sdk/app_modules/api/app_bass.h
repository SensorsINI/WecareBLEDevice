/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup BASS
 * @brief Battery Service Server Application API
 * @{
 *
 * @file app_bass.h
 *
 * @brief Battery server application header file.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _APP_BASS_H_
#define _APP_BASS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_BATT_SERVER)
#include <stdint.h>
#include "gpio.h"

/*
 * EXTERNAL VARIABLES DECLARATION
 ****************************************************************************************
 */

/// Current battery level
extern uint8_t cur_batt_level;
/// Battery alert enabled
extern uint8_t batt_alert_en;
/// Battery LED state
extern uint8_t bat_led_state;

/*
 * VARIABLES DECLARATION
 ****************************************************************************************
 */

/// BASS APP callbacks
struct app_bass_cb
{
    /// Callback upon 'batt_level_upd_rsp'
    void (*on_batt_level_upd_rsp)(uint8_t status);

    /// Callback upon 'batt_level_ntf_cfg_ind'
    void (*on_batt_level_ntf_cfg_ind)(uint8_t conidx, uint8_t ntf_cfg);
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Battery Service Application.
 ****************************************************************************************
 */
void app_batt_init(void);

/**
 ****************************************************************************************
 * @brief Enable the battery service profile.
 * @param[in] conidx Connection index
 ****************************************************************************************
 */
 void app_bass_enable(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Configure the battery service profile.
 * @param[in] batt_lvl          Battery level
 * @param[in] lvl_alert_used    Battery level to generate alert
 * @param[in] led_port          Port of used LED
 * @param[in] led_pin           Pin of used LED
 ****************************************************************************************
 */
void app_batt_config(uint8_t batt_lvl, uint8_t lvl_alert_used, GPIO_PORT led_port, 
                     GPIO_PIN led_pin);

/**
 ****************************************************************************************
 * @brief Create the battery service profile.
 ****************************************************************************************
 */
void app_bass_create_db(void);

/**
 ****************************************************************************************
 * @brief Reads devices current battery level and sends battery level update request to
 *        Battery Service.
 ****************************************************************************************
 */
void app_batt_lvl(void);

/**
 ****************************************************************************************
 * @brief Sends battery level update request to Battery Service.
 * @param[in] batt_level          Battery level to set
 ****************************************************************************************
 */
void app_batt_set_level(uint8_t batt_level);

/**
 ****************************************************************************************
 * @brief Starts battery level polling.
 * @param[in] poll_timeout      Battery Polling frequency
 ****************************************************************************************
 */
void app_batt_poll_start(uint16_t poll_timeout);

/**
 ****************************************************************************************
 * @brief Stops battery level polling.
 ****************************************************************************************
 */
void app_batt_poll_stop(void);

/**
 ****************************************************************************************
 * @brief Starts battery alert. Battery Low.
 ****************************************************************************************
*/
void app_batt_alert_start(void);

/**
 ****************************************************************************************
 * @brief Stops battery alert. Battery Low.
 ****************************************************************************************
 */
void app_batt_alert_stop(void);

/**
 ****************************************************************************************
 * @brief Reinit battery alert LED port. Must be called in periph_init().
 ****************************************************************************************
 */
void app_batt_port_reinit(void);

#endif // (BLE_BATT_SERVER)

#endif // _APP_BASS_H_

///@}
///@}
///@}
