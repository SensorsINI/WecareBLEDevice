/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup WSSS
 * @brief Weight Scale Service Server Application API
 * @{
 *
 * @file app_wsss.h
 *
 * @brief Weight Scale Profile Server Application header.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _APP_WSSS_H_
#define _APP_WSSS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_WSS_SERVER)
#include "wss_common.h"

/*
 * VARIABLES DECLARATION
 ****************************************************************************************
 */

/// WSSS APP callbacks
struct app_wsss_cb
{
    /// Callback upon 'wsss_bcs_reference_req'
    const struct att_incl_desc *(*on_wsss_bcs_reference_req)(void);

    /// Callback upon 'wsss_ind_cfg_ind'
    void (*on_wsss_ind_cfg_ind)(uint8_t conidx, bool ind_en);

    /// Callback upon 'wsss_send_measurement_cfm'
    void (*on_wsss_send_measurement_cfm)(uint8_t conidx, uint8_t status);
};


/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Weight Scale Service Application
 ****************************************************************************************
 */
void app_wsss_init(void);

/**
 ****************************************************************************************
 * @brief Add a Weight Scale Service instance in the DB
 ****************************************************************************************
 */
void app_wsss_create_db(void);

/**
 ****************************************************************************************
 * @brief Enable Weight Scale Profile Server.
 * param[in] conidx     Connection index
 ****************************************************************************************
 */
void app_wsss_enable(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Send Weight Scale measurement
 * param[in] conidx          Connection index
 * param[in] meas            Weight Scale measurement
 ****************************************************************************************
 */
void app_wsss_send_measurement(uint8_t conidx, const struct wss_wt_meas *meas);

/**
 ****************************************************************************************
 * @brief Set initial Measurement notifications status restored when service is enabled.
 * param[in] enabled     Whether notifications are enabled or not
 ****************************************************************************************
 */
void app_wsss_set_initial_measurement_ind_cfg(bool enabled);

#endif // (BLE_WSS_SERVER)

#endif // _APP_WSSS_H_

///@}
///@}
///@}
