/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup BCSS
 * @brief Body Composition Service Server Application API
 * @{
 *
 * @file app_bcss.h
 *
 * @brief Header file - Body Composition Service Server Application Layer.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_BCSS_H_
#define APP_BCSS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_BCS_SERVER)
#include "bcs_common.h"
#include "attm.h"

/*
 * VARIABLES DECLARATION
 ****************************************************************************************
 */

/// APP BCSS callbacks
struct app_bcss_cb
{
    /// Service enabled message callback
    void (*on_bcss_enable_rsp)(uint8_t conidx, uint8_t status, uint16_t init_cfg_ind);

    /// Measurement send response message callback
    void (*on_bcss_meas_val_ind_rsp)(uint8_t conidx, uint8_t status);

    /// Client Characteristic Configuration Descriptor changed message callback
    void (*on_bcss_ind_cfg_ind)(uint8_t conidx, bool status);
};


/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Body Composition Service Application
 ****************************************************************************************
 */
void app_bcss_init(void);

/**
 ****************************************************************************************
 * @brief Add a Body Composition Service instance in the DB
 ****************************************************************************************
 */
void app_bcss_create_db(void);

/**
 ****************************************************************************************
 * @brief Enable Body Composition Service Server.
 * param[in] conidx     Connection handle
 ****************************************************************************************
 */
void app_bcss_enable(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Send Body Composition measurement
 * param[in] connhdl          Connection handle
 * param[in] scan_refresh_val Scan Refresh value
 ****************************************************************************************
 */
void app_bcss_send_measurement(uint8_t conidx, const bcs_meas_t *meas);

/**
 ****************************************************************************************
 * @brief Set initial Measurement notifications status restored when service is enabled.
 * param[in] enabled     Whether notifications are enabled or not
 ****************************************************************************************
 */
void app_bcss_set_initial_measurement_ind_cfg(bool enabled);

#endif // (BLE_BCS_SERVER)

#endif

///@}
///@}
///@}
