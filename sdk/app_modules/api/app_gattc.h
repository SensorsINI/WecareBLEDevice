/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup GATTC
 * @brief Generic Attribute Profile Client Application API
 * @{
 *
 * @file app_gattc.h
 *
 * @brief Generic Attribute Profile Service Client Application entry point.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 *
 ****************************************************************************************
 */

#ifndef APP_GATTC_H_
#define APP_GATTC_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_GATT_CLIENT)

#include "gatt_client.h"

/*
 * GLOBAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/// GATTC APP callbacks
struct app_gattc_cb
{
    /// Callback upon 'enable'
    void (*on_gattc_enable)(uint8_t conidx, uint8_t status, const struct gatt_client_content *gatt);

    /// Callback upon 'rd_cfg_ind_rsp'
    void (*on_gattc_rd_cfg_ind_rsp)(uint8_t conidx, uint8_t status, bool enabled);

    /// Callback upon 'wr_cfg_ind_rsp'
    void (*on_gattc_wr_cfg_ind_rsp)(uint8_t conidx, uint8_t status);

    /// Callback upon 'svc_changed_ind'
    void (*on_gattc_svc_changed_ind)(uint8_t conidx, struct prf_svc *val);
};

/**
 ****************************************************************************************
 * @brief Initialize GATT Service Application
 ****************************************************************************************
 */
void app_gattc_init(void);

/**
 ****************************************************************************************
 * @brief Add a GATT Service instance
 ****************************************************************************************
 */
void app_gattc_create_task(void);

/**
 ****************************************************************************************
 * @brief Enable GATT Service client
 * @param[in] conidx            The connection id number
 ****************************************************************************************
 */
void app_gattc_enable(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Write GATT service indication configuration
 * @param[in] conidx            The connection id number
 * @param[in] enable            Indication enable/disable
 ****************************************************************************************
 */
void app_gattc_write_ind_cfg(uint8_t conidx, bool enable);

/**
 ****************************************************************************************
 * @brief Read GATT service indicaion configuration value
 * @param[in] conidx            The connection id number
 ****************************************************************************************
 */
void app_gattc_read_ind_cfg(uint8_t conidx);

#endif // (BLE_GATT_CLIENT)

#endif // APP_GATTC_H_

///@}
///@}
///@}
