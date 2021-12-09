/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup CTSC
 * @brief Current Time Service Client Application API
 * @{
 *
 * @file app_ctsc.h
 *
 * @brief Current Time Service Application entry point.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 *
 ****************************************************************************************
 */

#ifndef _APP_CTSC_H_
#define _APP_CTSC_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_CTS_CLIENT)

#include "cts_common.h"
#include "ctsc.h"

/*
 * GLOBAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/// CTSC APP callbacks
struct app_ctsc_cb
{
    /// Callback upon 'connect'
    void (*on_connect)(uint8_t conidx, const struct ctsc_cts_content *cts);

    /// Callback upon 'ct_read'
    void (*on_ct_read)(uint8_t conidx, uint8_t status, const struct cts_curr_time *ct, bool notification);

    /// Callback upon 'rti_read'
    void (*on_rti_read)(uint8_t conidx, uint8_t status, const struct cts_ref_time_info *ct);

    /// Callback upon 'lti_read'
    void (*on_lti_read)(uint8_t conidx, uint8_t status, const struct cts_loc_time_info *lt);

    /// Callback upon 'desc_read'
    void (*on_desc_read)(uint8_t conidx, uint8_t status, uint16_t value);

    /// Callback upon 'ct_write'
    void (*on_ct_write)(uint8_t conidx, uint8_t status);

    /// Callback upon 'lti_write'
    void (*on_lti_write)(uint8_t conidx, uint8_t status);

    /// Callback upon 'desc_write'
    void (*on_desc_write)(uint8_t conidx, uint8_t status);
};

/**
 ****************************************************************************************
 * @brief Initialize Current Time Service Application
 ****************************************************************************************
 */
void app_ctsc_init(void);

/**
 ****************************************************************************************
 * @brief Add a Current Time Service instance in the DB
 ****************************************************************************************
 */
void app_ctsc_create_task(void);

/**
 ****************************************************************************************
 * @brief Enable Current Time Service client profile.
 * @param[in] conidx            The connection id number
 ****************************************************************************************
 */
void app_ctsc_enable(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Read current time from peer
 ****************************************************************************************
 */
void app_ctsc_read_ct(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Read Local Time information from peer
 ****************************************************************************************
 */
void app_ctsc_read_lti(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Read Reference Time information from peer
 ****************************************************************************************
 */
void app_ctsc_read_rti(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Read Current Time conjuration descriptor from peer
 ****************************************************************************************
 */
void app_ctsc_read_desc(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Write current time to peer
 ****************************************************************************************
 */
void app_ctsc_write_ct(uint8_t conidx, const struct cts_curr_time *ct);

/**
 ****************************************************************************************
 * @brief Write local time information to peer
 ****************************************************************************************
 */
void app_ctsc_write_lti(uint8_t conidx, const struct cts_loc_time_info *lti);

/**
 ****************************************************************************************
 * @brief Write Current Time configuration descriptor to peer
 ****************************************************************************************
 */
void app_ctsc_write_desc(uint8_t conidx, uint16_t value);

#endif // (BLE_CTS_CLIENT)

#endif // _APP_CTSC_H_

///@}
///@}
///@}
