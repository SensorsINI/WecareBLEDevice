/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup CTSS
 * @brief Current Time Service Server Application API
 * @{
 *
 * @file app_ctss.h
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

#ifndef _APP_CTSS_H_
#define _APP_CTSS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_CTS_SERVER)

#include "cts_common.h"

/*
 * GLOBAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/// CTSS APP callbacks
struct app_ctss_cb
{
    /// Callback upon 'cur_time_read_req'
    void (*on_cur_time_read_req)(struct cts_curr_time *ct);

    /// Callback upon 'cur_time_write_req'
    uint8_t (*on_cur_time_write_req)(const struct cts_curr_time *ct);

    /// Callback upon 'cur_time_notified'
    void (*on_cur_time_notified)(uint8_t status);

    /// Callback upon 'loc_time_info_write_req'
    void (*on_loc_time_info_write_req)(const struct cts_loc_time_info *lt);

    /// Callback upon 'ref_time_info_read_req'
    void (*on_ref_time_info_read_req)(struct cts_ref_time_info *rt);
};

/**
 ****************************************************************************************
 * @brief Initialize Current Time Service Application
 ****************************************************************************************
 */
void app_ctss_init(void);

/**
 ****************************************************************************************
 * @brief Add a Current Time Service instance in the DB
 ****************************************************************************************
 */
void app_ctss_create_db(void);

/**
 ****************************************************************************************
 * @brief Notify Current Time Service clients
 ****************************************************************************************
 */
void app_ctss_notify_current_time(const struct cts_curr_time *curr_time);

#endif // (BLE_CTS_SERVER)

#endif // _APP_CTSS_H_

///@}
///@}
///@}
