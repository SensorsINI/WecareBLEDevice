/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup UDSS
 * @brief User Data Service Server Application API
 * @{
 *
 * @file app_udss.h
 *
 * @brief User Data Service Server Application header file
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_UDSS_H_
#define APP_UDSS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "user_profiles_config.h"

#if BLE_UDS_SERVER

#include "udss.h"
#include "uds_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Supported User Data Characteristics Defaults
#ifndef APP_UDS_ENABLED_UDS_CHARS
#define APP_UDS_ENABLED_UDS_CHARS (UDS_CHAR_FLAG_FIRST_NAME | UDS_CHAR_FLAG_LAST_NAME | UDS_CHAR_FLAG_GENDER)
#endif

/// If Host is able to update characteristic values - enables DB Change Increment CCC descriptor
#ifndef APP_UDS_LOCAL_CHAR_UPDATE_ENABLED
#define APP_UDS_LOCAL_CHAR_UPDATE_ENABLED (false)
#else
#undef APP_UDS_LOCAL_CHAR_UPDATE_ENABLED
#define APP_UDS_LOCAL_CHAR_UPDATE_ENABLED (true)
#endif

/*
 * VARIABLES DECLARATION
 ****************************************************************************************
 */

/// UDSS APP callbacks
struct app_udss_cb
{
    /// Callback upon 'cntl_point_write_ind'
    void (*on_udss_cntl_point_write_ind)(uint8_t conidx, const struct uds_ucp_req *req);

    /// Callback upon 'char_val_req'
    void (*on_udss_char_val_req)(uint8_t conidx, uint8_t char_code);

    /// Callback upon 'char_val_ind'
    void (*on_udss_char_val_ind)(uint8_t conidx, uint8_t char_code, uint8_t val_len, const uint8_t *val);
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes UDSS Apllication.
 ****************************************************************************************
 */
void app_udss_init(void);

/**
 ****************************************************************************************
 * @brief Creates UDSS service database.
 ****************************************************************************************
 */
void app_udss_create_db(void);

/**
 ****************************************************************************************
 * @brief Sends control point operation status to User Data Service Task.
 * @param[in] conidx        Connection index
 * @param[in] rsp           Control Point operation response
 ****************************************************************************************
 */
void app_udss_cntl_point_operation_cfm(uint8_t conidx, const struct uds_ucp_rsp *rsp);

/**
 ****************************************************************************************
 * @brief Sends characteristic value read response to User Data Service Task.
 * @param[in] conidx        Connection index
 * @param[in] char_code     Characteristic code
 * @param[in] length        Value length
 * @param[in] value         Value pointer
 ****************************************************************************************
 */
void app_udss_char_val_rsp(uint8_t conidx, uint8_t char_code, uint8_t length,
                           const uint8_t *value);

/**
 ****************************************************************************************
 * @brief Sends set characteristic value confirmation to User Data Service Task.
 * @param[in] conidx        Connection index
 * @param[in] char_code     Characteristic code
 * @param[in] status        Status code
 ****************************************************************************************
 */
void app_udss_set_char_val_cfm(uint8_t conidx, uint8_t char_code, uint8_t status);

/**
 ****************************************************************************************
 * @brief Notify Service Task that User Data Service Database has been updated.
 * @param[in] conidx                Connection index
 * @param[in] db_change_incr_val    Database change counter
 ****************************************************************************************
 */
void app_udss_db_updated_notify(uint8_t conidx, uint32_t db_change_incr_val);

#endif // BLE_UDS_SERVER

#endif // APP_UDSS_H_

///@}
///@}
///@}
