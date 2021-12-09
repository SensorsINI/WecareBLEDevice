/**
 ****************************************************************************************
 *
 * @file wsss_task.h
 *
 * @brief Header file - Weight Scale Service Server Task.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */


#ifndef WSSS_TASK_H_
#define WSSS_TASK_H_

/**
 ************************************************************************************
 * @addtogroup WSSSTASK Task
 * @ingroup WSSS
 * @brief Weight Scale Service Server Task
 * @{
 ************************************************************************************
*/

#if (BLE_WSS_SERVER)
/*
* INCLUDE FILES
****************************************************************************************
*/

#include "wss_common.h"

/*
 * DEFINES
 ********************************************************************************
 */

/// Possible states of the WSSS task
enum
{
    ///Idle state
    WSSS_IDLE,
    ///Connected state
    WSSS_BUSY,

    ///Number of defined states.
    WSSS_STATE_MAX
};

/// Messages for Weight Scale Service Server
enum
{
    ///Start the Weight Scale Service Task - at connection
    WSSS_ENABLE_REQ = KE_FIRST_MSG(TASK_ID_WSSS),

    /// Enable confirmation
    WSSS_ENABLE_CFM,
    ///Set the value of "Measurement" attribute
    WSSS_MEAS_SEND_REQ,

    ///Send weight measurement value confirmation to APP if correctly sent.
    WSSS_MEAS_SEND_CFM,

    ///Inform APP of new configuration value
    WSSS_CFG_INDNTF_IND,
};

/// Weight Measurement User id field
enum
{
    /// Unknown user
    WSSS_MEAS_UNKNOWN_USER = 0xFF,
};

/// Weight Feature characteristic bit values
enum
{
    /// Time Stamp
    WSSS_FEAT_TIME_STAMP_SUPPORTED  = 0x0001,

    /// Multiple Users
    WSSS_FEAT_MULTI_USER_SUPPORTED  = 0x0002,

    /// BMI supported
    WSSS_FEAT_BMI_SUPPORTED         = 0x0004,

    /// Weight Display Resolution
    /// Display resolution of 0.5kg   or 1lb
    WSSS_FEAT_WT_DISPLAY_500G_ACC   = 0x0008,
    /// Display resolution of 0.2kg   or 0.5lb
    WSSS_FEAT_WT_DISPLAY_200G_ACC   = 0x0010,
    /// Display resolution of 0.1kg   or 0.2lb
    WSSS_FEAT_WT_DISPLAY_100G_ACC   = 0x0018,
    /// Display resolution of 0.05kg  or 0.1lb
    WSSS_FEAT_WT_DISPLAY_50G_ACC    = 0x0020,
    /// Display resolution of 0.02kg  or 0.05lb
    WSSS_FEAT_WT_DISPLAY_20G_ACC    = 0x0028,
    /// Display resolution of 0.01kg  or 0.02lb
    WSSS_FEAT_WT_DISPLAY_10G_ACC    = 0x0030,
    /// Display resolution of 0.005kg or 0.01lb
    WSSS_FEAT_WT_DISPLAY_5G_ACC     = 0x0038,

    /// Height Display Resolution
    /// Display resolution of 0.01m  or 1in
    WSSS_FEAT_HT_DISPLAY_10MM_ACC   = 0x0080,
    /// Display resolution of 0.005m or 0.5in
    WSSS_FEAT_HT_DISPLAY_5MM_ACC    = 0x0100,
    /// Display resolution of 0.001m or 0.1in
    WSSS_FEAT_HT_DISPLAY_1MM_ACC    = 0x0180,
};


/*
 * API MESSAGES STRUCTURES
 ********************************************************************************
 */

struct wsss_db_cfg
{
    /// Weight Scale features:
    /// bit [0]: Time Stamp, bit[1]: Multiple Users Stamp, bit[2]: BMI,
    /// bit [3-6]: Weight display resolution,
    /// bit [7-9]: Height display resolution,
    /// bit [10-31]: RFU
    uint32_t ws_feature;
    /// Indicates if BCS should be included
    bool include_bcs_instance;
    /// BCS reference for the inclusion
    struct att_incl_desc bcs_ref;
};

/// Parameters of the @ref WSSS_ENABLE_REQ message
struct wsss_enable_req
{
    /// Weight Scale indication configuration
    uint16_t wt_meas_ind_en;
};

/// Parameters of the @ref WSSS_ENABLE_CFM message
struct wsss_enable_cfm
{
    /// Status of enable request
    uint8_t status;
};

/// Parameters of the @ref WSSS_MEAS_VAL_SET_REQ message - shall be dynamically allocated
struct wsss_meas_send_req
{
    /// Weight measurement
    struct wss_wt_meas meas_val;
};

///Parameters of the @ref WSSS_MEAS_SEND_CFM message
struct wsss_meas_send_cfm
{
    ///Status
    uint8_t status;
};

///Parameters of the @ref WSSS_CFG_INDNTF_IND message
struct wsss_cfg_indntf_ind
{
    ///Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler wsss_default_handler;
#endif //BLE_WSSS_SERVER
/// @} WSSSTASK
#endif // WSSS_TASK_H_
