/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Easy_Whitelist Easy Whitelist
 * @brief Whitelist-related  functions API
 * @{
 *
 * @file app_easy_whitelist.h
 *
 * @brief Whitelist management API header file.
 *
 * Copyright (C) 2019-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _APP_EASY_WHITELIST_H_
#define _APP_EASY_WHITELIST_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdbool.h>
#include "gapm_task.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Whitelist operation type
typedef enum
{
    /// Get White List Size.
    APP_WLIST_OP_GET_SIZE           = GAPM_GET_WLIST_SIZE,

    /// Add devices in white list.
    APP_WLIST_OP_ADD_DEV            = GAPM_ADD_DEV_IN_WLIST,

    /// Remove devices form white list.
    APP_WLIST_OP_RMV_DEV            = GAPM_RMV_DEV_FRM_WLIST,

    /// Clear all devices from white list.
    APP_WLIST_OP_CLEAR              = GAPM_CLEAR_WLIST,
} app_wlist_op_t;

/// Whitelist address type
typedef enum
{
    /// Public address
    APP_WLIST_ADDR_PUBLIC            = ADDR_PUBLIC,

    /// Random address
    APP_WLIST_ADDR_RANDOM            = ADDR_RAND,

} app_wlist_addr_t;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Function for whitelist management.
 * @param[in] op        Command opcode
 * @param[in] addr      Peer device address to be added or removed
 * @param[in] type      Peer address type (public or random)
 * @return true if whitelist management operation is allowed to be done, else false
 ****************************************************************************************
 */
bool app_easy_manage_wlist(app_wlist_op_t op, struct bd_addr addr, app_wlist_addr_t type);

#endif // _APP_EASY_WHITELIST_H_

///@}
///@}
