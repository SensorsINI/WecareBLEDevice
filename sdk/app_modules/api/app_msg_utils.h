/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Msg_Utils Msg Utils
 * @brief Message utilities API
 * @{
 *
 * @file app_msg_utils.h
 *
 * @brief Complementary API to handle user defined messages.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _APP_MSG_UTILS_H_
#define _APP_MSG_UTILS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "ke_msg.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if the BLE is active or not.
 * @return Returns true if BLE is active.
 ****************************************************************************************
 */
bool app_check_BLE_active(void);

/**
 ****************************************************************************************
 * @brief Sends a message and wakes BLE up if not active.
 * @param[in] cmd Pointer to the message.
 ****************************************************************************************
 */
void app_msg_send_wakeup_ble(void *cmd);

/**
 ****************************************************************************************
 * @brief Wrapper to the kernel send message function.
 * @param[in] cmd Pointer to the message.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void app_msg_send(void *cmd)
{
    ke_msg_send(cmd);
}

#endif // _APP_MSG_UTILS_H_

///@}
///@}
