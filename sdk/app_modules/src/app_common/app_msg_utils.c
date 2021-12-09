/**
 ****************************************************************************************
 *
 * @file app_easy_msg_utils.c
 *
 * @brief Message related helper function.
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_APP_PRESENT)
#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "ke_msg.h"
#include "app_msg_utils.h"
#include "arch_api.h"
#include "rwip.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

bool app_check_BLE_active(void)
{
    return ((GetBits16(CLK_RADIO_REG, BLE_ENABLE) == 1) &&
            (GetBits32(BLE_DEEPSLCNTL_REG, DEEP_SLEEP_STAT) == 0) &&
            !(rwip_prevent_sleep_get() & RW_WAKE_UP_ONGOING));
}

void app_msg_send_wakeup_ble(void *cmd)
{
   if (!app_check_BLE_active())
      arch_ble_force_wakeup();
    ke_msg_send(cmd);
}

#endif //(BLE_APP_PRESENT)

/// @} APP
