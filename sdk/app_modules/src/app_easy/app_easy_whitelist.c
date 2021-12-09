/**
 ****************************************************************************************
 *
 * @file app_easy_whitelist.c
 *
 * @brief Whitelist management API implementation.
 *
 * Copyright (C) 2019-2020 Dialog Semiconductor.
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

#include <stdint.h>
#include <stdbool.h>
#include "app_easy_whitelist.h"
#include "gapm_task.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

bool app_easy_manage_wlist(app_wlist_op_t op,
                           struct bd_addr addr,
                           app_wlist_addr_t type)
{
    uint8_t state = ke_state_get(TASK_GAPM);

    if (state != GAPM_DEVICE_SETUP)
    {
        struct gapm_white_list_mgt_cmd *cmd = KE_MSG_ALLOC_DYN(GAPM_WHITE_LIST_MGT_CMD,
                                                               TASK_GAPM,
                                                               TASK_APP,
                                                               gapm_white_list_mgt_cmd,
                                                               sizeof(struct gap_bdaddr));

        cmd->operation = op;
        cmd->nb        = 1;
        memcpy(&(cmd->devices->addr), &addr, BD_ADDR_LEN * sizeof(uint8_t));
        cmd->devices->addr_type = type;

        // Send the message
        ke_msg_send(cmd);

        return true;
    }
    return false;
}
/// @} APP
