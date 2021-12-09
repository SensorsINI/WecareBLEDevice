/**
 ****************************************************************************************
 *
 * @file app_bmss_task.c
 *
 * @brief Bond Management Service application task.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_APP_PRESENT)

#if (BLE_BMS_SERVER)

#include "bmss_task.h"
#include "app_entry_point.h"
#include "user_callback_config.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static int bmss_del_bond_req_ind_handler(ke_msg_id_t const msgid,
                                            struct bmss_del_bond_req_ind const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    CALLBACK_ARGS_2(user_app_bmss_cb.on_bmss_del_bond_req_ind, KE_IDX_GET(src_id),
                    &param->operation)

    return (KE_MSG_CONSUMED);
}

/*
 * LOCAL VARIABLES DEFINITION
 ****************************************************************************************
 */

static const struct ke_msg_handler app_bmss_process_handlers[] =
{
    {BMSS_DEL_BOND_REQ_IND, (ke_msg_func_t)bmss_del_bond_req_ind_handler},
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

enum process_event_response app_bmss_process_handler(ke_msg_id_t const msgid,
                                                     void const *param,
                                                     ke_task_id_t const dest_id,
                                                     ke_task_id_t const src_id,
                                                     enum ke_msg_status_tag *msg_ret)
{
    return app_std_process_event(msgid, param,src_id, dest_id, msg_ret, app_bmss_process_handlers,
                                    sizeof(app_bmss_process_handlers) / sizeof(struct ke_msg_handler));
}

#endif //(BLE_BMS_SERVER)

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
