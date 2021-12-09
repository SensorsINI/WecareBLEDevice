/**
 ****************************************************************************************
 *
 * @file app_bcss_task.c
 *
 * @brief Body Composition Service application task.
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

#if (BLE_BCS_SERVER)

#include "bcss_task.h"
#include "app_entry_point.h"
#include "user_callback_config.h"

/*
 * LOCAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Message handler for the BCS Server Application message (service enable).
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int bcss_enable_rsp_handler(ke_msg_id_t const msgid,
                                   struct bcss_enable_rsp const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    CALLBACK_ARGS_3(user_app_bcss_cb.on_bcss_enable_rsp, KE_IDX_GET(src_id),
                    param->status, param->init_cfg_ind);
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Message handler for the BCS measurement send response.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int bcss_meas_val_ind_rsp_handler(ke_msg_id_t const msgid,
                                         struct bcss_meas_val_ind_rsp const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    CALLBACK_ARGS_2(user_app_bcss_cb.on_bcss_meas_val_ind_rsp,
                    KE_IDX_GET(src_id), param->status);
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Message handler for the BCS CCC descriptor configuration change.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int bcss_meas_val_ind_cfg_ind_handler(ke_msg_id_t const msgid,
                                             struct bcss_meas_val_ind_cfg_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    bool enabled = param->ind_cfg ? true : false;

    CALLBACK_ARGS_2(user_app_bcss_cb.on_bcss_ind_cfg_ind,
                    KE_IDX_GET(src_id), enabled);

    return (KE_MSG_CONSUMED);
}

static const struct ke_msg_handler app_bcss_process_handlers[] =
{
    {BCSS_ENABLE_RSP,               (ke_msg_func_t)bcss_enable_rsp_handler},
    {BCSS_MEAS_VAL_IND_RSP,         (ke_msg_func_t)bcss_meas_val_ind_rsp_handler},
    {BCSS_MEAS_VAL_IND_CFG_IND,     (ke_msg_func_t)bcss_meas_val_ind_cfg_ind_handler},
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

enum process_event_response app_bcss_process_handler(ke_msg_id_t const msgid,
                                                     void const *param,
                                                     ke_task_id_t const dest_id,
                                                     ke_task_id_t const src_id,
                                                     enum ke_msg_status_tag *msg_ret)
{
    return app_std_process_event(msgid, param,src_id, dest_id, msg_ret, app_bcss_process_handlers,
                                    sizeof(app_bcss_process_handlers) / sizeof(struct ke_msg_handler));
}

#endif // (BLE_BCS_SERVER)

#endif // (BLE_APP_PRESENT)

/// @} APPTASK
