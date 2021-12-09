/**
 ****************************************************************************************
 *
 * @file app_ancc_task.h
 *
 * @brief Apple Notification Center Service Client Application Task header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_ANCC_TASK_H_
#define APP_ANCC_TASK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_ANC_CLIENT)

#include "ancc_task.h"
#include "ke_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximal number of APP ANC Task instances
#define APP_ANC_IDX_MAX        (1)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Process handler for the Application ANCC messages.
 * @param[in] msgid   Id of the message received
 * @param[in] param   Pointer to the parameters of the message
 * @param[in] dest_id ID of the receiving task instance (probably unused)
 * @param[in] src_id  ID of the sending task instance
 * @param[in] msg_ret Result of the message handler
 * @return Returns if the message is handled by the process handler
 ****************************************************************************************
 */
enum process_event_response app_ancc_process_handler(ke_msg_id_t const msgid,
                                                     void const *param,
                                                     ke_task_id_t const dest_id,
                                                     ke_task_id_t const src_id,
                                                     enum ke_msg_status_tag *msg_ret);

#endif // (BLE_ANC_CLIENT)

#endif // APP_ANCC_TASK_H_
