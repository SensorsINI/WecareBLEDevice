/**
 ****************************************************************************************
 *
 * @file app_ctsc_task.h
 *
 * @brief Header file - APPCTSTASK.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_CTSC_TASK_H_
#define APP_CTSC_TASK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_CTS_CLIENT)

#include "ctsc_task.h"
#include "ke_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximal number of APP CTS Task instances
#define APP_CTS_IDX_MAX        (1)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Process handler for the Application CTSC messages.
 * @param[in] msgid   Id of the message received
 * @param[in] param   Pointer to the parameters of the message
 * @param[in] dest_id ID of the receiving task instance (probably unused)
 * @param[in] src_id  ID of the sending task instance
 * @param[in] msg_ret Result of the message handler
 * @return Returns if the message is handled by the process handler
 ****************************************************************************************
 */
enum process_event_response app_ctsc_process_handler(ke_msg_id_t const msgid,
                                                     void const *param,
                                                     ke_task_id_t const dest_id,
                                                     ke_task_id_t const src_id,
                                                     enum ke_msg_status_tag *msg_ret);

#endif //(BLE_CTS_CLIENT)

#endif //APP_CTSC_TASK_H_
