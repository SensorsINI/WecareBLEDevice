/**
 ****************************************************************************************
 *
 * @file app_suotar_task.h
 *
 * @brief SUOTAR task header. SUOTA receiver handlers declaration.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_SUOTAR_TASK_H_
#define APP_SUOTAR_TASK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if BLE_SUOTA_RECEIVER

#include "suotar_task.h"
#include "suotar.h"
#include "ke_msg.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Process handler for the Application SUOTAR messages.
 * @param[in] msgid   Id of the message received
 * @param[in] param   Pointer to the parameters of the message
 * @param[in] dest_id ID of the receiving task instance (probably unused)
 * @param[in] src_id  ID of the sending task instance
 * @param[in] msg_ret Result of the message handler
 * @return Returns if the message is handled by the process handler
 ****************************************************************************************
 */
enum process_event_response app_suotar_process_handler(ke_msg_id_t const msgid,
                                                       void const *param,
                                                       ke_task_id_t const dest_id,
                                                       ke_task_id_t const src_id,
                                                       enum ke_msg_status_tag *msg_ret);

#endif //BLE_SUOTA_RECEIVER

#endif // APP_SUOTAR_TASK_H_
