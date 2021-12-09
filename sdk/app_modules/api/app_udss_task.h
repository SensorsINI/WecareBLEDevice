/**
 ****************************************************************************************
 *
 * @file app_udss_task.h
 *
 * @brief User Data Service application task header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_UDSS_TASK_H_
#define APP_UDSS_TASK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"

#if (BLE_UDS_SERVER)

#include "ke_msg.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Process handler for the Application UDSS messages.
 * @param[in] msgid   Id of the message received
 * @param[in] param   Pointer to the parameters of the message
 * @param[in] dest_id ID of the receiving task instance (probably unused)
 * @param[in] src_id  ID of the sending task instance
 * @param[in] msg_ret Result of the message handler
 * @return Returns if the message is handled by the process handler
 ****************************************************************************************
 */
enum process_event_response app_udss_process_handler(ke_msg_id_t const msgid,
                                                     void const *param,
                                                     ke_task_id_t const dest_id,
                                                     ke_task_id_t const src_id,
                                                     enum ke_msg_status_tag *msg_ret);

#endif // BLE_UDS_SERVER

#endif // APP_UDSS_TASK_H_
