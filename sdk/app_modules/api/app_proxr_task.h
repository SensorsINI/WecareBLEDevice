/**
 ****************************************************************************************
 *
 * @file app_proxr_task.h
 *
 * @brief Proximity Reporter task header. Proximity Reporter Handlers declaration.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _APP_PROXR_TASK_H_
#define _APP_PROXR_TASK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_PROX_REPORTER)

#include "proxr_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Process handler for the Application PROXR messages.
 * @param[in] msgid   Id of the message received
 * @param[in] param   Pointer to the parameters of the message
 * @param[in] dest_id ID of the receiving task instance (probably unused)
 * @param[in] src_id  ID of the sending task instance
 * @param[in] msg_ret Result of the message handler
 * @return Returns if the message is handled by the process handler
 ****************************************************************************************
 */
enum process_event_response app_proxr_process_handler(ke_msg_id_t const msgid,
                                                      void const *param,
                                                      ke_task_id_t const dest_id,
                                                      ke_task_id_t const src_id,
                                                      enum ke_msg_status_tag *msg_ret);


#endif //BLE_PROX_REPORTER

#endif // _APP_PROXR_TASK_H_
