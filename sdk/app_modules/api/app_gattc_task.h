/**
 ****************************************************************************************
 *
 * @file app_gattc_task.h
 *
 * @brief Generic Attribute Profile Service Client Application Task header file.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_GATTC_TASK_H_
#define APP_GATTC_TASK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_GATT_CLIENT)

#include "gatt_client_task.h"
#include "ke_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Process handler for the Application GATT Client messages.
 * @param[in] msgid   Id of the message received
 * @param[in] param   Pointer to the parameters of the message
 * @param[in] dest_id ID of the receiving task instance (probably unused)
 * @param[in] src_id  ID of the sending task instance
 * @param[in] msg_ret Result of the message handler
 * @return Returns if the message is handled by the process handler
 ****************************************************************************************
 */
enum process_event_response app_gattc_process_handler(ke_msg_id_t const msgid,
                                                      void const *param,
                                                      ke_task_id_t const dest_id,
                                                      ke_task_id_t const src_id,
                                                      enum ke_msg_status_tag *msg_ret);

#endif // (BLE_GATT_CLIENT)

#endif // APP_GATTC_TASK_H_
