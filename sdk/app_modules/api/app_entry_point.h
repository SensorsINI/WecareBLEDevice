/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Entry_Point Entry Point
 * @brief Application entry point API
 * @{
 *
 * @file app_entry_point.h
 *
 * @brief Application entry point header file.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _APP_ENTRY_POINT_H_
#define _APP_ENTRY_POINT_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "ke_msg.h"
#include "ke_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Helper macro to identify a task
#define KE_FIND_RELATED_TASK(task) ((ke_msg_id_t)((task) >> 10))

#if defined (__DA14531__) && !defined (__EXCLUDE_ROM_APP_TASK__) && (APP_EASY_MAX_ACTIVE_CONNECTION > 1)
    #error "Cannot use ROM app task symbols if number of connections is greater than 1. \
Workaround: Define the __EXCLUDE_ROM_APP_TASK__ flag and comment out the app task related symbols in ROM symbols file (da14531_symbols.txt)"
#endif

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Process event response
enum process_event_response
{
    /// Handled
    PR_EVENT_HANDLED = 0,

    /// Unhandled
    PR_EVENT_UNHANDLED
};

/// Format of a task message handler function
typedef enum process_event_response(*process_event_func_t)(ke_msg_id_t const msgid,
                                                           void const *param,
                                                           ke_task_id_t const dest_id,
                                                           ke_task_id_t const src_id,
                                                           enum ke_msg_status_tag *msg_ret);

/// Format of a catch rest event handler function
typedef void(*catch_rest_event_func_t)(ke_msg_id_t const msgid,
                                       void const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

#if defined (__DA14531__) && !defined (__EXCLUDE_ROM_APP_TASK__)

#include "app_prf_perm_types.h"

#if (BLE_APP_SEC)
#include "app_security.h"
#endif

/// APP Task configuration
typedef struct
{
    /// app_easy_gap_dev_configure() function
    void (*app_easy_gap_dev_configure)(void);

    /// app_db_init_next() function
    bool (*app_db_init_next)(void);

    /// User app callbacks
    const struct app_callbacks *user_app_callbacks;

    /// catch_rest_func() function
    const catch_rest_event_func_t catch_rest_func;

    /// APP process handlers
    const process_event_func_t *app_process_handlers;

    /// APP env
    struct app_env_tag *app_env;

    /// APP security env
    struct app_sec_bond_data_env_tag *app_sec_env;

    /// Profile access rights
    app_prf_srv_sec_t *app_prf_srv_perm;
} rom_app_task_cfg_t;

#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Application entry point handler.
 * @param[in] msgid       Message Id
 * @param[in] param       Pointer to message
 * @param[in] dest_id     Destination task Id
 * @param[in] src_id      Source task Id
 * @return Message status
 ****************************************************************************************
 */
int app_entry_point_handler(ke_msg_id_t const msgid,
                            void const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id);
                                       
/**
 ****************************************************************************************
 * @brief Application standard process event handler.
 * @param[in] msgid       Message Id
 * @param[in] param       Pointer to message
 * @param[in] src_id      Source task Id
 * @param[in] dest_id     Destination task Id
 * @param[in] msg_ret     Message status returned
 * @param[in] handlers    Pointer to message handlers
   @param[in] handler_num Handler number
 * @return process_event_response PR_EVENT_HANDLED or PR_EVENT_UNHANDLED
 ****************************************************************************************
 */
enum process_event_response app_std_process_event(ke_msg_id_t const msgid,
                                                  void const *param,
                                                  ke_task_id_t const src_id,
                                                  ke_task_id_t const dest_id,
                                                  enum ke_msg_status_tag *msg_ret,
                                                  const struct ke_msg_handler *handlers,
                                                  const int handler_num);

#endif // _APP_ENTRY_POINT_H_

///@}
///@}
