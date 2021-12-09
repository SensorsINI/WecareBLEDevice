/**
 ****************************************************************************************
 *
 * @file user_noncon.h
 *
 * @brief Non-connectable advertising application header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_NONCON_H_
#define _USER_NONCON_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup
 *
 * @brief
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gapc_task.h"                 // gap functions and messages
#include "app_task.h"                  // application task
#include "app.h"                       // application definitions
#include "app_callback.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/* Duration of timer for connection parameter update request */
#define APP_PARAM_UPDATE_REQUEST_TO         (1000)   // 1000*10ms = 10sec, The maximum allowed value is 41943sec (4194300 * 10ms)

/* Advertising data update timer */
#define APP_ADV_DATA_UPDATE_TO              (100)    // 100*10ms = 1sec, The maximum allowed value is 41943sec (4194300 * 10ms)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Advertising function.
 ****************************************************************************************
*/
void user_app_adv_start(void);

/**
 ****************************************************************************************
 * @brief Non-connectable advertising completion function.
 * @param[in] status Command complete event message status
 ****************************************************************************************
*/
void user_app_adv_nonconn_complete(uint8_t status);

/// @} APP

#endif //_USER_NONCON_H_
