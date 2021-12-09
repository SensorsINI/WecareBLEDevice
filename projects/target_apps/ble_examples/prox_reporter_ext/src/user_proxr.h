/**
 ****************************************************************************************
 *
 * @file user_proxr.h
 *
 * @brief Proximity reporter external processor user application header file.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_PROXR_H_
#define _USER_PROXR_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Accelerometer Application entry point.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"
#include "gapc_task.h"                  // gap functions and messages
#include "gapm_task.h"                  // gap functions and messages
#include "co_error.h"                   // error code definitions
#include "arch_api.h"                  // error code definitions
                  
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

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
 * @brief User code initialization function.
 ****************************************************************************************
*/

void user_on_init(void);

/// @} APP

#endif // _USER_PROXR_H_
