/**
 ****************************************************************************************
 *
 * @file user_proxr.c
 *
 * @brief Proximity reporter external processor user application source code.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"             // SW configuration
#include "user_periph_setup.h"             // SW configuration
#include "user_proxr.h"
#include "arch_api.h"
#include "user_config.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

/**
 ****************************************************************************************
 * @brief User code initialization function.
 ****************************************************************************************
*/

void user_on_init(void)
{
    arch_set_sleep_mode(app_default_sleep_mode);
}

/// @} APP
