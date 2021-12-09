/**
 ****************************************************************************************
 *
 * @file user_callback_config.h
 *
 * @brief Callback functions configuration file.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_CALLBACK_CONFIG_H_
#define _USER_CALLBACK_CONFIG_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "user_commands.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

static const struct arch_main_loop_callbacks user_app_main_loop_callbacks = {
    .app_on_init            = user_on_init,

    // By default the watchdog timer is reloaded and resumed when the system wakes up.
    // The user has to take into account the watchdog timer handling (keep it running,
    // freeze it, reload it, resume it, etc), when the app_on_ble_powered() is being
    // called and may potentially affect the main loop.
    .app_on_ble_powered     = NULL,

    // By default the watchdog timer is reloaded and resumed when the system wakes up.
    // The user has to take into account the watchdog timer handling (keep it running,
    // freeze it, reload it, resume it, etc), when the app_on_system_powered() is being
    // called and may potentially affect the main loop.
    .app_on_system_powered  = NULL,

    .app_before_sleep       = NULL,
    .app_validate_sleep     = NULL,
    .app_going_to_sleep     = NULL,
    .app_resume_from_sleep  = NULL,
};

#endif // _USER_CALLBACK_CONFIG_H_
