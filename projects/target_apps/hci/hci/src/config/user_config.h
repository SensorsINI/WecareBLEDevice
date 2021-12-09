/**
 ****************************************************************************************
 *
 * @file user_config.h
 *
 * @brief User configuration file.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "arch_api.h"

/*
 * LOCAL VARIABLES
 ****************************************************************************************
 */

/******************************************
 * Default sleep mode. Possible values are:
 *
 * - ARCH_SLEEP_OFF
 * - ARCH_EXT_SLEEP_ON
 * - ARCH_EXT_SLEEP_OTP_COPY_ON
 *
 ******************************************
 */
static const sleep_state_t app_default_sleep_mode = ARCH_EXT_SLEEP_ON;

#endif // _USER_CONFIG_H_
