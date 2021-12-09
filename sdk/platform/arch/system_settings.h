/**
 ****************************************************************************************
 *
 * @file system_settings.h
 *
 * @brief DA14585/586 RF preferred settings.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _SYSTEM_SETTINGS_H_
#define _SYSTEM_SETTINGS_H_

/*
 * Radio preferred settings
 ****************************************************************************************
 */
#if !defined (__DA14531__)
#define PREF_BLE_RADIOPWRUPDN_REG           0x754054C
#define PREF_RF_ENABLE_CONFIG13_REG         0xD030
#define PREF_RF_DC_OFFSET_CTRL3_REG         0xE6EB
#define PREF_RF_VCOCAL_CTRL_REG             0x63
#endif

#endif // _SYSTEM_SETTINGS_H_
