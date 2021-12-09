/**
 ****************************************************************************************
 * @addtogroup Core_Modules
 * @{
 * @addtogroup RF
 * @brief RF driver
 * @{
 *
 * @file rf_585.h
 *
 * @brief DA14585 RF control Header File.
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _RF_585_H_
#define _RF_585_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip.h"               // for RF API structure definition

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 *****************************************************************************************
 * @brief Initialize Radio module.
 * @param[in] api  Stucture of API functions of the RF driver that are used by the BLE
 *                 software.
 *****************************************************************************************
 */
void rf_init(struct rwip_rf_api *api);

/**
 *****************************************************************************************
 * @brief Re-initialize Radio module. Called when system exits sleep mode.
 *****************************************************************************************
 */
void rf_reinit(void);

/**
 *****************************************************************************************
 * @brief Initialize RF module registers.
 *****************************************************************************************
 */
void rf_regs(void);

/*
 * NEAR FIELD MODE API
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Checks if Near Field mode is enabled.
 * @return true if Near Field mode is enabled, otherwise false.
 ****************************************************************************************
 */
bool rf_nfm_is_enabled(void);

/**
 ****************************************************************************************
 * @brief Enables Near Field mode for all connections.
 ****************************************************************************************
 */
void rf_nfm_enable(void);

/**
 ****************************************************************************************
 * @brief Disables Near Field mode for all connections.
 ****************************************************************************************
 */
void rf_nfm_disable(void);

#endif // _RF_585_H_

/// @}
/// @}
