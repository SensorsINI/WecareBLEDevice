/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Utils
 * @brief Application utilities API
 * @{
 *
 * @file app_utils.h
 *
 * @brief Application generic utilities helper functions header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _APP_UTILS_H_
#define _APP_UTILS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "rwip_config.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Address types
enum app_addr_types
{
    /// Public address
    APP_PUBLIC_ADDR_TYPE,
    /// Random static address
    APP_RANDOM_STATIC_ADDR_TYPE,
    /// Resolvable private address
    APP_RANDOM_PRIVATE_RESOLV_ADDR_TYPE,
    /// Non-resolvable private address
    APP_RANDOM_PRIVATE_NONRESOLV_ADDR_TYPE,
    /// Address resolved by the Controller
    APP_ID_ADDR_TYPE,
    /// Unknown address type
    APP_ERROR_ADDR_TYPE,
};

/// Resolving address list operations
enum app_ral_operations
{
    /// Get resolving address list size
    APP_GET_RAL_SIZE = GAPM_GET_RAL_SIZE,
    /// Get resolving local address
    APP_GET_RAL_LOC_ADDR = GAPM_GET_RAL_LOC_ADDR,
    /// Get resolving peer address
    APP_GET_RAL_PEER_ADDR = GAPM_GET_RAL_PEER_ADDR,
    /// Add device in resolving address list
    APP_ADD_DEV_IN_RAL = GAPM_ADD_DEV_IN_RAL,
    /// Remove device from resolving address list
    APP_RMV_DEV_FRM_RAL = GAPM_RMV_DEV_FRM_RAL,
    /// Clear resolving address list
    APP_CLEAR_RAL = GAPM_CLEAR_RAL,
    /// Set Network Privacy Mode for peer in resolving list
    APP_NETWORK_MODE_RAL = GAPM_NETWORK_MODE_RAL,
    /// Set Network Privacy Mode for peer in resolving list
    APP_DEVICE_MODE_RAL = GAPM_DEVICE_MODE_RAL,
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Find the type of Bluetooth Device Address
 * @param[in] addr_type PUBLIC or RANDOM type of BDA
 * @param[in] addr      Bluetooth Device Address
 * @return              Type of Bluetooth Device Address
 ****************************************************************************************
 */
enum app_addr_types app_get_address_type(uint8_t addr_type, struct bd_addr addr);

/**
 ****************************************************************************************
 * @brief Fills an array with random bytes (starting from the end of the array)
 *          Assuming an array of array_size and a dynamic key size, so that key_size = M*4+N:
 *          Calls co_rand_word() M times and appends the 4 bytes of each 32 bit return value
 *          to the output array
 *          Calls co_rand_word() once and appends the N most significant bytes of the 32 bit
 *          return value to the output array
 *          Fills the rest bytes of the array with zeroes
 *
 * @param[in] *dst              The address of the array
 * @param[in] key_size          Number of bytes that shall be randomized
 * @param[in] array_size        Total size of the array
 ****************************************************************************************
 */
void app_fill_random_byte_array(uint8_t *dst, uint8_t key_size, uint8_t array_size);

#endif // _APP_UTILS_H_

///@}
///@}
