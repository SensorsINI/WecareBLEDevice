/**
 ****************************************************************************************
 *
 * @file custom_common.h
 *
 * @brief Custom Service profile common header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef __CUSTOM_COMMON_H
#define __CUSTOM_COMMON_H

#include "gattc_task.h"

/**
 ****************************************************************************************
 * @brief Validate the value of the Client Characteristic CFG.
 * @param[in] is_notification indicates whether the CFG is Notification (true) or Indication (false)
 * @param[in] param Pointer to the parameters of the message.
 * @return status.
 ****************************************************************************************
 */
int check_client_char_cfg(bool is_notification, const struct gattc_write_req_ind *param);

/**
 ****************************************************************************************
 * @brief Find the handle of the Characteristic Value having as input the Client Characteristic CFG handle
 * @param[in] cfg_handle the Client Characteristic CFG handle
 * @return the corresponding value handle
 ****************************************************************************************
 */
uint16_t get_value_handle(uint16_t cfg_handle);

/**
 ****************************************************************************************
 * @brief Find the handle of Client Characteristic CFG having as input the Characteristic value handle
 * @param[in] value_handle the Characteristic value handle
 * @return the corresponding Client Characteristic CFG handle
 ****************************************************************************************
 */
uint16_t get_cfg_handle(uint16_t value_handle);

#if (BLE_CUSTOM1_SERVER)
/**
 ****************************************************************************************
 * @brief Compute the handle of a given attribute based on its index
 * @details Specific to custom profile / service 1
 * @param[in] att_idx attribute index
 * @return the corresponding handle
 ****************************************************************************************
 */
uint16_t custs1_get_att_handle(uint8_t att_idx);

/**
 ****************************************************************************************
 * @brief Compute the handle of a given attribute based on its index
 * @details Specific to custom profile / service 1
 * @param[in] handle attribute handle
 * @param[out] att_idx attribute index
 * @return high layer error code
 ****************************************************************************************
 */
uint8_t custs1_get_att_idx(uint16_t handle, uint8_t *att_idx);
#endif // (BLE_CUSTOM1_SERVER)

#if (BLE_CUSTOM2_SERVER)
/**
 ****************************************************************************************
 * @brief Compute the handle of a given attribute based on its index
 * @details Specific to custom profile / service 2
 * @param[in] att_idx attribute index
 * @return the corresponding handle
 ****************************************************************************************
 */
uint16_t custs2_get_att_handle(uint8_t att_idx);

/**
 ****************************************************************************************
 * @brief Compute the handle of a given attribute based on its index
 * @details Specific to custom profile / service 2
 * @param[in] handle attribute handle
 * @param[out] att_idx attribute index
 * @return high layer error code
 ****************************************************************************************
 */
uint8_t custs2_get_att_idx(uint16_t handle, uint8_t *att_idx);
#endif // (BLE_CUSTOM2_SERVER)

#endif // __CUSTOM_COMMON_H
