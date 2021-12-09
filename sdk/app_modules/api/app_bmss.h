/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Profiles
 * @{
 * @addtogroup BMSS
 * @brief Bond Management Service Server Application API
 * @{
 *
 * @file app_bmss.h
 *
 * @brief Bond Management Service Server Application header file
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_BMSS_H_
#define APP_BMSS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "user_profiles_config.h"

#if BLE_BMS_SERVER

#include "bms_common.h"

/*
 * VARIABLES DECLARATION
 ****************************************************************************************
 */

/// BMSS APP callbacks
struct app_bmss_cb
{
    /// Callback upon 'del_bond_req_ind'
    void (*on_bmss_del_bond_req_ind)(uint8_t con_idx,
                                        const struct bms_cntl_point_op *operation);
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes BMSS Apllication.
 ****************************************************************************************
 */
void app_bmss_init(void);

/**
 ****************************************************************************************
 * @brief Creates BMSS service database.
 ****************************************************************************************
 */
void app_bmss_create_db(void);

/**
 ****************************************************************************************
 * @brief Sends bond operation status to Bond Management Service.
 * @param[in] conidx Connection index
 * @param[in] status Operation status
 ****************************************************************************************
 */
void app_bmss_del_bond_cfm(uint8_t conidx, uint8_t status);

#endif // BLE_BMS_SERVER

#endif // APP_BMSS_H_

///@}
///@}
///@}
