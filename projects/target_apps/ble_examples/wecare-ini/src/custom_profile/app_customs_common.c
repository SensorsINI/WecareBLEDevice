/**
 ****************************************************************************************
 *
 * @file app_customs_common.c
 * 
 * @brief Application Custom Service profile common file.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "rwble_config.h"              // SW configuration
#if (BLE_CUSTOM_SERVER)
#include <stdint.h>
#include "app_customs.h"
#include "app_prf_types.h"
#include "prf_types.h"
#include "attm_db.h"
#include "attm_db_128.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (BLE_CUSTOM1_SERVER)
uint8_t app_custs1_val_wr_validate(uint16_t att_idx, uint16_t offset, uint16_t len, uint8_t *value)
{
    return ATT_ERR_NO_ERROR;
}
#endif // (BLE_CUSTOM1_SERVER)

#if (BLE_CUSTOM2_SERVER)
uint8_t app_custs2_val_wr_validate(uint16_t att_idx, uint16_t offset, uint16_t len, uint8_t *value)
{
    return ATT_ERR_NO_ERROR;
}
#endif // (BLE_CUSTOM2_SERVER)

#endif // (BLE_CUSTOM_SERVER)
