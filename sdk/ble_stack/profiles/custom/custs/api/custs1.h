/**
 ****************************************************************************************
 *
 * @file custs1.h
 *
 * @briefCustom Server 1 (CUSTS1) profile header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _CUSTS1_H_
#define _CUSTS1_H_

/// Device Information Service Server Role
#if !defined (BLE_SERVER_PRF)
    #define BLE_SERVER_PRF          1
#endif
#if !defined (BLE_CUSTOM_SERVER)
    #define BLE_CUSTOM_SERVER       1
#endif

#if (BLE_CUSTOM1_SERVER)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "prf_types.h"
#include "prf.h"
#include "attm_db_128.h"

#include "custom_common.h"

#define CUSTS1_IDX_MAX        (1)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Value element
struct custs1_val_elmt
{
    /// list element header
    struct co_list_hdr hdr;
    /// value identifier
    uint8_t att_idx;
    /// value length
    uint8_t length;
    /// value data
    uint8_t data[__ARRAY_EMPTY];
};

/// custs environment variable
struct custs1_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Service Start Handle
    uint16_t shdl;
    /// To store the DB max number of attributes
    uint8_t max_nb_att;
    /// On-going operation
    struct ke_msg *operation;

    /// Cursor on connection used to notify peer devices
    uint8_t cursor;
    /// CCC handle index, used during notification/indication busy state
    uint8_t ccc_idx;

    /// List of values set by application
    struct co_list values;
    /// CUSTS1 task state
    ke_state_t state[CUSTS1_IDX_MAX];
};

#if defined (__DA14531__) && !defined (__EXCLUDE_ROM_CUSTS1__)
typedef struct
{
    const uint8_t *custs1_services;
    const uint8_t *custs1_services_size;
    struct attm_desc_128* custs1_att_db;
    const struct cust_prf_func_callbacks* (*custs_get_func_callbacks)(enum KE_API_ID task_id);
} rom_cust_prf_cfg_t;
#endif

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Disable actions grouped in getting back to IDLE and sending configuration to requester task
 * @param[in] conhdl    Connection Handle
 ****************************************************************************************
 */
void custs1_disable(uint16_t conhdl);

const struct prf_task_cbs *custs1_prf_itf_get(void);

#endif // (BLE_CUSTOM1_SERVER)

#endif // _CUSTS1_H_
