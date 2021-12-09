/**
 ****************************************************************************************
 *
 * @file bcss.h
 *
 * @brief Header file - Body Composition Service Server.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef BCSS_H_
#define BCSS_H_

/**
 ****************************************************************************************
 * @addtogroup BCSS Body Composition Service Server
 * @ingroup BCS
 * @brief Body Composition Service Server
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_BCS_SERVER)

#include "prf_types.h"
#include "prf_utils.h"
#include "bcs_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define BCSS_IDX_MAX            (1)
#define BCSS_MANDATORY_MASK     (0x3F)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Body Composition Service Attributes Indexes
enum
{
    /// Body Composition Service
    BCS_IDX_SVC,
    /// Body Composition Feature
    BCS_IDX_FEAT_CHAR,
    BCS_IDX_FEAT_VAL,
    /// Body Composition Measurement
    BCS_IDX_MEAS_CHAR,
    BCS_IDX_MEAS_VAL,
    BCS_IDX_MEAS_IND_CFG,
    /// Number of attributes
    BCS_IDX_NB,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
///Body Composition Service Server Environment Variable
struct bcss_env_tag
{
    /// Connection Info
    prf_env_t prf_env;
    /// Service Start HandleVAL
    uint16_t shdl;
    /// Last measurement set by the App
    bcs_meas_t *meas;
    // Body Composition Feature
    uint32_t bc_feat;
    ///Event (notification/indication) config
    uint16_t ind_cfg[BLE_CONNECTION_MAX];
    /// State of different task instances
    ke_state_t state[BCSS_IDX_MAX];
    /// Features for measurements that needs to be indicated
    uint16_t ind_cont_feat;
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve BCS service profile interface
 *
 * @return BCS service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs* bcss_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Measurement Value Indication.
 * @param[in] conidx    Connection index.
 * @param[in] features  Measurement features.
 * @param[in] meas      Measurement to indicate.
 ****************************************************************************************
 */
void bcss_indicate(uint8_t conidx, uint16_t features, const bcs_meas_t *meas);

/**
 ****************************************************************************************
 * @brief Send a BCSS_MEAS_VAL_IND_RSP message to the application to inform it about the
 * status of the indication that was send
 * @param[in] conidx    Connection index.
 * @param[in] status    Status.
 ****************************************************************************************
 */
void bcss_ind_rsp_send(uint8_t conidx, uint8_t status);

#endif // BLE_BCS_SERVER

/// @} BCSS

#endif // BCSS_H_
