/**
 ****************************************************************************************
 *
 * @file proxr_task.c
 *
 * @brief Proximity Reporter Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup PROXRTASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_PROX_REPORTER)

#include "gap.h"
#include "gattc_task.h"
#include "attm.h"
#include "atts.h"
#include "proxr.h"
#include "proxr_task.h"
#include "attm_cfg.h"
#include "prf_utils.h"
#include "rwip.h"
#include "lld.h"
#include "gapc.h"
#include "reg_ble_em_cs.h"
#if defined (__DA14531__)
#include "rf_531.h"
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of write request message.
 * The handler will analyse what has been set and decide alert level
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_write_req_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_write_req_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{

    int msg_status = KE_MSG_CONSUMED;

    if(ke_state_get(dest_id) == PROXR_IDLE)
    {
        uint8_t alert_lvl = 0x0000;
        uint8_t conidx = KE_IDX_GET(src_id);
        // Allocate write confirmation message.
        struct gattc_write_cfm *cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM,
                src_id, dest_id, gattc_write_cfm);

        // Get the address of the environment
        struct proxr_env_tag *proxr_env = PRF_ENV_GET(PROXR, proxr);

        // Fill in the parameter structure
        cfm->handle = param->handle;
        cfm->status = PRF_APP_ERROR;

        if(param->value[0] <= PROXR_ALERT_HIGH)
        {
            alert_lvl = param->value[0];
            // Store alert level on the environment
            if(param->handle == (proxr_env->lls_start_hdl + LLS_IDX_ALERT_LVL_VAL))
            {
                proxr_env->lls_alert_lvl[conidx] = alert_lvl;
                cfm->status = GAP_ERR_NO_ERROR;
            }
            else if(param->handle == (proxr_env->ias_start_hdl + IAS_IDX_ALERT_LVL_VAL))
            {
                // Allocate the alert value change indication
                struct proxr_alert_ind *ind = KE_MSG_ALLOC(PROXR_ALERT_IND,
                        prf_dst_task_get(&(proxr_env->prf_env), conidx),
                        dest_id, proxr_alert_ind);
                // Fill in the parameter structure
                ind->alert_lvl = alert_lvl;
                ind->char_code = PROXR_IAS_CHAR;
                ind->conidx = conidx;

                // Send the message
                ke_msg_send(ind);

                cfm->status = GAP_ERR_NO_ERROR;
            }
            else
            {
                cfm->status = ATT_ERR_REQUEST_NOT_SUPPORTED;
            }
        }

        // Send the message
        ke_msg_send(cfm);
    }
    else if(ke_state_get(dest_id) == PROXR_BUSY)
    {
        msg_status = KE_MSG_SAVED;
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the read request from peer device
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_read_req_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_read_req_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_CONSUMED;

    if(ke_state_get(dest_id) == PROXR_IDLE)
    {
        // Get the address of the environment
        struct proxr_env_tag *proxr_env = PRF_ENV_GET(PROXR, proxr);
        uint8_t conidx = KE_IDX_GET(src_id);

        uint8_t value = PROXR_ALERT_NONE;
        uint8_t status = ATT_ERR_NO_ERROR;

        if(param->handle == (proxr_env->lls_start_hdl + LLS_IDX_ALERT_LVL_VAL))
        {
            value = proxr_env->lls_alert_lvl[conidx];
        }
        else if(param->handle == (proxr_env->txp_start_hdl + TXPS_IDX_TX_POWER_LVL_VAL))
        {
            // send a request to read TX power level on GAPM
            struct gapm_get_dev_info_cmd* cmd = KE_MSG_ALLOC(GAPM_GET_DEV_INFO_CMD, TASK_GAPM, dest_id,
                    gapm_get_dev_info_cmd);
            cmd->operation = GAPM_GET_DEV_ADV_TX_POWER;
            ke_msg_send(cmd);

            // save the connection index
            proxr_env->conidx_saved = conidx;

            // need to wait for GAPM response, go into busy state
            ke_state_set(dest_id, PROXR_BUSY);

            status = PRF_APP_ERROR;
        }
        else
        {
            status = ATT_ERR_REQUEST_NOT_SUPPORTED;
        }

        // Don't send the confirmation if gap read request has been initiated.
        if(status != PRF_APP_ERROR)
        {
            // Send data to peer device
            struct gattc_read_cfm* cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id,
                    gattc_read_cfm, sizeof(uint8_t));
            cfm->length = sizeof(uint8_t);
            cfm->value[0] = value;
            cfm->handle = param->handle;
            cfm->status = status;

            // Send value to peer device.
            ke_msg_send(cfm);
        }
    }
    else if(ke_state_get(dest_id) == PROXR_BUSY)
    {
        msg_status = KE_MSG_SAVED;
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GAPM_DEV_ADV_TX_POWER_IND message.
 * This generic event is received for different requests, so need to keep track.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_dev_adv_tx_power_ind_handler(ke_msg_id_t const msgid,
                                            struct gapm_dev_adv_tx_power_ind const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct proxr_env_tag *proxr_env = PRF_ENV_GET(PROXR, proxr);

    if( ke_state_get(dest_id) == PROXR_BUSY )
    {
        // Send data to peer device
        struct gattc_read_cfm* cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, KE_BUILD_ID(TASK_GATTC, proxr_env->conidx_saved), dest_id,
                                                        gattc_read_cfm, sizeof(uint8_t));
        cfm->length = sizeof(uint8_t);
#if defined (__DA14531__)
            // 8-bit signed integer representation of the dBm values (see also
            // the rf_tx_pwr_lvl_t enum type in rf_531.h):
            // -19.5dBm --> -20 --> 0xEC --> RF_ATTR_REG.PA_POWER_SETTING = 1  (index 0 in lookup table)
            // -13.5dBm --> -14 --> 0xF2 --> RF_ATTR_REG.PA_POWER_SETTING = 2  (index 1 in lookup table)
            // -10dBm   --> -10 --> 0xF6 --> RF_ATTR_REG.PA_POWER_SETTING = 3  (index 2 in lookup table)
            // -7dBm    --> -7  --> 0xF9 --> RF_ATTR_REG.PA_POWER_SETTING = 4  (index 3 in lookup table)
            // -5dBm    --> -5  --> 0xFB --> RF_ATTR_REG.PA_POWER_SETTING = 5  (index 4 in lookup table)
            // -4dBm    --> -4  --> 0xFC --> RF_ATTR_REG.PA_POWER_SETTING = 6  (index 5 in lookup table)
            // -2dBm    --> -2  --> 0xFE --> RF_ATTR_REG.PA_POWER_SETTING = 7  (index 6 in lookup table)
            // -1dBm    --> -1  --> 0xFF --> RF_ATTR_REG.PA_POWER_SETTING = 8  (index 7 in lookup table)
            //  0       -->  0  --> 0x00 --> RF_ATTR_REG.PA_POWER_SETTING = 9  (index 8 in lookup table)
            // +1dBm    -->  1  --> 0x01 --> RF_ATTR_REG.PA_POWER_SETTING = 10 (index 9 in lookup table)
            // +1.5dBm  -->  2  --> 0x02 --> RF_ATTR_REG.PA_POWER_SETTING = 11 (index 10 in lookup table)
            // +2.5dBm  -->  3  --> 0x03 --> RF_ATTR_REG.PA_POWER_SETTING = 12 (index 11 in lookup table)
            uint8_t tx_pwr[RF_TX_PWR_LVL_TOTAL_VALUES] = {0xEC, 0xF2, 0xF6, 0xF9, 0xFB, 0xFC, 0xFE, 0xFF, 0x00, 0x01, 0x02, 0x03};
            // Provide the current (connection) TX power in dBm
            uint8_t tmp = rwip_rf.txpwr_dbm_get(proxr_env->conidx_saved, TX_PWR_LVL_CURRENT);
            cfm->value[0] = tx_pwr[tmp - 1];
#else
            // DA14585, DA14586
            cfm->value[0] = rwip_rf.txpwr_dbm_get(ble_txpwr_getf(gapc_get_conhdl(proxr_env->conidx_saved)), MOD_GFSK);
#endif
        cfm->handle = proxr_env->txp_start_hdl + TXPS_IDX_TX_POWER_LVL_VAL;
        cfm->status = GAP_ERR_NO_ERROR;

        // Send value to peer device.
        ke_msg_send(cfm);

        ke_state_set(dest_id, PROXR_IDLE);
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GAPM_CMP_EVT message.
 * This generic event is received for different requests, so need to keep track.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct proxr_env_tag *proxr_env = PRF_ENV_GET(PROXR, proxr);

    if( (ke_state_get(dest_id) == PROXR_BUSY) && (param->status != GAP_ERR_NO_ERROR))
    {
        // Send data to peer device
        struct gattc_read_cfm* cfm = KE_MSG_ALLOC(GATTC_READ_CFM, KE_BUILD_ID(TASK_GATTC,
                                        proxr_env->conidx_saved), dest_id, gattc_read_cfm);
        cfm->length = sizeof(uint8_t);
        cfm->handle = proxr_env->txp_start_hdl + TXPS_IDX_TX_POWER_LVL_VAL;
        cfm->status = PRF_APP_ERROR;

        // Send value to peer device.
        ke_msg_send(cfm);

        ke_state_set(dest_id, PROXR_IDLE);
    }
    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler proxr_default_state[] =
{
    {GATTC_WRITE_REQ_IND,                 (ke_msg_func_t) gattc_write_req_ind_handler},
    {GATTC_READ_REQ_IND,                  (ke_msg_func_t) gattc_read_req_ind_handler},
    {GAPM_DEV_ADV_TX_POWER_IND,           (ke_msg_func_t) gapm_dev_adv_tx_power_ind_handler},
    {GAPM_CMP_EVT,                        (ke_msg_func_t) gapm_cmp_evt_handler},
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler proxr_default_handler = KE_STATE_HANDLER(proxr_default_state);

#endif //BLE_PROX_REPORTER

/// @} PROXRTASK
