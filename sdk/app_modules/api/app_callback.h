/**
 ****************************************************************************************
 * @addtogroup APP_Modules
 * @{
 * @addtogroup Callbacks
 * @brief Application Callbacks API
 * @{
 *
 * @file app_callback.h
 *
 * @brief Application callbacks definitions.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _APP_CALLBACK_H_
#define _APP_CALLBACK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "co_bt.h"
#include "gapc_task.h"
#include "gapm_task.h"
#include "gap.h"
#include "app.h"

#if (BLE_APP_SEC)
#include "app_security.h"
#endif // (BLE_APP_SEC)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// APP callbacks
struct app_callbacks
{
    /// Callback upon 'connection'
    void (*app_on_connection)(const uint8_t, struct gapc_connection_req_ind const *);

    /// Callback upon 'disconnect'
    void (*app_on_disconnect)(struct gapc_disconnect_ind const *);

    /// Callback upon 'update_params_rejected'
    void (*app_on_update_params_rejected)(const uint8_t);

    /// Callback upon 'update_params_complete'
    void (*app_on_update_params_complete)(void);

    /// Callback upon 'set_dev_config_complete'
    void (*app_on_set_dev_config_complete)(void);

    /// Callback upon 'adv_nonconn_complete'
    void (*app_on_adv_nonconn_complete)(const uint8_t);

    /// Callback upon 'adv_undirect_complete'
    void (*app_on_adv_undirect_complete)(const uint8_t);

    /// Callback upon 'adv_direct_complete'
    void (*app_on_adv_direct_complete)(const uint8_t);

    /// Callback upon 'db_init_complete'
    void (*app_on_db_init_complete)(void);

    /// Callback upon 'scanning_completed'
    void (*app_on_scanning_completed)(const uint8_t);

    /// Callback upon 'adv_report_ind'
    void (*app_on_adv_report_ind)(struct gapm_adv_report_ind const *);

    /// Callback upon 'connect_failed'
    void (*app_on_connect_failed)(void);

    /// Callback upon 'get_dev_name'
    void (*app_on_get_dev_name)(struct app_device_name *);

    /// Callback upon 'get_dev_appearance'
    void (*app_on_get_dev_appearance)(uint16_t*);

    /// Callback upon 'get_dev_slv_pref_params'
    void (*app_on_get_dev_slv_pref_params)(struct gap_slv_pref*);

    /// Callback upon 'set_dev_info'
    void (*app_on_set_dev_info)(struct gapc_set_dev_info_req_ind const *, uint8_t*);

    /// Callback upon 'data_length_change'
    void (*app_on_data_length_change)(const uint8_t, struct gapc_le_pkt_size_ind *);

    /// Callback upon 'update_params_request'
    void (*app_on_update_params_request)(struct gapc_param_update_req_ind const *, struct gapc_param_update_cfm *);

    /// Callback upon 'generate_static_random_addr'
    void (*app_on_generate_static_random_addr)(struct bd_addr *);

    /// Callback upon 'svc_changed_cfg_ind'
    void (*app_on_svc_changed_cfg_ind)(uint8_t, uint16_t);

    /// Callback upon 'get_peer_features'
    void (*app_on_get_peer_features)(const uint8_t, struct gapc_peer_features_ind const *);

#if (BLE_APP_SEC)
    /// Callback upon 'pairing_request'
    void (*app_on_pairing_request)(const uint8_t, struct gapc_bond_req_ind const *);

    /// Callback upon 'tk_exch'
    void (*app_on_tk_exch)(const uint8_t, struct gapc_bond_req_ind const *);

    /// Callback upon 'irk_exch'
    void (*app_on_irk_exch)(struct gapc_bond_req_ind const *);

    /// Callback upon 'csrk_exch'
    void (*app_on_csrk_exch)(const uint8_t, struct gapc_bond_req_ind const *);

    /// Callback upon 'ltk_exch'
    void (*app_on_ltk_exch)(const uint8_t, struct gapc_bond_req_ind const *);

    /// Callback upon 'pairing_succeeded'
    void (*app_on_pairing_succeeded)(const uint8_t);

    /// Callback upon 'encrypt_ind'
    void (*app_on_encrypt_ind)(const uint8_t, const uint8_t);

    /// Callback upon 'encrypt_req_ind'
    void (*app_on_encrypt_req_ind)(const uint8_t, struct gapc_encrypt_req_ind const *);

    /// Callback upon 'security_req_ind'
    void (*app_on_security_req_ind)(const uint8_t);

    /// Callback upon 'addr_solved_ind'
    void (*app_on_addr_solved_ind)(const uint8_t, struct gapm_addr_solved_ind const *);

    /// Callback upon 'addr_resolve_failed'
    void (*app_on_addr_resolve_failed)(const uint8_t);

    /// Callback upon 'ral_cmp_evt'
    void (*app_on_ral_cmp_evt)(struct gapm_cmp_evt const *);

    /// Callback upon 'ral_size_ind'
    void (*app_on_ral_size_ind)(const uint8_t);

    /// Callback upon 'ral_addr_ind'
    void (*app_on_ral_addr_ind)(const uint8_t, const uint8_t *);
#endif // (BLE_APP_SEC)
};

#if (BLE_APP_SEC)
/// APP Bond Database callbacks
struct app_bond_db_callbacks
{
    /// Callback upon 'bdb_init'
    void (*app_bdb_init)(void);

    /// Callback upon 'bdb_get_size'
    uint8_t (*app_bdb_get_size)(void);

    /// Callback upon 'bdb_add_entry'
    void (*app_bdb_add_entry)(struct app_sec_bond_data_env_tag *);

    /// Callback upon 'bdb_remove_entry'
    void (*app_bdb_remove_entry)(enum bdb_search_by_type, enum bdb_remove_type, void *, uint8_t);

    /// Callback upon 'bdb_search_entry'
    const struct app_sec_bond_data_env_tag * (*app_bdb_search_entry)(enum bdb_search_by_type, void *, uint8_t);

    /// Callback upon 'bdb_get_number_of_stored_irks'
    uint8_t (*app_bdb_get_number_of_stored_irks)(void);

    /// Callback upon 'bdb_get_stored_irks'
    uint8_t (*app_bdb_get_stored_irks)(struct gap_sec_key *);

    /// Callback upon 'bdb_get_device_info_from_slot'
    bool (*app_bdb_get_device_info_from_slot)(uint8_t, struct gap_ral_dev_info *);
};
#endif // (BLE_APP_SEC)

/*
 * DEFINES
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Helper macro calling a callback function taking no arguments.
 * @note Has no effect if the provided callback function is not defined.
 * @param[in] cb The callback function.
 ****************************************************************************************
 */
#define CALLBACK_ARGS_0(cb)                         {if (cb != NULL) cb();}

/**
 ****************************************************************************************
 * @brief Helper macro calling a callback function taking one argument.
 * @note Has no effect if the provided callback function is not defined.
 * @param[in] cb The callback function.
 * @param[in] arg1 The argument the callback function takes.
 ****************************************************************************************
 */
#define CALLBACK_ARGS_1(cb, arg1)                   {if (cb != NULL) cb(arg1);}

/**
 ****************************************************************************************
 * @brief Helper macro calling a callback function taking two arguments.
 * @note Has no effect if the provided callback function is not defined.
 * @param[in] cb The callback function.
 * @param[in] arg1 The first argument the callback function takes.
 * @param[in] arg2 The second argument the callback function takes.
 ****************************************************************************************
 */
#define CALLBACK_ARGS_2(cb, arg1, arg2)             {if (cb != NULL) cb(arg1, arg2);}

/**
 ****************************************************************************************
 * @brief Helper macro calling a callback function taking three arguments.
 * @note Has no effect if the provided callback function is not defined.
 * @param[in] cb The callback function.
 * @param[in] arg1 The first argument the callback function takes.
 * @param[in] arg2 The second argument the callback function takes.
 * @param[in] arg3 The third argument the callback function takes.
 ****************************************************************************************
 */
#define CALLBACK_ARGS_3(cb, arg1, arg2, arg3)       {if (cb != NULL) cb(arg1, arg2, arg3);}

/**
 ****************************************************************************************
 * @brief Helper macro calling a callback function taking four arguments.
 * @note Has no effect if the provided callback function is not defined.
 * @param[in] cb The callback function.
 * @param[in] arg1 The first argument the callback function takes.
 * @param[in] arg2 The second argument the callback function takes.
 * @param[in] arg3 The third argument the callback function takes.
 * @param[in] arg4 The fourth argument the callback function takes.
 ****************************************************************************************
 */
#define CALLBACK_ARGS_4(cb, arg1, arg2, arg3, arg4) {if (cb != NULL) cb(arg1, arg2, arg3, arg4);}

#endif // _APP_CALLBACK_H_

///@}
///@}
