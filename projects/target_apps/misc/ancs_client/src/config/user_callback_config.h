/**
 ****************************************************************************************
 *
 * @file user_callback_config.h
 *
 * @brief Callback functions configuration file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_CALLBACK_CONFIG_H_
#define _USER_CALLBACK_CONFIG_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdio.h>
#include "app_callback.h"
#include "app_default_handlers.h"
#include "app_entry_point.h"
#include "app_prf_types.h"
#if (BLE_APP_SEC)
#include "app_bond_db.h"
#endif // (BLE_APP_SEC)
#include "user_ancs_client.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

static const struct app_gattc_cb user_app_gattc_cb = {
    .on_gattc_enable            = user_on_gattc_enable,
    .on_gattc_rd_cfg_ind_rsp    = NULL,
    .on_gattc_wr_cfg_ind_rsp    = NULL,
    .on_gattc_svc_changed_ind   = user_on_gattc_svc_change_indication,
};

static const struct app_ancc_cb user_app_ancc_cb = {
    .on_ancc_enable             = user_on_annc_enable_cb,
    .on_wr_cfg_ntf_src_rsp      = user_on_ancc_wr_cfg_ntf_src_cb,
    .on_wr_cfg_data_src_rsp     = user_on_ancc_wr_cfg_data_src_cb,
    .on_rd_cfg_ntf_src_rsp      = NULL,
    .on_rd_cfg_data_src_rsp     = NULL,
    .on_ntf_src_ind             = user_on_ancc_ntf_src_ind_cb,
    .on_ntf_att_ind             = user_on_ancc_ntf_att_ind_cb,
    .on_app_att_ind             = user_on_ancc_app_att_ind_cb,
    .on_get_ntf_atts_cmd_cmp    = user_on_ancc_get_ntf_att_cmp_cb,
    .on_get_app_atts_cmd_cmp    = user_on_ancc_get_app_att_cmp_cb,
    .on_perf_ntf_act_cmd_cmp    = user_on_ancc_perf_ntf_act_cmp_cb
};

static const struct app_callbacks user_app_callbacks = {
    .app_on_connection                  = user_app_connection,
    .app_on_disconnect                  = user_app_disconnect,
    .app_on_update_params_rejected      = NULL,
    .app_on_update_params_complete      = NULL,
    .app_on_set_dev_config_complete     = default_app_on_set_dev_config_complete,
    .app_on_adv_nonconn_complete        = NULL,
    .app_on_adv_undirect_complete       = NULL,
    .app_on_adv_direct_complete         = NULL,
    .app_on_db_init_complete            = default_app_on_db_init_complete,
    .app_on_scanning_completed          = NULL,
    .app_on_adv_report_ind              = NULL,
    .app_on_get_dev_name                = default_app_on_get_dev_name,
    .app_on_get_dev_appearance          = default_app_on_get_dev_appearance,
    .app_on_get_dev_slv_pref_params     = default_app_on_get_dev_slv_pref_params,
    .app_on_set_dev_info                = default_app_on_set_dev_info,
    .app_on_data_length_change          = NULL,
    .app_on_update_params_request       = default_app_update_params_request,
    .app_on_generate_static_random_addr = default_app_generate_static_random_addr,
    .app_on_svc_changed_cfg_ind         = NULL,
    .app_on_get_peer_features           = NULL,
#if (BLE_APP_SEC)
    .app_on_pairing_request             = default_app_on_pairing_request,
    .app_on_tk_exch                     = user_app_on_tk_exch,
    .app_on_irk_exch                    = NULL,
    .app_on_csrk_exch                   = default_app_on_csrk_exch,
    .app_on_ltk_exch                    = default_app_on_ltk_exch,
    .app_on_pairing_succeeded           = user_app_on_pairing_succeeded,
    .app_on_encrypt_ind                 = NULL,
    .app_on_encrypt_req_ind             = default_app_on_encrypt_req_ind,
    .app_on_security_req_ind            = NULL,
    .app_on_addr_solved_ind             = default_app_on_addr_solved_ind,
    .app_on_addr_resolve_failed         = default_app_on_addr_resolve_failed,
    .app_on_ral_cmp_evt                 = NULL,
    .app_on_ral_size_ind                = NULL,
    .app_on_ral_addr_ind                = NULL,
#endif // (BLE_APP_SEC)
};

#if (BLE_APP_SEC)
static const struct app_bond_db_callbacks user_app_bond_db_callbacks = {
    .app_bdb_init                       = default_app_bdb_init,
    .app_bdb_get_size                   = NULL,
    .app_bdb_add_entry                  = default_app_bdb_add_entry,
    .app_bdb_remove_entry               = NULL,
    .app_bdb_search_entry               = default_app_bdb_search_entry,
    .app_bdb_get_number_of_stored_irks  = default_app_bdb_get_number_of_stored_irks,
    .app_bdb_get_stored_irks            = default_app_bdb_get_stored_irks,
    .app_bdb_get_device_info_from_slot  = NULL,
};
#endif // (BLE_APP_SEC)

static const catch_rest_event_func_t app_process_catch_rest_cb = (catch_rest_event_func_t)user_catch_rest_hndl;

static const struct arch_main_loop_callbacks user_app_main_loop_callbacks = {
    .app_on_init            = user_app_init,

    // By default the watchdog timer is reloaded and resumed when the system wakes up.
    // The user has to take into account the watchdog timer handling (keep it running,
    // freeze it, reload it, resume it, etc), when the app_on_ble_powered() is being
    // called and may potentially affect the main loop.
    .app_on_ble_powered     = NULL,

    // By default the watchdog timer is reloaded and resumed when the system wakes up.
    // The user has to take into account the watchdog timer handling (keep it running,
    // freeze it, reload it, resume it, etc), when the app_on_system_powered() is being
    // called and may potentially affect the main loop.
    .app_on_system_powered  = NULL,

    .app_before_sleep       = NULL,
    .app_validate_sleep     = NULL,
    .app_going_to_sleep     = NULL,
    .app_resume_from_sleep  = NULL,
};


// Default Handler Operations
static const struct default_app_operations user_default_app_operations = {
    .default_operation_adv = default_advertise_operation,
};

// Place in this structure the app_<profile>_db_create and app_<profile>_enable functions
// for SIG profiles that do not have this function already implemented in the SDK
// or if you want to override the functionality. Check the prf_func array in the SDK
// for your reference of which profiles are supported.
static const struct prf_func_callbacks user_prf_funcs[] =
{
    {TASK_ID_INVALID,     NULL, NULL}   // DO NOT MOVE. Must always be last
};

#endif // _USER_CALLBACK_CONFIG_H_
