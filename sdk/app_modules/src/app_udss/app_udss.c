/**
 ****************************************************************************************
 *
 * @file app_udss.c
 *
 * @brief User Data Service Server Application entry point
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"

#if (BLE_UDS_SERVER)

#include "app_api.h"
#include "app_udss.h"
#include "app_prf_perm_types.h"

#include "uds_common.h"
#include "udss_task.h"

/**
 ****************************************************************************************
 * User Data Service Server Application Functions
 ****************************************************************************************
 */

void app_udss_init(void)
{
    // nothing to do
}

void app_udss_create_db(void)
{
    struct udss_db_cfg* db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM,
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd,
                                                             sizeof(struct udss_db_cfg));

    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = get_user_prf_srv_perm(TASK_ID_UDSS);
    req->prf_task_id = TASK_ID_UDSS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    db_cfg = (struct udss_db_cfg* ) req->param;
    db_cfg->enabled_chars = APP_UDS_ENABLED_UDS_CHARS;
    db_cfg->local_char_update_enabled = APP_UDS_LOCAL_CHAR_UPDATE_ENABLED;

    ke_msg_send(req);
}

void app_udss_cntl_point_operation_cfm(uint8_t conidx, const struct uds_ucp_rsp *rsp)
{
    struct udss_env_tag *udss_env = PRF_ENV_GET(UDSS, udss);

    struct udss_ucp_req_cfm * cfm = KE_MSG_ALLOC(UDSS_UCP_REQ_CFM,
                                                 prf_src_task_get(&udss_env->prf_env, conidx),
                                                 TASK_APP, udss_ucp_req_cfm);

    memcpy(&cfm->ucp_rsp, rsp, sizeof(struct uds_ucp_rsp));

    ke_msg_send(cfm);
}

void app_udss_char_val_rsp(uint8_t conidx, uint8_t char_code, uint8_t length,
                           const uint8_t *value)
{
    struct udss_env_tag *udss_env = PRF_ENV_GET(UDSS, udss);
    struct udss_char_val *rsp = KE_MSG_ALLOC_DYN(UDSS_CHAR_VAL_RSP,
                                                 prf_src_task_get(&udss_env->prf_env, conidx),
                                                 TASK_APP, udss_char_val, length);

    if (length)
    {
        memcpy(rsp->value, value, length);
    }

    rsp->length = length;
    rsp->char_code = char_code;

    ke_msg_send(rsp);
}

void app_udss_set_char_val_cfm(uint8_t conidx, uint8_t char_code, uint8_t status)
{
    struct udss_env_tag *udss_env = PRF_ENV_GET(UDSS, udss);
    struct udss_char_val_cfm *cfm = KE_MSG_ALLOC(UDSS_SET_CHAR_VAL_CFM,
                                                 prf_src_task_get(&udss_env->prf_env, conidx),
                                                 TASK_APP, udss_char_val_cfm);
    cfm->status = status;
    cfm->char_code = char_code;

    ke_msg_send(cfm);
}

void app_udss_db_updated_notify(uint8_t conidx, uint32_t db_change_incr_val)
{
    struct udss_env_tag *udss_env = PRF_ENV_GET(UDSS, udss);
    struct udss_db_updated_ntf_req *ntf = KE_MSG_ALLOC(UDSS_DB_UPDATED_NTF_REQ,
                                                       prf_src_task_get(&udss_env->prf_env, conidx),
                                                       TASK_APP, udss_db_updated_ntf_req);
    ntf->db_change_incr_val = db_change_incr_val;

    ke_msg_send(ntf);
}

#endif // (BLE_UDS_SERVER)

/// @} APP
