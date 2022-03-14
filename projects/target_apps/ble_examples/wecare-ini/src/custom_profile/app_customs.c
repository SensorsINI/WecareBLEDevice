/**
 ****************************************************************************************
 *
 * @file app_customs.c
 *
 * @brief Custom profiles application file.
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
#include "app_customs.h"
#include "custs1_task.h"
#include "custs2_task.h"
#include "attm_db.h"
#include "attm_db_128.h"
#include "gapc.h"
#include "prf_types.h"
#include "app_prf_types.h"
#include "app_prf_perm_types.h"
#include "user_custs_config.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

const struct cust_prf_func_callbacks *custs_get_func_callbacks(enum KE_API_ID task_id)
{
    int i = 0;

    // max number of service characteristics
    while (cust_prf_funcs[i].task_id != TASK_ID_INVALID)
    {
        if(cust_prf_funcs[i].task_id == task_id)
            return &cust_prf_funcs[i];
        else
            i++;
    }
    return NULL;
}

#if (BLE_CUSTOM1_SERVER)
void app_custs1_init(void)
{
    // Nothing to do
}

void app_custs1_create_db(void)
{
    struct custs1_db_cfg *db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM,
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd,
                                                             sizeof(struct custs1_db_cfg));

    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = get_user_prf_srv_perm(TASK_ID_CUSTS1);
    req->prf_task_id = TASK_ID_CUSTS1;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct custs1_db_cfg *) req->param;
    // Attribute table. In case the handle offset needs to be saved
    db_cfg->att_tbl = NULL;
    db_cfg->cfg_flag = 0;
    db_cfg->features = 0;

    // Send the message
    ke_msg_send(req);
}
#endif // (BLE_CUSTOM1_SERVER)

#if (BLE_CUSTOM2_SERVER)
void app_custs2_init(void)
{
    // Nothing to do
}

void app_custs2_create_db(void)
{
    struct custs2_db_cfg *db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM,
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd,
                                                             sizeof(struct custs2_db_cfg));

    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = get_user_prf_srv_perm(TASK_ID_CUSTS2);
    req->prf_task_id = TASK_ID_CUSTS2;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct custs2_db_cfg *) req->param;
    // Attribute table. In case the handle offset needs to be saved
    db_cfg->att_tbl = NULL;
    db_cfg->cfg_flag = 0;
    db_cfg->features = 0;

    // Send the message
    ke_msg_send(req);
}

#endif // (BLE_CUSTOM2_SERVER)
#endif // (BLE_CUSTOM_SERVER)
