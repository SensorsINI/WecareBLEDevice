/**
 ****************************************************************************************
 *
 * @file app_task.c
 *
 * @brief Handling of ble events and responses.
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_task.h" 
#include "app.h" 
#include "queue.h" 
#include "console.h" 
#include "proxr_task.h"
#include "diss_task.h"
#include "proxr.h"
#include "ble_msg.h"
#include "user_config.h"


// application alert state structure
app_alert_state alert_state;

void app_find_device_name(unsigned char * adv_data, unsigned char adv_data_len, unsigned char dev_indx)
{
    unsigned char indx = 0;

    while (indx < adv_data_len)
    {
        if (adv_data[indx+1] == 0x09)
        {
            memcpy(app_env.devices[dev_indx].data, &adv_data[indx+2], (size_t) adv_data[indx]);
            app_env.devices[dev_indx].data_len = (unsigned char ) adv_data[indx];
        }
        indx += (unsigned char ) adv_data[indx]+1;
    }

}

int gapm_set_dev_config_completion_handler(ke_msg_id_t msgid,
                                           struct gapm_cmp_evt *param,
                                           ke_task_id_t dest_id,
                                           ke_task_id_t src_id)
{
    app_env.state = APP_CONNECTABLE;

    app_diss_create_db();

    return (KE_MSG_CONSUMED);
}

int gapm_device_ready_ind_handler(ke_msg_id_t msgid,
                                  struct gap_ready_evt *param,
                                  ke_task_id_t dest_id,
                                  ke_task_id_t src_id)
{
    // We are now in Connectable State
    if (dest_id == TASK_ID_GTL)
    {
        app_rst_gap();
    }
    
    return 0;
}

int gapm_adv_report_ind_handler(ke_msg_id_t msgid,
                                struct gapm_adv_report_ind *param,
                                ke_task_id_t dest_id,
                                ke_task_id_t src_id)
{
    if (app_env.state != APP_SCAN)
        return -1;

    return 0;
}

unsigned int start_pair;
int gapc_connection_req_ind_handler(ke_msg_id_t msgid,
                                    struct gapc_connection_req_ind *param,
                                    ke_task_id_t dest_id,
                                    ke_task_id_t src_id)
{
    if (app_env.state == APP_IDLE || app_env.state == APP_CONNECTABLE)
    {
        // We are now connected
        app_env.state = APP_CONNECTED;
        // Retrieve the connection index from the GAPC task instance for the connection
        app_env.proxr_device.device.conidx = KE_IDX_GET(src_id);
        // Retrieve the connection handle from the parameters
        app_env.proxr_device.device.conhdl = param->conhdl;
        memcpy(app_env.proxr_device.device.adv_addr.addr, param->peer_addr.addr, sizeof(struct bd_addr));
        app_env.proxr_device.rssi = 0xFF;
        app_env.proxr_device.txp = 0xFF;
        app_env.proxr_device.llv = 0xFF;                        

        alert_state.txp_lvl = 0x00;

        app_connect_confirm(GAP_AUTH_REQ_NO_MITM_NO_BOND, 1);


        ConsoleConnected(0);
    }

    return 0;
}

int gapc_disconnect_ind_handler(ke_msg_id_t msgid,
                                struct gapc_disconnect_ind *param,
                                ke_task_id_t dest_id,
                                ke_task_id_t src_id)
{
    if (param->conhdl == app_env.proxr_device.device.conhdl)
    {
        app_send_disconnect(TASK_ID_PROXR, param->conhdl, param->reason);
        app_env.state = APP_IDLE;
        printf("Device Disconnected\n");
        app_set_mode();
    }

    return 0;
}

int gapc_con_rssi_ind_handler(ke_msg_id_t msgid,
                              struct gapc_con_rssi_ind *param,
                              ke_task_id_t dest_id,
                              ke_task_id_t src_id)
{
    app_env.proxr_device.rssi = param->rssi;
    ConsoleConnected(1);

    return 0;
}

int gapc_get_dev_info_req_ind_handler(ke_msg_id_t msgid,
                                      struct gapc_get_dev_info_req_ind *param,
                                      ke_task_id_t dest_id,
                                      ke_task_id_t src_id)
{
    switch (param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm *cfm = BleMsgDynAlloc(GAPC_GET_DEV_INFO_CFM,
                                                               src_id,
                                                               dest_id,
                                                               sizeof(struct gapc_get_dev_info_cfm),
                                                               device_info.dev_name.length);

            cfm->req = GAPC_DEV_NAME;
            cfm->info.name.length = device_info.dev_name.length;
            memcpy(cfm->info.name.value, device_info.dev_name.name, device_info.dev_name.length);
            BleSendMsg(cfm);
        }
        break;

        case GAPC_DEV_APPEARANCE:
        {
            struct gapc_get_dev_info_cfm *cfm = BleMsgAlloc(GAPC_GET_DEV_INFO_CFM,
                                                            src_id,
                                                            dest_id,
                                                            sizeof(struct gapc_get_dev_info_cfm));

            cfm->req = GAPC_DEV_APPEARANCE;
            cfm->info.appearance = device_info.appearance;

            BleSendMsg(cfm);

        }
        break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            struct gapc_get_dev_info_cfm *cfm = BleMsgAlloc(GAPC_GET_DEV_INFO_CFM,
                                                            src_id,
                                                            dest_id,
                                                            sizeof(struct gapc_get_dev_info_cfm));

            cfm->req = GAPC_DEV_SLV_PREF_PARAMS;
            cfm->info.slv_params.con_intv_min = MS_TO_DOUBLESLOTS(10);
            cfm->info.slv_params.con_intv_max = MS_TO_DOUBLESLOTS(20);
            cfm->info.slv_params.slave_latency = 0;
            cfm->info.slv_params.conn_timeout = MS_TO_TIMERUNITS(1250);

            BleSendMsg(cfm);
        }
        break;

        case GAPC_DEV_CENTRAL_RPA:
        {

        }
        break;

        case GAPC_DEV_RPA_ONLY:
        {

        }
        break;

        default: break;
     }

   return 0;
}

int gapc_set_dev_info_req_ind_handler(ke_msg_id_t msgid,
                                      struct gapc_set_dev_info_req_ind *param,
                                      ke_task_id_t dest_id,
                                      ke_task_id_t src_id)
{

    uint8_t status = GAP_ERR_REJECTED;
    switch (param->req)
    {
        case GAPC_DEV_NAME:
        {
            device_info.dev_name.length = param->info.name.length;
            memcpy(device_info.dev_name.name, param->info.name.value, device_info.dev_name.length);
            status = GAP_ERR_NO_ERROR;
        }
        break;
        case GAPC_DEV_APPEARANCE:
        {
            device_info.appearance = param->info.appearance;
            status = GAP_ERR_NO_ERROR;
        }
        break;
        default:
        {
            status = GAP_ERR_REJECTED;
        }
        break;
    }


    struct gapc_set_dev_info_cfm *cfm = BleMsgAlloc(GAPC_SET_DEV_INFO_CFM,
                                                    src_id,
                                                    dest_id,
                                                    sizeof(struct gapc_set_dev_info_cfm));
    cfm->status = status;
    cfm->req = param->req;

    BleSendMsg(cfm);

    return 0;
}

int gapc_param_update_req_ind_handler(ke_msg_id_t msgid,
                                      struct gapc_param_update_req_ind *param,
                                      ke_task_id_t dest_id,
                                      ke_task_id_t src_id)
{

    struct gapc_param_update_cfm* cfm = BleMsgAlloc(GAPC_PARAM_UPDATE_CFM,
                                                    src_id,
                                                    dest_id,
                                                    sizeof(struct gapc_param_update_cfm));

    cfm->accept = true;
    cfm->ce_len_min = 0;
    cfm->ce_len_max = 0;
    BleSendMsg(cfm);
    return 0;
}

int gapc_bond_ind_handler(ke_msg_id_t msgid,
                          struct gapc_bond_ind *param,
                          ke_task_id_t dest_id,
                          ke_task_id_t src_id)
{
    switch (param->info)
    {
        case GAPC_PAIRING_SUCCEED:
            if (param->data.auth | GAP_AUTH_BOND)
            {
                app_env.proxr_device.bonded = 1;
            }
            ConsoleConnected(0);
            break;

        case GAPC_IRK_EXCH:
            memcpy (app_env.proxr_device.irk.irk.key, param->data.irk.irk.key, KEY_LEN);
            memcpy (app_env.proxr_device.irk.addr.addr.addr, param->data.irk.addr.addr.addr, BD_ADDR_LEN);
            app_env.proxr_device.irk.addr.addr_type = param->data.irk.addr.addr_type;
            
            break;

        case GAPC_LTK_EXCH:
            app_env.proxr_device.ltk.ediv = param->data.ltk.ediv;
            app_env.proxr_device.ltk.key_size = param->data.ltk.key_size;
            memcpy (app_env.proxr_device.ltk.ltk.key, param->data.ltk.ltk.key, param->data.ltk.key_size);
            memcpy (app_env.proxr_device.ltk.randnb.nb, param->data.ltk.randnb.nb, RAND_NB_LEN);
            break;

        case GAPC_PAIRING_FAILED:
            app_env.proxr_device.bonded = 0;
            app_disconnect();
            break;
    }

    return 0;
}
 
int gapm_reset_completion_handler(ke_msg_id_t msgid,
                                  struct gapm_cmp_evt *param,
                                  ke_task_id_t dest_id,
                                  ke_task_id_t src_id)
{
    // We are now in Connectable State
    if (dest_id == TASK_ID_GTL)
    {
        app_env.state = APP_IDLE;           
        alert_state.ll_alert_lvl = 2; // Link Loss default Alert Level is high
        alert_state.adv_toggle = 0; // clear advertise toggle
        app_set_mode(); // initialize gap mode
    }
    
    return 0;
}

int gapm_profile_added_ind_handler(ke_msg_id_t msgid,
                                  struct gapm_profile_added_ind *param,
                                  ke_task_id_t dest_id,
                                  ke_task_id_t src_id)
{
    if (dest_id != TASK_ID_GTL)
    {
            printf("Error creating DB. Destination ID is not TASK_GTL \n");
            return (KE_MSG_CONSUMED);
    }
    else
    {
        switch (param->prf_task_id)
        {
            case TASK_ID_DISS:
                if (app_env.state == APP_CONNECTABLE)
                {
                    app_proxr_create_db();
                }
                break;
            case TASK_ID_PROXR:
                if (app_env.state == APP_CONNECTABLE)
                {
                    app_adv_start(); // start advertising
                }
                break;
            default:
                break;
        }
    }

    return (KE_MSG_CONSUMED);
}

int gapc_bond_req_ind_handler(ke_msg_id_t msgid,
                              struct gapc_bond_req_ind *param,
                              ke_task_id_t dest_id,
                              ke_task_id_t src_id)
{
    app_gap_bond_cfm(param);
    
    return 0;
}

int gapc_encrypt_req_ind_handler(ke_msg_id_t const msgid,
                                 struct gapc_encrypt_req_ind * param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    struct gapc_encrypt_cfm* cfm = BleMsgAlloc(GAPC_ENCRYPT_CFM, src_id, dest_id, sizeof (struct gapc_encrypt_cfm));

    if(((app_env.proxr_device.bonded)
        && (memcmp(&(app_env.proxr_device.ltk.randnb), &(param->rand_nb), RAND_NB_LEN) == 0)
        && (app_env.proxr_device.ltk.ediv == param->ediv)))
    {
        cfm->found = true;
        cfm->key_size = app_env.proxr_device.ltk.key_size;
        memcpy(&(cfm->ltk), &(app_env.proxr_device.ltk.ltk), app_env.proxr_device.ltk.key_size);
        // update connection auth
        app_connect_confirm(GAP_AUTH_REQ_NO_MITM_BOND, 1);
    }
    else
    {
        cfm->found = false;
    }

    BleSendMsg(cfm);

    return (KE_MSG_CONSUMED);
}

int gapc_encrypt_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_encrypt_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    printf("Received GAPC_ENCRYPT_IND auth = %d\n", param->auth);

    return (KE_MSG_CONSUMED);
}

void diss_value_req_ind_handler(ke_msg_id_t const msgid,
                                struct diss_value_req_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    // Initialize length
    uint8_t len = 0;
    // Pointer to the data
    uint8_t *data = NULL;

    // Check requested value
    switch (param->value)
    {
        case DIS_MANUFACTURER_NAME_CHAR:
        {
            // Set information
            len = APP_DIS_MANUFACTURER_NAME_LEN;
            data = (uint8_t *)APP_DIS_MANUFACTURER_NAME;
        } break;

        case DIS_MODEL_NB_STR_CHAR:
        {
            // Set information
            len = APP_DIS_MODEL_NB_STR_LEN;
            data = (uint8_t *)APP_DIS_MODEL_NB_STR;
        } break;

        case DIS_SYSTEM_ID_CHAR:
        {
            // Set information
            len = APP_DIS_SYSTEM_ID_LEN;
            data = (uint8_t *)APP_DIS_SYSTEM_ID;
        } break;

        case DIS_PNP_ID_CHAR:
        {
            // Set information
            len = APP_DIS_PNP_ID_LEN;
            data = (uint8_t *)APP_DIS_PNP_ID;
        } break;

        case DIS_SERIAL_NB_STR_CHAR:
        {
            // Set information
            len = APP_DIS_SERIAL_NB_STR_LEN;
            data = (uint8_t *)APP_DIS_SERIAL_NB_STR;
        } break;

        case DIS_HARD_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_HARD_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_HARD_REV_STR;
        } break;

        case DIS_FIRM_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_FIRM_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_FIRM_REV_STR;
        } break;

        case DIS_SW_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_SW_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_SW_REV_STR;
        } break;

        case DIS_IEEE_CHAR:
        {
            // Set information
            len = APP_DIS_IEEE_LEN;
            data = (uint8_t *)APP_DIS_IEEE;
        } break;

        default:

            break;
    }

    // Allocate confirmation to send the value
    struct diss_value_cfm *cfm_value = BleMsgDynAlloc(DISS_VALUE_CFM,
                                                      src_id,
                                                      dest_id,
                                                      sizeof (struct diss_value_cfm),
                                                      (sizeof (uint8_t) * len));


    // Set parameters
    cfm_value->value = param->value;
    cfm_value->length = len;
    if (len)
    {
        // Copy data
        memcpy(&cfm_value->data[0], data, len);
    }

    // Send the message
    BleSendMsg((void *) cfm_value);
}

static void update_visual_alert_indication(uint8_t alert_level)
{
    if (alert_level == PROXR_ALERT_NONE)
        printf("ALERT STOPPED.\n");
    else
        printf("ALERT STARTED. Level:%d\n", alert_level);
}

void proxr_alert_ind_handler(ke_msg_id_t msgid,
                             struct proxr_alert_ind  *param,
                             ke_task_id_t dest_id,
                             ke_task_id_t src_id)
{
    if (param->char_code == PROXR_IAS_CHAR)
    {
        printf("IAS Alert Level updated by peer. New value = %d\n", param->alert_lvl);
        update_visual_alert_indication(param->alert_lvl);
    }
    else if (param->char_code == PROXR_LLS_CHAR)
    {
        printf("LLS Alert Level updated by peer. New value = %d\n", param->alert_lvl);
        update_visual_alert_indication(param->alert_lvl);
    }

}

