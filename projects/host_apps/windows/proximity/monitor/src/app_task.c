/**
 ****************************************************************************************
 *
 * @file app_task.c
 *
 * @brief Handling of ble events and responses.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */


#include "app_task.h"
#include "app.h"
#include "queue.h"
#include "console.h"
#include "gapc_task.h"
#include "gapm_task.h"
#include "proxm.h"
#include "proxm_task.h"
#include "disc.h"
#include "disc_task.h"
#include "ble_msg.h"
#include "user_config.h"

extern unsigned int proxm_trans_in_prog;
extern uint8_t last_char[MAX_CONN_NUMBER];

/**
 ****************************************************************************************
 * @brief Extracts device name from adv data if present and copy it to app_env.
 *
 * @param[in] adv_data      Pointer to advertise data.
 * @param[in] adv_data_len  Advertise data length.
 * @param[in] dev_indx      Devices index in device list.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static void app_find_device_name(unsigned char * adv_data, unsigned char adv_data_len, unsigned char dev_indx)
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


int gapm_cmp_evt_handler(ke_msg_id_t msgid,
                         struct gapm_cmp_evt *param,
                         ke_task_id_t dest_id,
                         ke_task_id_t src_id)
{
    if (param->status ==  CO_ERROR_NO_ERROR)
    {
        if(param->operation == GAPM_RESET)
        {
            app_set_mode(); //initialize gap mode
        }
        else if(param->operation == GAPM_SET_DEV_CONFIG)
        {
            app_disc_create();
            Sleep(100);
            app_proxm_create();
            app_env.state = APP_SCAN;
            Sleep(100);

            app_inq();  //start scanning
            ConsoleScan();

        }
    }
    else
    {
        if(!console_main_pass_state)
        {
            if(param->operation == GAPM_SCAN_ACTIVE || param->operation == GAPM_SCAN_PASSIVE)
            {
                app_env.state = APP_IDLE;
                printf ("\rSelect number of device to connect or 'S' to Rescan:\n");
            }
        }
    }
    return (KE_MSG_CONSUMED);
}


int gapc_cmp_evt_handler(ke_msg_id_t msgid,
                         struct gapc_cmp_evt *param,
                         ke_task_id_t dest_id,
                         ke_task_id_t src_id)
{
    int con_id;
    con_id=res_conidx(KE_IDX_GET(src_id));

    switch (param->operation)
    {
    case GAPC_ENCRYPT:
        if (param->status != CO_ERROR_NO_ERROR)
        {
            app_env.proxr_device[con_id].bonded = 0;
            app_disconnect(con_id);
        }
        break;

    default:
        break;
    }

    return 0;
}


int gapm_device_ready_ind_handler(ke_msg_id_t msgid,
                                  void *param,
                                  ke_task_id_t dest_id,
                                  ke_task_id_t src_id)
{
    return 0;
}


int gapm_adv_report_ind_handler(ke_msg_id_t msgid,
                                struct gapm_adv_report_ind *param,
                                ke_task_id_t dest_id,
                                ke_task_id_t src_id)
{
    unsigned char recorded;

    recorded = app_device_recorded(&param->report.adv_addr);

    if (recorded <MAX_SCAN_DEVICES) //update Name
        {
        app_find_device_name(param->report.data,param->report.data_len, recorded);
        ConsoleScan();
        }
    else if (app_env.num_of_devices < MAX_SCAN_DEVICES)
    {
        app_env.devices[app_env.num_of_devices].free = false;
        app_env.devices[app_env.num_of_devices].rssi = param->report.rssi;
        memcpy(app_env.devices[app_env.num_of_devices].adv_addr.addr, param->report.adv_addr.addr, BD_ADDR_LEN );
        app_find_device_name(param->report.data,param->report.data_len, app_env.num_of_devices);
        ConsoleScan();
        app_env.num_of_devices++;
    }

    return 0;
}


unsigned int start_pair;
int gapc_connection_req_ind_handler(ke_msg_id_t msgid,
                                    struct gapc_connection_req_ind *param,
                                    ke_task_id_t dest_id,
                                    ke_task_id_t src_id)
{
    int i;
    int con_id;
    con_id=rtrn_fst_avail();
    app_env.proxr_device[con_id].isactive =true;
    start_pair = 1;

    if ((app_env.state == APP_IDLE) || console_main_pass_state)
    {
        // We are now connected

        app_env.state = APP_CONNECTED;

        // Retrieve the connection index from the GAPC task instance for this connection
        app_env.proxr_device[con_id].device.conidx = KE_IDX_GET(src_id);

        // Retrieve the connection handle from the parameters
        app_env.proxr_device[con_id].device.conhdl = param->conhdl;

        // On Reconnection check if device is bonded and send pairing request. Otherwise it is not bonded.
        if (bdaddr_compare(&app_env.proxr_device[con_id].device.adv_addr, &param->peer_addr))
        {
            if (app_env.proxr_device[con_id].bonded)
                start_pair = 0;
        }

        memcpy(app_env.proxr_device[con_id].device.adv_addr.addr, param->peer_addr.addr, sizeof(struct bd_addr));

        for (i =0 ; i<RSSI_SAMPLES; i++)
            app_env.proxr_device[con_id].rssi[i] = -127;

        app_env.proxr_device[con_id].alert = 0;
        app_env.proxr_device[con_id].rssi_indx = 0;
        app_env.proxr_device[con_id].avg_rssi = -127;
        app_env.proxr_device[con_id].txp = -127;
        app_env.proxr_device[con_id].llv = 0xFF;
        memset(&app_env.proxr_device[con_id].dis, 0, sizeof(dis_env));

        ConsoleConnected(0);

        app_connect_confirm(GAP_AUTH_REQ_NO_MITM_NO_BOND,con_id);

        app_proxm_enable(con_id);
        app_disc_enable(con_id);
    }

    return 0;
}


int gapc_disconnect_ind_handler(ke_msg_id_t msgid,
                                struct gapc_disconnect_ind *param,
                                ke_task_id_t dest_id,
                                ke_task_id_t src_id)
{
    int con_id;
    con_id=res_conidx(KE_IDX_GET(src_id));
    if (param->conhdl == app_env.proxr_device[con_id].device.conhdl)
    {

        if(!console_main_pass_state)
        {
            app_env.state = APP_SCAN;
            Sleep(100);
            if ((param->reason != CO_ERROR_REMOTE_USER_TERM_CON) &&
                (param->reason != CO_ERROR_CON_TERM_BY_LOCAL_HOST))
            {

                app_rst_gap();
            }
        }
        app_env.proxr_device[con_id].isactive =false;
        app_env.proxr_device[con_id].bonded=0;
        app_env.proxr_device[con_id].llv = 0xFF;
        app_env.proxr_device[app_env.cur_dev].txp = -127;
        last_char[con_id]=0xFF;
        app_env.cur_dev=rtrn_prev_avail_conn();
        conn_num_flag=false;
        ConsoleScan();
        if(rtrn_con_avail()==0 && console_main_pass_state)
        {
            console_main_pass_state=false;
            app_env.state = APP_IDLE;
            ConsoleSendScan();
        }
    }

    return 0;
}


int gapc_con_rssi_ind_handler(  ke_msg_id_t msgid,
                                struct gapc_con_rssi_ind *param,
                                ke_task_id_t dest_id,
                                ke_task_id_t src_id)
{
    int count = 0;
    int sum = 0;
    int con_id;

    con_id=res_conidx(KE_IDX_GET(src_id));

    app_env.proxr_device[con_id].rssi[(uint8_t)app_env.proxr_device[con_id].rssi_indx] = param->rssi;
    app_env.proxr_device[con_id].rssi_indx ++;

    if (app_env.proxr_device[con_id].rssi_indx == RSSI_SAMPLES)
        app_env.proxr_device[con_id].rssi_indx = 0;

    while ((count <RSSI_SAMPLES) && (app_env.proxr_device[con_id].rssi[count]) != -127)
    {
        sum += app_env.proxr_device[con_id].rssi[count];
        count ++;
    }
    app_env.proxr_device[con_id].avg_rssi = (char) (sum/count);

    if (((char)app_env.proxr_device[con_id].avg_rssi - (char)app_env.proxr_device[con_id].txp) < -80)
    {
        if (!app_env.proxr_device[con_id].alert)
        {
            app_proxm_write(PROXM_SET_IMMDT_ALERT,PROXM_ALERT_MILD,con_id);
            app_env.proxr_device[con_id].alert = 1;
        }
    }
    else if (app_env.proxr_device[con_id].alert)
    {
        app_proxm_write(PROXM_SET_IMMDT_ALERT,PROXM_ALERT_NONE,con_id);
        app_env.proxr_device[con_id].alert = 0;

    }

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
    int con_id = res_conidx(KE_IDX_GET(src_id));
    switch (param->info)
    {
    case GAPC_PAIRING_SUCCEED:
        if (param->data.auth & GAP_AUTH_BOND)
            app_env.proxr_device[con_id].bonded = 1;
        break;

    case GAPC_IRK_EXCH:
        memcpy (app_env.proxr_device[con_id].irk.irk.key, param->data.irk.irk.key, KEY_LEN);
        memcpy (app_env.proxr_device[con_id].irk.addr.addr.addr, param->data.irk.addr.addr.addr, KEY_LEN);
        app_env.proxr_device[con_id].irk.addr.addr_type = param->data.irk.addr.addr_type;
        break;

    case GAPC_LTK_EXCH:
        app_env.proxr_device[con_id].ediv = param->data.ltk.ediv;
        memcpy (app_env.proxr_device[con_id].rand_nb, param->data.ltk.randnb.nb, RAND_NB_LEN);
        app_env.proxr_device[con_id].ltk.key_size = param->data.ltk.key_size;
        memcpy (app_env.proxr_device[con_id].ltk.ltk.key, param->data.ltk.ltk.key, param->data.ltk.key_size);
        app_env.proxr_device[con_id].ltk.ediv = param->data.ltk.ediv;
        memcpy (app_env.proxr_device[con_id].ltk.randnb.nb, param->data.ltk.randnb.nb, RAND_NB_LEN);
        break;

    case GAPC_PAIRING_FAILED:
        app_env.proxr_device[con_id].bonded = 0;
        app_disconnect(con_id);
        break;
    }

    return 0;
}


int gapc_bond_req_ind_handler(ke_msg_id_t msgid,
                              struct gapc_bond_req_ind *param,
                              ke_task_id_t dest_id,
                              ke_task_id_t src_id)
{
    int con_id=res_conidx(KE_IDX_GET(src_id));
    switch (param->request)
    {
    case GAPC_CSRK_EXCH:
        app_gap_bond_cfm(con_id);
        break;

    default:
        break;
    }

    return 0;
}


int gap_reset_req_cmp_evt_handler(ke_msg_id_t msgid,
                                  struct gap_reset_req_cmp_evt *param,
                                  ke_task_id_t dest_id,
                                  ke_task_id_t src_id)
{
    // We are now in Connectable State
    if (dest_id == TASK_ID_GTL)
    {
        app_env.state = APP_IDLE;
        app_set_mode(); //initialize gap mode

    }

    return 0;
}


int  proxm_enable_rsp_handler(ke_msg_id_t msgid,
                              struct proxm_enable_rsp *param,
                              ke_task_id_t dest_id,
                              ke_task_id_t src_id)
{
    int con_id;
    con_id=res_conidx(KE_IDX_GET(src_id));
    if (param->status == CO_ERROR_NO_ERROR)
    {
        //initialize proximity reporter's values to non valid
        app_env.proxr_device[con_id].llv = 0xFF;
        app_env.proxr_device[con_id].txp = 0xFF;


        ConsoleConnected(1);

        if (start_pair)
        {
            app_env.proxr_device[con_id].bonded = 0;
            app_security_enable(con_id);
        }
        else
            app_start_encryption(con_id);
    }

    app_proxm_read_txp(con_id);

    return 0;
}


int  proxm_rd_char_rsp_handler(ke_msg_id_t msgid,
                               struct proxm_rd_rsp *param,
                               ke_task_id_t dest_id,
                               ke_task_id_t src_id)
{
    int con_id;
    con_id=res_conidx(KE_IDX_GET(src_id));
    if (param->status == CO_ERROR_NO_ERROR)
    {
        if (last_char[con_id] == PROXM_RD_LL_ALERT_LVL)
        {
            app_env.proxr_device[con_id].llv = param->value;
        }
        else if (last_char[con_id] == PROXM_RD_TX_POWER_LVL)
        {
            app_env.proxr_device[con_id].txp = param->value;
        }
        last_char[con_id] = 0xFF;
        proxm_trans_in_prog = 0;

        ConsoleConnected(1);
    }

    return 0;
}


int  proxm_wr_alert_lvl_rsp_handler(ke_msg_id_t msgid,
                                    struct proxm_wr_alert_lvl_rsp *param,
                                    ke_task_id_t dest_id,
                                    ke_task_id_t src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        proxm_trans_in_prog = 0;
        ConsoleConnected(1);
    }

    return 0;
}


int  disc_enable_rsp_handler(ke_msg_id_t msgid,
                             struct disc_enable_rsp *param,
                             ke_task_id_t dest_id,
                             ke_task_id_t src_id)
{
    uint8_t i;
    int con_id = res_conidx(KE_IDX_GET(src_id));
    if (param->status == CO_ERROR_NO_ERROR)
    {

        for ( i = 0; i < DISC_CHAR_MAX; i++)
        {
            app_env.proxr_device[con_id].dis.chars[i].val_hdl = param->dis.chars[i].val_hdl;
        }

        app_disc_rd_char(DISC_MANUFACTURER_NAME_CHAR, con_id);

    }

    return 0;
}


int  disc_rd_char_rsp_handler(ke_msg_id_t msgid,
                              struct disc_rd_char_rsp *param,
                              ke_task_id_t dest_id,
                              ke_task_id_t src_id)
{
    uint8_t i;
    uint8_t char_code = DISC_CHAR_MAX;
    static uint8_t char_code_idx=0;
    uint8_t con_id = res_conidx(KE_IDX_GET(src_id));


    if (param->info.status == CO_ERROR_NO_ERROR)
    {

        for ( i = 0; i < DISC_CHAR_MAX; i++)
        {
            if (app_env.proxr_device[con_id].dis.chars[i].val_hdl == param->info.handle)
            {
                char_code = i;
                break;
            }
        }

        if (char_code < DISC_CHAR_MAX)
        {
            memcpy (app_env.proxr_device[con_id].dis.chars[char_code].val, param->info.value, param->info.length);
            app_env.proxr_device[con_id].dis.chars[char_code].val[param->info.length] = '\0';
            app_env.proxr_device[con_id].dis.chars[char_code].len = param->info.length;
        }
    }

    // read next DISS characteristic value
    read_disc_chars(++char_code_idx, con_id);
    if(char_code_idx == DISC_CHAR_MAX)
    {
        char_code_idx = 0;
    }
    return 0;
}


void read_disc_chars(uint8_t id, uint8_t con_id)
{

    switch(id)
    {
    case DISC_MODEL_NB_STR_CHAR:
        app_disc_rd_char(DISC_MODEL_NB_STR_CHAR, con_id);
        break;
    case DISC_SERIAL_NB_STR_CHAR:
        app_disc_rd_char(DISC_SERIAL_NB_STR_CHAR, con_id);
        break;
    case DISC_HARD_REV_STR_CHAR:
        app_disc_rd_char(DISC_HARD_REV_STR_CHAR, con_id);
        break;
    case DISC_FIRM_REV_STR_CHAR:
        app_disc_rd_char(DISC_FIRM_REV_STR_CHAR, con_id);
        break;
    case DISC_SW_REV_STR_CHAR:
        app_disc_rd_char(DISC_SW_REV_STR_CHAR, con_id);
        break;
    case DISC_SYSTEM_ID_CHAR:
        app_disc_rd_char(DISC_SYSTEM_ID_CHAR, con_id);
        break;
    case DISC_IEEE_CHAR:
        app_disc_rd_char(DISC_IEEE_CHAR, con_id);
        break;
    case DISC_PNP_ID_CHAR:
        app_disc_rd_char(DISC_PNP_ID_CHAR, con_id);
        break;
    default:
        break;

    }
    return;
}
