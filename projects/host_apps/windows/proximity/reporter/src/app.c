/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Proximity Reporter Host demo application.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <windows.h>
#include <conio.h>
#include <stdint.h>
#include <assert.h>
#include "console.h"
#include "queue.h"
#include "ble_msg.h"
#include "uart.h"
#include "gap.h"
#include "app.h"
#include "app_task.h"
#include "proxr.h"
#include "proxr_task.h"
#include "diss_task.h"
#include "atts.h"
#include "gapc_task.h"
#include "gapm_task.h"
#include "user_config.h"

struct app_env_tag app_env;
struct app_device_info device_info;
unsigned short Comport = 0xffff;
int poll_count;
unsigned int proxm_trans_in_prog = true;

void app_exit(void)
{
    // Stop the threads
    StopConsoleTask = TRUE;
    StopRxTask = TRUE;

    Sleep(100);
}

void app_set_mode(void)
{
    struct gapm_set_dev_config_cmd *msg = BleMsgAlloc(GAPM_SET_DEV_CONFIG_CMD,
                                                      TASK_ID_GAPM,
                                                      TASK_ID_GTL,
                                                      sizeof(struct gapm_set_dev_config_cmd ));

    msg->operation = GAPM_SET_DEV_CONFIG;
    // Device Role
    msg->role = GAP_ROLE_PERIPHERAL;
    // Maximal MTU
    msg->max_mtu = 23;
    // Device Address Type
    msg->addr_type = GAPM_CFG_ADDR_PUBLIC;
    // Duration before regenerate device address when privacy is enabled
    msg->renew_dur = 0 ;
    memset( msg->irk.key, 0, sizeof(struct gap_sec_key));
    //ATT database configuration
    msg->att_cfg= GAPM_MASK_ATT_SVC_CHG_EN;
    // GAP service start handle
    msg->gap_start_hdl = 0;
    // GATT service start handle
    msg->gatt_start_hdl = 0;
    // Maximal MPS
    msg->max_mps = 0;
    // Maximal Tx octets
    msg->max_txoctets = 0;
    // Maximal Tx time
    msg->max_txtime = 0;

    BleSendMsg((void *)msg);

    return;
}

void app_rst_gap(void)
{
    struct gapm_reset_cmd *msg = BleMsgAlloc(GAPM_RESET_CMD,
                                             TASK_ID_GAPM,
                                             TASK_ID_GTL,
                                             sizeof(struct gapm_reset_cmd));
    int i;

    msg->operation = GAPM_RESET;
    app_env.state = APP_IDLE;
    //init scan devices list
    app_env.num_of_devices = 0;
    // Initialize device info
    device_info.dev_name.length = (USER_DEVICE_NAME_LEN <= GAP_MAX_NAME_SIZE) ? USER_DEVICE_NAME_LEN : GAP_MAX_NAME_SIZE;
    memcpy(device_info.dev_name.name, USER_DEVICE_NAME, device_info.dev_name.length);
    device_info.appearance = 0x0000;

    for (i=0; i < MAX_SCAN_DEVICES; i++)
    {
        app_env.devices[i].free = true;
        app_env.devices[i].adv_addr.addr[0] = '\0';
        app_env.devices[i].data[0] = '\0';
        app_env.devices[i].data_len = 0;
        app_env.devices[i].rssi = 0;
    }

    BleSendMsg((void *)msg);

    return;
}

void app_inq(void)
{
    struct gapm_start_scan_cmd *msg = BleMsgAlloc(GAPM_START_SCAN_CMD,
                                                  TASK_ID_GAPM,
                                                  TASK_ID_GTL,
                                                  sizeof(struct gapm_start_scan_cmd));
    int i;

    //init scan devices list
    app_env.num_of_devices = 0;

    for (i=0; i < MAX_SCAN_DEVICES; i++)
    {
        app_env.devices[i].free = true;
        app_env.devices[i].adv_addr.addr[0] = '\0';
        app_env.devices[i].data[0] = '\0';
        app_env.devices[i].data_len = 0;
        app_env.devices[i].rssi = 0;
    }

    msg->mode = GAP_GEN_DISCOVERY;
    msg->op.code = GAPM_SCAN_ACTIVE;
    msg->interval = 10;
    msg->window = 5;

    BleSendMsg((void *)msg);

    return;
}

void app_adv_start(void)
{
    uint8_t device_name_length;
    uint8_t device_name_avail_space;
    uint8_t device_name_temp_buf[64];

    // Allocate a message for GAP
    struct gapm_start_advertise_cmd *cmd = BleMsgAlloc(GAPM_START_ADVERTISE_CMD,
                                                       TASK_ID_GAPM,
                                                       TASK_ID_GTL,
                                                       sizeof (struct gapm_start_advertise_cmd));

    cmd->op.code     = GAPM_ADV_UNDIRECT;
    cmd->op.addr_src = GAPM_STATIC_ADDR;
    cmd->intv_min    = APP_ADV_INT_MIN;
    cmd->intv_max    = APP_ADV_INT_MAX;
    cmd->channel_map = ADV_ALL_CHNLS_EN;

    cmd->info.host.mode = GAP_GEN_DISCOVERABLE;
    cmd->info.host.adv_filt_policy = ADV_ALLOW_SCAN_ANY_CON_ANY;

    /*-----------------------------------------------------------------------------------
     * Set the Advertising Data and the Scan Response Data
     *---------------------------------------------------------------------------------*/
    cmd->info.host.adv_data_len       = APP_ADV_DATA_MAX_SIZE;
    cmd->info.host.scan_rsp_data_len  = APP_SCAN_RESP_DATA_MAX_SIZE;

    // Advertising Data
    #if (NVDS_SUPPORT)
    if(nvds_get(NVDS_TAG_APP_BLE_ADV_DATA, &cmd->info.host.adv_data_len,
                &cmd->info.host.adv_data[0]) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        cmd->info.host.adv_data_len = APP_DFLT_ADV_DATA_LEN;
        memcpy(&cmd->info.host.adv_data[0], APP_DFLT_ADV_DATA, cmd->info.host.adv_data_len);
    }

    // Scan Response Data
    #if (NVDS_SUPPORT)
    if(nvds_get(NVDS_TAG_APP_BLE_SCAN_RESP_DATA, &cmd->info.host.scan_rsp_data_len,
                &cmd->info.host.scan_rsp_data[0]) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        cmd->info.host.scan_rsp_data_len = APP_SCNRSP_DATA_LENGTH;
        memcpy(&cmd->info.host.scan_rsp_data[0], APP_SCNRSP_DATA, cmd->info.host.scan_rsp_data_len);
    }

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    device_name_avail_space = APP_ADV_DATA_MAX_SIZE - cmd->info.host.adv_data_len - 2;

    // Check if data can be added to the Advertising data
    if (device_name_avail_space > 0)
    {
        // Get the Device Name to add in the Advertising Data (Default one or NVDS one)
        #if (NVDS_SUPPORT)
        device_name_length = NVDS_LEN_DEVICE_NAME;
        if (nvds_get(NVDS_TAG_DEVICE_NAME, &device_name_length, &device_name_temp_buf[0]) != NVDS_OK)
        #endif //(NVDS_SUPPORT)
        {
            // Get default Device Name (No name if not enough space)
            device_name_length = strlen(USER_DEVICE_NAME);
            memcpy(&device_name_temp_buf[0], USER_DEVICE_NAME, device_name_length);
        }

        if(device_name_length > 0)
        {
            // Check available space
            device_name_length = min(device_name_length, device_name_avail_space);

            // Fill Length
            cmd->info.host.adv_data[cmd->info.host.adv_data_len]     = device_name_length + 1;
            // Fill Device Name Flag
            cmd->info.host.adv_data[cmd->info.host.adv_data_len + 1] = '\x09';
            // Copy device name
            memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len + 2], device_name_temp_buf, device_name_length);

            // Update Advertising Data Length
            cmd->info.host.adv_data_len += (device_name_length + 2);
        }
    }

    // Send the message
    printf("Advertising...\n");
    BleSendMsg(cmd);

    return;
}

void app_connect(unsigned char indx)
{
    struct gapm_start_connection_cmd *msg;

    if (app_env.devices[indx].free == true)
    {
        return;
    }

    msg = (struct gapm_start_connection_cmd *) BleMsgAlloc(GAPM_START_CONNECTION_CMD ,
                                                           TASK_ID_GAPM,
                                                           TASK_ID_GTL,
                                                           sizeof(struct gapm_start_connection_cmd ));

    msg->nb_peers = 1;
    memcpy((void *) &msg->peers[0].addr, (void *)&app_env.devices[indx].adv_addr.addr, BD_ADDR_LEN);
    msg->con_intv_min = 100;
    msg->con_intv_max = 100;
    msg->ce_len_min = 0x0;
    msg->ce_len_max = 0x5;
    msg->con_latency = 0;
    msg->superv_to = 0x7D0;
    msg->scan_interval = 0x180;
    msg->scan_window = 0x160;
    msg->op.code = GAPM_CONNECTION_DIRECT;

    BleSendMsg((void *) msg);
}

void app_send_disconnect(uint16_t dst, uint16_t conhdl, uint8_t reason)
{
    struct gapc_disconnect_ind * disconnect_ind = BleMsgAlloc(GAPC_DISCONNECT_IND,
                                                              dst,
                                                              TASK_ID_GTL,
                                                              sizeof(struct gapc_disconnect_ind));

    // fill parameters
    disconnect_ind->conhdl   = conhdl;
    disconnect_ind->reason   = reason;

    // send indication
    BleSendMsg(disconnect_ind);
}

void app_read_rssi(void)
{
    struct gapc_get_info_cmd * req = BleMsgAlloc(GAPC_GET_INFO_CMD,
                                                 KE_BUILD_ID(TASK_ID_GAPC, app_env.proxr_device.device.conidx),
                                                 TASK_ID_GTL,
                                                 sizeof(struct gapc_get_info_cmd));

    req->operation = GAPC_GET_CON_RSSI;

    BleSendMsg((void *) req);
}

void app_disconnect(void)
{
    struct gapc_disconnect_cmd * req = BleMsgAlloc(GAPC_DISCONNECT_CMD,
                                                   KE_BUILD_ID(TASK_ID_GAPC, app_env.proxr_device.device.conidx),
                                                   TASK_ID_GTL,
                                                   sizeof(struct gapc_disconnect_cmd ));

    req->operation = GAPC_DISCONNECT;
    req->reason = CO_ERROR_REMOTE_USER_TERM_CON;

    BleSendMsg((void *) req);
}

void app_security_enable(void)
{
    // Allocate the message
   struct gapc_bond_cmd * req = BleMsgAlloc(GAPC_BOND_CMD,
                                            KE_BUILD_ID(TASK_ID_GAPC, app_env.proxr_device.device.conidx),
                                            TASK_ID_GTL,
                                            sizeof(struct gapc_bond_cmd));


    req->operation = GAPC_BOND;
    //GAP_SEC1_NOAUTH_PAIR_ENC;
    req->pairing.sec_req = GAP_NO_SEC;

    // OOB information
    req->pairing.oob = GAP_OOB_AUTH_DATA_NOT_PRESENT;

    // IO capabilities
    req->pairing.iocap          = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;

    // Authentication requirements
    req->pairing.auth           = GAP_AUTH_REQ_NO_MITM_BOND; //SMP_AUTH_REQ_NO_MITM_NO_BOND;

    // Encryption key size
    req->pairing.key_size       = 16;

    //Initiator key distribution
    req->pairing.ikey_dist      = 0x04; //SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY | SMP_KDIST_SIGNKEY;

    //Responder key distribution
    req->pairing.rkey_dist      = 0x03; //SMP_KDIST_ENCKEY | SMP_KDIST_IDKEY | SMP_KDIST_SIGNKEY;

    // Send the message
    BleSendMsg(req);

}

void app_connect_confirm(enum gap_auth auth, uint8_t svc_changed_ind_enable)
{
    // confirm connection
    struct gapc_connection_cfm *cfm = BleMsgAlloc(GAPC_CONNECTION_CFM,
                                                  KE_BUILD_ID(TASK_ID_GAPC, app_env.proxr_device.device.conidx),
                                                  TASK_ID_GTL,
                                                  sizeof (struct gapc_connection_cfm));

    cfm->auth = auth;
    cfm->svc_changed_ind_enable = svc_changed_ind_enable;

    // Send the message
    BleSendMsg(cfm);
}

/**
 ****************************************************************************************
 * @brief Fills an array with random bytes (starting from the end of the array)
 *          Assuming an array of array_size and a dynamic key size, so that key_size = M*4+N:
 *          Calls rand() M times and appends the 4 bytes of each 32 bit return value to the output array
 *          Calls rand() once and appends the N most significant bytes of the 32 bit return value to the output array
 *          Fills the rest bytes of the array with zeroes
 *
 * @param[in] *dst              The address of the array
 * @param[in] key_size          Number of bytes that shall be randomized
 * @param[in] array_size        Total size of the array
 ****************************************************************************************
 */
static void fill_random_byte_array(uint8_t *dst, uint8_t key_size, uint8_t array_size)
{
    uint32_t rand_val;
    uint8_t rand_bytes_cnt = 0;
    uint8_t remaining_bytes;
    uint8_t i;

    // key_size must not be greater than array_size
    assert(array_size>=key_size);

    // key_size = M*4+N
    // Randomize the M most significant bytes of the array
    for (i = 0; i < key_size-3; i+=4)
    {
        // Calculate a random 32 bit value
        rand_val = rand();
        // Assign each of the 4 rand bytes to the byte array
        dst[array_size - (i+0)-1] = (rand_val >> 24) & 0xFF;
        dst[array_size - (i+1)-1] = (rand_val >> 16) & 0xFF;
        dst[array_size - (i+2)-1] = (rand_val >> 8) & 0xFF;
        dst[array_size - (i+3)-1] = (rand_val >> 0) & 0xFF;
        // Count randomized bytes
        rand_bytes_cnt += 4;
    }

    // Randomize the remaining N most significant bytes of the array. (Max N = 3)
    remaining_bytes = key_size - rand_bytes_cnt;
    if (remaining_bytes)
    {
        rand_val = rand();
        for (i = 0; i < remaining_bytes; i++)
        {
            dst[array_size -(rand_bytes_cnt+i)-1] = (rand_val >> ((3-i)<<3)) & 0xFF;
        }
    }

    // Fill the least significant bytes of the array with zeroes
    remaining_bytes = array_size - key_size;
    for (i = 0; i < remaining_bytes; i++)
    {
        dst[array_size - (key_size + i)-1] = 0;
    }
}

void app_gen_csrk(void)
{
    // Randomly generate the CSRK
    fill_random_byte_array(app_env.proxr_device.csrk.key, KEY_LEN, KEY_LEN);
}

void app_sec_gen_ltk(uint8_t key_size)
{
    app_env.proxr_device.ltk.key_size = key_size;

    // Randomly generate the LTK and the Random Number
    fill_random_byte_array(app_env.proxr_device.ltk.randnb.nb, RAND_NB_LEN, RAND_NB_LEN);

    // Randomly generate the end of the LTK
    fill_random_byte_array(app_env.proxr_device.ltk.ltk.key, key_size, KEY_LEN);

    // Randomly generate the EDIV
    app_env.proxr_device.ltk.ediv = rand()%65536;
}

uint32_t app_gen_tk(void)
{
    // Generate a PIN Code (Between 100000 and 999999)
    return (100000 + (rand()%900000));
}

void app_gap_bond_cfm(struct gapc_bond_req_ind *param)
{
    struct gapc_bond_cfm * cfm = BleMsgAlloc(GAPC_BOND_CFM,
                                             KE_BUILD_ID(TASK_ID_GAPC, app_env.proxr_device.device.conidx),
                                             TASK_ID_GTL,
                                             sizeof(struct gapc_bond_cfm));

    switch(param->request)
    {
        // Bond Pairing request
        case GAPC_PAIRING_REQ:
        {
            cfm->request = GAPC_PAIRING_RSP;
            cfm->accept = true;

            // OOB information
            cfm->data.pairing_feat.oob            = GAP_OOB_AUTH_DATA_NOT_PRESENT;
            // Encryption key size
            cfm->data.pairing_feat.key_size       = KEY_LEN;
            // IO capabilities
            cfm->data.pairing_feat.iocap          = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
            // Authentication requirements
            cfm->data.pairing_feat.auth           = GAP_AUTH_REQ_NO_MITM_BOND;
            //Initiator key distribution
            cfm->data.pairing_feat.ikey_dist      = GAP_KDIST_SIGNKEY;
            //Responder key distribution
            cfm->data.pairing_feat.rkey_dist      = GAP_KDIST_ENCKEY;
            //Security requirements
            cfm->data.pairing_feat.sec_req        = GAP_NO_SEC;
        }
        break;

        // Used to retrieve pairing Temporary Key
        case GAPC_TK_EXCH:
        {
            if(param->data.tk_type == GAP_TK_DISPLAY)
            {
                uint32_t pin_code = app_gen_tk();
                cfm->request = GAPC_TK_EXCH;
                cfm->accept = true;

                memset(cfm->data.tk.key, 0, KEY_LEN);

                cfm->data.tk.key[12] = (uint8_t)((pin_code & 0xFF000000) >> 24);
                cfm->data.tk.key[13] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
                cfm->data.tk.key[14] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
                cfm->data.tk.key[15] = (uint8_t)((pin_code & 0x000000FF) >>  0);

            }
            else
            {
                assert(0);
            }
        }
        break;

        // Used for Long Term Key exchange
        case GAPC_LTK_EXCH:
        {
            // generate ltk
            app_sec_gen_ltk(param->data.key_size);

            cfm->request = GAPC_LTK_EXCH;

            cfm->accept = true;

            cfm->data.ltk.key_size = app_env.proxr_device.ltk.key_size;
            cfm->data.ltk.ediv = app_env.proxr_device.ltk.ediv;

            memcpy(&(cfm->data.ltk.randnb), &(app_env.proxr_device.ltk.randnb) , RAND_NB_LEN);
            memcpy(&(cfm->data.ltk.ltk), &(app_env.proxr_device.ltk.ltk) , KEY_LEN);
        }
        break;

        default:
        {
            BleFreeMsg(cfm); // free allocated message
            return;
        }
        break;
    }

    // Send the message
    BleSendMsg(cfm);
}

void app_start_encryption(void)
{
    // Allocate the message
    struct gapc_encrypt_cmd * req = BleMsgAlloc(GAPC_ENCRYPT_CMD,
                                                KE_BUILD_ID(TASK_ID_GAPC, app_env.proxr_device.device.conidx),
                                                TASK_ID_GTL,
                                                sizeof(struct gapc_encrypt_cmd));

    req->operation = GAPC_ENCRYPT;
    memcpy(&req->ltk.ltk, &app_env.proxr_device.ltk, sizeof(struct gapc_ltk));

    // Send the message
    BleSendMsg(req);
}

bool bdaddr_compare(struct bd_addr *bd_address1,
                    struct bd_addr *bd_address2)
{
    unsigned char idx;

    for (idx=0; idx < BD_ADDR_LEN; idx++)
    {
        /// checks if the addresses are similar
        if (bd_address1->addr[idx] != bd_address2->addr[idx])
        {
           return (false);
        }
    }
    return(true);
}

unsigned char app_device_recorded(struct bd_addr *padv_addr)
{
    int i;

    for (i=0; i < MAX_SCAN_DEVICES; i++)
    {
        if (app_env.devices[i].free == false)
            if (bdaddr_compare(&app_env.devices[i].adv_addr, padv_addr))
                break;
    }

    return i;
}

void app_proxr_create_db(void)
{
    struct proxr_db_cfg* db_cfg;
    // Add PROXR in the database
    struct gapm_profile_task_add_cmd * req = BleMsgDynAlloc(GAPM_PROFILE_TASK_ADD_CMD,
                                                            TASK_ID_GAPM,
                                                            TASK_ID_GTL,
                                                            sizeof(struct gapm_profile_task_add_cmd),
                                                            sizeof(struct proxr_db_cfg));

    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 4;
    req->prf_task_id = TASK_ID_PROXR;
    req->app_task = TASK_ID_GTL;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct proxr_db_cfg* ) req->param;
    db_cfg->features = PROXR_IAS_TXPS_SUP;

    // Send the message
    BleSendMsg((void *) req);
}

void app_diss_create_db(void)
{
    struct diss_db_cfg* db_cfg;
    // Add DISS in the database
    struct gapm_profile_task_add_cmd *req = BleMsgDynAlloc(GAPM_PROFILE_TASK_ADD_CMD,
                                                           TASK_ID_GAPM,
                                                           TASK_ID_GTL,
                                                           sizeof(struct gapm_profile_task_add_cmd),
                                                           sizeof(struct diss_db_cfg));

    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 4;
    req->prf_task_id = TASK_ID_DISS;
    req->app_task = TASK_ID_GTL;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct diss_db_cfg* ) req->param;
    db_cfg->features = APP_DIS_FEATURES;

    // Send the message
    BleSendMsg((void *) req);
}

void ConsoleEvent(void)
{
    console_msg *msg;
    WaitForSingleObject(ConsoleQueueSem, INFINITE);

    if(ConsoleQueue.First != NULL)
    {
        msg = (console_msg*) DeQueue(&ConsoleQueue);
        switch(msg->type)
        {
            case CONSOLE_DEV_DISC_CMD:
                 if (app_env.state == APP_IDLE)
                 {
                    app_env.state = APP_SCAN;
                    ConsoleScan();
                    app_inq();
                 }
                 else if (app_env.state == APP_SCAN)
                 {
                     app_rst_gap();
                 }
                 break;
            case CONSOLE_CONNECT_CMD:
                if (app_env.state == APP_IDLE)
                {
                    app_connect(msg->val);
                }
                break;
            case CONSOLE_WR_IAS_CMD:
                if (app_env.state == APP_CONNECTED && !proxm_trans_in_prog)
                {
                    proxm_trans_in_prog = 3;
                }
                break;
            case CONSOLE_DISCONNECT_CMD:
                 if (app_env.state == APP_CONNECTED)
                 {
                    app_disconnect();
                 }
                break;
            case CONSOLE_EXIT_CMD:
                app_exit();
                break;

            default:
                 break;
        }

        free(msg);
    }

    ReleaseMutex(ConsoleQueueSem);
}

void app_env_init(void)
{
    memset(app_env.proxr_device.device.adv_addr.addr, 0,  sizeof(app_env.proxr_device.device.adv_addr.addr));
    app_env.proxr_device.bonded = 0;
    app_env.num_of_devices = 0;
}

static unsigned int ConsoleReadPort(void)
{
    char buffer[256];
    char *buffer2;

    buffer2 = fgets(buffer, 256, stdin);

    if (buffer2 && buffer2[0] != '\n')
    {
        return atoi(buffer2);
    }
    else
        return 0xffffffff;
}

int main(int argc, char** argv)
{

    // initializing com port with cmd line args
    ConsoleTitle();

    if((argc == 2) && ((argv[1][1] >= '0') && (argv[1][1] <= '9')) && ((argv[1][0] >= '0') && (argv[1][0] <= '9')))
    {
        printf ("DBG: %c  %c", argv[1][0], argv[1][1]);
        Comport = ((argv[1][1]) - '0');
        Comport += (((argv[1][0]) - '0') * 10);
    }
    else if ((argc == 2) && ((argv[1][0] >= '0') && (argv[1][0] <= '9')))
    {
        Comport = ((argv[1][0]) - '0');
    }
    else
    {
        printf("No cmdline arguments.\n");
        // keep prompting user until he enters a value between 1-65535 or a blank line
        while(1) {
            int com_port_number;

            printf("Enter COM port number (values: 1-65535, blank to exit): ");
            com_port_number= ConsoleReadPort();

            if (com_port_number > 0 && com_port_number <= 0xFFFF)
            {
                Comport = com_port_number;
                break;
            }

            if (com_port_number == 0xFFFFFFFF)
                exit(0);
        }
    }

    if (!InitUART(Comport, 115200))
        InitTasks();
    else
        exit (EXIT_FAILURE);

    printf("Waiting for DA14585 Device\n");

    app_rst_gap();

    while((StopConsoleTask == FALSE) && (StopRxTask == FALSE))
    {
        BleReceiveMsg();
        ConsoleEvent();

        if (proxm_trans_in_prog > 0)
            proxm_trans_in_prog--;

    }

    exit (EXIT_SUCCESS);
}
