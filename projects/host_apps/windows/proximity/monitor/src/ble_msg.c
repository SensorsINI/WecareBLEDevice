/**
 ****************************************************************************************
 *
 * @file ble_msg.c
 *
 * @brief Reception of ble messages sent from DA14585 embedded application over UART interface.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */


#include "gapc_task.h"
#include "gapm_task.h"
#include "ble_msg.h" 
#include "queue.h" 
#include "app_task.h"
#include "proxm_task.h" 
#include "disc_task.h"
#include "uart.h"



void BleSendMsg(void *msg)
{
    ble_msg *blemsg = (ble_msg *)((unsigned char *)msg - sizeof (ble_hdr));

    UARTSend(blemsg->bLength + sizeof(ble_hdr), (unsigned char *) blemsg);

    free(blemsg);
}


void *BleMsgAlloc(unsigned short id, unsigned short dest_id,
    unsigned short src_id, unsigned short param_len)
{
    ble_msg *blemsg = (ble_msg *) malloc(sizeof(ble_msg) + param_len - sizeof (unsigned char));

    blemsg->bType    = id;
    blemsg->bDstid   = dest_id;
    blemsg->bSrcid   = src_id;
    blemsg->bLength  = param_len;

    if (param_len)
        memset(blemsg->bData, 0, param_len);

    return blemsg->bData;
}

void *BleMsgDynAlloc(unsigned short id, unsigned short dest_id,
    unsigned short src_id, unsigned short param_len, unsigned short length)
{
    return (BleMsgAlloc(id, dest_id, src_id, (param_len+length)));
}

void HandleBleMsg(ble_msg *blemsg)
{

    if (blemsg->bDstid != TASK_ID_GTL )
    {
        return;
    }
    switch (blemsg->bType)
    {
    //
    // GAPM events
    //
    // Command Complete event
    case GAPM_CMP_EVT:
        gapm_cmp_evt_handler(GAPM_CMP_EVT, (struct gapm_cmp_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Event triggered to inform that lower layers are ready
    case GAPM_DEVICE_READY_IND:
        gapm_device_ready_ind_handler(GAPM_DEVICE_READY_IND, (struct gap_ready_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Local device version indication event
    case GAPM_DEV_VERSION_IND:
        break;

        // Local device BD Address indication event
    case GAPM_DEV_BDADDR_IND:
        break;

        // White List Size indication event
    case GAPM_WHITE_LIST_SIZE_IND:
        break;

        // Advertising or scanning report information event
    case GAPM_ADV_REPORT_IND:
        gapm_adv_report_ind_handler(GAPM_ADV_REPORT_IND, (struct gapm_adv_report_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Name of peer device indication
    case GAPM_PEER_NAME_IND:
        break;

        // Indicate that Resolvable Private Address has been solved
    case GAPM_ADDR_SOLVED_IND:
        break;

        //  AES-128 block result indication
    case GAPM_USE_ENC_BLOCK_IND:
        break;

        // Random Number Indication
    case GAPM_GEN_RAND_NB_IND:
        break;

        // Indication containing information about memory usage.
    case GAPM_DBG_MEM_INFO_IND:
        break;

        // Advertising channel Tx power level
    case GAPM_DEV_ADV_TX_POWER_IND:
        break;

        // Limited discoverable timeout indication
    case GAPM_LIM_DISC_TO_IND:
        break;

        // Scan timeout indication
    case GAPM_SCAN_TO_IND:
        break;

        // Address renewal timeout indication
    case GAPM_ADDR_RENEW_TO_IND:
        break;

        //
        // GAPC events
        //

        // Command Complete event
    case GAPC_CMP_EVT:
        gapc_cmp_evt_handler(GAPC_CMP_EVT, (struct gapc_cmp_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Indicate that a connection has been established
    case GAPC_CONNECTION_REQ_IND:
        gapc_connection_req_ind_handler(GAPC_CONNECTION_REQ_IND, (struct gapc_connection_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Indicate that a link has been disconnected
    case GAPC_DISCONNECT_IND:
        gapc_disconnect_ind_handler(GAPC_DISCONNECT_IND, (struct gapc_disconnect_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Name,Appearance of peer device indication
    case GAPC_PEER_ATT_INFO_IND:
        break;

        // Indication of peer version info
    case GAPC_PEER_VERSION_IND:
        break;

        // Indication of peer features info
    case GAPC_PEER_FEATURES_IND:
        break;

        // Indication of ongoing connection RSSI
    case GAPC_CON_RSSI_IND:
        gapc_con_rssi_ind_handler(GAPC_CON_RSSI_IND, (struct gapc_con_rssi_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Device info request indication
    case GAPC_GET_DEV_INFO_REQ_IND:
        gapc_get_dev_info_req_ind_handler(blemsg->bType, (struct gapc_get_dev_info_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Peer device request to modify local device info such as name or appearance
    case GAPC_SET_DEV_INFO_REQ_IND:
         gapc_set_dev_info_req_ind_handler(blemsg->bType, (struct gapc_set_dev_info_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
         break;

        // Request of updating connection parameters indication
    case GAPC_PARAM_UPDATE_REQ_IND:
        gapc_param_update_req_ind_handler(blemsg->bType, (struct gapc_param_update_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Connection parameters updated indication
    case GAPC_PARAM_UPDATED_IND:
        break;

        // Bonding requested by peer device indication message.
    case GAPC_BOND_REQ_IND:
        gapc_bond_req_ind_handler(GAPC_BOND_REQ_IND, (struct gapc_bond_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Bonding information indication message
    case GAPC_BOND_IND:
        gapc_bond_ind_handler(GAPC_BOND_IND, (struct gapc_bond_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        // Encryption requested by peer device indication message.
    case GAPC_ENCRYPT_REQ_IND:
        break;

        // Encryption information indication message
    case GAPC_ENCRYPT_IND:
        break;

        // Security requested by peer device indication message
    case GAPC_SECURITY_IND:
        break;

        // Indicate the current sign counters to the application
    case GAPC_SIGN_COUNTER_IND:
        break;

        // Indication of ongoing connection Channel Map
    case GAPC_CON_CHANNEL_MAP_IND:
        break;

        // Parameter update procedure timeout indication
    case GAPC_PARAM_UPDATE_TO_IND:
        break;

        //
        // PROXM events
        //
    case PROXM_ENABLE_RSP:
        proxm_enable_rsp_handler(PROXM_ENABLE_RSP, (struct proxm_enable_rsp *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

    case PROXM_RD_RSP:
        proxm_rd_char_rsp_handler(PROXM_RD_RSP, (struct proxm_rd_rsp *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

    case PROXM_WR_ALERT_LVL_RSP:
        proxm_wr_alert_lvl_rsp_handler(PROXM_WR_ALERT_LVL_RSP, (struct proxm_wr_alert_lvl_rsp *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

        //
        // DISC events
        //
    case DISC_ENABLE_RSP:
        disc_enable_rsp_handler(DISC_ENABLE_RSP, (struct disc_enable_rsp *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;
    case DISC_RD_CHAR_RSP:
        disc_rd_char_rsp_handler(DISC_RD_CHAR_RSP, (struct disc_rd_char_rsp *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
        break;

    default:
        break;
    }

}


void BleReceiveMsg(void)
{
    ble_msg *msg;
    WaitForSingleObject(UARTRxQueueSem, INFINITE);

    if(UARTRxQueue.First != NULL)
    {
        msg = (ble_msg*) DeQueue(&UARTRxQueue); 
        HandleBleMsg(msg);
        free(msg);
    }

    ReleaseMutex(UARTRxQueueSem);
}


