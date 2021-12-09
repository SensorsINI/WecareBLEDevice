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
#include "gattc_task.h"
#include "gattm_task.h"
#include "ble_msg.h" 
#include "queue.h" 
#include "app.h" 
#include "app_task.h"
#include "proxr_task.h"
#include "diss_task.h"
#include "uart.h"

/** Internal Functions**/

int HandleGapmCmpEvt(ke_msg_id_t msgid,struct gapm_cmp_evt *param, ke_task_id_t dest_id, ke_task_id_t src_id);
int HandleGapcCmpEvt(ke_msg_id_t msgid,struct gapc_cmp_evt *param, ke_task_id_t dest_id, ke_task_id_t src_id);
int HandleGattcCmpEvt(ke_msg_id_t msgid,struct gattc_cmp_evt *param, ke_task_id_t dest_id, ke_task_id_t src_id);

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

void BleFreeMsg(void *msg)
{
    ble_msg *blemsg = (ble_msg *)((unsigned char *)msg - sizeof (ble_hdr));

    free(blemsg);
}

void HandleBleMsg(ble_msg *blemsg)
{
    if (blemsg->bDstid != TASK_ID_GTL)
    {
        puts("no task GTL");
        return;
    }

    switch (blemsg->bType)
    {
//************************************ GAPM events ************************************//

        // Command Complete event
        case GAPM_CMP_EVT:
            HandleGapmCmpEvt(blemsg->bType, (struct gapm_cmp_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break; 

        // Event triggered to inform that lower layers are ready
        case GAPM_DEVICE_READY_IND: 
            gapm_device_ready_ind_handler(blemsg->bType, (struct gap_ready_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        // Local device version indication event
        case GAPM_DEV_VERSION_IND:
            break;

        // Local device BD Address indication event
        case GAPM_DEV_BDADDR_IND:
            break;

        // Advertising channel Tx power level
        case GAPM_DEV_ADV_TX_POWER_IND:
            break;

        // Indication containing information about memory usage
        case GAPM_DBG_MEM_INFO_IND:
            break;

        // White List Size indication event
        case GAPM_WHITE_LIST_SIZE_IND:
            break;

        // Advertising or scanning report information event
        case GAPM_ADV_REPORT_IND:
            gapm_adv_report_ind_handler(blemsg->bType, (struct gapm_adv_report_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
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

        // Inform that profile task has been added.
        case GAPM_PROFILE_ADDED_IND:
            gapm_profile_added_ind_handler(blemsg->bType, (struct gapm_profile_added_ind *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        // Indicate that a message has been received on an unknown task
        case GAPM_UNKNOWN_TASK_IND:
            break;

        // Suggested Default Data Length indication
        case GAPM_SUGG_DFLT_DATA_LEN_IND:
            break;

        // Maximum Data Length indication
        case GAPM_MAX_DATA_LEN_IND:
            break;

        // Resolving address list address indication
        case GAPM_RAL_SIZE_IND:
            break;

        // Resolving address list address indication
        case GAPM_RAL_ADDR_IND:
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

        // DHKEY P256 block result indication
        case GAPM_USE_P256_BLOCK_IND:
            break;

//************************************ GAPC events ************************************//

        // Command Complete event
        case GAPC_CMP_EVT:  
            HandleGapcCmpEvt(blemsg->bType, (struct gapc_cmp_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break; 

        // Indicate that a connection has been established
        case GAPC_CONNECTION_REQ_IND:
            gapc_connection_req_ind_handler(blemsg->bType, (struct gapc_connection_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        // Indicate that a link has been disconnected
        case GAPC_DISCONNECT_IND:
            gapc_disconnect_ind_handler(blemsg->bType,  (struct gapc_disconnect_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        // Peer device attribute DB info such as Device Name, Appearance or Slave Preferred Parameters
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
            gapc_con_rssi_ind_handler(blemsg->bType, (struct gapc_con_rssi_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
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
            gapc_bond_req_ind_handler(blemsg->bType, (struct gapc_bond_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        // Bonding information indication message
        case GAPC_BOND_IND:
             gapc_bond_ind_handler(blemsg->bType,  (struct gapc_bond_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        // Encryption requested by peer device indication message.
        case GAPC_ENCRYPT_REQ_IND:
            gapc_encrypt_req_ind_handler(blemsg->bType, (struct gapc_encrypt_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        // Encryption information indication message
        case GAPC_ENCRYPT_IND:
            gapc_encrypt_ind_handler(blemsg->bType, (struct gapc_encrypt_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
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

        // LE credit based connection request indication
        case GAPC_LECB_CONNECT_REQ_IND:
            break;

        // LE credit based connection indication
        case GAPC_LECB_CONNECT_IND:
            break;

        // LE credit based credit addition indication
        case GAPC_LECB_ADD_IND:
            break;

        // disconnect indication
        case GAPC_LECB_DISCONNECT_IND:
            break;

        // LE Ping timeout indication
        case GAPC_LE_PING_TO_VAL_IND:
            break;

        // LE Ping timeout expires indication
        case GAPC_LE_PING_TO_IND:
            break;

        //Indication of LE Data Length
        case GAPC_LE_PKT_SIZE_IND:
            break;

        // Signature result
        case GAPC_SIGN_IND:
            break;

        // Parameter update procedure timeout indication
        case GAPC_PARAM_UPDATE_TO_IND:
            break;

        // Pairing procedure timeout indication
        case GAPC_SMP_TIMEOUT_TIMER_IND:
            break;

        // Pairing repeated attempts procedure timeout indication
        case GAPC_SMP_REP_ATTEMPTS_TIMER_IND:
            break;

        // Connection procedure timeout indication
        case GAPC_LECB_CONN_TO_IND:
            break;

        // Disconnection procedure timeout indication
        case GAPC_LECB_DISCONN_TO_IND:
            break;

        // Peer device sent a keypress notification
        case GAPC_KEYPRESS_NOTIFICATION_IND:
            break;

//************************************ GATTM events ************************************//
        // Add service in database response
        case GATTM_ADD_SVC_RSP:
            break;

        // Get permission settings of service response
        case GATTM_SVC_GET_PERMISSION_RSP:
            break;

        // Set permission settings of service response
        case GATTM_SVC_SET_PERMISSION_RSP:
            break;

        // Get permission settings of attribute response
        case GATTM_ATT_GET_PERMISSION_RSP:
            break;

        // Set permission settings of attribute response
        case GATTM_ATT_SET_PERMISSION_RSP:
            break;

        // Get attribute value response
        case GATTM_ATT_GET_VALUE_RSP:
            break;

        // Set attribute value response
        case GATTM_ATT_SET_VALUE_RSP:
            break;

        // DEBUG ONLY: Destroy Attribute database response
        case GATTM_DESTROY_DB_RSP:
            break;

        /// DEBUG ONLY: Retrieve list of services response
        case GATTM_SVC_GET_LIST_RSP:
            break;

        /// DEBUG ONLY: Retrieve information of attribute response
        case GATTM_ATT_GET_INFO_RSP:
            break;

//************************************ GATTC events ************************************//
        // Command Complete event
        case GATTC_CMP_EVT:
            HandleGattcCmpEvt(blemsg->bType, (struct gattc_cmp_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        // Indicate that the ATT MTU has been updated (negotiated)
        case GATTC_MTU_CHANGED_IND:
            break;

        // Discovery services indication
        case GATTC_DISC_SVC_IND:
            break;

        // Discover included services indication
        case GATTC_DISC_SVC_INCL_IND:
            break;

        // Discover characteristic indication
        case GATTC_DISC_CHAR_IND:
            break;

        // Discovery characteristic descriptor indication
        case GATTC_DISC_CHAR_DESC_IND:
            break;

        // Read response
        case GATTC_READ_IND:
            break;

        // peer device triggers an event (notification)
        case GATTC_EVENT_IND:
            break;

        // peer device triggers an event that requires a confirmation (indication)
        case GATTC_EVENT_REQ_IND:
            break;

        // Inform the application when sending of Service Changed indications has been enabled or disabled
        case GATTC_SVC_CHANGED_CFG_IND:
            break;

        // Read command indicated to upper layers.
        case GATTC_READ_REQ_IND:
            break;

        // Write command indicated to upper layers.
        case GATTC_WRITE_REQ_IND:
            break;

        // Request Attribute info to upper layer - could be trigger during prepare write
        case GATTC_ATT_INFO_REQ_IND:
            break;

        // Service Discovery indicate that a service has been found.
        case GATTC_SDP_SVC_IND:
            break;

        // Transaction Timeout Error Event no more transaction will be accepted
        case GATTC_TRANSACTION_TO_ERROR_IND:
            break;

        // Client Response timeout indication
        case GATTC_CLIENT_RTX_IND:
            break;

        // Server indication confirmation timeout indication
        case GATTC_SERVER_RTX_IND:
            break;

//************************************ PROXR events ************************************//

        case PROXR_ALERT_IND:
            proxr_alert_ind_handler(blemsg->bDstid, (struct proxr_alert_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

//************************************ DISS events ************************************//

        case DISS_VALUE_REQ_IND:
            diss_value_req_ind_handler(blemsg->bDstid, (struct diss_value_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

//************************************ Unknown event ************************************//

        default:
            printf("Rcved UNKNOWN Msg 0x%x\n", blemsg->bType);
            break;

    }
}

/*
 ****************************************************************************************
 * @brief Receives ble message from UART iface.
 ****************************************************************************************
*/
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

int HandleGapmCmpEvt(ke_msg_id_t msgid,
                    struct gapm_cmp_evt *param,
                    ke_task_id_t dest_id,
                    ke_task_id_t src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
    {
        switch(param->operation)
        {
            case GAPM_NO_OP:// No operation.
                break;
            case GAPM_RESET:// Reset BLE subsystem: LL and HL.
                gapm_reset_completion_handler (msgid, (struct gapm_cmp_evt *)param, dest_id, src_id);
                break;
            case GAPM_CANCEL:// Cancel currently executed operation.
                break;
            case GAPM_SET_DEV_CONFIG:// Set device configuration
                gapm_set_dev_config_completion_handler(msgid, (struct gapm_cmp_evt *)param, dest_id, src_id);
                break;
            case GAPM_SET_CHANNEL_MAP:
                break;
            case GAPM_GET_DEV_VERSION:
                break;
            case GAPM_GET_DEV_BDADDR:
                break;
            case GAPM_GET_DEV_ADV_TX_POWER:
                break;
            case GAPM_GET_WLIST_SIZE:
                break;
            case GAPM_ADD_DEV_IN_WLIST:
                break;
            case GAPM_RMV_DEV_FRM_WLIST:
                break;
            case GAPM_CLEAR_WLIST:
                break;
            case GAPM_ADV_NON_CONN:
                break;
            case GAPM_ADV_UNDIRECT:
                break;
            case GAPM_ADV_DIRECT:
                break;
            case GAPM_ADV_DIRECT_LDC:
                break;
            case GAPM_UPDATE_ADVERTISE_DATA:
                break;
            case GAPM_SCAN_ACTIVE:
                break;
            case GAPM_SCAN_PASSIVE:
                break;
            case GAPM_CONNECTION_DIRECT:
                break;
            case GAPM_CONNECTION_AUTO:
                break;
            case GAPM_CONNECTION_SELECTIVE:
                break;
            case GAPM_CONNECTION_NAME_REQUEST:
                break;
            case GAPM_RESOLV_ADDR:
                break;
            case GAPM_GEN_RAND_ADDR:
                break;
            case GAPM_USE_ENC_BLOCK:
                break;
            case GAPM_GEN_RAND_NB:
                break;
            case GAPM_PROFILE_TASK_ADD:
                break;
            case GAPM_DBG_GET_MEM_INFO:
                break;
            case GAPM_PLF_RESET:
                break;
            case GAPM_SET_SUGGESTED_DFLT_LE_DATA_LEN:
                break;
            case GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN:
                break;
            case GAPM_GET_MAX_LE_DATA_LEN:
                break;
            case GAPM_GET_RAL_SIZE:
                break;
            case GAPM_GET_RAL_LOC_ADDR:
                break;
            case GAPM_GET_RAL_PEER_ADDR:
                break;
            case GAPM_ADD_DEV_IN_RAL:
                break;
            case GAPM_RMV_DEV_FRM_RAL:
                break;
            case GAPM_CLEAR_RAL:
                break;
            case GAPM_USE_P256_BLOCK:
                break;
            case GAPM_NETWORK_MODE_RAL:
                break;
            case GAPM_DEVICE_MODE_RAL:
                break;
            case GAPM_LAST:
                break;
        }
    }

    return (KE_MSG_CONSUMED);
}

int HandleGapcCmpEvt(ke_msg_id_t msgid,
                    struct gapc_cmp_evt *param,
                    ke_task_id_t dest_id,
                    ke_task_id_t src_id)
{
    switch(param->operation)
    {
        case GAPC_NO_OP:
            break;
        case GAPC_DISCONNECT:
            break;
        case GAPC_GET_PEER_NAME:
            break;
        case GAPC_GET_PEER_VERSION:
            break;
        case GAPC_GET_PEER_FEATURES:
            break;
        case GAPC_GET_PEER_APPEARANCE:
            break;
        case GAPC_GET_PEER_SLV_PREF_PARAMS:
            break;
        case GAPC_GET_CON_RSSI:
            break;
        case GAPC_GET_CON_CHANNEL_MAP:
            break;
        case GAPC_UPDATE_PARAMS:
            break;
        case GAPC_BOND:
            break;
        case GAPC_ENCRYPT:
            break;
        case GAPC_SECURITY_REQ:
            break;
        case GAPC_LE_CB_CREATE:
            break;
        case GAPC_LE_CB_DESTROY:
            break;
        case GAPC_LE_CB_CONNECTION:
            break;
        case GAPC_LE_CB_DISCONNECTION:
            break;
        case GAPC_LE_CB_ADDITION:
            break;
        case GAPC_GET_LE_PING_TO:
            break;
        case GAPC_SET_LE_PING_TO:
            break;
        case GAPC_SET_LE_PKT_SIZE:
            break;
        case GAPC_GET_PEER_CENTRAL_RPA:
            break;
        case GAPC_GET_PEER_RPA_ONLY:
            break;
        case GAPC_SIGN_PACKET:
            break;
        case GAPC_SIGN_CHECK:
            break;
        case GAPC_LAST:
            break;
    }
    return (KE_MSG_CONSUMED);
}

int HandleGattcCmpEvt(ke_msg_id_t msgid,
                    struct gattc_cmp_evt *param,
                    ke_task_id_t dest_id,
                    ke_task_id_t src_id)
{
    switch(param->operation)
    {
        case GATTC_NO_OP:
            break;
        case GATTC_MTU_EXCH:
            break;
        case GATTC_DISC_ALL_SVC:
            break;
        case GATTC_DISC_BY_UUID_SVC:
            break;
        case GATTC_DISC_INCLUDED_SVC:
            break;
        case GATTC_DISC_ALL_CHAR:
            break;
        case GATTC_DISC_BY_UUID_CHAR:
            break;
        case GATTC_DISC_DESC_CHAR:
            break;
        case GATTC_READ:
            break;
        case GATTC_READ_LONG:
            break;
        case GATTC_READ_BY_UUID:
            break;
        case GATTC_READ_MULTIPLE:
            break;
        case GATTC_WRITE:
            break;
        case GATTC_WRITE_NO_RESPONSE:
            break;
        case GATTC_WRITE_SIGNED:
            break;
        case GATTC_EXEC_WRITE:
            break;
        case GATTC_REGISTER:
            break;
        case GATTC_UNREGISTER:
            break;
        case GATTC_NOTIFY:
            break;
        case GATTC_INDICATE:
            break;
        case GATTC_SVC_CHANGED:
            break;
        case GATTC_SDP_DISC_SVC:
            break;
        case GATTC_SDP_DISC_SVC_ALL:
            break;
        case GATTC_SDP_DISC_CANCEL:
            break;
        case GATTC_LAST:
            break;
    }
    return (KE_MSG_CONSUMED);
}

