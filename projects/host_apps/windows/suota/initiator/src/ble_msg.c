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

#include "ble_msg.h"
#include "queue.h"

#include "gapm_task.h"
#include "gapc_task.h"
#include "gattc_task.h"

#include "app_task.h"
#include "uart.h"

/*
 ****************************************************************************************
 * @brief Send message to UART iface.
 *
 * @param[in] msg   pointer to message.
 ****************************************************************************************
*/
void BleSendMsg(void *msg)
{
    ble_msg *blemsg = (ble_msg *)((unsigned char *)msg - sizeof (ble_hdr));

    UARTSend(blemsg->bLength + sizeof(ble_hdr), (unsigned char *) blemsg);

    free(blemsg);
}

/*
 ****************************************************************************************
 * @brief Allocate memory for ble message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @param[in] param_len Parameters length.
 *
 * @return pointer to allocated message
 ****************************************************************************************
*/
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

/*
 ****************************************************************************************
 * @brief Allocate memory for ble message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @param[in] param_len Parameters length.
 * @param[in] length    Length for the data
 *
 * @return pointer to allocated message
 ****************************************************************************************
*/
void *BleMsgDynAlloc(unsigned short id, unsigned short dest_id,
                     unsigned short src_id, unsigned short param_len, unsigned short length)
{
    return (BleMsgAlloc(id, dest_id, src_id, (param_len+length)));
}

/*
 ****************************************************************************************
 * @brief Handles ble message by calling corresponding procedure.
 *
 * @param[in] blemsg    Pointer to received message.
 ****************************************************************************************
*/
void HandleBleMsg(ble_msg *blemsg)
{
    if (blemsg->bDstid != TASK_ID_GTL)
        return;

    switch (blemsg->bType)
    {
        case GAPM_DEVICE_READY_IND:
            gapm_ready_evt_handler(GAPM_DEVICE_READY_IND, (struct gap_ready_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPM_CMP_EVT:
            gapm_cmp_evt_handler(GAPM_CMP_EVT, (struct gapm_cmp_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPM_ADV_REPORT_IND:
            gapm_dev_inq_result_handler(GAPM_ADV_REPORT_IND, (struct gapm_adv_report_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPC_CMP_EVT:
            gapc_cmp_evt_handler(GAPC_CMP_EVT, (struct gapc_cmp_evt *)blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPC_CONNECTION_REQ_IND:
            gapc_connection_req_ind_handler(GAPC_CONNECTION_REQ_IND, (struct gapc_connection_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPC_DISCONNECT_IND:
            gapc_disconnect_ind_handler(GAPC_DISCONNECT_IND, (struct gapc_disconnect_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPC_PARAM_UPDATE_REQ_IND:
            gapc_param_update_req_ind_handler(blemsg->bType, (struct gapc_param_update_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPC_GET_DEV_INFO_REQ_IND:
             gapc_get_dev_info_req_ind_handler(blemsg->bType, (struct gapc_get_dev_info_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
             break;

            // Peer device request to modify local device info such as name or appearance
        case GAPC_SET_DEV_INFO_REQ_IND:
             gapc_set_dev_info_req_ind_handler(blemsg->bType, (struct gapc_set_dev_info_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
             break;

        case GAPC_BOND_IND:
            gapc_bond_ind_handler(GAPC_BOND_IND, (struct gapc_bond_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPC_BOND_REQ_IND:
            gapc_bond_req_ind_handler(GAPC_BOND_REQ_IND, (struct gapc_bond_req_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GAPC_LE_PKT_SIZE_IND:
            gapc_le_pkt_size_ind_handler(GAPC_LE_PKT_SIZE_IND, (struct gapc_le_pkt_size_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GATTC_DISC_SVC_IND:
            gattc_disc_svc_ind_handler(GATTC_DISC_SVC_IND, (struct gattc_disc_svc_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GATTC_DISC_CHAR_IND:
            gattc_disc_char_ind_handler(GATTC_DISC_CHAR_IND, (struct gattc_disc_char_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GATTC_DISC_CHAR_DESC_IND:
            gattc_disc_char_desc_ind_handler(GATTC_DISC_CHAR_IND, (struct gattc_disc_char_desc_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GATTC_EVENT_IND:
            gattc_event_ind_handler(GATTC_EVENT_IND, (struct gattc_event_ind *) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GATTC_CMP_EVT:
            gattc_cmp_evt_handler(GATTC_CMP_EVT, (struct gattc_cmp_evt*) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GATTC_READ_IND:
            gattc_read_ind_handler(GATTC_READ_IND, (struct gattc_read_ind*) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        case GATTC_MTU_CHANGED_IND:
            gattc_mtu_changed_ind_handler(GATTC_MTU_CHANGED_IND, (struct gattc_mtu_changed_ind*) blemsg->bData, blemsg->bDstid, blemsg->bSrcid);
            break;

        default:
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
