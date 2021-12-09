/**
 ****************************************************************************************
 *
 * @file ble_msg.h
 *
 * @brief Header file for reception of ble messages sent from DA14585 embedded application over UART interface.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef BLE_MSG_H_
#define BLE_MSG_H_

#include "rwble_config.h"

typedef struct {
    unsigned short bType;
    unsigned short bDstid;
    unsigned short bSrcid;
    unsigned short bLength;
} ble_hdr;


typedef struct {
    unsigned short bType;
    unsigned short bDstid;
    unsigned short bSrcid;
    unsigned short bLength;
    unsigned char  bData[1];
} ble_msg;

 /*
 ****************************************************************************************
 * @brief Send message to UART iface.
 *
 * @param[in] msg   pointer to message.
 ****************************************************************************************
*/
void BleSendMsg(void *msg);

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
                  unsigned short src_id, unsigned short param_len);

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
                     unsigned short src_id, unsigned short param_len, unsigned short length);


/*
 ****************************************************************************************
 * @brief Free ble msg.
 *
 * @param[in] msg   pointer to message.
 ****************************************************************************************
*/
void BleFreeMsg(void *msg);

/*
 ****************************************************************************************
 * @brief Free ble msg.
 *
 * @param[in] msg   pointer to message.
 ****************************************************************************************
*/
void BleFreeMsg(void *msg);

/*
 ****************************************************************************************
 * @brief Handles ble by calling corresponding procedure.
 *
 * @param[in] blemsg    Pointer to received message.
 ****************************************************************************************
*/
void HandleBleMsg(ble_msg *blemsg);

/*
 ****************************************************************************************
 * @brief Receives ble message from UART iface.
 ****************************************************************************************
*/
void BleReceiveMsg(void);



#endif //BLE_MSG_H_
