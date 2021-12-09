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


void *BleMsgAlloc(unsigned short id, unsigned short dest_id,
                   unsigned short src_id, unsigned short param_len);

void *BleMsgDynAlloc(unsigned short id, unsigned short dest_id,
                     unsigned short src_id, unsigned short param_len, unsigned short length);

void BleSendMsg(void *msg);

void HandleBleMsg(ble_msg *msg);

void BleReceiveMsg(void);


#endif //BLE_MSG_H_
