/**
 ****************************************************************************************
 *
 * @file uart.h
 *
 * @brief Header file for uart interface.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _UART_H_
#define _UART_H_

#define MAX_PACKET_LENGTH 350
#define MIN_PACKET_LENGTH 9

uint8_t InitUART(int Port, int BaudRate);
VOID UARTProc(PVOID unused);
VOID UARTSend(unsigned short size, unsigned char *data);


#endif /* _UART_H_ */
