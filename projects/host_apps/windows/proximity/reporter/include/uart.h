/**
 ****************************************************************************************
 *
 * @file uart.h
 *
 * @brief Definitions for uart interface.
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
 /*
 ****************************************************************************************
 * @brief Write message to UART.
 *
 *  @param[in] size  Message's size.
 *  @param[in] data  Pointer to message's data.
 ****************************************************************************************
*/
void UARTSend(unsigned short size, unsigned char *data);
/*
 ****************************************************************************************
 * @brief Send message received from UART to application's main thread.
 *
 *  @param[in] length           Message's size.
 *  @param[in] bInputDataPtr    Pointer to message's data.
 ****************************************************************************************
*/
void SendToMain(unsigned short length, uint8_t *bInputDataPtr);
/*
 ****************************************************************************************
 * @brief UART Reception thread loop.
 ****************************************************************************************
*/
void UARTProc(PVOID unused);

/*
 ****************************************************************************************
 * @brief Init UART iface.
 *
 *  @param[in] Port         COM prot number.
 *  @param[in] BaudRate     Baud rate.
 *
 * @return -1 on failure / 0 on success.
 ****************************************************************************************
*/
uint8_t InitUART(int Port, int BaudRate);


#endif /* _UART_H_ */
