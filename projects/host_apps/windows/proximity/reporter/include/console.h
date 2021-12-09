/**
 ****************************************************************************************
 *
 * @file console.h
 *
 * @brief Header file for basic console user interface of the host application.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include "rwble_config.h"

typedef struct{
    unsigned char type;
    unsigned char val;
} console_msg;

enum
{
    CONSOLE_DEV_DISC_CMD,
    CONSOLE_CONNECT_CMD,
    CONSOLE_DISCONNECT_CMD,
    CONSOLE_RD_LLV_CMD,
    CONSOLE_RD_TXP_CMD,
    CONSOLE_WR_LLV_CMD,
    CONSOLE_WR_IAS_CMD,
    CONSOLE_EXIT_CMD,
};

/*
 ****************************************************************************************
 * @brief Sends Discover devices message to application 's main thread.
 ****************************************************************************************
*/
void ConsoleSendScan(void);

/*
 ****************************************************************************************
 * @brief Sends Connect to device message to application 's main thread.
 *
 *  @param[in] indx Device's index in discovered devices list.
 ****************************************************************************************
*/
void ConsoleSendConnnect(int indx);

/*
 ****************************************************************************************
 * @brief Sends Read request message to application 's main thread.
 ****************************************************************************************
*/
void ConsoleSendDisconnnect(void);

/*
 ****************************************************************************************
 * @brief Sends Read request message to application 's main thread.
 *
 *  @param[in] type  Attribute type to read.
 ****************************************************************************************
*/
void ConsoleRead(unsigned char type);

/*
 ****************************************************************************************
 * @brief Sends write request message to application 's main thread.
 *
 *  @param[in] type  Attribute type to write.
 *  @param[in] val   Attribute value.
 ****************************************************************************************
*/
void ConsoleWrite(unsigned char type, unsigned char val);

/*
 ****************************************************************************************
 * @brief Sends a message to application 's main thread to exit.
 ****************************************************************************************
*/
void ConsoleSendExit(void);

/*
 ****************************************************************************************
 * @brief Handles keypress events and sends corresponding message to application's main thread.
 ****************************************************************************************
*/
void HandleKeyEvent(int Key);

/*
 ****************************************************************************************
 * @brief Demo application Console Menu header
 ****************************************************************************************
*/
void ConsoleTitle(void);

/*
 ****************************************************************************************
 * @brief Prints Console menu in Scan state.
 ****************************************************************************************
*/
void ConsoleScan(void);

/*
 ****************************************************************************************
 * @brief Prints Console menu in Idle state.
 ****************************************************************************************
*/
void ConsoleIdle(void);

/*
 ****************************************************************************************
 * @brief Prints List of discovered devices.
 ****************************************************************************************
*/
void ConsolePrintScanList(void);

/*
 ****************************************************************************************
 * @brief Prints Console Menu in connected state.
 *
 * @param[in] full   If true, prints peers attributes values.
 ****************************************************************************************
*/
void ConsoleConnected(int full);

/*
 ****************************************************************************************
 * @brief Console Thread main loop.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
*/
VOID ConsoleProc(PVOID unused);

#endif //CONSOLE_H_
