/**
 ****************************************************************************************
 *
 * @file uart_booter.h
 *
 * @brief user booter protocol settings
 *
 * Copyright (C) 2012-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
 
#ifndef _UART_BOOTER_H
#define _UART_BOOTER_H

#if defined (__DA14531__)
#define MAX_CODE_LENGTH             (0x7B00)
#define SYSRAM_COPY_BASE_ADDRESS    (0x500)
#else
#define MAX_CODE_LENGTH             (0x12000)
#define SYSRAM_COPY_BASE_ADDRESS    (0x2000)
#endif
#define MAX_CODE_LENGTH_SPI         (MAX_CODE_LENGTH + SYSRAM_COPY_BASE_ADDRESS)
#define MAX_CODE_LENGTH_I2C         (MAX_CODE_LENGTH + SYSRAM_COPY_BASE_ADDRESS)
#define SYSRAM_BASE_ADDRESS         (0x7FC0000)
#define SOH     (0x01)
#define STX     (0x02)
#define ACK     (0x06)
#define NAK     (0x15)

#endif
