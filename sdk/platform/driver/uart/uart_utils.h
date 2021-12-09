/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup UART
 * @{
 * @addtogroup UART_UTILS UART Driver Utilities
 * @brief UART driver utilities API
 * @{
 *
 * @file uart_utils.h
 *
 * @brief UART utilities header file.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _UART_UTILS_H_
#define _UART_UTILS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "uart.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief print a Byte in hex format
 * @param[in] uart              Identifies which UART to use
 * @param[in] ch                Character to print
 ****************************************************************************************
 */
void printf_byte(uart_t *uart, uint8_t ch);

/**
 ****************************************************************************************
 * @brief UART print string function
 * @param[in] uart              Identifies which UART to use
 * @param[in] str               String to print
 ****************************************************************************************
 */
void printf_string(uart_t *uart, char *str);

/**
 ****************************************************************************************
 * @brief Prints a (16-bit) half-word in hex format using printf_byte
 * @param[in] uart              Identifies which UART to use
 * @param[in] aHalfWord         16-bit half-word to print
 ****************************************************************************************
 */
void print_hword(uart_t *uart, uint16_t aHalfWord);

 /**
 ****************************************************************************************
 * @brief Prints a (32-bit) word in hex format using printf_byte
 * @param[in] uart              Identifies which UART to use
 * @param[in] aWord             32-bit half-word to print
 ****************************************************************************************
 */
void print_word(uart_t *uart, uint32_t aWord);

 /**
 ****************************************************************************************
 * @brief prints a byte in dec format
 * @param[in] uart              Identifies which UART to use
 * @param[in] a                 Byte to print
 ****************************************************************************************
 */
void printf_byte_dec(uart_t *uart, int a);

#endif //_UART_PRINT_H_

///@}
///@}
///@}
