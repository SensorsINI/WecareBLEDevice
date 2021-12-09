/**
 ****************************************************************************************
 *
 * @file uart_common.h
 *
 * @brief UART examples common header file.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _UART_COMMON_H
#define _UART_COMMON_H

#include "uart.h"

// Send examples functions

/**
 ****************************************************************************************
 * @brief UART send in blocking mode example.
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_send_blocking_example(uart_t* uart);

/**
 ****************************************************************************************
 * @brief UART send in interrupt mode example.
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_send_interrupt_example(uart_t* uart);

/**
 ****************************************************************************************
 * @brief UART send in DMA mode example.
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_send_dma_example(uart_t* uart);

// Receive examples functions
/**
 ****************************************************************************************
 * @brief UART receive in blocking mode example.
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_receive_blocking_example(uart_t* uart);

/**
 ****************************************************************************************
 * @brief UART receive in interrupt mode example.
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_receive_interrupt_example(uart_t* uart);

/**
 ****************************************************************************************
 * @brief UART receive in DMA mode example.
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_receive_dma_example(uart_t* uart);

// Loopback examples functions

/**
 ****************************************************************************************
 * @brief UART Interrupt loopback example
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_loopback_interrupt_example(uart_t *uart);


#endif // _UART2_PRINT_STRING_H
