/**
 ****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
 *
 * Copyright (C) 2015-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gpio.h"
#include "uart.h"

/*
 * DEFINES
 ****************************************************************************************
 */
// Define UART1, UART2 Pads
#if defined (__DA14531__)
    #define UART1_TX_PORT           GPIO_PORT_0
    #define UART1_TX_PIN            GPIO_PIN_6
    #define UART1_RX_PORT           GPIO_PORT_0
    #define UART1_RX_PIN            GPIO_PIN_7

    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_6
    #define UART2_RX_PORT           GPIO_PORT_0
    #define UART2_RX_PIN            GPIO_PIN_7

#else // DA14585, DA14586
    #define UART1_TX_PORT           GPIO_PORT_0
    #define UART1_TX_PIN            GPIO_PIN_4
    #define UART1_RX_PORT           GPIO_PORT_0
    #define UART1_RX_PIN            GPIO_PIN_5

    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_4
    #define UART2_RX_PORT           GPIO_PORT_0
    #define UART2_RX_PIN            GPIO_PIN_5
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes application's peripherals and pins
 ****************************************************************************************
 */
void periph_init(void);

/**
 ****************************************************************************************
 * @brief Initializes UART Peripheral (Pads and Configuration)
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_periph_init(uart_t *uart);

/**
 ****************************************************************************************
 * @brief Set gpio port function mode
 ****************************************************************************************
 */
void set_pad_functions(void);

#endif // _USER_PERIPH_SETUP_H_
