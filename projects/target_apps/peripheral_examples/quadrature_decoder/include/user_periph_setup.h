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

#include "uart.h"
#include "gpio.h"
#include "wkupct_quadec.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/****************************************************************************************/
/* UART2 configuration to print messages                                                */
/****************************************************************************************/
#define UART                        UART2
// Define UART2 Tx Pad
#if defined (__DA14531__)
    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_8
#else
    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_4
#endif

// Define UART2 Settings
#define UART2_BAUDRATE              UART_BAUDRATE_115200
#define UART2_DATABITS              UART_DATABITS_8
#define UART2_PARITY                UART_PARITY_NONE
#define UART2_STOPBITS              UART_STOPBITS_1
#define UART2_AFCE                  UART_AFCE_DIS
#define UART2_FIFO                  UART_FIFO_EN
#define UART2_TX_FIFO_LEVEL         UART_TX_FIFO_LEVEL_0
#define UART2_RX_FIFO_LEVEL         UART_RX_FIFO_LEVEL_0


/****************************************************************************************/
/* Quadrature Decoder options                                                           */
/****************************************************************************************/
#define QUADRATURE_ENCODER_CHY_CONFIGURATION QUAD_DEC_CHYA_NONE_AND_CHYB_NONE
#define QUADRATURE_ENCODER_CHZ_CONFIGURATION QUAD_DEC_CHZA_NONE_AND_CHZB_NONE
#if defined (__DA14531__)
#define QUADRATURE_ENCODER_CHX_A_PORT        GPIO_PORT_0
#define QUADRATURE_ENCODER_CHX_A_PIN         GPIO_PIN_6
#define QUADRATURE_ENCODER_CHX_B_PORT        GPIO_PORT_0
#define QUADRATURE_ENCODER_CHX_B_PIN         GPIO_PIN_7
#define QUADRATURE_ENCODER_CHX_CONFIGURATION QUAD_DEC_CHXA_P06_AND_CHXB_P07
#define WKUP_TEST_BUTTON_1_PORT              GPIO_PORT_0
#define WKUP_TEST_BUTTON_1_PIN               GPIO_PIN_11
#define WKUP_TEST_BUTTON_2_PORT              GPIO_PORT_0
#define WKUP_TEST_BUTTON_2_PIN               GPIO_PIN_1
#else
#define QUADRATURE_ENCODER_CHX_A_PORT        GPIO_PORT_0
#define QUADRATURE_ENCODER_CHX_A_PIN         GPIO_PIN_0
#define QUADRATURE_ENCODER_CHX_B_PORT        GPIO_PORT_0
#define QUADRATURE_ENCODER_CHX_B_PIN         GPIO_PIN_1
#define QUADRATURE_ENCODER_CHX_CONFIGURATION QUAD_DEC_CHXA_P00_AND_CHXB_P01
#define WKUP_TEST_BUTTON_1_PORT              GPIO_PORT_0
#define WKUP_TEST_BUTTON_1_PIN               GPIO_PIN_6
#define WKUP_TEST_BUTTON_2_PORT              GPIO_PORT_1
#define WKUP_TEST_BUTTON_2_PIN               GPIO_PIN_1
#endif
#define QDEC_CLOCK_DIVIDER                   (1)
#define QDEC_EVENTS_COUNT_TO_INT             (1)

#define DEVELOPMENT_DEBUG                    0

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Initializes application's peripherals and pins
 ****************************************************************************************
 */
void periph_init(void);

/**
 ****************************************************************************************
 * @brief Set gpio port function mode
 ****************************************************************************************
 */
void set_pad_functions(void);

#endif // _USER_PERIPH_SETUP_H_
