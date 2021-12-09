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
#include "i2c_eeprom.h"

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
    #define UART2_TX_PIN            GPIO_PIN_6
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
/* I2C configuration                                                                    */
/****************************************************************************************/
// Define I2C Pins
#if defined (__DA14531__)
    #define I2C_SCL_PORT            GPIO_PORT_0
    #define I2C_SCL_PIN             GPIO_PIN_3
    #define I2C_SDA_PORT            GPIO_PORT_0
    #define I2C_SDA_PIN             GPIO_PIN_1
#else
    #define I2C_SCL_PORT            GPIO_PORT_2
    #define I2C_SCL_PIN             GPIO_PIN_3
    #define I2C_SDA_PORT            GPIO_PORT_2
    #define I2C_SDA_PIN             GPIO_PIN_1
#endif

// Select EEPROM characteristics
#define I2C_EEPROM_DEV_SIZE         0x20000               // EEPROM size in bytes
#define I2C_EEPROM_PAGE_SIZE        256                   // EEPROM page size in bytes
#define I2C_SLAVE_ADDRESS           0x50                  // Set slave device address
#define I2C_SPEED_MODE              I2C_SPEED_FAST        // Speed mode: I2C_SPEED_STANDARD (100 kbits/s), I2C_SPEED_FAST (400 kbits/s)
#define I2C_ADDRESS_MODE            I2C_ADDRESSING_7B     // Addressing mode: {I2C_ADDRESSING_7B, I2C_ADDRESSING_10B}
#define I2C_ADDRESS_SIZE            I2C_2BYTES_ADDR       // Address width: {I2C_1BYTE_ADDR, I2C_2BYTES_ADDR, I2C_3BYTES_ADDR}



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
