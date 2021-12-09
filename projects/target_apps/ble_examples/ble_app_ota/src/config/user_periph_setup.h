/**
 ****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
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
#include "spi.h"
#include "spi_flash.h"
#include "i2c.h"
#include "i2c_eeprom.h"



/*
 * DEFINES
 ****************************************************************************************
 */


/****************************************************************************************/
/* UART1 configuration                                                                  */
/****************************************************************************************/
// Define UART1 Pads
#define UART1_TX_PORT               GPIO_PORT_0
#define UART1_TX_PIN                GPIO_PIN_4

#define UART1_RX_PORT               GPIO_PORT_0
#define UART1_RX_PIN                GPIO_PIN_5

#define UART1_RTSN_PORT             GPIO_PORT_0
#define UART1_RTSN_PIN              GPIO_PIN_6

#define UART1_CTSN_PORT             GPIO_PORT_0
#define UART1_CTSN_PIN              GPIO_PIN_7

// Define UART1 Settings
#define UART1_BAUDRATE              UART_BAUDRATE_115200
#define UART1_DATABITS              UART_DATABITS_8

/* The following UART1 settings can be used if the SDK driver is
   selected for UART1.
*/
#define UART1_PARITY                UART_PARITY_NONE
#define UART1_STOPBITS              UART_STOPBITS_1
#define UART1_AFCE                  UART_AFCE_EN
#define UART1_FIFO                  UART_FIFO_EN
#define UART1_TX_FIFO_LEVEL         UART_TX_FIFO_LEVEL_0
#define UART1_RX_FIFO_LEVEL         UART_RX_FIFO_LEVEL_0


/****************************************************************************************/
/* UART2 configuration                                                                  */
/****************************************************************************************/
// Define UART2 Pads
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
/* SPI configuration                                                                    */
/****************************************************************************************/
// Define SPI Pads
#if defined (__DA14531__)
    #define SPI_EN_PORT             GPIO_PORT_0
    #define SPI_EN_PIN              GPIO_PIN_1

    #define SPI_CLK_PORT            GPIO_PORT_0
    #define SPI_CLK_PIN             GPIO_PIN_4

    #define SPI_DO_PORT             GPIO_PORT_0
    #define SPI_DO_PIN              GPIO_PIN_0

    #define SPI_DI_PORT             GPIO_PORT_0
    #define SPI_DI_PIN              GPIO_PIN_3

#elif !defined (__DA14586__)
    #define SPI_EN_PORT             GPIO_PORT_0
    #define SPI_EN_PIN              GPIO_PIN_3

    #define SPI_CLK_PORT            GPIO_PORT_0
    #define SPI_CLK_PIN             GPIO_PIN_0

    #define SPI_DO_PORT             GPIO_PORT_0
    #define SPI_DO_PIN              GPIO_PIN_6

    #define SPI_DI_PORT             GPIO_PORT_0
    #define SPI_DI_PIN              GPIO_PIN_5
#endif

// Define SPI Configuration
    #define SPI_MS_MODE             SPI_MS_MODE_MASTER
    #define SPI_CP_MODE             SPI_CP_MODE_0
    #define SPI_WSZ                 SPI_MODE_8BIT
    #define SPI_CS                  SPI_CS_0

#if defined(__DA14531__)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
    #define SPI_EDGE_CAPTURE        SPI_MASTER_EDGE_CAPTURE
#else // (DA14585, DA14586)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
#endif

/****************************************************************************************/
/* SPI Flash configuration                                                              */
/****************************************************************************************/
#if !defined (__DA14586__)
#define SPI_FLASH_DEV_SIZE          (256 * 1024)
#endif

/****************************************************************************************/
/* I2C configuration                                                                    */
/****************************************************************************************/
// Define I2C Pads
#define I2C_SCL_PORT                GPIO_PORT_0
#define I2C_SCL_PIN                 GPIO_PIN_2

#define I2C_SDA_PORT                GPIO_PORT_0
#define I2C_SDA_PIN                 GPIO_PIN_1

// Define I2C Configuration
#define I2C_SLAVE_ADDRESS           (0x50)
#define I2C_SPEED_MODE              I2C_SPEED_FAST
#define I2C_ADDRESS_MODE            I2C_ADDRESSING_7B
#define I2C_ADDRESS_SIZE            I2C_2BYTES_ADDR

/****************************************************************************************/
/* I2C EEPROM configuration                                                             */
/****************************************************************************************/
#define I2C_EEPROM_DEV_SIZE         (0x20000)
#define I2C_EEPROM_PAGE_SIZE        (256)

/***************************************************************************************/
/* Production debug output configuration                                               */
/***************************************************************************************/
#if PRODUCTION_DEBUG_OUTPUT
#if defined (__DA14531__)
    #define PRODUCTION_DEBUG_PORT   GPIO_PORT_0
    #define PRODUCTION_DEBUG_PIN    GPIO_PIN_11
#else
     #define PRODUCTION_DEBUG_PORT   GPIO_PORT_2
     #define PRODUCTION_DEBUG_PIN    GPIO_PIN_5
#endif
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable pad and peripheral clocks assuming that peripheral power domain
 *        is down. The UART and SPI clocks are set.
 ****************************************************************************************
 */
void periph_init(void);

/**
 ****************************************************************************************
 * @brief Map port pins. The UART and SPI port pins and GPIO ports are mapped
 ****************************************************************************************
 */
void set_pad_functions(void);

/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 ****************************************************************************************
 */
void GPIO_reservations(void);

#endif // _USER_PERIPH_SETUP_H_
