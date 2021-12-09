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

#include "arch.h"
#include "gpio.h"
#include "uart.h"


/*
 * DEFINES
 ****************************************************************************************
 */


/****************************************************************************************/
/* UART1 configuration                                                                  */
/****************************************************************************************/
// Define UART1 Pads
#if defined (__DA14531__)
    #define UART1_TX_PORT           GPIO_PORT_0
    #define UART1_TX_PIN            GPIO_PIN_0

    #define UART1_RX_PORT           GPIO_PORT_0
    #define UART1_RX_PIN            GPIO_PIN_1

    #define UART1_RTSN_PORT         GPIO_PORT_0
    #define UART1_RTSN_PIN          GPIO_PIN_3

    #define UART1_CTSN_PORT         GPIO_PORT_0
    #define UART1_CTSN_PIN          GPIO_PIN_4
#else
    #define UART1_TX_PORT           GPIO_PORT_0
    #define UART1_TX_PIN            GPIO_PIN_4

    #define UART1_RX_PORT           GPIO_PORT_0
    #define UART1_RX_PIN            GPIO_PIN_5

    #define UART1_RTSN_PORT         GPIO_PORT_0
    #define UART1_RTSN_PIN          GPIO_PIN_7

    #define UART1_CTSN_PORT         GPIO_PORT_0
    #define UART1_CTSN_PIN          GPIO_PIN_6
#endif

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
/* UART2 configuration to use with arch_console print messages                          */
/****************************************************************************************/
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


/****************************************************************************************/
/* External CPU to DA1458x wake-up pin selection                                        */
/****************************************************************************************/
#if defined (CFG_EXTERNAL_WAKEUP)
/* Auto select the external GPIO wakeup signal according to the
   HCI_EIF_SELECT_PORT / HCI_EIF_SELECT_PIN configuration
*/
    #define EIF_WAKEUP_GPIO                             (1)

    #if EIF_WAKEUP_GPIO
        #if defined (CFG_HCI_BOTH_EIF)
            #define EXTERNAL_WAKEUP_GPIO_PORT                                           \
                    (GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)   \
                    ? UART1_CTSN_PORT : SPI_EN_PORT

            #define EXTERNAL_WAKEUP_GPIO_PIN                                            \
                    (GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)   \
                    ? UART1_CTSN_PIN : SPI_EN_PIN

            #define EXTERNAL_WAKEUP_GPIO_POLARITY                                       \
                    (GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)   \
                    ? 1 : 0
        #else
            #if defined (CFG_HCI_SPI) || defined (CFG_GTL_SPI)
                #define EXTERNAL_WAKEUP_GPIO_PORT       SPI_EN_PORT
                #define EXTERNAL_WAKEUP_GPIO_PIN        SPI_EN_PIN
                #define EXTERNAL_WAKEUP_GPIO_POLARITY   (0)
            #else // UART
                #define EXTERNAL_WAKEUP_GPIO_PORT       UART1_CTSN_PORT
                #define EXTERNAL_WAKEUP_GPIO_PIN        UART1_CTSN_PIN
                #define EXTERNAL_WAKEUP_GPIO_POLARITY   (1)
            #endif
        #endif
    #else
#if defined (__DA14531__)
        #define EXTERNAL_WAKEUP_GPIO_PORT               GPIO_PORT_0
        #define EXTERNAL_WAKEUP_GPIO_PIN                GPIO_PIN_9
#else
        #define EXTERNAL_WAKEUP_GPIO_PORT               GPIO_PORT_1
        #define EXTERNAL_WAKEUP_GPIO_PIN                GPIO_PIN_1
#endif
        #define EXTERNAL_WAKEUP_GPIO_POLARITY           (1)
    #endif
#endif

/****************************************************************************************/
/* DA1458x to external CPU wake-up pin selection                                        */
/****************************************************************************************/
#if defined (CFG_WAKEUP_EXT_PROCESSOR)
#if defined (__DA14531__)
    #define EXT_WAKEUP_PORT         GPIO_PORT_0
    #define EXT_WAKEUP_PIN          GPIO_PIN_7
#else
    #define EXT_WAKEUP_PORT         GPIO_PORT_1
    #define EXT_WAKEUP_PIN          GPIO_PIN_2
#endif
#endif



/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if DEVELOPMENT_DEBUG
/**
 ****************************************************************************************
 * @brief   Reserves application's specific GPIOs
 * @details Used only in Development mode (#if DEVELOPMENT_DEBUG)
 *          i.e. to reserve P0_1 as Generic Purpose I/O:
 *          RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);
 ****************************************************************************************
 */
void GPIO_reservations(void);
#endif

/**
 ****************************************************************************************
 * @brief   Sets the functionality of application pads
 * @details i.e. to set P0_1 as Generic purpose Output:
 *          GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_1, OUTPUT, PID_GPIO, false);
 ****************************************************************************************
 */
void set_pad_functions(void);

/**
 ****************************************************************************************
 * @brief   Initializes application's peripherals and pins
 ****************************************************************************************
 */
void periph_init(void);


#endif // _USER_PERIPH_SETUP_H_
