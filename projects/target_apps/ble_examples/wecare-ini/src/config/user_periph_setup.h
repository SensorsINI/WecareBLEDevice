/**
 ****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
 *
 * Copyright (c) 2015-2019 Dialog Semiconductor. All rights reserved.
 *
 * This software ("Software") is owned by Dialog Semiconductor.
 *
 * By using this Software you agree that Dialog Semiconductor retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Dialog Semiconductor is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Dialog Semiconductor products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * DIALOG SEMICONDUCTOR BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
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
#include "DAC70508M.h"
#include "MCR35614R.h"



/*
 * DEFINES
 ****************************************************************************************
 */

/****************************************************************************************/
/* UART2 configuration                                                                  */
/****************************************************************************************/
// Define UART2 Tx Pad
#if defined (__DA14531__)
    #define UART1_TX_PORT           GPIO_PORT_0
    #define UART1_TX_PIN            GPIO_PIN_5
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
/* BAT configuration                                                                  */
/****************************************************************************************/
#define USE_BAT_LEVEL_ALERT         0

#if defined (__DA14531__)
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_0
    // #define GPIO_ALERT_LED_PIN      GPIO_PIN_9        // The original pin is PIN 9, but used for SPI
    #define GPIO_BAT_LED_PORT       GPIO_PORT_0
    // #define GPIO_BAT_LED_PIN        GPIO_PIN_8
#else
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_1
    #define GPIO_ALERT_LED_PIN      GPIO_PIN_0
    #define GPIO_BAT_LED_PORT       GPIO_PORT_1
    #define GPIO_BAT_LED_PIN        GPIO_PIN_2
#endif

/****************************************************************************************/
/* LED configuration                                                                    */
/****************************************************************************************/
#if defined (__DA14531__)
    #define GPIO_LED_PORT           GPIO_PORT_0
    // #define GPIO_LED_PIN            GPIO_PIN_9      // The original pin is PIN 9, but used for SPI
#else
    #define GPIO_LED_PORT           GPIO_PORT_1
    #define GPIO_LED_PIN            GPIO_PIN_0
#endif

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

#if defined (__DA14531__)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
    #define SPI_EDGE_CAPTURE        SPI_MASTER_EDGE_CAPTURE
#else // (DA14585, DA14586)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
#endif


/****************************************************************************************/
/* SPI Flash configuration                                                              */
/****************************************************************************************/
#if !defined (__DA14586__)
#define SPI_FLASH_DEV_SIZE          (128 * 1024)
#endif

#if defined (CFG_SPI_FLASH_ENABLE)
// Configuration struct for SPI
static const spi_cfg_t spi_cfg = {
    .spi_ms = SPI_MS_MODE,
    .spi_cp = SPI_CP_MODE,
    .spi_speed = SPI_SPEED_MODE,
    .spi_wsz = SPI_WSZ,
    .spi_cs = SPI_CS,
    .cs_pad.port = SPI_EN_PORT,
    .cs_pad.pin = SPI_EN_PIN,
#if defined (__DA14531__)
    .spi_capture = SPI_EDGE_CAPTURE,
#endif
};

// Configuration struct for SPI FLASH
static const spi_flash_cfg_t spi_flash_cfg = {
    .chip_size = SPI_FLASH_DEV_SIZE,
};
#endif

/****************************************************************************************/
/* SPI2 configuration (for IO, DAC and ADCs. MOSI, MISO and SCK are shared together)                                                                   */
/****************************************************************************************/
// Define SPI2 Pads
#if defined (__DA14531__)
    #define SPI2_CLK_PORT           GPIO_PORT_0
    #define SPI2_CLK_PIN            GPIO_PIN_11  

    #define SPI2_DO_PORT            GPIO_PORT_0
    #define SPI2_DO_PIN             GPIO_PIN_0  

    #define SPI2_DI_PORT            GPIO_PORT_0
    #define SPI2_DI_PIN             GPIO_PIN_6  

// SPI IO Max7317 CS
		#define SPI2_IO_CS_PORT            GPIO_PORT_0
    #define SPI2_IO_CS_PIN             GPIO_PIN_9  

// Define extra SPI chip select signals: 1 DAC and 2 ADCs
    #define SPI2_ADC1_CS_PORT            GPIO_PORT_0
    #define SPI2_ADC1_CS_PIN             GPIO_PIN_8  

    #define SPI2_ADC2_CS_PORT            GPIO_PORT_0
    #define SPI2_ADC2_CS_PIN             GPIO_PIN_7  
		
    #define SPI2_DAC_CS_PORT            GPIO_PORT_0
    #define SPI2_DAC_CS_PIN             GPIO_PIN_10  
#endif


// Initialize SPI2 IO driver
static spi_cfg_t spi2_cfg = {  .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_0,            // SPI Mode 0,0
                        .spi_speed = SPI_SPEED_MODE_2MHz,
                        .spi_wsz = SPI_MODE_16BIT,
                        .spi_cs = SPI_CS_0,                 
                        .cs_pad.port = SPI2_IO_CS_PORT,
                        .cs_pad.pin = SPI2_IO_CS_PIN
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};

// Initialize SPI2 DAC driver
static spi_cfg_t spi2_dac_cfg = {  
	                      .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_1,            // SPI Mode 0,1
                        .spi_speed = SPI_SPEED_MODE_2MHz,
                        .spi_wsz = SPI_MODE_32BIT,
                        .spi_cs = SPI_CS_0,                
                        .cs_pad.port = SPI2_DAC_CS_PORT,
                        .cs_pad.pin = SPI2_DAC_CS_PIN
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};

// Initialize SPI2 ADC1 driver
static spi_cfg_t spi2_adc1_cfg = {  
	                      .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_0,            // SPI Mode 0,0
                        .spi_speed = SPI_SPEED_MODE_2MHz,
                        .spi_wsz = SPI_MODE_8BIT,
                        .spi_cs = SPI_CS_1,                 
                        .cs_pad.port = SPI2_ADC1_CS_PORT,
                        .cs_pad.pin = SPI2_ADC1_CS_PIN
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};

// Initialize SPI2 ADC2 driver
static spi_cfg_t spi2_adc2_cfg = {  
	                      .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_0,            // SPI Mode 0,0
                        .spi_speed = SPI_SPEED_MODE_2MHz,
                        .spi_wsz = SPI_MODE_8BIT,
                        .spi_cs = SPI_CS_0,                 
                        .cs_pad.port = SPI2_ADC2_CS_PORT,
                        .cs_pad.pin = SPI2_ADC2_CS_PIN		
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};


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

#if DEVELOPMENT_DEBUG
/**
 ****************************************************************************************
 * @brief   Reserves application's specific GPIOs
 * @details Used only in Development mode (#if DEVELOPMENT_DEBUG)
 *          i.e. to reserve P0_1 as Generic Purpose I/O:
 *          RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);
 * @return  void
 ****************************************************************************************
 */
void GPIO_reservations(void);
#endif

/**
 ****************************************************************************************
 * @brief   Sets the functionality of application pads
 * @details i.e. to set P0_1 as Generic purpose Output:
 *          GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_1, OUTPUT, PID_GPIO, false);
 * @return  void
 ****************************************************************************************
 */
void set_pad_functions(void);

/**
 ****************************************************************************************
 * @brief   Initializes application's peripherals and pins
 * @return  void
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
#endif // _USER_PERIPH_SETUP_H_

