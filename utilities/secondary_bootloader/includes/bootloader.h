/**
 ****************************************************************************************
 *
 * @file bootloader.h
 *
 * @brief firmware bootloader protocol settings
 *
 * Copyright (C) 2012-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _DOWNLOADER_H
#define _DOWNLOADER_H

#include <stdint.h>
#if defined(__DA14531__)
#include "gpio.h"
#endif

/************** Encrypted image support **************/
#define AES_ENCRYPTED_IMAGE_SUPPORTED 1

/************** 2-wire UART support **************/
#undef UART_SUPPORTED

/************** 1-wire UART support ************/
#if defined (__DA14531__)
#undef ONE_WIRE_UART_SUPPORTED
#endif

/************** SPI flash memory support **************/
#define SPI_FLASH_SUPPORTED

/************** I2C EEPROM memory support **************/
#undef I2C_EEPROM_SUPPORTED

#if  defined (SPI_FLASH_SUPPORTED ) && defined (I2C_EEPROM_SUPPORTED)
    #error "Both SPI_FLASH_SUPPORTED and I2C_EEPROM_SUPPORTED configuration flags are defined."
#endif 

/************** AN-B-001 based image / fast bootloader support **************/
// Define it if AN-B-001 based image is burned in SPI flash and the
// bootloader is running from the OTP
#undef SUPPORT_AN_B_001 // Enable it for the secondary bootloader

// Define ALLOW_NO_HEADER to enable loading 0x7f00 bytes from memory offset 0
// in case no valid AN-B-001 header is found.
#define ALLOW_NO_HEADER

#if defined (SUPPORT_AN_B_001) && (AES_ENCRYPTED_IMAGE_SUPPORTED == 1)
    #warning "Encryption is not supported by the secondary bootloader"
#endif

// boot header flags
#define IMAGE_BOOT_SIGNATURE1               0x70
#define IMAGE_BOOT_SIGNATURE2               0x50

#define AN001_SPI_HEADER_SIZE               8
#define AN001_SPI_STARTCODE_POSITION        8

#define BOOT_SPECIFIC                        (0xAA)

#if defined (__DA14531__)
#if defined (CFG_SPI_SET_DEFAULT_BOOTER_SPI_SETTINGS)
#define USE_SPI_SET_DEFAULT_BOOTER_SPI_SETTINGS (1)
#else
#define USE_SPI_SET_DEFAULT_BOOTER_SPI_SETTINGS (0)
#endif
#else
#define USE_SPI_SET_DEFAULT_BOOTER_SPI_SETTINGS (0)
#endif

#if defined (__DA14531__)
#if defined (CFG_ENTER_LOOP_WITH_DEBUGGER_ENABLED_ON_NO_APP)
#define USE_ENTER_LOOP_WITH_DEBUGGER_ENABLED_ON_NO_APP (1)
#else
#define USE_ENTER_LOOP_WITH_DEBUGGER_ENABLED_ON_NO_APP (0)
#endif
#else
#define USE_ENTER_LOOP_WITH_DEBUGGER_ENABLED_ON_NO_APP (0)
#endif

#if defined (__DA14531__)
#if defined (CFG_UART_BOOT_LOCK_OPTION)
#define USE_UART_BOOT_LOCK_OPTION (1)
#else
#define USE_UART_BOOT_LOCK_OPTION (0)
#endif
#else
#define USE_UART_BOOT_LOCK_OPTION (0)
#endif

#if defined (__DA14531__)
#if defined (CFG_SPI_BOOT_SPECIFIC_PORT_MAPPING)
#define USE_SPI_BOOT_SPECIFIC_PORT_MAPPING (1)
#else
#define USE_SPI_BOOT_SPECIFIC_PORT_MAPPING (0)
#endif
#else
#define USE_SPI_BOOT_SPECIFIC_PORT_MAPPING (0)
#endif

// 58x boot header: supports image sizes > 64kB
typedef struct __bootHeader58x_spi {
    uint8_t preamble[2]; // 0x70 0x50
    uint8_t empty[3];
    uint8_t len[3];
} s_bootHeader58x_spi;

#define AN001_EEPROM_HEADER_SIZE            8
#define AN001_EEPROM_STARTCODE_POSITION     32

// 58x: AN-B-001 header for I2C/EEPROM (sizes < 64kB)
typedef struct __bootHeader580_i2c {
    uint8_t preamble[2]; // 0x70 0x50
    uint8_t len[2];
    uint8_t crc;
    uint8_t dummy[27];
} s_bootHeader580_i2c;

// 585 boot header: supports image sizes > 64kB
typedef struct __bootHeader585_i2c{
    uint8_t preamble[2]; // 0x70 0x50
    uint8_t reserved[2]; // 0x0, 0x0
    uint8_t len[3];
    uint8_t crc;
    uint8_t dummy[24];
} s_bootHeader585_i2c;

/************** Dual Image bootloader section **************/
 
// product header flags 
#define PRODUCT_HEADER_POSITION     0x1F000
#define PRODUCT_HEADER_SIGNATURE1   0x70
#define PRODUCT_HEADER_SIGNATURE2   0x52
// image header flags 
#define IMAGE_HEADER_SIGNATURE1     0x70
#define IMAGE_HEADER_SIGNATURE2     0x51
#define CODE_OFFSET                 64
#define STATUS_INVALID_IMAGE        0x0
#define STATUS_VALID_IMAGE          0xAA
//product header structure
typedef struct __productHeader {
    uint8_t signature[2];
    uint8_t version[2];
    uint32_t offset1;
    uint32_t offset2;
}s_productHeader;
//image header structure
typedef struct __imageHeader {
    uint8_t signature[2];
    uint8_t validflag;
    uint8_t imageid;
    uint32_t code_size;
    uint32_t CRC;
    uint8_t version[16];
    uint32_t timestamp;
    uint8_t encryption;
    uint8_t reserved[31];
}s_imageHeader;

/************** Modified bootloader section**************/
typedef struct
{
#if (USE_UART_BOOT_LOCK_OPTION)
    bool is_uart_enabled;
#endif
    bool is_debugger_enabled;
#if (USE_SPI_BOOT_SPECIFIC_PORT_MAPPING)
    GPIO_PORT spi_en_port;
    GPIO_PORT spi_clk_port;
    GPIO_PORT spi_mosi_port;
    GPIO_PORT spi_miso_port;
    GPIO_PIN spi_en_pin;
    GPIO_PIN spi_clk_pin;
    GPIO_PIN spi_mosi_pin;
    GPIO_PIN spi_miso_pin;
#endif
} sb_conf_t;

// Functions Prototype
/**
****************************************************************************************
* @brief Load the active from a SPI flash
* @return Success (0) or Error Code.
* 
****************************************************************************************
**/
int spi_loadActiveImage(void);

/**
****************************************************************************************
* @brief Load the active from an EEPROM flash
* @return Success (0) or Error Code.
* 
****************************************************************************************
**/
int eeprom_loadActiveImage(void);

#endif
