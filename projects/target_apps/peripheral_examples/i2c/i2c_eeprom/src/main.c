/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief I2C EEPROM example
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include "arch_system.h"
#include "user_periph_setup.h"
#include "gpio.h"
#include "uart.h"
#include "uart_utils.h"
#include "i2c_eeprom.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define BUFFER_SIZE             (256)

/*
 * VARIABLE DEFINITIONS
 ****************************************************************************************
 */

uint8_t wr_data[BUFFER_SIZE];
uint8_t rd_data[BUFFER_SIZE];

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief I2C test function
 ****************************************************************************************
 */
void i2c_test(void);

/**
 ****************************************************************************************
 * @brief Test write data.
 * @param[in] data              Pointer to the array of bytes to be written
 * @param[in] address           Starting address of the write process
 * @param[in] size              Size of the data to be written
 * @param[in|out] bytes_written Bytes that were actually written (due to memory size limitation)
 ****************************************************************************************
 */
static void test_write_data(uint8_t *data, uint32_t address, uint32_t size, uint32_t *bytes_written)
{
    printf_string(UART, "\n\rWriting page to EEPROM (values 0x00-FF)...\n\r");
    i2c_error_code code = i2c_eeprom_write_data(data, address, size, bytes_written);
    if (code == I2C_NO_ERROR)
    {
        printf_string(UART, "\n\rPage written.\n\r");
        printf_string(UART, "\n\rBytes written: 0x");
        printf_byte(UART, ((*bytes_written) >> 16) & 0xFF);
        printf_byte(UART, ((*bytes_written) >> 8) & 0xFF);
        printf_byte(UART, (*bytes_written) & 0xFF);
        printf_string(UART, "\n\r");
    }
    else
    {
        printf_string(UART, " - Test failed! - I2C Error code: 0x");
        printf_byte(UART, code & 0xFF);
        printf_string(UART, "\n\r");
    }
}

/**
 ****************************************************************************************
 * @brief Test write byte.
 * @param[in] address Memory position to write the byte to
 * @param[in] byte    Byte to be written
 ****************************************************************************************
 */
static void test_write_byte(uint32_t address, uint8_t byte)
{
    printf_string(UART, "\n\rWrite byte (0x");
    printf_byte(UART, byte & 0xFF);
    printf_string(UART, ") @ address ");
    printf_byte_dec(UART, address & 0xFF);
    printf_string(UART, " (zero based)...\n\r");
    i2c_error_code code = i2c_eeprom_write_byte(address, byte);
    if (code == I2C_NO_ERROR)
    {
        printf_string(UART, "\n\rWrite done.\n\r");
    }
    else
    {
        printf_string(UART, " - Test failed! - I2C Error code: 0x");
        printf_byte(UART, code & 0xFF);
        printf_string(UART, "\n\r");
    }
}

/**
 ****************************************************************************************
 * @brief Test read byte.
 * @param[in] address  Memory address to read the byte from
 * @param[in|out] byte Pointer to the read byte
 ****************************************************************************************
 */
static void test_read_byte(uint32_t address, uint8_t *byte)
{
    i2c_error_code code = i2c_eeprom_read_byte(address, byte);
    printf_string(UART, "\n\rRead byte @ address ");
    printf_byte_dec(UART, address & 0xFF);
    printf_string(UART, ": 0x");
    printf_byte(UART, (*byte) & 0xFF);
    printf_string(UART, "\n\r");
    if (code != I2C_NO_ERROR)
    {
        printf_string(UART, " - Test failed! - I2C Error code: 0x");
        printf_byte(UART, code & 0xFF);
        printf_string(UART, "\n\r");
    }
}

/**
 ****************************************************************************************
 * @brief Test read data.
 * @param[in] data           Pointer to the array of bytes to be read
 * @param[in] address        Starting memory address
 * @param[in] size           Size of the data to be read
 * @param[in|out] bytes_read Bytes that were actually read (due to memory size limitation)
 ****************************************************************************************
 */
static void test_read_data(uint8_t *data, uint32_t address, uint32_t size, uint32_t *bytes_read)
{
    i2c_error_code code = i2c_eeprom_read_data(data, address, size, bytes_read);
    if (code == I2C_NO_ERROR)
    {
        printf_string(UART, "\n\r");
        // Display Results
        for (uint32_t i = 0 ; i < size ; i++)
        {
            printf_byte(UART, data[i]);
            printf_string(UART, " ");
        }
        printf_string(UART, "\n\r\n\rBytes read: 0x");
        printf_byte(UART, ((*bytes_read) >> 16) & 0xFF);
        printf_byte(UART, ((*bytes_read) >> 8) & 0xFF);
        printf_byte(UART, (*bytes_read) & 0xFF);
        printf_string(UART, "\n\r");
    }
    else
    {
        printf_string(UART, " - Test failed! - I2C Error code: 0x");
        printf_byte(UART, code & 0xFF);
        printf_string(UART, "\n\r");
    }
}

/**
 ****************************************************************************************
 * @brief  Main routine of the I2C EEPROM example
 ****************************************************************************************
 */
int main (void)
{
    system_init();
    i2c_test();
    while(1);
}

void i2c_test(void)
{
    uint8_t byte;
    uint16_t i;
    uint32_t bytes_read = 0;
    uint32_t bytes_written = 0;
    uint16_t read_size = BUFFER_SIZE;  // the desired read size
    uint16_t write_size = BUFFER_SIZE; // the desired write size

    // Populate write vector
    for (i = 0 ; i < write_size ; i++)
    {
        wr_data[i] = i;
    }
    // Reset read vector
    for (i = 0 ; i < read_size ; i++)
    {
        rd_data[i] = 0;
    }

    printf_string(UART, "\n\r\n\r************");
    printf_string(UART, "\n\r* I2C TEST *\n\r");
    printf_string(UART, "************\n\r");


    // Page Write
    printf_string(UART, "\n\r*** Test 1 ***\n\r");
    test_write_data(wr_data, 0, write_size, &bytes_written);

    // Page Read
    printf_string(UART, "\n\r*** Test 2 ***\n\r");
    printf_string(UART, "\n\rReading EEPROM...\n\r");
    test_read_data(rd_data, 0, read_size, &bytes_read);

    // Setting 1 : 0x5A@22 (dec)
    // Write Byte
    printf_string(UART, "\n\r*** Test 3 ***\n\r");
    test_write_byte(22, 0x5A);

    // Random Read Byte
    printf_string(UART, "\n\r*** Test 4 ***\n\r");
    test_read_byte(22, &byte);

    // Setting 2 : 0x6A@00 (dec)
    // Write Byte
    printf_string(UART, "\n\r*** Test 5 ***\n\r");
    test_write_byte(0, 0x6A);

    // Random Read Byte
    printf_string(UART, "\n\r*** Test 6 ***\n\r");
    test_read_byte(0, &byte);

    // Setting 3 : 0x7A@255 (dec)
    // Write Byte
    printf_string(UART, "\n\r*** Test 7 ***\n\r");
    test_write_byte(255, 0x7A);

    // Random Read Byte
    printf_string(UART, "\n\r*** Test 8 ***\n\r");
    test_read_byte(255, &byte);

    // Setting 4 : 0xFF@30 (dec)
    // Write Byte
    printf_string(UART, "\n\r*** Test 9 ***\n\r");
    test_write_byte(30, 0xFF);

    // Random Read Byte
    printf_string(UART, "\n\r*** Test 10 ***\n\r");
    test_read_byte(30, &byte);

    // Page read for verification
    printf_string(UART, "\n\r*** Test 11 ***\n\r");
    printf_string(UART, "\n\rPage Read to verify that new bytes have been written correctly\n\r");
    printf_string(UART, "--------------------------------------------------------------\n\r");
    test_read_data(rd_data, 0, read_size, &bytes_read);

    i2c_eeprom_release();
    printf_string(UART, "\n\r");
    printf_string(UART, "\n\rEnd of test\n\r");
}
