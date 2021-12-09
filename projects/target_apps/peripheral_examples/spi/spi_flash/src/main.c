/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief SPI flash example
 *
 * Copyright (C) 2012-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include "arch_system.h"
#include "user_periph_setup.h"
#include "gpio.h"
#include "uart.h"
#include "uart_utils.h"
#include "spi.h"
#include "spi_flash.h"

uint8_t rd_data[512];
uint8_t wr_data[512];
uint8_t dev_id;

/**
 ****************************************************************************************
 * @brief SPI Flash test function
 ****************************************************************************************
 */
void spi_flash_test(void);

/**
 ****************************************************************************************
 * @brief Main routine of the SPI flash example
 ****************************************************************************************
 */
int main (void)
{
    system_init();
    spi_flash_test();
    while(1);
}

/**
 ****************************************************************************************
 * @brief SPI and SPI Flash Initialization function
 ****************************************************************************************
 */
static void spi_flash_peripheral_init(void)
{
    // Release the SPI flash memory from power down
    spi_flash_release_from_power_down();

    // Disable the SPI flash memory protection (unprotect all sectors)
    spi_flash_configure_memory_protection(SPI_FLASH_MEM_PROT_NONE);

    // Try to auto-detect the SPI flash memory
    spi_flash_auto_detect(&dev_id);
}

/**
 ****************************************************************************************
 * @brief SPI Flash protection features test
 ****************************************************************************************
 */
static void spi_protection_features_test(void)
{
    unsigned char buf1[][2] = {{0xE0, 0x0E}, {0xD0, 0x0D}, {0xB0, 0x0B}, {0x70, 0x07}};
    uint32_t actual_size;
    printf_string(UART, "\n\r\n\r");
    switch(dev_id)
    {
        case W25X10CL_DEV_INDEX:
        {
            printf_string(UART, "\n\rW25X10CL SPI memory protection features demo.\n\r");

            // 1. Unprotected memory
            printf_string(UART, "\n\r1) Unprotected memory test\n\r");
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x10000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xFF \t [0x10000] = 0xFF\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xE0 \t [0x10000] = 0x0E\n\r");
            spi_flash_write_data(&buf1[0][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[0][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0x0E\n\r");

            // 2. Full protected memory
            printf_string(UART, "\n\r2) Full protected memory test\n\r");
            spi_flash_configure_memory_protection(W25X10CL_MEM_PROT_ALL);
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase();

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x10000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0x0E\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xD0 \t [0x10000] = 0x0D\n\r");
            spi_flash_write_data(&buf1[1][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[1][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0x0E\n\r");

            // 3. Lower half protected memory
            printf_string(UART, "\n\r3) Lower half protected memory test\n\r");
            spi_flash_configure_memory_protection(W25X10CL_MEM_PROT_LOWER_HALF);
            printf_string(UART, "\tPerforming a Sector Erase at addresses 0x00000 and 0x10000...\n\r");
            spi_flash_block_erase(0, SPI_FLASH_OP_SE);
            spi_flash_block_erase(0x10000, SPI_FLASH_OP_SE);

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x10000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0xFF\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xB0 \t [0x10000] = 0x0B\n\r");
            spi_flash_write_data(&buf1[2][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[2][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0x0B\n\r");

            // 4. Upper half protected memory
            printf_string(UART, "\n\r4) Upper half protected memory test\n\r");
            spi_flash_configure_memory_protection(W25X10CL_MEM_PROT_UPPER_HALF);
            printf_string(UART, "\tPerforming a Sector Erase at addresses 0x00000 and 0x10000...\n\r");
            spi_flash_block_erase(0, SPI_FLASH_OP_SE);
            spi_flash_block_erase(0x10000, SPI_FLASH_OP_SE);

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x10000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xFF \t [0x10000] = 0x0B\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0x70 \t [0x10000] = 0x07\n\r");
            spi_flash_write_data(&buf1[3][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[3][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0x70 \t [0x10000] = 0x0B\n\r");

            break;
        }

        case W25X20CL_DEV_INDEX:
        {
            printf_string(UART, "\n\rW25X20CL SPI memory protection features demo.\n\r");

            // 1. Unprotected memory
            printf_string(UART, "\n\r1) Unprotected memory test\n\r");
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x20000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xFF \t [0x20000] = 0xFF\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xE0 \t [0x20000] = 0x0E\n\r");
            spi_flash_write_data(&buf1[0][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[0][1], 0x20000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0x0E\n\r");

            // 2. Full protected memory
            printf_string(UART, "\n\r2) Full protected memory test\n\r");
            spi_flash_configure_memory_protection(W25X20CL_MEM_PROT_ALL);
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase();

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x20000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0x0E\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xD0 \t [0x20000] = 0x0D\n\r");
            spi_flash_write_data(&buf1[1][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[1][1], 0x20000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0x0E\n\r");

            // 3. Lower half protected memory
            printf_string(UART, "\n\r3) Lower half protected memory test\n\r");
            spi_flash_configure_memory_protection(W25X20CL_MEM_PROT_LOWER_HALF);
            printf_string(UART, "\tPerforming a Sector Erase at addresses 0x00000 and 0x20000...\n\r");
            spi_flash_block_erase(0, SPI_FLASH_OP_SE);
            spi_flash_block_erase(0x20000, SPI_FLASH_OP_SE);

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x20000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0xFF\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xB0 \t [0x20000] = 0x0B\n\r");
            spi_flash_write_data(&buf1[2][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[2][1], 0x20000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0x0B\n\r");

            // 4. Upper half protected memory
            printf_string(UART, "\n\r4) Upper half protected memory test\n\r");
            spi_flash_configure_memory_protection(W25X20CL_MEM_PROT_UPPER_HALF);
            printf_string(UART, "\tPerforming a Sector Erase at addresses 0x00000 and 0x20000...\n\r");
            spi_flash_block_erase(0, SPI_FLASH_OP_SE);
            spi_flash_block_erase(0x20000, SPI_FLASH_OP_SE);

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x20000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xFF \t [0x20000] = 0x0B\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0x70 \t [0x20000] = 0x07\n\r");
            spi_flash_write_data(&buf1[3][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[3][1], 0x20000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0x70 \t [0x20000] = 0x0B\n\r");

            break;
        }

        case AT25DX011_DEV_INDEX:
        {
            printf_string(UART, "\n\rAT25Dx011 SPI memory protection features demo.");

            // 1. Unprotected memory
            printf_string(UART, "\n\r1) Unprotected memory test\n\r");
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function

            printf_string(UART, "\n\r2) Retrieving the two bytes at addresses 0x00000 and 0x10000");
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xFF and [0x10000] = 0xFF, as the memory has been cleared)");

            printf_string(UART, "\n\r3) Writing [0x00000]<- 0xE0 and [0x10000]<- 0x0E to the unprotected memory");
            spi_flash_write_data(&buf1[0][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[0][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xE0 and [0x10000] = 0x0E)");

            // 2. Full protected memory
            printf_string(UART, "\n\r4) Enabling memory protection for the whole memory array.");

            spi_flash_configure_memory_protection(AT25DX011_MEM_PROT_ALL);
            printf_string(UART, "\n\r5) Writing [0x00000]<- 0xD0 and [0x10000]<- 0x0D to the fully protected memory");
            spi_flash_write_data(&buf1[1][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[1][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xE0 and [0x10000] = 0x0E), the old values)");

            printf_string(UART, "\n\r6) Unprotecting the whole memory array and doing a full erase");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function
            break;
        }

        case P25Q10U_DEV_INDEX:
        {
            printf_string(UART, "\n\rP25Q10U SPI memory protection features demo.\n\r");

            // 1. Unprotected memory
            printf_string(UART, "\n\r1) Unprotected memory test\n\r");
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x10000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xFF \t [0x10000] = 0xFF\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xE0 \t [0x10000] = 0x0E\n\r");
            spi_flash_write_data(&buf1[0][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[0][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0x0E\n\r");

            // 2. Full protected memory
            printf_string(UART, "\n\r2) Full protected memory test\n\r");
            spi_flash_configure_memory_protection(P25Q10U_MEM_PROT_ALL);
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase();

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x10000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0x0E\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xD0 \t [0x10000] = 0x0D\n\r");
            spi_flash_write_data(&buf1[1][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[1][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0x0E\n\r");


            // 3. Lower half protected memory
            printf_string(UART, "\n\r3) Lower half protected memory test\n\r");
            spi_flash_configure_memory_protection(P25Q10U_MEM_PROT_LOWER_HALF);
            printf_string(UART, "\tPerforming a Sector Erase at addresses 0x00000 and 0x10000...\n\r");
            spi_flash_block_erase(0, SPI_FLASH_OP_SE);
            spi_flash_block_erase(0x10000, SPI_FLASH_OP_SE);

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x10000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0xFF\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xB0 \t [0x10000] = 0x0B\n\r");
            spi_flash_write_data(&buf1[2][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[2][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x10000] = 0x0B\n\r");

            // 4. Upper half protected memory
            printf_string(UART, "\n\r4) Upper half protected memory test\n\r");
            spi_flash_configure_memory_protection(P25Q10U_MEM_PROT_UPPER_HALF);
            printf_string(UART, "\tPerforming a Sector Erase at addresses 0x00000 and 0x10000...\n\r");
            spi_flash_block_erase(0, SPI_FLASH_OP_SE);
            spi_flash_block_erase(0x10000, SPI_FLASH_OP_SE);

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x10000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xFF \t [0x10000] = 0x0B\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0x70 \t [0x10000] = 0x07\n\r");
            spi_flash_write_data(&buf1[3][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[3][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0x70 \t [0x10000] = 0x0B\n\r");

            break;
        }

        case GD25WD20_DEV_INDEX:
        {
            printf_string(UART, "\n\rGD25WD20 SPI memory protection features demo.\n\r");

            // 1. Unprotected memory
            printf_string(UART, "\n\r1) Unprotected memory test\n\r");
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x20000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xFF \t [0x20000] = 0xFF\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xE0 \t [0x20000] = 0x0E\n\r");
            spi_flash_write_data(&buf1[0][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[0][1], 0x20000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0x0E\n\r");

            // 2. Full protected memory
            printf_string(UART, "\n\r2) Full protected memory test\n\r");
            spi_flash_configure_memory_protection(GD25WD20_MEM_PROT_ALL);
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase();

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x20000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0x0E\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xD0 \t [0x20000] = 0x0D\n\r");
            spi_flash_write_data(&buf1[1][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[1][1], 0x20000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0x0E\n\r");

            // 3. Lower half protected memory
            printf_string(UART, "\n\r3) Lower half protected memory test\n\r");
            spi_flash_configure_memory_protection(GD25WD20_MEM_PROT_LOWER_HALF);
            printf_string(UART, "\tPerforming a Sector Erase at addresses 0x00000 and 0x20000...\n\r");
            spi_flash_block_erase(0, SPI_FLASH_OP_SE);
            spi_flash_block_erase(0x20000, SPI_FLASH_OP_SE);

            printf_string(UART, "\tRetrieving the two bytes at addresses 0x00000 and 0x20000\n\r");
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0xFF\n\r");

            printf_string(UART, "\t Write:    [0x00000] = 0xB0 \t [0x20000] = 0x0B\n\r");
            spi_flash_write_data(&buf1[2][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[2][1], 0x20000, 1, &actual_size);
            printf_string(UART, "\t Read:     [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " \t [0x20000] = 0x");
            spi_flash_read_data(rd_data, 0x20000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r\t Expected: [0x00000] = 0xE0 \t [0x20000] = 0x0B\n\r");

            break;
        }

        case AT25DF021A_DEV_INDEX:
        {
            printf_string(UART, "\n\rAT25DF021A SPI memory protection features demo.");

            // 1. Unprotected memory
            printf_string(UART, "\n\r1) Unprotected memory test\n\r");
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function

            printf_string(UART, "\n\r2) Retrieving the two bytes at addresses 0x00000 and 0x10000");
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xFF and [0x10000] = 0xFF, as the memory has been cleared)");

            printf_string(UART, "\n\r3) Writing [0x00000]<- 0xE0 and [0x10000]<- 0x0E to the unprotected memory");
            spi_flash_write_data(&buf1[0][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[0][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xE0 and [0x10000] = 0x0E)");

            // 2. Full protected memory
            printf_string(UART, "\n\r4) Enabling memory protection for the whole memory array.");

            spi_flash_configure_memory_protection(AT25DF021A_MEM_PROT_ALL);
            printf_string(UART, "\n\r5) Writing [0x00000]<- 0xD0 and [0x10000]<- 0x0D to the fully protected memory");
            spi_flash_write_data(&buf1[1][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[1][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xE0 and [0x10000] = 0x0E), the old values)");

            printf_string(UART, "\n\r6) Unprotecting the whole memory array and doing a full erase");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function
            break;
        }

        case AT25EU0021A_DEV_INDEX:
        {
            printf_string(UART, "\n\rAT25EU0021A SPI memory protection features demo.");

            // 1. Unprotected memory
            printf_string(UART, "\n\r1) Unprotected memory test\n\r");
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function

            printf_string(UART, "\n\r2) Retrieving the two bytes at addresses 0x00000 and 0x10000");
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xFF and [0x10000] = 0xFF, as the memory has been cleared)");

            printf_string(UART, "\n\r3) Writing [0x00000]<- 0xE0 and [0x10000]<- 0x0E to the unprotected memory");
            spi_flash_write_data(&buf1[0][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[0][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xE0 and [0x10000] = 0x0E)");

            // 2. Full protected memory
            printf_string(UART, "\n\r4) Enabling memory protection for the whole memory array.");

            spi_flash_configure_memory_protection(AT25EU0021A_MEM_PROT_ALL);
            printf_string(UART, "\n\r5) Writing [0x00000]<- 0xD0 and [0x10000]<- 0x0D to the fully protected memory");
            spi_flash_write_data(&buf1[1][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[1][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xE0 and [0x10000] = 0x0E), the old values)");

            printf_string(UART, "\n\r6) Unprotecting the whole memory array and doing a full erase");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function
            break;
        }

        case AT25XE041D_DEV_INDEX:
        {
            printf_string(UART, "\n\rAT25XE041D SPI memory protection features demo.");

            // 1. Unprotected memory
            printf_string(UART, "\n\r1) Unprotected memory test\n\r");
            printf_string(UART, "\tPerforming a full erase...\n\r");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function

            printf_string(UART, "\n\r2) Retrieving the two bytes at addresses 0x00000 and 0x10000");
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xFF and [0x10000] = 0xFF, as the memory has been cleared)");

            printf_string(UART, "\n\r3) Writing [0x00000]<- 0xE0 and [0x10000]<- 0x0E to the unprotected memory");
            spi_flash_write_data(&buf1[0][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[0][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xE0 and [0x10000] = 0x0E)");

            // 2. Full protected memory
            printf_string(UART, "\n\r4) Enabling memory protection for the whole memory array.");

            spi_flash_configure_memory_protection(AT25XE041D_MEM_PROT_ALL);
            printf_string(UART, "\n\r5) Writing [0x00000]<- 0xD0 and [0x10000]<- 0x0D to the fully protected memory");
            spi_flash_write_data(&buf1[1][0], 0, 1, &actual_size);
            spi_flash_write_data(&buf1[1][1], 0x10000, 1, &actual_size);
            printf_string(UART, "\n\r   Reading [0x00000] = 0x");
            spi_flash_read_data(rd_data, 0, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, " and [0x10000] = 0x");
            spi_flash_read_data(rd_data, 0x10000, 1, &actual_size);
            printf_byte(UART, rd_data[0]);
            printf_string(UART, "\n\r  (must be [0x00000] = 0xE0 and [0x10000] = 0x0E), the old values)");

            printf_string(UART, "\n\r6) Unprotecting the whole memory array and doing a full erase");
            spi_flash_chip_erase_forced();  // the whole memory array gets unprotected in this function
            break;
        }

        default:
            printf_string(UART, "\n\r Not known SPI flash memory device detected");
    }
}

/**
 ****************************************************************************************
 * @brief SPI Flash power mode for Macronix test
 ****************************************************************************************
 */
static void print_MX25R_power_mode(void)
{
    uint32_t power_mode;
    // Get the power mode
    spi_flash_get_power_mode(&power_mode);
    printf_string(UART, "\n\r\n\rMX25R Power Mode: ");
    if ((power_mode & SPI_FLASH_PM_HPM)== SPI_FLASH_PM_HPM)
        printf_string(UART, " High Performance Mode ");
    else
        printf_string(UART, " Low Power Mode ");
}

/**
 ****************************************************************************************
 * @brief SPI Flash Read/Write test
 ****************************************************************************************
 */
static void spi_read_write_test(void)
{
    uint32_t write_size = 512; // set this variable to the desired read size
    uint32_t read_size = 256;  // set this variable to the desired read size
    uint32_t actual_size;
    uint32_t i;

    printf_string(UART, "\r\n\r\n\r\nSPI Flash Erase/Program/Read test...");
    spi_flash_block_erase(0, SPI_FLASH_OP_SE);

    // Program Page (256 bytes)
    printf_string(UART, "\n\r\n\rPerforming Program Page...");
    spi_flash_page_program(wr_data, 0, 256);
    printf_string(UART, " Page programmed.");

    // Read SPI Flash first 256 bytes
    printf_string(UART, "\r\n\r\nReading SPI Flash first 256 bytes...\r\n");
    spi_flash_read_data(rd_data, 0, read_size, &actual_size);
    // Display Results
    for (i = 0; i < read_size; i++)
    {
        printf_byte(UART, rd_data[i]);
        printf_string(UART, " ");
    }
    printf_string(UART, "\r\nBytes Read: 0x");
    printf_byte(UART, (actual_size >> 8) & 0xFF);
    printf_byte(UART, (actual_size) & 0xFF);

    printf_string(UART, "\r\n\r\nPerforming Sector Erase...");
    spi_flash_block_erase(0, SPI_FLASH_OP_SE);
    printf_string(UART, "Sector erased.");

    // Write data example (512 bytes)
    printf_string(UART, "\r\n\r\nPerforming 512 byte write...");
    spi_flash_write_data(wr_data, 0, 512, &actual_size);
    printf_string(UART, "Data written.");

    // Read SPI Flash first 512 bytes
    printf_string(UART, "\r\n\r\nReading SPI Flash first 512 bytes...");
    spi_flash_read_data(rd_data, 0, write_size, &actual_size);
    // Display Results
    for (i = 0; i < write_size; i++)
    {
        printf_byte(UART, rd_data[i]);
        printf_string(UART, " ");
    }
    printf_string(UART, "\r\nBytes Read: 0x");
    printf_byte(UART, (actual_size >> 8) & 0xFF);
    printf_byte(UART, (actual_size) & 0xFF);
}

/**
 ****************************************************************************************
 * @brief SPI Flash Read/Write test
 ****************************************************************************************
 */
static void spi_read_write_test_buffer(void)
{
    uint32_t write_size = 512; // set this variable to the desired read size
    uint32_t read_size = 256;  // set this variable to the desired read size
    uint32_t actual_size;
    uint32_t i;

    printf_string(UART, "\r\n\r\n\r\nSPI Flash Erase/Program/Read Buffer test...");
    spi_flash_block_erase(0, SPI_FLASH_OP_SE);

    // Program Page (256 bytes)
    printf_string(UART, "\n\r\n\rPerforming Program Page...");
    spi_flash_page_program_buffer(wr_data, 0, 256);
    printf_string(UART, " Page programmed.");

    // Read SPI Flash first 256 bytes
    printf_string(UART, "\r\n\r\nReading SPI Flash first 256 bytes...\r\n");
    spi_flash_read_data_buffer(rd_data, 0, read_size, &actual_size);
    // Display Results
    for (i = 0; i < read_size; i++)
    {
        printf_byte(UART, rd_data[i]);
        printf_string(UART, " ");
    }
    printf_string(UART, "\r\nBytes Read: 0x");
    printf_byte(UART, (actual_size >> 8) & 0xFF);
    printf_byte(UART, (actual_size) & 0xFF);

    printf_string(UART, "\r\n\r\nPerforming Sector Erase...");
    spi_flash_block_erase(0, SPI_FLASH_OP_SE);
    printf_string(UART, "Sector erased.");

    // Write data example (512 bytes)
    printf_string(UART, "\r\n\r\nPerforming 512 byte write...");
    spi_flash_write_data_buffer(wr_data, 0, 512, &actual_size);
    printf_string(UART, "Data written.");

    // Read SPI Flash first 512 bytes
    printf_string(UART, "\r\n\r\nReading SPI Flash first 512 bytes...");
    spi_flash_read_data_buffer(rd_data, 0, write_size, &actual_size);
    // Display Results
    for (i = 0; i < write_size; i++)
    {
        printf_byte(UART, rd_data[i]);
        printf_string(UART, " ");
    }
    printf_string(UART, "\r\nBytes Read: 0x");
    printf_byte(UART, (actual_size >> 8) & 0xFF);
    printf_byte(UART, (actual_size) & 0xFF);
}

#if defined (CFG_SPI_DMA_SUPPORT)
static void spi_read_write_test_dma(void)
{
    uint32_t write_size = 512; // set this variable to the desired read size
    uint32_t read_size = 256;  // set this variable to the desired read size
    uint32_t actual_size;
    uint32_t i;

    printf_string(UART, "\r\n\r\n\r\nSPI Flash Erase/Program/Read DMA test...");
    spi_flash_block_erase(0, SPI_FLASH_OP_SE);

    // Program Page (256 bytes)
    printf_string(UART, "\n\r\n\rPerforming Program Page...");
    spi_flash_page_program_dma(wr_data, 0, 256);
    printf_string(UART, " Page programmed.");

    // Read SPI Flash first 256 bytes
    printf_string(UART, "\r\n\r\nReading SPI Flash first 256 bytes...\r\n");
    spi_flash_read_data_dma(rd_data, 0, read_size, &actual_size);
    // Display Results
    for (i = 0; i < read_size; i++)
    {
        printf_byte(UART, rd_data[i]);
        printf_string(UART, " ");
    }
    printf_string(UART, "\r\nBytes Read: 0x");
    printf_byte(UART, (actual_size >> 8) & 0xFF);
    printf_byte(UART, (actual_size) & 0xFF);

    printf_string(UART, "\r\n\r\nPerforming Sector Erase...");
    spi_flash_block_erase(0, SPI_FLASH_OP_SE);
    printf_string(UART, "Sector erased.");

    // Write data example (512 bytes)
    printf_string(UART, "\r\n\r\nPerforming 512 byte write...");
    spi_flash_write_data_dma(wr_data, 0, 512, &actual_size);
    printf_string(UART, "Data written.");

    // Read SPI Flash first 512 bytes
    printf_string(UART, "\r\n\r\nReading SPI Flash first 512 bytes...");
    spi_flash_read_data_dma(rd_data, 0, write_size, &actual_size);
    // Display Results
    for (i = 0; i < write_size; i++)
    {
        printf_byte(UART, rd_data[i]);
        printf_string(UART, " ");
    }
    printf_string(UART, "\r\nBytes Read: 0x");
    printf_byte(UART, (actual_size >> 8) & 0xFF);
    printf_byte(UART, (actual_size) & 0xFF);
}
#endif

void spi_flash_test(void)
{
    uint16_t man_dev_id;
    uint64_t unique_id;
    uint32_t jedec_id;
    uint32_t actual_size;
    uint32_t i;

    uint16_t read_size = 256;  // set this variable to the desired read size
    wr_data[0] = 0;

    for (i = 1; i < 512; i++)
    {
        wr_data[i] = wr_data[i-1] + 1;
    }

    printf_string(UART, "\n\r******************");
    printf_string(UART, "\n\r* SPI FLASH TEST *");
    printf_string(UART, "\n\r******************\n\r");

    // Enable FLASH and SPI
    spi_flash_peripheral_init();
    // spi_flash_chip_erase();
    // Read SPI Flash Manufacturer/Device ID
    spi_flash_read_rems_id(&man_dev_id);

    spi_cs_low();

    spi_cs_high();

    // Erase flash
    spi_flash_chip_erase_forced();

    // Read SPI Flash first 256 bytes
    printf_string(UART, "\n\r\n\rReading SPI Flash first 256 bytes...");
    spi_flash_read_data(rd_data, 0, read_size, &actual_size);
    // Display Results
    for (i = 0; i < read_size ; i++)
    {
        printf_byte(UART, rd_data[i]);
        printf_string(UART, " ");
    }
    printf_string(UART, "\n\r\n\rBytes Read: 0x");
    printf_byte(UART, (actual_size >> 8) & 0xFF);
    printf_byte(UART, (actual_size) & 0xFF);
    printf_string(UART, "\n\r");

    // Read SPI Flash JEDEC ID
    spi_flash_read_jedec_id(&jedec_id);
    printf_string(UART, "\n\rSPI flash JEDEC ID is ");
    printf_byte(UART, (jedec_id >> 16) & 0xFF);
    printf_byte(UART, (jedec_id >> 8) & 0xFF);
    printf_byte(UART, (jedec_id) & 0xFF);

    switch (dev_id)
    {
        case W25X10CL_DEV_INDEX:
            printf_string(UART, "\n\rYou are using W25X10 (1-MBit) SPI flash device.");
        break;

        case W25X20CL_DEV_INDEX:
            printf_string(UART, "\n\rYou are using W25X20 (2-MBit) SPI flash device.");
        break;

        case AT25DX011_DEV_INDEX:
            printf_string(UART, "\n\rYou are using AT25DF011 or AT25DN011 (1-MBit) SPI flash device.");
        break;

        case MX25V1006E_DEV_INDEX:
            printf_string(UART, "\n\rYou are using MX25V1006E 1-Mbit SPI flash device.");
        break;

        case MX25R1035F_DEV_INDEX:
            printf_string(UART, "\n\rYou are using MX25R1035F 1-Mbit SPI flash device.");
        break;

        case MX25R2035F_DEV_INDEX:
            printf_string(UART, "\n\rYou are using MX25R2035F 2-Mbit SPI flash device.");
        break;

        case P25Q10U_DEV_INDEX:
            printf_string(UART, "\n\rYou are using P25Q10U 1-Mbit SPI flash device.");
        break;

        case GD25WD20_DEV_INDEX:
            printf_string(UART, "\n\rYou are using GD25WD20 2-Mbit SPI flash device.");
        break;

        case AT25DF021A_DEV_INDEX:
            printf_string(UART, "\n\rYou are using AT25DF021A 2-Mbit SPI flash device.");
        break;

        case AT25EU0021A_DEV_INDEX:
            printf_string(UART, "\n\rYou are using AT25EU0021A 2-Mbit SPI flash device.");
        break;

        case AT25XE041D_DEV_INDEX:
            printf_string(UART, "\n\rYou are using AT25XE041D 4-Mbit SPI flash device.");
        break;

        default:
            printf_string(UART, "\n\rNo SPI flash device detected");
        break;
    }

    if ((dev_id == W25X10CL_DEV_INDEX)    ||
        (dev_id == W25X20CL_DEV_INDEX)    ||
        (dev_id == P25Q10U_DEV_INDEX)     ||
        (dev_id == GD25WD20_DEV_INDEX)    ||
        (dev_id == AT25DF021A_DEV_INDEX)  ||
        (dev_id == AT25EU0021A_DEV_INDEX) ||
        (dev_id == AT25XE041D_DEV_INDEX))
    {
        // Read SPI Flash Manufacturer/Device ID
        spi_flash_read_rems_id(&man_dev_id);
        printf_string(UART, "\n\r\n\rSPI flash Manufacturer/Device ID is ");
        printf_byte(UART, (man_dev_id >> 8) & 0xFF);
        printf_byte(UART, (man_dev_id) & 0xFF);

        spi_flash_read_unique_id(&unique_id);
        printf_string(UART, "\n\r\n\rSPI flash Unique ID Number is ");
        printf_byte(UART, ((unique_id >> 32) >> 24) & 0xFF);
        printf_byte(UART, ((unique_id >> 32) >>16) & 0xFF);
        printf_byte(UART, ((unique_id >> 32) >>8) & 0xFF);
        printf_byte(UART, (unique_id >> 32) & 0xFF);
        printf_byte(UART, (unique_id >> 24) & 0xFF);
        printf_byte(UART, (unique_id >> 16) & 0xFF);
        printf_byte(UART, (unique_id >> 8) & 0xFF);
        printf_byte(UART, (unique_id) & 0xFF);
    }

    if ((dev_id == MX25R1035F_DEV_INDEX) ||
        (dev_id == MX25R2035F_DEV_INDEX))
    {
        //---------------- MX25R Low Power Mode Tests -------------------------------

        spi_flash_set_power_mode(SPI_FLASH_PM_LPM);

        print_MX25R_power_mode();

        spi_read_write_test();
        spi_read_write_test_buffer();
#if defined (CFG_SPI_DMA_SUPPORT)
        spi_read_write_test_dma();
#endif

        //---------------- MX25R High Performance Mode Tests -------------------------------

        spi_flash_set_power_mode(SPI_FLASH_PM_HPM);

        print_MX25R_power_mode();

        spi_read_write_test();
        spi_read_write_test_buffer();
#if defined (CFG_SPI_DMA_SUPPORT)
        spi_read_write_test_dma();
#endif
    }
    else
    {
        spi_read_write_test();
        spi_read_write_test_buffer();
#if defined (CFG_SPI_DMA_SUPPORT)
        spi_read_write_test_dma();
#endif

        // SPI FLASH memory protection features
        spi_protection_features_test();
    }
    printf_string(UART, "\n\rEnd of test\n\r");
}
