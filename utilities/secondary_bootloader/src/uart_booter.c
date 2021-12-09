/**
 ****************************************************************************************
 *
 * @file uart_booter.c
 *
 * @brief Firmware download through UART
 *
 * Copyright (C) 2012-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include "uart_booter.h"
#include "user_periph_setup.h"
#include "gpio.h"
#include "uart.h"
#include "systick.h"


static volatile bool timeout;
/**
 ****************************************************************************************
 * @brief SysTick timer callback
 ****************************************************************************************
 */
static void systick_callback(void)
{
    timeout = true;
}

static volatile bool received = false;
/**
 ****************************************************************************************
 * @brief callback UART receiver
 ****************************************************************************************
 */
void uart_receive_callback(void)
{
    received = true;
}

/**
 ****************************************************************************************
 * @brief UART error callback
 ****************************************************************************************
 */
void uart_error_callback(uart_t *uart, uint8_t uart_err_status)
{
}

/**
****************************************************************************************
* @brief Receive byte from UART
 *
 * @param[in] ch    Pointer to the variable that will store the received byte
 *
 * @return 1 on success, 0 on failure.
****************************************************************************************
*/
static int uart_receive_byte(uint8_t *recv_byte)
{
    received = false;
    timeout = false;

    systick_register_callback(systick_callback);
    // Timeout set to 60ms to cover all UART baud rates
    // and support up to 60ms RX waiting periods.
    systick_start(60000, true);
    uart_receive(UART1, recv_byte, sizeof(uint8_t), UART_OP_INTR);
    while (!received && !timeout);
    systick_stop();

    if (timeout && !received)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


/**
****************************************************************************************
* @brief download firmware through UART
****************************************************************************************
*/
int FwDownload(void)
{
    int fw_size;
    int i;
    char *code;
    char crc_code;
    uint8_t recv_byte;

    uart_write_byte(UART1, STX);                                // send start TX char

    if (0 == uart_receive_byte(&recv_byte))
    {
        return -1;  // receive SOH
    }
    if (recv_byte != SOH)
    {
        return -2;
    }

    if (0 == uart_receive_byte(&recv_byte))
    {
        return -3;  // receive FW length LSB
    }
    fw_size = recv_byte;

    if (0 == uart_receive_byte(&recv_byte))
    {
        return -4;  // receive FW length MSB
    }
    fw_size += (recv_byte << 8);

    // Support the extended length protocol
    if (fw_size == 0)
    {
        if (0 == uart_receive_byte(&recv_byte))
        {
            return -3;  // receive FW length LSB
        }
        fw_size = recv_byte;

        if (0 == uart_receive_byte(&recv_byte))
        {
            return -4;  // receive FW length MiddleSB
        }
        fw_size += (recv_byte << 8);

        if (0 == uart_receive_byte(&recv_byte))
        {
            return -9;  // receive FW length MSB
        }
        fw_size += (recv_byte << 16);
    }

    if ((fw_size == 0) || fw_size > MAX_CODE_LENGTH)    // check if the program fits into memory
    {
        uart_write_byte(UART1, NAK);                            // if it does not fit send ACK
        return -5;
    }
    else
    {
        uart_write_byte(UART1, ACK);                            // if it fits send ACK
    }

    crc_code = 0;
    code = (char*) (SYSRAM_COPY_BASE_ADDRESS);          // set pointer to SYSRAM
    for (i = 0; i < fw_size; i++)                       // copy code from UART to RAM
    {
        if (0 == uart_receive_byte(&recv_byte))
        {
            return -6; // receive code byte
        }
        crc_code ^= recv_byte;                          // update CRC
        *code = recv_byte;                              // write to RAM
        code++;                                         // increase RAM pointer
    }
    uart_write_byte(UART1, crc_code);                           // send CRC byte

    if (0 == uart_receive_byte(&recv_byte))
    {
        return -7;  // receive ACK
    }

    if (recv_byte != ACK)
    {
        return -8;
    }

    return fw_size;
}

/**
****************************************************************************************
* @brief download firmware through 1-wire UART
****************************************************************************************
*/

int FwDownloadOneWireUART(void)
{
    int fw_size;
    int i;
    char *code;
    char crc_code;
    uint8_t recv_byte;

    GPIO_ConfigurePin(UART_GPIO_PORT, GPIO_PIN_5, OUTPUT, PID_UART1_TX, false);
    uart_write_byte(UART1, STX);                                // send start TX char
    uart_wait_tx_finish(UART1);

    GPIO_ConfigurePin(UART_GPIO_PORT, GPIO_PIN_5, INPUT, PID_UART1_RX, false);

    if (0 == uart_receive_byte(&recv_byte))
    {
        return -1;  // receive SOH
    }
    if (recv_byte != SOH)
    {
        return -2;
    }

    if (0 == uart_receive_byte(&recv_byte))
    {
        return -3;  // receive FW length LSB
    }
    fw_size = recv_byte;

    if (0 == uart_receive_byte(&recv_byte))
    {
        return -4;  // receive FW length MSB
    }
    fw_size += (recv_byte << 8);

    // Support the extended length protocol
    if (fw_size == 0)
    {
        if (0 == uart_receive_byte(&recv_byte))
        {
            return -3;  // receive FW length LSB
        }
        fw_size = recv_byte;

        if (0 == uart_receive_byte(&recv_byte))
        {
            return -4;  // receive FW length MiddleSB
        }
        fw_size += (recv_byte << 8);

        if (0 == uart_receive_byte(&recv_byte))
        {
            return -9;  // receive FW length MSB
        }
        fw_size += (recv_byte << 16);
    }

    if ((fw_size == 0) || fw_size > MAX_CODE_LENGTH)    // check if the program fits into memory
    {
        GPIO_ConfigurePin(UART_GPIO_PORT, GPIO_PIN_5, OUTPUT, PID_UART1_TX, false);
        uart_write_byte(UART1, NAK);                            // if it does not fit send ACK
        uart_wait_tx_finish(UART1);
        return -5;
    }
    else
    {
        GPIO_ConfigurePin(UART_GPIO_PORT, GPIO_PIN_5, OUTPUT, PID_UART1_TX, false);
        uart_write_byte(UART1, ACK);                            // if it fits send ACK
        uart_wait_tx_finish(UART1);
    }

    GPIO_ConfigurePin(UART_GPIO_PORT, GPIO_PIN_5, INPUT, PID_UART1_RX, false);
    crc_code = 0;
    code = (char*) (SYSRAM_COPY_BASE_ADDRESS);          // set pointer to SYSRAM
    for (i = 0; i < fw_size; i++)                       // copy code from UART to RAM
    {
        if (0 == uart_receive_byte(&recv_byte))
        {
            return -6; // receive code byte
        }
        crc_code ^= recv_byte;                          // update CRC
        *code = recv_byte;                              // write to RAM
        code++;                                         // increase RAM pointer
    }
    GPIO_ConfigurePin(UART_GPIO_PORT, GPIO_PIN_5, OUTPUT, PID_UART1_TX, false);
    uart_write_byte(UART1, crc_code);                           // send CRC byte
    uart_wait_tx_finish(UART1);

    GPIO_ConfigurePin(UART_GPIO_PORT, GPIO_PIN_5, INPUT, PID_UART1_RX, false);
    if (0 == uart_receive_byte(&recv_byte))
    {
        return -7;  // receive ACK
    }

    if (recv_byte != ACK)
    {
        return -8;
    }

    return fw_size;
}
