/**
 ****************************************************************************************
 *
 * @file uart_loopback_examples.c
 *
 * @brief UART loopback examples.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include "uart.h"
#include "uart_utils.h"
#include "uart_common.h"
#include "ring_buffer.h"


static uint8_t rxbuf[1];
static uint8_t txbuf[1];
static volatile uint8_t tx_in_progress = 0;
static uart_t* uart_id;
volatile uint16_t total_cnt=0;

static void uart_read_cb(uint16_t length)
{
    total_cnt += length;
    // Put received byte in ring buffer
    buffer_put_byte(rxbuf[0]);

    // Start the next asynchronous read of 1 character.
    uart_receive(uart_id, rxbuf, 1, UART_OP_INTR);
}

static void uart_write_cb(uint16_t length)
{
    tx_in_progress = 0;
}

void uart_loopback_interrupt_example(uart_t *uart)
{
    total_cnt = 0;
    uart_id = uart;

    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart_id == UART1)
    {
        printf_string(uart_id, "* UART1 Loopback Interrupt example *\n\r");
    }
    else
    {
        printf_string(uart_id, "* UART2 Loopback Interrupt example *\n\r");
    }
    printf_string(uart_id, "****************************************\n\r\n\r");

    uart_register_rx_cb(uart_id, uart_read_cb);
    uart_register_tx_cb(uart_id, uart_write_cb);

    uart_receive(uart_id, rxbuf, 1, UART_OP_INTR);
    while(total_cnt < 20)
    {
        int status;
        // Attemp to get a byte from the ring buffer
        status = buffer_get_byte(&txbuf[0]);

        // Check if a character was read successfully from the ring buffer
        if (!status)
        {
            // Write the character to UART2 and then wait synchronously for completion.
            tx_in_progress = 1;
            uart_send(uart_id, txbuf, 1, UART_OP_INTR);
            while (tx_in_progress);
        }
    }

    printf_string(uart, "\r\n");
    printf_string(uart_id, "****************************************\n\r\n\r");
}
