/**
 ****************************************************************************************
 *
 * @file uart_receive_example.c
 *
 * @brief UART receive examples
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

const int READ_CHAR_COUNT = 5;

static char buffer[READ_CHAR_COUNT + 1];

volatile bool uart_receive_finished = false;
volatile uint16_t data_received_cnt = 0;

static void uart_receive_cb(uint16_t length)
{
    data_received_cnt = length;
    uart_receive_finished = true;
}

void uart_receive_blocking_example(uart_t* uart)
{
    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart == UART1)
    {
        printf_string(uart, "* UART1 Receive Blocking example *\n\r");
    }
    else
    {
        printf_string(uart, "* UART2 Receive Blocking example *\n\r");
    }
    printf_string(uart, "****************************************\n\r\n\r");

    printf_string(uart, "Type 5 characters.\r\n");

    uart_receive(uart, (uint8_t *)buffer, READ_CHAR_COUNT, UART_OP_BLOCKING);

    buffer[READ_CHAR_COUNT] = 0; // make it a null terminated string

    printf_string(uart, "You typed the following characters:\r\n");
    printf_string(uart, buffer);
    printf_string(uart, "\r\n");

    printf_string(uart, "****************************************\n\r\n\r");
}

void uart_receive_interrupt_example(uart_t* uart)
{
    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart == UART1)
    {
        printf_string(uart, "* UART1 Receive Interrupt example *\n\r");
    }
    else
    {
        printf_string(uart, "* UART2 Receive Interrupt example *\n\r");
    }
    printf_string(uart, "****************************************\n\r\n\r");

    uart_receive_finished = false;
    data_received_cnt = 0;
    uart_register_rx_cb(uart, uart_receive_cb);

    printf_string(uart, "Type 5 characters.\r\n");

    uart_receive(uart, (uint8_t *)buffer, READ_CHAR_COUNT, UART_OP_INTR);
    while (!uart_receive_finished);
    buffer[READ_CHAR_COUNT] = 0; // make it a null terminated string

    printf_string(uart, "You typed the following characters:\r\n");
    printf_string(uart, buffer);
    printf_string(uart, "\r\n");

    printf_string(uart, "****************************************\n\r\n\r");
}

#if defined (CFG_UART_DMA_SUPPORT)
void uart_receive_dma_example(uart_t* uart)
{
    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart == UART1)
    {
        printf_string(uart, "* UART1 Receive DMA example *\n\r");
    }
    else
    {
        printf_string(uart, "* UART2 Receive DMA example *\n\r");
    }
    printf_string(uart, "****************************************\n\r\n\r");

    uart_receive_finished = false;
    data_received_cnt = 0;
    uart_register_rx_cb(uart, uart_receive_cb);

    printf_string(uart, "Type 5 characters.\r\n");

    uart_receive(uart, (uint8_t *)buffer, READ_CHAR_COUNT, UART_OP_DMA);
    while (!uart_receive_finished);
    buffer[READ_CHAR_COUNT] = 0; // make it a null terminated string

    printf_string(uart, "You typed the following characters:\r\n");
    printf_string(uart, buffer);
    printf_string(uart, "\r\n");

    printf_string(uart, "****************************************\n\r\n\r");
}
#endif
