/**
 ****************************************************************************************
 *
 * @file uart_send_examples.c
 *
 * @brief UART send examples.
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

static const char OUTPUT_STRING[] = "0123456789 0123456789 0123456789 0123456789 0123456789\r\n"
                                    "0123456789 0123456789 0123456789 0123456789 0123456789\r\n"
                                    "0123456789 0123456789 0123456789 0123456789 0123456789\r\n"
                                    "0123456789 0123456789 0123456789 0123456789 0123456789\r\n"
                                    "0123456789 0123456789 0123456789 0123456789 0123456789\r\n"
                                    ;

volatile bool uart_send_finished = false;
volatile uint16_t data_sent_cnt = 0;

static void uart_send_cb(uint16_t length)
{
    data_sent_cnt = length;
    uart_send_finished = true;
}

void uart_send_blocking_example(uart_t* uart)
{
    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart == UART1)
    {
        printf_string(uart, "* UART1 Send Blocking example *\n\r");
    }
    else
    {
        printf_string(uart, "* UART2 Send Blocking example *\n\r");
    }
    printf_string(uart, "****************************************\n\r\n\r");

    uart_send(uart, (uint8_t *)OUTPUT_STRING, sizeof(OUTPUT_STRING) - 1, UART_OP_BLOCKING);

    printf_string(uart, "****************************************\n\r\n\r");
}

void uart_send_interrupt_example(uart_t* uart)
{
    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart == UART1)
    {
        printf_string(uart, "* UART1 Send Interrupt example *\n\r");
    }
    else
    {
        printf_string(uart, "* UART2 Send Interrupt example *\n\r");
    }
    printf_string(uart, "****************************************\n\r\n\r");

    uart_send_finished = false;
    data_sent_cnt = 0;

    uart_register_tx_cb(uart, uart_send_cb);
    uart_send(uart, (uint8_t *)OUTPUT_STRING, sizeof(OUTPUT_STRING) - 1, UART_OP_INTR);
    while (!uart_send_finished);
    printf_string(uart, "\n\rData sent: 0x");
    print_hword(uart, data_sent_cnt);
    printf_string(uart, " Bytes\n\r\n\r");
    printf_string(uart, "****************************************\n\r\n\r");
}

#if defined (CFG_UART_DMA_SUPPORT)
void uart_send_dma_example(uart_t* uart)
{
    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart == UART1)
    {
        printf_string(uart, "* UART1 Send DMA example *\n\r");
    }
    else
    {
        printf_string(uart, "* UART2 Send DMA example *\n\r");
    }
    printf_string(uart, "****************************************\n\r\n\r");

    uart_send_finished = false;
    data_sent_cnt = 0;

    uart_register_tx_cb(uart, uart_send_cb);
    uart_send(uart, (uint8_t *)OUTPUT_STRING, sizeof(OUTPUT_STRING) - 1, UART_OP_DMA);
    while (!uart_send_finished);
    printf_string(uart, "\n\rData sent: 0x");
    print_hword(uart, data_sent_cnt);
    printf_string(uart, " Bytes\n\r\n\r");
    printf_string(uart, "****************************************\n\r\n\r");
}
#endif
