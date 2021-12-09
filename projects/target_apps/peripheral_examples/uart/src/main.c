/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief UART example for DA14585, DA14586 and DA14531 devices.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#include <stdio.h>
#include "arch_system.h"
#include "user_periph_setup.h"
#include "uart_common.h"
#include "uart_utils.h"
#include "gpio.h"

/**
 ****************************************************************************************
 * @brief UART print example
 * @param[in] uart_id           Identifies which UART to use
 ****************************************************************************************
 */
void uart_print_example(uart_t * uart);

/**
 ****************************************************************************************
 * @brief Main routine of the UART example
 ****************************************************************************************
 */
int main(void)
{
    system_init();

    // Setup UART1 pins and configuration
    uart_periph_init(UART1);

    // Run UART1 print example
    uart_print_example(UART1);

    // Run UART1 send blocking example
    uart_send_blocking_example(UART1);

    // Run UART1 send interrupt example
    uart_send_interrupt_example(UART1);

#if defined (CFG_UART_DMA_SUPPORT)
    // Run UART1 send dma example
    uart_send_dma_example(UART1);
#endif
    // Run UART1 receive blocking example
    uart_receive_blocking_example(UART1);

    // Run UART1 receive interrupt example
    uart_receive_interrupt_example(UART1);

#if defined (CFG_UART_DMA_SUPPORT)
    // Run UART1 receive dma example
    uart_receive_dma_example(UART1);
#endif

    // Run UART1 loopback example
    uart_loopback_interrupt_example(UART1);


    // Setup UART2 pins and configuration
    uart_periph_init(UART2);

    // Run UART2 print example
    uart_print_example(UART2);

    // Run UART2 send blocking example
    uart_send_blocking_example(UART2);

    // Run UART2 send interrupt example
    uart_send_interrupt_example(UART2);

#if defined (CFG_UART_DMA_SUPPORT)
    // Run UART2 send dma example
    uart_send_dma_example(UART2);
#endif

    // Run UART2 receive blocking example
    uart_receive_blocking_example(UART2);

    // Run UART2 receive interrupt example
    uart_receive_interrupt_example(UART2);

#if defined (CFG_UART_DMA_SUPPORT)
    // Run UART2 receive dma example
    uart_receive_dma_example(UART2);
#endif

    // Run UART2 loopback example
    uart_loopback_interrupt_example(UART2);

    printf_string(UART2, "\n\n\n\r* UART examples finished successfully *\n\r");
    while(1);
}

void uart_print_example(uart_t * uart)
{
    printf_string(uart, "\n\r\n\r****************************************\n\r");
    if (uart == UART1)
    {
        printf_string(uart, "* UART1 Print example *\n\r");
    }
    else
    {
        printf_string(uart, "* UART2 Print example *\n\r");
    }
    printf_string(uart, "****************************************\n\r\n\r");
    printf_string(uart, " printf_string() = Hello World! \n\r");
    printf_string(uart, "\n\r print_hword()   = 0x");
    print_hword(uart, 0xAABB);
    printf_string(uart, "\n\r");
    printf_string(uart, "\n\r print_word()    = 0x");
    print_word(uart, 0x11223344);
    printf_string(uart, "\n\r");
    printf_string(uart, "****************************************\n\r\n\r");
}
