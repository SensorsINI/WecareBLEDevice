/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Blinky example
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#include <stdio.h>
#include "arch_system.h"
#include "uart.h"
#include "uart_utils.h"
#include "user_periph_setup.h"
#include "gpio.h"



#define LED_OFF_THRESHOLD           10000
#define LED_ON_THRESHOLD            400000

/**
 ****************************************************************************************
 * @brief Blinky test function
 ****************************************************************************************
 */
void blinky_test(void);

/**
 ****************************************************************************************
 * @brief Main routine of the Blinky example
 ****************************************************************************************
 */
int main (void)
{
    system_init();
    blinky_test();
    while(1);
}

void blinky_test(void)
{
    volatile int i=0;

    printf_string(UART, "\n\r\n\r");
    printf_string(UART, "***************\n\r");
    printf_string(UART, "* BLINKY DEMO *\n\r");
    printf_string(UART, "***************\n\r");

    while(1)
    {
        i++;

        if (LED_OFF_THRESHOLD == i)
        {
            GPIO_SetActive(LED_PORT, LED_PIN);
            printf_string(UART, "\n\r *LED ON* ");
        }

        if (LED_ON_THRESHOLD == i)
        {
            GPIO_SetInactive(LED_PORT, LED_PIN);
            printf_string(UART, "\n\r *LED OFF* ");
        }

        if (i == 2 * LED_ON_THRESHOLD)
        {
            i = 0;
        }
    }
}
