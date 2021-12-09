/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief SysTick example
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
#include "systick.h"
#include "gpio.h"

#define SYSTICK_PERIOD_US   1000000     // period for systick timer in us, so 1000000ticks = 1second
#define SYSTICK_EXCEPTION   1           // generate systick exceptions
int i = 0;

/**
 ****************************************************************************************
 * @brief SysTick Test routine
 ****************************************************************************************
 */
void systick_test(void);

/**
 ****************************************************************************************
 * @brief Main routine of the SysTick example
 ****************************************************************************************
 */
int main (void)
{
    system_init();
    systick_test();
    while(1);
}

/**
 ****************************************************************************************
 * @brief SysTick ISR routine
 ****************************************************************************************
 */
static void systick_isr(void)
{
    if (i == 0)
    {
        GPIO_SetActive(LED_PORT, LED_PIN);
        i = 1;
    }
    else
    {
        GPIO_SetInactive(LED_PORT, LED_PIN);
        i = 0;
    }
}


void systick_test(void)
{
    systick_register_callback(systick_isr);
    // Systick will be initialized to use a reference clock frequency of 1 MHz
    systick_start(SYSTICK_PERIOD_US, SYSTICK_EXCEPTION);
}
