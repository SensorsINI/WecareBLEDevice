/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Battery level example
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
#include "gpio.h"
#include "uart.h"
#include "uart_utils.h"
#include "battery.h"
#include "datasheet.h"
#if defined (__DA14531__)
#include "hw_otpc.h"
#include "syscntl.h"
#include "otp_hdr.h"
#include "otp_cs.h"
#include "arch.h"
#endif

/**
 ****************************************************************************************
 * @brief Battery Level Indication example function
 ****************************************************************************************
 */
static void batt_test(void);

/**
 ****************************************************************************************
 * @brief Main routine of the battery level example
 ****************************************************************************************
 */
int main (void)
{
    system_init();
    batt_test();
    while(1);
}

void batt_test(void)
{
    printf_string(UART, "\n\r\n\r");
    printf_string(UART, "*******************\n\r");
    printf_string(UART, "* BATTERY TEST *\n\r");
    printf_string(UART, "*******************\n\r");

    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED) != 0x1)
    {
        printf_string(UART, "\n\rBattery type: CR2032");
        printf_string(UART, "\n\rCurrent battery level (%): ");
        printf_byte_dec(UART, battery_get_lvl(BATT_CR2032));
    }
    else
    {
        // Device operates in boost mode

        printf_string(UART, "\n\rBattery type: Alkaline");
        printf_string(UART, "\n\rCurrent battery level (%): ");
        printf_byte_dec(UART, battery_get_lvl(BATT_ALKALINE));
    }

    printf_string(UART, "% left");

    printf_string(UART, "\n\rEnd of test\n\r");
}
