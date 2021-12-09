/**
 ****************************************************************************************
 *
 * @file systick.c
 *
 * @brief SysTick driver.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include "compiler.h"
#include "systick.h"
#include "datasheet.h"

/// Callback function to be called within SysTick_Handler.
systick_callback_function_t callback_function __SECTION_ZERO("retention_mem_area0");

void systick_register_callback(systick_callback_function_t callback)
{
    callback_function = callback;
}

#if !defined (__EXCLUDE_SYSTICK_HANDLER__)
void SysTick_Handler(void)
{
    if (callback_function != NULL)
        callback_function();
}
#endif

void systick_start(uint32_t usec, uint8_t exception)
{
    SetBits32(&SysTick->CTRL, SysTick_CTRL_ENABLE_Msk, 0);          // disable systick
    SetBits32(&SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, usec-1);     // set systick timeout based on 1MHz clock
    SetBits32(&SysTick->VAL,  SysTick_VAL_CURRENT_Msk, 0);          // clear the Current Value Register and the COUNTFLAG to 0
    SetBits32(&SysTick->CTRL, SysTick_CTRL_TICKINT_Msk, exception); // generate or not the SysTick exception
    SetBits32(&SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk, 0);       // use a reference clock frequency of 1 MHz
    SetBits32(&SysTick->CTRL, SysTick_CTRL_ENABLE_Msk, 1);          // enable systick
}

void systick_stop(void)
{
    SetBits32(&SysTick->VAL,  SysTick_VAL_CURRENT_Msk, 0);            // clear the Current Value Register and the COUNTFLAG to 0
    SetBits32(&SysTick->CTRL, SysTick_CTRL_ENABLE_Msk, 0);            // disable systick
}

void systick_wait(uint32_t usec)
{
    systick_start(usec, false);

    // wait for the counter flag to become 1
    // Because the flag is cleared automatically when the register is read, there is no need to clear it
    while (!GetBits32(&SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk));

    systick_stop();
}

uint32_t systick_value(void)
{
    return GetBits32(&SysTick->VAL, SysTick_VAL_CURRENT_Msk);
}
