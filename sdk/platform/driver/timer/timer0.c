/**
 ****************************************************************************************
 *
 * @file timer0.c
 *
 * @brief DA14585/DA14586/DA14531 Timer0 driver source file.
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdint.h>
#include <stddef.h>
#include "compiler.h"
#include "datasheet.h"
#include "timer0.h"

timer0_handler_function_t *TIMER0_callback      __SECTION_ZERO("retention_mem_area0");

void timer0_init(TIM0_CLK_SEL_t tim0_clk_sel, PWM_MODE_t pwm_mode, TIM0_CLK_DIV_t tim0_clk_div)
{
    uint16_t temp = (((uint16_t) pwm_mode) << 3) +
                    (((uint16_t) tim0_clk_div) << 2) +
                    (((uint16_t) tim0_clk_sel) << 1);

    SetWord16(TIMER0_CTRL_REG, temp);

    /* Set Priority for TIM0 Interrupt to level 2. */
    NVIC_SetPriority (SWTIM_IRQn, 2);
}

void timer0_start(void)
{
    /* Timer0 is running. */
    SetBits16(TIMER0_CTRL_REG, TIM0_CTRL, TIM0_CTRL_RUNNING);
}

void timer0_stop(void)
{
    /* Timer0 is off and in reset state. */
    SetBits16(TIMER0_CTRL_REG, TIM0_CTRL, TIM0_CTRL_OFF_RESET);
}

void timer0_release(void)
{
    timer0_stop();
}

void timer0_set(uint16_t pwm_on, uint16_t pwm_high, uint16_t pwm_low)
{
    SetWord16(TIMER0_ON_REG, pwm_on);
    SetWord16(TIMER0_RELOAD_M_REG, pwm_high);
    SetWord16(TIMER0_RELOAD_N_REG, pwm_low);
}

void timer0_enable_irq(void)
{
    NVIC_EnableIRQ(SWTIM_IRQn);
}

void timer0_disable_irq(void)
{
    NVIC_DisableIRQ(SWTIM_IRQn);
}

void timer0_register_callback(timer0_handler_function_t *callback)
{
    TIMER0_callback = callback;
}

#if !defined(__EXCLUDE_SWTIM_HANDLER__)
/**
 ****************************************************************************************
 * @brief SWTIM_Handler. The SWTIM_IRQn interrupt handler
 ****************************************************************************************
 */
void SWTIM_Handler(void)
{
    if (TIMER0_callback != NULL)
        TIMER0_callback();
}
#endif
