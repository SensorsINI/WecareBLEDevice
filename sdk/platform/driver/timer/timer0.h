/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup Timers
 * @{
 * @addtogroup Timer0
 * @brief Timer0 driver API
 * @{
 *
 * @file timer0.h
 *
 * @brief DA14585/DA14586/DA14531 Timer0 driver header file.
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _TIMER0_H_
#define _TIMER0_H_

#include <stdint.h>
#include "datasheet.h"

/// Timer0 PWM mode
typedef enum
{
    /// PWM signals are '1' during high time
    PWM_MODE_ONE              = 0,

    /// PWM signals send out the (fast) clock divided by 2 during high time
    PWM_MODE_CLOCK_DIV_BY_TWO = 1
} PWM_MODE_t;

/// Timer0 clock div (applies to the ON-counter only)
typedef enum
{
    /// Timer0 uses selected clock frequency as is
    TIM0_CLK_DIV_BY_10 = 0,

    /// Timer0 uses selected clock frequency divided by 10
    TIM0_CLK_NO_DIV    = 1
} TIM0_CLK_DIV_t;

/// Timer0 clock source
typedef enum
{
    /// Timer0 uses the LP clock
    TIM0_CLK_32K  = 0,

    /// Timer0 uses the system clock
    TIM0_CLK_FAST = 1
} TIM0_CLK_SEL_t;

/// Timer0 off/on
typedef enum
{
    /// Timer0 is off and in reset state
    TIM0_CTRL_OFF_RESET = 0,

    /// Timer0 is running
    TIM0_CTRL_RUNNING   = 1
} TIM0_CTRL_t;

/// Callback type to be associated with SWTIM_IRQn
typedef void (timer0_handler_function_t)(void);

/**
 ****************************************************************************************
 * @brief  Initializes TIMER0.
 * @param[in] tim0_clk_sel  Timer0 uses 16MHz (fast) or 32kHz (slow) clock frequency
 * @param[in] pwm_mode      (during high time) '1' or (fast) clock divided by 2
 * @param[in] tim0_clk_div  Division factor for TIMER0 "on" time in peripheral divider register
 ****************************************************************************************
 */
void timer0_init(TIM0_CLK_SEL_t tim0_clk_sel, PWM_MODE_t pwm_mode, TIM0_CLK_DIV_t tim0_clk_div);

/**
 ****************************************************************************************
 * @brief Starts TIMER0
 ****************************************************************************************
 */
void timer0_start(void);

/**
 ****************************************************************************************
 * @brief Stops TIMER0.
 ****************************************************************************************
 */
void timer0_stop(void);

/**
 ****************************************************************************************
 * @brief Releases TIMER0. Does not disable the TIM clock, as it is shared with TIMER2.
 ****************************************************************************************
 */
void timer0_release(void);

/**
 ****************************************************************************************
 * @brief Configures TIMER0 ON counter reload value.
 * @param[in] pwm_on ON COUNTER reload value
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void timer0_set_pwm_on_counter(uint16_t pwm_on)
{
    SetWord16(TIMER0_ON_REG, pwm_on);
}

/**
 ****************************************************************************************
 * @brief Configures TIMER0 M counter reload value (duration of PWM0 high part).
 * @param[in] pwm_high M COUNTER reload value
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void timer0_set_pwm_high_counter(uint16_t pwm_high)
{
    SetWord16(TIMER0_RELOAD_M_REG, pwm_high);
}

/**
 ****************************************************************************************
 * @brief Configures TIMER0 N counter reload value (duration of PWM0 low part).
 * @param[in] pwm_low N COUNTER reload value
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void timer0_set_pwm_low_counter(uint16_t pwm_low)
{
    SetWord16(TIMER0_RELOAD_N_REG, pwm_low);
}

/**
 ****************************************************************************************
 * @brief Configures TIMER0 pwm timing parameters.
 * @param[in] pwm_on   ON COUNTER reload value
 * @param[in] pwm_high M COUNTER reload value
 * @param[in] pwm_low  N COUNTER reload value
 ****************************************************************************************
 */
void timer0_set(uint16_t pwm_on, uint16_t pwm_high, uint16_t pwm_low);

/**
 ****************************************************************************************
 * @brief Enables SWTIM_IRQn.
 ****************************************************************************************
 */
void timer0_enable_irq(void);

/**
 ****************************************************************************************
 * @brief Disables SWTIM_IRQn.
 ****************************************************************************************
 */
void timer0_disable_irq(void);

/**
 ****************************************************************************************
 * @brief Registers a callback function to be called within SWTIM_Handler.
 * @param[in] callback Callback function
 ****************************************************************************************
 */
void timer0_register_callback(timer0_handler_function_t *callback);

#endif /* _TIMER0_H_ */

///@}
///@}
///@}
