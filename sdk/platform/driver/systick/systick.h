/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup SysTick
 * @brief SysTick driver API
 * @{
 *
 * @file systick.h
 *
 * @brief SysTick driver header file.
 *
 * Copyright (C) 2014-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#ifndef _SYSTICK_H_
#define _SYSTICK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Callback type to be associated with SysTick_IRQn.
typedef void (*systick_callback_function_t)(void);

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Register Callback function for SysTick exception.
 * @param[in] callback Callback function's reference
 ****************************************************************************************
 */
void systick_register_callback(systick_callback_function_t callback);

/**
 ****************************************************************************************
 * @brief Function to start the SysTick timer.
 * @param[in] usec      The duration of the countdown
 * @param[in] exception Set to TRUE to generate an exception when the timer counts down
 *                      to 0, FALSE not to
 ****************************************************************************************
 */
void systick_start(uint32_t usec, uint8_t exception);

/**
 ****************************************************************************************
 * @brief Function to create a delay.
 * @param[in] usec the duration of the delay
 ****************************************************************************************
 */
void systick_wait(uint32_t usec);

/**
 ****************************************************************************************
 * @brief Function to stop the SysTick timer.
 ****************************************************************************************
 */
void systick_stop(void);

/**
 ****************************************************************************************
 * @brief Function to read the current value of the timer.
 * @return the current value of the timer
 ****************************************************************************************
 */
uint32_t systick_value(void);

#endif // _SYSTICK_H_

///@}
///@}
