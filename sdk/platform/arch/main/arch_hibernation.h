/**
 ****************************************************************************************
 * @addtogroup ARCH Architecture
 * @{
 * @addtogroup ARCH_SYSTEM System
 * @brief System API
 * @{
 *
 * @file arch_hibernation.h
 *
 * @brief Hibernation-related API
 *
 * Copyright (C) 2019-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#if defined (__DA14531__)

#ifndef _ARCH_HIBERNATION_H_
#define _ARCH_HIBERNATION_H_

#include <stdbool.h>
#include "arch_api.h"

/**
 ****************************************************************************************
 * @brief       Puts the system into hibernation mode.
 * @param[in]   wkup_mask      Selects the GPIO(s) mask which will wake up the system from
 *                             hibernation. The system can wake up from P01 to P05.
 *                             For example, write 0xA (0b1010) if P03 and P01 are used.
 * @param[in]   ram1           Selects the RAM1 state (on/off) during the hibernation.
 * @param[in]   ram2           Selects the RAM2 state (on/off) during the hibernation.
 * @param[in]   ram3           Selects the RAM3 state (on/off) during the hibernation.
 * @param[in]   remap_addr0    Selects which memory is located at address 0x0000 for
 *                             execution (after waking up from hibernation).
 * @param[in]   pad_latch_en   true = Enable latching of pads state during sleep,
 *                             false = Disable latching of pads state during sleep.
 ****************************************************************************************
 */
void arch_set_hibernation(uint8_t wkup_mask,
                          pd_sys_down_ram_t ram1,
                          pd_sys_down_ram_t ram2,
                          pd_sys_down_ram_t ram3,
                          remap_addr0_t remap_addr0,
                          bool pad_latch_en);

/**
 ****************************************************************************************
 * @brief       Puts the system into stateful hibernation mode.
 * @param[in]   wkup_mask      Selects the GPIO(s) mask which will wake up the system from
 *                             hibernation. The system can wake up from P01 to P05.
 *                             For example, write 0xA (0b1010) if P03 and P01 are used.
 * @param[in]   ram1           Selects the RAM1 state (on/off) during the hibernation.
 * @param[in]   ram2           Selects the RAM2 state (on/off) during the hibernation.
 * @param[in]   ram3           Selects the RAM3 state (on/off) during the hibernation.
 * @param[in]   remap_addr0    Selects which memory is located at address 0x0000 for
 *                             execution (after waking up from hibernation).
 * @param[in]   pad_latch_en   true = Enable latching of pads state during sleep,
 *                             false = Disable latching of pads state during sleep.
 * @warning Since this function saves the system state (code and data), RAM blocks
 *          shall be retained according to application memory layout, e.g., using
 *          the default linker script, RAM1 and RAM3 blocks shall be always retained
 *          and RAM2 block shall be retained only if holding state code and/or data.
 * @note Assumption is that either RC32M or XTAL32M is the main clock before the
 *       system goes to stateful hibernation.
 ****************************************************************************************
 */
void arch_set_stateful_hibernation(uint8_t wkup_mask,
                                   pd_sys_down_ram_t ram1,
                                   pd_sys_down_ram_t ram2,
                                   pd_sys_down_ram_t ram3,
                                   stateful_hibern_remap_addr0_t remap_addr0,
                                   bool pad_latch_en);

#endif // _ARCH_HIBERNATION_H_

#endif // __DA14531__

///@}
///@}
