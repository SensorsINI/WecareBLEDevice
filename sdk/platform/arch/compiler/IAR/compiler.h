/**
 ****************************************************************************************
 *
 * @file iar/compiler.h
 *
 * @brief Definitions of compiler specific directives.
 *
 * Copyright (C) 2019-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _COMPILER_H_
#define _COMPILER_H_

#ifndef __ICCARM__
#error "File only included with ARM IAR ICC"
#endif // __ICCARM__

#include "cmsis_compiler.h"

/// define the IRQ handler attribute for this compiler
#define __IRQ __irq __arm

/// define the BLE IRQ handler attribute for this compiler
#define __BTIRQ

/// define the BLE IRQ handler attribute for this compiler
#define __BLEIRQ

/// define the FIQ handler attribute for this compiler
#define __FIQ __fiq __arm

/// define size of an empty array (used to declare structure with an array size not defined)
#define __ARRAY_EMPTY

/// Put a variable in a memory maintained during deep sleep
#define __LOWPOWER_SAVED

/// Put a variable in a zero-initialised memory section
#define __SECTION_ZERO(sec_name) __attribute__((section(sec_name)))

/// Put a variable in a given memory section
#define __SECTION(sec_name) __attribute__((section(sec_name)))


#endif // _COMPILER_H_
