/**
 ****************************************************************************************
 * @addtogroup ARCH Architecture
 * @{
 * @addtogroup ARCH_RAM RAM Utilities
 * @brief RAM utilities API
 * @{
 *
 * @file arch_ram.h
 *
 * @brief System RAM definitions.
 *
 * Copyright (C) 2017-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _ARCH_RAM_H_
#define _ARCH_RAM_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "datasheet.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#if defined (__DA14531__)
// All RAM blocks are powered off (not retained) in Extended Sleep, Deep Sleep or Hibernation mode
#define ALL_RAM_BLOCKS_OFF           (0x15)
#endif

#if defined (__DA14531__)
    // The base addresses of the 3 RAM blocks
    #define RAM_1_BASE_ADDR                 (0x07FC0000) // RAM1 block = 16KB (0x4000)
    #define RAM_2_BASE_ADDR                 (0x07FC4000) // RAM2 block = 12KB (0x3000)
    #define RAM_3_BASE_ADDR                 (0x07FC7000) // RAM3 block = 20KB (0x5000)

    #define RAM_END_ADDR                    (RAM_3_BASE_ADDR + 0x5000)
#else
    // The base addresses of the 4 RAM blocks
    #define RAM_1_BASE_ADDR                 (0x07FC0000) // RAM1 block = 32KB (0x8000)
    #define RAM_2_BASE_ADDR                 (0x07FC8000) // RAM2 block = 16KB (0x4000)
    #define RAM_3_BASE_ADDR                 (0x07FCC000) // RAM3 block = 16KB (0x4000)
    #define RAM_4_BASE_ADDR                 (0x07FD0000) // RAM4 block = 32KB (0x8000)

    #define RAM_END_ADDR                    (RAM_4_BASE_ADDR + 0x8000)
#endif

#if defined (CFG_CUSTOM_SCATTER_FILE)

    #if defined (__DA14531__)
        #if defined (CFG_RETAIN_RAM_1_BLOCK)
            #define RAM_1_BLOCK      (0x14)
        #else
            #define RAM_1_BLOCK      (ALL_RAM_BLOCKS_OFF)
        #endif

        #if defined (CFG_RETAIN_RAM_2_BLOCK)
            #define RAM_2_BLOCK      (0x11)
        #else
            #define RAM_2_BLOCK      (ALL_RAM_BLOCKS_OFF)
        #endif
    #else
        #if defined (CFG_RETAIN_RAM_1_BLOCK)
            #define RAM_1_BLOCK      (0x01)
        #else
            #define RAM_1_BLOCK      (0)
        #endif

        #if defined (CFG_RETAIN_RAM_2_BLOCK)
            #define RAM_2_BLOCK      (0x02)
        #else
            #define RAM_2_BLOCK      (0)
        #endif

        #if defined (CFG_RETAIN_RAM_3_BLOCK)
            #define RAM_3_BLOCK      (0x04)
        #else
            #define RAM_3_BLOCK      (0)
        #endif
    #endif
#else
    #if defined (__DA14531__)
        #define RAM_1_BLOCK          (0x14)
        #define RAM_2_BLOCK          (0x11)
    #else
        #define RAM_1_BLOCK          (0x01)
        #define RAM_2_BLOCK          (0x02)
        #define RAM_3_BLOCK          (0x04)
    #endif
#endif

#if defined (__DA14531__)
#define RAM_3_BLOCK                  (0x05)
#else
#define RAM_4_BLOCK                  (0x08)
#endif

#if defined (__DA14531__)
    #define ALL_RAM_BLOCKS           (0)
#else
    #define ALL_RAM_BLOCKS           (RAM_1_BLOCK | RAM_2_BLOCK | RAM_3_BLOCK | RAM_4_BLOCK)
#endif

/**
 ****************************************************************************************
 * @brief Sets the RAM retention mode.
 * @param[in] mode      RAM retention mode
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void arch_ram_set_retention_mode(uint8_t mode)
{
#if defined (__DA14531__)
    SetWord16(RAM_PWR_CTRL_REG, mode);
#else
    SetBits16(PMU_CTRL_REG, RETENTION_MODE, mode);
#endif
}

#if ((USE_HIGH_TEMPERATURE + USE_AMB_TEMPERATURE + USE_MID_TEMPERATURE + USE_EXT_TEMPERATURE) > 1)
    #error "Config error: Multiple temperature ranges were defined."
#endif

#if !defined (__DA14531__)
// Possible combinations of the 4 RAM blocks that produce a total size more than 64KB
#define RAM_SIZE_80KB_OPT1       (RAM_4_BLOCK | RAM_2_BLOCK | RAM_1_BLOCK)
#define RAM_SIZE_80KB_OPT2       (RAM_4_BLOCK | RAM_3_BLOCK | RAM_1_BLOCK)
#define RAM_SIZE_96KB_OPT1       (RAM_4_BLOCK | RAM_3_BLOCK | RAM_2_BLOCK | RAM_1_BLOCK)

#if (USE_HIGH_TEMPERATURE)
    #if (USE_POWER_OPTIMIZATIONS)
        #error "Config error: High temperature support is not compatible with power optimizations."
    #else
        #define DEFAULT_LDO_SET_64K     (0x05)
        #define DEFAULT_LDO_SET_96K     (0x04)
    #endif
#elif (USE_EXT_TEMPERATURE)
    #define DEFAULT_LDO_SET_64K         (0x07)
    #define DEFAULT_LDO_SET_96K         (0x06)
#elif (USE_MID_TEMPERATURE)
    #define DEFAULT_LDO_SET             (0x08)
#endif
#endif

#endif // _ARCH_RAM_H_

/// @}
/// @}
