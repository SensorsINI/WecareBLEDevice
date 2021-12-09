/**
 ****************************************************************************************
 * @addtogroup ARCH Architecture
 * @{
 * @addtogroup ARCH_WDG Watchdog
 * @brief Watchdog management API
 * @{
 *
 * @file arch_wdg.h
 *
 * @brief Watchdog helper functions.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _ARCH_WDG_H_
#define _ARCH_WDG_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "datasheet.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define     WATCHDOG_DEFAULT_PERIOD 0xC8

/// Watchdog enable/disable
#ifndef USE_WDOG
#if defined(CFG_WDOG)
#define USE_WDOG            1
#else
#define USE_WDOG            0
#endif //CFG_WDOG
#endif

/**
 ****************************************************************************************
 * @brief     Watchdog resume.
 * About: Start the Watchdog
 ****************************************************************************************
 */
__STATIC_INLINE void  wdg_resume(void)
{
#if (USE_WDOG)
    // Start WDOG
    SetWord16(RESET_FREEZE_REG, FRZ_WDOG);
#endif
}

/**
****************************************************************************************
* @brief     Freeze Watchdog. Call wdg_resume() to resume operation.
* About: Freeze the Watchdog
****************************************************************************************
*/
__STATIC_INLINE void  wdg_freeze(void)
{
    // Freeze WDOG
    SetWord16(SET_FREEZE_REG, FRZ_WDOG);
}

/**
 ****************************************************************************************
 * @brief     Watchdog reload.
 * @param[in] period measured in 10.24ms units.
 * About: Load the default value and resume the watchdog
 ****************************************************************************************
 */
__STATIC_INLINE void  wdg_reload(const int period)
{
#if (USE_WDOG)
    // WATCHDOG_DEFAULT_PERIOD * 10.24ms
    SetWord16(WATCHDOG_REG, WATCHDOG_DEFAULT_PERIOD);
#endif
}

/**
 ****************************************************************************************
 * @brief     Initiliaze and start the Watchdog unit.
 * @param[in] freeze If 0 Watchdog is SW freeze capable
 * About: Enable the Watchdog and configure it to
 ****************************************************************************************
 */
 __STATIC_INLINE void  wdg_init (const int freeze)
{
#if (USE_WDOG)
    wdg_reload(WATCHDOG_DEFAULT_PERIOD);
    // if freeze equals to zero WDOG can be frozen by SW!
    // it will generate an NMI when counter reaches 0 and a WDOG (SYS) Reset when it reaches -16!
    SetWord16(WATCHDOG_CTRL_REG, (freeze&0x1));
    wdg_resume ();
#else
    wdg_freeze();
#endif
}

#endif // _ARCH_WDG_H_

/// @}
/// @}
