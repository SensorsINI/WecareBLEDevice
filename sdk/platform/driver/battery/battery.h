/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup Battery
 * @brief Battery driver API
 * @{
 *
 * @file battery.h
 *
 * @brief Battery driver header file.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
 
#ifndef _BATTERY_H_
#define _BATTERY_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */

#if defined (__DA14531__)
/// Supported battery types
typedef enum
{
    /// CR2032 lithium cell battery
    BATT_CR2032   = 0,

    /// Alkaline cell battery
    BATT_ALKALINE = 1
} batt_t;
#else
/// Supported battery types
typedef enum
{
    /// CR2032 lithium cell battery
    BATT_CR2032   = 0,

    /// CR1225 lithium cell battery
    BATT_CR1225   = 1,

    /// Alkaline cell battery (1 cell in boost, 2 cells in buck mode)
    BATT_ALKALINE = 2
} batt_t;

/// @name Battery measurements
///@{
/** ADC value */
#define BATTERY_MEASUREMENT_BOOST_AT_1V5    (0x340)
#define BATTERY_MEASUREMENT_BOOST_AT_1V0    (0x230)
#define BATTERY_MEASUREMENT_BOOST_AT_0V9    (0x1F0)
#define BATTERY_MEASUREMENT_BOOST_AT_0V8    (0x1B0)

#define BATTERY_MEASUREMENT_BUCK_AT_3V0     (0x6B0)
#define BATTERY_MEASUREMENT_BUCK_AT_2V8     (0x640)
#define BATTERY_MEASUREMENT_BUCK_AT_2V6     (0x5D0)
#define BATTERY_MEASUREMENT_BUCK_AT_2V4     (0x550)
///@}
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief Returns voltage of the @p batt_type battery.
 * @param[in] batt_type Battery type.
 * @return battery voltage in mV
 ****************************************************************************************
 */
uint16_t battery_get_voltage(const batt_t batt_type);
#endif

/**
 ****************************************************************************************
 * @brief Returns battery level for @p batt_type.
 * @param[in] batt_type Battery type.
 * @return battery level (0-100%)
 ****************************************************************************************
 */
uint8_t battery_get_lvl(const batt_t batt_type);

#endif // _BATTERY_H_

///@}
///@}
