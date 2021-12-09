/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup SYSCNTL
 * @brief System controller driver API
 * @{
 *
 * @file syscntl.h
 *
 * @brief System controller driver header file.
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef SYSCNTL_H_
#define SYSCNTL_H_

#include "datasheet.h"

#if defined (__DA14531__)
#include <stdbool.h>
#include "arch.h"
#endif

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

#if defined (__DA14531__)

/* Error Codes */

/// DCDC no error
#define SYSCNTL_DCDC_ERR_NO_ERROR               (0)
/// DCDC reserved by OTP error
#define SYSCNTL_DCDC_ERR_RESERVED_BY_OTP        (-1)
/// DCDC error PMU not in boost configuration
#define SYSCNTL_DCDC_ERR_NOT_IN_BOOST           (-2)
/// DCDC error PMU not in buck  configuration
#define SYSCNTL_DCDC_ERR_NOT_IN_BUCK            (-3)
/// DCDC POR trigger error
#define SYSCNTL_DCDC_ERR_POR_TRIGGER            (-4)
/// DCDC POR trigger warning
#define SYSCNTL_DCDC_WARNING_POR_TRIGGER        (-5)
/// DCDC error PMU in bypass configuration
#define SYSCNTL_DCDC_ERR_IN_BYPASS              (-6)

/// DCDC converter nominal voltage levels
enum {
    /// DCDC nominal voltage level 1.1V
    SYSCNTL_DCDC_NOMINAL_LEVEL_1V1      = 0U,

    /// DCDC nominal voltage level 1.8V
    SYSCNTL_DCDC_NOMINAL_LEVEL_1V8      = 1U,

    /// DCDC nominal voltage level 2.5V
    SYSCNTL_DCDC_NOMINAL_LEVEL_2V5      = 2U,

    /// DCDC nominal voltage level 3.0V
    SYSCNTL_DCDC_NOMINAL_LEVEL_3V0      = 3U,
};

/// DCDC converter trim values
enum {
    /// DCDC trim -75mV
    SYSCNTL_DCDC_TRIM_N_75mV      = 0U,

    /// DCDC trim -50mV
    SYSCNTL_DCDC_TRIM_N_50mV      = 1U,

    /// DCDC trim -25mV
    SYSCNTL_DCDC_TRIM_N_25mV      = 2U,

    /// DCDC trim 0mV
    SYSCNTL_DCDC_TRIM_0mV         = 3U,

    /// DCDC trim 25mV
    SYSCNTL_DCDC_TRIM_P_25mV      = 4U,

    /// DCDC trim 50mV
    SYSCNTL_DCDC_TRIM_P_50mV      = 5U,

    /// DCDC trim 75mV
    SYSCNTL_DCDC_TRIM_P_75mV      = 6U,

    /// DCDC trim 100mV
    SYSCNTL_DCDC_TRIM_P_100mV     = 7U,
};

/// Defines the DCDC converter voltage levels (nominal voltage + trim)
typedef enum {
    /// DCDC voltage level 1.025V
    SYSCNTL_DCDC_LEVEL_1V025     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V1 << 3U) + SYSCNTL_DCDC_TRIM_N_75mV),

    /// DCDC voltage level 1.05V
    SYSCNTL_DCDC_LEVEL_1V05      = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V1 << 3U) + SYSCNTL_DCDC_TRIM_N_50mV),

    /// DCDC voltage level 1.075V
    SYSCNTL_DCDC_LEVEL_1V075     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V1 << 3U) + SYSCNTL_DCDC_TRIM_N_25mV),

    /// DCDC voltage level 1.1V
    SYSCNTL_DCDC_LEVEL_1V1       = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V1 << 3U) + SYSCNTL_DCDC_TRIM_0mV),

    /// DCDC voltage level 1.125V
    SYSCNTL_DCDC_LEVEL_1V125     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V1 << 3U) + SYSCNTL_DCDC_TRIM_P_25mV),

    /// DCDC voltage level 1.150V
    SYSCNTL_DCDC_LEVEL_1V150     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V1 << 3U) + SYSCNTL_DCDC_TRIM_P_50mV),

    /// DCDC voltage level 1.175V
    SYSCNTL_DCDC_LEVEL_1V175     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V1 << 3U) + SYSCNTL_DCDC_TRIM_P_75mV),

    /// DCDC voltage level 1.2V
    SYSCNTL_DCDC_LEVEL_1V2       = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V1 << 3U) + SYSCNTL_DCDC_TRIM_P_100mV),


    /// DCDC voltage level 1.725V
    SYSCNTL_DCDC_LEVEL_1V725     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V8 << 3U) + SYSCNTL_DCDC_TRIM_N_75mV),

    /// DCDC voltage level 1.75V
    SYSCNTL_DCDC_LEVEL_1V75      = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V8 << 3U) + SYSCNTL_DCDC_TRIM_N_50mV),

    /// DCDC voltage level 1.775V
    SYSCNTL_DCDC_LEVEL_1V775     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V8 << 3U) + SYSCNTL_DCDC_TRIM_N_25mV),

    /// DCDC voltage level 1.8V
    SYSCNTL_DCDC_LEVEL_1V8       = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V8 << 3U) + SYSCNTL_DCDC_TRIM_0mV),

    /// DCDC voltage level 1.825V
    SYSCNTL_DCDC_LEVEL_1V825     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V8 << 3U) + SYSCNTL_DCDC_TRIM_P_25mV),

    /// DCDC voltage level 1.850V
    SYSCNTL_DCDC_LEVEL_1V850     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V8 << 3U) + SYSCNTL_DCDC_TRIM_P_50mV),

    /// DCDC voltage level 1.875V
    SYSCNTL_DCDC_LEVEL_1V875     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V8 << 3U) + SYSCNTL_DCDC_TRIM_P_75mV),

    /// DCDC voltage level 1.9V
    SYSCNTL_DCDC_LEVEL_1V9       = ((SYSCNTL_DCDC_NOMINAL_LEVEL_1V8 << 3U) + SYSCNTL_DCDC_TRIM_P_100mV),


    /// DCDC voltage level 2.4025V
    SYSCNTL_DCDC_LEVEL_2V425     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_2V5 << 3U) + SYSCNTL_DCDC_TRIM_N_75mV),

    /// DCDC voltage level 2.45V
    SYSCNTL_DCDC_LEVEL_2V45      = ((SYSCNTL_DCDC_NOMINAL_LEVEL_2V5 << 3U) + SYSCNTL_DCDC_TRIM_N_50mV),

    /// DCDC voltage level 2.475V
    SYSCNTL_DCDC_LEVEL_2V475     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_2V5 << 3U) + SYSCNTL_DCDC_TRIM_N_25mV),

    /// DCDC voltage level 2.5V
    SYSCNTL_DCDC_LEVEL_2V5       = ((SYSCNTL_DCDC_NOMINAL_LEVEL_2V5 << 3U) + SYSCNTL_DCDC_TRIM_0mV),

    /// DCDC voltage level 2.525V
    SYSCNTL_DCDC_LEVEL_2V525     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_2V5 << 3U) + SYSCNTL_DCDC_TRIM_P_25mV),

    /// DCDC voltage level 2.550V
    SYSCNTL_DCDC_LEVEL_2V550     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_2V5 << 3U) + SYSCNTL_DCDC_TRIM_P_50mV),

    /// DCDC voltage level 2.575V
    SYSCNTL_DCDC_LEVEL_2V575     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_2V5 << 3U) + SYSCNTL_DCDC_TRIM_P_75mV),

    /// DCDC voltage level 2.6V
    SYSCNTL_DCDC_LEVEL_2V6       = ((SYSCNTL_DCDC_NOMINAL_LEVEL_2V5 << 3U) + SYSCNTL_DCDC_TRIM_P_100mV),


    /// DCDC voltage level 2.925V
    SYSCNTL_DCDC_LEVEL_2V925     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_3V0 << 3U) + SYSCNTL_DCDC_TRIM_N_75mV),

    /// DCDC voltage level 2.95V
    SYSCNTL_DCDC_LEVEL_2V95      = ((SYSCNTL_DCDC_NOMINAL_LEVEL_3V0 << 3U) + SYSCNTL_DCDC_TRIM_N_50mV),

    /// DCDC voltage level 2.975V
    SYSCNTL_DCDC_LEVEL_2V975     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_3V0 << 3U) + SYSCNTL_DCDC_TRIM_N_25mV),

    /// DCDC voltage level 3.0V
    SYSCNTL_DCDC_LEVEL_3V0       = ((SYSCNTL_DCDC_NOMINAL_LEVEL_3V0 << 3U) + SYSCNTL_DCDC_TRIM_0mV),

    /// DCDC voltage level 3.025V
    SYSCNTL_DCDC_LEVEL_3V025     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_3V0 << 3U) + SYSCNTL_DCDC_TRIM_P_25mV),

    /// DCDC voltage level 3.050V
    SYSCNTL_DCDC_LEVEL_3V050     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_3V0 << 3U) + SYSCNTL_DCDC_TRIM_P_50mV),

    /// DCDC voltage level 3.075V
    SYSCNTL_DCDC_LEVEL_3V075     = ((SYSCNTL_DCDC_NOMINAL_LEVEL_3V0 << 3U) + SYSCNTL_DCDC_TRIM_P_75mV),

    /// DCDC voltage level 3.1V
    SYSCNTL_DCDC_LEVEL_3V1       = ((SYSCNTL_DCDC_NOMINAL_LEVEL_3V0 << 3U) + SYSCNTL_DCDC_TRIM_P_100mV),
} syscntl_dcdc_level_t;

/// DCDC converter state
typedef struct
{
    /// DCDC converter voltage level
    syscntl_dcdc_level_t level;

    /// DCDC converter status
    bool status;
}syscntl_dcdc_state_t;

/// Defines the XTAL32M amplitude regulator mode
typedef enum {
    /// Normal mode (following radio enable)
    XTAL32M_AMP_REG_NORMAL      = 0,

    /// Continues tracking mode
    XTAL32M_AMP_REG_TRACKING    = 1,

    /// Hold mode
    XTAL32M_AMP_REG_HOLD        = 2,
} xtal32m_amp_reg_t;

/// Defines the POR on VBAT HIGH/LOW Mask mode
typedef enum {
    /// Mask OFF
    POR_VBAT_MASK_OFF           = 0,

    /// Mask ON
    POR_VBAT_MASK_ON            = 1,
} por_vbat_mask_t;
#else
/// Nominal VBAT3V output voltage of the boost converter
enum SYSCNTL_DCDC_VBAT3V_LEVEL
{
    /// DCDC nominal voltage level 2.4V
    SYSCNTL_DCDC_VBAT3V_LEVEL_2V4    = 4,

    /// DCDC nominal voltage level 2.5V
    SYSCNTL_DCDC_VBAT3V_LEVEL_2V5    = 5,

    /// DCDC nominal voltage level 2.62V
    SYSCNTL_DCDC_VBAT3V_LEVEL_2V62   = 6,

    /// DCDC nominal voltage level 2.76V
    SYSCNTL_DCDC_VBAT3V_LEVEL_2V76   = 7,
};
#endif

/*
 * DEFINITIONS
 *****************************************************************************************
 */

#if defined (__DA14531__)
/// Clock AMBA register reserved bits
#define CLK_AMBA_REG_RSV3_RSV2       (0x000C)
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Sets AMBA clocks to minimum frequency
 ****************************************************************************************
 */
void syscntl_use_lowest_amba_clocks(void);

/**
 ****************************************************************************************
 * @brief Sets AMBA clocks to maximum frequency.
 ****************************************************************************************
 */
void syscntl_use_highest_amba_clocks(void);

#if defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief Store debugger configuration to CLK_AMBA_REG reserved bits 3 and 2.
 * @note DA1585/586: Dedicated pins are used for the SWD interface.
 *       DA14531:    The SWD interface pins are configurable.
 * @param[in] cfg    The debugger pins configurarion
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void syscntl_store_debugger_cfg(uint8_t cfg)
{
    SetBits16(CLK_AMBA_REG, CLK_AMBA_REG_RSV3_RSV2, cfg);
}
#endif

/**
 ****************************************************************************************
 * @brief Load debugger configuration stored in CLK_AMBA_REG reserved bits 3 and 2.
 * @note DA1585/586: Dedicated pins are used for the SWD interface.
 *       DA14531:    The SWD interface pins are configurable.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void syscntl_load_debugger_cfg(void)
{
#if defined (__DA14531__)
    uint16_t rsv3_rsv2 = GetBits16(CLK_AMBA_REG, CLK_AMBA_REG_RSV3_RSV2);
    SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, rsv3_rsv2);
#else
    SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 1);
#endif
}

#if !defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief Sets the Nominal VBAT3V output voltage of the boost converter.
 * @param[in] level DCDC VBAT3V output voltage level (see enum SYSCNTL_DCDC_VBAT3V_LEVEL).
 ****************************************************************************************
 */
void syscntl_set_dcdc_vbat3v_level(enum SYSCNTL_DCDC_VBAT3V_LEVEL level);
#endif

#if defined (__DA14531__)

/**
 ****************************************************************************************
 * @brief Gets the voltage level of DCDC converter.
 * @return the voltage level of DCDC converter (nominal level + trim value)
 ****************************************************************************************
 */
syscntl_dcdc_level_t syscntl_dcdc_get_level(void);

/**
 ****************************************************************************************
 * @brief Sets the DCDC converter voltage level on hardware (nominal + trim value).
 * @param[in] level     The voltage level to be set.
 ****************************************************************************************
 */
void syscntl_dcdc_set_level(syscntl_dcdc_level_t level);

/**
 ****************************************************************************************
 * @brief Disables DCDC converter. In Buck mode enables the LDO_LOW.
 * @return Error code
 ****************************************************************************************
 */
int8_t syscntl_dcdc_turn_off(void);

/**
 ****************************************************************************************
 * @brief Enables DCDC converter in the provided voltage level in order to supply
 *        VBAT_HIGH in Boost mode.
 * @param[in] level The voltage level of DCDC converter (nominal level + trim value).
 * @return Error code
 ****************************************************************************************
 */
int8_t syscntl_dcdc_turn_on_in_boost(syscntl_dcdc_level_t level);

/**
 ****************************************************************************************
 * @brief Enables DCDC converter in the provided voltage level in order to supply
 *        VBAT_LOW and disables LDO_LOW.
 * @param[in] level The voltage level of DCDC converter (nominal level + trim value).
 * @return Error code
 ****************************************************************************************
 */
int8_t syscntl_dcdc_turn_on_in_buck(syscntl_dcdc_level_t level);

/**
 ****************************************************************************************
 * @brief Sets the XTAL32M amplitude regulator mode.
 * @param[in] mode      Normal, Tracking or Hold mode.
 ****************************************************************************************
 */
void syscntl_cfg_xtal32m_amp_reg(xtal32m_amp_reg_t mode);

/**
 ****************************************************************************************
 * @brief Configures the POR_VBAT_HIGH circuit.
 * @param[in] mask       POR_VBAT_HIGH Mask ON/OFF
 ****************************************************************************************
 */
void syscntl_por_vbat_high_cfg(por_vbat_mask_t mask);

/**
 ****************************************************************************************
 * @brief Configures the POR_VBAT_LOW circuit.
 * @param[in] mask       POR_VBAT_LOW Mask ON/OFF
 ****************************************************************************************
 */
void syscntl_por_vbat_low_cfg(por_vbat_mask_t mask);
#endif

#endif // SYSCNTL_H_

///@}
///@}
