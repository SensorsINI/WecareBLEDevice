/**
 ****************************************************************************************
 *
 * @file syscntl.c
 *
 * @brief System controller driver.
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "datasheet.h"
#include "syscntl.h"

#if defined (__DA14531__)
#include <stdbool.h>
#include "hw_otpc.h"
#include "arch.h"
#endif

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

#if defined (__DA14531__)
/// DCDC converter state
syscntl_dcdc_state_t syscntl_dcdc_state;
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void syscntl_use_lowest_amba_clocks(void)
{
    SetBits16(CLK_AMBA_REG, PCLK_DIV, 3);   // lowest is 250KHz (div 8, source is @2MHz)
    SetBits16(CLK_AMBA_REG, HCLK_DIV, 3);   // lowest is 2MHz (div 8, source is @16MHz)
}

void syscntl_use_highest_amba_clocks(void)
{
    SetBits16(CLK_AMBA_REG, PCLK_DIV, 0);
    SetBits16(CLK_AMBA_REG, HCLK_DIV, 0);
}

#if defined (__DA14531__)
void syscntl_cfg_xtal32m_amp_reg(xtal32m_amp_reg_t mode)
{
    SetBits16(XTAL32M_TESTCTRL0_REG, CORE_HOLD_AMP_REG_OVERRIDE, (uint8_t) mode);
}

void syscntl_por_vbat_high_cfg(por_vbat_mask_t mask)
{
    // Enable POR_VBAT_HIGH
    SetBits16(POWER_CTRL_REG, POR_VBAT_HIGH_DISABLE, 0);

    // Set POR_VBAT_HIGH mask mode
    SetBits(POWER_AON_CTRL_REG, POR_VBAT_HIGH_RST_MASK, mask);
}

void syscntl_por_vbat_low_cfg(por_vbat_mask_t mask)
{
    // Enable POR_VBAT_LOW
    SetBits16(POWER_CTRL_REG, POR_VBAT_LOW_DISABLE, 0);

    // Set POR_VBAT_LOW mask mode
    SetBits(POWER_AON_CTRL_REG, POR_VBAT_LOW_RST_MASK, mask);
}

/**
 ****************************************************************************************
 * @brief Configures and enables DCDC converter in the provided voltage level.
 * @param[in] level The voltage level of DCDC converter (nominal level + trim value).
 ****************************************************************************************
 */
#if (!USE_POWER_MODE_BYPASS)
static void dcdc_cfg(syscntl_dcdc_level_t level)
{
    // Set DCDC converter nominal voltage level
    SetBits16(POWER_LEVEL_REG, DCDC_LEVEL, level >> 3U);

    // Set DCDC converter trim value
    SetBits16(POWER_LEVEL_REG, DCDC_TRIM, level & 0x7);

    // Enable the DCDC converter
    SetBits16(DCDC_CTRL_REG, DCDC_ENABLE, 1);

    // Wait for the indication that VBAT_LOW (buck mode) or VBAT_HIGH (boost mode) is OK
    while (!GetBits(ANA_STATUS_REG, DCDC_OK));
}
#endif

syscntl_dcdc_level_t syscntl_dcdc_get_level(void)
{
    uint16_t dcdc_level = GetBits16(POWER_LEVEL_REG, DCDC_LEVEL);
    uint16_t dcdc_trim = GetBits16(POWER_LEVEL_REG, DCDC_TRIM);
    return ((syscntl_dcdc_level_t) ((dcdc_level << 3U) + dcdc_trim));
}

void syscntl_dcdc_set_level(syscntl_dcdc_level_t level)
{
    // Set DCDC converter nominal voltage level
    SetBits16(POWER_LEVEL_REG, DCDC_LEVEL, level >> 3U);

    // Set DCDC converter trim value
    SetBits16(POWER_LEVEL_REG, DCDC_TRIM, level & 0x7);
}

int8_t syscntl_dcdc_turn_off(void)
{
#if (USE_POWER_MODE_BYPASS)
    return SYSCNTL_DCDC_ERR_IN_BYPASS;
#else
    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED))
    {
        if (hw_otpc_is_dcdc_reserved())
        {
            return SYSCNTL_DCDC_ERR_RESERVED_BY_OTP;
        }
        if ((GetBits16(POWER_CTRL_REG, POR_VBAT_HIGH_DISABLE) == 0x0) &&
            (GetBits16(POWER_AON_CTRL_REG, POR_VBAT_HIGH_RST_MASK) == 0x0))
        {
            return SYSCNTL_DCDC_ERR_POR_TRIGGER;
        }
    }
    else
    {
        // Enable LDO_LOW
        SetBits16(POWER_CTRL_REG, LDO_LOW_CTRL_REG, 2);
    }

    // Disable the DCDC
    SetBits16(DCDC_CTRL_REG, DCDC_ENABLE, 0);

    return SYSCNTL_DCDC_ERR_NO_ERROR;
#endif
}

int8_t syscntl_dcdc_turn_on_in_boost(syscntl_dcdc_level_t level)
{
#if (USE_POWER_MODE_BYPASS)
    return SYSCNTL_DCDC_ERR_IN_BYPASS;
#else
    if (!GetBits16(ANA_STATUS_REG, BOOST_SELECTED))
    {
        return SYSCNTL_DCDC_ERR_NOT_IN_BOOST;
    }
    if (hw_otpc_is_dcdc_reserved() && (level < syscntl_dcdc_get_level()))
    {
        return SYSCNTL_DCDC_ERR_RESERVED_BY_OTP;
    }
    // POR is generated if VBAT_HIGH is below 1.66V
    if ((GetBits16(POWER_CTRL_REG, POR_VBAT_HIGH_DISABLE) == 0x0) &&
        (GetBits16(POWER_AON_CTRL_REG, POR_VBAT_HIGH_RST_MASK) == 0x0) &&
        (level < SYSCNTL_DCDC_LEVEL_1V725))
    {
        return SYSCNTL_DCDC_WARNING_POR_TRIGGER;
    }

    syscntl_dcdc_state.level = level;
    syscntl_dcdc_state.status = true;
    dcdc_cfg(level);

    return SYSCNTL_DCDC_ERR_NO_ERROR;
#endif
}

int8_t syscntl_dcdc_turn_on_in_buck(syscntl_dcdc_level_t level)
{
#if (USE_POWER_MODE_BYPASS)
    return SYSCNTL_DCDC_ERR_IN_BYPASS;
#else
    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED))
    {
        return SYSCNTL_DCDC_ERR_NOT_IN_BUCK;
    }

    dcdc_cfg(level);

    // Disable LDO_LOW
    SetBits16(POWER_CTRL_REG, LDO_LOW_CTRL_REG, 1);

    return SYSCNTL_DCDC_ERR_NO_ERROR;
#endif
}

#else
void syscntl_set_dcdc_vbat3v_level(enum SYSCNTL_DCDC_VBAT3V_LEVEL level)
{
    SetBits16(DCDC_CTRL2_REG, DCDC_VBAT3V_LEV, level);
}
#endif
