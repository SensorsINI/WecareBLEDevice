/**
 ****************************************************************************************
 *
 * @file arch_hibernation.c
 *
 * @brief Implementation of the hibernation-related API
 *
 * Copyright (C) 2019-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#if defined (__DA14531__)

#include <stdbool.h>
#include "datasheet.h"
#include "arch_hibernation.h"
#include "arch.h"
#include "arch_api.h"
#include "otp_cs.h"
#include "arch_ram.h"
#include "arch_system.h"
#include "system_library.h"

#include "reg_blecore.h"
#include "rwip.h"
#include "ke_event.h"

#if (WLAN_COEX_ENABLED)
#include "wlan_coex.h"
#endif

#include "user_periph_setup.h"

/// Structure containing info about debugger configuration and OTP CS registers
/// normally handled by the Booter - defined in otp_cs.c
extern otp_cs_booter_val_t booter_val;

uint16_t clk_freq_trim_reg_backup   __SECTION_ZERO("retention_mem_area0");
uint16_t clk_ctrl_reg_backup        __SECTION_ZERO("retention_mem_area0");
uint16_t clk_rcx_reg_backup         __SECTION_ZERO("retention_mem_area0");
uint16_t clk_amba_reg_backup        __SECTION_ZERO("retention_mem_area0");

extern int8_t last_temp;

extern void init_pwr_and_clk_ble(void);
extern void patched_ble_regs_push(void);
extern void patched_ble_regs_pop(void);
extern void rf_reinit(void);
__WEAK extern void _store_primask(void);
__WEAK extern void _store_core_registers(void);

/**
 ****************************************************************************************
 * @brief Set required LDO_RET_TRIM value (VDD clamp level setting for hibernation mode)
 *        in POWER_AON_CTRL_REG. This value depends on whether RAM is retained
 *        (all 48 KB) or not and the working temperature range.
 ****************************************************************************************
 */
static void set_ldo_ret_trim(void)
{
// Temperature range (-40C to +105C)
#if (USE_HIGH_TEMPERATURE)
    SetBits(POWER_AON_CTRL_REG, LDO_RET_TRIM, ((GetWord16(RAM_PWR_CTRL_REG) & ALL_RAM_BLOCKS_OFF) == ALL_RAM_BLOCKS_OFF) ? 0x09 : 0x08);
// Temperature range (-40C to +85C)
#elif (USE_EXT_TEMPERATURE)
    SetBits(POWER_AON_CTRL_REG, LDO_RET_TRIM, ((GetWord16(RAM_PWR_CTRL_REG) & ALL_RAM_BLOCKS_OFF) == ALL_RAM_BLOCKS_OFF) ? 0x0B : 0x0A);
// Temperature range (-40C to +60C)
#elif (USE_MID_TEMPERATURE)
    SetBits(POWER_AON_CTRL_REG, LDO_RET_TRIM, ((GetWord16(RAM_PWR_CTRL_REG) & ALL_RAM_BLOCKS_OFF) == ALL_RAM_BLOCKS_OFF) ? 0x0D : 0x0C);
 // Temperature range (-40C to +40C)
#else
    SetBits(POWER_AON_CTRL_REG, LDO_RET_TRIM, ((GetWord16(RAM_PWR_CTRL_REG) & ALL_RAM_BLOCKS_OFF) == ALL_RAM_BLOCKS_OFF) ? 0x0E : 0x0D);
#endif
}

void arch_set_hibernation(uint8_t wkup_mask,
                          pd_sys_down_ram_t ram1,
                          pd_sys_down_ram_t ram2,
                          pd_sys_down_ram_t ram3,
                          remap_addr0_t remap_addr0,
                          bool pad_latch_en)
{
    // Check GPIOs wake up mask
    wkup_mask >>= 1;
    ASSERT_WARNING((wkup_mask & ~0x1F) == 0);

    // Assert warning if RAM 1 or RAM 3 is remapped to address 0 and the respective block is not retained.
    ASSERT_WARNING(remap_addr0 == REMAP_ADDR0_TO_RAM1 ? (ram1 == PD_SYS_DOWN_RAM_ON) : \
                   remap_addr0 == REMAP_ADDR0_TO_RAM3 ? (ram3 == PD_SYS_DOWN_RAM_ON) : true);

    // Stop watchdog timer
    wdg_freeze();

    // Disable Interrupts
    GLOBAL_INT_STOP()

    // Clear all pending interrupts
    NVIC->ICPR[0] = 0xFFFFFFFFUL;

    // Store the debugger configuration
    booter_val.dbg_cfg = GetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE);

    // Debugger must be disabled before entering a sleep mode. Wait until debugger has been disabled.
    while (GetBits16(SYS_STAT_REG, DBG_IS_UP) != 0);
    SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0);

    // Set the wake up pins
    SetBits16(HIBERN_CTRL_REG, HIBERN_WKUP_MASK, wkup_mask);

    // Check the output of the clockless wakeup XOR tree to determine the wake up polarity
    if (GetBits16(ANA_STATUS_REG, CLKLESS_WAKEUP_STAT) == 0)
    {
        uint16_t hibern_wkup_polarity = GetBits16(HIBERN_CTRL_REG, HIBERN_WKUP_POLARITY);
        SetBits16(HIBERN_CTRL_REG, HIBERN_WKUP_POLARITY, hibern_wkup_polarity == 0 ? 1 : 0);
    }

    while (GetBits16(ANA_STATUS_REG, CLKLESS_WAKEUP_STAT) == 0);

    // Set hibernation sleep mode
    SetBits16(HIBERN_CTRL_REG, HIBERNATION_ENABLE, 1);
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);

    // Configure the state of RAM blocks during hibernation mode
    SetBits16(RAM_PWR_CTRL_REG, RAM1_PWR_CTRL, ram1);
    SetBits16(RAM_PWR_CTRL_REG, RAM2_PWR_CTRL, ram2);
    SetBits16(RAM_PWR_CTRL_REG, RAM3_PWR_CTRL, ram3);

    // Remap address 0
    SetBits16(SYS_CTRL_REG, REMAP_ADR0, remap_addr0);

#if (WLAN_COEX_ENABLED)
    // Drive to inactive state the pin used for the BLE event in progress signal
    wlan_coex_set_ble_eip_pin_inactive();
#endif

    // Enable/Disable latching of pads state during sleep
    SetBits16(PAD_LATCH_REG, PAD_LATCH_EN, pad_latch_en);

    // Disable the TLS (Transparent Light Sleep) core feature
    SetBits16(RAM_LPMX_REG, RAMx_LPMX, 7);

    // Set required LDO_RET_TRIM value
    set_ldo_ret_trim();

    // Disable the testmode comparator
    SetBits(POWER_AON_CTRL_REG, FORCE_RUNNING_COMP_DIS, 1);

    // Perform the following steps when in boost (or bypass) mode
    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED) == 0x1)
    {
        // Force connection between VBAT_HIGH and VBAT_LOW
        SetBits(POWER_AON_CTRL_REG, VBAT_HL_CONNECT_RES_CTRL, 2);

        // Do not charge VBAT_HIGH in boost mode
        SetBits(POWER_AON_CTRL_REG, CHARGE_VBAT_DISABLE, 1);
    }
    // Perform the following steps, when in buck mode
    else
    {
        // Set automatic connection control between VBAT_HIGH and VBAT_LOW
        SetBits(POWER_AON_CTRL_REG, VBAT_HL_CONNECT_RES_CTRL, 3);
    }

    // Disable the POR triggered from VBAT_HIGH voltage level sensing
    SetBits(POWER_AON_CTRL_REG, POR_VBAT_HIGH_RST_MASK, 1);

    // Enable the POR triggered from VBAT_LOW voltage level sensing
    SetBits(POWER_AON_CTRL_REG, POR_VBAT_LOW_RST_MASK, 0);

    // Set for proper RCX operation
    SetBits16(GP_DATA_REG, 0x60, 2);

    __NOP();
    __NOP();
    __NOP();

    // Enter Hibernation Mode
    __WFI();
}

void arch_set_stateful_hibernation(uint8_t wkup_mask,
                                   pd_sys_down_ram_t ram1,
                                   pd_sys_down_ram_t ram2,
                                   pd_sys_down_ram_t ram3,
                                   stateful_hibern_remap_addr0_t remap_addr0,
                                   bool pad_latch_en)
{
    // Check GPIOs wake up mask
    wkup_mask >>= 1;
    ASSERT_WARNING((wkup_mask & ~0x1F) == 0);

    // Assert warning if RAM 1 or RAM 3 is remapped to address 0 and the respective block is not retained.
    ASSERT_WARNING(remap_addr0 == STATEFUL_HIBERN_REMAP_ADDR0_TO_RAM1 ? (ram1 == PD_SYS_DOWN_RAM_ON) : \
                   remap_addr0 == STATEFUL_HIBERN_REMAP_ADDR0_TO_RAM3 ? (ram3 == PD_SYS_DOWN_RAM_ON) : true);

    // Stop watchdog timer
    wdg_freeze();

    // Store PRIMASK and disable interrupts
    _store_primask();

#if !defined (__NON_BLE_EXAMPLE__)
    // Disable the BLE core clocks
    SetBits16(CLK_RADIO_REG, BLE_ENABLE, 0);

    // Apply HW reset to BLE_Timers
    SetBits16(CLK_RADIO_REG, BLE_LP_RESET, 1);

    patched_ble_regs_push();
#endif // __NON_BLE_EXAMPLE__

    // Store the CLK_FREQ_TRIM_REG, CLK_CTRL_REG and CLK_RCX_REG values
    clk_freq_trim_reg_backup = GetWord16(CLK_FREQ_TRIM_REG);
    clk_ctrl_reg_backup      = GetWord16(CLK_CTRL_REG);
    clk_rcx_reg_backup       = GetWord16(CLK_RCX_REG);

#if !defined (__NON_BLE_EXAMPLE__)
    // Flush all pending events
    ke_event_flush();
#endif

    // Store the debugger configuration
    booter_val.dbg_cfg  = GetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE);
    clk_amba_reg_backup = GetWord16(CLK_AMBA_REG);

    // Debugger must be disabled before entering a sleep mode. Wait until debugger has been disabled.
    while (GetBits16(SYS_STAT_REG, DBG_IS_UP) != 0);
    SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0);

    // Set the wake up pins
    SetBits16(HIBERN_CTRL_REG, HIBERN_WKUP_MASK, wkup_mask);

    // Check the output of the clockless wakeup XOR tree to determine the wake up polarity
    if (GetBits16(ANA_STATUS_REG, CLKLESS_WAKEUP_STAT) == 0)
    {
        uint16_t hibern_wkup_polarity = GetBits16(HIBERN_CTRL_REG, HIBERN_WKUP_POLARITY);
        SetBits16(HIBERN_CTRL_REG, HIBERN_WKUP_POLARITY, hibern_wkup_polarity == 0 ? 1 : 0);
    }

    while (GetBits16(ANA_STATUS_REG, CLKLESS_WAKEUP_STAT) == 0);

    // Set hibernation sleep mode
    SetBits16(HIBERN_CTRL_REG, HIBERNATION_ENABLE, 1);
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);

    // Configure the state of RAM blocks during hibernation mode
    SetBits16(RAM_PWR_CTRL_REG, RAM1_PWR_CTRL, ram1);
    SetBits16(RAM_PWR_CTRL_REG, RAM2_PWR_CTRL, ram2);
    SetBits16(RAM_PWR_CTRL_REG, RAM3_PWR_CTRL, ram3);

    // Remap address 0
    SetBits16(SYS_CTRL_REG, REMAP_ADR0, remap_addr0);

#if (WLAN_COEX_ENABLED)
    // Drive to inactive state the pin used for the BLE event in progress signal
    wlan_coex_set_ble_eip_pin_inactive();
#endif

    // Enable/Disable latching of pads state during sleep
    SetBits16(PAD_LATCH_REG, PAD_LATCH_EN, pad_latch_en);

    // Disable the TLS (Transparent Light Sleep) core feature
    SetBits16(RAM_LPMX_REG, RAMx_LPMX, 7);

    // Set required LDO_RET_TRIM value
    set_ldo_ret_trim();

    // Disable the testmode comparator
    SetBits(POWER_AON_CTRL_REG, FORCE_RUNNING_COMP_DIS, 1);

    // Perform the following steps when in boost (or bypass) mode
    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED) == 0x1)
    {
        // Force connection between VBAT_HIGH and VBAT_LOW
        SetBits(POWER_AON_CTRL_REG, VBAT_HL_CONNECT_RES_CTRL, 2);

        // Do not charge VBAT_HIGH in boost mode
        SetBits(POWER_AON_CTRL_REG, CHARGE_VBAT_DISABLE, 1);
    }
    // Perform the following steps, when in buck mode
    else
    {
        // Set automatic connection control between VBAT_HIGH and VBAT_LOW
        SetBits(POWER_AON_CTRL_REG, VBAT_HL_CONNECT_RES_CTRL, 3);
    }

    // Disable the POR triggered from VBAT_HIGH voltage level sensing
    SetBits(POWER_AON_CTRL_REG, POR_VBAT_HIGH_RST_MASK, 1);

    // Enable the POR triggered from VBAT_LOW voltage level sensing
    SetBits(POWER_AON_CTRL_REG, POR_VBAT_LOW_RST_MASK, 0);

    // Set for proper RCX operation
    SetBits16(GP_DATA_REG, 0x60, 2);

    // Enter stateful hibernation mode; function storing the ARM core state -
    // written in assembly
    _store_core_registers();
}

/**
 ****************************************************************************************
 * @brief Restores DA14531 register values.
 ****************************************************************************************
 */
void arch_hibernation_restore(void)
{
    // Enable the TLS (Transparent Light Sleep) core feature to minimize RAM
    // power consumption
    SetBits16(RAM_LPMX_REG, RAMx_LPMX, 0);

    // Apply the BANDGAP_REG value
    SetWord16(BANDGAP_REG, booter_val.bandgap_reg);
    // Apply the CLK_RCX_REG value
    SetWord16(CLK_RCX_REG, clk_rcx_reg_backup);
    // Apply the CLK_RC32M_REG value
    SetWord16(CLK_RC32M_REG, booter_val.clk_rc32m_reg);
    // Apply the CLK_RC32K_REG value
    SetWord16(CLK_RC32K_REG, booter_val.clk_rc32k_reg);
    // Apply the debugger configuration
    SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, booter_val.dbg_cfg);
    // Apply the CLK_AMBA_REG value
    SetWord16(CLK_AMBA_REG, clk_amba_reg_backup);

    // Stop watchdog timer
    wdg_freeze();

    // Ensure that XTAL32M will be enabled; switch to XTAL32M will be done
    // conditionally, later
    SetBits16(CLK_CTRL_REG, XTAL32M_DISABLE, 0);

    // Configure XTAL32M
    SetBits16(TRIM_CTRL_REG, XTAL_TRIM_SELECT, 2);  // only CLK_FREQ_TRIM_REG will be used for XTAL32M trimming
    SetBits16(XTAL32M_START_REG, XTAL32M_START, 0);
    SetBits16(XTAL32M_CTRL0_REG, CORE_CUR_SET, 2);

    if ((clk_ctrl_reg_backup & RUNNING_AT_XTAL32M) == RUNNING_AT_XTAL32M)
    {
        set_xtal32m_trim_value(clk_freq_trim_reg_backup);
    }

#if (USE_RANGE_EXT)
    // Register range extender driver API
    range_ext_init(&range_ext);
#endif

    // This is a wakeup from hibernation
    // Disable Hibernation
    SetBits16(HIBERN_CTRL_REG, HIBERNATION_ENABLE, 0);

    // Set clamp output below LDO_CORE level
    SetBits16(POWER_AON_CTRL_REG, LDO_RET_TRIM, 0xF);

    // Enable LDO_CORE_RET
    SetBits16(POWER_CTRL_REG, LDO_CORE_RET_ENABLE, 1);

    // Recommended settings copied from rf_531.c
    SetBits16(DCDC_CTRL_REG, DCDC_CLK_DIV, 0x0);
    SetBits16(POWER_AON_CTRL_REG, FORCE_RUNNING_COMP_DIS, 0x1);
    SetBits16(POWER_CTRL_REG, POR_VBAT_HIGH_HYST_DIS, 0);    // Sets the POR VBAT HIGH threshold at 1.66V
    SetBits16(POWER_LEVEL_REG, LDO_LOW_TRIM, 3);

    // Perform the following steps when in Boost (or Bypass) mode
    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED))
    {
        SetBits16(DCDC_CTRL_REG, DCDC_ILIM_MIN, 0x4);
        // Set ILIM_MAX to half its maximum value to limit inrush current
        // during wake-up or cold boot for high-ohmic batteries.
        SetBits16(DCDC_CTRL_REG, DCDC_ILIM_MAX, 0x8);

        // Disable the POR triggered from VBAT_HIGH voltage level sensing
        SetBits16(POWER_AON_CTRL_REG, POR_VBAT_HIGH_RST_MASK, 1);

        // Enable the POR triggered from VBAT_LOW voltage level sensing
        SetBits16(POWER_AON_CTRL_REG, POR_VBAT_LOW_RST_MASK, 0);

        // Disable the DC/DC. It has been left enabled by the booter.
        // - It has to be enabled for OTP access and GPIOs.
        SetBits16(DCDC_CTRL_REG, DCDC_ENABLE, 0);

        // Charge VBAT_HIGH in boost mode
        SetBits16(POWER_AON_CTRL_REG, CHARGE_VBAT_DISABLE, 0);
    }
    // Perform the following steps, when in Buck mode
    else
    {
        SetBits16(DCDC_CTRL_REG, DCDC_ILIM_MIN, 0x6);
        SetBits16(DCDC_CTRL_REG, DCDC_ILIM_MAX, 0xF);

        // Enable the POR triggered from VBAT_HIGH voltage level sensing
        SetBits16(POWER_AON_CTRL_REG, POR_VBAT_HIGH_RST_MASK, 0);

        // Enable the POR triggered from VBAT_LOW voltage level sensing
        SetBits16(POWER_AON_CTRL_REG, POR_VBAT_LOW_RST_MASK, 0);

        // Set DC/DC level to 1.1V
        SetBits16(POWER_LEVEL_REG, DCDC_LEVEL, 0);

        // Enable the DC/DC
        SetBits16(DCDC_CTRL_REG, DCDC_ENABLE, 1);

        while(GetBits16(ANA_STATUS_REG, DCDC_OK) != 1);
    }

    // Connection between VBAT_HIGH and VBAT_LOW: Automatic connection control
    SetBits16(POWER_AON_CTRL_REG, VBAT_HL_CONNECT_RES_CTRL, 3);

    // Turn off LDO_LOW.
    // Boost (or Bypass) mode: it must be turned off always.
    // Buck mode: it must be turned off since DC/DC is used for VBAT_LOW. It has to be turned on, as late as possible, before sleep.
    SetBits16(POWER_CTRL_REG, LDO_LOW_CTRL_REG, 1);

    // Set for proper RCX operation
    SetBits16(GP_DATA_REG, 0x60, 1);
#if (DISABLE_QUADEC_ON_START_UP)
    // Clear quadec pending interrupt - mask quadec interrupt
    SetWord16(QDEC_CTRL_REG, QDEC_IRQ_STATUS);

    // Close peripheral clock
    SetBits16(CLK_PER_REG, QUAD_ENABLE, 0);

    // Clear respective NVIC entry
    NVIC_ClearPendingIRQ(WKUP_QUADEC_IRQn);
#endif

#if !defined (__NON_BLE_EXAMPLE__)
    last_temp = 127;
#endif

    // Peripheral initialization
    periph_init();

#if !defined (__NON_BLE_EXAMPLE__)
    /*
     ************************************************************************************
     * BLE initialization
     ************************************************************************************
     */
    init_pwr_and_clk_ble();

    // Get saved BLE registers back
    patched_ble_regs_pop();

#if (USE_RANGE_EXT)
    // Enable range extender
    range_ext.enable(MAX_POWER, NULL);
#endif

#ifdef DG_DBG_DIAG_PORT
    dbg_init_diag_port();
#endif

    // Re-enable external wake-up by default
    // NOTE: Derived from rwip_wakeup(), lld_sleep_wakeup() in particular.
    ble_extwkupdsb_setf(0);

    // Derived from dlg_slp_isr()
    // Radio subsystem initialization
    rf_reinit();

    // Start rcx calibration if selected as LP clock
    rcx20_calibrate();

    // Check time and temperature to run radio calibration
    conditionally_run_radio_cals();

#endif // (__NON_BLE_EXAMPLE__)

#if defined (CFG_RF_DIAG_INT)
    // Enable RF DIAG interrupts
    enable_rf_diag_irq(RF_DIAG_IRQ_MODE_RXTX); // argument is unused for DA14531
#endif

    // Restore ARM Sleep mode
    // Reset SCR[2]=SLEEPDEEP bit else the mode=idle __WFI() will cause a deep sleep
    // instead of a processor halt
    SCB->SCR &= ~(1<<2);
}

#endif // __DA14531__
