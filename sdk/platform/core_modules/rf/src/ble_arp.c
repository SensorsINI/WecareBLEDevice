/**
 ****************************************************************************************
 *
 * @file ble_arp.c
 *
 * @brief DA14531 RF driver
 *
 * Copyright (C) 2018-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#if defined (__DA14531__)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"        // RW SW configuration

#include <string.h>             // for memcpy
#include "co_utils.h"           // common utility definition
#include "co_math.h"            // common math functions
#include "co_endian.h"          // endian definitions
#include "rf.h"                 // RF interface
#include "plf.h"                // Platform functions

#include "rwip.h"               // for RF API structure definition
#include "arch.h"

#if (BLE_EMB_PRESENT)
#include "reg_blecore.h"        // ble core registers
#include "reg_ble_em_cs.h"      // control structure definitions
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
#include "lc_epc.h"             // Enhanced Power Control definitions
#include "reg_btcore.h"         // bt core registers
#include "reg_bt_em_cs.h"       // control structure definitions
#endif //BT_EMB_PRESENT

#include "_reg_common_em_et.h"
#include "em_map.h"
#include "rf_531.h"
#include "syscntl.h"
#include "otp_cs.h"
#include "arch_system.h"
#include "lld.h"

#if defined (CFG_ENHANCED_TX_PWR_CTRL)
#include "lld_evt.h"
#endif

#include "da1458x_scatter_config.h"     // ble_base value

static void ble_update_arp(void);
static void ble_update_arp_adpll_div_val(void);
static void update_radiopwrupdn_pref_setting(void);
static uint8_t rf_txpwr_dbm_get(uint8_t conidx, uint8_t req_pwr_level);

uint32_t rf_reg_rd (uint16_t address);
void rf_reg_wr (uint16_t address, uint32_t data);

/*
 * DEFINES
 ****************************************************************************************
 */
#define ARP_TX_DELAY   (10)
#define ARP_RX_DELAY   (10)

/*****************************************************************************************
 * @brief Ripple specific read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 ***************************************************************************************
 */

uint32_t rf_reg_rd (uint16_t address)
{
    return 0;
}

/*****************************************************************************************
 * @brief Ripple specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************
 */
void rf_reg_wr (uint16_t address, uint32_t data)
{

}


/**
 *****************************************************************************************
 * @brief Init RF sequence after reset.
 *****************************************************************************************
 */
static void rf_reset(void)
{

}

#ifdef CFG_BT
static void rf_txpwr_max(uint8_t link_id)
{
}

/**
 *****************************************************************************************
 * @brief Increase the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be increased
 *
 * @return true when maximum power is reached, false otherwise
 *****************************************************************************************
 */
static bool rf_txpwr_inc(uint8_t link_id)
{
}
/**
 *****************************************************************************************
 * @brief Increase the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be increased
 *
 * @return true when maximum power is reached, false otherwise
 *****************************************************************************************
 */
static bool rf_txpwr_epc_inc(uint8_t link_id)
{
}
/**
 *****************************************************************************************
 * @brief Decrease the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be decreased
 *
 * @return true when minimum power is reached, false otherwise
 *****************************************************************************************
 */
static bool rf_txpwr_dec(uint8_t link_id)
{
}

/**
 *****************************************************************************************
 * @brief Decrease the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be decreased
 *
 * @return true when minimum power is reached, false otherwise
 *****************************************************************************************
 */
static bool rf_txpwr_epc_dec(uint8_t link_id)
{
}


/**
 ****************************************************************************************
 * @brief Execute the EPC request received from the peer device.
 *
 * @param[in] link_id     Link Identifier
 * @param[in] action      Increase, decrease or go to maximum
 *
 * @return Tx power status depending the modulation
 *
 ****************************************************************************************
 */
static uint8_t rf_txpwr_epc_req(uint8_t link_id, uint8_t action)
{
}


/**
 *****************************************************************************************
 * @brief Get the TX power as control structure TX power field from a value in dBm.
 *
 * @param[in] txpwr_dbm   TX power in dBm
 *
 * @return The index of the TX power
 *
 *****************************************************************************************
 */
static uint8_t rf_txpwr_cs_get (int8_t txpwr_dbm)
{
}
#endif

/**
 *****************************************************************************************
 * @brief Convert RSSI to dBm
 *
 * @param[in] rssi_reg RSSI read from the HW registers
 *
 * @return The converted RSSI
 *
 *****************************************************************************************
 */
static uint8_t rf_rssi_convert (uint8_t rssi_reg)
{
    uint8_t RSSI_Value_temp = rssi_reg;

    /* RSSI calculation formula:
     * The equation to use for Input Power = 0.498 * RSSI - 127,
     * where the RSSI is 8-bit value used by SDK.
     * The range of valid RSSI values is 40 to 230.
     * Any values lower then 40 should be ceiled to 40.
     * Any values higher than 230 should be floored to 230.
    */
    if (rssi_reg < 40)
    {
        RSSI_Value_temp = 40;
    }
    else if (rssi_reg > 230)
    {
        RSSI_Value_temp = 230;
    }

    if (RSSI_Value_temp == 0)
    {
       return (uint8_t) (-127);
    }
    else
    {
       return (uint8_t) ((uint8_t) (-127) + (uint8_t) ((498 * RSSI_Value_temp) / 1000));
    }
}

#ifdef CFG_BLE
/**
 ****************************************************************************************
 * @brief ISR to be called in BLE ISR routine when RF Interrupt occurs.
 *****************************************************************************************
 */
static void RADIOCNTL_Handler(void)
{
}

/**
 *****************************************************************************************
 * @brief Enable/disable force AGC mechanism
 *
 * @param[in]  True: Enable / False: disable
 *****************************************************************************************
 */
static void rf_force_agc_enable(bool en)
{
}
#endif //CFG_BLE

/**
 *****************************************************************************************
 * @brief Gets the register value which controls the TX output power
 *
 * @param[in] conidx        Connection index (0xFF for advertising)
 * @param[in] req_pwr_level Requested power level (max or current)
 *
 * @return The register value which controls the TX output power
 *
 *****************************************************************************************
 */
static uint8_t rf_txpwr_dbm_get(uint8_t conidx, uint8_t req_pwr_level)
{
    rf_tx_pwr_lvl_t tx_pow_lvl = RF_TX_PWR_LVL_0d0;
    ASSERT_ERROR((req_pwr_level == TX_PWR_LVL_MAX) || (req_pwr_level == TX_PWR_LVL_CURRENT));
    ASSERT_ERROR((conidx < CFG_MAX_CONNECTIONS) || (conidx == 0xFF));

    if (conidx == 0xFF)
    {
        // Advertising mode
#if defined (CFG_ENHANCED_TX_PWR_CTRL)
        tx_pow_lvl = rf_pa_pwr_adv_get();
#else
        tx_pow_lvl = rf_pa_pwr_get();
#endif
    }
    else
    {
         // Connected mode
        if (req_pwr_level == TX_PWR_LVL_MAX)
        {
            tx_pow_lvl = RF_TX_PWR_LVL_PLUS_2d5;
        }
        else
        {
#if defined (CFG_ENHANCED_TX_PWR_CTRL)
            tx_pow_lvl = rf_pa_pwr_conn_get(conidx);
#else
            tx_pow_lvl = rf_pa_pwr_get();
#endif
        }
    }

    return (uint8_t) tx_pow_lvl;
}

/**
 *****************************************************************************************
 * @brief Sleep function for the RF.
 *****************************************************************************************
 */
static void rf_sleep(void)
{
    ble_deepslcntl_set(ble_deepslcntl_get() |
                      BLE_DEEP_SLEEP_ON_BIT |    // RW BLE Core sleep
                      BLE_RADIO_SLEEP_EN_BIT |   // Radio sleep
                      BLE_OSC_SLEEP_EN_BIT);     // Oscillator sleep

    SetBits32(BLE_CNTL2_REG, BB_ONLY,0);
}

/**
 *****************************************************************************************
 * @brief Get RF preferred setting for radiopwrupdn.
 * 
 * For BLE this value will be increased with ARP_RX_DELAY and ARP_TX_DELAY
 * NOTE: BE AWARE THIS FUNCTION MAY BE CALLED ONLY ONCE, PREFERABLE DIRECT AFTER SET_RECOMMENDED_SETTINGS
 *****************************************************************************************
 */
static void update_radiopwrupdn_pref_setting(void)
{
    uint32_t reg = GetWord32(BLE_RADIOPWRUPDN_REG);
    uint32_t ble_rxpwrup = GetBits32(&reg, RXPWRUP) + ARP_RX_DELAY; 
    uint32_t ble_txpwrup = GetBits32(&reg, TXPWRUP) + ARP_TX_DELAY; 

    reg &= ~(RXPWRUP | TXPWRUP);
    reg |= (ble_rxpwrup << SHIF32(RXPWRUP)) | (ble_txpwrup << SHIF32(TXPWRUP));
    SetWord32(BLE_RADIOPWRUPDN_REG, reg);
}

/**
 *****************************************************************************************
 * @brief Initialization of RF registers.
 *****************************************************************************************
 */
static void rf_regs(void)
{
    syscntl_cfg_xtal32m_amp_reg(XTAL32M_AMP_REG_HOLD);      // switch to HOLD mode

    /***
     *** Init BLE
     ***/
    // Note: ble_report_check_rx() will toggle BLE_RSSI_SEL on each Rx !
    SetBits32(BLE_CNTL2_REG, BLE_RSSI_SEL, 0);    // 0: Select Peak-hold RSSI value (default), 1: Select current Average RSSI.

    SetWord32(BLE_RADIOCNTL1_REG, 0);
    
    SetBits32(BLE_RADIOCNTL1_REG, XRFSEL, 2);       // Select on-chip radio type: 2 is the only choice

    SetBits32(BLE_RWBLECNTL_REG, RXWINSZDEF, 14);   // Using 14us, just like final Application does.
                                                    // 10=recommended. 0 is not a valid value.
                                                    // Only in slave or initiator or scanner device and only during the 1st Rx
                                                    // the ctrl_struct->rxwinsz will be used.
}

/*
 * RADIO FUNCTION INTERFACE
 ****************************************************************************************
 */
#define EM_BLE_FREQ_TABLE_LEN       EM_RF_FREQ_TABLE_LEN

void rf_init_func(struct rwip_rf_api *api)
{
    uint8_t idx = 0;
    uint8_t temp_freq_tbl[EM_BLE_FREQ_TABLE_LEN];

    // Initialize the RF driver API structure
    api->reg_rd = rf_reg_rd;
    api->reg_wr = rf_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;

    api->txpwr_max = 0;//RPL_POWER_MAX;
    api->sleep = rf_sleep;
    api->reset = rf_reset;
#if defined (CFG_BLE)
    api->isr = RADIOCNTL_Handler;
    api->force_agc_enable = rf_force_agc_enable;
#endif //CFG_BLE

    api->rssi_convert = rf_rssi_convert;

#if defined (CFG_BT)
    api->txpwr_inc = rf_txpwr_inc;
    api->txpwr_dec = rf_txpwr_dec;
    api->txpwr_epc_req = rf_txpwr_epc_req;
    api->txpwr_cs_get = rf_txpwr_cs_get;
    api->rssi_convert = rf_rssi_convert;
    api->rssi_high_thr = (uint8_t)RPL_RSSI_20dB_THRHLD;
    api->rssi_low_thr = (uint8_t)RPL_RSSI_60dB_THRHLD;
    api->rssi_interf_thr = (uint8_t)RPL_RSSI_70dB_THRHLD;
    #ifdef CFG_BTCORE_30
    api->wakeup_delay = RPL_WK_UP_DELAY;
    #endif //CFG_BTCORE_30
    api->skew = RPL_RADIO_SKEW;
#endif //CFG_BT

#if !defined (__FPGA__)
    rf_power_up();
#endif

    SetBits32(BLE_RADIOCNTL1_REG, XRFSEL, 2);

    SetBits32(BLE_CNTL2_REG, SW_RPL_SPI, 0);
    while (idx < EM_BLE_FREQ_TABLE_LEN) {
            temp_freq_tbl[idx] = idx ;
            idx++;
    }
    em_ble_burst_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_BLE_FREQ_TABLE_LEN);

#if !defined (CFG_ENHANCED_TX_PWR_CTRL)
    // Initialize Tx output power level to default 0dBm
    rf_pa_pwr_set(RF_TX_PWR_LVL_0d0);

    //  Near Field Mode Enable
    if (rf_nfm_is_enabled())
    {
        rf_pa_pwr_set(RF_TX_PWR_LVL_MINUS_19d5);
    }
#else
    // Initialize Tx output power level to default 0dBm
    rf_pa_pwr_adv_set(RF_TX_PWR_LVL_0d0);
#endif

    // Activate ADPLL block to make its registers accessible
    rf_adplldig_activate();

    otp_cs_load_pd_rad();                // Preferred settings for PD_RAD from OTP CS
    
    otp_cs_load_pd_adpll();              // Preferred settings for PD_ADPLL from OTP CS

#if defined (CFG_ENHANCED_TX_PWR_CTRL)
    // NOTE: Needed in ble_update_arp() call below!!
    SetWord32(ADPLL_DIV_CTRL_REG, otp_cs_get_adpll_div_ctrl_0dBm());
#endif

    set_recommended_settings();          // Preferred settings

    update_radiopwrupdn_pref_setting();

    rf_regs();

    rf_calibration();                           // Calibration
    ble_update_arp();                           // update with cal results

    // Deactivate ADPLL. It will be automatically enabled during TX/RX event
    rf_adplldig_deactivate();
}

void rf_reinit_func(void)
{
    SetBits32(BLE_RADIOCNTL1_REG, XRFSEL, 2);
    SetBits32(BLE_RWBLECNTL_REG, SYNCERR, 0); //this must be always '0'

    SetBits16(CLK_RADIO_REG, RFCU_ENABLE, 1);

    if ( arch_clk_is_XTAL32( ) )
    {
        //If LP clock is XTAL32 in Boost mode set XTAL32K_CUR to 1.
        if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED) == 0x1)
            SetBits16(CLK_XTAL32K_REG,  XTAL32K_CUR, 1);
    }

#if !defined (CFG_ENHANCED_TX_PWR_CTRL)
    //  Near Field Mode Enable
    if (rf_nfm_is_enabled())
    {
        rf_pa_pwr_set(RF_TX_PWR_LVL_MINUS_19d5);
    }
#endif

    // Activate ADPLL block to make its registers accessible
    rf_adplldig_activate();
    
    otp_cs_load_pd_rad();                // Trim values for PD_RAD from OTP CS
    
    otp_cs_load_pd_adpll();              // Trim values for PD_ADPLL from OTP CS

    set_recommended_settings();          // Preferred settings

    update_radiopwrupdn_pref_setting();

    rf_regs();

#if !defined (CFG_ENHANCED_TX_PWR_CTRL)
    ble_update_arp_adpll_div_val();
#endif

    // Deactivate ADPLL. It will be automatically enabled during TX/RX event
    rf_adplldig_deactivate();

#if defined (CFG_RF_DIAG_INT)
    #include "system_library.h"
    enable_rf_diag_irq(RF_DIAG_IRQ_MODE_RXTX); // argument is unused for DA14531
#endif
}

// definitions
/*
 * ARP - Automatic Radio Programming
 */
#define ARPT_PWR_ENTRIES    (1)

#if defined (CFG_ENHANCED_TX_PWR_CTRL)
#define ARPT_PREF_ENTRIES   (16)
#else
#define ARPT_PREF_ENTRIES   (14)
#endif

#define ARPT_TX_ENTRIES     (3)
#define ARPT_RX_ENTRIES     (3)

#if defined (CFG_ENHANCED_TX_PWR_CTRL)
#define ARPT_RF_PA_CTRL_REG_IDX     (13)
#define ARPT_RF_ATTR_REG_IDX        (14)
#endif

/*
 * ARP table entry definition
 */
typedef union {
    uint32_t value;
    struct {
        unsigned data      : 16;
        unsigned addr      : 14;
        unsigned data8or16n: 1;    // 1:8bit data, 0:16bit data
        unsigned valid     : 1;    // 1: valid, 0:invalid
    } bf;
} __attribute((aligned(4))) t_arp_reg;

/*
 * ARP Table structure
 */
struct s_arp_table {
    // ARPT_PWR
    t_arp_reg  arpt_pwr[ARPT_PWR_ENTRIES];
    uint32_t   null1;

    // ARPT_PREF
    uint16_t   tx_tbase;
    uint16_t   rx_tbase;
    t_arp_reg  arpt_pref[ARPT_PREF_ENTRIES];
    uint32_t   null2;

    // ARPT_TX
    t_arp_reg  tx_en_delay;
    t_arp_reg  arpt_tx[ARPT_TX_ENTRIES];
    uint32_t   null3;

    // ARPT_RX
    t_arp_reg  rx_en_delay;
    t_arp_reg  arpt_rx[ARPT_RX_ENTRIES];
    uint32_t   null4;
} __attribute((aligned(4)));


// arp_table has to be allocated after the freq_idx_table and before ctrl_struct
volatile struct s_arp_table *const arp_table = (struct s_arp_table *) (uint32_t) (__SCT_BLE_BASE + EM_RF_ARP_OFFSET);

static void ble_update_arp_adpll_calib_vals(void)
{
    //read out reg, be sure calibration has be done and adpll ldo-on and hclk-en
    arp_table->arpt_pref[4].bf.data = GetWord16(ADPLL_KDTCTDC_CAL_CTRL1_REG);
    arp_table->arpt_pref[5].bf.data = GetWord16(ADPLL_KDTCTDC_CAL_CTRL1_REG+2);
}

static void ble_update_arp_adpll_div_val(void)
{
    arp_table->arpt_pref[7].bf.data = GetWord16(ADPLL_DIV_CTRL_REG);
    arp_table->arpt_pref[8].bf.data = GetWord16(ADPLL_DIV_CTRL_REG+2);
}

static void ble_update_arp(void)
{
    ble_update_arp_adpll_calib_vals();
    ble_update_arp_adpll_div_val();
    arp_table->arpt_pref[3].bf.data = GetWord16(ADPLL_KDCO_CAL_CTRL1_REG);
    arp_table->arpt_pref[9].bf.data = GetWord16(ADPLL_ACC_CTRL_REG);
    arp_table->arpt_pref[10].bf.data = GetWord16(ADPLL_ACC_CTRL_REG+2);

//    arp_table->arpt_pref[xx].bf.data = GetWord16(ADPLL_DCOAMP_CAL_CTRL_REG);  For bit 0..15 an entry in ble_arp block is not necessary, is same as reset value
    arp_table->arpt_pref[11].bf.data = GetWord16(ADPLL_DCOAMP_CAL_CTRL_REG+2);

    arp_table->arpt_pref[12].bf.data = GetWord16(ADPLL_TXMOD_CTRL_REG);
}

const struct s_arp_table arp_init_table = {
    // ARPT_PWR
    .arpt_pwr[0].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & RF_RADIO_INIT_REG,
                      .data        = ADPLLDIG_PWR_SW1_EN,                                   // ADPLLDIG_LDO enable and set ADPLLDIG register file reset
                      .data8or16n  = 0,                                                     // Word write
    },

    // The delay between enabling ADPLLDIG_PWR_SW1_EN and the parsing of arpt_pref table is defined by RF_ADPLLDIG_CTRL_REG[PWR_SW_TIM_CTRL]

    // ARPT_PREF
    .arpt_pref[0].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (RF_RADIO_INIT_REG + 1),                    // 2nd byte address
                      .data        = (ADPLLDIG_HRESET_N | ADPLLDIG_HCLK_EN) >> 8,           // Release ADPLLDIG register file reset and enable the HCLK
                      .data8or16n  = 1,                                                     // Byte write
    },

    .arpt_pref[1].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_ANA_CTRL_REG + 1),
                      .data        = (0xFF) & ((ADPLL_ANA_CTRL_REG_RESET | 0xB00) >> 8),    // Pref setting (DTC offset)
                      .data8or16n  = 1,                                                     // Byte write
    },

    .arpt_pref[2].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_RFPT_CTRL_REG),
                      .data        = (0xFF) & (ADPLL_RFPT_CTRL_REG_RESET | INV_CKRFPT),     // Pref setting (Invert RFPT clock)
                      .data8or16n  = 1,                                                     // Byte write
    },

    .arpt_pref[3].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_KDCO_CAL_CTRL1_REG),
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp()
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_pref[4].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_KDTCTDC_CAL_CTRL1_REG),
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp_adpll_calib_vals()
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_pref[5].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_KDTCTDC_CAL_CTRL1_REG + 2),
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp_adpll_calib_vals()
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_pref[6].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_OVERRULE_CTRL1_REG+1),
                      .data        = (0xFF) & ((OVR_VPAEN_WR | OVR_VPAEN_SEL) >> 8),        // Pref setting (VPA enable overrule)
                      .data8or16n  = 1,                                                     // Byte write
    },

    .arpt_pref[7].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_DIV_CTRL_REG),
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp_adpll_div_val()
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_pref[8].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_DIV_CTRL_REG + 2),                   // 2nd word address
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp_adpll_div_val()
                      .data8or16n  = 0,                                                     // Word write
    },

    
    .arpt_pref[9].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_ACC_CTRL_REG),
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp()
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_pref[10].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_ACC_CTRL_REG + 2),
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp()
                      .data8or16n  = 0,                                                     // Word write
    },

    
    .arpt_pref[11].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_DCOAMP_CAL_CTRL_REG + 2),
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp()
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_pref[12].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (ADPLL_TXMOD_CTRL_REG),
                      .data        = 0,                                                     // Dummy value it will be updated in ble_update_arp()
                      .data8or16n  = 0,                                                     // Word write
    },

#if defined (CFG_ENHANCED_TX_PWR_CTRL)
    .arpt_pref[ARPT_RF_PA_CTRL_REG_IDX].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (RF_PA_CTRL_REG),
                      .data        = 0x300,                                                 // Set PA_RAMP_STEP_SPEED, TRIM_DUTY_NEG, TRIM_DUTY_POS (reset values)
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_pref[ARPT_RF_ATTR_REG_IDX].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (RF_ATTR_REG + 2),                          // 2nd word address
                      .data        = (RF_TX_PWR_LVL_0d0 << 8),                              // Set output power to default 0dBm
                      .data8or16n  = 0,                                                     // Word write
    },
#endif

    .arpt_pref[ARPT_PREF_ENTRIES - 1].bf = {                                                // This must always be the last entry
                      .valid       = 1,
                      .addr        = (0x7FFF) & (RF_RADIO_INIT_REG + 1),                    // 2nd byte address
                      .data        = (ADPLLDIG_HRESET_N >> 8),                              // Disable ADPLLDIG register interface clock
                      .data8or16n  = 1,                                                     // Byte write
    },

    // ARPT_TX
    .tx_en_delay.bf = {
                      .valid       = 1,
                      .data        = ARP_TX_DELAY,
                      .addr        = 0,                                                     // Dummy value
    },

    .arpt_tx[0].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (RF_RADIO_INIT_REG ),
                      .data        = (ADPLLDIG_PWR_SW1_EN | ADPLLDIG_HRESET_N | ADPLLDIG_HCLK_EN),     // Release ADPLLDIG register file reset and enable the HCLK
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_tx[1].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & ADPLL_CN_CTRL_REG,                          // addr section defines the location where arp_cn[] should be written
                      .data        = 0,                                                     // Don't care, arp_cn[] comes from the BLE block
                      .data8or16n  = 1,                                                     // Byte write
    },

    .arpt_tx[2].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (RF_RADIO_INIT_REG + 2),                    // 2nd byte address
                      .data        = (RADIO_REGS_RDY | ADPLLDIG_HCLK_DIS) >> 16,            // Disable ADPLLDIG register interface clock and set the RADIO_REGS_RDY bit field to 1
                      .data8or16n  = 1,                                                     // Byte write
    },

    // ARPT_RX
    .rx_en_delay.bf = {
                      .valid       = 1,
                      .data        = ARP_RX_DELAY,                                          // ARP_RX_DELAY JS
                      .addr        = 0,                                                     // Dummy value
    },

    .arpt_rx[0].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (RF_RADIO_INIT_REG),
                      .data        = (ADPLLDIG_PWR_SW1_EN | ADPLLDIG_HRESET_N | ADPLLDIG_HCLK_EN), // Release ADPLLDIG register file reset and enable the HCLK
                      .data8or16n  = 0,                                                     // Word write
    },

    .arpt_rx[1].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & ADPLL_CN_CTRL_REG,                          // addr section defines the location where arp_cn[] should be written. arp_cn[] comes from the BLE block
                      .data        = 0,                                                     // Don't care, arp_cn[] comes from the BLE block
                      .data8or16n  = 1,                                                     // Byte write
    },

    .arpt_rx[2].bf = {
                      .valid       = 1,
                      .addr        = (0x7FFF) & (RF_RADIO_INIT_REG + 2),                    // 2nd byte address
                      .data        = (RADIO_REGS_RDY | ADPLLDIG_HCLK_DIS) >> 16,            // Disable ADPLLDIG register interface clock and set the RADIO_REGS_RDY bit field to 1
                      .data8or16n  = 1,                                                     // Byte write
    },
};

/*********************************************************************************
 *** ble_init_arp()
 *** About: Load ARP Table values.
 ***        must be executed after ble_init_em()
 ***        in order to have the table pointer up to date and have the Table preloaded with zeros.
 ***/
void ble_init_arp_func(uint32_t base_addr)
{
    ASSERT_WARNING(sizeof(arp_init_table) <= EM_RF_ARP_SIZE_MAX);

    memcpy((uint32_t *)arp_table, &arp_init_table, sizeof(arp_init_table));

    arp_table->tx_tbase                   = (uint32_t)&(arp_table->tx_en_delay) - (uint32_t)base_addr;
    arp_table->rx_tbase                   = (uint32_t)&(arp_table->rx_en_delay) - (uint32_t)base_addr;
}

void rf_recalibration(void) 
{
    rf_adplldig_activate();
    otp_cs_load_pd_adpll();              // Trim values for PD_ADPLL from OTP CS

    set_recommended_settings();          // Preferred settings
    update_radiopwrupdn_pref_setting();

    rf_calibration();                    // Calibration
    ble_update_arp_adpll_calib_vals();

    // Deactivate ADPLL. It will be automatically enabled during TX/RX event
    rf_adplldig_deactivate();
}

#if defined (CFG_ENHANCED_TX_PWR_CTRL)
/// BLE RF mode
typedef enum
{
    /// Advertising mode
    RF_PWR_MODE_ADV = 0,

    /// Connected mode
    RF_PWR_MODE_CONN = 1
} rf_pwr_mode_t;

/**
 ****************************************************************************************
 * @brief Helper function being called by @c ble_arp_update_tx_pwr.
 * @param[in] pwr_mode The BLE RF mode: advertising; connected.
 * @param[in] conidx The connection index.
 ****************************************************************************************
 */
static void update_pwr(rf_pwr_mode_t pwr_mode, uint8_t conidx)
{
    rf_tx_pwr_lvl_t tx_pwr;

    if (pwr_mode == RF_PWR_MODE_ADV)
    {
        // BLE RF advertising mode
        tx_pwr = rf_pa_pwr_adv_get();
    }
    else
    {
        // BLE RF connected mode
        tx_pwr = rf_pa_pwr_conn_get(conidx);
    }

    // Differentiate loaded settings based on 3dBm or 0dBm
    // 0dBm mode for any power setting equal to 9 or lower; 3dBm for 10 or higher
    if (tx_pwr > RF_TX_PWR_LVL_0d0)
    {
        // Use TRIM_DUTY_NEG and TRIM_DUTY_POS for 3dBm (residing in the lower
        // 16 bits of RF_PA_CTRL_REG)
        arp_table->arpt_pref[ARPT_RF_PA_CTRL_REG_IDX].bf.data = (uint16_t) otp_cs_get_rf_pa_ctrl_3dBm();

        // Use TXDIV_TRIM for 3dBm (residing in the upper 16 bits of ADPLL_DIV_CTRL_REG)
        arp_table->arpt_pref[8].bf.data = (uint16_t) (otp_cs_get_adpll_div_ctrl_3dBm() >> 16);

        // Load user radio power settings depending on the detected mode
        arp_table->arpt_pref[ARPT_RF_ATTR_REG_IDX].bf.data = (tx_pwr << 8);
    }
    else
    {
        // Use TRIM_DUTY_NEG and TRIM_DUTY_POS for 0dBm (residing in the lower
        // 16 bits of RF_PA_CTRL_REG)
        arp_table->arpt_pref[ARPT_RF_PA_CTRL_REG_IDX].bf.data = (uint16_t) otp_cs_get_rf_pa_ctrl_0dBm();

        // Use TXDIV_TRIM for 0dBm (residing in the upper 16 bits of ADPLL_DIV_CTRL_REG)
        arp_table->arpt_pref[8].bf.data = (uint16_t) (otp_cs_get_adpll_div_ctrl_0dBm() >> 16);

        if (rf_nfm_is_enabled())
        {
            // Near Field Mode Enable
            arp_table->arpt_pref[ARPT_RF_ATTR_REG_IDX].bf.data = (RF_TX_PWR_LVL_MINUS_19d5 << 8);
        }
        else
        {
            // Load user radio power settings depending on the detected mode
            arp_table->arpt_pref[ARPT_RF_ATTR_REG_IDX].bf.data = (tx_pwr << 8);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Updates the radio output power depending on the detected BLE event.
 ****************************************************************************************
 */
void ble_arp_update_tx_pwr(void)
{
    // Get the scheduled BLE event
    struct ea_elt_tag *elt  = (struct ea_elt_tag *) co_list_pick(&lld_evt_env.elt_prog);
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    if (evt->mode == LLD_EVT_ADV_MODE)
    {
        update_pwr(RF_PWR_MODE_ADV, 0);
    }
    else if ((evt->mode == LLD_EVT_MST_MODE) || (evt->mode == LLD_EVT_SLV_MODE))
    {
        update_pwr(RF_PWR_MODE_CONN, evt->conhdl);
    }
}
#endif // CFG_ENHANCED_TX_PWR_CTRL

#endif // __DA14531__
