/**
 ****************************************************************************************
 * @addtogroup Core_Modules
 * @{
 * @addtogroup RF
 * @brief RF driver
 * @{
 *
 * @file rf_531.h
 *
 * @brief RF functions DA14531 header file.
 *
 * Copyright (C) 2018-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _RF_531_H_
#define _RF_531_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdbool.h>
#include <stdint.h>
#include "datasheet.h"
#include "compiler.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// KDCO result size
#define KDCO_RESULT_SZ          (8)
/// KDTC result size
#define KDTC_RESULT_SZ          (8)
/// Tx-Div2 result size
#define TX_DIV2_RESULT_SZ       (8)
/// IQ-Div2 result size
#define IQ_DIV2_RESULT_SZ       (8)

/// Defines the transmit output power level from -19.5 dBm up to +2.5 dBm
typedef enum
{
    /// -19.5 dBm
    RF_TX_PWR_LVL_MINUS_19d5     = 1U,

    /// -13.5 dBm
    RF_TX_PWR_LVL_MINUS_13d5     = 2U,

    /// -10 dBm
    RF_TX_PWR_LVL_MINUS_10d0     = 3U,

    /// -7 dBm
    RF_TX_PWR_LVL_MINUS_7d0      = 4U,

    /// -5 dBm
    RF_TX_PWR_LVL_MINUS_5d0      = 5U,

    /// -3.5 dBm
    RF_TX_PWR_LVL_MINUS_3d5      = 6U,

    /// -2 dBm
    RF_TX_PWR_LVL_MINUS_2d0      = 7U,

    /// -1 dBm
    RF_TX_PWR_LVL_MINUS_1d0      = 8U,

    /// 0 dBm
    RF_TX_PWR_LVL_0d0            = 9U,

    /// +1 dBm
    RF_TX_PWR_LVL_PLUS_1d0       = 10U,

    /// +1.5 dBm
    RF_TX_PWR_LVL_PLUS_1d5       = 11U,

    /// +2.5 dBm
    RF_TX_PWR_LVL_PLUS_2d5       = 12U,

    /// Number of dBm values
    RF_TX_PWR_LVL_TOTAL_VALUES   = RF_TX_PWR_LVL_PLUS_2d5
} rf_tx_pwr_lvl_t;

/// @name Delay routines
///@{
/// WAIT Delay routine
#define WAIT()     for (volatile int wait=0; wait < 0xB;  wait++) {}
/// WAIT10 Delay routine
#define WAIT10()   for (volatile int wait=0; wait < 0x10;  wait++) {}
/// WAIT100 Delay routine
#define WAIT100()   for (volatile int wait=0; wait < 0x100;  wait++) {} // wait  30 us
/// WAIT400 Delay routine
#define WAIT400()   for (volatile int wait=0; wait < 0x400;  wait++) {} // wait  120 us
/// WAIT1000 Delay routine
#define WAIT1000()   for (volatile int wait=0; wait < 0x1000;  wait++) {} // wait  500 us
///@}

/// Define DA14531
#define DA14531

/**
 *****************************************************************************************
 * @brief Set RF preferred setting.
 *****************************************************************************************
 */
void set_recommended_settings(void);

/**
 *****************************************************************************************
 * @brief DC-offset calibration.
 * @return true if DC-offset calibration completed, otherwise false.
 *****************************************************************************************
 */
bool dcoff_calibration(void);

/**
 *****************************************************************************************
 * @brief KDTC calibration.
 * @return true.
 *****************************************************************************************
 */
bool kdtc_calibration(void);

/**
 *****************************************************************************************
 * @brief KDCO calibration.
 * @return true.
 *****************************************************************************************
 */
bool kdco_calibration(void);

/**
 *****************************************************************************************
 * @brief RF calibration.
 *****************************************************************************************
 */
void rf_calibration(void);

/**
 *****************************************************************************************
 * @brief RF power up.
 *****************************************************************************************
 */
void rf_power_up(void);

/**
 *****************************************************************************************
 * @brief Activate radio LDOs.
 *****************************************************************************************
 */
void rf_ldo_cont_mode_en(void);

/**
 *****************************************************************************************
 * @brief Activate the ADPLL Dig related LDOs.
 *****************************************************************************************
 */
void rf_adplldig_ldo_on(void);

/**
 *****************************************************************************************
 * @brief Set pulse shaper settings for ADPLLDIG.
 *****************************************************************************************
 */
void rf_adplldig_txmod(void);

/**
 *****************************************************************************************
 * @brief Disable hclk for the ADPLL AHB.
 *****************************************************************************************
 */
void dis_hclk(void);

/**
 *****************************************************************************************
 * @brief Enable hclk for the ADPLL AHB.
 *****************************************************************************************
 */
void en_hclk(void);

/**
 *****************************************************************************************
 * @brief KDTC calibration init.
 *****************************************************************************************
 */
void kdtc_cal_init(void);

/**
 *****************************************************************************************
 * @brief KDTC calibration end.
 *****************************************************************************************
 */
void kdtc_cal_end(void);

/**
 *****************************************************************************************
 * @brief Set parameters for KDTC calibration mode 1.
 *****************************************************************************************
 */
void en_kdtc_cal_mod1(void);

/**
 *****************************************************************************************
 * @brief Set parameters for KDTC calibration mode 0.
 *****************************************************************************************
 */
void en_kdtc_cal_mod0(void);

/**
 *****************************************************************************************
 * @brief Disable KDTC calibration.
 *****************************************************************************************
 */
void dis_kdtc_cal(void);

/**
 *****************************************************************************************
 * @brief KDCO calibration init.
 *****************************************************************************************
 */
void kdco_cal_init(void);

/**
 *****************************************************************************************
 * @brief KDCO calibration end.
 *****************************************************************************************
 */
void kdco_cal_end(void);

/**
 *****************************************************************************************
 * @brief Disable KDCO calibration.
 *****************************************************************************************
 */
void dis_kdco_cal(void);


/**
 *****************************************************************************************
 * @brief Activate ADPLL in TX mode.
 *****************************************************************************************
 */
void en_adpll_tx(void);

/**
 *****************************************************************************************
 * @brief Disable ADPLL.
 *****************************************************************************************
 */
void dis_adpll(void);

/**
 *****************************************************************************************
 * @brief Activate ADPLL.
 *****************************************************************************************
 */
void rf_adplldig_activate(void);

/**
 *****************************************************************************************
 * @brief Deactivate ADPLL.
 *****************************************************************************************
 */
void rf_adplldig_deactivate(void);

#if defined (CFG_ENHANCED_TX_PWR_CTRL)
/**
 ****************************************************************************************
 * @brief Sets the Tx output power level of advertising events through the ARP Table.
 * @details Initializes the transmit output power level of connection events as well.
 * @param[in] level The transmit output power level.
 ****************************************************************************************
 */
void rf_pa_pwr_adv_set(rf_tx_pwr_lvl_t level);

/**
 *****************************************************************************************
 * @brief Gets the Tx output power level of advertising events.
 * @return the Tx output power level value.
 *****************************************************************************************
 */
rf_tx_pwr_lvl_t rf_pa_pwr_adv_get(void);

/**
 ****************************************************************************************
 * @brief Sets the Tx output power level of connection events through the ARP Table.
 * @param[in] level The transmit output power level.
 * @param[in] conidx The connection index.
 ****************************************************************************************
 */
void rf_pa_pwr_conn_set(rf_tx_pwr_lvl_t level, uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Gets the Tx output power level of connection events.
 * @param[in] conidx The connection index.
 * @return the Tx output power level value.
 ****************************************************************************************
 */
rf_tx_pwr_lvl_t rf_pa_pwr_conn_get(uint8_t conidx);
#else
/**
 *****************************************************************************************
 * @brief Selects the Tx output power level. The selected value will be applied to 
 *        hardware when @p set_recommended_settings() is called.
 * @param[in] level    The Tx output power level from -18dBm up to 3dBm
 *****************************************************************************************
 */
void rf_pa_pwr_set(rf_tx_pwr_lvl_t level);

/**
 *****************************************************************************************
 * @brief Get the Tx output power level value saved in retained memory block.
 * @return the Tx output power level value.
 * @note It may be different than the value read from RF_ATTR_REG.PA_POWER_SETTING field.
 *****************************************************************************************
 */
__WEAK rf_tx_pwr_lvl_t rf_pa_pwr_get(void);
#endif

/// Structure holding the results of the calibration
struct rf_calibration_result {
        /// DC-offset result
        uint8_t dcoff_result;
        /// KDCO result
        uint8_t kdco_result[KDCO_RESULT_SZ];
        /// KDTC result
        uint16_t kdtc_result[KDTC_RESULT_SZ];
        /// Tx-Div2 result
        uint8_t tx_div2_result[TX_DIV2_RESULT_SZ];
        /// IQ-Div2 result
        uint8_t iq_div2_result[IQ_DIV2_RESULT_SZ];
};

/// Calibration result
extern volatile struct rf_calibration_result calibration_res;

/*
 * NEAR FIELD MODE API
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Checks if Near Field mode is enabled.
 * @return true if Near Field mode is enabled, otherwise false.
 ****************************************************************************************
 */
bool rf_nfm_is_enabled(void);

/**
 ****************************************************************************************
 * @brief Enables Near Field mode for all connections. Tx output power is set to -18dBm.
 ****************************************************************************************
 */
void rf_nfm_enable(void);

/**
 ****************************************************************************************
 * @brief Disables Near Field mode for all connections. The previous Tx output power is
 * restored.
 ****************************************************************************************
 */
void rf_nfm_disable(void);

void rf_power_down(void);

#endif //_RF_531_H_

/// @}
/// @}
