/**
 ****************************************************************************************
* @addtogroup UTILITIES Utilities
 * @{
 * @addtogroup OTP_HEADER OTP Header
 * @brief OTP Header API
 * @{
 *
 *
 * @file otp_hdr.h
 *
 * @brief Definition of OTP header field offsets and addresses.
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _OTP_HDR_H_
#define _OTP_HDR_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "datasheet.h"
#include "arch.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#if !defined (__DA14531__)

/*
 * DA14585/DA14586: OTP header field offsets from OTP memory starting address
 */

/// OTP header offset
#define OTP_HDR_BASE_OFFSET                 (0xFE00)

/// SWD enable flag offset
#define OTP_HDR_SWD_ENABLE_OFFSET           (0xFFF8)
/// OTP DMA length offset
#define OTP_HDR_OTP_DMA_LENGTH_OFFSET       (0xFFF0)
/// Reserved
#define OTP_HDR_RESERVED1_OFFSET            (0xFFE8)
/// Reserved
#define OTP_HDR_RESERVED3_OFFSET            (0xFFB8)
/// Company number offset
#define OTP_HDR_COMPANY_NUMBER_OFFSET       (0xFFB0)
/// Bluetooth Device Address offset
#define OTP_HDR_BDADDR_OFFSET               (0xFFA8)
/// Signature Algorithm Selector offset
#define OTP_HDR_SIGN_ALG_SEL_OFFSET         (0xFFA0)
/// Signature of Customer Code offset
#define OTP_HDR_SIGN_CUST_CODE_OFFSET       (0xFF28)
/// Trim value for the VCO offset
#define OTP_HDR_TRIM_VCO_OFFSET             (0xFF20)
/// Trim value for the XTAL16M oscillator offset
#define OTP_HDR_TRIM_XTAL16M_OFFSET         (0xFF18)
/// Trim value for the RC16M oscillator offset
#define OTP_HDR_TRIM_RC16M_OFFSET           (0xFF10)
/// Trim value for the BandGap offset
#define OTP_HDR_TRIM_BANDGAP_OFFSET         (0xFF08)
/// Trim value for the RFIO capacitance offset
#define OTP_HDR_TRIM_RFIO_CAP_OFFSET        (0xFF00)
/// Trim value for the LNA offset
#define OTP_HDR_TRIM_LNA_OFFSET             (0xFEF8)
/// Calibration Flags offset
#define OTP_HDR_CALIBRATION_FLAGS_OFFSET    (0xFEF0)
/// Sleep Clock Source Flag offset
#define OTP_HDR_SLEEP_CLK_SOURCE_OFFSET     (0xFEE8)
/// Package Flag offset
#define OTP_HDR_PACKAGE_OFFSET              (0xFEE0)
/// Reserved
#define OTP_HDR_RESERVED2_OFFSET            (0xFEC8)
/// Boot specific mapping offset
#define OTP_HDR_BOOT_MAPPING_OFFSET         (0xFEC0)
/// UART STX Timeout offset
#define OTP_HDR_UART_STX_TO_OFFSET          (0xFEB8)
/// OTP control value offset
#define OTP_HDR_OTP_CONTROL_OFFSET          (0xFEB0)
/// Customer Specific Field (21 64-bit words) offset
#define OTP_HDR_CUSTOMER_FIELD_OFFSET       (0xFE20)
/// CRC for Trim and Calibration values offset
#define OTP_HDR_CRC_TRIM_CAL_OFFSET         (0xFE18)
/// IQ Trim value offset
#define OTP_HDR_IQ_TRIM_VALUE_OFFSET        (0xFE10)
/// Application Programmed Flag #2 offset
#define OTP_HDR_APP_PROG_FLAG2_OFFSET       (0xFE08)
/// Application Programmed Flag #1 offset
#define OTP_HDR_APP_PROG_FLAG1_OFFSET       (0xFE00)

/*
 * DA14585/DA14586: OTP header field memory addresses
 */
#define OTP_HDR_BASE_ADDR               (MEMORY_OTP_BASE + OTP_HDR_BASE_OFFSET)

#define OTP_HDR_SWD_ENABLE_ADDR         (MEMORY_OTP_BASE + OTP_HDR_SWD_ENABLE_OFFSET)
#define OTP_HDR_OTP_DMA_LENGTH_ADDR     (MEMORY_OTP_BASE + OTP_HDR_OTP_DMA_LENGTH_OFFSET)
#define OTP_HDR_RESERVED1_ADDR          (MEMORY_OTP_BASE + OTP_HDR_RESERVED1_OFFSET)
#define OTP_HDR_RESERVED3_ADDR          (MEMORY_OTP_BASE + OTP_HDR_RESERVED3_OFFSET)
#define OTP_HDR_COMPANY_NUMBER_ADDR     (MEMORY_OTP_BASE + OTP_HDR_COMPANY_NUMBER_OFFSET)
#define OTP_HDR_BDADDR_ADDR             (MEMORY_OTP_BASE + OTP_HDR_BDADDR_OFFSET)
#define OTP_HDR_SIGN_ALG_SEL_ADDR       (MEMORY_OTP_BASE + OTP_HDR_SIGN_ALG_SEL_OFFSET)
#define OTP_HDR_SIGN_CUST_CODE_ADDR     (MEMORY_OTP_BASE + OTP_HDR_SIGN_CUST_CODE_OFFSET)
#define OTP_HDR_TRIM_VCO_ADDR           (MEMORY_OTP_BASE + OTP_HDR_TRIM_VCO_OFFSET)
#define OTP_HDR_TRIM_XTAL16M_ADDR       (MEMORY_OTP_BASE + OTP_HDR_TRIM_XTAL16M_OFFSET)
#define OTP_HDR_TRIM_RC16M_ADDR         (MEMORY_OTP_BASE + OTP_HDR_TRIM_RC16M_OFFSET)
#define OTP_HDR_TRIM_BANDGAP_ADDR       (MEMORY_OTP_BASE + OTP_HDR_TRIM_BANDGAP_OFFSET)
#define OTP_HDR_TRIM_RFIO_CAP_ADDR      (MEMORY_OTP_BASE + OTP_HDR_TRIM_RFIO_CAP_OFFSET)
#define OTP_HDR_TRIM_LNA_ADDR           (MEMORY_OTP_BASE + OTP_HDR_TRIM_LNA_OFFSET)
#define OTP_HDR_CALIBRATION_FLAGS_ADDR  (MEMORY_OTP_BASE + OTP_HDR_CALIBRATION_FLAGS_OFFSET)
#define OTP_HDR_SLEEP_CLK_SOURCE_ADDR   (MEMORY_OTP_BASE + OTP_HDR_SLEEP_CLK_SOURCE_OFFSET)
#define OTP_HDR_PACKAGE_ADDR            (MEMORY_OTP_BASE + OTP_HDR_PACKAGE_OFFSET)
#define OTP_HDR_RESERVED2_ADDR          (MEMORY_OTP_BASE + OTP_HDR_RESERVED2_OFFSET)
#define OTP_HDR_BOOT_MAPPING_ADDR       (MEMORY_OTP_BASE + OTP_HDR_BOOT_MAPPING_OFFSET)
#define OTP_HDR_UART_STX_TO_ADDR        (MEMORY_OTP_BASE + OTP_HDR_UART_STX_TO_OFFSET)
#define OTP_HDR_OTP_CONTROL_ADDR        (MEMORY_OTP_BASE + OTP_HDR_OTP_CONTROL_OFFSET)
#define OTP_HDR_CUSTOMER_FIELD_ADDR     (MEMORY_OTP_BASE + OTP_HDR_CUSTOMER_FIELD_OFFSET)
#define OTP_HDR_CRC_TRIM_CAL_ADDR       (MEMORY_OTP_BASE + OTP_HDR_CRC_TRIM_CAL_OFFSET)
#define OTP_HDR_IQ_TRIM_VALUE_ADDR      (MEMORY_OTP_BASE + OTP_HDR_IQ_TRIM_VALUE_OFFSET)
#define OTP_HDR_APP_PROG_FLAG2_ADDR     (MEMORY_OTP_BASE + OTP_HDR_APP_PROG_FLAG2_OFFSET)
#define OTP_HDR_APP_PROG_FLAG1_ADDR     (MEMORY_OTP_BASE + OTP_HDR_APP_PROG_FLAG1_OFFSET)

#else

/*
 * DA14531: OTP header field offsets from OTP memory starting address
 */

/// OTP header offset
#define OTP_HDR_BASE_OFFSET                 (0x7FC0)

/// Reserved
#define OTP_HDR_RESERVED_OFFSET             (0x7FEC)
/// Timestamp offset
#define OTP_HDR_TIMESTAMP_OFFSET            (0x7FE8)
/// Tester offset
#define OTP_HDR_TESTER_OFFSET               (0x7FE4)
/// Position offset
#define OTP_HDR_POSITION_OFFSET             (0x7FE0)
/// OTP DMA length offset
#define OTP_HDR_OTP_DMA_LENGTH_OFFSET       (0x7FDC)
/// Bluetooth Device Address offset
#define OTP_HDR_BDADDR_OFFSET               (0x7FD4)
/// Device and Package Flag offset
#define OTP_HDR_PACKAGE_OFFSET              (0x7FD0)
/// Boot specific mapping offset
#define OTP_HDR_BOOT_MAPPING_OFFSET         (0x7FCC)
/// Boot specific configuration offset
#define OTP_HDR_BOOT_CONFIG_OFFSET          (0x7FC8)
/// Application Programmed Flag #2 offset
#define OTP_HDR_APP_PROG_FLAG2_OFFSET       (0x7FC4)
/// Application Programmed Flag #1 offset
#define OTP_HDR_APP_PROG_FLAG1_OFFSET       (0x7FC0)

/*
 * DA14531: OTP header field memory addresses
 */
#define OTP_HDR_BASE_ADDR               (MEMORY_OTP_BASE + OTP_HDR_BASE_OFFSET)

#define OTP_HDR_RESERVED_ADDR           (MEMORY_OTP_BASE + OTP_HDR_RESERVED_OFFSET)
#define OTP_HDR_TIMESTAMP_ADDR          (MEMORY_OTP_BASE + OTP_HDR_TIMESTAMP_OFFSET)
#define OTP_HDR_TESTER_ADDR             (MEMORY_OTP_BASE + OTP_HDR_TESTER_OFFSET)
#define OTP_HDR_POSITION_ADDR           (MEMORY_OTP_BASE + OTP_HDR_POSITION_OFFSET)
#define OTP_HDR_OTP_DMA_LENGTH_ADDR     (MEMORY_OTP_BASE + OTP_HDR_OTP_DMA_LENGTH_OFFSET)
#define OTP_HDR_BDADDR_ADDR             (MEMORY_OTP_BASE + OTP_HDR_BDADDR_OFFSET)
#define OTP_HDR_PACKAGE_ADDR            (MEMORY_OTP_BASE + OTP_HDR_PACKAGE_OFFSET)
#define OTP_HDR_BOOT_MAPPING_ADDR       (MEMORY_OTP_BASE + OTP_HDR_BOOT_MAPPING_OFFSET)
#define OTP_HDR_BOOT_CONFIG_ADDR        (MEMORY_OTP_BASE + OTP_HDR_BOOT_CONFIG_OFFSET)
#define OTP_HDR_APP_PROG_FLAG2_ADDR     (MEMORY_OTP_BASE + OTP_HDR_APP_PROG_FLAG2_OFFSET)
#define OTP_HDR_APP_PROG_FLAG1_ADDR     (MEMORY_OTP_BASE + OTP_HDR_APP_PROG_FLAG1_OFFSET)

#endif

/**
 ****************************************************************************************
 * @brief Get the BD address from OTP Header.
 * @param[out] bd_addr       The BD address value returned.
 ****************************************************************************************
 */
void otp_hdr_get_bd_address(uint8_t *bd_addr);

#endif //OTP_HDR_H

/// @}
/// @}
