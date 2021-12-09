/**
 ****************************************************************************************
 *
 * @file da1458x_config_basic.h
 *
 * @brief Basic compile configuration file.
 *
 * Copyright (C) 2015-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _DA1458X_CONFIG_BASIC_H_
#define _DA1458X_CONFIG_BASIC_H_

#if defined (__DA14531__)

/****************************************************************************************************************/
/* Notify the SDK about the fixed power mode (currently used only for Bypass):                                  */
/*     - CFG_POWER_MODE_BYPASS = Bypass mode                                                                    */
/****************************************************************************************************************/
#undef CFG_POWER_MODE_BYPASS

/****************************************************************************************************************/
/* Max buffer size used for the communication with the external tool over UART                                  */
/****************************************************************************************************************/
#define ALLOWED_DATA_UART       (0x08000)

/****************************************************************************************************************/
/* Max buffer size used for the communication with the external tool over SWD                                   */
/****************************************************************************************************************/
#define ALLOWED_DATA_SWD        (0x04000)

/****************************************************************************************************************/
/* Enables the SPI flash memory extensions feature. When enabled, the DA14531 controls, through a dedicated     */
/* GPIO, the activation/deactivation of the SPI flash memory device. This feature is mainly used when the       */
/* DA14531 is in boost mode and the SPI flash memory is supplied by VBAT_H (VBAT_L is the input voltage and the */
/* DCDC converter generates a programmable output voltage at VBAT_H). In boost mode the DCDC converter has a    */
/* limit of 50uA in VBAT_H during the DA14531 power-up. Due to this limitation some SPI flash memories may fail */
/* to start-up for certain values of VBAT_L. Therefore, their power voltage must be externally controlled using */
/* the SPI flash memory extensions feature. Refer to SDK6 documentation for further details.                    */
/****************************************************************************************************************/
#undef CFG_SPI_FLASH_EXTENSIONS

/****************************************************************************************************************/
/* The GPIO pin may be directly connected to a capacitor. If the pin status changes, the capacitor may be       */
/* discharged abruptly through the GPIO pin. If the macro is defined, the pull-down resistor will be enabled    */
/* first before enabling the pin push-pull operation.                                                           */
/* Note: The feature is used in the CFG_SPI_FLASH_EXTENSIONS context when the dedicated pin is connected        */
/* directly to the SPI flash memory voltage input.                                                              */
/****************************************************************************************************************/
#define CFG_SPI_FLASH_CAP_SLOW_DISCHARGE

/****************************************************************************************************************/
/* The GPIO pin may be directly connected to a capacitor. If the pin status changes abruptly, the GPIO pin may  */
/* be stressed since a large current may flow through the pin to charge the capacitor. Therefore, initially the */
/* internal pull-up resistor will be enabled.                                                                   */
/* Note: The feature is used in the CFG_SPI_FLASH_EXTENSIONS context when the dedicated pin is connected        */
/* directly to the SPI flash memory voltage input.                                                              */
/****************************************************************************************************************/
#define CFG_SPI_FLASH_CAP_SLOW_CHARGE

#else

/****************************************************************************************************************/
/* Max buffer size used for the communication with the external tool over UART                                  */
/****************************************************************************************************************/
#define ALLOWED_DATA_UART       (0x0FFFF)

/****************************************************************************************************************/
/* Max buffer size used for the communication with the external tool over SWD                                   */
/****************************************************************************************************************/
#define ALLOWED_DATA_SWD        (0x10000)

#endif // __DA14531__

/****************************************************************************************************************/
/* Enable extra wait time after release from Ultra Deep Power-Down (UDPD). Mainly required for the              */
/* Adesto/Dialog SPI flash memories which support the UDPD command.                                             */
/****************************************************************************************************************/
#undef CFG_SPI_FLASH_ADESTO_UDPD

/****************************************************************************************************************/
/* Control the memory protection feature of the SPI flash chip through its Status Register 1.                   */
/****************************************************************************************************************/
#define CFG_SPI_FLASH_MEM_PROTECT_USING_STATUS_REG1

#endif // _DA1458X_CONFIG_BASIC_H_
