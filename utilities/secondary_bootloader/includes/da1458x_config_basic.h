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

/****************************************************************************************************************/
/* Apply default SPI mode (mode 3)                                                                              */
/****************************************************************************************************************/
#undef CFG_SPI_SET_DEFAULT_BOOTER_SPI_SETTINGS

/****************************************************************************************************************/
/* Enables the secondary bootloader code logic to activate/deactivate the Debugger at the end of the booter.    */
/****************************************************************************************************************/
#undef CFG_ENTER_LOOP_WITH_DEBUGGER_ENABLED_ON_NO_APP

/****************************************************************************************************************/
/* Reads the OTP header in order to enable/disable the UART, when the secondary bootloader downloads the        */
/* application f/w to RAM.                                                                                      */
/****************************************************************************************************************/
#undef CFG_UART_BOOT_LOCK_OPTION

/****************************************************************************************************************/
/* Reads the OTP header in order to apply specific pins for the SPI serial interface. These pins will be used   */
/* by the secondary bootloader.                                                                                 */
/****************************************************************************************************************/
#undef CFG_SPI_BOOT_SPECIFIC_PORT_MAPPING

#endif // __DA14531__

/****************************************************************************************************************/
/* Enable extra wait time after release from Ultra Deep Power-Down (UDPD). Mainly required for the              */
/* Adesto/Dialog SPI flash memories which support the UDPD command.                                             */
/****************************************************************************************************************/
#undef CFG_SPI_FLASH_ADESTO_UDPD

/****************************************************************************************************************/
/* Enables/Disables the DMA Support for the SPI interface                                                       */
/****************************************************************************************************************/
#define CFG_SPI_DMA_SUPPORT

/****************************************************************************************************************/
/* Control the memory protection feature of the SPI flash chip through its Status Register 1.                   */
/****************************************************************************************************************/
#undef CFG_SPI_FLASH_MEM_PROTECT_USING_STATUS_REG1

#endif // _DA1458X_CONFIG_BASIC_H_
