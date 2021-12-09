/**
 ****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief Peripherals setup and initialization.
 *
 * Copyright (C) 2015-2021 Dialog Semiconductor.
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
#include "user_periph_setup.h"
#include "gpio.h"
#include "syscntl.h"
#include "bootloader.h"
#if (USE_SPI_FLASH_EXTENSIONS)
#include "spi_flash.h"
#endif

void periph_init(void)
{
#if defined (__DA14531__)
#if defined (SPI_FLASH_SUPPORTED) || defined (UART_SUPPORTED) || defined (ONE_WIRE_UART_SUPPORTED)
    GPIO_Disable_HW_Reset();
#endif
    // In Boost mode enable the DCDC converter to supply VBAT_HIGH for the used GPIOs
    syscntl_dcdc_turn_on_in_boost(SYSCNTL_DCDC_LEVEL_3V0);
#endif

#if !defined (__DA14531__)
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));
#endif

    // Enable the pads
    GPIO_set_pad_latch_en(true);

#if (USE_SPI_FLASH_EXTENSIONS)
    spi_flash_enable_power_pin();
#endif
}
