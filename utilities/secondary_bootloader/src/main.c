/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Secondary Bootloader application
 *
 * Copyright (C) 2012-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include "arch_system.h"
#include "datasheet.h"
#include "syscntl.h"
#include "user_periph_setup.h"
#include "uart_booter.h"
#include "i2c_eeprom.h"
#include "spi_flash.h"
#include "bootloader.h"
#include "gpio.h"
#include "uart.h"
#include "hw_otpc.h"

#if defined (__DA14531__)
#include "adc.h"
#include "otp_cs.h"
#include "otp_hdr.h"
#endif

#if defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief Enable DA14531 HW reset functionality.
 ****************************************************************************************
 */
static void enable_hw_reset(void)
{
    GPIO_DisablePorPin();
    GPIO_SetPinFunction(GPIO_PORT_0, GPIO_PIN_0, INPUT_PULLDOWN, PID_GPIO);
    // Wait 10us for the line to be discharged
    for (int i = 0; i < 4 * 10; i++);
    GPIO_Enable_HW_Reset();
}
#endif

/**
 ****************************************************************************************
 * @brief Run user application after receiving a binany from uart or reading the
 *        binary from a non-volatile bootable memory
 ****************************************************************************************
 */
static void Start_run_user_application(void)
{
    volatile unsigned short tmp;

    if (*(volatile unsigned*)0x07FC0000 & 0x07FC0000)
    {
        tmp = GetWord16(SYS_CTRL_REG);
        //Remap to SysRAM and trigger SW Reset
        tmp = (tmp & ~REMAP_ADR0) | 0x2;
        tmp |= SW_RESET;
        SetWord16(SYS_CTRL_REG, tmp);
    }
    tmp = GetWord16(SYS_CTRL_REG);
    //  tmp&=~0x0003;
    tmp |= 0x8000;
    SetWord16(SYS_CTRL_REG,tmp);
}

extern int FwDownload(void);
extern void uart_receive_callback(uint16_t length);
extern void uart_error_callback(uart_t *uart, uint8_t uart_err_status);
extern int FwDownloadOneWireUART(void);

#if defined (UART_SUPPORTED) || defined (ONE_WIRE_UART_SUPPORTED)
static const uart_cfg_t uart_cfg =
{
    .baud_rate = UART_FRAC_BAUDRATE,
    .data_bits = UART_DATABITS_8,
    .parity = UART_PARITY_NONE,
    .stop_bits = UART_STOPBITS_1,
    .auto_flow_control = UART_AFCE_DIS,
    .use_fifo = UART_FIFO_EN,
    .tx_fifo_tr_lvl = UART_TX_FIFO_LEVEL_0,
    .rx_fifo_tr_lvl = UART_RX_FIFO_LEVEL_0,
    .intr_priority = 2,
    .uart_rx_cb = uart_receive_callback,
    .uart_err_cb = uart_error_callback,
};
#endif

/************** Secondary bootloader configuration section **************/
#if defined (__DA14531__)
sb_conf_t sb_conf;
#endif

#if (USE_SPI_BOOT_SPECIFIC_PORT_MAPPING)
/**
 ****************************************************************************************
 * @brief Set the default spi configuration
 ****************************************************************************************
 */
static void set_default_spi_conf(void)
{
    sb_conf.spi_en_port = SPI_EN_PORT;
    sb_conf.spi_en_pin = SPI_EN_PIN;

    sb_conf.spi_clk_port = SPI_CLK_PORT;
    sb_conf.spi_clk_pin = SPI_CLK_PIN;

    sb_conf.spi_mosi_port = SPI_DO_PORT;
    sb_conf.spi_mosi_pin = SPI_DO_PIN;

    sb_conf.spi_miso_port = SPI_DI_PORT;
    sb_conf.spi_miso_pin = SPI_DI_PIN;
}
#endif

#if (USE_UART_BOOT_LOCK_OPTION)
/**
****************************************************************************************
* @brief Read the OTP reserved for future needs region (0x07F87FEC) to determine if
*        UART must be enabled or not.
* @note: The reserved for future needs space in OTP header is 20 bytes.
****************************************************************************************
*/
static void sb_read_uart_status(void)
{
    sb_conf.is_uart_enabled = true;
    const uint32_t otp_rfu_first_address = OTP_HDR_RESERVED_ADDR;
    const uint32_t otp_rfu_last_address = otp_rfu_first_address + 16;
    const uint32_t otp_uart_enable_value = 0x54524155; // ASCII capital letters for UART in reverse
    for (uint32_t address = otp_rfu_first_address; address <= otp_rfu_last_address; address += 4)
    {
        // Read all reserved for future needs space
        uint32_t value = GetWord32(address);
        if (value == otp_uart_enable_value)
        {
            sb_conf.is_uart_enabled = false;
        }
    }
}
#endif

#if (USE_SPI_BOOT_SPECIFIC_PORT_MAPPING)
/**
****************************************************************************************
* @brief Read the OTP boot specific config and boot specific port mapping
****************************************************************************************
*/
static void sb_read_boot_specific_config(void)
{
    // Readout the OTP specific flag
    uint32_t otp_boot_specific_config = (GetWord32(OTP_HDR_BOOT_CONFIG_ADDR)) & 0x000000FF;
    if (otp_boot_specific_config == BOOT_SPECIFIC)
    {
        // Readout specific SPI configuration (ports and pin mapping) from OTP
        uint32_t otp_boot_specific_port_mapping = GetWord32(OTP_HDR_BOOT_MAPPING_ADDR);

        bool is_configuration_valid = true; // Assume that the configuration is valid
        uint8_t otp_boot_specific_port_mapping_byte[4] = {0};
        for (int i = 0; i < 4; i++)
        {
            otp_boot_specific_port_mapping_byte[i] = (uint8_t)((otp_boot_specific_port_mapping >> (i * 8)) & 0xFF);
            GPIO_PIN pin = (GPIO_PIN)((uint8_t)(otp_boot_specific_port_mapping_byte[0] & 0x0F));
            if (!GPIO_is_valid(GPIO_PORT_0, pin))
            {
                is_configuration_valid = false;
                break;
            }
        }
        // A valid pin mapping is found and default values should be changed.
        // In DA14531 the port is always zero.
        if (is_configuration_valid)
        {
            sb_conf.spi_en_port = GPIO_PORT_0;
            sb_conf.spi_en_pin = (GPIO_PIN)((uint8_t)(otp_boot_specific_port_mapping_byte[1] & 0x0F));

            sb_conf.spi_clk_port = GPIO_PORT_0;
            sb_conf.spi_clk_pin = (GPIO_PIN)((uint8_t)(otp_boot_specific_port_mapping_byte[0] & 0x0F));

            sb_conf.spi_mosi_port = GPIO_PORT_0;
            sb_conf.spi_mosi_pin = (GPIO_PIN)((uint8_t)(otp_boot_specific_port_mapping_byte[2] & 0x0F));

            sb_conf.spi_miso_port = GPIO_PORT_0;
            sb_conf.spi_miso_pin = (GPIO_PIN)((uint8_t)(otp_boot_specific_port_mapping_byte[3] & 0x0F));
        }
        else
        {
            // One or more pins are invalid. Set the default configuration option.
            set_default_spi_conf();
        }
    }
    else
    {
        // No boot specific config found
        set_default_spi_conf();
    }
}
#endif

/**
****************************************************************************************
* @brief Read the OTP header to determine the boot specific port mapping and the UART
         enable/disable mode.
****************************************************************************************
*/
void sb_read_otp_header(void)
{
#if (USE_UART_BOOT_LOCK_OPTION)
    sb_read_uart_status();
#endif

#if (USE_SPI_BOOT_SPECIFIC_PORT_MAPPING)
    sb_read_boot_specific_config();
#endif
}

#if defined (__DA14531__)
void sb_set_debugger_mode(bool mode)
{
    sb_conf.is_debugger_enabled = mode;
}

/**
****************************************************************************************
* @brief Enable the debugger if it is not disabled by the OTP CS configuration script
****************************************************************************************
*/
static void enable_debugger(void)
{
    if (sb_conf.is_debugger_enabled)
    {
        syscntl_load_debugger_cfg();
    }
}
#endif

/**
 ****************************************************************************************
 * @brief Configure the SPI pins to be inputs with pulldown enabled. The CS will be held
          high if cs_status variable is true or low otherwise
 ****************************************************************************************
 */
#if defined (SPI_FLASH_SUPPORTED)
static void reset_spi_gpios(bool cs_status)
{
#if (USE_SPI_BOOT_SPECIFIC_PORT_MAPPING)
    GPIO_ConfigurePin(sb_conf.spi_en_port, sb_conf.spi_en_pin, INPUT_PULLDOWN, PID_GPIO, cs_status);
    GPIO_ConfigurePin(sb_conf.spi_clk_port, sb_conf.spi_clk_pin, INPUT_PULLDOWN, PID_GPIO, false);
    GPIO_ConfigurePin(sb_conf.spi_mosi_port, sb_conf.spi_mosi_pin, INPUT_PULLDOWN, PID_GPIO, false);
    GPIO_ConfigurePin(sb_conf.spi_miso_port, sb_conf.spi_miso_pin, INPUT_PULLDOWN, PID_GPIO, false);
#else
    GPIO_ConfigurePin(SPI_EN_PORT, SPI_EN_PIN, INPUT_PULLDOWN, PID_GPIO, cs_status);
    GPIO_ConfigurePin(SPI_CLK_PORT, SPI_CLK_PIN, INPUT_PULLDOWN, PID_GPIO, false);
    GPIO_ConfigurePin(SPI_DO_PORT, SPI_DO_PIN, INPUT_PULLDOWN, PID_GPIO, false);
    GPIO_ConfigurePin(SPI_DI_PORT, SPI_DI_PIN, INPUT_PULLDOWN, PID_GPIO, false);
#endif
}
#endif // SPI_FLASH_SUPPORTED

int main(void)
{
#if defined (SPI_FLASH_SUPPORTED ) || defined (I2C_EEPROM_SUPPORTED)
    int ret = -1;
#endif

#if defined (UART_SUPPORTED) || defined (ONE_WIRE_UART_SUPPORTED)
    int fw_size;
    unsigned int i;
    char *from ;
    char *to;
#endif

    /*
     ************************************************************************************
     * Watchdog start (maximum value) - PLEASE DISABLE WATCHDOG WHILE TESTING!
     ************************************************************************************
     */
    SetBits(WATCHDOG_CTRL_REG, NMI_RST, 1);         // WDOG will generate HW reset upon expiration
    SetWord16(WATCHDOG_REG, WATCHDOG_REG_RESET);    // Reset WDOG
    SetWord16(RESET_FREEZE_REG, FRZ_WDOG);          // Start WDOG

    system_init();

    while (1)
    {
#if defined (UART_SUPPORTED)
#if (USE_UART_BOOT_LOCK_OPTION)
        if (sb_conf.is_uart_enabled)
        {
#endif
            if (GPIO_GetPinStatus(UART_GPIO_PORT, UART_RX_PIN))
            {
                uart_initialize(UART1, &uart_cfg);

                // Initialize UART pins
                GPIO_ConfigurePin(UART_GPIO_PORT, UART_TX_PIN, OUTPUT, PID_UART1_TX, false);
                GPIO_ConfigurePin(UART_GPIO_PORT, UART_RX_PIN, INPUT, PID_UART1_RX, false);

                // Download FW
                fw_size = FwDownload();

                uart_disable(UART1);

                if (fw_size > 0)
                {
                    from = (char *)SYSRAM_COPY_BASE_ADDRESS;
                    to   = (char *)SYSRAM_BASE_ADDRESS;
                    for (i = 0; i < fw_size; i++)
                    {
                        to[i] = from[i];
                    }

                    SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
                    SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG

                    // Reset UART pins
                    GPIO_ConfigurePin(UART_GPIO_PORT, UART_TX_PIN, INPUT, PID_GPIO, false);
                    GPIO_ConfigurePin(UART_GPIO_PORT, UART_RX_PIN, INPUT, PID_GPIO, false);

#if defined (__DA14531__)
                    enable_hw_reset();
                    enable_debugger();
#endif

                    Start_run_user_application();
                }
            }
#if (USE_UART_BOOT_LOCK_OPTION)
        }
#endif
#endif // defined (UART_SUPPORTED)

#if defined (ONE_WIRE_UART_SUPPORTED)
        {
#if (USE_UART_BOOT_LOCK_OPTION)
            if (sb_conf.is_uart_enabled)
            {
#endif
                uart_initialize(UART1, &uart_cfg);

                // Initialize UART pins
                fw_size = FwDownloadOneWireUART();
                uart_disable(UART1);
                if (fw_size > 0)
                {
                    from = (char *)SYSRAM_COPY_BASE_ADDRESS;
                    to   = (char *)SYSRAM_BASE_ADDRESS;
                    for (i = 0; i < fw_size; i++)
                    {
                        to[i] = from[i];
                    }

                    SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
                    SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG

                    // Reset the 1-wire UART pin
                    GPIO_ConfigurePin(UART_GPIO_PORT, GPIO_PIN_5, INPUT, PID_GPIO, false);

#if defined (__DA14531__)
                    enable_hw_reset();
                    enable_debugger();
#endif

                    Start_run_user_application();
                }
#if (USE_UART_BOOT_LOCK_OPTION)
            }
#endif
        }
#endif // defined (ONE_WIRE_UART_SUPPORTED)

#if defined (SPI_FLASH_SUPPORTED)
        ret = spi_loadActiveImage();
        if (!ret)
        {
            SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
            SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG

            spi_release();

            // Reset SPI pins but keep the CS high
            reset_spi_gpios(true);

#if defined (__DA14531__)
            enable_hw_reset();
            enable_debugger();
#endif

            Start_run_user_application();
        }
        spi_release();
        // Reset SPI pins including CS
        reset_spi_gpios(false);
        // Disable the external SPI flash power pin if it was previously enabled
        spi_flash_disable_power_pin();
#endif

#if defined (I2C_EEPROM_SUPPORTED)
        ret = eeprom_loadActiveImage();
        if (!ret)
        {
            SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
            SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG

            i2c_eeprom_release();

            // reset I2C GPIOs
            GPIO_ConfigurePin(I2C_SCL_PORT, I2C_SCL_PIN, INPUT_PULLDOWN, PID_GPIO, false);
            GPIO_ConfigurePin(I2C_SDA_PORT, I2C_SDA_PIN, INPUT_PULLDOWN, PID_GPIO, false);

#if defined (__DA14531__)
            enable_hw_reset();
            enable_debugger();
#endif

            Start_run_user_application();
        }
#endif

#if (USE_ENTER_LOOP_WITH_DEBUGGER_ENABLED_ON_NO_APP)
        // If no app is found and debugger is enabled then behave as the primary booter
        if (sb_conf.is_debugger_enabled)
        {
            // Preparing to enter the endless loop with debugger enabled.
            // The rst pin should be enabled as well.
            enable_hw_reset();
            enable_debugger();
            while(1)
            {
                SetWord16(WATCHDOG_REG, WATCHDOG_REG_RESET);
            }
        }
#endif
    } // while (1)
}

