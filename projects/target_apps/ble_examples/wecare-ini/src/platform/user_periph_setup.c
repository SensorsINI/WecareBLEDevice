/**
 ****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief Peripherals setup and initialization.
 *
 * Copyright (c) 2015-2019 Dialog Semiconductor. All rights reserved.
 *
 * This software ("Software") is owned by Dialog Semiconductor.
 *
 * By using this Software you agree that Dialog Semiconductor retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Dialog Semiconductor is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Dialog Semiconductor products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * DIALOG SEMICONDUCTOR BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "user_periph_setup.h"
#include "datasheet.h"
#include "system_library.h"
#include "rwip_config.h"
#include "gpio.h"
#include "uart.h"
#include "syscntl.h"

#include "timer0.h"
#include "timer0_2.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 *
 * @return void
 ****************************************************************************************
 */

#if DEVELOPMENT_DEBUG

void GPIO_reservations(void)
{
/*
    i.e. to reserve P0_1 as Generic Purpose I/O:
    RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);
*/

#if (BLE_BATT_SERVER && USE_BAT_LEVEL_ALERT)
    // Battery alert LED
    RESERVE_GPIO(RED_LED, GPIO_BAT_LED_PORT, GPIO_BAT_LED_PIN, PID_GPIO);
#endif

#if defined (CFG_PRINTF_UART2)
    // UART2 Tx pin
    RESERVE_GPIO(UART2_TX, UART2_TX_PORT, UART2_TX_PIN, PID_UART2_TX);
#endif
		 RESERVE_GPIO(UART1_TX, UART1_TX_PORT, UART1_TX_PIN, PID_UART1_TX);
	
#if defined (CFG_SPI_FLASH_ENABLE) && !defined (__DA14586__)
    // SPI Flash
    RESERVE_GPIO(SPI_EN, SPI_EN_PORT, SPI_EN_PIN, PID_SPI_EN);
    RESERVE_GPIO(SPI_CLK, SPI_CLK_PORT, SPI_CLK_PIN, PID_SPI_CLK);
    RESERVE_GPIO(SPI_DO, SPI_DO_PORT, SPI_DO_PIN, PID_SPI_DO);
    RESERVE_GPIO(SPI_DI, SPI_DI_PORT, SPI_DI_PIN, PID_SPI_DI);
#endif
}

#endif

void set_pad_functions(void)
{
/*
    i.e. to set P0_1 as Generic purpose Output:
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_1, OUTPUT, PID_GPIO, false);
*/

#if defined (__DA14586__)
    // Disallow spontaneous DA14586 SPI Flash wake-up
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_3, OUTPUT, PID_GPIO, true);
#else
    // Disallow spontaneous SPI Flash wake-up
    GPIO_ConfigurePin(SPI_EN_PORT, SPI_EN_PIN, OUTPUT, PID_SPI_EN, true);
#endif

#if defined (CFG_PRINTF_UART2)
    // Configure UART2 TX Pad
    GPIO_ConfigurePin(UART2_TX_PORT, UART2_TX_PIN, OUTPUT, PID_UART2_TX, false);
#endif

    // GPIO_ConfigurePin(GPIO_LED_PORT, GPIO_LED_PIN, OUTPUT, PID_GPIO, false);

#if defined (CFG_SPI_FLASH_ENABLE)
    // SPI Flash
    GPIO_ConfigurePin(SPI_EN_PORT, SPI_EN_PIN, OUTPUT, PID_SPI_EN, true);
    GPIO_ConfigurePin(SPI_CLK_PORT, SPI_CLK_PIN, OUTPUT, PID_SPI_CLK, false);
    GPIO_ConfigurePin(SPI_DO_PORT, SPI_DO_PIN, OUTPUT, PID_SPI_DO, false);
    GPIO_ConfigurePin(SPI_DI_PORT, SPI_DI_PIN, INPUT, PID_SPI_DI, false);
#endif

    // Configure SPI2 Pad
//#if DEVELOPMENT_DEBUG
//    RESERVE_GPIO(SPI2_CLK, SPI2_CLK_PORT, SPI2_CLK_PIN, PID_SPI_CLK);
//    RESERVE_GPIO(SPI2_DO, SPI2_DO_PORT, SPI2_DO_PIN, PID_SPI_DO);
//    RESERVE_GPIO(SPI2_DI, SPI2_DI_PORT, SPI2_DI_PIN, PID_SPI_DI);
//		
//    RESERVE_GPIO(SPI2_IO_CS, SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, PID_GPIO);
//		RESERVE_GPIO(SPI2_ADC1_CS, SPI2_ADC1_CS_PORT, SPI2_ADC1_CS_PIN, PID_SPI_EN1);
//		RESERVE_GPIO(SPI2_ADC2_CS, SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, PID_SPI_EN);
//		RESERVE_GPIO(SPI2_DAC_CS, SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, PID_GPIO);		
//#endif
//    GPIO_ConfigurePin(SPI2_CLK_PORT, SPI2_CLK_PIN, OUTPUT, PID_SPI_CLK, false);
//    GPIO_ConfigurePin(SPI2_DO_PORT, SPI2_DO_PIN, OUTPUT, PID_SPI_DO, false);
//    GPIO_ConfigurePin(SPI2_DI_PORT, SPI2_DI_PIN, INPUT, PID_SPI_DI, false);
//		
//		// Configure SPI extra CS
//		GPIO_ConfigurePin(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, OUTPUT, PID_GPIO, true);
//		GPIO_ConfigurePin(SPI2_ADC1_CS_PORT, SPI2_ADC1_CS_PIN, OUTPUT, PID_SPI_EN1, true);
//		GPIO_ConfigurePin(SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, OUTPUT, PID_SPI_EN, true);
//		GPIO_ConfigurePin(SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, OUTPUT, PID_GPIO, true);
}

#if defined (CFG_PRINTF_UART2)
// Configuration struct for UART2
static const uart_cfg_t uart_cfg = {
    .baud_rate = UART2_BAUDRATE,
    .data_bits = UART2_DATABITS,
    .parity = UART2_PARITY,
    .stop_bits = UART2_STOPBITS,
    .auto_flow_control = UART2_AFCE,
    .use_fifo = UART2_FIFO,
    .tx_fifo_tr_lvl = UART2_TX_FIFO_LEVEL,
    .rx_fifo_tr_lvl = UART2_RX_FIFO_LEVEL,
    .intr_priority = 2,
};
#endif

//volatile uint8_t timeout_expiration;
static tim0_2_clk_div_config_t clk_div_config =
{
    .clk_div  = TIM0_2_CLK_DIV_8
};

/**
 ****************************************************************************************
 * @brief Timer0 general user callback function
 *
 ****************************************************************************************
 */
static void timer0_user_callback_function(void)
{
  	// GPIO_SetActive(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN);
		// spi_ctrl_reg_spi_fifo_reset_setf(SPI_BIT_EN);
	  timer0_2_clk_disable();
}

// Timer to control CS signal length
void timer0_setup(uint32_t times_microseconds)
{
    // Stop timer for enter settings
    timer0_stop();

    // register callback function for SWTIM_IRQn irq
    timer0_register_callback(timer0_user_callback_function);

    // Enable the Timer0/Timer2 input clock
    timer0_2_clk_enable();

    // Set the Timer0/Timer2 input clock division factor to 8, so 16 MHz / 8 = 2 MHz input clock
    timer0_2_clk_div_set(&clk_div_config);

    // clear PWM settings register to not generate PWM
    timer0_set_pwm_high_counter(0);
    timer0_set_pwm_low_counter(0);

    // Set timer with 2MHz source clock divided by 10 so Fclk = 2MHz/1 = 2MHz
    timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_NO_DIV);

    // Clock is 2M, so multiple by 2 changes it to 1M
    timer0_set_pwm_on_counter(times_microseconds * 2);

    // Enable SWTIM_IRQn irq
    // timer0_enable_irq(); 		

    // Disable the Timer0/Timer2 input clock
    timer0_2_clk_disable();
}

void periph_init(void)
{
#if defined (__DA14531__)
    // In Boost mode enable the DCDC converter to supply VBAT_HIGH for the used GPIOs
    syscntl_dcdc_turn_on_in_boost(SYSCNTL_DCDC_LEVEL_3V0);
#else
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));
    SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_ENABLE, 1);
#endif

    // ROM patch
    patch_func();

    // Initialize peripherals
#if defined (CFG_PRINTF_UART2)
    // Initialize UART2
    uart_initialize(UART2, &uart_cfg);
#endif
 
#if defined (CFG_SPI_FLASH_ENABLE)
    // Configure SPI Flash environment
    spi_flash_configure_env(&spi_flash_cfg);

    // Initialize SPI
    spi_initialize(&spi_cfg);
#endif

	  // Disable P0 used as Reset (default function)
	  // We need P0 as the MOSI signal
	  GPIO_Disable_HW_Reset();

    // Set pad functionality
    set_pad_functions();

    // Enable the pads
    GPIO_set_pad_latch_en(true);
		
		// Disable debugger
		// SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, NO_SWD);   
		
		timer0_setup(25);
}

// Configuration struct for UART
static const uart_cfg_t uart_cfg = {
    // Set Baud Rate
    .baud_rate = UART_BAUDRATE_115200,
    // Set data bits
    .data_bits = UART_DATABITS_8,
    // Set parity
    .parity = UART_PARITY_NONE,
    // Set stop bits
    .stop_bits = UART_STOPBITS_1,
    // Set flow control
    .auto_flow_control = UART_AFCE_DIS,
    // Set FIFO enable
    .use_fifo = UART_FIFO_EN,
    // Set Tx FIFO trigger level
    .tx_fifo_tr_lvl = UART_TX_FIFO_LEVEL_0,
    // Set Rx FIFO trigger level
    .rx_fifo_tr_lvl = UART_RX_FIFO_LEVEL_0,
    // Set interrupt priority
    .intr_priority = 2,
#if defined (CFG_UART_DMA_SUPPORT)
    // Set UART DMA Channel Pair Configuration
    .uart_dma_channel = UART_DMA_CHANNEL_01,
    // Set UART DMA Priority
    .uart_dma_priority = DMA_PRIO_0,
#endif
};

void uart_periph_init(uart_t *uart)
{
    // Initialize UART
    uart_initialize(uart, &uart_cfg);

    GPIO_ConfigurePin(UART1_TX_PORT, UART1_TX_PIN, OUTPUT, PID_UART1_TX, false);

    // Enable the pads
    GPIO_set_pad_latch_en(true);
}
