/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup GPIO
 * @brief GPIO driver API
 * @{
 *
 * @file gpio.h
 *
 * @brief Hardware GPIO abstraction layer API.
 *
 * Copyright (C) 2012-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _GPIO_H_
#define _GPIO_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>
#include "datasheet.h"
#include "arch.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// GPIO mode
typedef enum {
    /// Input
    INPUT = 0,

    /// Input pullup
    INPUT_PULLUP = 0x100,

    /// Input pulldown
    INPUT_PULLDOWN = 0x200,

    /// Output
    OUTPUT = 0x300,
} GPIO_PUPD;

/// GPIO power rail
typedef enum {
    /// GPIO power rail at 3V
    GPIO_POWER_RAIL_3V = 0,

    /// GPIO power rail at 1V
    GPIO_POWER_RAIL_1V = 1,
} GPIO_POWER_RAIL;

/// GPIO port
typedef enum {
    /// Port 0
    GPIO_PORT_0 = 0U,
#if !defined (__DA14531__)
    /// Port 1
    GPIO_PORT_1 = 1U,

    /// Port 2
    GPIO_PORT_2 = 2U,

    /// Port 3
    GPIO_PORT_3 = 3U,

    /// Port 3 remap
    GPIO_PORT_3_REMAP = 4U,
#endif
} GPIO_PORT;

#if defined (__DA14531__)
/// GPIO pin
typedef enum {
    /// Pin 0
    GPIO_PIN_0 = 0U,

    /// Pin 1
    GPIO_PIN_1 = 1U,

    /// Pin 2
    GPIO_PIN_2 = 2U,

    /// Pin 3
    GPIO_PIN_3 = 3U,

    /// Pin 4
    GPIO_PIN_4 = 4U,

    /// Pin 5
    GPIO_PIN_5 = 5U,
#if !defined (CFG_WLCSP_PACKAGE)
    /// Pin 6
    GPIO_PIN_6 = 6U,

    /// Pin 7
    GPIO_PIN_7 = 7U,

    /// Pin 8
    GPIO_PIN_8 = 8U,

    /// Pin 9
    GPIO_PIN_9 = 9U,

    /// Pin 10
    GPIO_PIN_10 = 10U,

    /// Pin 11
    GPIO_PIN_11 = 11U
#endif
} GPIO_PIN;
#else // (DA14585/586)
/// GPIO pin
typedef enum {
    /// Pin 0
    GPIO_PIN_0 = 0U,

    /// Pin 1
    GPIO_PIN_1 = 1U,

    /// Pin 2
    GPIO_PIN_2 = 2U,

    /// Pin 3
    GPIO_PIN_3 = 3U,

    /// Pin 4
    GPIO_PIN_4 = 4U,

    /// Pin 5
    GPIO_PIN_5 = 5U,

    /// Pin 6
    GPIO_PIN_6 = 6U,

    /// Pin 7
    GPIO_PIN_7 = 7U,

    /// Pin 8
    GPIO_PIN_8 = 8U,

    /// Pin 9
    GPIO_PIN_9 = 9U,
} GPIO_PIN;
#endif

/// GPIO pin function
#if defined (__DA14531__)
typedef enum {
    /// GPIO
    PID_GPIO = FUNC_GPIO,

    /// UART1_RX
    PID_UART1_RX = FUNC_UART_RX,

    /// UART1_TX
    PID_UART1_TX = FUNC_UART_TX,

    /// UART2_RX
    PID_UART2_RX = FUNC_UART2_RX,

    /// UART2_TX
    PID_UART2_TX = FUNC_UART2_TX,

    /// SYS_CLK
    PID_SYS_CLK = FUNC_SYS_CLK,

    /// LP_CLK
    PID_LP_CLK = FUNC_LP_CLK,

    /// I2C_SCL
    PID_I2C_SCL = FUNC_I2C_SCL,

    /// I2C_SDA
    PID_I2C_SDA = FUNC_I2C_SDA,

    /// PWM5
    PID_PWM5 = FUNC_PWM5,

    /// PWM6
    PID_PWM6 = FUNC_PWM6,

    /// PWM7
    PID_PWM7 = FUNC_PWM7,

    /// ADC
    PID_ADC = FUNC_ADC,

    /// PWM0
    PID_PWM0 = FUNC_PWM0,

    /// PWM1
    PID_PWM1 = FUNC_PWM1,

    /// BLE_DIAG
    PID_BLE_DIAG = FUNC_BLE_DIAG,

    /// UART1_CTSN
    PID_UART1_CTSN = FUNC_UART_CTSN,

    /// UART1_RTSN
    PID_UART1_RTSN = FUNC_UART_RTSN,

    /// PWM2
    PID_PWM2 = FUNC_PWM2,

    /// PWM3
    PID_PWM3 = FUNC_PWM3,

    /// PWM4
    PID_PWM4 = FUNC_PWM4,

    /// SPI_DI
    PID_SPI_DI = FUNC_SPI_DI,

    /// SPI_DO
    PID_SPI_DO = FUNC_SPI_DO,

    /// SPI_CLK
    PID_SPI_CLK = FUNC_SPI_CLK,

    /// SPI_CSN0
    PID_SPI_EN = FUNC_SPI_CSN0,

    /// SPI_CSN1
    PID_SPI_EN1 = FUNC_SPI_CSN1
} GPIO_FUNCTION;
#else // (DA14585/586)
typedef enum {
    /// GPIO
    PID_GPIO = FUNC_GPIO,

    /// UART1_RX
    PID_UART1_RX = FUNC_UART_RX,

    /// UART1_TX
    PID_UART1_TX = FUNC_UART_TX,

    /// UART2_RX
    PID_UART2_RX = FUNC_UART2_RX,

    /// UART2_TX
    PID_UART2_TX = FUNC_UART2_TX,

    /// SPI_DI
    PID_SPI_DI = FUNC_SPI_DI,

    /// SPI_DO
    PID_SPI_DO = FUNC_SPI_DO,

    /// SPI_CLK
    PID_SPI_CLK = FUNC_SPI_CLK,

    /// SPI_EN
    PID_SPI_EN = FUNC_SPI_EN,

    /// I2C_SCL
    PID_I2C_SCL = FUNC_I2C_SCL,

    /// I2C_SDA
    PID_I2C_SDA = FUNC_I2C_SDA,

    /// UART1_IRDA_RX
    PID_UART1_IRDA_RX = FUNC_UART_IRDA_RX,

    /// UART1_IRDA_TX
    PID_UART1_IRDA_TX = FUNC_UART_IRDA_TX,

    /// UART2_IRDA_RX
    PID_UART2_IRDA_RX = FUNC_UART2_IRDA_RX,

    /// UART2_IRDA_TX
    PID_UART2_IRDA_TX = FUNC_UART2_IRDA_TX,

    /// ADC
    PID_ADC = FUNC_ADC,

    /// PWM0
    PID_PWM0 = FUNC_PWM0,

    /// PWM1
    PID_PWM1 = FUNC_PWM1,

    /// BLE_DIAG
    PID_BLE_DIAG = FUNC_BLE_DIAG,

    /// UART1_CTSN
    PID_UART1_CTSN = FUNC_UART_CTSN,

    /// UART1_RTSN
    PID_UART1_RTSN = FUNC_UART_RTSN,

    /// UART2_CTSN
    PID_UART2_CTSN = FUNC_UART2_CTSN,

    /// UART2_RTSN
    PID_UART2_RTSN = FUNC_UART2_RTSN,

    /// PWM2
    PID_PWM2 = FUNC_PWM2,

    /// PWM3
    PID_PWM3 = FUNC_PWM3,

    /// PWM4
    PID_PWM4 = FUNC_PWM4,

    /// PCM_DI
    PID_PCM_DI = FUNC_PCM_DI,

    /// PCM_DO
    PID_PCM_DO = FUNC_PCM_DO,

    /// PCM_FSC
    PID_PCM_FSC = FUNC_PCM_FSC,

    /// PCM_CLK
    PID_PCM_CLK = FUNC_PCM_CLK,

    /// PDM_DATA
    PID_PDM_DATA = FUNC_PDM_DATA,

    /// PDM_CLK
    PID_PDM_CLK = FUNC_PDM_CLK
} GPIO_FUNCTION;
#endif

/// GPIO IRQ input level
typedef enum {
    /// High
    GPIO_IRQ_INPUT_LEVEL_HIGH = 0U,

    /// Low
    GPIO_IRQ_INPUT_LEVEL_LOW  = 1U
} GPIO_IRQ_INPUT_LEVEL;

/// POR pin polarity (only for GPIO)
typedef enum {
    /// Low polarity
    GPIO_POR_PIN_POLARITY_LOW  = 0U,

    /// High polarity
    GPIO_POR_PIN_POLARITY_HIGH = 1U
} GPIO_POR_PIN_POLARITY;

/// GPIO base address
#define GPIO_BASE   P0_DATA_REG

/**
 ****************************************************************************************
 * @brief Helper macro that calculates the value to be written into POR_PIN_REG register.
 * @param[in] port          GPIO port.
 * @param[in] pin           GPIO pin.
 * @param[in] polarity      0 = Active low, 1 = Active high.
 * @return The value to be written into POR_PIN_REG register. If the port/pin/polarity
 *         values are illegal, the returned value is POR_PIN_REG_RESET (reset value).
 ****************************************************************************************
 */
#if defined (__DA14531__)
#if !defined(CFG_WLCSP_PACKAGE)
#define GPIO_POR_PIN_REG(port, pin, polarity)                                                                                            \
        ( ((port == GPIO_PORT_0) && (pin <= GPIO_PIN_11) && (polarity <= GPIO_POR_PIN_POLARITY_HIGH)) ? ((polarity << 7) | (pin + 1))  : \
        POR_PIN_REG_RESET )
#else
#define GPIO_POR_PIN_REG(port, pin, polarity)                                                                                            \
        ( ((port == GPIO_PORT_0) && (pin <= GPIO_PIN_5) && (polarity <= GPIO_POR_PIN_POLARITY_HIGH)) ? ((polarity << 7) | (pin + 1))  :  \
        POR_PIN_REG_RESET )
#endif
#else
#define GPIO_POR_PIN_REG(port, pin, polarity)                                                                                            \
        ( ((port == GPIO_PORT_0) && (pin <= GPIO_PIN_7) && (polarity <= GPIO_POR_PIN_POLARITY_HIGH)) ? ((polarity << 7) | (pin + 1))  :  \
          ((port == GPIO_PORT_1) && (pin <= GPIO_PIN_5) && (polarity <= GPIO_POR_PIN_POLARITY_HIGH)) ? ((polarity << 7) | (pin + 9))  :  \
          ((port == GPIO_PORT_2) && (pin <= GPIO_PIN_9) && (polarity <= GPIO_POR_PIN_POLARITY_HIGH)) ? ((polarity << 7) | (pin + 15)) :  \
          ((port == GPIO_PORT_3) && (pin <= GPIO_PIN_7) && (polarity <= GPIO_POR_PIN_POLARITY_HIGH)) ? ((polarity << 7) | (pin + 25)) :  \
        POR_PIN_REG_RESET )
#endif

/**
 ****************************************************************************************
 * @brief Helper macro used for pin definition structure.
 * @param[in] name Name indicating pin usage and/or module using that pin.
 * @param[in] port #GPIO_PORT.
 * @param[in] pin #GPIO_PIN.
 * @param[in] func #GPIO_FUNCTION.
 ****************************************************************************************
 */
#if DEVELOPMENT_DEBUG && !defined(GPIO_DRV_PIN_ALLOC_MON_DISABLED)
#define RESERVE_GPIO( name, port, pin, func )   { GPIO[port][pin] = (GPIO[port][pin] != 0) ? (-1) : 1; GPIO_status |= ((uint64_t)GPIO[port][pin] << pin) << (port * 16);}
#else
#define RESERVE_GPIO( name, port, pin, func )   {}
#endif //DEVELOPMENT_DEBUG && !defined(GPIO_DRV_PIN_ALLOC_MON_DISABLED)

#if DEVELOPMENT_DEBUG
#ifndef GPIO_DRV_PIN_ALLOC_MON_DISABLED

#if defined (__DA14531__)
#define NO_OF_PORTS 1   // cannot be bigger than 1
// NOTE: Modify for small package!!
#define NO_OF_MAX_PINS_PER_PORT 12  // cannot be bigger than 12
#else
#define NO_OF_PORTS 4   // cannot be bigger than 4
#define NO_OF_MAX_PINS_PER_PORT 10  // cannot be bigger than 10
#endif

extern int GPIO[NO_OF_PORTS][NO_OF_MAX_PINS_PER_PORT];

extern volatile uint64_t GPIO_status;
#endif //GPIO_DRV_PIN_ALLOC_MON_DISABLED
#endif //DEVELOPMENT_DEBUG

#if USE_SPI_FLASH_EXTENSIONS
#define FLASH_GPIO_POWER_PORT                   GPIO_PORT_0
#define FLASH_GPIO_POWER_PIN                    GPIO_PIN_7
#endif

/// Callback type to be associated with GPIO handler.
typedef void (*GPIO_handler_function_t)(void);

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Register Callback function for GPIO IRQ.
 * @param[in] irq       The handler of this IRQ will call the function
 * @param[in] callback  Callback function's reference.
 ****************************************************************************************
 */
void GPIO_RegisterCallback(IRQn_Type irq, GPIO_handler_function_t callback);

/**
 ****************************************************************************************
 * @brief Function to set the interrupt generated by the GPIO pin.
 * @param[in] port          GPIO port
 * @param[in] pin           GPIO pin
 * @param[in] irq           GPIO IRQ
 * @param[in] low_input     TRUE generates an IRQ if the input is low
 * @param[in] release_wait  TRUE waits for key release after interrupt was reset
 * @param[in] debounce_ms   duration of a debounce sequence before an IRQ is generated
 ****************************************************************************************
 */
void GPIO_EnableIRQ(GPIO_PORT port,
                    GPIO_PIN pin,
                    IRQn_Type irq,
                    bool low_input,
                    bool release_wait,
                    uint8_t debounce_ms);

/**
 ****************************************************************************************
 * @brief Returns the GPIO input level that triggers a GPIO IRQ.
 * @param[in] irq   GPIO IRQ number (e.g. GPIO2_IRQn)
 * @return input level that triggers the GPIO IRQ (see GPIO_IRQ_INPUT_LEVEL)
 ****************************************************************************************
 */
GPIO_IRQ_INPUT_LEVEL GPIO_GetIRQInputLevel(IRQn_Type irq);

/**
 ****************************************************************************************
 * @brief Set the GPIO input level that triggers a GPIO IRQ.
 * @param[in] irq       GPIO IRQ (e.g. GPIO2_IRQn)
 * @param[in] level     Input level that triggers the GPIO IRQ (see GPIO_IRQ_INPUT_LEVEL)
 ****************************************************************************************
 */
void GPIO_SetIRQInputLevel(IRQn_Type irq, GPIO_IRQ_INPUT_LEVEL level);

/**
 ****************************************************************************************
 * @brief Function to reset the interrupt generated by the GPIO pin.
 * @param[in] irq   GPIO IRQ
 ****************************************************************************************
 */
void GPIO_ResetIRQ(IRQn_Type irq);

/**
 ****************************************************************************************
 * @brief Initialize the GPIO assignment check variables.
 ****************************************************************************************
 */
void GPIO_init(void);

/**
 ****************************************************************************************
 * @brief Set the pin type and mode
 * @param[in] port     GPIO port
 * @param[in] pin      GPIO pin
 * @param[in] mode     GPIO pin mode. INPUT = 0, INPUT_PULLUP = 0x100, INPUT_PULLDOWN = 0x200, OUTPUT = 0x300,
 * @param[in] function GPIO pin usage. GPIO_FUNCTION enumeration.
 ****************************************************************************************
 */
void GPIO_SetPinFunction(GPIO_PORT port, GPIO_PIN pin, GPIO_PUPD mode,
                                GPIO_FUNCTION function);

/**
 ****************************************************************************************
 * @brief Get the function of the pin
 * @param[in] port     GPIO port
 * @param[in] pin      GPIO pin
 * @return the function of the pin
 ****************************************************************************************
 */
GPIO_FUNCTION GPIO_GetPinFunction(GPIO_PORT port, GPIO_PIN pin);

/**
 ****************************************************************************************
 * @brief Combined function to set the state and the type and mode of the GPIO pin.
 * @param[in] port     GPIO port
 * @param[in] pin      GPIO pin
 * @param[in] mode     GPIO pin mode. INPUT = 0, INPUT_PULLUP = 0x100, INPUT_PULLDOWN = 0x200, OUTPUT = 0x300,
 * @param[in] function GPIO pin usage. GPIO_FUNCTION enumaration.
 * @param[in] high     set to TRUE to set the pin into high else low
 ****************************************************************************************
 */
void GPIO_ConfigurePin(GPIO_PORT port, GPIO_PIN pin, GPIO_PUPD mode,
                              GPIO_FUNCTION function, const bool high);

/**
 ****************************************************************************************
 * @brief Sets a pin high. The GPIO should have been previously configured as output!
 * @param[in] port     GPIO port
 * @param[in] pin      GPIO pin
 ****************************************************************************************
 */
void GPIO_SetActive(GPIO_PORT port, GPIO_PIN pin);

/**
 ****************************************************************************************
 * @brief Sets the GPIO low. The GPIO should have been previously configured as output!
 * @param[in] port     GPIO port
 * @param[in] pin      GPIO pin
 ****************************************************************************************
 */
void GPIO_SetInactive(GPIO_PORT port, GPIO_PIN pin);

/**
 ****************************************************************************************
 * @brief Gets the GPIO status. The GPIO should have been previously configured as input!
 * @param[in] port     GPIO port
 * @param[in] pin      GPIO pin
 * @return bool        TRUE if the pin is high, FALSE if low.
 ****************************************************************************************
*/
bool GPIO_GetPinStatus(GPIO_PORT port, GPIO_PIN pin);

/**
 ****************************************************************************************
 * @brief Set the power source of the GPIO pin.
 * @param[in] port          GPIO port
 * @param[in] pin           GPIO pin
 * @param[in] power_rail    GPIO power rail. See GPIO_POWER_RAIL enumeration.
 ****************************************************************************************
 */
void GPIO_ConfigurePinPower(GPIO_PORT port, GPIO_PIN pin, GPIO_POWER_RAIL power_rail);

/**
 ****************************************************************************************
 * @brief Function to verify if the GPIO port and pin combination is valid.
 * @param[in] port      GPIO port
 * @param[in] pin       GPIO pin
 * @return bool         TRUE if the combination is valid, FALSE if it is not.
 ****************************************************************************************
 */
bool GPIO_is_valid(GPIO_PORT port, GPIO_PIN pin);

/**
 ****************************************************************************************
 * @brief       Controls the latch configuration of the GPIO pads
 * @param[in]   pad_en          When set to true, the mode and state of the pad registers
 *                              will be passed directly to the respective pads.
 *                              When set to false, the state of the pads will be retained.
 *                              Any change on the mode and state of the pad registers will
 *                              not affect the pads until PAD_LATCH_EN field is set to 1.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void GPIO_set_pad_latch_en(bool pad_en)
{
#if defined (__DA14531__)
    SetBits16(PAD_LATCH_REG, PAD_LATCH_EN, pad_en);
#else
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, pad_en);
#endif
}

/**
 ****************************************************************************************
 * @brief Enable the GPIO Power-On Reset (POR) source.
 * @param[in] port          GPIO port
 * @param[in] pin           GPIO pin
 * @param[in] polarity      GPIO por pin polarity. 0 = Active low, 1 = Active high.
 * @param[in] por_time      Time for the Power-On Reset to happen. The time is calculated
 *                          based on the following equation:
 *                          Time = por_time x 4096 x RC32 clock period
 ****************************************************************************************
 */
void GPIO_EnablePorPin(GPIO_PORT port, GPIO_PIN pin, GPIO_POR_PIN_POLARITY polarity, uint8_t por_time);

/**
 ****************************************************************************************
 * @brief Disable the GPIO Power-On Reset (POR) source.
 ****************************************************************************************
 */
void GPIO_DisablePorPin(void);

/**
 ****************************************************************************************
 * @brief Set the Power-On Reset (POR) time.
 * @param[in] por_time      Time for the Power-On Reset to happen. The time is calculated
 *                          based on the following equation:
 *                          Time = por_time x 4096 x RC32 clock period
 ****************************************************************************************
 */
void GPIO_SetPorTime(uint8_t por_time);

/**
 ****************************************************************************************
 * @brief Get the Power-On Reset (POR) time.
 * @return The por_time value.
 *         The time for the POR to happen is: Time = por_time x 4096 x RC32 clock period.
 ****************************************************************************************
 */
uint8_t GPIO_GetPorTime(void);

#if defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief Disable hardware reset functionality on P00.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void GPIO_Disable_HW_Reset(void)
{
    SetWord16(HWR_CTRL_REG, 1);
}

/**
 ****************************************************************************************
 * @brief Enable hardware reset functionality on P00.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void GPIO_Enable_HW_Reset(void)
{
    SetWord16(HWR_CTRL_REG, 0);
}
#endif

/**
 ****************************************************************************************
 * @brief GPIO IRQn Handler
 * @param[in] irq   The IRQ that this handler services
 ****************************************************************************************
 */
void GPIOn_Handler(IRQn_Type irq);

/**
 ****************************************************************************************
 * @brief GPIO IRQ0 Handler
 ****************************************************************************************
 */
#if !defined (__EXCLUDE_GPIO0_HANDLER__)
void GPIO0_Handler(void);
#endif //(__EXCLUDE_GPIO0_HANDLER__)

/**
 ****************************************************************************************
 * @brief GPIO IRQ1 Handler
 ****************************************************************************************
 */
#if !defined (__EXCLUDE_GPIO1_HANDLER__)
void GPIO1_Handler(void);
#endif //(__EXCLUDE_GPIO1_HANDLER__)

/**
 ****************************************************************************************
 * @brief GPIO IRQ2 Handler
 ****************************************************************************************
 */
#if !defined (__EXCLUDE_GPIO2_HANDLER__)
void GPIO2_Handler(void);
#endif //(__EXCLUDE_GPIO2_HANDLER__)

/**
 ****************************************************************************************
 * @brief GPIO IRQ3 Handler
 ****************************************************************************************
 */
#if !defined (__EXCLUDE_GPIO3_HANDLER__)
void GPIO3_Handler(void);
#endif //(__EXCLUDE_GPIO3_HANDLER__)

/**
 ****************************************************************************************
 * @brief GPIO IRQ4 Handler
 ****************************************************************************************
 */
#if !defined (__EXCLUDE_GPIO4_HANDLER__)
void GPIO4_Handler(void);
#endif //(__EXCLUDE_GPIO4_HANDLER__)

#endif // _GPIO_H_

///@}
///@}
