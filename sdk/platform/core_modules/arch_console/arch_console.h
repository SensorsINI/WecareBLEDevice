/**
 ****************************************************************************************
 * @addtogroup Core_Modules
 * @{
 * @addtogroup Arch_console Serial Logging
 * @brief Serial logging API
 * @{
 *
 * @file arch_console.h
 *
 * @brief Serial logging interface header file.
 *
 * Copyright (C) 2013-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _ARCH_CONSOLE_H_
#define _ARCH_CONSOLE_H_

#if defined (CFG_PRINTF)

#include <stdarg.h>

/// Print message node
typedef struct __print_msg {
    /// Pointer to the next message
    struct __print_msg *pNext;
    /// Message
    char pBuf[__ARRAY_EMPTY];
} printf_msg;

#if defined (__DA14531__) && !defined (__EXCLUDE_ROM_ARCH_CONSOLE__)
/// ROM console configuration
typedef struct
{
    /// Message buffer
    char *current_msg_buffer;
    /// Message offset
    uint8_t *current_msg_offset;
    /// UART busy
    volatile bool *uart_busy;
    /// Message list
    printf_msg **printf_msg_list;
    /// Defer sending
    bool *defer_sending;
    /// UART write
    void (*uart_write)(uint8_t *bufptr, uint32_t size, void (*callback) (uint8_t));
    /// UART finish
    void (*uart_finish)();
    /// Arch restore sleep mode
    void (*arch_restore_sleep_mode)();
    /// Arch force active mode
    void (*arch_force_active_mode)();
} rom_console_cfg_t;
#endif


/**
 ****************************************************************************************
 * @brief Push the string s to the UART queue.
 * @param[in] s    String
 ****************************************************************************************
 */
void arch_puts(const char *s);

/**
 ****************************************************************************************
 * @brief Push formatted data from variable argument list to the UART queue.
 * @param[in] fmt    Formatted output
 * @param[in] args   Arguments
 * @return 1
 ****************************************************************************************
 */
int arch_vprintf(const char *fmt, va_list args);

/**
 ****************************************************************************************
 * @brief Place the formatted output into the UART queue.
 * @param[in] fmt    Formatted output
 * @return 1
 ****************************************************************************************
 */
int arch_printf(const char *fmt, ...);

/**
 ****************************************************************************************
 * @brief Flush printf.
 ****************************************************************************************
 */
void arch_printf_flush(void);

/**
 ****************************************************************************************
 * @brief This function is called periodically from the main loop to
 *        push the contents of the queue into the UART.
 ****************************************************************************************
 */
void arch_printf_process(void);

#else // CFG_PRINTF

#define arch_puts(s) {}
#define arch_vprintf(fmt, args) {}
#define arch_printf(fmt, args...) {}
#define arch_printf_process() {}
#define arch_printf_flush() {}

#endif // CFG_PRINTF

#endif // _ARCH_CONSOLE_H_

/// @}
/// @}
