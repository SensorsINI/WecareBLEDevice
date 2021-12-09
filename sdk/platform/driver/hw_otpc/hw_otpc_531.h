/**
 ****************************************************************************************
 * @addtogroup Drivers
 * @{
 * @addtogroup OTP
 * @brief OTP Controller driver API
 * @{
 *
 * @file hw_otpc_531.h
 *
 * @brief DA14531 OTP Controller driver header file.
 *
 * Copyright (C) 2018-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#if defined (__DA14531__)

#ifndef _HW_OTPC_531_H_
#define _HW_OTPC_531_H_

#include <stdbool.h>
#include "datasheet.h"
#include "arch.h"
#include "ll.h"

/**
 * @brief OTP Controller mode
 */
typedef enum {
    HW_OTPC_MODE_DSTBY      = 0U,    /**< OTP cell is powered on, LDO is inactive*/
    HW_OTPC_MODE_STBY       = 1U,    /**< OTP cell and LDO are powered on, chip select is deactivated*/
    HW_OTPC_MODE_READ       = 2U,    /**< OTP cell can be read*/
    HW_OTPC_MODE_PROG       = 3U,    /**< OTP cell can be programmed*/
    HW_OTPC_MODE_PVFY       = 4U,    /**< OTP cell can be read in PVFY margin read mode*/
    HW_OTPC_MODE_RINI       = 5U,    /**< OTP cell can be read in RINI margin read mode*/
    HW_OTPC_MODE_AREAD      = 6U     /**< OTP cell can be read by the internal DMA*/
} hw_otpc_mode_t;

/// OTP cell size in bytes
#define HW_OTP_CELL_SIZE        (0x04)
/// Maximum number of OTP cells
#define HW_OTP_CELL_NUM         (0x8000 / HW_OTP_CELL_SIZE)

/**
 * @brief Word inside cell to program/read
 *
 * Cell contents in memory starts with low word (i.e. to program/read both words in cell at once,
 * HW_OTPC_WORD_LOW should be used for addressing).
 */
typedef enum {
    HW_OTPC_WORD_LOW  = 0U,
    HW_OTPC_WORD_HIGH = 1U
} hw_otpc_word_t;

/**
 * @brief System clock frequency in MHz
 */
typedef enum {
    HW_OTPC_CLK_FREQ_2MHz  = 0U,
    HW_OTPC_CLK_FREQ_4MHz  = 1U,
    HW_OTPC_CLK_FREQ_8MHz  = 2U,
    HW_OTPC_CLK_FREQ_16MHz = 3U,
} hw_otpc_clk_freq_t;

/**
 * @brief Check if DCDC converter is reserved by OTP.
 * @return true if DCDC converter is reserved by OTP, else false.
 */
__WEAK bool hw_otpc_is_dcdc_reserved(void);

/**
 * @brief Clear reservation status of DCDC converter.
 */
__WEAK void hw_otpc_clear_dcdc_reserved(void);

/**
 * @brief Check OTPC_STAT_MRDY bit.
 */
__STATIC_FORCEINLINE void hw_otpc_wait_mode_change(void)
{
    while (!GetBits32(OTPC_STAT_REG, OTPC_STAT_MRDY));
}

/**
 * @brief Check OTPC_STAT_PRDY bit.
 */
__STATIC_FORCEINLINE void hw_otpc_wait_while_busy_programming(void)
{
    while (!GetBits32(OTPC_STAT_REG, OTPC_STAT_PRDY));
}

/**
 * @brief Check OTPC_STAT_PBUF_EMPTY bit.
 */
__STATIC_FORCEINLINE void hw_otpc_wait_while_programming_buffer_is_full(void)
{
    while (!GetBits32(OTPC_STAT_REG, OTPC_STAT_PBUF_EMPTY));
}

/**
 * @brief Check if the OTP Controller is active.
 * @return 1 if it is active, else 0.
 */
__STATIC_FORCEINLINE bool hw_otpc_is_active(void)
{
    /*
     * Check if the OTPC clock is enabled
     */
    return GetBits16(CLK_AMBA_REG, OTP_ENABLE);
}

/**
 * @brief Close the OTP Controller.
 */
__STATIC_FORCEINLINE void hw_otpc_close(void)
{
    /*
     * Disable OTPC clock
     */
    GLOBAL_INT_DISABLE();
    SetBits16(CLK_AMBA_REG, OTP_ENABLE, 0x00);
    GLOBAL_INT_RESTORE();
}

/**
 * @brief Initialize the OTP Controller.
 */
void hw_otpc_init(void);

/**
 * @brief Moves the OTPC in new mode
 * @param[in] mode      The new mode
 */
void hw_otpc_enter_mode(hw_otpc_mode_t mode);

/**
 * @brief Program OTP with a word
 * @param[in] wdata        The data to be programmed
 * @param[in] cell_offset  The offset of cell to be writtent in 32 bit words
 */
__STATIC_FORCEINLINE void hw_otpc_word_prog(uint32_t wdata, uint32_t cell_offset)
{
    /* Check if we are in program mode, if not enter */
    hw_otpc_enter_mode(HW_OTPC_MODE_PROG);
    SetWord32(OTPC_PWORD_REG, wdata);
    SetWord32(OTPC_PADDR_REG, cell_offset);
    hw_otpc_wait_while_busy_programming();
}

/**
 * @brief Set the access speed of the OTP Controller based on the system clock.
 * @param[in] clk_speed     The frequency of the system clock: 16, 8, 4, 2 MHz
 * @warning The OTP clock must have been enabled (OTP_ENABLE == 1)
 */
void hw_otpc_set_speed(hw_otpc_clk_freq_t clk_speed);

/**
 * @brief Moves the OTPC in power down state
 */
void hw_otpc_disable(void);

/**
 * @brief Read a word from OTP
 * @param[in] cell_offset       The offset of cell to be read in 32 bit words
 * @return otp cell value
 */
uint32_t hw_otpc_word_read(uint32_t cell_offset);

/**
 * @brief Program specific bits in OTP
 * @param[in] wdata         The data to be programmed
 * @param[in] mask          The mask of the bit field
 * @param[in] pos           The position (offset) of the bitfield
 * @param[in] cell_offset   The offset of cell to be written in 32 bit words
 */
void hw_otpc_bits_prog(uint32_t wdata, const uint32_t mask, const uint32_t pos, uint32_t cell_offset);

/**
 * @brief Program specific bits in OTP with verification
 * @param[in] wdata         The data to be programmed
 * @param[in] mask          The mask of the bit field
 * @param[in] pos           The position (offset) of the bitfield
 * @param[in] cell_offset   The offset of cell to be written in 32 bit words
 * @return cell true if success or false in fail
 */
bool hw_otpc_bits_prog_and_verify(uint32_t wdata, const uint32_t mask, const uint32_t pos, uint32_t cell_offset);

/**
 * @brief Program OTP with a block of data.
 * @param[in] p_data        Pointer to the data to be programmed
 * @param[in] cell_offset   The offset of cell to be written in 32 bit words
 * @param[in] num_of_words  Number of words to be written
 */
void hw_otpc_prog(uint32_t *p_data, uint32_t cell_offset, uint32_t num_of_words);

/**
 * @brief Program OTP with a block of data with verify.
 * @param[in] p_data        Pointer to the data to be programmed
 * @param[in] cell_offset   The offset of cell to be written in 32 bit words
 * @param[in] num_of_words  Number of words to be written
 * @return cell true if success or false in fail
 * @warning The comparison is in a word by word basis while writing. On the first fail the function exits.
 */
bool hw_otpc_prog_and_verify(uint32_t *p_data, uint32_t cell_offset, uint32_t num_of_words);

/**
 * @brief Read an OTP block
 * @param[in] p_data        Pointer to the data to be read
 * @param[in] cell_offset   The offset of cell to be read in 32 bit words
 * @param[in] num_of_words  Number of words to be read
 */
void hw_otpc_read(uint32_t *p_data, uint32_t cell_offset, uint32_t num_of_words);

/**
 * @brief Get cell memory address
 * Returns mapped memory address for given cell
 * @param[in] cell_offset cell offset
 * @return cell memory address
 */
__STATIC_FORCEINLINE void *hw_otpc_cell_to_mem(uint32_t cell_offset)
{
    return (void *)(MEMORY_OTP_BASE + (cell_offset << 2)); // cell size is 4 bytes
}

/**
 * @brief Program OTP and verify.
 * @param[in] wdata         The data to be programmed
 * @param[in] cell_offset   The offset of cell to be written in 32 bit words
 * @return cell true if success or false in fail
 */
bool hw_otpc_word_prog_and_verify(uint32_t wdata, uint32_t cell_offset);

#endif /* _HW_OTPC_531_H_ */

#endif // __DA14531__

///@}
///@}
