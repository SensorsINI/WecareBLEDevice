/**
 ****************************************************************************************
 * @addtogroup Third-party Third-party APIs
 * @{
 * @addtogroup Rand
 * @brief ChaCha20 API
 * @{
 *
 * @file chacha20.h
 *
 * @brief ChaCha20 random generator API.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _CHACHA20_H_
#define _CHACHA20_H_

#if (USE_CHACHA20_RAND)

#if defined (__DA14531__) && !defined (__EXCLUDE_ROM_CHACHA20__)
/// ChaCha20 configuration
typedef struct
{
    /// Key
    uint32_t *key;

    /// Counter
    uint64_t *counter;

    /// Random output
    uint32_t *random_output;

    /// Random output left
    uint8_t *random_output_left;
} rom_chacha20_cfg_t;
#endif

/**
 ****************************************************************************************
 * @brief Initialize ChaCha20 seed
 * @param[in] key initialization value
 ****************************************************************************************
 */
void csprng_seed(const uint8_t key[16]);

/**
 ****************************************************************************************
 * @brief Get ChaCha20 random 32-bit number
 * @return Random number
 ****************************************************************************************
 */
uint32_t csprng_get_next_uint32(void);

#endif
#endif

///@}
///@}
