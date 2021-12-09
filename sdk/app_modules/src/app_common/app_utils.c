/**
 ****************************************************************************************
 *
 * @file app_utils.c
 *
 * @brief Application generic utilities helper functions source file.
 *
 * Copyright (C) 2017-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_utils.h"
#include "co_bt.h"
#include "co_math.h"
#include "gap.h"

#if !defined (__DA14531__) || defined (__EXCLUDE_ROM_APP_UTILS__)

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

enum app_addr_types app_get_address_type(uint8_t addr_type, struct bd_addr addr)
{
    if (addr_type == ADDR_PUBLIC)
    {
        return APP_PUBLIC_ADDR_TYPE;
    }
    if ((addr_type == ADDR_RAND) && ((addr.addr[BD_ADDR_LEN-1] & 0xC0) == GAP_STATIC_ADDR))
    {
        return APP_RANDOM_STATIC_ADDR_TYPE;
    }
    if ((addr_type == ADDR_RAND) && ((addr.addr[BD_ADDR_LEN-1] & 0xC0) == GAP_RSLV_ADDR))
    {
        return APP_RANDOM_PRIVATE_RESOLV_ADDR_TYPE;
    }
    if ((addr_type == ADDR_RAND) && ((addr.addr[BD_ADDR_LEN-1] & 0xC0) == GAP_NON_RSLV_ADDR))
    {
        return APP_RANDOM_PRIVATE_NONRESOLV_ADDR_TYPE;
    }
    if ((addr_type == ADDR_RPA_PUBLIC) || (addr_type == ADDR_RPA_RAND))
    {
        return APP_ID_ADDR_TYPE;
    }
    return APP_ERROR_ADDR_TYPE;
}

void app_fill_random_byte_array(uint8_t *dst, uint8_t key_size, uint8_t array_size)
{
    uint32_t rand_val;
    uint8_t rand_bytes_cnt = 0;
    uint8_t remaining_bytes;
    uint8_t i;

    // key_size must not be greater than array_size
    ASSERT_ERROR(array_size >= key_size);

    // key_size = M*4+N
    // Randomize the M most significant bytes of the array
    for (i = 0; i < key_size-3; i+=4)
    {
        // Calculate a random 32 bit value
        rand_val = co_rand_word();
        // Assign each of the 4 rand bytes to the byte array
        dst[array_size - (i+0)-1] = (rand_val >> 24) & 0xFF;
        dst[array_size - (i+1)-1] = (rand_val >> 16) & 0xFF;
        dst[array_size - (i+2)-1] = (rand_val >> 8) & 0xFF;
        dst[array_size - (i+3)-1] = (rand_val >> 0) & 0xFF;
        // Count randomized bytes
        rand_bytes_cnt += 4;
    }

    // Randomize the remaining N most significant bytes of the array. (Max N = 3)
    remaining_bytes = key_size - rand_bytes_cnt;
    if (remaining_bytes)
    {
        rand_val = co_rand_word();
        for (i = 0; i < remaining_bytes; i++)
        {
            dst[array_size -(rand_bytes_cnt+i)-1] = (rand_val >> ((3-i)<<3)) & 0xFF;
        }
    }

    // Fill the least significant bytes of the array with zeroes
    remaining_bytes = array_size - key_size;
    for (i = 0; i < remaining_bytes; i++)
    {
        dst[array_size - (key_size + i)-1] = 0;
    }
}

#else // use SDK functions which are built in ROM (only for DA14531)

extern enum app_addr_types app_get_address_type_ROM(uint8_t addr_type, struct bd_addr addr);

extern void app_fill_random_byte_array_ROM(uint8_t *dst, uint8_t key_size, uint8_t array_size);

enum app_addr_types app_get_address_type(uint8_t addr_type, struct bd_addr addr)
{
    typedef enum app_addr_types (* app_get_address_type_t)(uint8_t addr_type, struct bd_addr addr);
    
    if ((addr_type == ADDR_RPA_PUBLIC) || (addr_type == ADDR_RPA_RAND))
    {
        return APP_ID_ADDR_TYPE;
    }
    else
    {
        app_get_address_type_t f = (app_get_address_type_t) (app_get_address_type_ROM);
        return f(addr_type, addr);
    }
}

void app_fill_random_byte_array(uint8_t *dst, uint8_t key_size, uint8_t array_size)
{
    typedef void (* app_fill_random_byte_array_t)(uint8_t *dst, uint8_t key_size, uint8_t array_size);
    
    // key_size must not be greater than array_size
    if (array_size >= key_size)
    {
        app_fill_random_byte_array_t f = (app_fill_random_byte_array_t) (app_fill_random_byte_array_ROM);
        f(dst, key_size, array_size);
    }
}

#endif

/// @} APP
