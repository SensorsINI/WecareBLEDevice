/**
 ****************************************************************************************
 *
 * @file decrypt.c
 *
 * @brief Decrypt API for secondary booloader
 *
 * Copyright (C) 2017-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include "sw_aes.h"
#include "uart_booter.h"
#if !defined (__DA14531__)
#include "datasheet.h"
#endif

AES_CTX ctx;
const uint8_t Key[16]= {0x06,0xa9,0x21,0x40,0x36,0xb8,0xa1,0x5b,0x51,0x2e,0x03,0xd5,0x34,0x12,0x00,0x06};
const uint8_t IV[16] = {0x3d,0xaf,0xba,0x42,0x9d,0x9e,0xb4,0x30,0xb4,0x22,0xda,0x80,0x2c,0x9f,0xac,0x41};

/**
 ****************************************************************************************
 * @brief Decrypts the encrypted image in place.
 * @param[in] nsize the size of the encrypted image which is expected to be a multiple
 *                  of AES_BLOCKSIZE.
 ****************************************************************************************
 */
void Decrypt_Image(int nsize)
{
    AES_set_key(&ctx,Key,IV,AES_MODE_128);
    AES_convert_key(&ctx);
#if defined (__DA14531__)
    AES_cbc_decrypt(&ctx, (const uint8_t *)SYSRAM_BASE_ADDRESS, (uint8_t *)SYSRAM_BASE_ADDRESS, nsize);
#else
    const int DECRYPT_CHUNK = 32 * AES_BLOCKSIZE;
    uint8_t *sys_ram = (uint8_t *)SYSRAM_BASE_ADDRESS;

    for (int i = nsize; i >= 0; i -= DECRYPT_CHUNK)
    {
        AES_cbc_decrypt(&ctx, (const uint8_t *)sys_ram, sys_ram, DECRYPT_CHUNK);
        SetWord16(WATCHDOG_REG, WATCHDOG_REG_RESET);
        sys_ram += DECRYPT_CHUNK;
    }
#endif
}
