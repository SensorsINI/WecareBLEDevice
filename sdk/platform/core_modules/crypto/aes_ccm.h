/**
 ****************************************************************************************
 * @addtogroup Core_Modules
 * @{
 * @addtogroup Crypto
 * @{
 * @addtogroup AES_CCM AES CCM
 * @brief Advanced Encryption Standard CCM API.
 * @{
 *
 * @file aes_ccm.h
 *
 * @brief AES-CCM Encryption/Decryption implementation header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef AES_CCM_H_
#define AES_CCM_H_

/*
 * DEFINES
 ****************************************************************************************
 */

/// Encrypt process
#define ENCRYPT_PROCESS 0
/// Decrypt process
#define DECRYPT_PROCESS 1

/// Block size
#define CCM_BLK_SIZE    (16)

/// Formatting block + adata + formatted adata_len + payload
#define OUT_BUFFER_SIZE(payload_len, CCM_T) (payload_len + CCM_T)

/// AES CCM T
enum {
    AES_CCM_T4  = 4,
    AES_CCM_T6  = 6,
    AES_CCM_T8  = 8,
    AES_CCM_T10 = 10,
    AES_CCM_T12 = 12,
    AES_CCM_T14 = 14,
    AES_CCM_T16 = 16,
};

/// AES CCM Q
enum {
    AES_CCM_Q2 = 2,
    AES_CCM_Q3 = 3,
    AES_CCM_Q4 = 4,
    AES_CCM_Q5 = 5,
    AES_CCM_Q6 = 6,
    AES_CCM_Q7 = 7,
    AES_CCM_Q8 = 8,
};

/// AES CCM N
enum {
    AES_CCM_N7  = 15 - AES_CCM_Q8,
    AES_CCM_N8  = 15 - AES_CCM_Q7,
    AES_CCM_N9  = 15 - AES_CCM_Q6,
    AES_CCM_N10 = 15 - AES_CCM_Q5,
    AES_CCM_N11 = 15 - AES_CCM_Q4,
    AES_CCM_N12 = 15 - AES_CCM_Q3,
    AES_CCM_N13 = 15 - AES_CCM_Q2,
};

/*
 * STRUCTURES
 ****************************************************************************************
 */


/// AES_CCM environment structure
struct aes_ccm_env_tag
{
    /// Nonce array
    const uint8_t *Nonce;

    /// Payload length
    uint16_t payload_len;
    /// Number of payload blocks
    uint8_t payload_blk_cnt;
    /// Payload input
    const uint8_t *payload_input;

    /// Adata block length
    uint16_t adata_len;
    /// Number of adata blocks
    uint8_t adata_blk_cnt;
    /// Adata array
    uint8_t (*adata_blk)[CCM_BLK_SIZE];

    /// CBC-MAC blocks
    uint8_t b_blk_cnt;
    /// B block array
    uint8_t (*b_blk)[CCM_BLK_SIZE];
    /// X block array
    uint8_t (*x_blk)[CCM_BLK_SIZE];

    /// CTR blocks
    uint32_t a_blk_cnt;
    /// A block array
    uint8_t (*a_blk)[CCM_BLK_SIZE];
    /// S block array
    uint8_t (*s_blk)[CCM_BLK_SIZE];
};

/*
 * PUBLIC FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief AES CCM encryption
 * @param[in] payload      Data to be encrypted/decrypted
 * @param[in] payload_len  payload length in bytes, up to 32
 * @param[in] Nonce        Nonce array, should be unique for each AES-CCM operation
 * @param[in] Adata        Adata, or header, up to 32 bytes
 * @param[in] Adata_len    Adata length in bytes, up to 32
 * @param[out] output      where encrypted cipher to be placed. Header is not included
 ****************************************************************************************
 */
void aes_ccm_encrypt(uint8_t *payload, uint16_t payload_len, uint8_t *Nonce,
                     uint8_t *Adata, uint16_t Adata_len, uint8_t *output);

/**
 ****************************************************************************************
 * @brief AES CCM decryption
 * @param[in] payload      Data to be encrypted/decrypted
 * @param[in] payload_len  payload length in bytes, up to 32
 * @param[in] Nonce        Nonce array, should be unique for each AES-CCM operation
 * @param[in] Adata        Adata, or header, up to 32 bytes
 * @param[in] Adata_len    Adata length in bytes, up to 32
 * @param[out] output      where encrypted cipher to be placed. Header is not included
 * @return    0 if auth data matches up. 1 if something goes wrong
 ****************************************************************************************
 */
uint8_t aes_ccm_decrypt(uint8_t *payload, uint16_t payload_len, uint8_t *Nonce,
                        uint8_t *Adata, uint16_t Adata_len, uint8_t *output);

/**
 ****************************************************************************************
 * @brief Set AES key and memory pool type
 * @param[in] key           key to be used, should be 16 bytes
 * @param[in] T             Tag length
 * @param[in] N             Nonce length
 * @param[in] ke_mem_type   memory pool type used for buffer allocation
 ****************************************************************************************
 */
void aes_ccm_init(uint8_t *key, uint8_t T, uint8_t N, uint8_t ke_mem_type);

/**
 ****************************************************************************************
 * @brief Deinitialize AES-CCM data block
 ****************************************************************************************
 */
void aes_ccm_cleanup(void);

#endif // AES_CCM_H_

/// @}
/// @}
/// @}
