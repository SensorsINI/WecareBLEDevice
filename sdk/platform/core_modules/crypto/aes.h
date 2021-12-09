/**
 ****************************************************************************************
 * @addtogroup Core_Modules
 * @{
 * @addtogroup Crypto
 * @{
 * @addtogroup AES_INIT AES Initialization
 * @brief Advanced Encryption Standard initialization API.
 * @{
 *
 * @file aes.h
 *
 * @brief AES header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef AES_H_
#define AES_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "co_bt.h"
#include "ke_msg.h"
#include "sw_aes.h"

/*
 * STRUCTURES
 ****************************************************************************************
 */

/// AES key
typedef AES_CTX AES_KEY;

/// AES environment struct
struct aes_env_tag
{
    /// Request operation Kernel message
    void *operation;
    
    /// Operation requester task id
    ke_task_id_t requester;

    /// Encryption/Decryption operation mode selection
    uint8_t enc_dec;
    
    /// Key length
    uint32_t key_len;
    
    /// Key
    uint8_t *key;
    
    /// AES key
    AES_KEY aes_key;
    
    /// Plaintext
    uint8_t *in;
    
    /// Plaintext length
    uint32_t in_len;
    
    /// Plaintext index
    uint32_t in_cur;
    
    /// Ciphered text
    uint8_t *out;
    
    /// Ciphered text length
    uint32_t out_len;
    
    /// Ciphered text index
    uint32_t out_cur;

    /// Callback function to be called upon encryption/decryption completion
    void (*aes_done_cb)(uint8_t status);

    /// Synchronous/asynchronous mode selection
    uint8_t ble_flags;
 };

/*
 * GLOBAL VARIABLES DECLARATION
 ****************************************************************************************
 */

/// AES environment
extern struct aes_env_tag aes_env;

/// AES output
extern uint8_t aes_out[ENC_DATA_LEN];

/*
 * PUBLIC FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the AES task.
 * @param[in] reset       FALSE will create the task, TRUE will just reset the environment.
 * @param[in] aes_done_cb Callback function to be called upon encryption/decryption completion
 ****************************************************************************************
 */
void aes_init(bool reset, void (*aes_done_cb)(uint8_t status));

/**
 ****************************************************************************************
 * @brief AES encrypt/decrypt operation.
 * @param[in] key           The key data.
 * @param[in] key_len       The key data length in bytes (should be 16).
 * @param[in] in            The input data block.
 * @param[in] in_len        The input data block length.
 * @param[in] out           The output data block.
 * @param[in] out_len       The output data block length.
 * @param[in] enc_dec       AES_ENCRYPT for encryption, AES_DECRYPT for decryption.
 * @param[in] aes_done_cb   The callback to be called at the end of each operation.
 * @param[in] ble_flags     Used to specify whether the encryption/decryption
 *                          will be performed synchronously or asynchronously (message based)
 *                          also if ble_safe is specified in ble_flags rwip_schedule() will be called
 *                          to avoid missing any BLE events.
 * @return                   0 if successful,
 *                          -1 if userKey or key are NULL,
 *                          -2 if AES task is busy,
 *                          -3 if enc_dec not 0/1,
 *                          -4 if key_len not 16.
 ****************************************************************************************
 */
int aes_operation(uint8_t *key,
                  uint8_t key_len,
                  uint8_t *in,
                  uint32_t in_len,
                  uint8_t *out,
                  uint32_t out_len,
                  uint8_t enc_dec,
                  void (*aes_done_cb)(uint8_t status),
                  uint8_t ble_flags);

/**
 ****************************************************************************************
 * @brief Send a aes_CMP_EVT message to the task which sent the request.
 *
 * @param[in] cmd_src_id            Command source ID
 * @param[in] operation             Command operation code
 * @param[in] status                Status of the request
 ****************************************************************************************
 */
void aes_send_cmp_evt(ke_task_id_t cmd_src_id, uint8_t operation, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send an encryption request to the LLM.
 * @param[in] operand_1             Operand 1
 * @param[in] operand_2             Operand 2
 ****************************************************************************************
 */
void aes_send_encrypt_req(uint8_t *operand_1, uint8_t *operand_2);

#endif // (AES_H_)

/// @}
/// @}
/// @}
