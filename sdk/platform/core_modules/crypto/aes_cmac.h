/**
 ****************************************************************************************
 * @addtogroup Core_Modules
 * @{
 * @addtogroup Crypto
 * @{
 * @addtogroup AES_CMAC AES CMAC
 * @brief Advanced Encryption Standard CMAC API.
 * @{
 *
 * @file aes_cmac.h
 *
 * @brief AES-CMAC header file.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef AES_CMAC_H_
#define AES_CMAC_H_

/*
 * DEFINES
 ****************************************************************************************
 */

/// Input block size (128 bits)
#define AES_CMAC_BLK_SIZE_128 (16)

/*
 * PUBLIC FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Generate tag using AES-CMAC
 * @param[in] payload           Input payload
 * @param[in] payload_len       Payload length (in bytes)
 * @param[in] key               Key (128bit)
 * @param[out] mac              MAC output buffer
 * @param[in] mac_len           MAC buffer length (in bytes)
 * @return                      Error code if something goes wrong, 0 otherwise
 ****************************************************************************************
 */
uint8_t aes_cmac_generate(const uint8_t *payload, uint16_t payload_len,
                          const uint8_t *key, uint8_t *mac, uint8_t mac_len);

/**
 ****************************************************************************************
 * @brief Verify tag of the message using AES-CMAC
 * @param[in] payload           Input payload
 * @param[in] payload_len       Payload length (in bytes)
 * @param[in] key               Key (128bit)
 * @param[in] mac               MAC to verify
 * @param[in] mac_len           MAC buffer length (in bytes)
 * @return                      True if provided mac was successfully verified, False otherwise
 ****************************************************************************************
 */
bool aes_cmac_verify(const uint8_t *payload, uint16_t payload_len,
                     const uint8_t *key, const uint8_t *mac, uint8_t mac_len);

#endif // AES_CMAC_H_

/// @}
/// @}
/// @}
