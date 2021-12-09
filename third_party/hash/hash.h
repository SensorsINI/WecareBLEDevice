/**
 ****************************************************************************************
 * @addtogroup Third-party Third-party APIs
 * @{
 * @addtogroup Hash
 * @brief Hashing API
 * @{
 *
 * @file hash.h
 *
 * @brief Hashing functions header file.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _HASH_H_
#define _HASH_H_

/**
 ****************************************************************************************
 * @brief Integer hash function. Accepts an integer hash key and returns an integer hash
 *        result with uniform distribution.
 * @param[in] a     integer hash key.
 * @return          integer hash result
 ****************************************************************************************
 */
uint32_t hash(uint32_t a);

#endif // _HASH_H_

///@}
///@}
