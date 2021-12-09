/**
 ****************************************************************************************
 * @addtogroup Core_Modules
 * @{
 * @addtogroup Crypto
 * @{
 * @addtogroup AES_TASK AES Task
 * @brief Task management for the Advanced Encryption Standard module
 *
 * The aes task is responsible for all security related functions not related to a
 * specific connection with a peer.
 * @{
 *
 * @file aes_task.h
 *
 * @brief AES task implementation header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef AES_TASK_H_
#define AES_TASK_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "rwip_config.h"
#include "gapm_task.h"          // GAP Manager Task Definitions
#include "gap.h"                // GAP Definitions

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum AES instance number
#define AES_IDX_MAX                    (0x01)

/**
 * Operation Codes (Mapped on GAPM Operation Codes)
 */

/// Use Encryption Block
#define AES_OP_USE_ENC_BLOCK           (GAPM_USE_ENC_BLOCK)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// States of AES Task
enum aes_state_id
{
    /// IDLE state
    AES_IDLE,
    /// BUSY state: Communication with LLM ongoing state
    AES_BUSY,

    /// Number of defined states.
    AES_STATE_MAX
};


/// AES block API message IDs
enum aes_msg_id
{
    /// Encryption Toolbox Access
    AES_USE_ENC_BLOCK_CMD          = KE_FIRST_MSG(TASK_AES),

    /// Command Complete Event
    AES_CMP_EVT
};

/*
 * STRUCTURES
 ****************************************************************************************
 */

/// AES Encrypt complete event structure
struct aes_cmp_evt
{
    /// Status of the command reception
    uint8_t status;
    ///Encrypted data to return to command source.
    uint8_t encrypted_data[16];
};

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

/// State handlers table
extern const struct ke_state_handler aes_state_handler[AES_STATE_MAX];

/// Default state handler
extern const struct ke_state_handler aes_default_handler;

/// States of all the task instances
extern ke_state_t aes_state[AES_IDX_MAX];

#endif // (AES_TASK_H_)

/// @}
/// @}
/// @}
