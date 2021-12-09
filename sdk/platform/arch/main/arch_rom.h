/**
 ****************************************************************************************
 * @addtogroup ARCH
 * @{
 * @addtogroup ARCH_ROM ROM Utilities
 * @brief ROM utilities API
 * @{
 *
 * @file arch_rom.h
 *
 * @brief ROM initialization functions and declarations.
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _ARCH_ROM_H_
#define _ARCH_ROM_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>
#include "co_hci.h"
#include "gap_cfg.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum packet header size (considering unpacked structure sizes)
#define MAX_HEADER_SIZE       HCI_ACL_HDR_LEN

///H4TL RX states
enum H4TL_STATE_RX
{
    ///H4TL RX Start State - receive message type
    H4TL_STATE_RX_START,
    ///H4TL RX Header State - receive message header
    H4TL_STATE_RX_HDR,
    ///H4TL RX Header State - receive (rest of) message payload
    H4TL_STATE_RX_PAYL,
    ///H4TL RX Out Of Sync state - receive message type
    H4TL_STATE_RX_OUT_OF_SYNC
};

///H4TL TX states
enum H4TL
{
     ///H4TL TX Start State - when packet is ready to be sent
     H4TL_STATE_TX_ONGOING,
     ///H4TL TX Done State - TX ended with no error
     H4TL_STATE_TX_IDLE,
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern uint8_t hci_cmd_desc_root_tab[];                          // ROM variable
extern uint8_t rom_hci_cmd_desc_root_tab[];                      // ROM variable (init data for hci_cmd_desc_root_tab)
extern char use_h4tl;                                            // ROM variable
extern const struct gap_cfg_user_struct gap_cfg_user_var_struct; // forward declaration
extern uint32_t *rom_func_addr_table;                            // ROM variable
extern uint32_t *rom_cfg_table;                                  // ROM variable
extern void * const rom_func_addr_table_var[];                   // defined in jump_table.c
extern uint32_t rom_cfg_table_var[];                             // defined in jump_table.c
extern bool length_exchange_needed;                              // ROM variable

extern struct h4tl_env_tag h4tl_env;

/// H4TL out of synchronization recovery variables
struct h4tl_out_of_sync_tag
{
    uint8_t byte;
    uint8_t index;
};

/// H4TL Environment context structure
struct h4tl_env_tag
{
    /// pointer to External interface api
    const struct rwip_eif_api* ext_if;

    ///Latest received message header, 4 byte buffer for eother configuration.
    uint8_t curr_hdr_buff[MAX_HEADER_SIZE];
    ///Pointer to space reserved for received payload.
    uint8_t *curr_payl_buff;

    /// HCI synchronization error parameters
    struct h4tl_out_of_sync_tag out_of_sync;

    ///TX callback for indicating the end of transfer
    void (*tx_callback)(void);
    ///Rx state - can be receiving message type, header, payload or error
    uint8_t  rx_state;
    ///Tx state - either transmitting, done or error.
    uint8_t  tx_state;
    ///Latest received message type: CMD/EVT/ACL.
    uint8_t  rx_type;
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize ROM variables.
 ****************************************************************************************
 */
void arch_rom_init(void);

#endif // _ARCH_ROM_H_

/// @}
/// @}
