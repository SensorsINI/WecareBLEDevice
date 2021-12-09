/**
 ****************************************************************************************
 *
 * @file arch_rom.c
 *
 * @brief ROM initialization functions.
 *
 * Copyright (C) 2016-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDES
 ****************************************************************************************
 */

#include <string.h>
#include "llm.h"
#include "gap_cfg.h"
#include "da1458x_scatter_config.h"
#include "_reg_ble_em_rx_buffer.h"
#include "_reg_ble_em_tx_buffer.h"
#include "em_map_ble.h"
#include "em_map.h"
#include "arch_rom.h"
#include "system_library.h"
#include "arch_ram.h"

#if defined (__DA14531__) && (USE_TRNG)
#include "trng.h"
#include "boot.h"
#endif

#ifdef CFG_DBG_TASK
#include "hci_vs.h"
#endif

/**
 * @addtogroup DRIVERS
 * @{
 */

/*
 * DEFINES
 ****************************************************************************************
 */

#if defined (__DA14531__)
#define MY_BLE_FEATURES_BYTE0   BLE_FEATURES_BYTE0

#if (USE_TRNG)
#define RAM_UNINIT_START_ADDRESS    (0x07FCB000)
#define RAM_UNINIT_END_ADDRESS      (0x07FCBA00)
#endif

#endif

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

uint16_t heap_log __SECTION_ZERO("retention_mem_area0");

#if defined (__DA14531__)
#include "llc.h"
#include "lld.h"

int PATCHED_llc_data_ind_handler(ke_msg_id_t const msgid,
                                 struct llc_data_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

int PATCHED_llcp_length_req_handler(ke_msg_id_t const msgid,
                                    struct llcp_length_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

int PATCHED_llcp_length_rsp_handler(ke_msg_id_t const msgid,
                                    struct llcp_length_rsp const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

int PATCHED_lld_data_ind_handler(ke_msg_id_t const msgid,
                                 struct lld_data_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

const struct custom_msg_handler my_custom_msg_handlers[]=
{
    { TASK_LLC, LLC_DATA_IND, (ke_msg_func_t)PATCHED_llc_data_ind_handler},
    { TASK_LLC, LLCP_LENGTH_REQ, (ke_msg_func_t)PATCHED_llcp_length_req_handler},
    { TASK_LLC, LLCP_LENGTH_RSP, (ke_msg_func_t)PATCHED_llcp_length_rsp_handler},
    { TASK_LLC, LLD_DATA_IND, (ke_msg_func_t)PATCHED_lld_data_ind_handler},
    { 0, 0, NULL}   //it must end with func = NULL so that loop can terminate correctly
};

extern struct custom_msg_handler *custom_msg_handlers;

// _rand_state stdlib/rand.c RW variable provided by ROM project
extern uint32_t _rand_state_ROM_DATA;

#if (USE_TRNG)
extern uint32_t trng_state_val;
#endif

#endif // defined (__DA14531__)

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void arch_rom_init(void)
{
#if defined (__DA14531__) && (!USE_CHACHA20_RAND)
    // Store value of _rand_state stdlib/rand.c RW variable
    uint32_t rand_temp = _rand_state_ROM_DATA;
#endif

#if defined (__DA14531__) && (USE_TRNG)
    // Given that the DA14531 secondary bootloader and flash programmer
    // (UART version) leaves untouched the memory area between
    // RAM_UNINIT_START_ADDRESS and RAM_UNINIT_END_ADDRESS address,
    // this memory area can be used to feed the TRNG buffer with
    // true random data (random values after RAM block power up).
    // The maximum size of the TRNG buffer is the difference between the
    // RAM_UNINIT_END_ADDRESS and RAM_UNINIT_START_ADDRESS.
    //
    // Note: The maximum TRNG buffer size should be recalculated
    //       if the application uses the Free Area region (check the
    //       scatter file). In any case, if the used TRNG buffer size is
    //       small enough, the TRNG algorithm will operate with true random
    //       data independently to the maximum size used in the following
    //       memory copy. For further details check the SDK6 documentation.
    if (trng_state_val != 0x12348765)
    {
        memcpy((uint8_t *) DB_HEAP_BASE,
               (uint8_t *) RAM_UNINIT_START_ADDRESS,
               RAM_UNINIT_END_ADDRESS - RAM_UNINIT_START_ADDRESS);
    }
#endif

    // Fill BLE EM + ROM data area in last RAM block, excluding Free Area
    //  region, (RAM4 for DA14585/586 - RAM3 for DA14531) with zeros.
    uint32_t *temp_ptr = (uint32_t *) __SCT_BLE_BASE;
    memset(temp_ptr, 0, CO_ALIGN4_HI(__SCT_EM_BLE_END));

    temp_ptr = (uint32_t *) ROM_DATA_BASE_ADDR;
    memset(temp_ptr, 0 , RAM_END_ADDR - ROM_DATA_BASE_ADDR);

#if defined (__DA14531__) && (!USE_CHACHA20_RAND)
    // Restore _rand_state stdlib/rand.c RW variable
    _rand_state_ROM_DATA = rand_temp;
#endif

    // Initialize variables used by ROM patches
    patch_global_vars_init();

#if defined (__DA14531__)
    custom_msg_handlers = (struct custom_msg_handler *) my_custom_msg_handlers;
#endif

    // Enable heap log
    heap_log = LOG_HEAP_USAGE;

    // Mandatory HCI root table setup
    memcpy(hci_cmd_desc_root_tab, rom_hci_cmd_desc_root_tab, 48);

#if defined (__DA14531__)
    struct le_features my_llm_local_le_feats = {{ MY_BLE_FEATURES_BYTE0, 0, 0, 0, 0, 0, 0, 0 }};
    memcpy(&llm_local_le_feats, &my_llm_local_le_feats, FEATS_LEN);

    struct supp_cmds my_llm_local_cmds = {{BLE_CMDS_BYTE0, 0, BLE_CMDS_BYTE2, 0, 0, BLE_CMDS_BYTE5, 0, 0,
                                           0, 0, BLE_CMDS_BYTE10, 0, 0, 0, BLE_CMDS_BYTE14, BLE_CMDS_BYTE15,
                                           0, 0 ,0 ,0 ,0, 0, BLE_CMDS_BYTE22, 0,
                                           0, BLE_CMDS_BYTE25, BLE_CMDS_BYTE26, BLE_CMDS_BYTE27, BLE_CMDS_BYTE28, 0, 0, 0,
                                           BLE_CMDS_BYTE32, BLE_CMDS_BYTE33, BLE_CMDS_BYTE34, BLE_CMDS_BYTE35, 0, 0, 0, BLE_CMDS_BYTE39,
                                           0, BLE_CMDS_BYTE41, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0, 0, 0}};

    memcpy(&llm_local_cmds, &my_llm_local_cmds, SUPP_CMDS_LEN);
#endif

#ifdef CFG_DBG_TASK
    // Vendor specific HCI commands initialization
    hci_vs_initialize_commands_tab();
#endif

#ifdef AUTO_DATA_LENGTH_NEGOTIATION_UPON_NEW_CONNECTION
    length_exchange_needed = true;
#endif

    use_h4tl = CFG_USE_H4TL;

    gap_cfg_user = (struct gap_cfg_user_struct *)&gap_cfg_user_var_struct;

    rom_func_addr_table = (uint32_t*)&rom_func_addr_table_var[0];
    rom_cfg_table = &rom_cfg_table_var[0];

    ble_duplicate_filter_max = CFG_BLE_DUPLICATE_FILTER_MAX;
#ifdef CFG_BLE_DUPLICATE_FILTER_FOUND
    ble_duplicate_filter_found = true;
#else
    ble_duplicate_filter_found = false;
#endif
    llm_resolving_list_max = CFG_LLM_RESOLVING_LIST_MAX;

    BLE_TX_DESC_DATA_USER           = __SCT_BLE_TX_DESC_DATA;
    BLE_TX_DESC_CNTL_USER           = __SCT_BLE_TX_DESC_CNTL;

    LLM_LE_ADV_DUMMY_IDX            = __SCT_BLE_TX_DESC_DATA          + __SCT_BLE_TX_DESC_CNTL - 1;
    LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX = LLM_LE_ADV_DUMMY_IDX            + 1;
    LLM_LE_SCAN_RSP_IDX             = LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX + 1;
    LLM_LE_ADV_IDX                  = LLM_LE_SCAN_RSP_IDX             + 1;

    REG_BLE_EM_RX_BUFFER_SIZE       = __SCT_REG_BLE_EM_RX_BUFFER_SIZE;
    REG_BLE_EM_TX_BUFFER_SIZE       = __SCT_REG_BLE_EM_TX_BUFFER_SIZE;

    ASSERT_ERROR(
        (BLE_TX_DESC_DATA == __SCT_BLE_TX_DESC_DATA) &&
        (BLE_TX_DESC_CNTL == __SCT_BLE_TX_DESC_CNTL) &&
        // offsets
        (_EM_BLE_RX_BUFFER_OFFSET == __SCT__EM_BLE_RX_BUFFER_OFFSET) &&
        (EM_BLE_RX_BUFFER_OFFSET  == __SCT_EM_BLE_RX_BUFFER_OFFSET ) &&
        (_EM_BLE_CNXADD_OFFSET    == __SCT__EM_BLE_CNXADD_OFFSET   ) &&
        (EM_BLE_CNXADD_OFFSET     == __SCT_EM_BLE_CNXADD_OFFSET    ) &&
        (_EM_BLE_END              == __SCT__EM_BLE_END             ) &&
        (EM_BLE_END               == __SCT_EM_BLE_END              )
    );

    _ble_base = __SCT_BLE_BASE; // Initialize BLE EM base
}

/// @} DRIVERS
