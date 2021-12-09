/**
 ****************************************************************************************
 *
 * @file user_noncon.c
 *
 * @brief Non-connectable advertising project source code.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
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

#include "rwip_config.h"
#include "gap.h"
#include "app_easy_timer.h"
#include "user_noncon.h"
#include "co_bt.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

timer_hnd app_adv_data_update_timer_used            __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

uint8_t initial_adv_data[ADV_DATA_LEN]              __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t initial_adv_data_len                        __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

uint8_t user_adv_data[ADV_DATA_LEN]                 __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t user_adv_data_len                           __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

uint8_t initial_scan_rsp_data[SCAN_RSP_DATA_LEN]    __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t initial_scan_rsp_data_len                   __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

uint8_t user_scan_rsp_data[SCAN_RSP_DATA_LEN]       __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t user_scan_rsp_data_len                      __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

uint8_t added_adv_data_len                          __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t added_scan_rsp_data_len                     __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

uint8_t adv_data_cursor                             __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t scan_rsp_data_cursor                        __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

bool update_adv_data                                __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize data values kept in global variables.
 ****************************************************************************************
 */
static void data_init()
{
    // Load initial advertising data and length
    user_adv_data_len = initial_adv_data_len;
    memcpy(user_adv_data, initial_adv_data, user_adv_data_len);

    // Load initial scan response data and length
    user_scan_rsp_data_len = initial_scan_rsp_data_len;
    memcpy(user_scan_rsp_data, initial_scan_rsp_data, user_scan_rsp_data_len);

    added_adv_data_len = 0;
    added_scan_rsp_data_len = 0;

    adv_data_cursor = initial_adv_data_len;
    scan_rsp_data_cursor = initial_scan_rsp_data_len;

    update_adv_data = true;
}

/**
 ****************************************************************************************
 * @brief Adds a byte to ADV_DATA if there is space.
 * @return true if ADV_DATA is full, otherwise false
 ****************************************************************************************
 */
static bool is_adv_data_full(void)
{
    if ((user_adv_data_len < ADV_DATA_LEN) && update_adv_data)
    {
        added_adv_data_len++;
        adv_data_cursor++;

        user_adv_data[initial_adv_data_len] = added_adv_data_len;

        user_adv_data[adv_data_cursor] = 0xA5;
        user_adv_data_len = initial_adv_data_len + added_adv_data_len + 1;

        return false;
    }
    return true;
}

/**
 ****************************************************************************************
 * @brief Adds a byte to SCAN_RSP_DATA if there is space.
 * @return true if SCAN_RSP_DATA is full, otherwise false
 ****************************************************************************************
 */
static bool is_scan_rsp_data_full(void)
{
    if ((user_scan_rsp_data_len < SCAN_RSP_DATA_LEN) && !update_adv_data)
    {
        added_scan_rsp_data_len++;
        scan_rsp_data_cursor++;

        user_scan_rsp_data[initial_scan_rsp_data_len] = added_scan_rsp_data_len;

        user_scan_rsp_data[scan_rsp_data_cursor] = 0xA5;
        user_scan_rsp_data_len = initial_scan_rsp_data_len + added_scan_rsp_data_len + 1;

        return false;
    }
    return true;
}

/**
 ****************************************************************************************
 * @brief Advertisement data update timer callback function.
 ****************************************************************************************
 */
static void adv_data_update_timer_cb(void)
{
    app_adv_data_update_timer_used = EASY_TIMER_INVALID_TIMER;

    if (is_adv_data_full() && update_adv_data)
    {
        // Ready to switch to SCAN_RSP_DATA
        update_adv_data = false;

        // Since the ADV_DATA has reached its limit (31-bytes), we start to fill the SCAN_RSP_DATA
        is_scan_rsp_data_full();

        // Stop advertising when switching from ADV_NONCONN_IND to ADV_SCAN_IND type and vice versa
        // On the fly update of the ADV_DATA or SCAN_RSP_DATA cannot be applied when the advertising type is dynamically changed.
        app_easy_gap_advertise_stop();

        // Exit here
        return;
    }

    if (is_scan_rsp_data_full() && !update_adv_data)
    {
        // Ready to switch to ADV_DATA
        update_adv_data = true;

        // Stop advertising when switching from ADV_NONCONN_IND to ADV_SCAN_IND type and vice versa
        // On the fly update of the ADV_DATA or SCAN_RSP_DATA cannot be applied when the advertising type is dynamically changed.
        app_easy_gap_advertise_stop();

        // Exit here
        return;
    }

    // Update advertising data on the fly
    app_easy_gap_update_adv_data(user_adv_data, user_adv_data_len, user_scan_rsp_data, user_scan_rsp_data_len);

    // Restart timer for the next advertising update
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);
}

void user_app_adv_start(void)
{
    // Schedule the next advertising data update
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);

    struct gapm_start_advertise_cmd *cmd = app_easy_gap_non_connectable_advertise_get_active();

    // Store initial advertising data and length
    initial_adv_data_len = cmd->info.host.adv_data_len;
    memcpy(initial_adv_data, cmd->info.host.adv_data, initial_adv_data_len);

    // Store initial scan response data and length
    initial_scan_rsp_data_len = cmd->info.host.scan_rsp_data_len;
    memcpy(initial_scan_rsp_data, cmd->info.host.scan_rsp_data, initial_scan_rsp_data_len);

    // Initialize data
    data_init();

    app_easy_gap_non_connectable_advertise_start();
}

void user_app_adv_nonconn_complete(uint8_t status)
{
    // If advertising was canceled then update advertising data and start advertising again
    if (status == GAP_ERR_CANCELED)
    {
        struct gapm_start_advertise_cmd *cmd = app_easy_gap_non_connectable_advertise_get_active();

        // Initialize ADV_DATA and SCAN_RSP_DATA to restart the process
        if (update_adv_data)
        {
            // Initialize data
            data_init();
        }

        // Load advertising data and length
        cmd->info.host.adv_data_len = user_adv_data_len;
        memcpy(cmd->info.host.adv_data, user_adv_data, user_adv_data_len);

        // Load scan response data and length
        cmd->info.host.scan_rsp_data_len = user_scan_rsp_data_len;
        memcpy(cmd->info.host.scan_rsp_data, user_scan_rsp_data, user_scan_rsp_data_len);

        app_easy_gap_non_connectable_advertise_start();

        // Schedule the next advertising data update
        app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);
    }
}

/// @} APP
