/**
 ****************************************************************************************
 *
 * @file user_ancs_client.c
 *
 * @brief ANCS client demo project source code.
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

#include <string.h>

#include "rwip_config.h"             // SW configuration
#include "gap.h"
#include "app_easy_timer.h"
#include "user_ancs_client.h"
#include "co_bt.h"
#include "arch_console.h"
#include "app_easy_security.h"
#include "co_list.h"
#include "gpio.h"

#define ANCC_ATTS_MAX_LEN       (25)
#define ANCC_ATTS_MSG_MAX_LEN   (75)
#define APP_ANCC_DELAY          (1000)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

// User application state
enum
{
    APP_STATE_DISCONNECTED,
    APP_STATE_CONNECTED,
    APP_STATE_BROWSING,
    APP_STATE_BROWSING_COMPLETE,
};

// ANCS notification details info
struct notification
{
    struct co_list_hdr hdr;
    struct anc_ntf_src ntf;
    char title[ANCC_ATTS_MAX_LEN];
    char date[ANCC_ATTS_MAX_LEN];
    char message[ANCC_ATTS_MSG_MAX_LEN];
    char *app_id;
};

// ANCS application info
struct application
{
    struct co_list_hdr hdr;
    char *app_id;
    char *display_name;
};

// Application service client data
struct svc_info
{
    bool browse_complete;
    struct prf_svc svc;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// Retained variables
static uint8_t app_connection_idx                       __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
static timer_hnd app_param_update_request_timer_used    __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
static timer_hnd app_ancc_delay_timer                   __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
static uint8_t app_user_state                           __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
static uint32_t last_notif_uid                          __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
static struct co_list notif_queue                       __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
static struct co_list app_queue                         __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
struct notification *pending_notif                      __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
struct svc_info gattc_svc                               __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
struct svc_info ancc_svc                                __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

/**
 ****************************************************************************************
 * @brief Returns category description string
 *
 * @param[in] category  ANCS category
 * @return category string
 ****************************************************************************************
 */
static const char *notifcategory2str(uint8_t category)
{
    switch (category) {
        case CAT_ID_OTHER:
                return "Other";
        case CAT_ID_CALL:
                return "Incoming call";
        case CAT_ID_MISSED_CALL:
                return "Missed call";
        case CAT_ID_VOICE_MAIL:
                return "Voicemail";
        case CAT_ID_SOCIAL:
                return "Social";
        case CAT_ID_SCHEDULE:
                return "Schedule";
        case CAT_ID_EMAIL:
                return "E-mail";
        case CAT_ID_NEWS:
                return "News";
        case CAT_ID_HEALTH_FITNESS:
                return "Health and Fitness";
        case CAT_ID_BUSINESS_FINANCE:
                return "Business and Finance";
        case CAT_ID_LOCATION:
                return "Location";
        case CAT_ID_ENTERTAINMENT:
                return "Entertainment";
        }

        return "<unknown>";
}

/**
 ****************************************************************************************
 * @brief Print notification details
 *
 * @param[in] notif    notification data
 * @param[in] app      application data
 ****************************************************************************************
 */
static void print_notification(const struct notification *notif, const struct application *app)
{
    const char *app_name = app ? app->display_name : "<unknown>";

    arch_printf("Notification from %s (%s)\r\n", app_name, (app && notif->app_id) ? notif->app_id : "<unknown>");
    arch_printf("\tCategory: %s\r\n", notifcategory2str(notif->ntf.cat_id));
    arch_printf("\t    Date: %s\r\n", notif->date);
    arch_printf("\t   Title: %s\r\n", notif->title);
    arch_printf("\t Message: %s\r\n", notif->message);
    arch_printf("\n");
}

/**
 ****************************************************************************************
 * @brief Free memory allocated for notification data
 *
 * @param[in] notif    notification data
 ****************************************************************************************
 */
static void free_notification(struct notification *notif)
{
    if (notif)
    {
        if (notif->app_id)
        {
            ke_free(notif->app_id);
        }
        ke_free(notif);
    }
}

/**
 ****************************************************************************************
 * @brief Clear notification queue
 ****************************************************************************************
 */
static void clear_notification_queue()
{
    while (!co_list_is_empty(&notif_queue))
    {
        struct notification *notif = (struct notification *)co_list_pop_front(&notif_queue);
        free_notification(notif);
    }
}

/**
 ****************************************************************************************
 * @brief Store notification data if possible
 *
 * @param[in] ntf    notification data
 ****************************************************************************************
 */
static void add_notification(const struct anc_ntf_src *ntf)
{
    if (ke_check_malloc(sizeof(struct notification), KE_MEM_ENV))
    {
        struct notification *notif = (struct notification *)ke_malloc(sizeof(struct notification), KE_MEM_ENV);
        memset(notif, 0, sizeof(struct notification));
        notif->ntf = *ntf;
        co_list_push_back(&notif_queue, (void *)notif);
    }
}

/**
 ****************************************************************************************
 * @brief Fetch notification from list and get attributes
 ****************************************************************************************
 */
static void fetch_next_notification()
{
    pending_notif = (struct notification *)co_list_pop_front(&notif_queue);
    if (!pending_notif)
    {
        return;
    }

    uint8_t atts = NTF_ATT_ID_APP_ID_PRESENT | NTF_ATT_ID_DATE_PRESENT |
                   NTF_ATT_ID_TITLE_PRESENT | NTF_ATT_ID_MSG_PRESENT;

    app_ancc_get_ntf_atts(app_connection_idx, pending_notif->ntf.ntf_uid, atts, ANCC_ATTS_MAX_LEN, 0, ANCC_ATTS_MSG_MAX_LEN);
}

/**
 ****************************************************************************************
 * @brief Free pending notification and check if fetch attributes for next one.
 ****************************************************************************************
 */
void free_pending_notif(void)
{
    if (pending_notif)
    {
        free_notification(pending_notif);
        pending_notif = NULL;
        fetch_next_notification();
    }
}

/**
 ****************************************************************************************
 * @brief Store application data if possible
 *
 * @param[in] ntf    notification data
 ****************************************************************************************
 */
static struct application *add_application(const char *app_id)
{
    struct application *app = NULL;
    uint32_t size = strlen(app_id);

    if (ke_check_malloc((size + 1), KE_MEM_ENV))
    {
        app = ke_malloc(sizeof(struct application), KE_MEM_ENV);
        memset(app, 0, sizeof(struct application));
        app->app_id = ke_malloc(size + 1, KE_MEM_ENV);
        memset(app->app_id, 0, size + 1);
        strcpy(app->app_id, app_id);
        co_list_push_back(&app_queue, (void*)app);
    }

    return app;
}

/**
 ****************************************************************************************
 * @brief Store application data if possible
 *
 * @param[in] ntf    notification data
 ****************************************************************************************
 */
static struct application *find_application(const char *app_id)
{
    if (!co_list_is_empty(&app_queue)) {
        struct co_list_hdr *el = app_queue.first;
        struct application *app = NULL;

        while (true)
        {
            app = (struct application *)el;
            if (!strcmp(app->app_id, app_id))
            {
                return (struct application*)el;
            }
            else if (el == app_queue.last)
            {
                return NULL;
            }
            else
            {
                el = co_list_next(el);
            }
        }
    }

    return NULL;
}

/**
 ****************************************************************************************
 * @brief Free memory allocated for application data
 *
 * @param[in] notif    application data
 ****************************************************************************************
 */
static void free_application(struct application *app)
{
    if (!app)
    {
        return;
    }
    if (app->app_id)
    {
        ke_free(app->app_id);
    }
    if(app->display_name)
    {
        ke_free(app->display_name);
    }

    ke_free(app);
}

/**
 ****************************************************************************************
 * @brief Clear application queue
 ****************************************************************************************
 */
void clear_application_queue()
{
    while (!co_list_is_empty(&app_queue))
    {
        free_application((struct application *)co_list_pop_front(&app_queue));
    }
}

/**
 ****************************************************************************************
 * @brief Clear ancs related data
 ****************************************************************************************
 */
void clear_ancs_data()
{
    clear_notification_queue();
    clear_application_queue();
    if (pending_notif)
    {
        free_notification(pending_notif);
        pending_notif = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Clear service information
 *
 * @param[in] svc    pointer to service handles
 ****************************************************************************************
 */
void clear_svc_info(struct svc_info *svc)
{
    svc->browse_complete = false;
    svc->svc.shdl = ATT_INVALID_HANDLE;
    svc->svc.ehdl = ATT_INVALID_HANDLE;
}

/**
 ****************************************************************************************
 * @brief Start services browsing
 ****************************************************************************************
 */
static void start_browsing(void)
{
    if (app_user_state != APP_STATE_BROWSING)
    {
        app_user_state = APP_STATE_BROWSING;
        if (gattc_svc.svc.shdl == ATT_INVALID_HANDLE)
        {
            app_gattc_enable(app_connection_idx);
        }

        if (ancc_svc.svc.shdl == ATT_INVALID_HANDLE)
        {
            app_ancc_enable(app_connection_idx);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Browsing timeout callback
 ****************************************************************************************
 */
static void browsing_delay_timer_cb(void)
{
    app_ancc_delay_timer = EASY_TIMER_INVALID_TIMER;
    arch_printf("Browsing...\r\n");
    start_browsing();
}

/**
 ****************************************************************************************
 * @brief Start browsing delay timer
 ****************************************************************************************
 */
static void browsing_delay_timer_start(void)
{
    app_ancc_delay_timer = app_easy_timer(MS_TO_TIMERUNITS(APP_ANCC_DELAY), browsing_delay_timer_cb);
}

/**
 ****************************************************************************************
 * @brief Stop browsing delay timer
 ****************************************************************************************
 */
static void browsing_delay_timer_stop()
{
    if (app_ancc_delay_timer != EASY_TIMER_INVALID_TIMER)
    {
        app_easy_timer_cancel(app_ancc_delay_timer);
        app_ancc_delay_timer = EASY_TIMER_INVALID_TIMER;
    }
}

/**
 ****************************************************************************************
 * @brief Clear services clients related data
 *
 * @param[in] svc    pointer to service handles
 ****************************************************************************************
 */
void purge_clients()
{
    clear_svc_info(&gattc_svc);
    clear_svc_info(&ancc_svc);
    clear_ancs_data();
    prf_reset_func(TASK_ID_ANCC, app_connection_idx);
    prf_reset_func(TASK_ID_GATT_CLIENT, app_connection_idx);
}

/**
 ****************************************************************************************
 * @brief Check if browsing operation is finished
 ****************************************************************************************
 */
static void svc_discovered()
{
    if (app_user_state == APP_STATE_BROWSING)
    {
        if (gattc_svc.browse_complete && ancc_svc.browse_complete)
        {
            app_user_state = APP_STATE_BROWSING_COMPLETE;
            arch_printf("Browse completed\r\n");

            arch_printf("\tANCS: %s\r\n", (ancc_svc.svc.shdl != ATT_INVALID_HANDLE) ? "found" : "not found");
            arch_printf("\tGATT: %s\r\n", (gattc_svc.svc.shdl != ATT_INVALID_HANDLE) ? "found" : "not found");
        }
    }
}

/**
 ****************************************************************************************
 * @brief Notification action button callback function
 ****************************************************************************************
 */
void notif_button_cb(void)
{
    GPIO_ResetIRQ(GPIO1_IRQn);
    if (app_user_state == APP_STATE_BROWSING_COMPLETE)
    {
        app_ancc_ntf_action(app_connection_idx, last_notif_uid, false);
    }
}

/**
 ****************************************************************************************
 * @brief Parameter update request timer callback function.
 ****************************************************************************************
*/
static void param_update_request_timer_cb()
{
    app_easy_gap_param_update_start(app_connection_idx);
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
}

void user_app_init(void)
{
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;

    default_app_on_init();

    app_easy_security_bdb_init();

    // Action button configuration
    NVIC_DisableIRQ(GPIO1_IRQn);
    GPIO_RegisterCallback(GPIO1_IRQn, notif_button_cb);
    GPIO_EnableIRQ(GPIO_PORT_1, GPIO_PIN_1, GPIO1_IRQn, true, true, 40);

    app_user_state = APP_STATE_DISCONNECTED;
    arch_printf("Start advertising...\r\n");
}

void user_app_connection(uint8_t conidx, struct gapc_connection_req_ind const *param)
{
    if (app_env[conidx].conidx != GAP_INVALID_CONIDX)
    {
        app_connection_idx = conidx;

        if (app_param_update_request_timer_used != EASY_TIMER_INVALID_TIMER)
        {
            app_easy_timer_cancel(app_param_update_request_timer_used);
            app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
        }
        // Check if the parameters of the established connection are the preferred ones.
        // If not then schedule a connection parameter update request.
        if ((param->con_interval < user_connection_param_conf.intv_min) ||
            (param->con_interval > user_connection_param_conf.intv_max) ||
            (param->con_latency != user_connection_param_conf.latency) ||
            (param->sup_to != user_connection_param_conf.time_out))
        {
            // Connection params are not these that we expect
            app_param_update_request_timer_used = app_easy_timer(APP_PARAM_UPDATE_REQUEST_TO, param_update_request_timer_cb);
        }

        app_user_state = APP_STATE_CONNECTED;
        arch_printf("Device connected\r\n");
        browsing_delay_timer_start();
    }

    default_app_on_connection(conidx, param);
}

void user_app_disconnect(struct gapc_disconnect_ind const *param)
{
    // Cancel the parameter update request timer
    if (app_param_update_request_timer_used != EASY_TIMER_INVALID_TIMER)
    {
        app_easy_timer_cancel(app_param_update_request_timer_used);
        app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
    }
    // Make sure that the delay timer is not running
    browsing_delay_timer_stop();
    app_user_state = APP_STATE_DISCONNECTED;
    purge_clients();
    arch_printf("Device disconnected\r\n");
    default_app_on_disconnect(param);
}

void user_app_on_tk_exch(uint8_t conidx,
                         struct gapc_bond_req_ind const * param)
{
    bool accept = true;

    if (param->data.tk_type == GAP_TK_OOB)
    {
        uint8_t oob_tk[KEY_LEN] = APP_SECURITY_OOB_TK_VAL;
        app_easy_security_tk_exch(conidx, (uint8_t*) oob_tk, KEY_LEN, accept);
    }
    else if (param->data.tk_type == GAP_TK_DISPLAY)
    {
        #if defined (CFG_PRINTF)
        uint32_t passkey = app_sec_gen_tk();
        arch_printf("\r\nEnter Passkey: %u", passkey);
        #else
        uint32_t passkey = APP_SECURITY_MITM_PASSKEY_VAL;
        #endif

        app_easy_security_tk_exch(conidx, (uint8_t*) &passkey, sizeof(passkey), accept);
    }
    else if (param->data.tk_type == GAP_TK_KEY_ENTRY)
    {
        uint32_t passkey = APP_SECURITY_MITM_PASSKEY_VAL;

        #if defined (CFG_PRINTF)
        arch_printf("\r\nPasskey Entered: %u", passkey);
        #endif

        app_easy_security_tk_exch(conidx, (uint8_t*) &passkey, sizeof(passkey), accept);
    }
    else if (param->data.tk_type == GAP_TK_KEY_CONFIRM)
    {
        #if defined (CFG_PRINTF)
        uint32_t passkey;
        // Print the 6 Least Significant Digits of Passkey
        char buf[6];
        passkey = (param->tk.key[0] << 0 ) | (param->tk.key[1] << 8 ) |
                  (param->tk.key[2] << 16) | (param->tk.key[3] << 24);
        arch_printf("\r\nConfirmation Value: ");
        for (uint8_t i=0; i<6; i++){
            buf[5-i] = passkey%10;
            passkey /= 10;
        }
        for (uint8_t i=0; i<6; i++){
            arch_printf("%u",buf[i]);
        }
        #endif

        app_easy_security_tk_exch(conidx, (uint8_t *) &param->tk, sizeof(param->tk), accept);
    }
}

void user_app_on_pairing_succeeded(uint8_t conidx)
{
    default_app_on_pairing_succeeded(conidx);
    app_ancc_wr_cfg_data_src(conidx, true);
}

void user_on_annc_enable_cb(uint8_t conidx, uint8_t status, const struct ancc_content *anc)
{
    ancc_svc.browse_complete = true;
    if (status == GAP_ERR_NO_ERROR)
    {
        ancc_svc.svc = anc->svc;
        app_ancc_wr_cfg_data_src(app_connection_idx, true);
    }
    svc_discovered();
}

void user_on_ancc_wr_cfg_ntf_src_cb(uint8_t conidx, uint8_t status)
{
    if (status != GAP_ERR_NO_ERROR)
    {
        app_easy_gap_disconnect(conidx);
    }
}

void user_on_ancc_wr_cfg_data_src_cb(uint8_t conidx, uint8_t status)
{
    if (status == GAP_ERR_NO_ERROR)
    {
        app_ancc_wr_cfg_ntf_src(conidx, true);
    }
    else if (status == ATT_ERR_INSUFF_AUTHEN)
    {
        app_easy_security_request(conidx);
    }
    else
    {
        // Disconnect in case of error
        app_easy_gap_disconnect(conidx);
    }
}

void user_on_ancc_ntf_src_ind_cb(uint8_t conidx, const struct anc_ntf_src *ntf)
{
    if (ntf->event_id == EVT_ID_NTF_ADDED)
    {
#if CFG_VERBOSE_LOG
        arch_printf("| Notification added (0x%08x)\r\n", ntf->ntf_uid);
        arch_printf("|\tflags=0x%02x\r\n", ntf->event_flags);
        arch_printf("|\tcategory=%d\r\n", ntf->cat_id);
        arch_printf("|\tcategory_count=%d\r\n", ntf->cat_cnt);
        arch_printf("\n");
#endif
        add_notification(ntf);
        last_notif_uid = ntf->ntf_uid;

        if (!pending_notif)
        {
            fetch_next_notification();
        }
    }
    else if (ntf->event_id == EVT_ID_NTF_MODIFIED)
    {
#if CFG_VERBOSE_LOG
        arch_printf("| Notification modified (0x%08x)\r\n", ntf->ntf_uid);
        arch_printf("|\tflags=0x%02x\r\n", ntf->event_flags);
        arch_printf("|\tcategory=%d\r\n", ntf->cat_id);
        arch_printf("|\tcategory_count=%d\r\n", ntf->cat_cnt);
        arch_printf("\n");
#endif
    }
    else if (ntf->event_id == EVT_ID_NTF_REMOVED)
    {
#if CFG_VERBOSE_LOG
        arch_printf("| Notification removed (0x%08x)\r\n", ntf->ntf_uid);
        arch_printf("\n");
#endif
    }
}

void user_on_ancc_ntf_att_ind_cb(uint8_t conidx, uint32_t uid, uint8_t att_id, uint8_t *val)
{
    uint32_t len = 0;
#if CFG_VERBOSE_LOG
    arch_printf("| Notification (%08x) attribute (%d)\r\n", uid, att_id);
    arch_printf("|\t%s\r\n", val);
    arch_printf("\n");
#endif

    if (!pending_notif)
    {
        return;
    }

    switch (att_id)
    {
        case NTF_ATT_ID_TITLE:
            strcpy(pending_notif->title, (char *)val);
        break;
        case NTF_ATT_ID_DATE:
            strcpy(pending_notif->date, (char *)val);
        break;
        case NTF_ATT_ID_MSG:
            strcpy(pending_notif->message, (char *)val);
        break;
        case NTF_ATT_ID_APP_ID:
            len = strlen((char *)val) + 1;
            if (ke_check_malloc(len, KE_MEM_ENV))
            {
                pending_notif->app_id = ke_malloc(len, KE_MEM_ENV);
                memset(pending_notif->app_id, 0, len);
                strcpy(pending_notif->app_id, (char *)val);
            }
        break;
        default:
        break;
    }
}

void user_on_ancc_app_att_ind_cb(uint8_t conidx, uint8_t att_id, uint8_t *app_id, uint8_t *att)
{
    struct application *app;
    uint32_t size = 0;

#if CFG_VERBOSE_LOG
    arch_printf("| Application (%s) attribute (%d)\r\n", app_id, att_id);
    arch_printf("|\t%s\r\n", att);
    arch_printf("\n");
#endif

    app = (struct application*) find_application((char*)app_id);
    if (!app)
    {
        app = add_application((char*)app_id);
    }

    if (app != NULL)
    {
        if (att_id == APP_ATT_ID_DISPLAY_NAME)
        {
            if (!app->display_name)
            {
                size = strlen((char*)att) + 1;
                if (ke_check_malloc(size, KE_MEM_ENV))
                {
                    app->display_name = ke_malloc(size, KE_MEM_ENV);
                    memset(app->display_name, 0, size);
                    strcpy(app->display_name, (char*)att);
                }
            }
        }
    }
}

void user_on_ancc_get_ntf_att_cmp_cb(uint8_t conidx, uint8_t status)
{
    struct application *app = NULL;

    if (!pending_notif)
    {
        return;
    }

    if (status != ATT_ERR_NO_ERROR)
    {
#if CFG_VERBOSE_LOG
        arch_printf("| FAILED to get attributes for 0x%08x\r\n\n", pending_notif->ntf.ntf_uid);
#endif
        free_pending_notif();
        return;
    }

    if (pending_notif->app_id)
    {
        app = (struct application *)find_application(pending_notif->app_id);
        if (!app)
        {
            app_ancc_get_app_atts(conidx, APP_ATT_ID_DISPLAY_NAME_PRESENT, (uint8_t*)pending_notif->app_id);
            return;
        }
    }

    print_notification(pending_notif, app);
    free_pending_notif();
}

void user_on_ancc_get_app_att_cmp_cb(uint8_t conidx, uint8_t status)
{
#if CFG_VERBOSE_LOG
    if (status != ATT_ERR_NO_ERROR)
    {
        printf("| FAILED to get attributes for %s\r\n\n", pending_notif->app_id);
    }
#endif

    if (pending_notif)
    {
        struct application *app = find_application(pending_notif->app_id);
        print_notification(pending_notif, app);
        free_pending_notif();
    }
}

void user_on_ancc_perf_ntf_act_cmp_cb(uint8_t conidx, uint8_t status)
{
#if CFG_VERBOSE_LOG
        arch_printf("| Perform notification action status: %d\r\n", status);
#endif
}

void user_on_gattc_enable(uint8_t conidx, uint8_t status, const struct gatt_client_content *gatt)
{
    gattc_svc.browse_complete = true;
    if (status == GAP_ERR_NO_ERROR)
    {
        app_gattc_write_ind_cfg(app_connection_idx, true);
        gattc_svc.svc = gatt->svc;
    }
    svc_discovered();
}

void user_on_gattc_svc_change_indication(uint8_t conidx, struct prf_svc *val)
{
#if CFG_VERBOSE_LOG
        arch_printf("| Service changed notification: start_h: 0x%04x, end_h: 0x%04x\r\n",
                    val->shdl, val->ehdl);
#endif

    if (app_user_state != APP_STATE_BROWSING)
    {
        if (val->shdl <= ancc_svc.svc.shdl && val->ehdl >= ancc_svc.svc.ehdl)
        {// ANCS service change - rediscover
            arch_printf("Service changed, browsing...\r\n");
            clear_svc_info(&ancc_svc);
            clear_ancs_data();
            start_browsing();
        }
        else if (ancc_svc.svc.shdl == ATT_INVALID_HANDLE && ancc_svc.svc.ehdl == ATT_INVALID_HANDLE)
        {// ANCS is not present but service change, check if ANCS was added
            arch_printf("Service changed, browsing...\r\n");
            start_browsing();
        }
    }
}

void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case GAPC_PARAM_UPDATED_IND:
        {
            // Cast the "param" pointer to the appropriate message structure
            struct gapc_param_updated_ind const *msg_param = (struct gapc_param_updated_ind const *)(param);

            // Check if updated Conn Params filled to preferred ones
            if ((msg_param->con_interval >= user_connection_param_conf.intv_min) &&
                (msg_param->con_interval <= user_connection_param_conf.intv_max) &&
                (msg_param->con_latency == user_connection_param_conf.latency) &&
                (msg_param->sup_to == user_connection_param_conf.time_out))
            {
            }
        } break;

        case GATTC_EVENT_REQ_IND:
        { // Confirm unhandled indication to avoid GATT timeout
            struct gattc_event_cfm * cfm = KE_MSG_ALLOC(GATTC_EVENT_CFM, src_id, dest_id, gattc_event_cfm);
            cfm->handle = ((struct gattc_event_ind const *)param)->handle;
            ke_msg_send(cfm);
        } break;

        default:
            break;
    }
}

/// @} APP
