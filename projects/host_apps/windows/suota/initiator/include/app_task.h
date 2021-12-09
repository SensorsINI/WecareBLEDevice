/**
 ****************************************************************************************
 *
 * @file app_task.h
 *
 * @brief Header file for application handlers for ble events and responses.
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_TASK_H_
#define APP_TASK_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>          // standard integer
#include "ke_task.h"         // kernel task
#include "ke_msg.h"          // kernel message
#include "gapm_task.h"
#include "gapc_task.h"
#include "gattc_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

// application states
enum
{
    // Idle state
    APP_IDLE,
    // Scanning state
    APP_SCAN,
    // Connected state
    APP_CONNECTED,

    APP_EN_SERV_STATUS_NOTIFICATIONS,

    APP_WR_MEM_DEV,

    APP_WR_GPIO_MAP,

    APP_RD_MEM_INFO,

    APP_WR_PATCH_LEN,

    APP_WR_PATCH_DATA,

    APP_WR_END_OF_SUOTA,

    APP_RD_MEM_INFO_2,

    APP_RD_SUOTA_VERSION,

    APP_RD_MTU_SIZE,

    APP_RD_PD_CHAR_SIZE,

    APP_DLE_NEGOTIATION,

    APP_GATT_MTU_NEGOTIATION,

    APP_DISCONNECTING,

    // Number of defined states.
    APP_STATE_MAX
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
extern struct app_env_tag app_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_ready_evt_handler(ke_msg_id_t msgid,
                           void  *param,
                           ke_task_id_t dest_id,
                           ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles Inquiry result event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_dev_inq_result_handler(ke_msg_id_t msgid,
                                struct gapm_adv_report_ind *param,
                                ke_task_id_t dest_id,
                                ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles Connection request indication event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_connection_req_ind_handler(ke_msg_id_t msgid,
                                    struct gapc_connection_req_ind *param,
                                    ke_task_id_t dest_id,
                                    ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles Discconnection completion event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_disconnect_ind_handler(ke_msg_id_t msgid,
                                struct gapc_disconnect_ind *param,
                                ke_task_id_t dest_id,
                                ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles Param update req indication event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_param_update_req_ind_handler(ke_msg_id_t msgid,
                                      struct gapc_param_update_req_ind *param,
                                      ke_task_id_t dest_id,
                                      ke_task_id_t src_id);

/**

 ****************************************************************************************
 * @brief Get dev info req indication event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_get_dev_info_req_ind_handler(ke_msg_id_t msgid,
                                      struct gapc_get_dev_info_req_ind *param,
                                      ke_task_id_t dest_id,
                                      ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Set dev info req indication event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_set_dev_info_req_ind_handler(ke_msg_id_t msgid,
                                      struct gapc_set_dev_info_req_ind *param,
                                      ke_task_id_t dest_id,
                                      ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handle Bond indication.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_bond_ind_handler(ke_msg_id_t msgid,
                          struct gapc_bond_ind *param,
                          ke_task_id_t dest_id,
                          ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GAPC_BOND_REQ_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_bond_req_ind_handler(ke_msg_id_t msgid,
                              struct gapc_bond_req_ind *param,
                              ke_task_id_t dest_id,
                              ke_task_id_t src_id);

int gapc_le_pkt_size_ind_handler(ke_msg_id_t msgid,
                                 struct gapc_le_pkt_size_ind *param,
                                 ke_task_id_t dest_id,
                                 ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GAPM command completion events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_cmp_evt_handler(ke_msg_id_t msgid,
                         struct gapm_cmp_evt *param,
                         ke_task_id_t dest_id,
                         ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GAPC command completion events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_cmp_evt_handler(ke_msg_id_t msgid,
                         struct gapc_cmp_evt *param,
                         ke_task_id_t dest_id,
                         ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GATTC_DISC_SVC_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gattc_disc_svc_ind_handler(ke_msg_id_t msgid,
                               struct gattc_disc_svc_ind *param,
                               ke_task_id_t dest_id,
                               ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GATTC_DISC_CHAR_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gattc_disc_char_ind_handler(ke_msg_id_t msgid,
                                struct gattc_disc_char_ind *param,
                                ke_task_id_t dest_id,
                                ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GATTC_DISC_CHAR_DESC_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gattc_disc_char_desc_ind_handler(ke_msg_id_t msgid,
                                     struct gattc_disc_char_desc_ind *param,
                                     ke_task_id_t dest_id,
                                     ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GATTC command completion event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gattc_cmp_evt_handler(ke_msg_id_t msgid,
                          struct gattc_cmp_evt *param,
                          ke_task_id_t dest_id,
                          ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GATTC_READ_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gattc_read_ind_handler(ke_msg_id_t msgid,
                           struct gattc_read_ind *param,
                           ke_task_id_t dest_id,
                           ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GATTC_EVENT_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gattc_event_ind_handler(ke_msg_id_t msgid,
                            struct gattc_event_ind *param,
                            ke_task_id_t dest_id,
                            ke_task_id_t src_id);

/**
 ****************************************************************************************
 * @brief Handles GATTC_MTU_CHANGED_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gattc_mtu_changed_ind_handler(ke_msg_id_t msgid,
                                  struct gattc_mtu_changed_ind *param,
                                  ke_task_id_t dest_id,
                                  ke_task_id_t src_id);

#endif // APP_TASK_H_
