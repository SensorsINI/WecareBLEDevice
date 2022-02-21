#include <stdint.h>
#include "diss_task.h"          // Device Information Service Server Task API
#include "diss.h"               // Device Information Service Definitions
#include "app_diss.h"           // Device Information Service Application Definitions
#include "my_own_app_diss_task.h"      // Device Information Service Application Task API
#include "app_task.h"           // Application Task API
#include "app_entry_point.h"
#include "app.h"
#include "sdk_version.h"
#include "user_profiles_config.h"

int my_own_diss_value_req_ind_handler(ke_msg_id_t const msgid,
                                      struct diss_value_req_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Initialize length
    uint8_t len = 0;
    // Pointer to the data
    uint8_t *data = NULL;

    // Check requested value
    switch (param->value)
    {
        case DIS_MANUFACTURER_NAME_CHAR:
        {
            // Set information
            len = APP_DIS_MANUFACTURER_NAME_LEN;
            data = (uint8_t *)APP_DIS_MANUFACTURER_NAME;
        } break;

        case DIS_MODEL_NB_STR_CHAR:
        {
            // Set information
            len = APP_DIS_MODEL_NB_STR_LEN;
            data = (uint8_t *)APP_DIS_MODEL_NB_STR;
        } break;

        case DIS_SYSTEM_ID_CHAR:
        {
            // Set information
            len = APP_DIS_SYSTEM_ID_LEN;
            data = (uint8_t *)APP_DIS_SYSTEM_ID;
        } break;

        case DIS_PNP_ID_CHAR:
        {
            // Set information
            len = APP_DIS_PNP_ID_LEN;
            data = (uint8_t *)APP_DIS_PNP_ID;
        } break;

        case DIS_SERIAL_NB_STR_CHAR:
        {
            // Set information
            len = APP_DIS_SERIAL_NB_STR_LEN;
            data = (uint8_t *)APP_DIS_SERIAL_NB_STR;
        } break;

        case DIS_HARD_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_HARD_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_HARD_REV_STR;
        } break;

        case DIS_FIRM_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_FIRM_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_FIRM_REV_STR;
        } break;

        case DIS_SW_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_SW_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_SW_REV_STR;
        } break;

        case DIS_IEEE_CHAR:
        {
            // Set information
            len = APP_DIS_IEEE_LEN;
            data = (uint8_t *)APP_DIS_IEEE;
        } break;

        default:
            ASSERT_ERROR(0);
            break;
    }

    // Allocate confirmation to send the value
    struct diss_value_cfm *cfm_value = KE_MSG_ALLOC_DYN(DISS_VALUE_CFM,
            src_id, dest_id,
            diss_value_cfm,
            len);

    // Set parameters
    cfm_value->value = param->value;
    cfm_value->length = len;
    if (len)
    {
        // Copy data
        memcpy(&cfm_value->data[0], data, len);
    }
    // Send message
    ke_msg_send(cfm_value);

    return (KE_MSG_CONSUMED);
}
