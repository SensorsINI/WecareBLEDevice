/**
 ****************************************************************************************
 *
 * @file user_peripheral.c
 *
 * @brief Peripheral project source code.
 *
 * Copyright (c) 2015-2019 Dialog Semiconductor. All rights reserved.
 *
 * This software ("Software") is owned by Dialog Semiconductor.
 *
 * By using this Software you agree that Dialog Semiconductor retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Dialog Semiconductor is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Dialog Semiconductor products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * DIALOG SEMICONDUCTOR BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
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

#include "rwip_config.h"             // SW configuration
#include "gap.h"
#include "app_easy_timer.h"
#include "user_peripheral.h"
#include "user_custs1_impl.h"
#include "user_custs1_def.h"
#include "co_bt.h"
#include "uart_utils.h"

// TEST: set adc_val_2 in db
#include "custs1.h"
#include "attm_db.h"
#include "prf_utils.h"

// TEST: SPI2
#include "spi_531.h"
#include "spi.h"

#include "user_periph_setup.h"
#include "timer0.h"
#include "timer0_2.h"

#include "DAC70508M.h"
#include "MCR35614R.h"
#include "utils.h"
#include "da14531_printf.h"

#include "app_diss_task.h" 
// Outside value
extern uint16_t globalDACValBuf[8];
extern uint32_t globalADCValBuf[16];
extern uint32_t globalADC2ValBuf[16];

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

// Manufacturer Specific Data ADV structure type
struct mnf_specific_data_ad_structure
{
    uint8_t ad_structure_size;
    uint8_t ad_structure_type;
    uint8_t company_id[APP_AD_MSD_COMPANY_ID_LEN];
    uint8_t proprietary_data[APP_AD_MSD_DATA_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

uint8_t app_connection_idx                      __SECTION_ZERO("retention_mem_area0");
timer_hnd app_adv_data_update_timer_used        __SECTION_ZERO("retention_mem_area0");
timer_hnd app_param_update_request_timer_used   __SECTION_ZERO("retention_mem_area0");

// Retained variables
struct mnf_specific_data_ad_structure mnf_data  __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
// Index of manufacturer data in advertising data or scan response data (when MSB is 1)
uint8_t mnf_data_index                          __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_adv_data_len                     __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_scan_rsp_data_len                __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_adv_data[ADV_DATA_LEN]           __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint8_t stored_scan_rsp_data[SCAN_RSP_DATA_LEN] __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

// Test: SPI2 timer
timer_hnd app_spi2_led_timer_used                   __SECTION_ZERO("retention_mem_area0");

// Test: SPI2 DAC
timer_hnd app_spi2_dac_timer_used                   __SECTION_ZERO("retention_mem_area0");
static void spi2_dac_ctrl(void);

// Test: SPI2 ADC1
timer_hnd app_spi2_adc1_timer_used                   __SECTION_ZERO("retention_mem_area0");
static void spi2_adc1_ctrl(void);
// timer_hnd app_spi2_adc1_timer_init                   __SECTION_ZERO("retention_mem_area0");
static void spi2_adc1_init(void);

static uint32_t adcConversionCount = 0;
static uint32_t adc2ConversionCount = 0;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

/**
 ****************************************************************************************
 * @brief Initialize Manufacturer Specific Data
 * @return void
 ****************************************************************************************
 */
static void mnf_data_init()
{
    mnf_data.ad_structure_size = sizeof(struct mnf_specific_data_ad_structure ) - sizeof(uint8_t); // minus the size of the ad_structure_size field
    mnf_data.ad_structure_type = GAP_AD_TYPE_MANU_SPECIFIC_DATA;
    mnf_data.company_id[0] = APP_AD_MSD_COMPANY_ID & 0xFF; // LSB
    mnf_data.company_id[1] = (APP_AD_MSD_COMPANY_ID >> 8 )& 0xFF; // MSB
    mnf_data.proprietary_data[0] = 0;
    mnf_data.proprietary_data[1] = 0;
}

/**
 ****************************************************************************************
 * @brief Update Manufacturer Specific Data
 * @return void
 ****************************************************************************************
 */
static void mnf_data_update()
{
    uint16_t data;

    data = mnf_data.proprietary_data[0] | (mnf_data.proprietary_data[1] << 8);
    data += 1;
    mnf_data.proprietary_data[0] = data & 0xFF;
    mnf_data.proprietary_data[1] = (data >> 8) & 0xFF;

    if (data == 0xFFFF) {
         mnf_data.proprietary_data[0] = 0;
         mnf_data.proprietary_data[1] = 0;
    }
}

/**
 ****************************************************************************************
 * @brief Add an AD structure in the Advertising or Scan Response Data of the
  *       GAPM_START_ADVERTISE_CMD parameter struct.
 * @param[in] cmd               GAPM_START_ADVERTISE_CMD parameter struct
 * @param[in] ad_struct_data    AD structure buffer
 * @param[in] ad_struct_len     AD structure length
 * @param[in] adv_connectable   Connectable advertising event or not. It controls whether
 *                              the advertising data use the full 31 bytes length or only
 *                              28 bytes (Document CCSv6 - Part 1.3 Flags).
 * @return void
 */
static void app_add_ad_struct(struct gapm_start_advertise_cmd *cmd, void *ad_struct_data, uint8_t ad_struct_len, uint8_t adv_connectable)
{
    uint8_t adv_data_max_size = (adv_connectable) ? (ADV_DATA_LEN - 3) : (ADV_DATA_LEN);

    if ((adv_data_max_size - cmd->info.host.adv_data_len) >= ad_struct_len)
    {
        // Append manufacturer data to advertising data
        memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len], ad_struct_data, ad_struct_len);

        // Update Advertising Data Length
        cmd->info.host.adv_data_len += ad_struct_len;

        // Store index of manufacturer data which are included in the advertising data
        mnf_data_index = cmd->info.host.adv_data_len - sizeof(struct mnf_specific_data_ad_structure);
    }
    else if ((SCAN_RSP_DATA_LEN - cmd->info.host.scan_rsp_data_len) >= ad_struct_len)
    {
        // Append manufacturer data to scan response data
        memcpy(&cmd->info.host.scan_rsp_data[cmd->info.host.scan_rsp_data_len], ad_struct_data, ad_struct_len);

        // Update Scan Response Data Length
        cmd->info.host.scan_rsp_data_len += ad_struct_len;

        // Store index of manufacturer data which are included in the scan response data
        mnf_data_index = cmd->info.host.scan_rsp_data_len - sizeof(struct mnf_specific_data_ad_structure);
        // Mark that manufacturer data is in scan response and not advertising data
        mnf_data_index |= 0x80;
    }
    else
    {
        // Manufacturer Specific Data do not fit in either Advertising Data or Scan Response Data
        ASSERT_WARNING(0);
    }
    // Store advertising data length
    stored_adv_data_len = cmd->info.host.adv_data_len;
    // Store advertising data
    memcpy(stored_adv_data, cmd->info.host.adv_data, stored_adv_data_len);
    // Store scan response data length
    stored_scan_rsp_data_len = cmd->info.host.scan_rsp_data_len;
    // Store scan_response data
    memcpy(stored_scan_rsp_data, cmd->info.host.scan_rsp_data, stored_scan_rsp_data_len);
}

/**
 ****************************************************************************************
 * @brief Advertisement data update timer callback function.
 * @return void
 ****************************************************************************************
*/
static void adv_data_update_timer_cb()
{
    // If mnd_data_index has MSB set, manufacturer data is stored in scan response
    uint8_t *mnf_data_storage = (mnf_data_index & 0x80) ? stored_scan_rsp_data : stored_adv_data;

    // Update manufacturer data
    mnf_data_update();

    // Update the selected fields of the advertising data (manufacturer data)
    memcpy(mnf_data_storage + (mnf_data_index & 0x7F), &mnf_data, sizeof(struct mnf_specific_data_ad_structure));

    // Update advertising data on the fly
    app_easy_gap_update_adv_data(stored_adv_data, stored_adv_data_len, stored_scan_rsp_data, stored_scan_rsp_data_len);

    // Restart timer for the next advertising update
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);
}

/**
 ****************************************************************************************
 * @brief Parameter update request timer callback function.
 * @return void
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

    // Initialize Manufacturer Specific Data
    mnf_data_init();

    // Initialize Advertising and Scan Response Data
    memcpy(stored_adv_data, USER_ADVERTISE_DATA, USER_ADVERTISE_DATA_LEN);
    stored_adv_data_len = USER_ADVERTISE_DATA_LEN;
    memcpy(stored_scan_rsp_data, USER_ADVERTISE_SCAN_RESPONSE_DATA, USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN);
    stored_scan_rsp_data_len = USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN;

    default_app_on_init();
}

void user_app_adv_start(void)
{
    // Schedule the next advertising data update
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);

    struct gapm_start_advertise_cmd* cmd;
    cmd = app_easy_gap_undirected_advertise_get_active();

    // Add manufacturer data to initial advertising or scan response data, if there is enough space
    app_add_ad_struct(cmd, &mnf_data, sizeof(struct mnf_specific_data_ad_structure), 1);

    app_easy_gap_undirected_advertise_start();
}

void user_app_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
{
    if (app_env[connection_idx].conidx != GAP_INVALID_CONIDX)
    {
        app_connection_idx = connection_idx;

        // Stop the advertising data update timer
        app_easy_timer_cancel(app_adv_data_update_timer_used);

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
    }
    else
    {
        // No connection has been established, restart advertising
        user_app_adv_start();
    }

    default_app_on_connection(connection_idx, param);
}

void user_app_adv_undirect_complete(uint8_t status)
{
    // If advertising was canceled then update advertising data and start advertising again
    if (status == GAP_ERR_CANCELED)
    {
        user_app_adv_start();
    }
}

void user_app_db_init_complete(void)
{
		////////////////////////////////////////////////////////////////
		// TEST: set adc_val_2 in db

		// ---------------- Method 1: attmdb_att_set_value() ----------------

		// Dummy sample value
		static uint16_t sample = 0x55AA;

		// Need to include: "custs1.h" "prf_utils.h" "attm_db.h"
		struct custs1_env_tag *custs1_env = PRF_ENV_GET(CUSTS1, custs1);
		attmdb_att_set_value(custs1_env->shdl + SVC1_IDX_ADC_VAL_2_VAL, DEF_SVC1_ADC_VAL_2_CHAR_LEN, 0, (uint8_t *)&sample);
		static uint16_t dacInitVal = 0x1FFF;
		attmdb_att_set_value(custs1_env->shdl + SVC1_IDX_DAC_VALUE_VAL, DEF_SVC1_DAC_VALUE_CHAR_LEN, 0, (uint8_t *)&dacInitVal);
		static char * dacDescName = "DAC";
		attmdb_att_set_value(custs1_env->shdl + SVC1_IDX_DAC_VALUE_USER_DESC, sizeof(dacDescName) - 1, 0, (uint8_t *)dacDescName);

		//Set svc uuid value      
		volatile uint8_t conidx = KE_IDX_GET(TASK_APP);
		uint8_t att_idx = 0;
		// retrieve handle information
		uint8_t status = custs1_get_att_idx(SVC1_IDX_ADC_VAL_1_NTF_CFG, &att_idx);
		uint8_t value[2]={0x55,0xaa};
		custs1_set_ccc_value(conidx, 9, *(uint16_t *)value);	
		
//		struct diss_set_value_req *req = KE_MSG_ALLOC_DYN(DISS_SET_VALUE_REQ,
//																										 prf_get_task_from_id(TASK_ID_DISS),
//																										 TASK_APP,
//																										 diss_set_value_req,
//																										 10);
//		//req->conhdl = app_env->conhdl;
//		req->value = 0;
//		req->length = 10;
//		char *tmpData = "INI, UZH-ETHZ";
//		memcpy(req->data, tmpData, 10);
//		ke_msg_send(req);
		
		 
    // ---------------- Method 2: ke_msg_send() ----------------

    // // Dummy sample value
    // static uint16_t sample      __SECTION_ZERO("retention_mem_area0");
    // sample = 0x55AA;

    // struct custs1_val_set_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
    //                                                   prf_get_task_from_id(TASK_ID_CUSTS1),
    //                                                   TASK_APP,
    //                                                   custs1_val_set_req,
    //                                                   DEF_SVC1_ADC_VAL_2_CHAR_LEN);

    // //req->conhdl = app_env->conhdl;
    // req->handle = SVC1_IDX_ADC_VAL_2_VAL;
    // req->length = DEF_SVC1_ADC_VAL_2_CHAR_LEN;
    // memcpy(req->value, &sample, DEF_SVC1_ADC_VAL_2_CHAR_LEN);

    // ke_msg_send(req);

    ////////////////////////////////////////////////////////////////

    // Test: SPI2 timer start. Delay time: 50*10ms = 500ms
    // app_spi2_led_timer_used = app_easy_timer(50, spi2_led_toggle);
		app_spi2_dac_timer_used = app_easy_timer(50, spi2_dac_ctrl);
		app_spi2_adc1_timer_used = app_easy_timer(50, spi2_adc1_ctrl);		
		
		// Turn on a LED to show that the board is running
		spi2_led_ctrl(true, false);
		
    user_app_adv_start();
}

void user_app_disconnect(struct gapc_disconnect_ind const *param)
{
    // Cancel the parameter update request timer
    if (app_param_update_request_timer_used != EASY_TIMER_INVALID_TIMER)
    {
        app_easy_timer_cancel(app_param_update_request_timer_used);
        app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
    }
    // Update manufacturer data for the next advertsing event
    mnf_data_update();
    // Restart Advertising
    user_app_adv_start();
}

void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
		// printf_string(UART1, "Write something.\r\n");

	  // printf_string(UART1, "The message id is:");
    // print_word(UART1, msgid);
    // printf_string(UART1, ".\r\n");
//	  da14531_printf("Receive unhandled message from sdk app layer. The message type is: 0x%x.\r\n", msgid);
    switch(msgid)
    {
        case CUSTS1_VAL_WRITE_IND:
        {
            struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)(param);

            switch (msg_param->handle)
            {
                case SVC1_IDX_CONTROL_POINT_VAL:
                    user_svc1_ctrl_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_LED_STATE_VAL:
                    user_svc1_led_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_ADC_VAL_1_NTF_CFG:
                    user_svc1_adc_val_1_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_BUTTON_STATE_NTF_CFG:
                    user_svc1_button_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_INDICATEABLE_IND_CFG:
                    user_svc1_dac_val_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_DAC_VALUE_NTF_CFG:
                    user_svc1_dac_val_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_DAC_VALUE_VAL:
                    user_svc1_dac_val_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;
								case SVC2_WRITE_1_VAL:
										user_svc2_write_1_wr_ind_handler(msgid, msg_param, dest_id, src_id);
										break;

                default:
                    break;
            }
        } break;

        case CUSTS1_VAL_NTF_CFM:
        {
            struct custs1_val_ntf_cfm const *msg_param = (struct custs1_val_ntf_cfm const *)(param);

            switch (msg_param->handle)
            {
                case SVC1_IDX_ADC_VAL_1_VAL:
                    break;

                case SVC1_IDX_BUTTON_STATE_VAL:
                    break;

                case SVC1_IDX_DAC_VALUE_VAL:
                    break;

                default:
                    break;
            }
        } break;

        case CUSTS1_VAL_IND_CFM:
        {
            struct custs1_val_ind_cfm const *msg_param = (struct custs1_val_ind_cfm const *)(param);

            switch (msg_param->handle)
            {
                case SVC1_IDX_INDICATEABLE_VAL:
                    break;

                default:
                    break;
             }
        } break;

        case CUSTS1_ATT_INFO_REQ:
        {
            struct custs1_att_info_req const *msg_param = (struct custs1_att_info_req const *)param;

            switch (msg_param->att_idx)
            {
                case SVC1_IDX_DAC_VALUE_VAL:
                    user_svc1_dac_val_att_info_req_handler(msgid, msg_param, dest_id, src_id);
                    break;

                default:
                    user_svc1_rest_att_info_req_handler(msgid, msg_param, dest_id, src_id);
                    break;
             }
        } break;

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

        case CUSTS1_VALUE_REQ_IND:
        {
            struct custs1_value_req_ind const *msg_param = (struct custs1_value_req_ind const *) param;

            switch (msg_param->att_idx)
            {
                case SVC3_IDX_READ_4_VAL:
                {
                    user_svc3_read_non_db_val_handler(msgid, msg_param, dest_id, src_id);
                } break;

                default:
                {
                    // Send Error message
                    struct custs1_value_req_rsp *rsp = KE_MSG_ALLOC(CUSTS1_VALUE_REQ_RSP,
                                                                    src_id,
                                                                    dest_id,
                                                                    custs1_value_req_rsp);

                    // Provide the connection index.
                    rsp->conidx  = app_env[msg_param->conidx].conidx;
                    // Provide the attribute index.
                    rsp->att_idx = msg_param->att_idx;
                    // Force current length to zero.
                    rsp->length = 0;
                    // Set Error status
                    rsp->status  = ATT_ERR_APP_ERROR;
                    // Send message
                    ke_msg_send(rsp);
                } break;
             }
        } break;
				
			  case GAPM_CMP_EVT: 
				{
					 struct gapm_cmp_evt const *msg_param = (struct gapm_cmp_evt const *)(param);
        }  break;
								

        default:
					  da14531_printf("Caught message not handled by any function. The message type is: 0x%x\r\n", msgid);
            break;
    }
}


// Initialize SPI2 IO driver
spi_cfg_t spi2_cfg = {  .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_0,            // SPI Mode 0,0
                        .spi_speed = SPI_SPEED_MODE_2MHz,
                        .spi_wsz = SPI_MODE_16BIT,
                        .spi_cs = SPI_CS_0,                 
                        .cs_pad.port = SPI2_IO_CS_PORT,
                        .cs_pad.pin = SPI2_IO_CS_PIN
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};

// Initialize SPI2 DAC driver
spi_cfg_t spi2_dac_cfg = {  
	                      .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_1,            // SPI Mode 0,1
                        .spi_speed = SPI_SPEED_MODE_2MHz,
                        .spi_wsz = SPI_MODE_32BIT,
                        .spi_cs = SPI_CS_0,                
                        .cs_pad.port = SPI2_DAC_CS_PORT,
                        .cs_pad.pin = SPI2_DAC_CS_PIN
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};

// Initialize SPI2 ADC1 driver
spi_cfg_t spi2_adc1_cfg = {  
	                      .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_0,            // SPI Mode 0,0
                        .spi_speed = SPI_SPEED_MODE_2MHz,
                        .spi_wsz = SPI_MODE_8BIT,
                        .spi_cs = SPI_CS_1,                 
                        .cs_pad.port = SPI2_ADC1_CS_PORT,
                        .cs_pad.pin = SPI2_ADC1_CS_PIN
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};

// Initialize SPI2 ADC2 driver
spi_cfg_t spi2_adc2_cfg = {  
	                      .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_0,            // SPI Mode 0,0
                        .spi_speed = SPI_SPEED_MODE_2MHz,
                        .spi_wsz = SPI_MODE_8BIT,
                        .spi_cs = SPI_CS_0,                 
                        .cs_pad.port = SPI2_ADC2_CS_PORT,
                        .cs_pad.pin = SPI2_ADC2_CS_PIN		
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};

// LED2 on wecare board (MAX7317 P2)
// The higest 8bits are port number (MSB for R/W_n)
// The loweset 8bits are value.
// On the board, '0' turns on the LED, '1' turns off.
uint16_t reg_val_led2 = 0x0201;
// LED3 on wecare board (MAX7317 P3)
uint16_t reg_val_led3 = 0x0300;

uint16_t globalCnt = 0;

/**
 ****************************************************************************************
 * @brief SPI2 IO module (MAX7317) test timer callback function.
 * @return void
 ****************************************************************************************
*/
void spi2_led_toggle()
{
		GPIO_ConfigurePin(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, OUTPUT, PID_SPI_EN, true);  // Enable IO
		GPIO_ConfigurePin(SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, OUTPUT, PID_GPIO, true); // Disable DAC
		GPIO_ConfigurePin(SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, OUTPUT, PID_GPIO, true); //Disable ADC2
    spi_initialize(&spi2_cfg);
	
    // for (uint8_t i=0; i<0xFF; i++) {
    reg_val_led2 ^= 1;
    reg_val_led3 ^= 1;

    // For LED 2  
    // spi_ctrl_reg_spi_fifo_reset_setf(SPI_BIT_DIS);	
    // GPIO_SetInactive(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN);	
	  // timer0_2_clk_enable(); 
    // Enable SWTIM_IRQn irq
    // timer0_enable_irq(); 
    // Start Timer0
    // timer0_start();	
	  // for(uint64_t i = 0; i < 5; i++);
		// GPIO_SetActive(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN);

    spi_cs_low();
	  spi_send(&reg_val_led2, 1, SPI_OP_BLOCKING);
    spi_cs_high();
		  
    // For LED 3
    spi_cs_low();
    spi_send(&reg_val_led3, 1, SPI_OP_BLOCKING);
    spi_cs_high();
    // }

		app_spi2_led_timer_used = app_easy_timer(50, spi2_led_toggle);
}

/**
 ****************************************************************************************
 * @brief SPI2 DAC (DAC70508M) control function.
 * @return void
 ****************************************************************************************
*/
static void spi2_dac_ctrl()
{
		GPIO_ConfigurePin(SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, OUTPUT, PID_SPI_EN, true); //Enable DAC
		GPIO_ConfigurePin(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, OUTPUT, PID_GPIO, true);  // Disable IO
		GPIO_ConfigurePin(SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, OUTPUT, PID_GPIO, true); //Disable ADC2			
    spi_initialize(&spi2_dac_cfg);

    uint16_t regData; 

    if(!spi2_dac_read_register(DEVICE_ID, &regData)) 
		{
			  // da14531_printf("DEVICE ID is: 0x%x.\r\n", regData);			
		}

    if(!spi2_dac_read_register(GAIN, &regData)) 
		{
			  // da14531_printf("GAIN channel 4 register is: 0x%x.\r\n", regData);			
		}

    if(!spi2_dac_read_register(STATUS, &regData))  
		{
			  // da14531_printf("STATUS channel 4 register is: 0x%x.\r\n", regData);			
		}
				
//		// Reset the device
//		spi2_dac_write_register(TRIGGER, 0xa);
    
		spi2_dac_write_register(GAIN, 0x1ff);
    if(!spi2_dac_read_register(STATUS, &regData))  
		{
			  // da14531_printf("STATUS channel 4 register is: 0x%x.\r\n", regData);			
		}
    
		// spi2_dac_write_register(BRDCAST, 0xafff);
		
		const uint8_t DAC_CHS[8] = {DAC0, DAC1, DAC2, DAC3, DAC4, DAC5, DAC6, DAC7};
		for (int i = 0; i < 8; i++)
		{
				spi2_dac_write_register(DAC_CHS[i], globalDACValBuf[i]);
		}
		
		// spi2_dac_write_register(CONFIG, 0x0ff);
    app_spi2_dac_timer_used = app_easy_timer(50, spi2_dac_ctrl);
}

static bool initFlag = true;   // Make sure initilization only execute once
/**
 ****************************************************************************************
 * @brief SPI2 ADC1 module (MCP3564R) init function.
 * @return void
 ****************************************************************************************
*/
static void spi2_adc1_init(void)
{	
	  uint32_t regVal;
	  uint8_t sendBuf[TOTAL_BYTES] = {0};
	  uint8_t receiveBuf[TOTAL_BYTES] = {0}; 
 
		// Full reset
		spi2_adc_fast_command(FAST_CMD_FULL_RESET);

		// Read all the register values.
		spi2_adc_increment_read_register(ADCDATA, receiveBuf, TOTAL_BYTES);
		
		// Set PRE[1:0] to 1 (default) and OSR to some big enough value because ADC conversion rate is faster than SPI speed.
		// Here we set OSR to 20480. That is the minimum setting we tested by experiments.
		sendBuf[0] = 0x28;
		spi2_adc_write_register(CONFIG1, sendBuf, CONFIG1_BYTES);	
		
//		// Configure CONFIG2 register: GAIN('000'): set gain to 1/3.
//		sendBuf[0] = 0x83; 
//		spi2_adc_write_register(CONFIG2, sendBuf, CONFIG2_BYTES);	
		
		// Configure CONFIG3 register: CONV_MODE('11'): Continous conversion in scan mode. DATA_FORMAT('11'): 32bit with channel ID.
		sendBuf[0] = 0xF0; 
		spi2_adc_write_register(CONFIG3, sendBuf, CONFIG3_BYTES);	
		
		// Configure IRQ regiseter: IRQ_Mode('01'): IRQ output and inactive state is logic high
		// This configuration is important, because if we use the default configuration, then
		// IRQ is in high-Z state and because we don't have external pull-up resistor on board.
		// Therefore, IRQ cannot generate falling edge and SDO cannot be updated resulting in 
		// SPI reading on ADCDATA always 0.
		sendBuf[0] = 0x07;
		spi2_adc_write_register(IRQ, sendBuf, IRQ_BYTES);
		
		// Configure MUX register: AVDD reading
		sendBuf[0] = 0x98;
		spi2_adc_write_register(MUX, sendBuf, MUX_BYTES);
		
		// Configure SCAN register: 4 Differential Channels plus 8 Single-Ended Channels.
		sendBuf[0] = 0x00;
		sendBuf[1] = 0x0F;
		sendBuf[2] = 0xFF;
		spi2_adc_write_register(SCAN, sendBuf, SCAN_BYTES);

		// Configure CONFIG0 register: Internal V_ref and internal master clock, no bias and ADC conversion mode.
		// This register configuration should be put in the last of init function as this register is used to start
		// the conversion.
		sendBuf[0] = 0xF3;  // The first byte is the MSbs for registers have more than 8bits
		// sendBuf[1] = 0x02;
		// sendBuf[2] = 0x03;
		spi2_adc_write_register(CONFIG0, sendBuf, CONFIG0_BYTES);
		
		// Read CONFIG0 register.
		spi2_adc_static_read_register(CONFIG0, receiveBuf, CONFIG0_BYTES);	
		regVal = swapBufToRealVal(receiveBuf, CONFIG0_BYTES);
		// da14531_printf("CONFIG0 register data is: 0x%x.\r\n", regVal);
}
					
static uint32_t errorCnt = 0;
/**
 ****************************************************************************************
 * @brief SPI2 ADC1 module (MCP3564R) test timer callback function.
 * @return void
 ****************************************************************************************
*/
static void spi2_adc1_ctrl()
{ 
    spi_initialize(&spi2_adc1_cfg);
	
    uint32_t regVal = 0;	
		uint8_t sendBuf[TOTAL_BYTES] = {0};	
    uint8_t receiveBuf[TOTAL_BYTES] = {0}; 
 
		if(initFlag)
		{			
				spi2_adc1_init();		
				initFlag = false;
		}

//		da14531_printf("Reset the watchdog.\r\n");
//		wdg_reload(WATCHDOG_DEFAULT_PERIOD);		

//		//Disable the interrupts
//		GLOBAL_INT_STOP();	
		
		adcConversionCount++;
		da14531_printf("Start the %dth ADC conversion. \r\n", adcConversionCount);
		
		// Start ADC conversion
		spi2_adc_fast_command(FAST_CMD_START_CONVERSION);
	
		spi2_adc_static_read_register(CONFIG2, receiveBuf, CONFIG2_BYTES);	
		regVal = swapBufToRealVal(receiveBuf, CONFIG2_BYTES);
    // Obtain the GAIN bits value
		uint8_t gainReg = (regVal & 0x38) >> 3;
	  // Convert it to the real gain
		float gainFactor = 0.333;  // if gainReg == 0, then gainFactor is 1/3
		if(gainReg != 0)
		{
				gainFactor = 1 << (gainReg - 1);
		}		
		
		float voltage[12];
		bool validFlg = true; // Check if this conversion valid
		for(int i = 0; i < 12; i++)
		{		
				sendBuf[0] = ADC_CHANNEL_ID[i];
				spi2_adc_write_register(MUX, sendBuf, MUX_BYTES);		
			
				// Read IRQ register
				spi2_adc_static_read_register(IRQ, receiveBuf, IRQ_BYTES);
				uint8_t irqVal = swapBufToRealVal(receiveBuf, IRQ_BYTES);
			  // Wait the conversion finish
				while((irqVal & 0x40) != 0)
				{
						spi2_adc_static_read_register(IRQ, receiveBuf, IRQ_BYTES);
						irqVal = swapBufToRealVal(receiveBuf, IRQ_BYTES);
				}			
				
				// STATIC Read ADCDATA
				spi2_adc_static_read_register(ADCDATA, receiveBuf, ADCDATA_BYTES);	
				regVal = swapBufToRealVal(receiveBuf, ADCDATA_BYTES);
				volatile uint32_t channelID = (regVal >> 28) & 0xF;
				regVal = (regVal & 0xFFFFFFF) + (((regVal >> 24) & 0xF) << 28);
				int32_t voltageVal = (int32_t)(regVal);
				voltage[channelID] = voltageVal/(0x800000 * gainFactor) * 2.4;    // The internal reference voltage is 2.4V
				da14531_printf("The voltage of channel ID %d is: %.4fV.\r\n",  channelID, voltage[channelID]);
				
				if ((channelID + i) != 11)
				{
						da14531_printf("ADC conversion error.\r\n");
						validFlg = false;
				}
		}
		
		if(validFlg == false)
		{
				errorCnt++;
		}
		
		// Copy voltage hex buffer to value shared with BLE for sending to the host
		memcpy(globalADCValBuf, voltage, sizeof(float) * 12);			
    globalADCValBuf[12] = adcConversionCount; // Store the ADC conversion count
		globalADCValBuf[13] = errorCnt;
		globalADCValBuf[14] = validFlg;           // Indicate the current conversion valid or not
		
		// Shutdown ADC to save power after conversion
		spi2_adc_fast_command(FAST_CMD_ADC_SHUTDOWN);
		
		da14531_printf("ADC conversion error count is %d.\r\n", errorCnt);
		
//		// restore interrupts
//		GLOBAL_INT_START();		
		
    app_spi2_adc1_timer_used = app_easy_timer(50, spi2_adc1_ctrl);
}



static bool initFlagADC2 = true;   // Make sure initilization only execute once
/**
 ****************************************************************************************
 * @brief SPI2 ADC1 module (MCP3564R) init function.
 * @return void
 ****************************************************************************************
*/
static void spi2_adc2_init(void)
{	
	  uint32_t regVal;
	  uint8_t sendBuf[TOTAL_BYTES] = {0};
	  uint8_t receiveBuf[TOTAL_BYTES] = {0}; 
 
		// Full reset
		spi2_adc_fast_command(FAST_CMD_FULL_RESET);

		// Read all the register values.
		spi2_adc_increment_read_register(ADCDATA, receiveBuf, TOTAL_BYTES);
		
		// Set PRE[1:0] to 1 (default) and OSR to some big enough value because ADC conversion rate is faster than SPI speed.
		// Here we set OSR to 20480. That is the minimum setting we tested by experiments.
		sendBuf[0] = 0x28;
		spi2_adc_write_register(CONFIG1, sendBuf, CONFIG1_BYTES);	
		
//		// Configure CONFIG2 register: GAIN('000'): set gain to 1/3.
//		sendBuf[0] = 0x83; 
//		spi2_adc_write_register(CONFIG2, sendBuf, CONFIG2_BYTES);	
		
		// Configure CONFIG3 register: CONV_MODE('11'): Continous conversion in scan mode. DATA_FORMAT('11'): 32bit with channel ID.
		sendBuf[0] = 0xF0; 
		spi2_adc_write_register(CONFIG3, sendBuf, CONFIG3_BYTES);	
		
		// Configure IRQ regiseter: IRQ_Mode('01'): IRQ output and inactive state is logic high
		// This configuration is important, because if we use the default configuration, then
		// IRQ is in high-Z state and because we don't have external pull-up resistor on board.
		// Therefore, IRQ cannot generate falling edge and SDO cannot be updated resulting in 
		// SPI reading on ADCDATA always 0.
		sendBuf[0] = 0x07;
		spi2_adc_write_register(IRQ, sendBuf, IRQ_BYTES);
		
		// Configure MUX register: AVDD reading
		sendBuf[0] = 0x98;
		spi2_adc_write_register(MUX, sendBuf, MUX_BYTES);
		
		// Configure SCAN register: 4 Differential Channels plus 8 Single-Ended Channels.
		sendBuf[0] = 0x00;
		sendBuf[1] = 0x0F;
		sendBuf[2] = 0xFF;
		spi2_adc_write_register(SCAN, sendBuf, SCAN_BYTES);

		// Configure CONFIG0 register: Internal V_ref and internal master clock, no bias and ADC conversion mode.
		// This register configuration should be put in the last of init function as this register is used to start
		// the conversion.
		sendBuf[0] = 0xF3;  // The first byte is the MSbs for registers have more than 8bits
		// sendBuf[1] = 0x02;
		// sendBuf[2] = 0x03;
		spi2_adc_write_register(CONFIG0, sendBuf, CONFIG0_BYTES);
		
		// Read CONFIG0 register.
		spi2_adc_static_read_register(CONFIG0, receiveBuf, CONFIG0_BYTES);	
		regVal = swapBufToRealVal(receiveBuf, CONFIG0_BYTES);
		// da14531_printf("CONFIG0 register data is: 0x%x.\r\n", regVal);
}
					
static uint32_t errorCntADC2 = 0;
/**
 ****************************************************************************************
 * @brief SPI2 ADC1 module (MCP3564R) test timer callback function.
 * @return void
 ****************************************************************************************
*/
static void spi2_adc2_ctrl()
{ 
    spi_initialize(&spi2_adc2_cfg);
	
    uint32_t regVal = 0;	
		uint8_t sendBuf[TOTAL_BYTES] = {0};	
    uint8_t receiveBuf[TOTAL_BYTES] = {0}; 
 
		if(initFlagADC2)
		{			
				GPIO_ConfigurePin(SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, OUTPUT, PID_SPI_EN, true);		// Enable ADC2	
				GPIO_ConfigurePin(SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, OUTPUT, PID_GPIO, true); // Disable DAC
				GPIO_ConfigurePin(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, OUTPUT, PID_GPIO, true); //Disable IO	
				spi2_adc2_init();					
				initFlagADC2 = false;
		}

//		da14531_printf("Reset the watchdog.\r\n");
//		wdg_reload(WATCHDOG_DEFAULT_PERIOD);		

//		//Disable the interrupts
//		GLOBAL_INT_STOP();	
		
		adc2ConversionCount++;
		da14531_printf("Start the %dth ADC2 conversion. \r\n", adc2ConversionCount);
		
		// Start ADC conversion
		spi2_adc_fast_command(FAST_CMD_START_CONVERSION);
	
		spi2_adc_static_read_register(CONFIG2, receiveBuf, CONFIG2_BYTES);	
		regVal = swapBufToRealVal(receiveBuf, CONFIG2_BYTES);
    // Obtain the GAIN bits value
		uint8_t gainReg = (regVal & 0x38) >> 3;
	  // Convert it to the real gain
		float gainFactor = 0.333;  // if gainReg == 0, then gainFactor is 1/3
		if(gainReg != 0)
		{
				gainFactor = 1 << (gainReg - 1);
		}		
		
		float voltage[12];
		bool validFlg = true; // Check if this conversion valid
		for(int i = 0; i < 12; i++)
		{		
				sendBuf[0] = ADC_CHANNEL_ID[i];
				spi2_adc_write_register(MUX, sendBuf, MUX_BYTES);		
			
				// Read IRQ register
				spi2_adc_static_read_register(IRQ, receiveBuf, IRQ_BYTES);
				uint8_t irqVal = swapBufToRealVal(receiveBuf, IRQ_BYTES);
			  // Wait the conversion finish
				while((irqVal & 0x40) != 0)
				{
						spi2_adc_static_read_register(IRQ, receiveBuf, IRQ_BYTES);
						irqVal = swapBufToRealVal(receiveBuf, IRQ_BYTES);
				}			
				
				// STATIC Read ADCDATA
				spi2_adc_static_read_register(ADCDATA, receiveBuf, ADCDATA_BYTES);	
				regVal = swapBufToRealVal(receiveBuf, ADCDATA_BYTES);
				volatile uint32_t channelID = (regVal >> 28) & 0xF;
				regVal = (regVal & 0xFFFFFFF) + (((regVal >> 24) & 0xF) << 28);
				int32_t voltageVal = (int32_t)(regVal);
				voltage[channelID] = voltageVal/(0x800000 * gainFactor) * 2.4;    // The internal reference voltage is 2.4V
				da14531_printf("The voltage of channel ID %d is: %.4fV.\r\n",  channelID, voltage[channelID]);
				
				if ((channelID + i) != 11)
				{
						da14531_printf("ADC2 conversion error.\r\n");
						validFlg = false;
				}
		}
		
		if(validFlg == false)
		{
				errorCntADC2++;
		}
		
		// Copy voltage hex buffer to value shared with BLE for sending to the host
		memcpy(globalADCValBuf, voltage, sizeof(float) * 12);			
    globalADC2ValBuf[12] = adc2ConversionCount; // Store the ADC conversion count
		globalADC2ValBuf[13] = errorCntADC2;
		globalADC2ValBuf[14] = validFlg;           // Indicate the current conversion valid or not
		
		// Shutdown ADC to save power after conversion
		spi2_adc_fast_command(FAST_CMD_ADC_SHUTDOWN);
		
		da14531_printf("ADC conversion error count is %d.\r\n", errorCnt);
		
//		// restore interrupts
//		GLOBAL_INT_START();		
		
    app_spi2_adc1_timer_used = app_easy_timer(50, spi2_adc1_ctrl);
}

/**
 ****************************************************************************************
 * @brief API functions export
 ****************************************************************************************
*/
void spi2_led_ctrl(bool redLed, bool greenLed)
{
		// Disable LED controll from the timer first
		if (app_spi2_led_timer_used != EASY_TIMER_INVALID_TIMER)
		{
				app_easy_timer_cancel(app_spi2_led_timer_used);
				app_spi2_led_timer_used = EASY_TIMER_INVALID_TIMER;
		}
						
		GPIO_ConfigurePin(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, OUTPUT, PID_SPI_EN, true);  // Enable IO
		GPIO_ConfigurePin(SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, OUTPUT, PID_GPIO, true); // Disable DAC
		GPIO_ConfigurePin(SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, OUTPUT, PID_GPIO, true); //Disable ADC2
    spi_initialize(&spi2_cfg);

    uint16_t reg_val_ledRed = (RED_LED_PORT << 8) + redLed;
    uint16_t reg_val_ledGreen = (GREEN_LED_PORT << 8) + greenLed;

	  // For the red LED
    spi_cs_low();
	  spi_send(&reg_val_ledRed, 1, SPI_OP_BLOCKING);
    spi_cs_high();
		  
    // For the green LED
    spi_cs_low();
    spi_send(&reg_val_ledGreen, 1, SPI_OP_BLOCKING);
    spi_cs_high();
}

/// @} APP