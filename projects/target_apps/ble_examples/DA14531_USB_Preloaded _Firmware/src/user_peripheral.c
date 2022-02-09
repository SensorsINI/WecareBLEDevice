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
timer_hnd app_spi2_timer_used                   __SECTION_ZERO("retention_mem_area0");
static void spi2_io_ctrl(void);

// Test: SPI2 DAC
timer_hnd app_spi2_dac_timer_used                   __SECTION_ZERO("retention_mem_area0");
static void spi2_dac_ctrl(void);

// Test: SPI2 ADC1
timer_hnd app_spi2_adc1_timer_used                   __SECTION_ZERO("retention_mem_area0");
static void spi2_adc1_ctrl(void);
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
    // app_spi2_timer_used = app_easy_timer(50, spi2_io_ctrl);
		// app_spi2_dac_timer_used = app_easy_timer(50, spi2_dac_ctrl);
		app_spi2_adc1_timer_used = app_easy_timer(50, spi2_adc1_ctrl);		
    
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
                    user_svc1_long_val_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_LONG_VALUE_NTF_CFG:
                    user_svc1_long_val_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case SVC1_IDX_LONG_VALUE_VAL:
                    user_svc1_long_val_wr_ind_handler(msgid, msg_param, dest_id, src_id);
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

                case SVC1_IDX_LONG_VALUE_VAL:
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
                case SVC1_IDX_LONG_VALUE_VAL:
                    user_svc1_long_val_att_info_req_handler(msgid, msg_param, dest_id, src_id);
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

        default:
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
                        .spi_cs = SPI_CS_1,                
                        .cs_pad.port = SPI2_DAC_CS_PORT,
                        .cs_pad.pin = SPI2_DAC_CS_PIN
#if defined(CFG_SPI_DMA_SUPPORT)
#endif
};

// Initialize SPI2 DAC driver
spi_cfg_t spi2_adc1_cfg = {  
	                      .spi_ms = SPI_MS_MODE_MASTER,
                        .spi_cp = SPI_CP_MODE_3,            // SPI Mode 0,0
                        .spi_speed = SPI_SPEED_MODE_1MHz,
                        .spi_wsz = SPI_MODE_8BIT,
                        .spi_cs = SPI_CS_1,                 
                        .cs_pad.port = SPI2_ADC1_CS_PORT,
                        .cs_pad.pin = SPI2_ADC1_CS_PIN
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
static void spi2_io_ctrl()
{
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

    // Restart timer for the next SPI2 transaction. Delay time: 50*10ms = 500ms
    app_spi2_timer_used = app_easy_timer(50, spi2_io_ctrl);
}

/**
 ****************************************************************************************
 * @brief SPI2 DAC (DAC70508M) control function.
 * @return void
 ****************************************************************************************
*/
static void spi2_dac_ctrl()
{
    spi_initialize(&spi2_dac_cfg);

    uint32_t dac_command = 0x80000000;
    uint32_t receiveBuf; 


    dac_command = 0x810000; // Read Device ID command
    spi_cs_low();
    spi_send(&dac_command, 1, SPI_OP_BLOCKING);
    spi_cs_high();

    // Readback data
    spi_cs_low();
    spi_receive(&receiveBuf, 1, SPI_OP_BLOCKING);
    spi_cs_high();

    printf_string(UART1, "DEVICE ID is: 0x");
    print_word(UART1, receiveBuf);
    printf_string(UART1, ".\r\n");

    dac_command = 0x820000; // Read SYNC register command
    spi_cs_low();
    spi_send(&dac_command, 1, SPI_OP_BLOCKING);
    spi_cs_high();

    // Readback data
    spi_cs_low();
    spi_receive(&receiveBuf, 1, SPI_OP_BLOCKING);
    spi_cs_high();

    printf_string(UART1, "SYNC register data before writing is: 0x");
    print_word(UART1, receiveBuf);
    printf_string(UART1, ".\r\n");

    dac_command = 0x0200FF; // Write 0xFF to SYNC register command
    spi_cs_low();
    spi_send(&dac_command, 1, SPI_OP_BLOCKING);
    spi_cs_high();	

    dac_command = 0x820000; // Read SYNC register command
    spi_cs_low();
    spi_send(&dac_command, 1, SPI_OP_BLOCKING);
    spi_cs_high();

    // Readback data
    spi_cs_low();
    spi_receive(&receiveBuf, 1, SPI_OP_BLOCKING);
    spi_cs_high();

    printf_string(UART1, "SYNC register data after writing is: 0x");
    print_word(UART1, receiveBuf);
    printf_string(UART1, ".\r\n");


    app_spi2_dac_timer_used = app_easy_timer(50, spi2_dac_ctrl);
}


/**
 ****************************************************************************************
 * @brief SPI2 ADC1 module (MCP3564R) test timer callback function.
 * @return void
 ****************************************************************************************
*/
static void spi2_adc1_ctrl()
{
    spi_initialize(&spi2_adc1_cfg);
	  
	  uint32_t adc1_command;
	  uint32_t receiveBuf; 

	  adc1_command = 0x76;
	  uint32_t valToWrite = 0xa5;
	  spi_cs_low();
		spi_send(&adc1_command, 1, SPI_OP_BLOCKING);
    spi_send(&valToWrite, 1, SPI_OP_BLOCKING);
    spi_cs_high();
	
	  adc1_command = 0x75;
	  spi_cs_low();
		spi_send(&adc1_command, 1, SPI_OP_BLOCKING);
    spi_receive(&receiveBuf, 1, SPI_OP_BLOCKING);
    spi_cs_high();

    printf_string(UART1, "LOCK register data is: 0x");
    printf_byte(UART1, receiveBuf);
    printf_string(UART1, ".\r\n");
//	
    app_spi2_adc1_timer_used = app_easy_timer(50, spi2_adc1_ctrl);
}

/// @} APP
