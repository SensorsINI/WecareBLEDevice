/**
 ****************************************************************************************
 *
 * @file user_custs1_impl.c
 *
 * @brief Peripheral project Custom1 Server implementation source code.
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

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gpio.h"
#include "app_api.h"
#include "app.h"
#include "prf_utils.h"
#include "custs1.h"
#include "custs1_task.h"
#include "user_custs1_def.h"
#include "user_custs1_impl.h"
#include "user_peripheral.h"
#include "user_periph_setup.h"
#include "uart_utils.h"

// #include "da14531_printf.h"
#include "MCR35614R.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

ke_msg_id_t timer_used      __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint16_t indication_counter __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint16_t non_db_val_counter __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
extern timer_hnd app_spi2_led_timer_used;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void user_svc1_ctrl_wr_ind_handler(ke_msg_id_t const msgid,
                                      struct custs1_val_write_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t val = 0;
    memcpy(&val, &param->value[0], param->length);

    if (val != CUSTS1_CP_ADC_VAL1_DISABLE)
    {
        timer_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY, app_adcval1_timer_cb_handler);
    }
    else
    {
        if (timer_used != EASY_TIMER_INVALID_TIMER)
        {
            app_easy_timer_cancel(timer_used);
            timer_used = EASY_TIMER_INVALID_TIMER;
        }
    }
}

void user_svc1_led_wr_ind_handler(ke_msg_id_t const msgid,
                                     struct custs1_val_write_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t val = 0;
    memcpy(&val, &param->value[0], param->length);

	  // On the board, '0' turns on the LED, '1' turns off.
	  switch(val)
		{
				case 0:
						spi2_led_ctrl(false, false); // 0: both red and green on
						break;
				case 1:
					  spi2_led_ctrl(false, true); // 1: red on and green off
						break;
				case 2:
						spi2_led_ctrl(true, false); // 2: red off and green on
						break;
				case 3:
					  spi2_led_ctrl(true, true); // 3: red off and green off
						break;
				case 4:
					  app_spi2_led_timer_used = app_easy_timer(50, spi2_led_toggle);
						break;
				case 5:
						if (app_spi2_led_timer_used != EASY_TIMER_INVALID_TIMER)
						{
								app_easy_timer_cancel(app_spi2_led_timer_used);
								app_spi2_led_timer_used = EASY_TIMER_INVALID_TIMER;
						}
						break;
				default:
						break;
		}
}

void user_svc1_dac_val_cfg_ind_handler(ke_msg_id_t const msgid,
                                           struct custs1_val_write_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{

	  // da14531_printf("user_svc1_dac_val_cfg_ind_handler is called. And the parameter value 0 is: %x.\r\n", param->value[0]);
	
    // Generate indication when the central subscribes to it
    if (param->value[0])
    {
        uint8_t conidx = KE_IDX_GET(src_id);

        struct custs1_val_ind_req* req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_IND_REQ,
                                                          prf_get_task_from_id(TASK_ID_CUSTS1),
                                                          TASK_APP,
                                                          custs1_val_ind_req,
                                                          sizeof(indication_counter));

	 
        req->conidx = app_env[conidx].conidx;
        req->handle = SVC1_IDX_DAC_VALUE_VAL;
        req->length = sizeof(indication_counter);
        req->value[0] = (indication_counter >> 8) & 0xFF;
        req->value[1] = indication_counter & 0xFF;

				
        indication_counter++;

        ke_msg_send(req);
    }
}

// This value is sent from the host, and will be used to set DAC.
uint16_t globalDACValBuf[8] = {0};
uint32_t globalADCValBuf[16] = {0};
uint32_t globalADC2ValBuf[16] = {0};

void user_svc1_dac_val_wr_ind_handler(ke_msg_id_t const msgid,
                                          struct custs1_val_write_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
		uint16_t localDACValBuf[8] = {0};
		uint32_t ADC1ValBuf[16] = {0};
		uint32_t ADC2ValBuf[16] = {0};		
		uint32_t sweepModeADCValBuf[16] = {0};
		
		memcpy(localDACValBuf, &param->value[0], param->length);
		// da14531_printf("The value from the host to set DAC is: 0x%x.\r\n", localDACValBuf[0]);
		// Set the DAC
		spi2_dac_set_data(localDACValBuf);
		// Read the ADC output data
		spi2_adc1_readout(ADC1ValBuf);
	  // Read the ADC output data
		spi2_adc2_readout(ADC2ValBuf);
		
		for(int i = 0; i < 16; i++)
		{
				sweepModeADCValBuf[i] =  ADC2ValBuf[i];
		}		

		// replace the lower 4 data from ADC1
		for(int i = 0; i < 4; i++)
		{
				sweepModeADCValBuf[8 + i] =  ADC1ValBuf[i];
		}		
		
		// Send notification
    struct custs1_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                          prf_get_task_from_id(TASK_ID_CUSTS1),
                                                          TASK_APP,
                                                          custs1_val_ntf_ind_req,
                                                          DEF_SVC1_ADC_VAL_1_CHAR_LEN);

    struct custs1_val_set_req *req_set = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
                                                              prf_get_task_from_id(TASK_ID_CUSTS1),
                                                              TASK_APP,
                                                              custs1_val_set_req,
                                                              DEF_SVC1_ADC_VAL_1_CHAR_LEN);

    //req->conhdl = app_env->conhdl;
    req->handle = SVC1_IDX_ADC_VAL_1_VAL;
    req->length = DEF_SVC1_ADC_VAL_1_CHAR_LEN;
    req->notification = true;
    memcpy(req->value, sweepModeADCValBuf, DEF_SVC1_ADC_VAL_1_CHAR_LEN);

    ke_msg_send(req);

    req_set->handle = SVC1_IDX_ADC_VAL_1_VAL;
    req_set->length = DEF_SVC1_ADC_VAL_1_CHAR_LEN;
    memcpy(req_set->value, sweepModeADCValBuf, DEF_SVC1_ADC_VAL_1_CHAR_LEN);
    ke_msg_send(req_set);		
}

void user_svc1_dac_val_ntf_cfm_handler(ke_msg_id_t const msgid,
                                           struct custs1_val_write_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
}

void user_svc1_adc_val_1_cfg_ind_handler(ke_msg_id_t const msgid,
                                            struct custs1_val_write_ind const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
}

void user_svc1_adc_val_1_ntf_cfm_handler(ke_msg_id_t const msgid,
                                            struct custs1_val_write_ind const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
}

void user_svc1_button_cfg_ind_handler(ke_msg_id_t const msgid,
                                         struct custs1_val_write_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
}

void user_svc1_button_ntf_cfm_handler(ke_msg_id_t const msgid,
                                         struct custs1_val_write_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
}

void user_svc1_indicateable_cfg_ind_handler(ke_msg_id_t const msgid,
                                               struct custs1_val_write_ind const *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
}

void user_svc1_indicateable_ind_cfm_handler(ke_msg_id_t const msgid,
                                               struct custs1_val_write_ind const *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
}

void user_svc1_dac_val_att_info_req_handler(ke_msg_id_t const msgid,
                                                struct custs1_att_info_req const *param,
                                                ke_task_id_t const dest_id,
                                                ke_task_id_t const src_id)
{
    struct custs1_att_info_rsp *rsp = KE_MSG_ALLOC(CUSTS1_ATT_INFO_RSP,
                                                   src_id,
                                                   dest_id,
                                                   custs1_att_info_rsp);
    // Provide the connection index.
    rsp->conidx  = app_env[param->conidx].conidx;
    // Provide the attribute index.
    rsp->att_idx = param->att_idx;
    // Provide the current length of the attribute.
    rsp->length  = 0;
    // Provide the ATT error code.
    rsp->status  = ATT_ERR_NO_ERROR;

    ke_msg_send(rsp);
}

void user_svc1_rest_att_info_req_handler(ke_msg_id_t const msgid,
                                            struct custs1_att_info_req const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    struct custs1_att_info_rsp *rsp = KE_MSG_ALLOC(CUSTS1_ATT_INFO_RSP,
                                                   src_id,
                                                   dest_id,
                                                   custs1_att_info_rsp);
    // Provide the connection index.
    rsp->conidx  = app_env[param->conidx].conidx;
    // Provide the attribute index.
    rsp->att_idx = param->att_idx;
    // Force current length to zero.
    rsp->length  = 0;
    // Provide the ATT error code.
    rsp->status  = ATT_ERR_WRITE_NOT_PERMITTED;

    ke_msg_send(rsp);
}

void app_adcval1_timer_cb_handler()
{
		uint32_t ADC1ValBuf[16] = {0};
		
		// Read the ADC output data
		spi2_adc1_readout(ADC1ValBuf);
		
    struct custs1_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                          prf_get_task_from_id(TASK_ID_CUSTS1),
                                                          TASK_APP,
                                                          custs1_val_ntf_ind_req,
                                                          DEF_SVC1_ADC_VAL_1_CHAR_LEN);

    struct custs1_val_set_req *req_set = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
                                                              prf_get_task_from_id(TASK_ID_CUSTS1),
                                                              TASK_APP,
                                                              custs1_val_set_req,
                                                              DEF_SVC1_ADC_VAL_1_CHAR_LEN);

    //req->conhdl = app_env->conhdl;
    req->handle = SVC1_IDX_ADC_VAL_1_VAL;
    req->length = DEF_SVC1_ADC_VAL_1_CHAR_LEN;
    req->notification = true;
    memcpy(req->value, ADC1ValBuf, DEF_SVC1_ADC_VAL_1_CHAR_LEN);

    ke_msg_send(req);

    req_set->handle = SVC1_IDX_ADC_VAL_1_VAL;
    req_set->length = DEF_SVC1_ADC_VAL_1_CHAR_LEN;
    memcpy(req_set->value, ADC1ValBuf, DEF_SVC1_ADC_VAL_1_CHAR_LEN);
    ke_msg_send(req_set);

    if (ke_state_get(TASK_APP) == APP_CONNECTED)
    {
        // Set it once again until Stop command is received in Control Characteristic
        timer_used = app_easy_timer(APP_PERIPHERAL_CTRL_TIMER_DELAY, app_adcval1_timer_cb_handler);
    }
}
void user_svc2_write_1_wr_ind_handler(ke_msg_id_t const msgid,
																				struct custs1_val_write_ind const *param,
																				ke_task_id_t const dest_id,
																				ke_task_id_t const src_id)
{
    uint8_t val = 0;
    memcpy(&val, &param->value[0], param->length);
	
		struct custs1_env_tag *custs1_env = PRF_ENV_GET(CUSTS1, custs1);
		attmdb_att_set_value(custs1_env->shdl + SVC2_WRITE_1_VAL, DEF_SVC2_WRITE_VAL_1_CHAR_LEN, 0, (uint8_t *)&val);

	  // On the board, '0' disable the debugger, '1' enables the debuggger.
	  switch(val)
		{
				case 0:
						SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, NO_SWD);   // Disable debugger
						GPIO_ConfigurePin(SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, OUTPUT, PID_SPI_EN, true); //Enable DAC
						GPIO_ConfigurePin(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, OUTPUT, PID_GPIO, true);  // Disable IO
						GPIO_ConfigurePin(SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, OUTPUT, PID_GPIO, true); //Disable ADC2				
						// GPIO_EnablePorPin();
						break;
				case 1:
						SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, SWD_DATA_AT_P0_10);   // Enable debugger
						GPIO_ConfigurePin(SPI2_DAC_CS_PORT, SPI2_DAC_CS_PIN, OUTPUT, PID_GPIO, true); //Disable DAC
						GPIO_ConfigurePin(SPI2_IO_CS_PORT, SPI2_IO_CS_PIN, OUTPUT, PID_GPIO, true);  // Disable IO
						GPIO_ConfigurePin(SPI2_ADC2_CS_PORT, SPI2_ADC2_CS_PIN, OUTPUT, PID_GPIO, true); //Disable ADC2								
						break;
				default:
						break;
		}
}		


void user_svc3_read_non_db_val_handler(ke_msg_id_t const msgid,
                                           struct custs1_value_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    // Increase value by one
    non_db_val_counter++;

    struct custs1_value_req_rsp *rsp = KE_MSG_ALLOC_DYN(CUSTS1_VALUE_REQ_RSP,
                                                        prf_get_task_from_id(TASK_ID_CUSTS1),
                                                        TASK_APP,
                                                        custs1_value_req_rsp,
                                                        DEF_SVC3_READ_VAL_4_CHAR_LEN);

    // Provide the connection index.
    rsp->conidx  = app_env[param->conidx].conidx;
    // Provide the attribute index.
    rsp->att_idx = param->att_idx;
    // Force current length to zero.
    rsp->length  = sizeof(non_db_val_counter);
    // Provide the ATT error code.
    rsp->status  = ATT_ERR_NO_ERROR;
    // Copy value
    memcpy(&rsp->value, &non_db_val_counter, rsp->length);
    // Send message
    ke_msg_send(rsp);
}
