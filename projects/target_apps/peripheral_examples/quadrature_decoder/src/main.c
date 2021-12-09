/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Quadrature decoder example
 *
 * Copyright (C) 2012-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include "arch_system.h"
#include "user_periph_setup.h"
#include "wkupct_quadec.h"
#include "gpio.h"
#include "uart.h"
#include "uart_utils.h"



/**
 ****************************************************************************************
 * @brief Quadrature Decoder test function
 ****************************************************************************************
 */
void quad_decoder_test(void);

uint8_t terminate_quad_test;
uint8_t quad_started = 0;
uint8_t qdec_xcnt;
uint8_t qdec_ycnt;
uint8_t qdec_zcnt;
uint8_t qdec_new_data = false;

/**
 ****************************************************************************************
 * @brief Main routine of the quadrature decoder example
 *
 ****************************************************************************************
 */
int main (void)
{
    system_init();
    quad_decoder_test();
    while(1);
}

/**
 ****************************************************************************************
 * @brief  Quadrature Decoder callback function
 *
 ****************************************************************************************
 */
void quad_decoder_user_callback_function(int16_t qdec_xcnt_reg, int16_t qdec_ycnt_reg, int16_t qdec_zcnt_reg)
{
    qdec_xcnt = qdec_xcnt_reg;
    qdec_ycnt = qdec_ycnt_reg;
    qdec_zcnt = qdec_zcnt_reg;
    qdec_new_data = true;

    quad_decoder_enable_irq(1);
}

/**
 ****************************************************************************************
 * @brief WKUP callback function
 *
 ****************************************************************************************
 */
void wkup_callback_function(void)
{
    if(!GPIO_GetPinStatus(WKUP_TEST_BUTTON_1_PORT, WKUP_TEST_BUTTON_1_PIN))
    {
        terminate_quad_test = true;
    }

    if(!GPIO_GetPinStatus(WKUP_TEST_BUTTON_2_PORT, WKUP_TEST_BUTTON_2_PIN))
    {
        quad_started ^= true;
    }

    wkupct_enable_irq(WKUPCT_PIN_SELECT(WKUP_TEST_BUTTON_1_PORT, WKUP_TEST_BUTTON_1_PIN) ^
                      WKUPCT_PIN_SELECT(WKUP_TEST_BUTTON_2_PORT, WKUP_TEST_BUTTON_2_PIN),
                      WKUPCT_PIN_POLARITY(WKUP_TEST_BUTTON_1_PORT, WKUP_TEST_BUTTON_1_PIN,
                      WKUPCT_PIN_POLARITY_LOW) ^ WKUPCT_PIN_POLARITY(WKUP_TEST_BUTTON_2_PORT,
                      WKUP_TEST_BUTTON_2_PIN, WKUPCT_PIN_POLARITY_LOW), // polarity low
                      1,    // 1 event
                      40);  // debouncing time = 40

    return;
}

void quad_decoder_test(void)
{
    int16_t qdec_xcnt_reg;

    printf_string(UART, "Quadrature Decoder / WKUP controller\n\r");
    printf_string(UART, "SW3 button to start/stop Quadec polling\n\r");
    printf_string(UART, "SW2 button to terminate Quadec test\n\r");

    printf_string(UART, "\n\r Quadrature Decoder Test started!.");
    printf_string(UART, "\n\r Press SW2 button to terminate test.\n\r");

    QUAD_DEC_INIT_PARAMS_t quad_dec_init_param = {.chx_port_sel = QUADRATURE_ENCODER_CHX_CONFIGURATION,
                                                  .chy_port_sel = QUADRATURE_ENCODER_CHY_CONFIGURATION,
                                                  .chz_port_sel = QUADRATURE_ENCODER_CHZ_CONFIGURATION,
                                                  .qdec_clockdiv = QDEC_CLOCK_DIVIDER,
                                                  .qdec_events_count_to_trigger_interrupt = QDEC_EVENTS_COUNT_TO_INT,};
    quad_decoder_init(&quad_dec_init_param);
    quad_decoder_register_callback(quad_decoder_user_callback_function);
    quad_decoder_enable_irq(1);

    wkupct_register_callback(wkup_callback_function);
    wkupct_enable_irq(WKUPCT_PIN_SELECT(WKUP_TEST_BUTTON_1_PORT, WKUP_TEST_BUTTON_1_PIN) ^
                      WKUPCT_PIN_SELECT(WKUP_TEST_BUTTON_2_PORT, WKUP_TEST_BUTTON_2_PIN),
                      WKUPCT_PIN_POLARITY(WKUP_TEST_BUTTON_1_PORT, WKUP_TEST_BUTTON_1_PIN,
                      WKUPCT_PIN_POLARITY_LOW) ^ WKUPCT_PIN_POLARITY(WKUP_TEST_BUTTON_2_PORT,
                      WKUP_TEST_BUTTON_2_PIN, WKUPCT_PIN_POLARITY_LOW), // polarity low
                      1,    // 1 event
                      40);  // debouncing time = 40

    quad_started = false;
    terminate_quad_test = false;
    while(terminate_quad_test == false)
    {
        // polling
        if (quad_started)
        {
            qdec_xcnt_reg = quad_decoder_get_x_counter();
            printf_string(UART, "\n\r Quadec Polling DX: ");
            printf_byte(UART, qdec_xcnt_reg >> 8);
            printf_byte(UART, qdec_xcnt_reg & 0xFF);
        }

        // interrupt triggered
        if (qdec_new_data == true)
        {
            qdec_new_data = false;

            printf_string(UART, "\n\n\rQuadec ISR report:");

            printf_string(UART, "\n\r DX: ");
            printf_byte(UART, qdec_xcnt >> 8);
            printf_byte(UART, qdec_xcnt & 0xFF);

            printf_string(UART, " DY: ");
            printf_byte(UART, qdec_ycnt >> 8);
            printf_byte(UART, qdec_ycnt & 0xFF);

            printf_string(UART, " DZ: ");
            printf_byte(UART, qdec_zcnt >> 8);
            printf_byte(UART, qdec_zcnt & 0xFF);
        }
    }
    printf_string(UART, "\n\r Quadrature Decoder Test terminated!");
    printf_string(UART, "\n\rEnd of test\n\r");
}
