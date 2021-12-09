/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Timer0 PWM example
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
#include "uart.h"
#include "uart_utils.h"
#include "user_periph_setup.h"
#include "timer0_2.h"
#include "timer0.h"

// PWM settings
#define TIMER_ON        1000
#define PWM_HIGH        500
#define PWM_LOW         200
#define ALL_NOTES       26
#define TIMER0_PWM_TEST_EXPIRATION_COUNTER_RELOAD_VALUE 50

// table with values used for setting different PWM duty cycle to change the "melody" played by buzzer
const uint16_t notes[ALL_NOTES] = {1046, 987, 767, 932, 328, 880, 830, \
                                   609, 783, 991, 739, 989, 698, 456,  \
                                   659, 255, 622, 254, 587, 554, 365,  \
                                   523, 251, 493, 466, 440};

volatile uint16_t timer0_pwm_test_expiration_counter;

static tim0_2_clk_div_config_t clk_div_config =
{
    .clk_div  = TIM0_2_CLK_DIV_8
};

/**
 ****************************************************************************************
 * @brief PWM test function
 ****************************************************************************************
 */
void timer0_pwm_test(void);

/**
 ****************************************************************************************
 * @brief Main routine of the timer0 pwm example
 ****************************************************************************************
 */
int main (void)
{
    system_init();
    timer0_pwm_test();
    while(1);
}

/**
 ****************************************************************************************
 * @brief User callback function called by SWTIM_IRQn interrupt. Function is changing
 *        "note" played by buzzer by changing PWM duty cycle.
 ****************************************************************************************
 */ 
static void pwm0_user_callback_function(void)
{ 
    static uint8_t n = 0;
    static uint8_t i = 0;

    if (n == 10) //change "note" every 10 * 32,768ms
    {
        n = 0;
        // Change note and set ON-counter to Ton = 1/2Mhz * 65536 = 32,768ms
        timer0_set_pwm_on_counter(0xFFFF);
        timer0_set_pwm_high_counter(notes[i]/3 * 2);
        timer0_set_pwm_low_counter(notes[i]/3);

        printf_string(UART, "*");
        
        // limit i iterator to max index of table of notes
        i = (i+1) % (ALL_NOTES - 1);
        
        if (timer0_pwm_test_expiration_counter)
        {
            timer0_pwm_test_expiration_counter--;
        }
    }
    n++;
}

void timer0_pwm_test(void)
{
    printf_string(UART, "\n\r\n\r");
    printf_string(UART, "*******************\n\r");
    printf_string(UART, "* TIMER0 PWM TEST *\n\r");
    printf_string(UART, "*******************\n\r");

    // Enable the Timer0/Timer2 input clock
    timer0_2_clk_enable();

    // Set the Timer0/Timer2 input clock division factor to 8, so 16 MHz / 8 = 2 MHz input clock
    timer0_2_clk_div_set(&clk_div_config);

    // initialize PWM with the desired settings
    timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_NO_DIV);

    // set pwm Timer0 'On', Timer0 'high' and Timer0 'low' reload values
    timer0_set(TIMER_ON, PWM_HIGH, PWM_LOW);

    // register callback function for SWTIM_IRQn irq
    timer0_register_callback(pwm0_user_callback_function);

    // enable SWTIM_IRQn irq
    timer0_enable_irq();

    timer0_pwm_test_expiration_counter = TIMER0_PWM_TEST_EXPIRATION_COUNTER_RELOAD_VALUE;

    printf_string(UART, "\n\rTIMER0 starts!");
    printf_string(UART, "\n\rYou can hear the sound produced by PWM0 or PWM1");
#if defined (__DA14531__)
    printf_string(UART, "\n\rif you attach a buzzer on pins P0_9 or P0_8 respectively.");
#else
    printf_string(UART, "\n\rif you attach a buzzer on pins P0_2 or P0_3 respectively.");
#endif
    printf_string(UART, "\n\rPlaying melody. Please wait...\n\r");

    // start pwm0
    timer0_start();

    while (timer0_pwm_test_expiration_counter);

    timer0_stop();

    // Disable the Timer0/Timer2 input clock
    timer0_2_clk_disable();

    printf_string(UART, "\n\rTIMER0 stopped!\n\r");
    printf_string(UART, "\n\rEnd of test\n\r");
}
