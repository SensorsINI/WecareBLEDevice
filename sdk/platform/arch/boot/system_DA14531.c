/**************************************************************************//**
 * @file     system_DA14531.c
 * @brief    CMSIS Cortex-M# Device Peripheral Access Layer Source File for
 *           Device DA14531
 * @version  V5.00
 * @date     10. January 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Copyright (C) 2019-2021 Modified by Dialog Semiconductor
 */

#include <stdint.h>
#include "datasheet.h"
#include "otp_cs.h"
#include "arch.h"

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

#define __XTAL            (16000000UL)    /* Oscillator frequency             */

#define __SYSTEM_CLOCK    (__XTAL)


/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/

uint32_t SystemCoreClock = __SYSTEM_CLOCK;  /* System Clock Frequency (Core Clock)*/

/*----------------------------------------------------------------------------
  Structure containing info about debugger configuration and OTP CS registers
  normally handled by the Booter - defined in otp_cs.c
 *----------------------------------------------------------------------------*/
 extern otp_cs_booter_val_t booter_val;

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/

void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
    SystemCoreClock = __SYSTEM_CLOCK;
}

void SystemInit (void)
{
    // Enable the TLS (Transparent Light Sleep) core feature to minimize RAM
    // power consumption
    SetBits16(RAM_LPMX_REG, RAMx_LPMX, 0);

    // Size of booter_val shall not be greater than 0x20 bytes (= 0xC0 - 0xA0),
    // where 0xC0 is the maximum size of ER_IROM1 and 0xA0 the size of the
    // vector table residing in ER_IROM1 (see DA14531.sct for details)
    ASSERT_WARNING(sizeof(booter_val) <= 0x20)

    // Load certain registers whose values are written by the booter. When the
    // system wakes up from hibernation and the remap address 0 is set to RAM1
    // or RAM3 block, the booter is not invoked. Hence, SDK shall load those
    // registers. The RESET_STAT_REG (it belongs to AON domain, like the
    // HIBERN_CTRL_REG) is zeroed by the SDK in the main() function.
    if ((GetBits16(HIBERN_CTRL_REG, HIBERNATION_ENABLE) == 1) &&
        (GetWord16(RESET_STAT_REG) == 0))
    {
        // Apply the BANDGAP_REG value
        SetWord16(BANDGAP_REG, booter_val.bandgap_reg);

        // Apply the CLK_RC32M_REG value
        SetWord16(CLK_RC32M_REG, booter_val.clk_rc32m_reg);

        // Apply the CLK_RC32K_REG value
        SetWord16(CLK_RC32K_REG, booter_val.clk_rc32k_reg);

        // Apply the debugger configuration
        SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, booter_val.dbg_cfg);
    }

    SetWord16(TIMER0_CTRL_REG, 0x6);        // stop timer
    NVIC_DisableIRQ(SWTIM_IRQn);            // disable software timer interrupt

    NVIC_DisableIRQ(BLE_GEN_IRQn);
    NVIC_DisableIRQ(BLE_WAKEUP_LP_IRQn);

    // Ensure that XTAL32M will be enabled (switch to XTAL32M will be done later)
    SetBits16(CLK_CTRL_REG, XTAL32M_DISABLE, 0);

    // Set clocks to highest frequency (16MHz)
    SetWord16(CLK_AMBA_REG, 0x00);

    __enable_irq();
}
