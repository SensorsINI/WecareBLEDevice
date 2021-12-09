;/**************************************************************************//**
; * @file     startup_DA14585_586.s
; * @brief    CMSIS Cortex-M0 Core Device Startup File for
; *           Device DA14585/586
; * @version  V5.3.1
; * @date     09. July 2018
; ******************************************************************************/
;/*
; * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; */

;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;
; Copyright (C) 2019-2020 Modified by Dialog Semiconductor
;

                MODULE   ?cstartup

                ;; Forward declaration of sections.
                SECTION  CSTACK:DATA:NOROOT(3)

                SECTION  .intvec:CODE:NOROOT(2)

                EXTERN   __iar_program_start
                EXTERN   SystemInit
                PUBLIC   __vector_table
                PUBLIC   __vector_table_0x1c
                PUBLIC   __Vectors
                PUBLIC   __Vectors_End
                PUBLIC   __Vectors_Size

                DATA

__vector_table
                DCD      sfe(CSTACK)                         ;     Top of Stack
                DCD      Reset_Handler                       ;     Reset Handler
                DCD      NMI_Handler                         ; -14 NMI Handler
                DCD      HardFault_Handler                   ; -13 Hard Fault Handler
                DCD      MemManage_Handler                   ; -12 MPU Fault Handler
                DCD      BusFault_Handler                    ; -11 Bus Fault Handler
                DCD      UsageFault_Handler                  ; -10 Usage Fault Handler
__vector_table_0x1c
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      SVC_Handler                         ;  -5 SVCall Handler
                DCD      DebugMon_Handler                    ;  -4 Debug Monitor Handler
                DCD      0                                   ;     Reserved
                DCD      PendSV_Handler                      ;  -2 PendSV Handler
                DCD      SysTick_Handler                     ;  -1 SysTick Handler

        ; External Interrupts
                DCD     BLE_WAKEUP_LP_Handler
                DCD     rwble_isr
                DCD     UART_Handler
                DCD     UART2_Handler
                DCD     I2C_Handler
                DCD     SPI_Handler
                DCD     ADC_Handler
                DCD     KEYBRD_Handler
                DCD     BLE_RF_DIAG_Handler
                DCD     RFCAL_Handler
                DCD     GPIO0_Handler
                DCD     GPIO1_Handler
                DCD     GPIO2_Handler
                DCD     GPIO3_Handler
                DCD     GPIO4_Handler
                DCD     SWTIM_Handler
                DCD     WKUP_QUADEC_Handler
                DCD     PCM_Handler
                DCD     SRCIN_Handler
                DCD     SRCOUT_Handler
                DCD     DMA_Handler
                DCD     RESERVED21_Handler
                DCD     RESERVED22_Handler
                DCD     RESERVED23_Handler
__Vectors_End

__Vectors       EQU      __vector_table
__Vectors_Size  EQU      __Vectors_End - __Vectors


                THUMB

; Reset Handler

                PUBWEAK  Reset_Handler
                SECTION  .text:CODE:REORDER:NOROOT(2)
Reset_Handler
                LDR      R0, =SystemInit
                BLX      R0
                LDR      R0, =__iar_program_start
                BX       R0

NMI_Handler
                IMPORT NMI_HandlerC
                movs r0, #4
                mov r1, lr
                tst r0, r1
                beq NMI_stacking_used_MSP
                mrs r0, psp
                ldr r1,=NMI_HandlerC
                bx r1
NMI_stacking_used_MSP
                mrs r0, msp
                ldr r1,=NMI_HandlerC
                bx r1

HardFault_Handler
                IMPORT HardFault_HandlerC
                movs r0, #4
                mov r1, lr
                tst r0, r1
                beq HardFault_stacking_used_MSP
                mrs r0, psp
                ldr r1,=HardFault_HandlerC
                bx r1
HardFault_stacking_used_MSP
                mrs r0, msp
                ldr r1,=HardFault_HandlerC
                bx r1


                PUBWEAK NMI_Handler
                PUBWEAK HardFault_Handler
                PUBWEAK MemManage_Handler
                PUBWEAK BusFault_Handler
                PUBWEAK UsageFault_Handler
                PUBWEAK SVC_Handler
                PUBWEAK DebugMon_Handler
                PUBWEAK PendSV_Handler
                PUBWEAK SysTick_Handler

                PUBWEAK BLE_WAKEUP_LP_Handler
                PUBWEAK rwble_isr
                PUBWEAK UART_Handler
                PUBWEAK UART2_Handler
                PUBWEAK I2C_Handler
                PUBWEAK SPI_Handler
                PUBWEAK ADC_Handler
                PUBWEAK KEYBRD_Handler
                PUBWEAK BLE_RF_DIAG_Handler
                PUBWEAK RFCAL_Handler
                PUBWEAK GPIO0_Handler
                PUBWEAK GPIO1_Handler
                PUBWEAK GPIO2_Handler
                PUBWEAK GPIO3_Handler
                PUBWEAK GPIO4_Handler
                PUBWEAK SWTIM_Handler
                PUBWEAK WKUP_QUADEC_Handler
                PUBWEAK PCM_Handler
                PUBWEAK SRCIN_Handler
                PUBWEAK SRCOUT_Handler
                PUBWEAK DMA_Handler
                PUBWEAK RESERVED21_Handler
                PUBWEAK RESERVED22_Handler
                PUBWEAK RESERVED23_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
HardFault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
SVC_Handler
DebugMon_Handler
PendSV_Handler
SysTick_Handler

BLE_WAKEUP_LP_Handler
rwble_isr
UART_Handler
UART2_Handler
I2C_Handler
SPI_Handler
ADC_Handler
KEYBRD_Handler
BLE_RF_DIAG_Handler
RFCAL_Handler
GPIO0_Handler
GPIO1_Handler
GPIO2_Handler
GPIO3_Handler
GPIO4_Handler
SWTIM_Handler
WKUP_QUADEC_Handler
PCM_Handler
SRCIN_Handler
SRCOUT_Handler
DMA_Handler
RESERVED21_Handler
RESERVED22_Handler
RESERVED23_Handler
                B        .


                END
