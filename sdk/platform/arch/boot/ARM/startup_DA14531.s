;/**************************************************************************//**
; * @file     startup_DA14531.s
; * @brief    CMSIS Cortex-M ARMv7-M based Core Device Startup File for
; *           Device DA14531
; * @version  V5.00
; * @date     02. March 2016
; ******************************************************************************/
;/*
; * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
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
; *
; * Copyright (C) 2019-2020 Modified by Dialog Semiconductor
; */
;/*

;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/

                IF      :DEF:CFG_STATEFUL_HIBERNATION
; DA14531 register definition needed below
HIBERN_CTRL_REG EQU 0x50000310
                ENDIF

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000600

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000100

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                IF      :DEF:CFG_STATEFUL_HIBERNATION
; Added reserve space for the ARM core register dump

                AREA    stateful_hibernation, NOINIT, READWRITE, ALIGN=2
; Reserve space for stacking, in case an exception occurs while restoring the state
safe_stack_state    SPACE   32

stored_PRIMASK  ; reserve space for interrupt mask register (PRIMASK)
                SPACE 4
stored_R4       ; reserve space for R4, R5, R6, R7, R8, R9, R10, R11, R12, R14
                SPACE 40
stored_MSP          ; reserve space for MSP and PSP and control
                SPACE 12
stored_ISER         ; reserve space for NVIC->ISER, NVIC->IPR and SCB->SCR
                SPACE 68

hiber_magic_word    SPACE 4
                ENDIF

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

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
                DCD     SWTIM1_Handler
                DCD     RTC_Handler
                DCD     DMA_Handler
                DCD     XTAL32M_RDY_Handler
                DCD     RESERVED21_Handler
                DCD     RESERVED22_Handler
                DCD     RESERVED23_Handler
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

                IF      :DEF:CFG_STATEFUL_HIBERNATION

; Function to store PRIMASK
_store_primask  PROC
                EXPORT  _store_primask          [WEAK]
                ; Save PRIMASK to start of stored_PRIMASK and disable IRQ
                ldr     r3, =stored_PRIMASK
                mrs     r0, PRIMASK
                cpsid   i
                stm     r3!, {r0}
                bx      lr
                ENDP

; Function to store the ARM core state and let the system go to stateful
; hibernation
_store_core_registers   PROC
                EXPORT  _store_core_registers   [WEAK]
                ; store R4-R12, LR
                ldr     r3, =stored_R4
                stm     r3!, {r4-r7}    ; save R4-R7
                mov     R4, R8
                mov     R5, R9
                mov     R6, R10
                mov     R7, R11
                stm     r3!, {r4-r7}    ; save R8-R11

                mov     R4, R12
                mov     R5, R14
                stm     r3!, {r4-r5}    ; save R12, LR (R14)

                ; Store MSP, PSP, CONTROL
                ldr     r3, =stored_MSP
                mrs     r0, msp
                mrs     r1, psp
                mrs     r2, control
                stm     r3!, {r0-r2}

                ; Store NVIC->ISER[0]
                ldr     r3, =stored_ISER
                ldr     r0, =0xE000E100
                ldm     r0!, {r4}
                stm     r3!, {r4}
                ; Disable all interrupts in NVIC - NVIC->ICER[0U] = 0xFFFFFFFFUL
                ldr     r0, =0xE000E180
                ldr     r1, =0xFFFFFFFF
                str     r1, [r0]
                ; Clear all pending interrupts in NVIC - NVIC->ICPR[0U] = 0xFFFFFFFFUL
                ldr     r0, =0xE000E280
                ldr     r1, =0xFFFFFFFF
                str     r1, [r0]

                ; Save values of NVIC->IP[0..7] - for DA14531 we only have 8
                ; (since we have 21 IRQs)
                ; They fit in 8 32-bit registers
                ldr     r0, =0xE000E400         ; NVIC->IPR[] base address
                ldm     r0!, {r4-r7}
                stm     r3!, {r4-r7}
                ldm     r0!, {r4-r7}
                stm     r3!, {r4-r7}

                ldr     r0, =0xE000ED00         ; address of SCB

                ldr     r4, [r0, #0x04]         ; value of SCB->ICSR
                stm     r3!, {r4}

                ldr     r4, [r0, #0x10]         ; value of SCB->SCR
                ldr     r5, [r0, #0x14]         ; value of SCB->CCR
                ldr     r6, [r0, #0x1C]         ; value of SCB->SHPR2
                ldr     r7, [r0, #0x20]         ; value of SCB->SHPR3
                stm     r3!, {r4-r7}

                mov     r9, r0                  ; save address of SCB for later
                mov     r10, r4                 ; save value of SCB->SCR for later

                ; Set SCR.SLEEPDEEP
                ldr     r1, =0xE000ED00         ; address of SCB
                mov     r0, r4                  ; r4 = saved SCB->SCR value
                movs    r2, #4
                orrs    r0, r0, r2
                str     r0, [r1, #0x10]         ; r9 = SCB start address

                movs    r1, #0xA5
                ldr     r2, =hiber_magic_word
                str     r1, [r2]

                nop
                nop
                nop

                wfi

                ; we reach this point if device didn't actually go to stateful
                ; hibernation

                ; Restore SCR.SLEEPDEEP
                str     r4, [r1, #0x10]

                ; Return 0 to the caller of _store_core_registers()
                movs    r0, #0
                b       restore_system_state
                ENDP

; Function to restore the system state
restore_system_state PROC
                IMPORT arch_hibernation_restore
                ; disable interrupts, final state will be set when PRIMASK is restored
                cpsid   i

                ; set SP to a safe value, we have reserved some space for this case
                ldr     r0, =safe_stack_state
                mov     sp, r0


                ; restore values of NVIC->ISER, NVIC->PRIORITY{0,1,2},
                ; SCB->SCR, SCB->SHR[0] (SHPR2) and SCB->SHR[1] (SHPR3)

                ldr     r0, =stored_ISER

                ; Restore NVIC->ISER[0]  - for DA14531 we only have one
                ldm     r0!, {r4}
                ldr     r1, =0xE000E100         ; address of NVIC->ISER[0]
                stm     r1!, {r4}

                ; Restore values of NVIC->IPR[0..7] - we only have 8 IRQs for DA14531 which fit in 6 registers
                ldm     r0!, {r4-r7}
                ldr     r1, =0xE000E400         ; NVIC->IPR[] base address
                stm     r1!, {r4-r7}
                ldm     r0!, {r4-r7}
                ldr     r1, =0xE000E410         ; NVIC->IPR[] base address
                stm     r1!, {r4-r7}

                ; Restore
                ; SCB->SCR
                ; SCB->CCR
                ; SCB->SHPR2/3
                ldm     r0!, {r4}
                ldr     r1, =0xE000ED00

                ldr     r4, [r1, #0x04]
                ldm     r0!, {r4-r7}

                str     r4, [r1, #0x10]         ; SCB->SCR
                str     r5, [r1, #0x14]         ; SCB->CCR
                str     r6, [r1, #0x1C]         ; SCB->SHPR2
                str     r7, [r1, #0x20]         ; SCB->SHPR3

                ; restore MSP, PSP, CONTROL
                ldr     r0, =stored_MSP
                ldm     r0!, {r1-r3}
                msr     msp, r1
                msr     psp, r2
                msr     control, r3

                ; Return 1 to the caller of restore_system_state()
                movs    r0, #1
                LDR     R0, =arch_hibernation_restore
                BLX     R0

                ; Restore R4-R11, 8 32-bit registers in total
                ldr     r1, =stored_R4
                movs    r2, #0x10
                add     r1, r1, r2
                ldm     r1!, {r4-r7} ; get R8-R11
                mov     r8, r4
                mov     r9, r5
                mov     r10, r6
                mov     r11, r7

                ldr     r1, =stored_R4
                ldm     r1!, {r4-r7} ; get R4-R7

                ; Restore PRIMASK
                ldr     r1, =stored_PRIMASK
                ldr     r2, [r1]
                msr     PRIMASK, r2

                ; Restore R12 and LR
                ldr     r1, =stored_R4
                movs    r2, #0x20
                add     r1, r1, r2
                ldm     r1!, {r4-r5} ; get R12 and LR
                mov     r12, r4
                mov     r14, r5
                bx      lr
                ENDP

                ENDIF

; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                IF      :DEF:CFG_STATEFUL_HIBERNATION
                ; check HIBERN_CTRL_REG
                movs    r3, #1
                ldr     r1, =HIBERN_CTRL_REG
                ldrh    r2, [r1]
                ands    r2, r2, r3
                CMP     r2, #1
                bne     cold_reset

                ; if hiber_magic_word is equal to 0xA5, it means system woke up
                ; from stateful hibernation
                ldr     r1, =hiber_magic_word
                ldr     r2, [r1]

                ; reset hiber_magic_word
                movs    r3, #0
                str     r3, [r1]

                cmps    r2, #0xA5
                bne     cold_reset

                ldr     r3, =restore_system_state
                bx      r3
cold_reset
                ; Initialize hiber_magic_word to 0 on cold reset
                ldr     r1, =hiber_magic_word
                ldr     r2, [r1]

                ; reset hiber_magic_word
                movs    r3, #0
                str     r3, [r1]
                ENDIF

                ; normal reset behavior
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)
                IMPORT NMI_HandlerC
NMI_Handler\
                PROC
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
                ENDP

                IMPORT HardFault_HandlerC
HardFault_Handler\
                PROC
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
                ENDP

SVC_Handler\
                PROC
                EXPORT  SVC_Handler            [WEAK]
                B       .
                ENDP

PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler         [WEAK]
                B       .
                ENDP

SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler        [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT BLE_WAKEUP_LP_Handler   [WEAK]
                EXPORT rwble_isr               [WEAK]
                EXPORT UART_Handler            [WEAK]
                EXPORT UART2_Handler           [WEAK]
                EXPORT I2C_Handler             [WEAK]
                EXPORT SPI_Handler             [WEAK]
                EXPORT ADC_Handler             [WEAK]
                EXPORT KEYBRD_Handler          [WEAK]
                EXPORT BLE_RF_DIAG_Handler     [WEAK]
                EXPORT RFCAL_Handler           [WEAK]
                EXPORT GPIO0_Handler           [WEAK]
                EXPORT GPIO1_Handler           [WEAK]
                EXPORT GPIO2_Handler           [WEAK]
                EXPORT GPIO3_Handler           [WEAK]
                EXPORT GPIO4_Handler           [WEAK]
                EXPORT SWTIM_Handler           [WEAK]
                EXPORT WKUP_QUADEC_Handler     [WEAK]
                EXPORT SWTIM1_Handler          [WEAK]
                EXPORT RTC_Handler             [WEAK]
                EXPORT DMA_Handler             [WEAK]
                EXPORT XTAL32M_RDY_Handler     [WEAK]
                EXPORT RESERVED21_Handler      [WEAK]
                EXPORT RESERVED22_Handler      [WEAK]
                EXPORT RESERVED23_Handler      [WEAK]

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
SWTIM1_Handler
RTC_Handler
DMA_Handler
XTAL32M_RDY_Handler
RESERVED21_Handler
RESERVED22_Handler
RESERVED23_Handler
                B       .
                ENDP

                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
