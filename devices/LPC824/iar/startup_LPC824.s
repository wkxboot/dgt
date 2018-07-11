;/*****************************************************************************
; * @file:    startup_LPC824.s
; * @purpose: CMSIS Cortex-M0 Core Device Startup File
; *           LPC824
; * @version: 1.2
; * @date:    2017-6-8
; *----------------------------------------------------------------------------
; *
; The Clear BSD License
; Copyright 1997-2016 Freescale Semiconductor, Inc.
; Copyright 2016-2018 NXP
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted (subject to the limitations in the
; disclaimer below) provided that the following conditions are met:
;
; * Redistributions of source code must retain the above copyright
;   notice, this list of conditions and the following disclaimer.
;
; * Redistributions in binary form must reproduce the above copyright
;   notice, this list of conditions and the following disclaimer in the
;   documentation and/or other materials provided with the distribution.
;
; * Neither the name of the copyright holder nor the names of its
;   contributors may be used to endorse or promote products derived from
;   this software without specific prior written permission.
;
; NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
; GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
; HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
; MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
; BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
; WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
; OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
; IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     0
        DCD     0
        DCD     0
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     0
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     SPI0_IRQHandler  ; SPI0 interrupt
        DCD     SPI1_IRQHandler  ; SPI1 interrupt
        DCD     Reserved18_IRQHandler  ; Reserved interrupt
        DCD     USART0_IRQHandler  ; USART0 interrupt
        DCD     USART1_IRQHandler  ; USART1 interrupt
        DCD     USART2_IRQHandler  ; USART2 interrupt
        DCD     Reserved22_IRQHandler  ; Reserved interrupt
        DCD     I2C1_IRQHandler  ; I2C1 interrupt
        DCD     I2C0_IRQHandler  ; I2C0 interrupt
        DCD     SCT0_IRQHandler  ; State configurable timer interrupt
        DCD     MRT0_IRQHandler  ; Multi-rate timer interrupt
        DCD     CMP_CAPT_IRQHandler  ; Analog comparator interrupt or Capacitive Touch interrupt
        DCD     WDT_IRQHandler  ; Windowed watchdog timer interrupt
        DCD     BOD_IRQHandler  ; BOD interrupts
        DCD     FLASH_IRQHandler  ; flash interrupt
        DCD     WKT_IRQHandler  ; Self-wake-up timer interrupt
        DCD     ADC0_SEQA_IRQHandler  ; ADC0 sequence A completion.
        DCD     ADC0_SEQB_IRQHandler  ; ADC0 sequence B completion.
        DCD     ADC0_THCMP_IRQHandler  ; ADC0 threshold compare and error.
        DCD     ADC0_OVR_IRQHandler  ; ADC0 overrun
        DCD     DMA0_IRQHandler  ; DMA0 interrupt
        DCD     I2C2_IRQHandler  ; I2C2 interrupt
        DCD     I2C3_IRQHandler  ; I2C3 interrupt
        DCD     Reserved39_IRQHandler  ; Reserved interrupt
        DCD     PIN_INT0_IRQHandler  ; Pin interrupt 0 or pattern match engine slice 0 interrupt
        DCD     PIN_INT1_IRQHandler  ; Pin interrupt 1 or pattern match engine slice 1 interrupt
        DCD     PIN_INT2_IRQHandler  ; Pin interrupt 2 or pattern match engine slice 2 interrupt
        DCD     PIN_INT3_IRQHandler  ; Pin interrupt 3 or pattern match engine slice 3 interrupt
        DCD     PIN_INT4_IRQHandler  ; Pin interrupt 4 or pattern match engine slice 4 interrupt
        DCD     PIN_INT5_IRQHandler  ; Pin interrupt 5 or pattern match engine slice 5 interrupt
        DCD     PIN_INT6_IRQHandler  ; Pin interrupt 6 or pattern match engine slice 6 interrupt
        DCD     PIN_INT7_IRQHandler  ; Pin interrupt 7 or pattern match engine slice 7 interrupt
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size 	EQU 	__Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        LDR     r0, =SystemInit
        BLX     r0
        LDR     r0, =__iar_program_start
        BX      r0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B .

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B .

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B .

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B .

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B .

        PUBWEAK SPI0_IRQHandler
        PUBWEAK SPI0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SPI0_IRQHandler
        LDR     R0, =SPI0_DriverIRQHandler
        BX      R0
        PUBWEAK SPI1_IRQHandler
        PUBWEAK SPI1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SPI1_IRQHandler
        LDR     R0, =SPI1_DriverIRQHandler
        BX      R0
        PUBWEAK Reserved18_IRQHandler
        PUBWEAK Reserved18_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reserved18_IRQHandler
        LDR     R0, =Reserved18_DriverIRQHandler
        BX      R0
        PUBWEAK USART0_IRQHandler
        PUBWEAK USART0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USART0_IRQHandler
        LDR     R0, =USART0_DriverIRQHandler
        BX      R0
        PUBWEAK USART1_IRQHandler
        PUBWEAK USART1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USART1_IRQHandler
        LDR     R0, =USART1_DriverIRQHandler
        BX      R0
        PUBWEAK USART2_IRQHandler
        PUBWEAK USART2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USART2_IRQHandler
        LDR     R0, =USART2_DriverIRQHandler
        BX      R0
        PUBWEAK Reserved22_IRQHandler
        PUBWEAK Reserved22_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reserved22_IRQHandler
        LDR     R0, =Reserved22_DriverIRQHandler
        BX      R0
        PUBWEAK I2C1_IRQHandler
        PUBWEAK I2C1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
I2C1_IRQHandler
        LDR     R0, =I2C1_DriverIRQHandler
        BX      R0
        PUBWEAK I2C0_IRQHandler
        PUBWEAK I2C0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
I2C0_IRQHandler
        LDR     R0, =I2C0_DriverIRQHandler
        BX      R0
        PUBWEAK SCT0_IRQHandler
        PUBWEAK SCT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SCT0_IRQHandler
        LDR     R0, =SCT0_DriverIRQHandler
        BX      R0
        PUBWEAK MRT0_IRQHandler
        PUBWEAK MRT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
MRT0_IRQHandler
        LDR     R0, =MRT0_DriverIRQHandler
        BX      R0
        PUBWEAK CMP_CAPT_IRQHandler
        PUBWEAK CMP_CAPT_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CMP_CAPT_IRQHandler
        LDR     R0, =CMP_CAPT_DriverIRQHandler
        BX      R0
        PUBWEAK WDT_IRQHandler
        PUBWEAK WDT_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
WDT_IRQHandler
        LDR     R0, =WDT_DriverIRQHandler
        BX      R0
        PUBWEAK BOD_IRQHandler
        PUBWEAK BOD_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
BOD_IRQHandler
        LDR     R0, =BOD_DriverIRQHandler
        BX      R0
        PUBWEAK FLASH_IRQHandler
        PUBWEAK FLASH_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLASH_IRQHandler
        LDR     R0, =FLASH_DriverIRQHandler
        BX      R0
        PUBWEAK WKT_IRQHandler
        PUBWEAK WKT_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
WKT_IRQHandler
        LDR     R0, =WKT_DriverIRQHandler
        BX      R0
        PUBWEAK ADC0_SEQA_IRQHandler
        PUBWEAK ADC0_SEQA_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ADC0_SEQA_IRQHandler
        LDR     R0, =ADC0_SEQA_DriverIRQHandler
        BX      R0
        PUBWEAK ADC0_SEQB_IRQHandler
        PUBWEAK ADC0_SEQB_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ADC0_SEQB_IRQHandler
        LDR     R0, =ADC0_SEQB_DriverIRQHandler
        BX      R0
        PUBWEAK ADC0_THCMP_IRQHandler
        PUBWEAK ADC0_THCMP_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ADC0_THCMP_IRQHandler
        LDR     R0, =ADC0_THCMP_DriverIRQHandler
        BX      R0
        PUBWEAK ADC0_OVR_IRQHandler
        PUBWEAK ADC0_OVR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ADC0_OVR_IRQHandler
        LDR     R0, =ADC0_OVR_DriverIRQHandler
        BX      R0
        PUBWEAK DMA0_IRQHandler
        PUBWEAK DMA0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA0_IRQHandler
        LDR     R0, =DMA0_DriverIRQHandler
        BX      R0
        PUBWEAK I2C2_IRQHandler
        PUBWEAK I2C2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
I2C2_IRQHandler
        LDR     R0, =I2C2_DriverIRQHandler
        BX      R0
        PUBWEAK I2C3_IRQHandler
        PUBWEAK I2C3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
I2C3_IRQHandler
        LDR     R0, =I2C3_DriverIRQHandler
        BX      R0
        PUBWEAK Reserved39_IRQHandler
        PUBWEAK Reserved39_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reserved39_IRQHandler
        LDR     R0, =Reserved39_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT0_IRQHandler
        PUBWEAK PIN_INT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT0_IRQHandler
        LDR     R0, =PIN_INT0_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT1_IRQHandler
        PUBWEAK PIN_INT1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT1_IRQHandler
        LDR     R0, =PIN_INT1_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT2_IRQHandler
        PUBWEAK PIN_INT2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT2_IRQHandler
        LDR     R0, =PIN_INT2_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT3_IRQHandler
        PUBWEAK PIN_INT3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT3_IRQHandler
        LDR     R0, =PIN_INT3_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT4_IRQHandler
        PUBWEAK PIN_INT4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT4_IRQHandler
        LDR     R0, =PIN_INT4_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT5_IRQHandler
        PUBWEAK PIN_INT5_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT5_IRQHandler
        LDR     R0, =PIN_INT5_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT6_IRQHandler
        PUBWEAK PIN_INT6_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT6_IRQHandler
        LDR     R0, =PIN_INT6_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT7_IRQHandler
        PUBWEAK PIN_INT7_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT7_IRQHandler
        LDR     R0, =PIN_INT7_DriverIRQHandler
        BX      R0
SPI0_DriverIRQHandler
SPI1_DriverIRQHandler
Reserved18_DriverIRQHandler
USART0_DriverIRQHandler
USART1_DriverIRQHandler
USART2_DriverIRQHandler
Reserved22_DriverIRQHandler
I2C1_DriverIRQHandler
I2C0_DriverIRQHandler
SCT0_DriverIRQHandler
MRT0_DriverIRQHandler
CMP_CAPT_DriverIRQHandler
WDT_DriverIRQHandler
BOD_DriverIRQHandler
FLASH_DriverIRQHandler
WKT_DriverIRQHandler
ADC0_SEQA_DriverIRQHandler
ADC0_SEQB_DriverIRQHandler
ADC0_THCMP_DriverIRQHandler
ADC0_OVR_DriverIRQHandler
DMA0_DriverIRQHandler
I2C2_DriverIRQHandler
I2C3_DriverIRQHandler
Reserved39_DriverIRQHandler
PIN_INT0_DriverIRQHandler
PIN_INT1_DriverIRQHandler
PIN_INT2_DriverIRQHandler
PIN_INT3_DriverIRQHandler
PIN_INT4_DriverIRQHandler
PIN_INT5_DriverIRQHandler
PIN_INT6_DriverIRQHandler
PIN_INT7_DriverIRQHandler
DefaultISR
        B .

        END
