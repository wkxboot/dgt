/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_debug_console.h"
#include "board.h"

#if defined(FSL_FEATURE_SYSCON_HAS_PINT_SEL_REGISTER) && FSL_FEATURE_SYSCON_HAS_PINT_SEL_REGISTER
#include "fsl_syscon.h"
#else
#include "fsl_inputmux.h"
#endif /* FSL_FEATURE_SYSCON_HAS_PINT_SEL_REGISTER */
#include "fsl_pint.h"

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_PINT_PIN_INT0_SRC kSYSCON_GpioPort0Pin4ToPintsel
#define DEMO_PINT_PIN_INT1_SRC kSYSCON_GpioPort0Pin12ToPintsel
#define DEMO_PINT_PIN_INT2_SRC kSYSCON_GpioPort0Pin0ToPintsel

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
* @brief Call back for PINT Pin interrupt 0-7.
*/
void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    PRINTF("\f\r\nPINT Pin Interrupt %d event detected.", pintr);
}

/*!
* @brief Main function
*/
int main(void)
{
    /* Board pin, clock, debug console init */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 1U);

    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();

    /* Clear screen*/
    PRINTF("%c[2J", 27);
    /* Set cursor location at [0,0] */
    PRINTF("%c[0;0H", 27);
    PRINTF("\f\r\nPINT Pin interrupt example\r\n");

#if defined(FSL_FEATURE_SYSCON_HAS_PINT_SEL_REGISTER) && FSL_FEATURE_SYSCON_HAS_PINT_SEL_REGISTER
    /* Connect trigger sources to PINT */
    SYSCON_AttachSignal(SYSCON, kPINT_PinInt0, DEMO_PINT_PIN_INT0_SRC);
    SYSCON_AttachSignal(SYSCON, kPINT_PinInt1, DEMO_PINT_PIN_INT1_SRC);
#else
    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, DEMO_PINT_PIN_INT0_SRC);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt1, DEMO_PINT_PIN_INT1_SRC);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt2, DEMO_PINT_PIN_INT2_SRC);

    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);
#endif /* FSL_FEATURE_SYSCON_HAS_PINT_SEL_REGISTER */

    /* Initialize PINT */
    PINT_Init(PINT);

    /* Setup Pin Interrupt 0 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableRiseEdge, pint_intr_callback);

    /* Setup Pin Interrupt 1 for falling edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt1, kPINT_PinIntEnableFallEdge, pint_intr_callback);

    /* Setup Pin Interrupt 2 for both rising and falling edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt2, kPINT_PinIntEnableBothEdges, pint_intr_callback);

    /* Enable callbacks for PINT */
    PINT_EnableCallback(PINT);

    PRINTF("\r\nPINT Pin Interrupt events are configured\r\n");
    PRINTF("\r\nPress corresponding switches to generate events\r\n");
    while (1)
    {
        __WFI();
    }
}
