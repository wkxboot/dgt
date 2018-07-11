/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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

#include "fsl_acomp.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "fsl_power.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ACOMP ACOMP
#define DEMO_ACOMP_POSITIVE_INPUT 0U /* Voltage ladder output. */
#define DEMO_ACOMP_NEGATIVE_INPUT 3U /* ACMP_I3. */
#define DEMO_ACOMP_IRQ_NUMBER CMP_CAPT_IRQn
#define DEMO_ACOMP_IRQ_HANDLER_FUNC CMP_CAPT_IRQHandler
#define BOARD_LED_PORT 0U
#define BOARD_LED_PIN 12U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void DEMO_ACOMP_IRQ_HANDLER_FUNC(void)
{
    ACOMP_ClearInterruptsStatusFlags(DEMO_ACOMP);
    /* Check the comparison result and sets the LED state according to the result.*/
    if (ACOMP_GetOutputStatusFlags(DEMO_ACOMP))
    {
        LED_RED_ON();
    }
    else
    {
        LED_RED_OFF();
    }
}
/*!
 * @brief Main function
 */
int main(void)
{
    acomp_config_t acompConfigStruct;
    acomp_ladder_config_t ladderConfigStruct;

    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    

    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();
    /* Power on ACMP. */
    POWER_DisablePD(kPDRUNCFG_PD_ACMP);

    PRINTF("\r\nLPC_ACOMP Interrupt Example.\r\n");

    /* Init output LED GPIO. */
    LED_RED_INIT(LOGIC_LED_ON);

    /* Initialize ACOMP module. */
    ACOMP_GetDefaultConfig(&acompConfigStruct);
    ACOMP_Init(DEMO_ACOMP, &acompConfigStruct);

    /* Enable interrupt. */
    ACOMP_EnableInterrupts(DEMO_ACOMP, kACOMP_InterruptsBothEdgesEnable);
    NVIC_EnableIRQ(DEMO_ACOMP_IRQ_NUMBER);
    /* Configure ACOMP negative and positive input channels. */
    ACOMP_SetInputChannel(DEMO_ACOMP, DEMO_ACOMP_POSITIVE_INPUT, DEMO_ACOMP_NEGATIVE_INPUT);

    /* Configure internal voltage ladder. */
    ladderConfigStruct.ladderValue = 15U;                                /* Half of reference voltage. */
    ladderConfigStruct.referenceVoltage = kACOMP_LadderRefVoltagePinVDD; /* VDDA as the reference voltage. */
    ACOMP_SetLadderConfig(DEMO_ACOMP, &ladderConfigStruct);

    PRINTF("The example compares analog input to the voltage ladder output(ACOMP negative port).\r\n");
    PRINTF("The LED will be turned ON/OFF when the analog input is LOWER/HIGHER than the ladder's output.\r\n");
    PRINTF("Change the analog input voltage to see the LED status.\r\n");

    /* Check the comparison result and sets the LED state according to the result.*/
    if (ACOMP_GetOutputStatusFlags(DEMO_ACOMP))
    {
        LED_RED_ON();
    }
    else
    {
        LED_RED_OFF();
    }

    while (1)
    {
    }
}
