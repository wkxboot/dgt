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
#include "fsl_sctimer.h"

#include "pin_mux.h"
#include "fsl_inputmux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SCTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_Irc)
#define DEMO_FIRST_SCTIMER_OUT kSCTIMER_Out_4
#define DEMO_SECOND_SCTIMER_OUT kSCTIMER_Out_2

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    sctimer_config_t sctimerInfo;
    sctimer_pwm_signal_param_t pwmParam;
    uint32_t stateNumber;
    uint32_t eventFirstNumberOutput, eventSecondNumberOutput, eventNumberInput;
    uint32_t sctimerClock;

    /* Board pin, clock, debug console init */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 1U);
    
    /* configure the input mux for the sct timer input1 from external pin*/
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, 1U, kINPUTMUX_SctPin0ToSct0);

    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();

    /* Enable clock of sct. */
    CLOCK_EnableClock(kCLOCK_Sct);

    sctimerClock = SCTIMER_CLK_FREQ;

    /* Print a note to terminal */
    PRINTF("\r\nSCTimer example to output edge-aligned PWM signal\r\n");
    PRINTF("\r\nWhen user presses a switch the PWM signal will be seen from Out 2  ");
    PRINTF("\r\nWhen user presses the switch again PWM signal on Out 2 will turn off ");
    PRINTF("\r\nThe PWM signal from Out 4 will remain active all the time ");

    /* Default configuration operates the counter in 32-bit mode */
    SCTIMER_GetDefaultConfig(&sctimerInfo);

    /* Initialize SCTimer module */
    SCTIMER_Init(SCT0, &sctimerInfo);

    stateNumber = SCTIMER_GetCurrentState(SCT0);

    /* Configure PWM params with frequency 24kHZ from first output */
    pwmParam.output = DEMO_FIRST_SCTIMER_OUT;
    pwmParam.level = kSCTIMER_HighTrue;
    pwmParam.dutyCyclePercent = 10;

    /* Schedule events in current state; State 0 */
    /* Schedule events for generating a 24KHz PWM with 10% duty cycle from first Out in the current state */
    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm, 24000U, sctimerClock, &eventFirstNumberOutput) ==
        kStatus_Fail)
    {
        return -1;
    }

    /* Schedule an event to look for a rising edge on input 1 in this state */
    if (SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputRiseEvent, 0, kSCTIMER_Input_1, kSCTIMER_Counter_L,
                                       &eventNumberInput) == kStatus_Fail)
    {
        return -1;
    }

    /* Transition to next state when a rising edge is detected on input 1 */
    SCTIMER_SetupNextStateAction(SCT0, stateNumber + 1, eventNumberInput);

    /* Go to next state; State 1 */
    SCTIMER_IncreaseState(SCT0);

    /* Schedule events in State 1 */
    /* Schedule events for generating a 24KHz PWM with 50% duty cycle from second Out in this new state */
    pwmParam.output = DEMO_SECOND_SCTIMER_OUT;
    pwmParam.dutyCyclePercent = 50;
    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm, 24000U, sctimerClock, &eventSecondNumberOutput) ==
        kStatus_Fail)
    {
        return -1;
    }

    /* Re-enable PWM coming out from Out 4 by scheduling the PWM events in this new state */
    /* To get a PWM, the SCTIMER_SetupPwm() function creates 2 events; 1 for the pulse period and
     * and 1 for the pulse, we need to schedule both events in this new state
     */
    /* Schedule the period event for the PWM */
    SCTIMER_ScheduleEvent(SCT0, eventFirstNumberOutput);
    /* Schedule the pulse event for the PWM */
    SCTIMER_ScheduleEvent(SCT0, eventFirstNumberOutput + 1);

    /* Schedule an event to look for a rising edge on input 1 in this state */
    if (SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_InputRiseEvent, 0, kSCTIMER_Input_1, kSCTIMER_Counter_L,
                                       &eventNumberInput) == kStatus_Fail)
    {
        return -1;
    }

    /* Transition back to State 0 when a rising edge is detected on input 1 */
    /* State 0 has only 1 PWM active, there will be no PWM from Out 2 */
    SCTIMER_SetupNextStateAction(SCT0, stateNumber, eventNumberInput);

    /* Start the timer, use counter L as we are operating counter in 32-bit mode */
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);

    while (1)
    {
    }
}
