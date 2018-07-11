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
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SCTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_Irc)
#define DEMO_FIRST_SCTIMER_OUT kSCTIMER_Out_2
#define DEMO_SECOND_SCTIMER_OUT kSCTIMER_Out_4

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
    uint32_t eventCounterL, eventCounterH;
    uint32_t sctimerClock;
    uint32_t matchValueL, matchValueH;

    /* Board pin, clock, debug console init */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 1U);

    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();

    /* Enable clock of sct. */
    CLOCK_EnableClock(kCLOCK_Sct);

    sctimerClock = SCTIMER_CLK_FREQ;

    /* Print a note to terminal */
    PRINTF("\r\nSCTimer example to use it in 16-bit mode\r\n");
    PRINTF("\r\nThe example shows both 16-bit counters running and toggling an output periodically  ");

    SCTIMER_GetDefaultConfig(&sctimerInfo);

    /* Switch to 16-bit mode */
    sctimerInfo.enableCounterUnify = false;

    /* Calculate prescaler and match value for Counter L for 100ms interval */
    matchValueL = MSEC_TO_COUNT(100U, sctimerClock);
    sctimerInfo.prescale_l = matchValueL / 65536;
    matchValueL = matchValueL / (sctimerInfo.prescale_l + 1) - 1;

    /* Calculate prescaler and match value for Counter H for 200ms interval */
    matchValueH = MSEC_TO_COUNT(200U, sctimerClock);
    sctimerInfo.prescale_h = matchValueH / 65536;
    matchValueH = matchValueH / (sctimerInfo.prescale_h + 1) - 1;

    /* Initialize SCTimer module */
    SCTIMER_Init(SCT0, &sctimerInfo);

    /* Schedule a match event for Counter L every 0.1 seconds */
    if (SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_MatchEventOnly, matchValueL, 0, kSCTIMER_Counter_L,
                                       &eventCounterL) == kStatus_Fail)
    {
        return -1;
    }

    /* Toggle first output when Counter L event occurs */
    SCTIMER_SetupOutputToggleAction(SCT0, DEMO_FIRST_SCTIMER_OUT, eventCounterL);

    /* Reset Counter L when Counter L event occurs */
    SCTIMER_SetupCounterLimitAction(SCT0, kSCTIMER_Counter_L, eventCounterL);

    /* Schedule a match event for Counter H every 0.2 seconds */
    if (SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_MatchEventOnly, matchValueH, 0, kSCTIMER_Counter_H,
                                       &eventCounterH) == kStatus_Fail)
    {
        return -1;
    }

    /* Toggle second output when Counter H event occurs */
    SCTIMER_SetupOutputToggleAction(SCT0, DEMO_SECOND_SCTIMER_OUT, eventCounterH);

    /* Reset Counter H when Counter H event occurs */
    SCTIMER_SetupCounterLimitAction(SCT0, kSCTIMER_Counter_H, eventCounterH);

    /* Start the L counter */
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);

    /* Start the H counter */
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_H);

    while (1)
    {
    }
}
