/*
 * The Clear BSD License
 * Copyright 2018 NXP
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

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_wkt.h"

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_LED_INIT LED_RED_INIT(1)
#define APP_LED_TOGGLE (LED_RED_TOGGLE())
#define WKT_CLK_FREQ (CLOCK_GetFreq(kCLOCK_Irc) / 16)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool wktIsrFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void WKT_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    WKT_ClearStatusFlags(WKT, kWKT_AlarmFlag);
    wktIsrFlag = true;
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t wkt_clock;

    /* Structure of initialize WKT */
    wkt_config_t wktConfig;

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

    /* Initialize and enable LED */
    APP_LED_INIT;

    wkt_clock = WKT_CLK_FREQ;

    /* Print a note to terminal */
    PRINTF("\r\nWKT interrupt example\r\n");

    /* config->clockSource = kWKT_DividedFROClockSource */
    WKT_GetDefaultConfig(&wktConfig);

    /* Init wkt module */
    WKT_Init(WKT, &wktConfig);
    
    /* Clear Pending Interrupt */
    NVIC_ClearPendingIRQ(WKT_IRQn);
    /* Enable at the NVIC */
    EnableIRQ(WKT_IRQn);

    while(1)
    {
        /* Set counter value to start the timer counting. Timer counts down to 0, stops and 
         * generates an interrupt. When a new count value is loaded, Timer starts again.
         */
        WKT_StartTimer(WKT, USEC_TO_COUNT(250000U, wkt_clock));
        while(wktIsrFlag != true)
        {
        }
        PRINTF("\r\n Self-wake-up timer interrupt is occured !");
        APP_LED_TOGGLE;
        wktIsrFlag = false;
    }
}
