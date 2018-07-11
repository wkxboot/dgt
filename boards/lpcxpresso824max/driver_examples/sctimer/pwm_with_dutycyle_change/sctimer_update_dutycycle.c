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
#define DEMO_SCTIMER_OUT kSCTIMER_Out_4

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void delay(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool sctimerIsrFlag = false;
volatile bool brightnessUp = true; /* Indicate LED is brighter or dimmer */
volatile uint8_t updatedDutycycle = 10U;
uint32_t eventNumberOutput;

/*******************************************************************************
 * Code
 ******************************************************************************/
void delay(void)
{
    volatile uint32_t i = 0U;
    for (i = 0U; i < 80000U; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

/* The interrupt callback function is used to update the PWM dutycycle */
void SCTIMER_LED_HANDLER()
{
    sctimerIsrFlag = true;

    if (brightnessUp)
    {
        /* Increase duty cycle until it reach limited value, don't want to go upto 100% duty cycle
         * as channel interrupt will not be set for 100%
         */
        if (++updatedDutycycle >= 99U)
        {
            updatedDutycycle = 99U;
            brightnessUp = false;
        }
    }
    else
    {
        /* Decrease duty cycle until it reach limited value */
        if (--updatedDutycycle == 1U)
        {
            brightnessUp = true;
        }
    }

    if (SCTIMER_GetStatusFlags(SCT0) & (1 << eventNumberOutput))
    {
        /* Clear interrupt flag.*/
        SCTIMER_ClearStatusFlags(SCT0, (1 << eventNumberOutput));
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    sctimer_config_t sctimerInfo;
    sctimer_pwm_signal_param_t pwmParam;
    uint32_t sctimerClock;

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
    PRINTF("\r\nSCTimer example to output center-aligned PWM signal\r\n");
    PRINTF("\r\nYou will see a change in LED brightness if an LED is connected to the SCTimer output pin");
    PRINTF("\r\nIf no LED is connected to the pin, then probe the signal using an oscilloscope");

    SCTIMER_GetDefaultConfig(&sctimerInfo);

    /* Initialize SCTimer module */
    SCTIMER_Init(SCT0, &sctimerInfo);

    /* Configure PWM params with frequency 24kHZ from output */
    pwmParam.output = DEMO_SCTIMER_OUT;
    pwmParam.level = kSCTIMER_HighTrue;
    pwmParam.dutyCyclePercent = updatedDutycycle;
    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, 24000U, sctimerClock, &eventNumberOutput) ==
        kStatus_Fail)
    {
        return -1;
    }

    /* Enable interrupt flag for event associated with out 4, we use the interrupt to update dutycycle */
    SCTIMER_EnableInterrupts(SCT0, (1 << eventNumberOutput));

    /* Receive notification when event is triggered */
    SCTIMER_SetCallback(SCT0, SCTIMER_LED_HANDLER, eventNumberOutput);

    /* Enable at the NVIC */
    EnableIRQ(SCT0_IRQn);

    /* Start the timer */
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);

    /* Code below updates the PWM dutycycle for Out */
    while (1)
    {
        /* Use interrupt to update the PWM dutycycle on output */
        if (true == sctimerIsrFlag)
        {
            /* Disable interrupt to retain current dutycycle for a few seconds */
            SCTIMER_DisableInterrupts(SCT0, (1 << eventNumberOutput));

            sctimerIsrFlag = false;

            /* Update PWM duty cycle */
            SCTIMER_UpdatePwmDutycycle(SCT0, DEMO_SCTIMER_OUT, updatedDutycycle, eventNumberOutput);

            /* Delay to view the updated PWM dutycycle */
            delay();

            /* Enable interrupt flag to update PWM dutycycle */
            SCTIMER_EnableInterrupts(SCT0, (1 << eventNumberOutput));
        }
    }
}
