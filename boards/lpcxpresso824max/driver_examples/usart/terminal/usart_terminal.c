/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
 * All rights reserved.
 *
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

#include "board.h"
#include "fsl_usart.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_USART USART0
#define EXAMPLE_USART_CLK_SRC kCLOCK_MainClk
#define EXAMPLE_USART_CLK_FREQ CLOCK_GetFreq(EXAMPLE_USART_CLK_SRC)
#define EXAMPLE_USART_IRQn USART0_IRQn

/*! @brief Buffer size (Unit: Byte). */
#define RX_BUFFER_SIZE 32
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void EXAMPLE_USARTInit(void);
static void EXAMPLE_USARTSendToTerminal(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t g_demoInfo[] = "Usart functional API interrupt example.\r\nBoard receives characters then sends them out.\r\n";
uint8_t g_promptInfo[] = "Please input characters and press the Enter key to finish input:\r\n";
uint8_t g_logInfo[] = "The received characters are:";
uint8_t g_overFlowInfo[] = "\r\nYou have input too many characters, Please try again!\r\n";

uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxDataCounter = 0U;
volatile uint8_t txDataCounter = 0U;
volatile bool rxNewLineDetected = false;
volatile bool rxDataOverFlowFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void USART0_IRQHandler(void)
{
    uint8_t data;

    /* If TX is ready to send, and the `Enter` key is detected, send the received characters. */
    if (kUSART_TxReady & USART_GetStatusFlags(EXAMPLE_USART) && rxNewLineDetected)
    {
        USART_WriteByte(EXAMPLE_USART, rxBuffer[txDataCounter++]);

        if (txDataCounter == rxDataCounter)
        {
            /* Disable TX ready interrupt. */
            USART_DisableInterrupts(EXAMPLE_USART, kUSART_TxReadyInterruptEnable);
        }
    }

    /* If this Rx read flag is set, read data to buffer. */
    if (kUSART_RxReady & USART_GetStatusFlags(EXAMPLE_USART))
    {
        data = USART_ReadByte(EXAMPLE_USART);
        rxBuffer[rxDataCounter++] = data;
        /* Wait for TX is ready, send back the character. */
        while (!(kUSART_TxReady & USART_GetStatusFlags(EXAMPLE_USART)))
        {
        }
        USART_WriteByte(EXAMPLE_USART, data);
        /* Store the received character to rx buffer. */
        if (0x0D == data)
        {
            rxBuffer[rxDataCounter++] = 0x0A;
            /* Wait for TX is ready, send the character. */
            while (!(kUSART_TxReady & USART_GetStatusFlags(EXAMPLE_USART)))
            {
            }
            USART_WriteByte(EXAMPLE_USART, 0x0A);
            /* Set the rxNewLineDetected to true, and start a new line. */
            rxNewLineDetected = true;
        }
        else
        {
            /* If the data length is larger then the buffer size, set the rxDataOverFlowFlag.
             * The last 2 bytes of rx buffer is 0x0D and 0x0A.
             */
            if (rxDataCounter == RX_BUFFER_SIZE - 1)
            {
                rxDataOverFlowFlag = true;
            }
        }
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    

    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();

    /* Initialize the USART instance with configuration, and enable receive ready interrupt. */
    EXAMPLE_USARTInit();

    while (1)
    {
        /* If the Enter key was pressed, send the received characters out to the terminal, and start a new line.*/
        if (true == rxNewLineDetected)
        {
            EXAMPLE_USARTSendToTerminal();
        }
        /* If too many characters were received, send prompt info to terminal. */
        if (true == rxDataOverFlowFlag)
        {
            /* Send the overflow info to the PC terminal. */
            USART_WriteBlocking(EXAMPLE_USART, g_overFlowInfo,
                                (sizeof(g_overFlowInfo) / sizeof(g_overFlowInfo[0])) - 1);

            /* Reset rxDataOverFlowFlag and rxDataCounter for next loop. */
            rxDataOverFlowFlag = false;
            rxDataCounter = 0U;
        }
    }
}

static void EXAMPLE_USARTInit(void)
{
    usart_config_t config;
    /* Default config by using USART_GetDefaultConfig():
     * config.baudRate_Bps = 9600U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.bitCountPerChar = kUSART_8BitsPerChar;
     * config.loopback = false;
     * config.enableRx = false;
     * config.enableTx = false;
     * config.syncMode = kUSART_SyncModeDisabled;
     */
    USART_GetDefaultConfig(&config);
    config.enableRx = true;
    config.enableTx = true;
    config.baudRate_Bps = BOARD_DEBUG_USART_BAUDRATE;

    /* Initialize the USART with configuration. */
    USART_Init(EXAMPLE_USART, &config, EXAMPLE_USART_CLK_FREQ);

    /* Send demo info and prompt info out to terminal in blocking way. */
    USART_WriteBlocking(EXAMPLE_USART, g_demoInfo, (sizeof(g_demoInfo) / sizeof(g_demoInfo[0])) - 1);
    USART_WriteBlocking(EXAMPLE_USART, g_promptInfo, (sizeof(g_promptInfo) / sizeof(g_promptInfo[0])) - 1);

    /* Enable USART RX ready interrupt. */
    USART_EnableInterrupts(EXAMPLE_USART, kUSART_RxReadyInterruptEnable);
    EnableIRQ(EXAMPLE_USART_IRQn);
}

static void EXAMPLE_USARTSendToTerminal(void)
{
    /* Send log info to terminal in polling-mode. */
    USART_WriteBlocking(EXAMPLE_USART, g_logInfo, (sizeof(g_logInfo) / sizeof(g_logInfo[0])) - 1);

    /* Send back the received characters to terminal in interrupt-mode. */
    USART_EnableInterrupts(EXAMPLE_USART, kUSART_TxReadyInterruptEnable);

    /* Waitting for the Tx complete. */
    while (txDataCounter != rxDataCounter)
    {
    }
    /* Reset the txDataCounter, rxDataCounter and rxNewLineDetected for next loop. */
    txDataCounter = 0U;
    rxDataCounter = 0U;
    rxNewLineDetected = false;

    /* Send prompt info to terminal in polling-mode. */
    USART_WriteBlocking(EXAMPLE_USART, g_promptInfo, (sizeof(g_promptInfo) / sizeof(g_promptInfo[0])) - 1);
}
