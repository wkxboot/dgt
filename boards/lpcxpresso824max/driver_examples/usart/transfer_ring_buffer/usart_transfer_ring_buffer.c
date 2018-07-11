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
#define DELAY_TIME 0xFFFFFU

#define RX_RING_BUFFER_SIZE 20U
#define ECHO_BUFFER_SIZE 8U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void EXAMPLE_USARTUserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData);
static void EXAMPLE_USARTIinit(void);
static void EXAMPLE_USARTPrepareTransfer(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
usart_handle_t g_uartHandle;
uint8_t g_tipString[] = "USART RX ring buffer example.\r\nSend back received data.\r\nEcho every 8 bytes.\r\n";
uint8_t g_rxRingBuffer[RX_RING_BUFFER_SIZE] = {0}; /* RX ring buffer. */

uint8_t rxBuffer[ECHO_BUFFER_SIZE] = {0}; /* Buffer for receive data to echo. */
uint8_t txBuffer[ECHO_BUFFER_SIZE] = {0}; /* Buffer for send data to echo. */
volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

usart_transfer_t xfer;
usart_transfer_t sendXfer;
usart_transfer_t receiveXfer;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* USART user callback */
void EXAMPLE_USARTUserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_USART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing = false;
    }

    if (kStatus_USART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing = false;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    size_t receivedBytes;

    /* Initialize the hardware. */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    

    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();

    /* Initialize the USART instance. */
    EXAMPLE_USARTIinit();

    /* Prepare for transactional transmission. */
    EXAMPLE_USARTPrepareTransfer();

    /* Send tip information. */
    txOnGoing = true;
    USART_TransferSendNonBlocking(EXAMPLE_USART, &g_uartHandle, &xfer);

    /* Wait transmit finished. */
    while (txOnGoing)
    {
    }

    while (1)
    {
        /* If txBuffer is empty and rxBuffer is full, copy rxBuffer to txBuffer. */
        if ((!rxBufferEmpty) && (!txBufferFull))
        {
            memcpy(txBuffer, rxBuffer, ECHO_BUFFER_SIZE);
            rxBufferEmpty = true;
            txBufferFull = true;
        }

        /* If RX is idle and rxBuffer is empty, start to read data to rxBuffer. */
        if ((!rxOnGoing) && rxBufferEmpty)
        {
            rxOnGoing = true;
            USART_TransferReceiveNonBlocking(EXAMPLE_USART, &g_uartHandle, &receiveXfer, &receivedBytes);

            /* The receivedBytes is the current data size when API return, If this value equals to ECHO_BUFFER_SIZE,
             * it means all data were received from ring buffer.
             */
            if (ECHO_BUFFER_SIZE == receivedBytes)
            {
                rxBufferEmpty = false;
                rxOnGoing = false;
            }
        }

        /* If TX is idle and txBuffer is full, start to send data. */
        if ((!txOnGoing) && txBufferFull)
        {
            txOnGoing = true;
            USART_TransferSendNonBlocking(EXAMPLE_USART, &g_uartHandle, &sendXfer);
        }
    }
}

static void EXAMPLE_USARTIinit(void)
{
    usart_config_t config;

    /* Default config by using USART_GetDefaultConfig():
     * config->baudRate_Bps = 9600U;
     * config->parityMode = kUSART_ParityDisabled;
     * config->stopBitCount = kUSART_OneStopBit;
     * config->bitCountPerChar = kUSART_8BitsPerChar;
     * config->loopback = false;
     * config->enableRx = false;
     * config->enableTx = false;
     * config->syncMode = kUSART_SyncModeDisabled;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_USART_BAUDRATE;
    config.enableRx = true;
    config.enableTx = true;

    /* Initilize the USART with configuration. */
    USART_Init(EXAMPLE_USART, &config, EXAMPLE_USART_CLK_FREQ);
}
static void EXAMPLE_USARTPrepareTransfer(void)
{
    /* Create USART handle and install callback function. */
    USART_TransferCreateHandle(EXAMPLE_USART, &g_uartHandle, EXAMPLE_USARTUserCallback, NULL);
    /* Start ring buffer. */
    USART_TransferStartRingBuffer(EXAMPLE_USART, &g_uartHandle, g_rxRingBuffer, RX_RING_BUFFER_SIZE);

    /* Prepare transfer for sending tip string. */
    xfer.data = g_tipString;
    xfer.dataSize = sizeof(g_tipString) - 1;

    /* Prepare transfer for sending the received characters. */
    sendXfer.data = txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_SIZE;

    /* Prepare transfer for receiving characters. */
    receiveXfer.data = rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_SIZE;
}
