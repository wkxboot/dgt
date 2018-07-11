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

#include "fsl_spi.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SPI_MASTER SPI0
#define EXAMPLE_CLK_SRC kCLOCK_MainClk
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFreq(EXAMPLE_CLK_SRC)
#define EXAMPLE_SPI_MASTER_BAUDRATE 500000U
#define EXAMPLE_SPI_MASTER_SSEL kSPI_Ssel0Assert

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void EXAMPLE_SPIMasterInit(void);
static void EXAMPLE_MasterStartTransfer(void);
static void EXAMPLE_TransferDataCheck(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define BUFFER_SIZE (64)
static uint8_t txBuffer[BUFFER_SIZE];
static uint8_t rxBuffer[BUFFER_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void)
{
    /* Initizlize the hardware(clock/pins configuration/debug console). */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    

    /* Enable clock of spi0. */
    CLOCK_EnableClock(kCLOCK_Spi0);
    
    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();


    PRINTF("This is SPI polling transfer master example.\n\r");
    PRINTF("\n\rMaster start to send data to slave, please make sure the slave has been started!\n\r");

    /* Initialize the SPI master with configuration. */
    EXAMPLE_SPIMasterInit();

    /* Start transfer with slave board. */
    EXAMPLE_MasterStartTransfer();

    /* Check the received data. */
    EXAMPLE_TransferDataCheck();

    /* De-initialize the SPI. */
    SPI_Deinit(EXAMPLE_SPI_MASTER);

    while (1)
    {
    }
}

static void EXAMPLE_SPIMasterInit(void)
{
    spi_master_config_t userConfig = {0};
    uint32_t srcFreq = 0U;
    /* Note: The slave board using interrupt way, slave will spend more time to write data
     *       to TX register, to prevent TX data missing in slave, we will add some delay between
     *       frames and capture data at the second edge, this operation will make the slave
     *       has more time to prapare the data.
     */
    SPI_MasterGetDefaultConfig(&userConfig);
    userConfig.baudRate_Bps = EXAMPLE_SPI_MASTER_BAUDRATE;
    userConfig.sselNumber = EXAMPLE_SPI_MASTER_SSEL;
    userConfig.clockPhase = kSPI_ClockPhaseSecondEdge;
    userConfig.delayConfig.frameDelay = 0xFU;
    srcFreq = EXAMPLE_SPI_MASTER_CLK_FREQ;
    SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq);
}

static void EXAMPLE_MasterStartTransfer(void)
{
    uint32_t i = 0U;
    spi_transfer_t xfer = {0};

    /* Init Buffer*/
    for (i = 0; i < BUFFER_SIZE; i++)
    {
        txBuffer[i] = i % 256;
        rxBuffer[i] = 0U;
    }

    /*Start Transfer*/
    xfer.txData = txBuffer;
    xfer.rxData = rxBuffer;
    xfer.dataSize = sizeof(txBuffer);
    xfer.configFlags = kSPI_EndOfTransfer | kSPI_EndOfFrame;
    /* Transfer data in polling mode. */
    SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);
}

static void EXAMPLE_TransferDataCheck(void)
{
    uint32_t i = 0U, err = 0U;
    PRINTF("\n\rThe received data are:");
    for (i = 0; i < BUFFER_SIZE; i++)
    {
        /* Print 16 numbers in a line */
        if ((i & 0x0FU) == 0U)
        {
            PRINTF("\n\r");
        }
        PRINTF("  0x%02X", rxBuffer[i]);
        /* Check if data matched. */
        if (txBuffer[i] != rxBuffer[i])
        {
            err++;
        }
    }

    if (err == 0)
    {
        PRINTF("\n\rMaster polling transfer succeed!\n\r");
    }
    else
    {
        PRINTF("\n\rMaster polling transfer faild!\n\r");
    }
}