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
#define EXAMPLE_SPI_SLAVE SPI0


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void slaveCallback(SPI_Type *base, spi_slave_handle_t *slaveHandle, status_t status, void *userData);
static void EXAMPLE_SlaveInit(void);
static void EXAMPLE_SlaveStartTransfer(void);
static void EXAMPLE_TransferDataCheck(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define BUFFER_SIZE (64)
spi_slave_handle_t slaveHandle;
static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];
static volatile bool slaveFinished = false;
/*******************************************************************************
 * Code
 ******************************************************************************/
static void slaveCallback(SPI_Type *base, spi_slave_handle_t *slaveHandle, status_t status, void *userData)
{
    slaveFinished = true;
}

int main(void)
{
    /* Initialize the hardware. */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    

    /* Enable clock of spi0. */
    CLOCK_EnableClock(kCLOCK_Spi0);
    
    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();


    PRINTF("This is SPI polling transfer slave example.\n\r");
    PRINTF("\n\rSlave is working....\n\r");

    /* Initialize the slave SPI with configuration. */
    EXAMPLE_SlaveInit();

    /* Slave start transfer with master. */
    EXAMPLE_SlaveStartTransfer();

    /* Check transfer data. */
    EXAMPLE_TransferDataCheck();

    /* De-initialize the SPI. */
    SPI_Deinit(EXAMPLE_SPI_SLAVE);

    while (1)
    {
    }
}

static void EXAMPLE_SlaveInit(void)
{
    spi_slave_config_t userConfig;
    /* Default configuration for slave:
     * userConfig.enableSlave = true;
     * userConfig.polarity = kSPI_ClockPolarityActiveHigh;
     * userConfig.phase = kSPI_ClockPhaseFirstEdge;
     * userConfig.direction = kSPI_MsbFirst;
     * userConfig.dataWidth = kSPI_Data8Bits;
     * userConfig.sselPol = kSPI_SpolActiveAllLow;
     */
    SPI_SlaveGetDefaultConfig(&userConfig);
    /* To align configuration with master. */
    userConfig.clockPhase = kSPI_ClockPhaseSecondEdge;
    SPI_SlaveInit(EXAMPLE_SPI_SLAVE, &userConfig);
}

static void EXAMPLE_SlaveStartTransfer(void)
{
    spi_transfer_t xfer = {0};
    uint32_t i = 0U;

    /* Initialize the txBuffer and rxBuffer. */
    for (i = 0; i < BUFFER_SIZE; i++)
    {
        txBuffer[i] = i;
        rxBuffer[i] = 0U;
    }

    SPI_SlaveTransferCreateHandle(EXAMPLE_SPI_SLAVE, &slaveHandle, slaveCallback, NULL);

    /* Receive data from master. */
    xfer.txData = txBuffer;
    xfer.rxData = rxBuffer;
    xfer.dataSize = sizeof(txBuffer);
    SPI_SlaveTransferNonBlocking(EXAMPLE_SPI_SLAVE, &slaveHandle, &xfer);
}

static void EXAMPLE_TransferDataCheck(void)
{
    uint32_t i = 0U, err = 0U;

    /* Waiting for transfer complete, if transmission is completed, the slaveFinished
     * will be set to ture in slaveCallback.
     */
    while (slaveFinished != true)
    {
    }

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
        PRINTF("\n\rSlave interrupt transfer succeed!\n\r");
    }
    else
    {
        PRINTF("\n\rSlave interrupt transfer faild!\n\r");
    }
}
