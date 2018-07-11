/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
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
#define EXAMPLE_SPI_MASTER_IRQ SPI0_IRQn

#define SPI_MASTER_IRQHandler SPI0_IRQHandler

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
static uint32_t txIndex = BUFFER_SIZE;
static uint32_t rxIndex = BUFFER_SIZE;
static volatile bool slaveFinished = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void SPI_MASTER_IRQHandler(void)
{
    /* Check if data is ready in RX register. */
    if ((SPI_GetStatusFlags(EXAMPLE_SPI_MASTER) & kSPI_RxReadyFlag))
    {
        rxBuffer[BUFFER_SIZE - rxIndex] = SPI_ReadData(EXAMPLE_SPI_MASTER);
        rxIndex--;
    }
    /* Write data to TX regsiter if TX register is ready. */
    if ((SPI_GetStatusFlags(EXAMPLE_SPI_MASTER) & kSPI_TxReadyFlag) && (txIndex != 0U))
    {
        /* If this is the last byte to send. */
        if (1U == txIndex)
        {
            /* Add end of transfer configuration. */
            SPI_WriteConfigFlags(EXAMPLE_SPI_MASTER, kSPI_EndOfTransfer);
            SPI_WriteData(EXAMPLE_SPI_MASTER, txBuffer[BUFFER_SIZE - txIndex]);
        }
        else
        {
            SPI_WriteData(EXAMPLE_SPI_MASTER, (uint16_t)(txBuffer[BUFFER_SIZE - txIndex]));
        }
        txIndex--;
    }
    /* If no data to be transferred, disable the interrupt. */
    if ((txIndex == 0U) && (rxIndex == 0U))
    {
        slaveFinished = true;
        SPI_DisableInterrupts(EXAMPLE_SPI_MASTER, kSPI_RxReadyInterruptEnable);
    }
}

int main(void)
{
    /* Initialize the boards */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    

    /* Enable clock of spi0. */
    CLOCK_EnableClock(kCLOCK_Spi0);
    
    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();


    PRINTF("This is SPI interrupt functional master example.\n\r");
    PRINTF("\n\rMaster start to send data to slave, please make sure the slave has been started!\n\r");

    /* Initialize the SPI master with configuration. */
    EXAMPLE_SPIMasterInit();

    /* Start transfer with slave board. */
    EXAMPLE_MasterStartTransfer();

    /* Check the received data. */
    EXAMPLE_TransferDataCheck();

    /* De-initialize the SPI master. */
    SPI_Deinit(EXAMPLE_SPI_MASTER);

    while (1)
    {
    }
}

static void EXAMPLE_SPIMasterInit(void)
{
    spi_master_config_t masterConfig = {0};
    uint32_t srcFreq = 0U;

    /* configuration from using SPI_MasterGetDefaultConfig():
     * userConfig->enableLoopback = false;
     * userConfig->enableMaster = true;
     * userConfig->polarity = kSPI_ClockPolarityActiveHigh;
     * userConfig->phase = kSPI_ClockPhaseFirstEdge;
     * userConfig->direction = kSPI_MsbFirst;
     * userConfig->baudRate_Bps = 500000U;
     * userConfig->dataWidth = kSPI_Data8Bits;
     * userConfig->sselNum = kSPI_Ssel0Assert;
     * userConfig->sselPol = kSPI_SpolActiveAllLow;
     */
    SPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = EXAMPLE_SPI_MASTER_BAUDRATE;
    masterConfig.sselNumber = EXAMPLE_SPI_MASTER_SSEL;
    srcFreq = EXAMPLE_SPI_MASTER_CLK_FREQ;
    SPI_MasterInit(EXAMPLE_SPI_MASTER, &masterConfig, srcFreq);
}

static void EXAMPLE_MasterStartTransfer(void)
{
    uint32_t i = 0U;

    /* Init source buffer */
    for (i = 0U; i < BUFFER_SIZE; i++)
    {
        txBuffer[i] = i;
        rxBuffer[i] = 0U;
    }

    /* Write data to TXDAT register to trigger receive interrupt. */
    SPI_WriteData(EXAMPLE_SPI_MASTER, txBuffer[BUFFER_SIZE - (txIndex--)]);
    /* Enable SPI receive ready interrupt. */
    EnableIRQ(EXAMPLE_SPI_MASTER_IRQ);
    SPI_EnableInterrupts(EXAMPLE_SPI_MASTER, kSPI_RxReadyInterruptEnable);
}

static void EXAMPLE_TransferDataCheck(void)
{
    uint32_t i = 0U, err = 0U;
    /* Waiting for the transmission complete. */
    while (slaveFinished != true)
    {
    }

    PRINTF("\n\rThe received data are:");
    /*Check if the data is right*/
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
        PRINTF("\n\rMaster transfer succeed!\n\r");
    }
    else
    {
        PRINTF("\n\rMaster transfer faild!\n\r");
    }
}
