/*
 * The Clear BSD License
 * Copyright 2017 NXP
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

/*  Standard C Included Files */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_I2C_MASTER_BASE (I2C0_BASE)
#define I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)

#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define I2C_BAUDRATE 100000U
#define I2C_DATA_LENGTH 33U
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];
volatile bool g_MasterCompletionFlag = false;

/*******************************************************************************
 * Code

 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{
    i2c_master_config_t masterConfig;
    status_t reVal = kStatus_Fail;
    uint8_t deviceAddress = 0x01U;

    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    
    /* Enable clock of i2c0. */   
    CLOCK_EnableClock(kCLOCK_I2c0);

    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();

    PRINTF("\r\nI2C board2board polling example -- Master transfer.\r\n");

    /* Set up i2c master to send data to slave*/
    /* First data in txBuff is data length of the transmiting data. */
    g_master_txBuff[0] = I2C_DATA_LENGTH - 1U;
    for (uint32_t i = 1U; i < I2C_DATA_LENGTH; i++)
    {
        g_master_txBuff[i] = i - 1;
    }

    PRINTF("Master will send data :");
    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1U; i++)
    {
        if (i % 8 == 0)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%2x  ", g_master_txBuff[i + 1]);
    }
    PRINTF("\r\n\r\n");

    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kI2C_2PinOpenDrain;
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    /* Initialize the I2C master peripheral */
    I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);

    /* Send master blocking data to slave */
    if (kStatus_Success == I2C_MasterStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Write))
    {
        /* subAddress = 0x01, data = g_master_txBuff - write to slave.
          start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
        reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, &deviceAddress, 1, kI2C_TransferNoStopFlag);
        if (reVal != kStatus_Success)
        {
            return -1;
        }
        reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, g_master_txBuff, I2C_DATA_LENGTH, 0);
        if (reVal != kStatus_Success)
        {
            return -1;
        }

        reVal = I2C_MasterStop(EXAMPLE_I2C_MASTER);
        if (reVal != kStatus_Success)
        {
            return -1;
        }
    }

    PRINTF("Receive sent data from slave :");

    /* Receive blocking data from slave */
    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
      start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
    if (kStatus_Success == I2C_MasterStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Write))
    {
        /* subAddress = 0x01, data = g_master_txBuff - write to slave.
          start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
        reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, &deviceAddress, 1, kI2C_TransferNoStopFlag);
        if (reVal != kStatus_Success)
        {
            return -1;
        }

        reVal = I2C_MasterRepeatedStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Read);
        if (reVal != kStatus_Success)
        {
            return -1;
        }

        reVal = I2C_MasterReadBlocking(EXAMPLE_I2C_MASTER, g_master_rxBuff, I2C_DATA_LENGTH - 1, 0);
        if (reVal != kStatus_Success)
        {
            return -1;
        }

        reVal = I2C_MasterStop(EXAMPLE_I2C_MASTER);
        if (reVal != kStatus_Success)
        {
            return -1;
        }
    }

    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1; i++)
    {
        if (i % 8 == 0)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%2x  ", g_master_rxBuff[i]);
    }
    PRINTF("\r\n\r\n");

    /* Transfer completed. Check the data.*/
    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1; i++)
    {
        if (g_master_rxBuff[i] != g_master_txBuff[i + 1])
        {
            PRINTF("\r\nError occured in the transfer ! \r\n");
            break;
        }
    }

    PRINTF("\r\nEnd of I2C example .\r\n");
    while (1)
    {
    }
}
