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

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_adc.h"
#include "fsl_clock.h"
#include "fsl_power.h"
#include "fsl_dma.h"
#include "fsl_inputmux.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC_BASE ADC0
#define DEMO_ADC_SAMPLE_CHANNEL_NUMBER 1U
#define kINPUTMUX_Adc0SeqaIrqToDma kINPUTMUX_AdcASeqaIrqToDma
#define DEMO_ADC_CLOCK_SOURCE kCLOCK_Irc

#define DEMO_DMA_ADC_CHANNEL 0U
#define DEMO_ADC_BASE ADC0
#define DEMO_ADC_DATA_REG_ADDR (uint32_t)(&(ADC0->DAT[DEMO_ADC_SAMPLE_CHANNEL_NUMBER]))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void ADC_Configuration(void);
static void DMA_Configfuation(void);
static void NVIC_Configuration(void);

dma_handle_t gDmaHandleStruct;  /* Handler structure for using DMA. */
uint32_t gDemoAdcConvResult[1]; /* Keep the ADC conversion resulut moved from ADC data register by DMA. */
volatile bool bDmaTransferDone; /* Flag of DMA transfer done trigger by ADC conversion. */

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* Initialize board hardware. */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    

    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();
    
    /* Power on ADC0. */
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);
    PRINTF("ADC DMA example.\r\n");

    /* Configure peripherals. */
    NVIC_Configuration();
    DMA_Configfuation();
    ADC_Configuration();
    PRINTF("Configuration Done.\r\n");
    PRINTF("Type in any key to trigger the conversion ...\r\n");

    while (1)
    {
        /* Get the input from terminal and trigger the converter by software. */
        GETCHAR();

        bDmaTransferDone = false;
        DMA_StartTransfer(&gDmaHandleStruct);         /* Enable the DMA every time for each transfer. */
        ADC_DoSoftwareTriggerConvSeqA(DEMO_ADC_BASE); /* Trigger the ADC and start the conversion. */

        /* Wait for the converter & transfer to be done. */
        while (!bDmaTransferDone)
        {
        }
        PRINTF("Conversion word : 0x%X\r\n", gDemoAdcConvResult[0]);
        PRINTF("Conversion value: %d\r\n", (gDemoAdcConvResult[0] & ADC_DAT_RESULT_MASK) >> ADC_DAT_RESULT_SHIFT);
        PRINTF("\r\n");
    }
}

static void NVIC_Configuration(void)
{
    NVIC_EnableIRQ(DMA0_IRQn);
}

static void ADC_Configuration(void)
{
    adc_config_t adcConfigStruct;
    adc_conv_seq_config_t adcConvSeqConfigStruct;

    /* Configure the converter. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
    adcConfigStruct.clockMode = kADC_ClockSynchronousMode; /* Using sync clock source. */
#endif/* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE */
    adcConfigStruct.clockDividerNumber = 1;                /* The divider for sync clock is 2. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    adcConfigStruct.resolution = kADC_Resolution12bit;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_RESOL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL) & FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL
    adcConfigStruct.enableBypassCalibration = false;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_TSAMP) & FSL_FEATURE_ADC_HAS_CTRL_TSAMP
    adcConfigStruct.sampleTimeNumber = 0U;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_TSAMP */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE) & FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE
    adcConfigStruct.enableLowPowerMode = false;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE */
#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG
   adcConfigStruct.voltageRange = kADC_HighVoltageRange;
#endif/* FSL_FEATURE_ADC_HAS_TRIM_REG */
    ADC_Init(DEMO_ADC_BASE, &adcConfigStruct);

#if !(defined(FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC) && FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC)
#if defined(FSL_FEATURE_ADC_HAS_CALIB_REG) && FSL_FEATURE_ADC_HAS_CALIB_REG
    /* Calibration after power up. */
    if (ADC_DoSelfCalibration(DEMO_ADC_BASE))
#else
    uint32_t frequency;

#if defined(SYSCON_ADCCLKDIV_DIV_MASK)
    frequency = CLOCK_GetFreq(DEMO_ADC_CLOCK_SOURCE) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);
#else
    frequency = CLOCK_GetFreq(DEMO_ADC_CLOCK_SOURCE);
#endif /* SYSCON_ADCCLKDIV_DIV_MASK */
    /* Calibration after power up. */
    if (ADC_DoSelfCalibration(DEMO_ADC_BASE, frequency))
#endif/* FSL_FEATURE_ADC_HAS_CALIB_REG */
    {
        PRINTF("ADC_DoSelfCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC_DoSelfCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC */

#if !(defined(FSL_FEATURE_ADC_HAS_NO_INSEL) && FSL_FEATURE_ADC_HAS_NO_INSEL)
    /* Use the temperature sensor input to channel 0. */
    ADC_EnableTemperatureSensor(DEMO_ADC_BASE, true);
#endif /* FSL_FEATURE_ADC_HAS_NO_INSEL. */

    /* Enable channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER's conversion in Sequence A. */
    adcConvSeqConfigStruct.channelMask =
        (1U << DEMO_ADC_SAMPLE_CHANNEL_NUMBER); /* Includes channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER. */
    adcConvSeqConfigStruct.triggerMask = 0U;
    adcConvSeqConfigStruct.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
    adcConvSeqConfigStruct.enableSingleStep = false;
    adcConvSeqConfigStruct.enableSyncBypass = false;
    adcConvSeqConfigStruct.interruptMode = kADC_InterruptForEachSequence; /* Enable the interrupt/DMA trigger. */
    ADC_SetConvSeqAConfig(DEMO_ADC_BASE, &adcConvSeqConfigStruct);
    ADC_EnableConvSeqA(DEMO_ADC_BASE, true); /* Enable the conversion sequence A. */

    ADC_EnableInterrupts(DEMO_ADC_BASE, kADC_ConvSeqAInterruptEnable);
}

/* Software ISR for DMA transfer done. */
void DEMO_DMA_Callback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    bDmaTransferDone = true;
}

static void DMA_Configfuation(void)
{
    dma_transfer_config_t dmaTransferConfigStruct;
    dma_channel_trigger_t dmaChannelTriggerStruct;

    /* Configure DMAMUX. */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, DEMO_DMA_ADC_CHANNEL, kINPUTMUX_Adc0SeqaIrqToDma);

    /* Configure DMA. */
    DMA_Init(DMA0);
    DMA_EnableChannel(DMA0, DEMO_DMA_ADC_CHANNEL);
    DMA_CreateHandle(&gDmaHandleStruct, DMA0, DEMO_DMA_ADC_CHANNEL);
    DMA_SetCallback(&gDmaHandleStruct, DEMO_DMA_Callback, NULL);
    /*
     * Configure the DMA trigger:
     * The DATAVALID of ADC will trigger the interrupt. This signal is also for thie DMA triger, which is changed 0 ->
     * 1.
     */
    dmaChannelTriggerStruct.burst = kDMA_EdgeBurstTransfer1;
    dmaChannelTriggerStruct.type = kDMA_RisingEdgeTrigger;
    dmaChannelTriggerStruct.wrap = kDMA_NoWrap;
    DMA_ConfigureChannelTrigger(DMA0, DEMO_DMA_ADC_CHANNEL, &dmaChannelTriggerStruct);
    /* Prepare and submit the transfer. */
    DMA_PrepareTransfer(&dmaTransferConfigStruct,       /* To keep the configuration. */
                        (void *)DEMO_ADC_DATA_REG_ADDR, /* DMA transfer source address. */
                        (void *)gDemoAdcConvResult,     /* DMA transfer destination address. */
                        sizeof(uint32_t),               /* DMA transfer destination address width(bytes). */
                        sizeof(uint32_t),               /* DMA transfer bytes to be transferred. */
                        kDMA_MemoryToMemory,            /* DMA transfer type. */
                        NULL                            /* nextDesc Chain custom descriptor to transfer. */
                        );
    DMA_SubmitTransfer(&gDmaHandleStruct, &dmaTransferConfigStruct);
}
