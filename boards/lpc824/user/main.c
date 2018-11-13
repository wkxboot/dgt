/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
#include "cmsis_os.h"
#include "board.h"
#include "adc_task.h"
#include "cpu_task.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "version.h"

#include "log.h"
#define LOG_MODULE_NAME   "[main]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG  



/*******************************************************************************
 * Prototypes
 ******************************************************************************/

uint32_t log_time()
{
return osKernelSysTick();
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{ 
    int result;
    result = board_init();
    if(result != 0){
    while(1);
    }
    log_init();
    log_info("firmware version:%d.%d.\r\n",FIRMWARE_VERSION >> 8,FIRMWARE_VERSION & 0xff);
    
    osThreadDef(cpu_task, cpu_task, osPriorityNormal, 0, 140);
    cpu_task_hdl = osThreadCreate(osThread(cpu_task), NULL);

    osThreadDef(adc_task, adc_task, osPriorityNormal, 0, 128);
    adc_task_hdl = osThreadCreate(osThread(adc_task), NULL);

    osThreadDef(scale_task, scale_task, osPriorityNormal, 0, 180);
    scale_task_hdl = osThreadCreate(osThread(scale_task), NULL);

    osThreadDef(protocol_task, protocol_task, osPriorityNormal, 0, 160);
    protocol_task_hdl = osThreadCreate(osThread(protocol_task), NULL);
    


  /* Start scheduler */
    osKernelStart();


    while (1)
    {
    }
}
