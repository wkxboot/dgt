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
#include "watch_dog_task.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "utils.h"
#include "log.h"
#include "firmware_version.h"



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
    board_init();
    log_init();
    log_info("\r\nfirmware version:%s\r\n",FIRMWARE_VERSION_STR);


    /*电子秤任务消息队列*/
    osMessageQDef(scale_task_msg_q,5,uint32_t);
    scale_task_msg_q_id = osMessageCreate(osMessageQ(scale_task_msg_q),scale_task_hdl);
    log_assert(scale_task_msg_q_id);  
    /*通信协议任务消息队列*/
    osMessageQDef(protocol_task_msg_q,5,uint32_t);
    protocol_task_msg_q_id = osMessageCreate(osMessageQ(protocol_task_msg_q),protocol_task_hdl);
    log_assert(protocol_task_msg_q_id);  
    /*看门狗任务*/
    osThreadDef(watch_dog_task, watch_dog_task, osPriorityBelowNormal, 0, 200);
    watch_dog_task_hdl = osThreadCreate(osThread(watch_dog_task), NULL);
    log_assert(watch_dog_task_hdl);
    /*adc任务*/
    osThreadDef(adc_task, adc_task, osPriorityAboveNormal, 0, 200);
    adc_task_hdl = osThreadCreate(osThread(adc_task), NULL);
    log_assert(adc_task_hdl);
    /*电子秤功能任务*/
    osThreadDef(scale_task, scale_task, osPriorityNormal, 0, 200);
    scale_task_hdl = osThreadCreate(osThread(scale_task), NULL);
    log_assert(scale_task_hdl);
    /*通信协议任务*/
    osThreadDef(protocol_task, protocol_task, osPriorityNormal, 0, 200);
    protocol_task_hdl = osThreadCreate(osThread(protocol_task), NULL);
    log_assert(protocol_task_hdl);
    


  /* Start scheduler */
    osKernelStart();


    while (1)
    {
    }
}
