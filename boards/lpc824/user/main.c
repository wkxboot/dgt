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

#include "fsl_device_registers.h"
#include "fsl_wwdt.h"
#include "board.h"

#include "pin_mux.h"
#include "cmsis_os.h"
#include "modbus.h"
#include "cpu_utils.h"
#include "log.h"
#define LOG_MODULE_NAME   "[main]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG  
/*******************************************************************************
 * Definitions
 ******************************************************************************/
osThreadId defaultTaskHandle;
osThreadId cpu_task_hdl;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern serial_hal_driver_t modbus_serial_driver;

modbus_t *ctx;
modbus_mapping_t* map;
uint32_t log_time()
{
 return osKernelSysTick();
}
uint16_t reg[10];
uint8_t bits[10];

uint8_t req[20];

void cpu_task(void const * argument)
{
 wwdt_config_t config;
 config.enableWwdt = true;
 config.enableWatchdogReset = true;
 config.enableWatchdogProtect = false;
 config.enableLockOscillator = false;
 config.windowValue = 0xFFFFFFU;
 config.timeoutValue = 11000;
 config.warningValue = 0;
 WWDT_Init(WWDT,&config);
 WWDT_Enable(WWDT);
 log_warning("watch dog start...timeout value:%dms.\r\n",1100);
 
 
 while(1){
 WWDT_Refresh(WWDT);
 log_warning("feed dog ok. cur_cpu: %d%%.\r\n",osGetCPUUsage());
 osDelay(1000);
  }
}
void StartDefaultTask(void const * argument)
{
  int i=0;
  log_init();
    
  ctx = modbus_new_rtu(0, 115200, 8, 1,&modbus_serial_driver);
  
 if(ctx == NULL)
 {
 log_error("创建modbus rtu实体失败！\r\n");
 }
 if(modbus_set_slave(ctx, 1)== -1){
  log_error("设置slave id失败.\r\n");
 }
 if (modbus_connect(ctx) == -1) {
  log_error("打开串口失败.\r\n");
  }
 map = modbus_mapping_new(8,8,8,8);
 if(map == NULL){
 log_error("map reg err.\r\n");
goto err;
 }
   map->tab_input_registers[0]=0x1234;
   map->tab_input_registers[1]=0xabcd;
   map->tab_input_registers[2]=0x3344;
   map->tab_input_registers[3]=3333;
   map->tab_input_registers[4]=4444;
   map->tab_input_registers[5]=5555;
   map->tab_input_registers[6]=6666;
   map->tab_input_registers[7]=7777;
  for(;;)
  {
 /*
   i = modbus_read_input_registers(ctx, 0, 10, reg);
   if (i == -1) {
   modbus_flush(ctx);
   log_error("读input寄存器失败.\r\n");
   }else{
   log_info("读input寄存器成功.\r\n");
   }

   i = modbus_write_registers(ctx, 10, 10, reg);
   if (i == -1) {
   modbus_flush(ctx);
   log_error("写寄存器失败.\r\n");
   }else{
   log_info("写寄存器成功.\r\n");
   }
 
   i = modbus_read_bits(ctx,20,10,bits);
   if(i == -1){
    modbus_flush(ctx);
    log_error("读bits寄存器失败.\r\n");
   }else{
   log_info("读bits寄存器成功.\r\n");  
   }
   i = modbus_write_bits(ctx, 30, 10,bits);
   if (i == -1) {
   modbus_flush(ctx);
   log_error("写bits寄存器失败.\r\n");
   }else{
   log_info("写bits寄存器成功.\r\n");
   }

    modbus_flush(ctx);
*/
    
  log_debug("start wait for write...\r\n");
   
  i = modbus_receive(ctx,req);
  if(i == -1){
  log_error("master reg wait error.\r\n ");
  goto err;
  }else if(i > 0) {
   modbus_reply(ctx, req, i, map);
   /* Connection closed by the client or error */
  log_info("master reg ok.\r\n ");
   }
  continue;
  err:  
   osDelay(1000);
  }
  /* USER CODE END 5 */ 
}
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{

    /* Init board hardware. */
    /* Enable clock of uart0. */
    CLOCK_EnableClock(kCLOCK_Uart0);
    /* Ser DIV of uart0. */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);    

    BOARD_InitPins();
    BOARD_BootClockPll30M();

    
   osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 200);
   defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

   osThreadDef(cpu_task, cpu_task, osPriorityNormal, 0, 128);
   cpu_task_hdl = osThreadCreate(osThread(cpu_task), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
    osKernelStart();


    while (1)
    {
    }
}
