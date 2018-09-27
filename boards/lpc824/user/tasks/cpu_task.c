#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "cpu_task.h"
#include "nv.h"
#include "log.h"
#define LOG_MODULE_NAME   "[cpu_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


osThreadId   cpu_task_hdl;



void cpu_task(void const * argument)
{
 uint8_t nv_buffer[16];
 uint8_t cmd[3];
 uint8_t read_len;
 int rc;
 
 wwdt_config_t config;
 config.enableWwdt = true;
 config.enableWatchdogReset = true;
 config.enableWatchdogProtect = false;
 config.enableLockOscillator = false;
 config.windowValue = 0xFFFFFFU;
 config.timeoutValue = CPU_TASK_WTG_TIMEOUT_VALUE;
 config.warningValue = 0;
 //WWDT_Init(WWDT,&config);
 //WWDT_Enable(WWDT);
 log_warning("watch dog start...timeout value:%dms.\r\n",CPU_TASK_WTG_TIMEOUT_VALUE/10);
 bsp_sys_led_turn_on();
 while(1){
 //WWDT_Refresh(WWDT);
 //log_one_line("feed dog ok.cpu: %d%%. weight:%dg.",osGetCPUUsage());
 osDelay(CPU_TASK_INTERVAL_VALUE);
 bsp_sys_led_toggle();
 
 read_len = log_read(cmd,3);
 if(read_len == 3){
   switch(cmd[0]){
   case 'r':
     rc =nv_read(0x40,nv_buffer,16);
     if(rc ==0){
     log_debug("read success.\r\n");
     }else{
      log_error("read fail.\r\n");  
     }
     
     for(uint8_t i=0;i<16;i++){
     log_array("(%d).\r\n",nv_buffer[i]);
     }
     break;
     
  case 'w':
     for(uint8_t i=0;i<16;i++){
     nv_buffer[i] = i;
     }
    rc = nv_save(0x40,nv_buffer,16);
    if(rc == 0){
     log_debug("write success.\r\n");
     }else{
      log_error("write fail.\r\n");  
     }
   break;
   
        
  case 'p':
     for(uint8_t i=0;i<16;i++){
     log_array("(%d).\r\n",nv_buffer[i]);
     }
     break;
     }
   }
 }
        
   }
   
  
 