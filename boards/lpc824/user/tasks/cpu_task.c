#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "cpu_task.h"
#include "nv.h"
#include "log.h"
 


#define  APP_WDT_IRQ_HANDLER  WDT_IRQHandler


osThreadId   cpu_task_hdl;

void APP_WDT_IRQ_HANDLER(void)
{
    uint32_t wdtStatus = WWDT_GetStatusFlags(WWDT);
    /* The chip will reset before this happens */
    if(wdtStatus & kWWDT_TimeoutFlag)
    {
    /* A watchdog feed didn't occur prior to window timeout */
    /* Stop WDT */
    WWDT_Disable(WWDT);
    WWDT_ClearStatusFlags(WWDT, kWWDT_TimeoutFlag);
    /* Needs restart */
    WWDT_Enable(WWDT);
    }
    /* Handle warning interrupt */
    if (wdtStatus & kWWDT_WarningFlag)
    {
     while (WWDT->TV >= WWDT->WARNINT);
     /* Feed only for the first 5 warnings then allow for a WDT reset to occur */
     WWDT_Refresh(WWDT);
     /* A watchdog feed didn't occur prior to warning timeout */
     WWDT_ClearStatusFlags(WWDT, kWWDT_WarningFlag);
    }
}


void cpu_task(void const * argument)
{
    uint8_t  read_len;
    int      rc;
    uint8_t  nv_buffer[0x40];
    char     cmd[16];
    char     *pos;
    uint8_t  level;

    bsp_sys_led_turn_on();
 
 while (1){
    osDelay(CPU_TASK_INTERVAL_VALUE);
    bsp_sys_led_toggle();

    read_len = log_read(cmd,15);
    cmd[read_len] = 0;
    
    if (strncmp(cmd,"set ",4) == 0) {
        extern float a;
        a = atof(cmd + 4);
        log_debug("set a:%.4f.\r\n",a);
    }
           
    
 
    if ( strncmp(cmd,"read",4) == 0) {
        rc =nv_read(0x00,nv_buffer,0x40);
        if (rc ==0){
            log_debug("read success.\r\n");
        }else{
            log_error("read fail.\r\n");  
        }
     
        for(uint8_t i = 0;i < 0x40;i++){
            log_array("(%d).\r\n",nv_buffer[i]);
        }
    }

    if (strncmp(cmd,"write",5) == 0) {
    
        for(uint8_t i = 0;i < 0x40;i++){
            nv_buffer[i] = i;
        }
        rc = nv_save(0x00,nv_buffer,0x40);
        if (rc == 0){
            log_debug("write success.\r\n");
        }else{
            log_error("write fail.\r\n");  
        }
    }

    if (strncmp(cmd,"print",5) == 0) {    
        for(uint8_t i=0;i<0x40;i++){
        log_array("(%d).\r\n",nv_buffer[i]);
        }
   }

    if (strncmp(cmd,"set level ",10) == 0) {    
        pos = cmd + 10;
        level = atoi(pos);
        log_set_level(level);
    }

 }
}
   
  
 