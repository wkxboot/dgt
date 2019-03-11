#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "cpu_task.h"
#include "kalman_filter.h"
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
extern kalman1_state kalman_filter;

void cpu_task(void const * argument)
{
    uint8_t  read_cnt;
    char     cmd[16];
    uint8_t  level;
    float param;

    bsp_sys_led_turn_on();
 
 while (1){
    osDelay(CPU_TASK_INTERVAL_VALUE);
    bsp_sys_led_toggle();
    /*设置日志输出等级*/
    read_cnt = log_read(cmd,15);
    cmd[read_cnt] = 0;
    if (strncmp(cmd,"set level ",strlen("set level ")) == 0) {
        level = atoi(cmd + strlen("set level "));
        log_set_level(level);
    }
    if (strncmp(cmd,"a=",strlen("a=")) == 0) {
        param = atof(cmd + strlen("a="));
        kalman_filter.A = param;
    }
    
        if (strncmp(cmd,"h=",strlen("h=")) == 0) {
        param = atof(cmd + strlen("h="));
        kalman_filter.H = param;
    }
    
        if (strncmp(cmd,"q=",strlen("q=")) == 0) {
        param = atof(cmd + strlen("q="));
        kalman_filter.q = param;
    }
    
        if (strncmp(cmd,"r=",strlen("r=")) == 0) {
        param = atof(cmd + strlen("r="));
        kalman_filter.r = param;
    }
 }
}
   
  
 