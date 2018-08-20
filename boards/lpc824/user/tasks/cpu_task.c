#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "cpu_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[cpu_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 

void cpu_task(void const * argument)
{
 wwdt_config_t config;
 config.enableWwdt = true;
 config.enableWatchdogReset = true;
 config.enableWatchdogProtect = false;
 config.enableLockOscillator = false;
 config.windowValue = 0xFFFFFFU;
 config.timeoutValue = CPU_TASK_WTG_TIMEOUT_VALUE;
 config.warningValue = 0;
 WWDT_Init(WWDT,&config);
 WWDT_Enable(WWDT);
 log_warning("watch dog start...timeout value:%dms.\r\n",1100);
 
 while(1){
 WWDT_Refresh(WWDT);
 log_one_line("feed dog ok.cpu: %d%%.\r\n",osGetCPUUsage());
 osDelay(CPU_TASK_INTERVAL_VALUE);
 }

}