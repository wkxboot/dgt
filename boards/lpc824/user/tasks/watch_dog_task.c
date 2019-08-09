#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "watch_dog_task.h"
#if !defined(FSL_FEATURE_WWDT_HAS_NO_PDCFG) || (!FSL_FEATURE_WWDT_HAS_NO_PDCFG)
#include "fsl_power.h"
#endif
#include "kalman_filter.h"
#include "log.h"
 
#define  WDT_ENABLE_RESET           0
   
#define WDT_CLK_FREQ CLOCK_GetFreq(kCLOCK_WdtOsc)

osThreadId   watch_dog_task_hdl;


/*
* @brief 看门狗中断
* @param 无
* @param
* @return 无
* @note
*/
void WDT_IRQHandler(void)
{
    uint32_t wdtStatus = WWDT_GetStatusFlags(WWDT);

    /* Handle warning interrupt */
    if (wdtStatus & kWWDT_WarningFlag)
    {
        //log_debug("feed dog.tv:%d.\r\n",WWDT->TV);
        WWDT_ClearStatusFlags(WWDT, kWWDT_WarningFlag);
        WWDT_Refresh(WWDT);
    }
       

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */

#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*
* @brief 看门狗任务
* @param
* @param
* @return 
* @note
*/

void watch_dog_task(void const * argument)
{
    uint8_t read_cnt;
    char    cmd[16];
    uint8_t level;
    uint8_t turn_on = 0;

    wwdt_config_t config;
    uint32_t wdtFreq;
    /* Enable clock of wwdt. */
    CLOCK_EnableClock(kCLOCK_Wwdt);
    /*4 khz*/
    CLOCK_InitWdtOsc(kCLOCK_WdtAnaFreq600KHZ, 60U);

#if !defined(FSL_FEATURE_WWDT_HAS_NO_PDCFG) || (!FSL_FEATURE_WWDT_HAS_NO_PDCFG)
    POWER_DisablePD(kPDRUNCFG_PD_WDT_OSC);
#endif

    /* The WDT divides the input frequency into it by 4 */
    wdtFreq = WDT_CLK_FREQ / 4;
    NVIC_EnableIRQ(WDT_IRQn);
    WWDT_GetDefaultConfig(&config);

    /* Check if reset is due to Watchdog */
    if (WWDT_GetStatusFlags(WWDT) & kWWDT_TimeoutFlag) {
        log_warning("Watchdog reset last.\r\n");
    } else {
        log_warning("reset normal last.\r\n");
    }

    /*
     * Set watchdog feed time constant to approximately 2s
     * Set watchdog warning time to 1000 ticks after feed time constant
     * Set watchdog window time to 1s
     */
    config.timeoutValue = wdtFreq * 2;
    config.warningValue = 1000;
    config.windowValue = wdtFreq * 2;
#if  WDT_ENABLE_RESET  > 0 
    config.enableWatchdogReset = true;
#else
    config.enableWatchdogReset = false;
#endif
    /* Setup watchdog clock frequency(Hz). */
    config.clockFreq_Hz = WDT_CLK_FREQ;

    WWDT_Init(WWDT, &config);

    bsp_sys_led_turn_on();
    turn_on = 1;

    while (1){
        osDelay(WATCH_DOG_TASK_INTERVAL_VALUE);
        if (turn_on) {
            turn_on = 0;
            bsp_sys_led_turn_off();
        } else {
            turn_on = 1;
            bsp_sys_led_turn_on();
        }

        /*设置日志输出等级*/
        read_cnt = log_read(cmd,15);
        cmd[read_cnt] = 0;
        if (strncmp(cmd,"set level ",strlen("set level ")) == 0) {
            level = atoi(cmd + strlen("set level "));
            log_set_level(level);
        }
   
    }
}
   
  
 