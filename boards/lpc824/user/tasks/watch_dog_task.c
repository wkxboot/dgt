#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "watch_dog_task.h"
#if !defined(FSL_FEATURE_WWDT_HAS_NO_PDCFG) || (!FSL_FEATURE_WWDT_HAS_NO_PDCFG)
#include "fsl_power.h"
#endif
#include "kalman_filter.h"
#include "log.h"
 
#define  WDT_ENABLE           1
   
#define WDT_CLK_FREQ CLOCK_GetFreq(kCLOCK_WdtOsc)

osThreadId   watch_dog_task_hdl;
/*
* @brief 看门狗任务
* @param
* @param
* @return 
* @note
*/

void watch_dog_task(void const * argument)
{
    uint8_t  read_cnt;
    char     cmd[16];
    uint8_t  level;

#if  WDT_ENABLE  > 0 
    wwdt_config_t config;
    uint32_t wdtFreq;
    /* Enable clock of wwdt. */
    CLOCK_EnableClock(kCLOCK_Wwdt);

    CLOCK_InitWdtOsc(kCLOCK_WdtAnaFreq600KHZ, 2U);

#if !defined(FSL_FEATURE_WWDT_HAS_NO_PDCFG) || (!FSL_FEATURE_WWDT_HAS_NO_PDCFG)
    POWER_DisablePD(kPDRUNCFG_PD_WDT_OSC);
#endif

    /* The WDT divides the input frequency into it by 4 */
    wdtFreq = WDT_CLK_FREQ / 4;
    WWDT_GetDefaultConfig(&config);

    /* Check if reset is due to Watchdog */
    if (WWDT_GetStatusFlags(WWDT) & kWWDT_TimeoutFlag)
    {
        log_warning("Watchdog reset last.\r\n");
    }

    /*
     * Set watchdog feed time constant to approximately 2s
     * Set watchdog warning time to 512 ticks after feed time constant
     * Set watchdog window time to 1s
     */
    config.timeoutValue = wdtFreq * 2;
    config.warningValue = 512;
    config.windowValue = wdtFreq * 1;
    /* Configure WWDT to reset on timeout */
    config.enableWatchdogReset = true;
    /* Setup watchdog clock frequency(Hz). */
    config.clockFreq_Hz = WDT_CLK_FREQ;

    WWDT_Init(WWDT, &config);
#endif

    bsp_sys_led_turn_on();

    while (1){
        osDelay(WATCH_DOG_TASK_INTERVAL_VALUE);
        bsp_sys_led_toggle();
#if WDT_ENABLE > 0
        /*看门狗*/
        if (WWDT->TV < WWDT->WINDOW) {
            WWDT_Refresh(WWDT);
            log_debug("feed dog.\r\n");
        }
#endif

        /*设置日志输出等级*/
        read_cnt = log_read(cmd,15);
        cmd[read_cnt] = 0;
        if (strncmp(cmd,"set level ",strlen("set level ")) == 0) {
            level = atoi(cmd + strlen("set level "));
            log_set_level(level);
        }
   
    }
}
   
  
 