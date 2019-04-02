/*
 * How to set up clock using clock driver functions:
 *
 * 1. Setup clock sources.
 *
 * 2. Set up all dividers.
 *
 * 3. Set up all selectors to provide selected clocks.
 */

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v5.0
processor: LPC824
package_id: LPC824M201JDH20
mcu_data: ksdk2_0
processor_version: 5.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

#include "fsl_power.h"
#include "fsl_clock.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockRUN
called_from_default_init: true
outputs:
- {id: ADC_clock.outFreq, value: 30 MHz}
- {id: I2C0_clock.outFreq, value: 30 MHz}
- {id: I2C1_clock.outFreq, value: 30 MHz}
- {id: I2C2_clock.outFreq, value: 30 MHz}
- {id: I2C3_clock.outFreq, value: 30 MHz}
- {id: IOCON0_clock.outFreq, value: 30 MHz}
- {id: LowPower_clock.outFreq, value: 10 kHz}
- {id: SPI0_clock.outFreq, value: 30 MHz}
- {id: SPI1_clock.outFreq, value: 30 MHz}
- {id: SYSPLL_clock.outFreq, value: 60 MHz}
- {id: System_clock.outFreq, value: 30 MHz}
- {id: UART0_clock.outFreq, value: 60 MHz}
- {id: UART1_clock.outFreq, value: 60 MHz}
- {id: UART2_clock.outFreq, value: 60 MHz}
- {id: WWDT_clock.outFreq, value: 10 kHz}
- {id: divto750k_clock.outFreq, value: 750 kHz}
settings:
- {id: SYSCON.DIV.scale, value: '256'}
- {id: SYSCON.IOCONCLKDIV0.scale, value: '2', locked: true}
- {id: SYSCON.MAINCLKSEL.sel, value: SYSCON.PLL}
- {id: SYSCON.MULT.scale, value: '256'}
- {id: SYSCON.M_MULT.scale, value: '5', locked: true}
- {id: SYSCON.N_DIV.scale, value: '1', locked: true}
- {id: SYSCON.SYSAHBCLKDIV.scale, value: '2', locked: true}
- {id: SYSCON.USARTCLKDIV.scale, value: '1', locked: true}
- {id: SYSCON.WDT_DIV.scale, value: '60', locked: true}
- {id: SYSCON_PDRUNCFG0_PDEN_WDT_OSC_CFG, value: Power_up}
- {id: UARTFRGConfig, value: Enabled}
sources:
- {id: SYSCON.wwdt_osc.outFreq, value: 600 kHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
    /*!< Set up the clock sources */
    /*!< Set up IRC */
    POWER_DisablePD(kPDRUNCFG_PD_IRC_OUT);                   /*!< Ensure IRC OUT is on  */
    POWER_DisablePD(kPDRUNCFG_PD_IRC);                   /*!< Ensure IRC is on  */
    POWER_DisablePD(kPDRUNCFG_PD_SYSOSC);                  /*!< Ensure SYSOSC is on */
    CLOCK_Select(kSYSPLL_From_Irc);                         /*!< set IRC to pll select */
    clock_sys_pll_t config;
    config.src = kCLOCK_SysPllSrcIrc;                           /*!< set pll src  */
    config.targetFreq = 60000000U;                     /*!< set pll target freq */
    CLOCK_InitSystemPll(&config);                           /*!< set parameters */
    CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcSysPll);         /*!< select syspll for main clock */
    POWER_DisablePD(kPDRUNCFG_PD_WDT_OSC);                    /*!< Ensure wwdt osc is on */
    CLOCK_InitWdtOsc(kCLOCK_WdtAnaFreq600KHZ ,(29+1)*2);
    CLOCK_Select(kCLKOUT_From_Irc);                         /*!< select IRC for CLKOUT */
    CLOCK_SetCoreSysClkDiv(2U);
    CLOCK_SetClkDivider(kCLOCK_IOCONCLKDiv0, 2U);  /*!< set IOCOMCLK0 div */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 1U);     /*!< set UART div */
    SYSCON->UARTFRGDIV = SYSCON_UARTFRGDIV_DIV_MASK;            /*!> Set UARTFRGDIV */
    CLOCK_SetUARTFRGMULT(0U);                       /*!< Set UARTFRGMULT */
    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}

