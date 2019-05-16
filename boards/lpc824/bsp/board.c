/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

#include "board.h"
#include "log.h"

static void bsp_sys_led_ctrl_pin_init();
/*
* @brief hx711通信接口初始化
* @param 无
* @return 无
* @note
*/
static void bsp_hx711_comm_if_init();




/*板级初始化*/
int board_init()
{
    GPIO_PortInit(GPIO,0);
    BOARD_InitPins();
    BOARD_BootClockRUN();

    bsp_sys_led_ctrl_pin_init();
    bsp_hx711_comm_if_init();

    return 0;
}


/*
* @brief 硬件us级延时
* @param us 延时长度
* @return 无
* @note 不太准确 30MHz
*/
void bsp_hal_delay(uint32_t us)
{
    for (uint32_t i = 0; i < 3 * us ;i++) {
         __ASM("NOP");
    }
}

/*
* @brief hx711通信接口初始化
* @param 无
* @return 无
* @note
*/
static void bsp_hx711_comm_if_init()
{
    /*HX711 dout pin*/
    gpio_pin_config_t config;
    
    config.pinDirection = kGPIO_DigitalInput;
    config.outputLogic = 1;
    GPIO_PinInit(BOARD_HX711_DOUT_GPIO,BOARD_HX711_DOUT_PORT,BOARD_HX711_DOUT_PIN,&config);

    /*HX711 sclk pin*/
    config.pinDirection = kGPIO_DigitalOutput;
    config.outputLogic = 0;
    GPIO_PinInit(BOARD_HX711_SCLK_GPIO,BOARD_HX711_SCLK_PORT,BOARD_HX711_SCLK_PIN,&config);
}


 /*
 * @brief hx711时钟上升沿
 * @param 无
 * @return 无
 * @note
 */

void bsp_hx711_sclk_rise(void)
{
    GPIO_PortSet(BOARD_HX711_SCLK_GPIO,BOARD_HX711_SCLK_PORT,(1 << BOARD_HX711_SCLK_PIN));  
}
 /*
 * @brief hx711时钟下降沿
 * @param 无
 * @return 无
 * @note
 */
void bsp_hx711_sclk_fall(void)
{
    GPIO_PortClear(BOARD_HX711_SCLK_GPIO,BOARD_HX711_SCLK_PORT,(1 << BOARD_HX711_SCLK_PIN));  
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
uint8_t bsp_hx711_read_dout_status(void)
{
    return GPIO_PinRead(BOARD_HX711_DOUT_GPIO,BOARD_HX711_DOUT_PORT,BOARD_HX711_DOUT_PIN);
}


/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static void bsp_sys_led_ctrl_pin_init()
{
    gpio_pin_config_t config =
    {
        kGPIO_DigitalOutput, 0,
    };

    GPIO_PinInit(BOARD_LED_CTRL_GPIO, BOARD_LED_CTRL_PORT, BOARD_LED_CTRL_PIN ,&config);
}


/*led指示灯点亮*/
void bsp_sys_led_turn_on()
{
    GPIO_PortSet(BOARD_LED_CTRL_GPIO,BOARD_LED_CTRL_PORT,(1<<BOARD_LED_CTRL_PIN));  
}
/*led指示灯关闭*/
void bsp_sys_led_turn_off()
{
    GPIO_PortClear(BOARD_LED_CTRL_GPIO,BOARD_LED_CTRL_PORT,(1<<BOARD_LED_CTRL_PIN));
}
/*led指示灯取反状态*/
void bsp_sys_led_toggle()
{
    GPIO_PortToggle(BOARD_LED_CTRL_GPIO,BOARD_LED_CTRL_PORT,(1<<BOARD_LED_CTRL_PIN));
}







