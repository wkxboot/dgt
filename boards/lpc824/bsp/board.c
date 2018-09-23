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

static void bsp_485_ctrl_pin_init();
static void bsp_sys_led_ctrl_pin_init();
int bsp_ad7190_spi_int(uint8_t spi_port,uint32_t freq);
int nv_init();

/*板级初始化*/
int board_init()
{
 int result;

 BOARD_InitPins();
 BOARD_BootClockPll30M();
 bsp_485_ctrl_pin_init();
 bsp_sys_led_ctrl_pin_init();

 result = bsp_ad7190_spi_int(BSP_AD7190_SPI_PORT,BSP_AD7190_SPI_FREQ);
 if(result != 0){
 return -1; 
 }
result = nv_init();
if(result != 0){
return -1; 
}

return 0;
}

static void bsp_485_ctrl_pin_init()
{
  gpio_pin_config_t config = {
  kGPIO_DigitalOutput, 0,
  };
 GPIO_PortInit(BOARD_INITPINS_RWE_485_CTRL_GPIO, BOARD_INITPINS_RWE_485_CTRL_PORT);
 GPIO_PinInit(BOARD_INITPINS_RWE_485_CTRL_GPIO, BOARD_INITPINS_RWE_485_CTRL_PORT, BOARD_INITPINS_RWE_485_CTRL_PIN, &config);
}

static void bsp_sys_led_ctrl_pin_init()
{
  gpio_pin_config_t config = {
  kGPIO_DigitalOutput, 0,
  };
 GPIO_PortInit(BOARD_INITPINS_LED_CTRL_GPIO, BOARD_INITPINS_LED_CTRL_PORT);
 GPIO_PinInit(BOARD_INITPINS_LED_CTRL_GPIO, BOARD_INITPINS_LED_CTRL_PORT, BOARD_INITPINS_LED_CTRL_PIN ,&config);
}

/*半双工485使能读*/
void bsp_485_enable_read()
{
GPIO_PortSet(BOARD_INITPINS_RWE_485_CTRL_GPIO,BOARD_INITPINS_RWE_485_CTRL_PORT,(1<<BOARD_INITPINS_RWE_485_CTRL_PIN));   
}
/*半双工485使能写*/
void bsp_485_enable_write()
{
GPIO_PortClear(BOARD_INITPINS_RWE_485_CTRL_GPIO,BOARD_INITPINS_RWE_485_CTRL_PORT,(1<<BOARD_INITPINS_RWE_485_CTRL_PIN));   
}

/*led指示灯点亮*/
void bsp_sys_led_turn_on()
{
 GPIO_PortSet(BOARD_INITPINS_LED_CTRL_GPIO,BOARD_INITPINS_LED_CTRL_PORT,(1<<BOARD_INITPINS_LED_CTRL_PIN));  
}
/*led指示灯关闭*/
void bsp_sys_led_turn_off()
{
 GPIO_PortClear(BOARD_INITPINS_LED_CTRL_GPIO,BOARD_INITPINS_LED_CTRL_PORT,(1<<BOARD_INITPINS_LED_CTRL_PIN));
}
/*led指示灯取反状态*/
void bsp_sys_led_toggle()
{
 GPIO_PortToggle(BOARD_INITPINS_LED_CTRL_GPIO,BOARD_INITPINS_LED_CTRL_PORT,(1<<BOARD_INITPINS_LED_CTRL_PIN));
}







