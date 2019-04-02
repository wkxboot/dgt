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

#ifndef _BOARD_H_
#define _BOARD_H_

#include <stdint.h>
#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_spi.h"
#include "fsl_i2c.h"
#include "fsl_usart.h"
#include "fsl_wwdt.h"
#include "pin_mux.h"
#include "serial.h"
#include "hx711.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BOARD_LED_CTRL_GPIO      GPIO                          /*!<@brief GPIO device name: GPIO */
#define BOARD_LED_CTRL_PORT      0U                            /*!<@brief PORT device name: 0U */
#define BOARD_LED_CTRL_PIN       23U                           /*!<@brief 0U pin index: 23 */

#define BOARD_HX711_DOUT_GPIO    GPIO                          /*!<@brief GPIO device name: GPIO */
#define BOARD_HX711_DOUT_PORT    0U                            /*!<@brief PORT device name: 0U */
#define BOARD_HX711_DOUT_PIN     11U                           /*!<@brief 0U pin index: 23 */


#define BOARD_HX711_SCLK_GPIO    GPIO                          /*!<@brief GPIO device name: GPIO */
#define BOARD_HX711_SCLK_PORT    0U                            /*!<@brief PORT device name: 0U */
#define BOARD_HX711_SCLK_PIN     10U                           /*!<@brief 0U pin index: 23 */

int board_init();
void bsp_sys_led_turn_on();
void bsp_sys_led_turn_off();
void bsp_sys_led_toggle();
/*
* @brief 硬件us级延时
* @param us 延时长度
* @return 无
* @note 不准确
*/
void bsp_hal_delay(uint32_t us);
 /*
 * @brief hx711时钟上升沿
 * @param 无
 * @return 无
 * @note
 */

void bsp_hx711_sclk_rise(void);
 /*
 * @brief hx711时钟下降沿
 * @param 无
 * @return 无
 * @note
 */
void bsp_hx711_sclk_fall(void);

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
uint8_t bsp_hx711_read_dout_status(void);


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
