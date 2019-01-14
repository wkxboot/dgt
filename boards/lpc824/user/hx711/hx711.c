/*****************************************************************************
*  HX711驱动                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     hx711.c                                                   
*  @brief    HX711驱动                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.0                                                  
*  @date     2019/1/14                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#include "board.h"
#include "hx711.h"

/*
* @brief hx711系统复位
* @param
* @param
* @return 
* @note
*/
void hx711_soft_reset(void)
{
    /*保持60us以上即可进入断电复位状态*/
    bsp_hx711_sclk_rise();
    bsp_hal_delay(1000);

    bsp_hx711_sclk_fall();
    /*保持1000us以上即可进入正常状态*/
    bsp_hal_delay(1000);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

bool hx711_is_ready(void)
{
    return bsp_hx711_read_dout_status() == 0 ? true : false;
}


/*
* @brief 读取一次转换结果并选择下次转换增益和通道
* @param gain下次增益
* @param chn 下次通道
* @return 上次转换结果
* @note (0x800000---0x7FFFFF)补码
*/
uint32_t hx711_read_convertion_code(hx711_gain_t gain,hx711_chn_t chn)
{
    uint8_t add_cnt;
    uint32_t code = 0;
    /*增益128必须是CHN_A 忽略chn值*/
    if (gain == HX711_GAIN_128) {
        add_cnt = 1;
    }else if (gain == HX711_GAIN_64) {
        if (chn == HX711_CHN_A) {
            add_cnt = 2;
        }else {
            add_cnt = 3;
        }
    }
    /*读出24位数据 MSB---LSB*/
    for (int8_t i = 23; i >= 0; i--) {
        /*时钟rise*/
        bsp_hx711_sclk_rise();
        /*硬件延时*/
        bsp_hal_delay(HX711_CLK_DELAY);
        /*时钟fall*/
        bsp_hx711_sclk_fall();
        /*硬件延时*/
        bsp_hal_delay(HX711_CLK_DELAY);
        /*采样数据*/
        if (bsp_hx711_read_dout_status()) {
            code |= (1 << i);
        }
    }
    /*选择下次增益和通道*/
    for (uint8_t i = 0; i < add_cnt ; i++) {
        /*时钟rise*/
        bsp_hx711_sclk_rise();
        /*硬件延时*/
        bsp_hal_delay(HX711_CLK_DELAY);
        /*时钟fall*/
        bsp_hx711_sclk_fall();
        /*硬件延时*/
        bsp_hal_delay(HX711_CLK_DELAY);
    }
    
    return code;
}






