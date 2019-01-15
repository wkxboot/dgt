#ifndef  __HX711_H__
#define  __HX711_H__

#ifdef  __cplusplus
    extern "C" {
#endif


typedef enum
{
    HX711_GAIN_64,
    HX711_GAIN_128
}hx711_gain_t;

typedef enum
{
    HX711_CHN_A,
    HX711_CHN_B
}hx711_chn_t;


#define  HX711_CLK_DELAY        2

/*
* @brief hx711系统复位
* @param
* @param
* @return 
* @note
*/
void hx711_soft_reset(void);

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

bool hx711_is_ready(void);

/*
* @brief 读取一次转换结果并选择下次转换增益和通道
* @param gain下次增益
* @param chn 下次通道
* @return 上次转换结果
* @note (0x800000---0x7FFFFF)补码
*/
uint32_t hx711_read_convertion_code(hx711_gain_t gain,hx711_chn_t chn);









#ifdef  __cplusplus
    }
#endif


#endif