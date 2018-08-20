#include "board.h"
#include "cmsis_os.h"
#include "adc_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[adc_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


extern ad7190_io_driver_t ad7190_driver;


void adc_task(void const * argument)
{
 int result;
 uint8_t ad7190_id;
 uint8_t ready; 
 int32_t adc; 
 
 result = ad7190_register_io_driver(&ad7190_driver); 
 log_assert(result == 0);
 
ad7190_init: 
 result = ad7190_init();
 log_assert(result == 0);
 
 result = ad7190_pwr_down_switch_close(GENERAL_ENABLE);
 log_assert(result == 0);
 result = ad7190_channel_config(ADC_TASK_CHNNEL,ADC_TASK_CHOP,ADC_TASK_UB,ADC_TASK_GAIN);
 log_assert(result == 0);
 result = ad7190_internal_zero_scale_calibrate();
 log_assert(result == 0);
 result = ad7190_internal_full_scale_calibrate();
 log_assert(result == 0);
  
 result = ad7190_read_id(&ad7190_id);
 log_assert(result == 0); 
 log_debug("ad7190 id :%d.\r\n",ad7190_id);
 
 result = ad7190_convert_start(ADC_TASK_AD_MODE,ADC_TASK_AD_SYNC,ADC_TASK_AD_RATE);
 log_assert(result == 0); 
 
 while(1){
 osDelay(ADC_TASK_INTERVAL);
 ready = ad7190_is_adc_rdy();
 if(ready == TRUE){
 result = ad7190_read_conversion_result(&adc);
 
 
 
 
 } 
}