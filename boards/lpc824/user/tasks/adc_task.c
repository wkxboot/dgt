#include "board.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "scale_task.h"
#include "adc_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[adc_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


extern ad7190_io_driver_t ad7190_driver;

static uint32_t sample_sum;
static uint32_t sample_average;
static uint32_t sample_average_pre;
static uint8_t  sample_changed = FALSE;

static task_msg_t   scale_msg;

void adc_task(void const * argument)
{
 int result;
 uint8_t ad7190_id;
 uint8_t ready; 
 uint32_t adc; 
 uint16_t sample_cnt;
 uint16_t sample_err_cnt;
 uint16_t sample_timeout;
 
 result = ad7190_register_io_driver(&ad7190_driver); 
 log_assert(result == 0);
 
adc_task_restart: 
 sample_sum = 0;
 sample_cnt = 0;
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
 osDelay(ADC_TASK_INTERVAL_VALUE);
 ready = ad7190_is_adc_rdy();
 if(ready == TRUE){
  if(ad7190_is_adc_err() == FALSE){
  ad7190_read_conversion_result(&adc);
  sample_err_cnt = 0;
  sample_timeout = 0;
  sample_sum+= adc;
  sample_cnt++;
  if(sample_cnt >= ADC_TASK_SAMPLE_CNT_MAX){
   sample_average = sample_sum / sample_cnt;
   log_one_line("sample over.cnt:%d value:%d.",sample_cnt,sample_average);
   sample_sum = 0;
   sample_cnt = 0;  
   if(sample_average != sample_average_pre){
    sample_changed = TRUE; 
    sample_average_pre = sample_average;
    }  
    }
  }else{
  sample_err_cnt++; 
  }
 }else{
 sample_timeout+=ADC_TASK_INTERVAL_VALUE;    
 }
 
 if(sample_err_cnt >= ADC_TASK_SAMPLE_ERR_CNT_MAX || sample_timeout >= ADC_TASK_SAMPLE_TIMEOUT_VALUE){
   sample_average = ADC_TASK_SAMPLE_ERR_VALUE;
   if(sample_average != sample_average_pre){
   sample_changed = TRUE;
   sample_average_pre = sample_average;
   log_error("sample error.err_cnt:%d timeout:%d.\r\n",sample_err_cnt,sample_timeout);
   }
   sample_err_cnt = 0;
   sample_timeout = 0;
 }
 
 if(sample_changed == TRUE){
  sample_changed = FALSE;
  scale_msg.type = ADC_SAMPLE_COMPLETED;
  scale_msg.adc = sample_average;
  osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,ADC_TASK_MSG_PUT_TIMEOUT_VALUE);
  
  /*等待scale 发出restart信号*/
  osSignalWait(ADC_TASK_RESTART_SIGNAL,ADC_TASK_SIGNAL_WAIT_TIMEOUT_VALUE); 
 }
 
 if(sample_average == ADC_TASK_SAMPLE_ERR_VALUE){
 goto adc_task_restart;
 }
 } 
}