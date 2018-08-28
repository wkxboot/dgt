#include "board.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "scale_task.h"
#include "adc_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[adc_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


extern ad7190_io_driver_t ad7190_driver;
osThreadId adc_task_hdl;

typedef struct
{
uint32_t adc[ADC_TASK_SAMPLE_CNT_MAX];
uint32_t sum;
uint32_t average;
uint32_t average_pre;
uint16_t err_cnt;
uint16_t cnt;
uint16_t cnt_max;
uint16_t timeout;

bool     changed;
bool     err;
}adc_sample_t;

adc_sample_t sample = {
.sum = 0,
.average =0,
.average_pre =0,
.err_cnt =0,
.timeout =0,
.err = false,
.changed = false,
.cnt = 0,
.cnt_max = ADC_TASK_SAMPLE_CNT_MAX
};



static void adc_moving_average_filter(uint32_t adc)
{
  uint16_t pos;
  pos = sample.cnt % sample.cnt_max;
  
  if(sample.cnt < sample.cnt_max ){
  sample.adc[pos] = adc;
  sample.sum+=sample.adc[pos];
  }else{
  sample.sum -= sample.adc[pos];
  sample.adc[pos] = adc;
  sample.sum += sample.adc[pos];
  sample.average = sample.sum / sample.cnt_max;
  if(sample.average_pre !=sample.average){
  sample.average_pre = sample.average;
  sample.changed =true;
  }
  }
  sample.cnt++;
}



static task_msg_t   scale_msg;

void adc_task(void const * argument)
{
 int result;
 uint8_t ad7190_id;
 uint8_t ready; 
 uint32_t adc; 
 
 result = ad7190_register_io_driver(&ad7190_driver); 
 log_assert(result == 0);
 
adc_task_restart: 
 sample.sum = 0;
 sample.cnt = 0;
 result = ad7190_init();
 if(result != 0){
 log_error("ad7190 init err.\r\n");
 }
 
 result = ad7190_pwr_down_switch_close(GENERAL_ENABLE);
 if(result != 0){
 log_error("ad7190 pwr dwn sw err.\r\n");
 }
 result = ad7190_channel_config(ADC_TASK_CHNNEL,ADC_TASK_CHOP,ADC_TASK_UB,ADC_TASK_GAIN);
 if(result != 0){
 log_error("ad7190 chn config err.\r\n");
 }
 result = ad7190_internal_zero_scale_calibrate();
 if(result != 0){
 log_error("ad7190 calibrate zero err.\r\n");
 }
 result = ad7190_internal_full_scale_calibrate();
 if(result != 0){
 log_error("ad7190 calibrate full err.\r\n");
 }
 result = ad7190_read_id(&ad7190_id);
 if(result != 0){
 log_error("ad7190 read id err.\r\n");
 }else{
 log_debug("ad7190 id :%d.\r\n",ad7190_id);
 }

 
 while(1){
 result = ad7190_convert_start(ADC_TASK_AD_MODE,ADC_TASK_AD_SYNC,ADC_TASK_AD_RATE);
 if(result != 0){
 log_error("ad7190 start err.\r\n");
 }
 osDelay(ADC_TASK_INTERVAL_VALUE);
 ready = ad7190_is_adc_rdy();
 if(ready == TRUE){
  if(ad7190_is_adc_err() == FALSE){
  ad7190_read_conversion_result(&adc);
  sample.err_cnt = 0;
  sample.timeout = 0;
  sample.err = false;
  
  /*滑动平均滤波*/
  adc_moving_average_filter(adc);
  
  }else{
  sample.err_cnt++; 
  }
 }else{
 sample.timeout+=ADC_TASK_INTERVAL_VALUE;    
 }
  
 if(sample.err_cnt >= ADC_TASK_SAMPLE_ERR_CNT_MAX || sample.timeout >= ADC_TASK_SAMPLE_TIMEOUT_VALUE){
   sample.average = ADC_TASK_SAMPLE_ERR_VALUE;
   sample.err =true;
   if(sample.average != sample.average_pre){
   sample.changed = true;
   sample.average_pre = sample.average;
   log_error("sample error.err_cnt:%d timeout:%d.\r\n",sample.err_cnt,sample.timeout);
   }
   sample.err_cnt = 0;
   sample.timeout = 0;
 }
 
 if(sample.changed == true){
  sample.changed = false;
  scale_msg.type = ADC_SAMPLE_COMPLETED;
  scale_msg.adc = sample.average;
  osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,ADC_TASK_MSG_PUT_TIMEOUT_VALUE);
  
  /*等待scale 发出restart信号*/
  osSignalWait(ADC_TASK_RESTART_SIGNAL,ADC_TASK_SIGNAL_WAIT_TIMEOUT_VALUE); 
 }
 
 if(sample.err == true){
   sample.err =false;
   goto adc_task_restart;
 }
 } 
}