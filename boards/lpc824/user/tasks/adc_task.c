#include "board.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "scale_task.h"
#include "adc_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[adc_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_INFO 


extern ad7190_io_driver_t ad7190_driver;
osThreadId adc_task_hdl;


#if (ADC_TASK_SAMPLE_CHANNEL_CNT != 1 ) && (ADC_TASK_SAMPLE_CHANNEL_CNT != 2 )
#error "ADC_TASK_SAMPLE_CHANNEL_CNT must be 1 or 2."
#endif

static const uint8_t channel[2] = {ADC_TASK_CHANNEL1_VALUE, ADC_TASK_CHANNEL2_VALUE};

typedef struct
{
uint32_t adc_pre;
uint32_t adc[ADC_TASK_SAMPLE_CNT_MAX];
uint32_t sum;
uint32_t average;
uint32_t average_pre;
uint32_t cnt;
uint16_t cnt_max;
bool     changed;
}adc_sample_t;

typedef struct 
{
adc_sample_t sample;
uint8_t      id;
bool         is_done;
bool         err;
uint16_t     err_cnt;
uint16_t     timeout;
}sensor_chnnel_t;

typedef struct
{
sensor_chnnel_t channel[ADC_TASK_SAMPLE_CHANNEL_CNT];
}scale_sensor_t;

static scale_sensor_t scale_sensor;



/*任务参数初始化*/
static void adc_task_param_init()
{
uint8_t i=0;

for(i =0;i<ADC_TASK_SAMPLE_CHANNEL_CNT;i++){
scale_sensor.channel[i].sample.cnt_max = ADC_TASK_SAMPLE_CNT_MAX;
scale_sensor.channel[i].sample.average = 0;
scale_sensor.channel[i].sample.average_pre = 0;
scale_sensor.channel[i].sample.sum = 0;
scale_sensor.channel[i].sample.changed =false;
scale_sensor.channel[i].sample.cnt =0;
scale_sensor.channel[i].err = false;
scale_sensor.channel[i].id = channel[i];
}
}

/*一阶滞后滤波*/
/*
static void adc_one_order_lag_filter(adc_sample_t *sample,uint32_t adc)
{
 static float a = 0.5;
 sample->average =(uint32_t) (sample->average_pre * a+ (1-a) * adc ); 
 sample->average_pre = sample->average;
 if(sample->average != sample->average_pre){
 sample->changed = true;   
 }
}
*/

/*滑动平均滤波*/
/*
static void adc_moving_average_filter(adc_sample_t *sample,uint32_t adc)
{
  uint16_t pos;
  
  log_assert(sample);
  pos = sample->cnt % sample->cnt_max;
  
  if(sample->cnt < sample->cnt_max ){
  sample->adc[pos] = adc;
  sample->sum+=sample->adc[pos];
  }else{
  sample->sum -= sample->adc[pos];
  sample->adc[pos] = adc;
  sample->sum += sample->adc[pos];
  sample->average = sample->sum / sample->cnt_max;
  if(sample->average_pre !=sample->average){
  sample->average_pre = sample->average;
  sample->changed =true;
  }
  }
  sample->cnt++;
}
*/

/*一阶滞后+滑动平均滤波*/
static void adc_moving_average_plus_one_order_flag_filter(adc_sample_t *sample,uint32_t adc)
{
  uint16_t pos;
  static float a = 0.8866;
  
  log_assert(sample);
  adc =(uint32_t)( (float)sample->adc_pre * a + (1 - a)*(float)adc);
  sample->adc_pre = adc;
  
  pos = sample->cnt % sample->cnt_max;
  
  if(sample->cnt < sample->cnt_max ){
  sample->adc[pos] = adc;
  sample->sum+=sample->adc[pos];
  }else{
  sample->sum -= sample->adc[pos];
  sample->adc[pos] = adc;
  sample->sum += sample->adc[pos];
  sample->average = sample->sum / sample->cnt_max;
  
  //sample->average =(uint32_t) (sample->average_pre * a + (1 - a)*sample->average);
  if(sample->average_pre !=sample->average){
  sample->average_pre = sample->average;
  sample->changed =true;
  }
  }
  sample->cnt++;
}

static task_message_t   scale_msg;

void adc_task(void const * argument)
{
 int result;
 uint8_t channel;
 
 uint8_t ad7190_id;
 uint32_t adc; 
 
 result = ad7190_register_io_driver(&ad7190_driver); 
 log_assert(result == 0);
 
adc_task_restart: 
 adc_task_param_init();

 result = ad7190_init();
 if(result != 0){
 log_error("ad7190 init err.\r\n");
 }
 osDelay(ADC_TASK_START_CONFIG_TIME_VALUE);
 result = ad7190_pwr_down_switch_close(GENERAL_ENABLE);
 if(result != 0){
 log_error("ad7190 pwr dwn sw err.\r\n");
 }
 osDelay(ADC_TASK_START_CONFIG_TIME_VALUE);
 result = ad7190_channel_config(ADC_TASK_CHANNEL_SELECT,ADC_TASK_CHOP,ADC_TASK_UB,ADC_TASK_GAIN);
 if(result != 0){
 log_error("ad7190 chn config err.\r\n");
 }
 osDelay(ADC_TASK_START_CONFIG_TIME_VALUE);
 
 /*暂时不需要内部校准功能*/
 /*
 result = ad7190_internal_zero_scale_calibrate();
 if(result != 0){
 log_error("ad7190 calibrate zero err.\r\n");
 }
 osDelay(ADC_TASK_START_CONFIG_TIME_VALUE);

 result = ad7190_internal_full_scale_calibrate();
 if(result != 0){
 log_error("ad7190 calibrate full err.\r\n");
 }
 osDelay(ADC_TASK_START_CONFIG_TIME_VALUE);
 */
 
 result = ad7190_read_id(&ad7190_id);
 if(result != 0){
 log_error("ad7190 read id err.\r\n");
 }else{
 log_debug("ad7190 id :%d.\r\n",ad7190_id);
 }

 result = ad7190_convert_start(ADC_TASK_AD_MODE,ADC_TASK_AD_SYNC,ADC_TASK_AD_RATE);
 if(result != 0){
 log_error("ad7190 start err.\r\n");
 osDelay(ADC_TASK_RETRY_DELAY_TIMEOUT_VALUE);
 goto adc_task_restart;
 }
 
 while(1){
 osDelay(ADC_TASK_INTERVAL_VALUE);
 ad7190_read_status();
 if(ad7190_is_adc_rdy() == TRUE){
  ad7190_read_conversion_result(&adc);
  channel = ad7190_get_channel();  
  log_debug("channel %d adc is completed.\r\n",channel);
 
  for(uint8_t i=0;i < ADC_TASK_SAMPLE_CHANNEL_CNT;i++){
  if(channel == scale_sensor.channel[i].id){
  /*如果结果没有错误*/
  if(ad7190_is_adc_err() == FALSE){
  scale_sensor.channel[i].err_cnt = 0; 
  scale_sensor.channel[i].timeout = 0; 

  /*滑动平均滤波*/
  //adc_moving_average_filter(&scale_sensor.channel[i].sample,adc);
  /*一阶滞后滤波*/
  //adc_one_order_lag_filter(&scale_sensor.channel[i].sample,adc);
  /*滑动平均 + 一阶滞后滤波*/
  adc_moving_average_plus_one_order_flag_filter(&scale_sensor.channel[i].sample,adc);
  
  scale_sensor.channel[i].is_done = true;
  } else{
  scale_sensor.channel[i].err_cnt++; 
  }
  }else{
  scale_sensor.channel[i].timeout+=ADC_TASK_INTERVAL_VALUE;    
  }
  }
  /*处理错误*/
  for(uint8_t i=0;i < ADC_TASK_SAMPLE_CHANNEL_CNT;i++){
  if(scale_sensor.channel[i].err_cnt >= ADC_TASK_SAMPLE_ERR_CNT_MAX || scale_sensor.channel[i].timeout >= ADC_TASK_SAMPLE_TIMEOUT_VALUE){
   scale_sensor.channel[i].sample.average = ADC_TASK_SAMPLE_ERR_VALUE;
   scale_sensor.channel[i].err =true;  

   if(scale_sensor.channel[i].sample.average != scale_sensor.channel[i].sample.average_pre){
   scale_sensor.channel[i].sample.changed = true;
   scale_sensor.channel[i].sample.average_pre = scale_sensor.channel[i].sample.average;
   log_error("sample error.err_cnt:%d timeout:%d.\r\n",scale_sensor.channel[i].err_cnt,scale_sensor.channel[i].timeout);
   }
   scale_sensor.channel[i].err_cnt = 0;
   scale_sensor.channel[i].timeout = 0;
  }
 
 if(scale_sensor.channel[i].sample.changed == true){
  scale_sensor.channel[i].sample.changed = false;
  scale_msg.type = ADC_SAMPLE_COMPLETED;
  scale_msg.channel = i;

  scale_msg.adc = scale_sensor.channel[i].sample.average; 
  osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,ADC_TASK_MSG_PUT_TIMEOUT_VALUE);
  
  /*等待scale 发出restart信号*/
  osSignalWait(ADC_TASK_RESTART_SIGNAL,ADC_TASK_SIGNAL_WAIT_TIMEOUT_VALUE); 
  }
 
 if(scale_sensor.channel[i].err == true){
   scale_sensor.channel[i].err =false;
   goto adc_task_restart;
 }
 
 } 
 }
 
 
 }
}