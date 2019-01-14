#include "board.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "scale_task.h"
#include "adc_task.h"
#include "log.h"


osThreadId adc_task_hdl;

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
    sensor_chnnel_t channel_a;
}scale_sensor_t;

static scale_sensor_t scale_sensor;



/*任务参数初始化*/
static void adc_task_param_init()
{

    scale_sensor.channel_a.sample.cnt_max = ADC_TASK_SAMPLE_CNT_MAX;
    scale_sensor.channel_a.sample.average = 0;
    scale_sensor.channel_a.sample.average_pre = 0;
    scale_sensor.channel_a.sample.sum = 0;
    scale_sensor.channel_a.sample.changed =false;
    scale_sensor.channel_a.sample.cnt =0;
    scale_sensor.channel_a.err = false;
    scale_sensor.channel_a.timeout = 0;
    scale_sensor.channel_a.id = 'A';
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
    static float a = 0.6666;
  
    log_assert(sample);
    adc =(uint32_t)( (float)sample->adc_pre * a + (1 - a)*(float)adc);
    sample->adc_pre = adc;
  
    pos = sample->cnt % sample->cnt_max;
  
    if (sample->cnt < sample->cnt_max ){
        sample->adc[pos] = adc;
        sample->sum+=sample->adc[pos];
    }else{
        sample->sum -= sample->adc[pos];
        sample->adc[pos] = adc;
        sample->sum += sample->adc[pos];
        sample->average = sample->sum / sample->cnt_max;
  
        //sample->average =(uint32_t) (sample->average_pre * a + (1 - a)*sample->average);
        if (sample->average_pre !=sample->average){
            sample->average_pre = sample->average;
            sample->changed =true;
        }
    }
    sample->cnt++;
}

static task_message_t   scale_msg;

void adc_task(void const * argument)
{
    uint32_t adc; 
 
adc_task_restart: 
    adc_task_param_init();
    hx711_soft_reset();

    while(1){
        osDelay(ADC_TASK_INTERVAL_VALUE);

        if (hx711_is_ready() != true) {
            scale_sensor.channel_a.timeout += ADC_TASK_INTERVAL_VALUE;
            if (scale_sensor.channel_a.timeout >= ADC_TASK_SAMPLE_TIMEOUT_VALUE) {
                log_error("sensor sample timeout.reset.\r\n");
                goto adc_task_restart;
            }
            continue;
        }
        scale_sensor.channel_a.timeout  = 0;
        adc = hx711_read_convertion_code(ADC_TASK_GAIN,ADC_TASK_CHANNEL);
       
        /*滑动平均 + 一阶滞后滤波*/
        adc_moving_average_plus_one_order_flag_filter(&scale_sensor.channel_a.sample,adc);
  

        if (scale_sensor.channel_a.sample.average != scale_sensor.channel_a.sample.average_pre){
            scale_sensor.channel_a.sample.changed = true;
            scale_sensor.channel_a.sample.average_pre = scale_sensor.channel_a.sample.average;
        }
 
        if (scale_sensor.channel_a.sample.changed == true){
            scale_sensor.channel_a.sample.changed = false;

            scale_msg.type = TASK_MSG_ADC_COMPLETE;
            scale_msg.value = scale_sensor.channel_a.sample.average; 
            osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,ADC_TASK_MSG_PUT_TIMEOUT_VALUE);
  
            /*等待scale 发出restart信号*/
            osSignalWait(ADC_TASK_RESTART_SIGNAL,osWaitForever); 
         }

    }
}