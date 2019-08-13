#ifndef  __ADC_TASK_H__
#define  __ADC_TASK_H__


extern osThreadId   adc_task_hdl;
extern osMessageQId adc_task_msg_q_id;
void adc_task(void const * argument);

#define  ADC_TASK_INTERVAL_VALUE             2    /*任务运行间隔*/
#define  ADC_TASK_PUT_MSG_TIMEOUT            5    /*发送消息超时时间*/


#define  ADC_TASK_RESTART_SIGNAL             (1 << 0)

#define  ADC_TASK_SAMPLE_TIMEOUT_VALUE       20  /*ADC取样超时时间*/

#define  ADC_TASK_CHANNEL                    HX711_CHN_A
#define  ADC_TASK_GAIN                       HX711_GAIN_128


    

#define  DEBUG_CHART                         0
#define  CHART_RX_BUFFER_SIZE                4
#define  CHART_TX_BUFFER_SIZE                64
#define  CHART_PORT                          0
#define  CHART_BAUD_RATES                    115200
#define  CHART_DATA_BITS                     8
#define  CHART_STOP_BITS                     1



#endif