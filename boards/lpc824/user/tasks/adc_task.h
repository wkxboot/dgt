#ifndef  __ADC_TASK_H__
#define  __ADC_TASK_H__


extern osThreadId   adc_task_hdl;
extern osMessageQId adc_task_msg_q_id;
void adc_task(void const * argument);

#define  STANDARD_DEVIATION_LIMIT            1200 /*开始使用卡尔曼滤波算法的最小标准差*/

#define  ADC_TASK_INTERVAL_VALUE             2    /*任务运行间隔*/
#define  ADC_TASK_MSG_PUT_TIMEOUT_VALUE      5    /*发送消息超时时间*/


#define  ADC_TASK_RESTART_SIGNAL             (1 << 0)

#define  ADC_TASK_SAMPLE_TIMEOUT_VALUE       20  /*ADC取样超时时间*/
#define  ADC_TASK_SAMPLE_ERR_CNT_MAX         500  /*ADC取样错误次数最大值*/
#define  ADC_TASK_SAMPLE_ERR_VALUE           0xFFFFFFFFUL  /*ADC取样错误码*/


#define  ADC_TASK_CHANNEL                    HX711_CHN_A
#define  ADC_TASK_GAIN                       HX711_GAIN_128


    

#define  DEBUG_CHART                         1
#define  CHART_RX_BUFFER_SIZE                4
#define  CHART_TX_BUFFER_SIZE                32
#define  CHART_SERIAL_PORT                   0
#define  CHART_SERIAL_BAUDRATES              115200
#define  CHART_SERIAL_DATABITS               8
#define  CHART_SERIAL_STOPBITS               1






#endif