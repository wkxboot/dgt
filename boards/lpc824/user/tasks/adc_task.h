#ifndef  __ADC_TASK_H__
#define  __ADC_TASK_H__


extern osThreadId   adc_task_hdl;
extern osMessageQId adc_task_msg_q_id;
void adc_task(void const * argument);


#define  ADC_TASK_SAMPLE_CHANNEL_CNT         1

#define  ADC_TASK_CHANNEL1_VALUE             SR_CHNEL_AIN1_2
#define  ADC_TASK_CHANNEL2_VALUE             SR_CHNEL_AIN3_4

#define  ADC_TASK_CHANNEL_1                  CR_CHNEL_AIN1_2
#define  ADC_TASK_CHANNEL_2                  CR_CHNEL_AIN3_4

#define  ADC_TASK_RETRY_DELAY_TIMEOUT_VALUE  1000
#define  ADC_TASK_START_CONFIG_TIME_VALUE    200

#define  ADC_TASK_INTERVAL_VALUE             1    /*任务运行间隔*/
#define  ADC_TASK_MSG_PUT_TIMEOUT_VALUE      5    /*发送消息超时时间*/
#define  ADC_TASK_SIGNAL_WAIT_TIMEOUT_VALUE  2000 /*等待scale task 回应处理完成的时间*/

#define  ADC_TASK_RESTART_SIGNAL             (1<<0)

#define  ADC_TASK_SAMPLE_TIMEOUT_VALUE       500  /*ADC取样超时时间*/
#define  ADC_TASK_SAMPLE_ERR_CNT_MAX         500  /*ADC取样错误次数最大值*/

#define  ADC_TASK_SAMPLE_CNT_MAX             40   /*ADC取样平均值*/
#define  ADC_TASK_SAMPLE_ERR_VALUE           0xFFFFFFFFUL  /*ADC取样错误码*/





#define  ADC_TASK_AD_MODE                    MR_MODE_CONTINUE
#define  ADC_TASK_AD_SYNC                    GENERAL_ENABLE
#define  ADC_TASK_AD_RATE                    100

#if      ADC_TASK_SAMPLE_CHANNEL_CNT == 1
#define  ADC_TASK_CHANNEL_SELECT             ADC_TASK_CHANNEL_1
#elif    ADC_TASK_SAMPLE_CHANNEL_CNT == 2
#define  ADC_TASK_CHANNEL_SELECT             ADC_TASK_CHANNEL_1 | ADC_TASK_CHANNEL_2
#endif

#define  ADC_TASK_CHOP                       GENERAL_ENABLE
#define  ADC_TASK_UB                         GENERAL_DISABLE
#define  ADC_TASK_GAIN                       CR_GAIN_128











#endif