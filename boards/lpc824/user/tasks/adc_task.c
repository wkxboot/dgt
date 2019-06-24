#include "board.h"
#include "cmsis_os.h"
#include "utils.h"
#include "printf.h"
#include "task_msg.h"
#include "scale_task.h"
#include "adc_task.h"
#include "kalman_filter.h"
#include "math.h"
#include "log.h"


osThreadId adc_task_hdl;
 /** @brief ADC消息体*/
static task_message_t   scale_msg;

 /** @brief ADC转换结果结构*/
typedef struct
{
    uint32_t value;
    uint8_t is_err;
    uint8_t is_complete;
    uint16_t timeout;
}adc_conversion_t;
 /** @brief ADC过滤器结构体*/
typedef struct
{
    uint32_t *buffer;
    uint32_t average;
    uint32_t sum;
    uint32_t write;
    uint32_t mask;
    uint32_t size;
}moving_average_filter_t;

/** adc滤波结果对象*/
static adc_conversion_t adc_conversion;
/**滑动平均滤波对象*/
static moving_average_filter_t adc_filter1;
static moving_average_filter_t adc_filter2;

/*kalman滤波对象*/
kalman1_state kalman_filter;

/*标准差*/
static volatile float sd;

#define  ADC_TASK_SAMPLE1_CNT     32
#define  ADC_TASK_SAMPLE2_CNT     16

static uint32_t buffer1[ADC_TASK_SAMPLE1_CNT];
static uint32_t buffer2[ADC_TASK_SAMPLE2_CNT];

/*滑动平均滤波初始化*/
static int moving_average_filter_init(moving_average_filter_t *filter,uint32_t *buffer,uint8_t size)
{
    log_assert(filter);
    log_assert(buffer);
    log_assert(IS_POWER_OF_TWO(size));
    
    filter->buffer = buffer;
    filter->size = size;
    filter->mask = size - 1;
    filter->write = 0;
    filter->sum = 0;
    filter->average = 0;
  
    return 0;
}

/*滑动平均滤波*/
static uint32_t moving_average_filter_put(moving_average_filter_t *filter,uint32_t adc)
{
    if (filter->write >= filter->size) {            
        filter->sum -= filter->buffer[filter->write & filter->mask];    
    }
        
    filter->buffer[filter->write & filter->mask] = adc;     
    filter->write ++;
    filter->sum += adc;  
    filter->average = filter->sum / filter->size;
    return  filter->average;
}


/*标准方差*/
static float standard_deviation(moving_average_filter_t *filter)
{
    float sum = 0;
    float delta;
    float variance,sd;
    
    for (uint8_t i = 0;i < filter->size;i ++) {
        delta = filter->buffer[i] >= filter->average ? filter->buffer[i] - filter->average : filter->average - filter->buffer[i];
        sum += delta * delta;
    }
    /*方差*/
    variance = sum / (float)filter->size;
    /*开平方*/
    sd = sqrt(variance);
    return  sd;
}

/*图像化数据输出串口初始化*/
#if DEBUG_CHART > 0
#include "nxp_serial_uart_hal_driver.h"

static serial_handle_t chart_serial_handle;
static uint8_t recv_buffer[CHART_RX_BUFFER_SIZE];
static uint8_t send_buffer[CHART_TX_BUFFER_SIZE];
/*串口中断处理*/
void USART0_IRQHandler()
{
    if (chart_serial_handle.init) {
        nxp_serial_uart_hal_isr(&chart_serial_handle);
    }
}

#endif



void adc_task(void const * argument)
{
    uint32_t adc,adc1,adc2;
    bool kalman_filter_init = false;
    
#if DEBUG_CHART > 0
    /*串口输出，测试用*/
    int rc;
    extern serial_hal_driver_t nxp_serial_uart_hal_driver;
    char chart_buffer[60];
    uint8_t size;
    rc = serial_create(&chart_serial_handle,recv_buffer,CHART_RX_BUFFER_SIZE,send_buffer,CHART_TX_BUFFER_SIZE);
    log_assert(rc == 0);
    rc = serial_register_hal_driver(&chart_serial_handle,&nxp_serial_uart_hal_driver);
    log_assert(rc == 0);
 
    rc = serial_open(&chart_serial_handle,
                    CHART_SERIAL_PORT,
                    CHART_SERIAL_BAUDRATES,
                    CHART_SERIAL_DATABITS,
                    CHART_SERIAL_STOPBITS);
    log_assert(rc == 0); 
    log_debug("chart serial ok.\r\n");
    
    serial_flush(&chart_serial_handle);
#endif

    /*三次滑动平均滤波迭代初始化*/
    moving_average_filter_init(&adc_filter1,buffer1,ADC_TASK_SAMPLE1_CNT);
    moving_average_filter_init(&adc_filter2,buffer2,ADC_TASK_SAMPLE2_CNT);

    hx711_soft_reset();

    while(1){
        osDelay(ADC_TASK_INTERVAL_VALUE);
        /*转换完成标志未就绪*/
        if (hx711_is_ready() != true) {
            adc_conversion.timeout += ADC_TASK_INTERVAL_VALUE;
            if (adc_conversion.timeout >= ADC_TASK_SAMPLE_TIMEOUT_VALUE) {
                log_error("sensor sample timeout.reset.\r\n");
                hx711_soft_reset();
                adc_conversion.is_complete = 1;
                adc_conversion.is_err = 1;          
            }
        } else { /*转换完成标志就绪*/
            adc_conversion.is_err = 0;
            adc_conversion.is_complete = 1;
            adc = hx711_read_convertion_code(ADC_TASK_GAIN,ADC_TASK_CHANNEL);
            /*两次滑动平均滤波迭代*/     
            adc1 = moving_average_filter_put(&adc_filter1,adc);
            adc2 = moving_average_filter_put(&adc_filter2,adc1);
            /*计算标准差 小于阈值使用kalman滤波，否则使用滑动平均滤波*/
            sd = standard_deviation(&adc_filter2);
            if (sd <= STANDARD_DEVIATION_LIMIT) {
                if (kalman_filter_init == false) {
                    kalman1_init(&kalman_filter,adc2,100.00);
                    kalman_filter_init = true;
                }
                adc_conversion.value = (uint32_t)kalman1_filter(&kalman_filter,adc2);
            } else {
                kalman_filter_init = false;
                adc_conversion.value = adc2;
            }
   
            /*图像化数据输出*/
#if  DEBUG_CHART > 0
            uint32_t time = osKernelSysTick();
            size = snprintf(chart_buffer,60,"%d,%d;%d,%d;%d,%d\r\n",time,adc,time,adc1,time,adc_conversion.value);
            serial_write(&chart_serial_handle,chart_buffer,size);
#endif
        }

        /*判断转换结果*/
        if (adc_conversion.is_complete) {
            adc_conversion.timeout  = 0;
            adc_conversion.is_complete = 0;
            if (adc_conversion.is_err) {
                adc_conversion.is_err = 0;
                /*构建adc错误消息体*/
                scale_msg.type = TASK_MSG_ADC_ERROR;
            } else { /*构建adc完成消息体*/
                scale_msg.type = TASK_MSG_ADC_COMPLETE;
                scale_msg.value = adc_conversion.value;
            }
            /*发送消息给电子称*/
            osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,ADC_TASK_MSG_PUT_TIMEOUT_VALUE);
            /*等待scale 发出restart信号*/
            osSignalWait(ADC_TASK_RESTART_SIGNAL,osWaitForever); 
        }

    }
}