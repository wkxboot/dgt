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

static task_message_t   scale_msg;

typedef struct
{
    uint32_t *buffer;
    uint32_t average;
    uint32_t sum;
    uint32_t write;
    uint32_t mask;
    uint32_t size;
}moving_average_filter_t;

/*滑动平均滤波对象*/
static moving_average_filter_t adc_filter1;
static moving_average_filter_t adc_filter2;

/*kalman滤波对象*/
kalman1_state kalman_filter;

/*标准差*/
static volatile float sd;

#define  ADC_TASK_SAMPLE1_CNT     8
#define  ADC_TASK_SAMPLE2_CNT     32

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
    uint32_t adc,adc1,adc2,adc_filter;
    bool kalman_filter_init = false;
    uint16_t timeout = 0;
    
#if DEBUG_CHART > 0
    /*串口输出，测试用*/
    int rc;
    extern serial_hal_driver_t nxp_serial_uart_hal_driver;
    char chart_buffer[30];
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

    
adc_task_restart: 
    hx711_soft_reset();

    while(1){
        osDelay(ADC_TASK_INTERVAL_VALUE);

        if (hx711_is_ready() != true) {
            timeout += ADC_TASK_INTERVAL_VALUE;
            if (timeout >= ADC_TASK_SAMPLE_TIMEOUT_VALUE) {
                log_error("sensor sample timeout.reset.\r\n");
                timeout  = 0;
                goto adc_task_restart;
            }
            continue;
        }
        timeout  = 0;
        adc = hx711_read_convertion_code(ADC_TASK_GAIN,ADC_TASK_CHANNEL);
        /*两次滑动平均滤波迭代*/     
        adc1 = moving_average_filter_put(&adc_filter1,adc);
        adc2 = moving_average_filter_put(&adc_filter2,adc1);
        /*计算标准差 小于阈值使用kalman滤波，否则使用滑动平均滤波*/
        sd = standard_deviation(&adc_filter2);
        if (sd <= STANDARD_DEVIATION_LIMIT) {
            if (kalman_filter_init == false) {
                kalman1_init(&kalman_filter,adc2,200);
                kalman_filter_init = true;
            }
            adc_filter = (uint32_t)kalman1_filter(&kalman_filter,adc2);
        } else {
            kalman_filter_init = false;
            adc_filter = adc2;
        }
        /*构建电子秤消息体*/
        scale_msg.type = TASK_MSG_ADC_COMPLETE;
        scale_msg.value = adc_filter;
        
        /*图像化数据输出*/
#if  DEBUG_CHART > 0
        size = snprintf(chart_buffer,30,"%d,%d,%d\n",adc,adc1,adc_filter);
        serial_write(&chart_serial_handle,chart_buffer,size);

#endif
        /*发送消息给电子称*/
        osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,ADC_TASK_MSG_PUT_TIMEOUT_VALUE);
        

        /*等待scale 发出restart信号*/
        osSignalWait(ADC_TASK_RESTART_SIGNAL,osWaitForever); 
        }

    }