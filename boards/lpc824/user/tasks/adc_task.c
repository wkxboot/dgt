#include "board.h"
#include "cmsis_os.h"
#include "printf.h"
#include "scale_task.h"
#include "adc_task.h"
#include "circle_buffer.h"
#include "kalman_filter.h"
#include "math.h"
#include "log.h"


osThreadId adc_task_hdl;
 /** @brief ADC消息体*/
static scale_task_message_t   scale_msg;
/*标准差*/
static volatile float sd;

#define  ADC_TASK_SAMPLE1_CNT     16
#define  ADC_TASK_SAMPLE2_CNT     16

static uint32_t buffer1[ADC_TASK_SAMPLE1_CNT];
static uint32_t buffer2[ADC_TASK_SAMPLE2_CNT];

enum {
    STEP_SEARCH_HIGH,
    STEP_SEARCH_LOW
};

typedef struct
{
    uint32_t high;
    uint32_t low;
    uint32_t average;
    uint8_t step;
    uint8_t is_stable;
}data_wave_t;

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
    uint8_t size;
    uint8_t offset;
    uint8_t current_size;
}moving_average_filter_t;

/** adc滤波结果对象*/
static adc_conversion_t adc_conversion;
/**滑动平均滤波对象*/
static data_wave_t data_wave1;
static data_wave_t data_wave2;
static data_wave_t data_wave3;

moving_average_filter_t filter1;
moving_average_filter_t filter2;
kalman1_state kalman_filter;

/**
* @brief
* @details
* @param
* @param
* @return
* @attention
* @note
*/
static void moving_average_filter_init(moving_average_filter_t *filter,uint32_t *buffer,uint8_t size)
{
    log_assert_null_ptr(filter);
    log_assert_null_ptr(buffer);

    filter->buffer = buffer;
    filter->size = size;
    filter->current_size = 0;
    filter->offset = 0;
    filter->average = 0;
    filter->sum = 0;
}
/**
* @brief
* @details
* @param
* @param
* @return
* @attention
* @note
*/
static void moving_average_filter_put(moving_average_filter_t *filter,uint32_t adc)
{
    if (filter->current_size >= filter->size) {
        /*如果队列已经是满的，减去队首的数据*/
        filter->sum -= filter->buffer[filter->offset];
    } else {
        filter->current_size ++;
    }
    /*把数据放入对队尾*/
    filter->buffer[filter->offset ++] = adc;
    if (filter->offset >= filter->size) {
         filter->offset = 0;
    }
    filter->sum += adc;
    filter->average = filter->sum / filter->current_size;
}


/*返回1找到了波峰和波谷 返回0没有找到*/
static int data_wave_put(data_wave_t *wave,uint32_t adc)
{
    /*判断波峰*/
    if (wave->step == STEP_SEARCH_HIGH) {
        if (adc < wave->high) {
            wave->step = STEP_SEARCH_LOW;
            wave->low = adc;
        } else {
            wave->high = adc;
        }
    /*判断波谷*/
    } else {
        if (adc > wave->low) {
            /*计算平均值*/
            wave->average = (wave->high + wave->low) / 2;
            wave->is_stable = 1;
            wave->step = STEP_SEARCH_HIGH;
            wave->high = adc;
            return 1;
        } else {
            wave->low = adc;
        }
    }
    return 0;
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
    int rc;
    uint32_t adc;

#if DEBUG_CHART > 0
    /*串口输出，测试用*/
    int rc;
    extern serial_hal_driver_t nxp_serial_uart_hal_driver;
    char chart_buffer[60];
    uint8_t size;
    rc = serial_create(&chart_serial_handle,recv_buffer,CHART_RX_BUFFER_SIZE,send_buffer,CHART_TX_BUFFER_SIZE);
    log_assert_null_ptr(rc == 0);
    rc = serial_register_hal_driver(&chart_serial_handle,&nxp_serial_uart_hal_driver);
    log_assert_null_ptr(rc == 0);
 
    rc = serial_open(&chart_serial_handle,
                    CHART_SERIAL_PORT,
                    CHART_SERIAL_BAUDRATES,
                    CHART_SERIAL_DATABITS,
                    CHART_SERIAL_STOPBITS);
    log_assert_null_ptr(rc == 0); 
    log_debug("chart serial ok.\r\n");
    
    xuart_clear(&chart_serial_handle);
#endif
    moving_average_filter_init(&filter1,buffer1,ADC_TASK_SAMPLE1_CNT);
    moving_average_filter_init(&filter2,buffer2,ADC_TASK_SAMPLE2_CNT);
    hx711_soft_reset();

    while(1) {
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
        } else { 
            adc_conversion.is_err = 0;
            adc_conversion.timeout = 0;
            adc = hx711_read_convertion_code(ADC_TASK_GAIN,ADC_TASK_CHANNEL);

            /*1次滑动平均滤波迭代*/     
            moving_average_filter_put(&filter1,adc);
            moving_average_filter_put(&filter2,filter1.average);  
            rc = data_wave_put(&data_wave1,filter2.average);
            if (rc == 1) {
                /*转换完成标志就绪*/
                adc_conversion.is_complete = 1;
                adc_conversion.value = data_wave1.average;
            }
            /*图像化数据输出*/
#if  DEBUG_CHART > 0
            uint32_t time = osKernelSysTick();
            size = snprintf(chart_buffer,60,"%d,%d;%d,%d\r\n",time,data_wave1.average,time,data_wave2.average);
            serial_write(&chart_serial_handle,chart_buffer,size);
#endif
        }

        /*判断转换结果*/
        if (adc_conversion.is_complete) {
            adc_conversion.is_complete = 0;
            if (adc_conversion.is_err) {
                adc_conversion.is_err = 0;
                adc_conversion.timeout = 0;  
                /*构建adc错误消息体*/
                scale_msg.head.id = SCALE_TASK_MSG_ADC_ERROR;
            } else { /*构建adc完成消息体*/
                scale_msg.head.id = SCALE_TASK_MSG_ADC_COMPLETE;
                scale_msg.content.adc = adc_conversion.value;
            }
            /*发送消息给电子称*/
            log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&scale_msg,ADC_TASK_PUT_MSG_TIMEOUT) == pdPASS);
            /*等待scale 发出restart信号*/
            osSignalWait(ADC_TASK_RESTART_SIGNAL,osWaitForever); 
        }

    }
}