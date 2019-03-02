#include "board.h"
#include "cmsis_os.h"
#include "utils.h"
#include "printf.h"
#include "task_msg.h"
#include "scale_task.h"
#include "adc_task.h"
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
 
static moving_average_filter_t adc_filter1;
static moving_average_filter_t adc_filter2;
static moving_average_filter_t adc_filter3;

#define  ADC_TASK_SAMPLE1_CNT     32
#define  ADC_TASK_SAMPLE2_CNT     16
#define  ADC_TASK_SAMPLE3_CNT     16

static uint32_t buffer1[ADC_TASK_SAMPLE1_CNT];
static uint32_t buffer2[ADC_TASK_SAMPLE2_CNT];
static uint32_t buffer3[ADC_TASK_SAMPLE3_CNT];


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


static bool is_moving_average_filter_buffer_full(moving_average_filter_t *filter)
{
    log_assert(filter);
    return filter->write >= filter->size ? true : false;
}

static uint32_t moving_average_filter_put(moving_average_filter_t *filter,uint32_t adc)
{
    if (is_moving_average_filter_buffer_full(filter) == false) {     
        filter->buffer[filter->write & filter->mask] = adc;
    } else {
        filter->sum -= filter->buffer[filter->write & filter->mask];
        filter->buffer[filter->write & filter->mask] = adc;    
    }
    filter->write ++;
    filter->sum += adc;  
    filter->average = filter->sum / filter->size;
    return  filter->average;
}


#if DEBUG_CHART > 0

int chart_serial_handle;
#include "nxp_serial_uart_hal_driver.h"
/*串口中断处理*/
void USART0_IRQHandler()
{
  nxp_serial_uart_hal_isr(chart_serial_handle);
}

#endif


void adc_task(void const * argument)
{
    uint32_t adc,adc1,adc2,adc3;
    uint16_t timeout = 0;
    
#if DEBUG_CHART > 0
    /*串口输出，测试用*/
    int rc;
    extern serial_hal_driver_t nxp_serial_uart_hal_driver;
    char chart_buffer[30];
    uint8_t size;
    rc = serial_create(&chart_serial_handle,CHART_RX_BUFFER_SIZE,CHART_TX_BUFFER_SIZE);
    log_assert(rc == 0);
    rc = serial_register_hal_driver(chart_serial_handle,&nxp_serial_uart_hal_driver);
    log_assert(rc == 0);
 
    rc = serial_open(chart_serial_handle,
                    CHART_SERIAL_PORT,
                    CHART_SERIAL_BAUDRATES,
                    CHART_SERIAL_DATABITS,
                    CHART_SERIAL_STOPBITS);
    log_assert(rc == 0); 
    log_debug("chart serial ok.\r\n");
    
    serial_flush(chart_serial_handle);
#endif

    
    moving_average_filter_init(&adc_filter1,buffer1,ADC_TASK_SAMPLE1_CNT);
    moving_average_filter_init(&adc_filter2,buffer2,ADC_TASK_SAMPLE2_CNT);
    moving_average_filter_init(&adc_filter3,buffer3,ADC_TASK_SAMPLE3_CNT);
    
adc_task_restart: 
    hx711_soft_reset();

    while(1){
        osDelay(ADC_TASK_INTERVAL_VALUE);

        if (hx711_is_ready() != true) {
            timeout += ADC_TASK_INTERVAL_VALUE;
            if (timeout >= ADC_TASK_SAMPLE_TIMEOUT_VALUE) {
                log_error("sensor sample timeout.reset.\r\n");
                goto adc_task_restart;
            }
            continue;
        }
        timeout  = 0;
        adc = hx711_read_convertion_code(ADC_TASK_GAIN,ADC_TASK_CHANNEL);
       
        
        adc1 = moving_average_filter_put(&adc_filter1,adc);
        
        adc2 = moving_average_filter_put(&adc_filter2,adc1);
        
        adc3 = moving_average_filter_put(&adc_filter3,adc2);
               

        scale_msg.type = TASK_MSG_ADC_COMPLETE;
        scale_msg.value = adc3;
        
        osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,ADC_TASK_MSG_PUT_TIMEOUT_VALUE);
        
#if  DEBUG_CHART > 0
        size = snprintf(chart_buffer,30,"%d,%d,%d\n",adc1,adc2,adc3);
        serial_write(chart_serial_handle,chart_buffer,size);

#endif
        /*等待scale 发出restart信号*/
        osSignalWait(ADC_TASK_RESTART_SIGNAL,osWaitForever); 
        }

    }