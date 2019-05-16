#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "firmware_version.h"
#include "sensor_id.h"
#include "device_env.h"
#include "adc_task.h"
#include "protocol_task.h"
#include "scale_task.h"
#include "math.h"
#include "log.h"

osThreadId   scale_task_hdl;
osMessageQId scale_task_msg_q_id;

typedef struct
{
    float  a;
    float  b;
    int16_t tare_weight;
    uint8_t addr;
}scale_nv_param_t;

typedef struct
{
    scale_nv_param_t nv_param;
    uint32_t cur_adc;
    int16_t  net_weight;
    int16_t  gross_weight;
}scale_t;

typedef struct
{
    scale_t          scale; 
}digital_scale_t;

static digital_scale_t digital_scale;


static void scale_task_param_init()
{
    digital_scale.scale.nv_param.addr = SCALE_ADDR_DEFAULT;

    digital_scale.scale.nv_param.a = SCALE_TASK_DEFAULT_A_VALUE;
    digital_scale.scale.nv_param.b = SCALE_TASK_DEFAULT_B_VALUE;
    digital_scale.scale.nv_param.tare_weight = SCALE_TASK_DEFAULT_TARE_VALUE;

    digital_scale.scale.net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
    digital_scale.scale.gross_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
}


#if  SCALE_TASK_CALCULATE_VARIANCE > 0

#define  MOVE_SAMPLE_CNT                 16
/*定义启动变化阈值*/
#define  EVALUATE_TASK_VARIANCE_MAX      10
/*定义停止变化阈值 值越小稳定时间越长，值越精确 */
#define  EVALUATE_TASK_VARIANCE_MIN      1

typedef struct
{
    uint8_t  expired;
    uint32_t wr_pos;
    float    sample[MOVE_SAMPLE_CNT];  
    float    variance;
    float    value;
}move_sample_t;


static move_sample_t move_sample;

typedef enum
{
    STABLE_STATUS_IDEL_WAIT_START = 0,
    STABLE_STATUS_START_WAIT_IDEL
}stable_status_t;

typedef struct
{
    float         change_start_value;
    float         change_stop_value;
    float         change_value;
    uint32_t      change_start_time;
    uint32_t      change_stop_time;
    uint32_t      change_time;
    float         variance;
    int16_t       dir;
    stable_status_t status;
}stable_t;

static stable_t net_weight;

/*计算方差*/
static  int caculate_variance(move_sample_t *ms)
{
    float sum;
    float average;
    float variance;

    uint8_t cnt;

    cnt = ms->expired;
    sum = 0;

    for (uint8_t i = 0;i < cnt;i++) {
        sum += ms->sample[i];
    }
    average = sum / (float)cnt;

    sum = 0;
    for (uint8_t i = 0;i < cnt;i++) {
      sum += pow(ms->sample[i] > average ? ms->sample[i] - average : average - ms->sample[i],2);
    }

    variance = sqrt(sum / (float)(cnt - 1));
    ms->value = average;
    ms->variance = variance;

    return 0;
}

/*取样数据*/
static int move_sample_put(float value,move_sample_t *ms)
{

    ms->sample[ms->wr_pos % ms->expired] = value;
    ms->wr_pos++;
    if(ms->wr_pos >= ms->expired){
        return 0;
    }
    return -1;
}

/*取样复位*/
static int move_sample_reset(move_sample_t *ms)
{
    ms->wr_pos =0;
    ms->variance =0;
    ms->value =0;
    ms->expired = MOVE_SAMPLE_CNT;
    return 0;
}

#endif

/*获取四舍五入的重量值*/
static int16_t get_fine_weight(float weight)
{
    int16_t int_weight = 0;
    float float_weight;
    
    int_weight = (int16_t)weight;
    float_weight = weight - int_weight;
    
    if (float_weight >= 0.5) {
        int_weight += 1;
    } else if (float_weight <= -0.5) {
        int_weight -= 1;
    }
    
    return int_weight;
}

/*电子秤任务*/
void scale_task(void const *argument)
{

    osStatus status;
    osEvent  os_msg;
    uint8_t  result;

    int  rc;
    char *temp;
    float a,b;
    uint32_t zero_adc = 0;
    int16_t zero_weight = 0;
    int tare_weight;

    char buffer[SCALE_BUFFER_SIZE];
    int16_t  weight;
    float    temp_weight;
    task_message_t    msg;
    task_message_t    protocol_msg;

    device_env_init();
    scale_task_param_init();

#if SCALE_TASK_CALCULATE_VARIANCE > 0
    move_sample_reset(&move_sample);
#endif
 
    /*查看保存的地址是否有效*/
    temp = device_env_get(SCALE_ADDR_NAME_STR);
    if (temp == NULL) {
        log_info("addr use default:%d.\r\n",SCALE_ADDR_DEFAULT);
    }else {
        digital_scale.scale.nv_param.addr = atoi(temp);
        log_info("addr:%d.\r\n",digital_scale.scale.nv_param.addr);
    }

    /*查看保存的nv参数是否有效*/
    temp = device_env_get(SCALE_A_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.a = utils_atof(temp);  
    }

    temp = device_env_get(SCALE_B_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.b = utils_atof(temp);  
    }

    temp = device_env_get(SCALE_TARE_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.tare_weight = atoi(temp);  
    }

    while (1) {
        os_msg = osMessageGet(scale_task_msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
        if (os_msg.status == osEventMessage){
            msg = *(task_message_t*)&os_msg.value.v;
 
        /*实时计算毛重和净重*/
        if (msg.type ==  TASK_MSG_ADC_COMPLETE){
            digital_scale.scale.cur_adc = msg.value;
            /*计算毛重和净重*/
            temp_weight = ((float)digital_scale.scale.cur_adc *  digital_scale.scale.nv_param.a +  digital_scale.scale.nv_param.b);
            weight = get_fine_weight(temp_weight);
            if (weight >= SCALE_TASK_MAX_WEIGHT_VALUE ||
                weight <= SCALE_TASK_MIN_WEIGHT_VALUE  ){
                digital_scale.scale.gross_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
                digital_scale.scale.net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;      
            }else{
                digital_scale.scale.gross_weight = weight;
                digital_scale.scale.net_weight =  digital_scale.scale.gross_weight -  digital_scale.scale.nv_param.tare_weight; 
            }   
            /*向adc_task回应处理结果*/
            osSignalSet(adc_task_hdl,ADC_TASK_RESTART_SIGNAL);
 
        /*计算稳定时间*/
#if  SCALE_TASK_CALCULATE_VARIANCE > 0
    int rc;
    float variance;

    if ( digital_scale.scale.net_weight == SCALE_TASK_WEIGHT_ERR_VALUE){
        move_sample_reset(&move_sample);
        log_error("sample err.\r\n");
    } else {
        rc = move_sample_put(digital_scale.scale.net_weight,&move_sample);
        if (rc == 0){
            rc = caculate_variance(&move_sample);
            if (rc == 0){
                variance = move_sample.variance;
    
                if (net_weight.status == STABLE_STATUS_IDEL_WAIT_START && variance >= EVALUATE_TASK_VARIANCE_MAX){
                    net_weight.change_start_time = osKernelSysTick();
                    net_weight.change_start_value = net_weight.change_stop_value;
                    net_weight.status = STABLE_STATUS_START_WAIT_IDEL;
                    log_debug("start time:%d.\r\n",net_weight.change_start_time);
                }else if (net_weight.status == STABLE_STATUS_START_WAIT_IDEL && \
                        variance <= EVALUATE_TASK_VARIANCE_MIN){
                        net_weight.change_stop_time = osKernelSysTick();
                        net_weight.change_time = net_weight.change_stop_time - net_weight.change_start_time;
                        net_weight.change_stop_value = digital_scale.scale.net_weight;
                        net_weight.change_value = net_weight.change_stop_value -net_weight.change_start_value;
                        net_weight.variance = variance;    
                        net_weight.status = STABLE_STATUS_IDEL_WAIT_START;

                        if (net_weight.change_value > SCALE_TASK_DIFF_WEIGHT) {
                            log_debug("放下%dg.time:%dms.\r\n",(uint16_t)net_weight.change_value,net_weight.change_time);
                        }else if (net_weight.change_value < -SCALE_TASK_DIFF_WEIGHT) {
                            log_debug("拿起%dg.time:%dms.\r\n",(uint16_t)(net_weight.change_value * -1),net_weight.change_time);
                        }
                } 
            }
        }
    }
#endif
    }
   
    /*向protocol_task回应净重值*/
    if (msg.type ==  TASK_MSG_REQ_NET_WEIGHT){
        protocol_msg.type = TASK_MSG_RSP_NET_WEIGHT;
        protocol_msg.value = digital_scale.scale.net_weight;
  
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
    }
 
 
    /*向protocol_task回应0点校准结果*/
    if (msg.type ==  TASK_MSG_REQ_CALIBRATE_ZERO) {  
        if (digital_scale.scale.net_weight == SCALE_TASK_WEIGHT_ERR_VALUE ){
            rc = -1;
            result = SCALE_TASK_FAILURE; 
            goto calibrate_zero_msg_handle; 
        }

        /*暂存校验参数*/
        zero_weight = 0;
        zero_adc = digital_scale.scale.cur_adc;
        rc = 0;
        result = SCALE_TASK_SUCCESS;  

calibrate_zero_msg_handle:
        if (rc == 0) {
            log_info("calibrate zero ok.\r\n");        
        } else {
            log_error("calibrate zero fail.\r\n");
        }
        protocol_msg.type = TASK_MSG_RSP_CALIBRATE_ZERO;
        protocol_msg.value = result;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        log_assert(status == osOK);
   }
    
    /*向protocol_task回应满量程点校准结果*/
    if (msg.type ==  TASK_MSG_REQ_CALIBRATE_FULL) {
        if (digital_scale.scale.net_weight == SCALE_TASK_WEIGHT_ERR_VALUE ){
            rc = -1;
            result = SCALE_TASK_FAILURE; 
            goto calibrate_full_msg_handle; 
        }

        /*避免除法错误*/
        a = (float)(msg.value - zero_weight) / (float)(digital_scale.scale.cur_adc - zero_adc + 1);
        b = (float)msg.value - a * digital_scale.scale.cur_adc;
        tare_weight = 0;

        snprintf(buffer,SCALE_BUFFER_SIZE,"%f",a);
        /*预写入*/
        rc = device_env_set(SCALE_A_NAME_STR,buffer);
        if (rc != 0) {
            result = SCALE_TASK_FAILURE;
            goto calibrate_full_msg_handle;
        } 
   
        snprintf(buffer,SCALE_BUFFER_SIZE,"%f",b);
        /*预写入*/
        rc = device_env_set(SCALE_B_NAME_STR,buffer);
        if (rc != 0) {
            result = SCALE_TASK_FAILURE;
            goto calibrate_full_msg_handle;
        }  

        snprintf(buffer,SCALE_BUFFER_SIZE,"%d",tare_weight);
        rc = device_env_set(SCALE_TARE_NAME_STR,buffer);
        /*预写入*/
        if (rc != 0) {
            result = SCALE_TASK_FAILURE;
            goto calibrate_full_msg_handle;
        } 

        /*执行保存*/
        rc = device_env_do_save();
        if (rc != 0) {
            result = SCALE_TASK_FAILURE;
            goto calibrate_full_msg_handle;
        } 
    
        result = SCALE_TASK_SUCCESS;  
        digital_scale.scale.nv_param.a = a;
        digital_scale.scale.nv_param.b = b;
        digital_scale.scale.nv_param.tare_weight = tare_weight;

        result = SCALE_TASK_SUCCESS;   
calibrate_full_msg_handle:
        if (rc == 0) {
            log_info("calibrate full ok.a:%f b:%f.\r\n",a,b);
        } else {
            log_error("calibrate full fail.\r\n");
        }

        protocol_msg.type = TASK_MSG_RSP_CALIBRATE_FULL;
        protocol_msg.value = result;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        log_assert(status == osOK);
   }
 
    /*向protocol_task回应去皮结果*/
    if(msg.type ==  TASK_MSG_REQ_REMOVE_TARE_WEIGHT){
        if (digital_scale.scale.net_weight == SCALE_TASK_WEIGHT_ERR_VALUE ){
            rc = -1;
            result = SCALE_TASK_FAILURE; 
            goto remove_tare_weight_msg_handle; 
        }

        snprintf(buffer,SCALE_BUFFER_SIZE,"%d",digital_scale.scale.gross_weight);
        /*预写入*/
        rc = device_env_set(SCALE_TARE_NAME_STR,buffer);
        if (rc != 0) {
            result = SCALE_TASK_FAILURE;
            goto remove_tare_weight_msg_handle;
        } 
       
        /*执行保存*/
        rc = device_env_do_save();
        if (rc != 0) {
            result = SCALE_TASK_FAILURE;
            goto remove_tare_weight_msg_handle;
        }  
  
        digital_scale.scale.nv_param.tare_weight = digital_scale.scale.gross_weight;
        result = SCALE_TASK_SUCCESS;   

remove_tare_weight_msg_handle:
        if (rc == 0) {
            log_info("tare ok.\r\n",a,b);
        } else {
            log_error("tare fail.\r\n");
        }

        protocol_msg.type = TASK_MSG_RSP_REMOVE_TARE_WEIGHT;
        protocol_msg.value = result;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        log_assert(status == osOK);
    }
 
    /*向protocol_task回应当前地址值*/
    if (msg.type ==  TASK_MSG_REQ_ADDR){
        protocol_msg.type = TASK_MSG_RSP_ADDR;
        protocol_msg.value = digital_scale.scale.nv_param.addr;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
        log_assert(status == osOK);   
    } 

    /*向protocol_task回应设置地址结果*/
    if(msg.type ==  TASK_MSG_REQ_SET_ADDR){
        if (msg.value > SCALE_TASK_ADDR_VALUE_MAX || msg.value == 0){
            rc = -1;
            result = SCALE_TASK_FAILURE ;
            goto set_addr_msg_handle;
        }

        snprintf(buffer,SCALE_BUFFER_SIZE,"%d",msg.value);
        /*预写入*/
        rc = device_env_set(SCALE_ADDR_NAME_STR,buffer);
        if (rc != 0) {
            result = SCALE_TASK_FAILURE;
            goto set_addr_msg_handle;
        }  
      
         /*执行保存*/
        rc = device_env_do_save();
        if (rc != 0) {
            result = SCALE_TASK_FAILURE;
            goto set_addr_msg_handle;
        } 
  
        digital_scale.scale.nv_param.addr = msg.value;
        result = SCALE_TASK_SUCCESS;   

set_addr_msg_handle:
        if (rc == 0) {
            log_info("set addr:%d ok.\r\n", msg.value);
        } else {
            log_error("set addr err.\r\n");
        }
        protocol_msg.type = TASK_MSG_RSP_SET_ADDR;
        protocol_msg.value = result;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        log_assert(status == osOK);
  }
 
    /*向protocol_task回应传感器ID*/
    if (msg.type ==  TASK_MSG_REQ_SENSOR_ID){
        protocol_msg.type = TASK_MSG_RSP_SENSOR_ID;
        protocol_msg.value = SENSOR_ID;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
        log_assert(status == osOK);     
    }
 
 
    /*向protocol_task回应版本号*/
    if (msg.type ==  TASK_MSG_REQ_FW_VERSION){
        protocol_msg.type = TASK_MSG_RSP_FW_VERSION;
        protocol_msg.value = FIRMWARE_VERSION_HEX;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
        log_assert(status == osOK);   
    } 

  }
  }
  
}



