#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "firmware_version.h"
#include "sensor_id.h"
#include "nv.h"
#include "adc_task.h"
#include "protocol_task.h"
#include "scale_task.h"
#include "math.h"
#include "log.h"

osThreadId   scale_task_hdl;
osMessageQId scale_task_msg_q_id;


#define  SCALE_TASK_DEFAULT_A_VALUE       (1.39e-2)
#define  SCALE_TASK_DEFAULT_B_VALUE       (-1.17e5)

#define  SCALE_ADDR_DEFAULT                0x01


typedef struct
{
    float     a;
    float     b;
    uint8_t   valid;
    int16_t   zero_weight;
    int16_t   full_weight;
    uint32_t  zero_adc;
    uint32_t  full_adc;
    int16_t   tar_weight;
}scale_nv_param_t;

typedef struct
{
    uint16_t         nv_param_addr;
    scale_nv_param_t nv_param;
    uint32_t         cur_adc;
    int16_t          net_weight;
    int16_t          gross_weight;
}scale_t;

typedef struct
{
    uint8_t          addr;
    uint8_t          valid;
}scale_nv_addr_t;

typedef struct
{
    scale_nv_addr_t  nv_addr;
    uint8_t          nv_addr_addr;
    scale_t          scale; 
}digital_scale_t;

static digital_scale_t digital_scale;


static void scale_task_param_init()
{
    digital_scale.nv_addr_addr = SCALE_TASK_NV_ADDR_ADDR;
    digital_scale.scale.nv_param_addr =  SCALE_TASK_SCALE_NV_PARAM_ADDR;
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

stable_t net_weight;

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



void scale_task(void const *argument)
{
    osStatus status;
    osEvent  os_msg;
    uint8_t  result;

    int      nv_result;
    int16_t  weight;
    scale_nv_param_t  pre_nv_param;
    scale_nv_addr_t   pre_nv_addr;
    task_message_t    msg;
    task_message_t    protocol_msg;
 
    scale_task_param_init();

    log_info("\r\nfirmware version:%s.\r\n",FIRMWARE_VERSION_STR);

#if SCALE_TASK_CALCULATE_VARIANCE > 0
    move_sample_reset(&move_sample);
#endif
 
    /*查看保存的地址是否有效*/
    nv_read(digital_scale.nv_addr_addr,(uint8_t*)&digital_scale.nv_addr,sizeof(digital_scale.nv_addr));

    if (digital_scale.nv_addr.valid != SCALE_TASK_NV_VALID || digital_scale.nv_addr.addr > SCALE_TASK_ADDR_VALUE_MAX){
        digital_scale.nv_addr.valid = SCALE_TASK_NV_VALID;
        log_info("legacy addr invalid:%d. use default:%d.\r\n",digital_scale.nv_addr.addr,SCALE_ADDR_DEFAULT);
        digital_scale.nv_addr.addr = SCALE_ADDR_DEFAULT;   
    }else {
        log_info("legacy addr valid:%d.\r\n",digital_scale.nv_addr.addr);
    }
 
    nv_read(digital_scale.scale.nv_param_addr,(uint8_t*)&digital_scale.scale.nv_param,sizeof(digital_scale.scale.nv_param));
 
    /*如果上次断电后保存的数据无效的*/
    if (digital_scale.scale.nv_param.valid != SCALE_TASK_NV_VALID){
        digital_scale.scale.nv_param.valid = SCALE_TASK_NV_VALID;
        digital_scale.scale.nv_param.a = SCALE_TASK_DEFAULT_A_VALUE;
        digital_scale.scale.nv_param.b = SCALE_TASK_DEFAULT_B_VALUE;
        log_info("scale.nv is invlaid.use default.\r\n");
    }
 
 
    while (1) {
        os_msg = osMessageGet(scale_task_msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
        if (os_msg.status == osEventMessage){
            msg = *(task_message_t*)&os_msg.value.v;
 
        /*实时计算毛重和净重*/
        if (msg.type ==  TASK_MSG_ADC_COMPLETE){
            digital_scale.scale.cur_adc = msg.value;
            /*计算毛重和净重*/
            weight = (int16_t) ((float)digital_scale.scale.cur_adc *  digital_scale.scale.nv_param.a +  digital_scale.scale.nv_param.b);
            if (weight >= SCALE_TASK_MAX_WEIGHT_VALUE ||
                weight <= SCALE_TASK_MIN_WEIGHT_VALUE  ){
                digital_scale.scale.gross_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
                digital_scale.scale.net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;      
            }else{
                digital_scale.scale.gross_weight = weight;
                digital_scale.scale.net_weight =  digital_scale.scale.gross_weight -  digital_scale.scale.nv_param.tar_weight; 
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
 
    /*向protocol_task回应0点校准重量值*/
    if (msg.type ==  TASK_MSG_REQ_CALIBRATE_ZERO_VALUE){
        protocol_msg.type = TASK_MSG_RSP_CALIBRATE_ZERO_VALUE;
        protocol_msg.value = digital_scale.scale.nv_param.zero_weight;
  
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
    }
  
    /*向protocol_task回应增益校准重量值*/
    if (msg.type ==  TASK_MSG_REQ_CALIBRATE_FULL_VALUE){
        protocol_msg.type = TASK_MSG_RSP_CALIBRATE_FULL_VALUE;
        protocol_msg.value = digital_scale.scale.nv_param.full_weight;;

        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
    }
  
 
    /*向protocol_task回应0点校准结果*/
    if (msg.type ==  TASK_MSG_REQ_CALIBRATE_ZERO) {  
        if (digital_scale.scale.net_weight == SCALE_TASK_WEIGHT_ERR_VALUE ){
            result = SCALE_TASK_FAILURE; 
            log_error("scale calibrate zero fail.\r\n");
            goto calibrate_zero_msg_handle; 
        }
        /*保留先前参数*/
        pre_nv_param = digital_scale.scale.nv_param;
    
        digital_scale.scale.nv_param.zero_adc = digital_scale.scale.cur_adc;
        digital_scale.scale.nv_param.zero_weight = msg.value;
        /*避免除法错误*/
        if (digital_scale.scale.nv_param.zero_adc == digital_scale.scale.nv_param.full_adc){
            digital_scale.scale.nv_param.full_adc += 1;
        }
        digital_scale.scale.nv_param.a = (float)(digital_scale.scale.nv_param.full_weight - digital_scale.scale.nv_param.zero_weight) / (float)(digital_scale.scale.nv_param.full_adc - digital_scale.scale.nv_param.zero_adc);
        digital_scale.scale.nv_param.b = (float)digital_scale.scale.nv_param.zero_weight - digital_scale.scale.nv_param.a * digital_scale.scale.nv_param.zero_adc;
        digital_scale.scale.nv_param.tar_weight = 0;
       
        nv_result = nv_save(digital_scale.scale.nv_param_addr,(uint8_t *)&digital_scale.scale.nv_param,sizeof(digital_scale.scale.nv_param));
        if (nv_result != 0) {
            result = SCALE_TASK_FAILURE;
            /*恢复先前参数*/
            digital_scale.scale.nv_param = pre_nv_param ;
            log_error("scale calibrate zero fail.nv param err.\r\n");
            goto calibrate_zero_msg_handle;
        }    
        log_info("scale calibrate zero success.nv a:%.5f b:%.5f.\r\n",digital_scale.scale.nv_param.a,digital_scale.scale.nv_param.b);
        result = SCALE_TASK_SUCCESS;  

calibrate_zero_msg_handle:
        protocol_msg.type = TASK_MSG_RSP_CALIBRATE_ZERO;
        protocol_msg.value = result;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        log_assert(status == osOK);
   }
    
    /*向protocol_task回应满量程点校准结果*/
    if (msg.type ==  TASK_MSG_REQ_CALIBRATE_FULL) {
        if (digital_scale.scale.net_weight == SCALE_TASK_WEIGHT_ERR_VALUE ){
            result = SCALE_TASK_FAILURE; 
            log_error("scale calibrate full fail.\r\n");
            goto calibrate_full_msg_handle; 
        }
    /*保留先前参数*/
    pre_nv_param = digital_scale.scale.nv_param;
   
    digital_scale.scale.nv_param.full_adc = digital_scale.scale.cur_adc;
    digital_scale.scale.nv_param.full_weight = msg.value;
    /*避免除法错误*/
    if (digital_scale.scale.nv_param.zero_adc == digital_scale.scale.nv_param.full_adc){
        digital_scale.scale.nv_param.full_adc += 1;
    }
    digital_scale.scale.nv_param.a = (float)(digital_scale.scale.nv_param.full_weight - digital_scale.scale.nv_param.zero_weight) / (float)(digital_scale.scale.nv_param.full_adc - digital_scale.scale.nv_param.zero_adc);
    digital_scale.scale.nv_param.b = (float)digital_scale.scale.nv_param.full_weight - digital_scale.scale.nv_param.a * digital_scale.scale.nv_param.full_adc;
       
    nv_result = nv_save(digital_scale.scale.nv_param_addr,(uint8_t *)&digital_scale.scale.nv_param,sizeof(digital_scale.scale.nv_param));
    if (nv_result != 0){
        result = SCALE_TASK_FAILURE;
        /*恢复先前参数*/
        digital_scale.scale.nv_param = pre_nv_param ;
        log_error("scalecalibrate full fail.nv param err.\r\n");
        goto calibrate_full_msg_handle;
    }
    log_info("scale calibrate full success.nv a:%.5f b:%.5f.\r\n",digital_scale.scale.nv_param.a,digital_scale.scale.nv_param.b);
   
    result = SCALE_TASK_SUCCESS;   
calibrate_full_msg_handle:
    protocol_msg.type = TASK_MSG_RSP_CALIBRATE_FULL;
    protocol_msg.value = result;
    status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
   }
 
    /*向protocol_task回应去皮结果*/
    if(msg.type ==  TASK_MSG_REQ_REMOVE_TARE_WEIGHT){
        if (digital_scale.scale.net_weight == SCALE_TASK_WEIGHT_ERR_VALUE ){
            result = SCALE_TASK_FAILURE; 
            log_error("scale calibrate full fail.\r\n");
            goto calibrate_full_msg_handle; 
        }
        /*保留先前参数*/
        pre_nv_param = digital_scale.scale.nv_param;  
        digital_scale.scale.nv_param.tar_weight = digital_scale.scale.gross_weight;
        //scale.nv_param.b = (float)scale.gross_weight - scale.nv_param.a * scale.cur_adc; 
   
        nv_result = nv_save(digital_scale.scale.nv_param_addr,(uint8_t *)&digital_scale.scale.nv_param,sizeof(digital_scale.scale.nv_param));
        if (nv_result != 0){
            result = SCALE_TASK_FAILURE;
            /*恢复先前参数*/
            digital_scale.scale.nv_param = pre_nv_param ;
            log_error("scale remove tar weight fail.nv param err.\r\n");
            goto remove_tar_weight_msg_handle;
        }  
        log_info("scale remove tar weight success.\r\n"); 
        result = SCALE_TASK_SUCCESS;   
remove_tar_weight_msg_handle:
        protocol_msg.type = TASK_MSG_RSP_REMOVE_TARE_WEIGHT;
        protocol_msg.value = result;
        status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        log_assert(status == osOK);
    }
 
 
    /*向protocol_task回应设置地址结果*/
    if(msg.type ==  TASK_MSG_REQ_SET_ADDR){
    if (msg.value > SCALE_TASK_ADDR_VALUE_MAX || msg.value == 0){
        log_error("set addr fail.addr:%d.\r\n",msg.value); 
        result = SCALE_TASK_FAILURE ;
        goto set_addr_msg_handle;
    }
    /*保存先前地址*/
    pre_nv_addr = digital_scale.nv_addr;
   
    digital_scale.nv_addr.addr = msg.value;   
       
    nv_result = nv_save(digital_scale.nv_addr_addr,(uint8_t *)&digital_scale.nv_addr,sizeof(digital_scale.nv_addr));
    if (nv_result != 0){
        result = SCALE_TASK_FAILURE;
        /*恢复先前地址*/
        digital_scale.nv_addr = pre_nv_addr;
        log_error("set scale addr fail.nv addr err.\r\n");
        goto set_addr_msg_handle;
    }
    result = SCALE_TASK_SUCCESS;   
    log_info("set scale addr success.nv addr:%d.\r\n",digital_scale.nv_addr.addr);
set_addr_msg_handle:
    protocol_msg.type = TASK_MSG_RSP_SET_ADDR;
    protocol_msg.value = result;
    status = osMessagePut(protocol_task_msg_q_id,*(uint32_t*)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
  }
 
    /*向protocol_task回应当前地址值*/
    if (msg.type ==  TASK_MSG_REQ_ADDR){
        protocol_msg.type = TASK_MSG_RSP_ADDR;
        protocol_msg.value = digital_scale.nv_addr.addr;
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



