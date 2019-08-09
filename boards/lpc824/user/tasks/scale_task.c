#include "board.h"
#include "cpu_utils.h"
#include "xstring.h"
#include "cmsis_os.h"
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
    uint8_t addr;
    float  a;
    float  b;
    int32_t tare_weight;
    int16_t zero_weight;
    uint32_t zero_adc;
    int16_t full_weight;
    uint32_t full_adc;
}scale_nv_param_t;

typedef struct
{
    scale_nv_param_t nv_param;
    uint32_t cur_adc;
    int16_t  net_weight;
    int16_t  gross_weight;
    int32_t  net_weight_int32;
    int32_t  gross_weight_int32;
}scale_t;

typedef struct
{
    scale_t          scale; 
}digital_scale_t;

static digital_scale_t digital_scale;


static void scale_task_param_init()
{
    digital_scale.scale.nv_param.addr = SCALE_DEFAULT_ADDR;

    digital_scale.scale.nv_param.a = SCALE_DEFAULT_A_VALUE;
    digital_scale.scale.nv_param.b = SCALE_DEFAULT_B_VALUE;
    digital_scale.scale.nv_param.tare_weight = SCALE_DEFAULT_TARE_VALUE;
    digital_scale.scale.nv_param.zero_weight = SCALE_DEFAULT_ZERO_WEIGHT_VALUE;
    digital_scale.scale.nv_param.zero_adc = SCALE_DEFAULT_ZERO_ADC_VALUE;
    digital_scale.scale.nv_param.full_weight = SCALE_DEFAULT_FULL_WEIGHT_VALUE;
    digital_scale.scale.nv_param.full_adc = SCALE_DEFAULT_FULL_ADC_VALUE;

    digital_scale.scale.net_weight_int32 = SCALE_WEIGHT_ERR_VALUE;
    digital_scale.scale.gross_weight_int32 = SCALE_WEIGHT_ERR_VALUE;
    digital_scale.scale.net_weight = SCALE_WEIGHT_ERR_VALUE;
    digital_scale.scale.gross_weight = SCALE_WEIGHT_ERR_VALUE;
}

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
    uint8_t  result;

    int  rc;
    char *temp;
    float a,b;
    uint32_t zero_adc = 0;
    int16_t zero_weight = 0;
    uint32_t full_adc;
    int16_t full_weight;
    int32_t tare_weight;

    char buffer[SCALE_BUFFER_SIZE];
    int32_t  weight;
    float  temp_weight;
    scale_task_message_t msg_recv;
    protocol_task_message_t protocol_msg;

    device_env_init();
    scale_task_param_init();
 
    /*查看保存的地址是否有效*/
    temp = device_env_get(SCALE_ADDR_NAME_STR);
    if (temp == NULL) {
        log_info("addr use default:%d.\r\n",SCALE_DEFAULT_ADDR);
    }else {
        digital_scale.scale.nv_param.addr = atoi(temp);
        log_info("addr:%d.\r\n",digital_scale.scale.nv_param.addr);
    }

    /*查看保存的nv参数是否有效*/
    /*校准的a值*/
    temp = device_env_get(SCALE_A_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.a = xstring_atof(temp);  
    }
    /*校准的b值*/
    temp = device_env_get(SCALE_B_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.b = xstring_atof(temp);  
    }
    /*皮重值*/
    temp = device_env_get(SCALE_TARE_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.tare_weight = atoi(temp);  
    }
    /*0点校准时的重量值*/
    temp = device_env_get(SCALE_ZERO_WEIGHT_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.zero_weight = atoi(temp);  
    }
    /*0点校准时的ADC值*/
    temp = device_env_get(SCALE_ZERO_ADC_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.zero_adc = atoi(temp);  
    }
    /*增益校准时的重量值*/
    temp = device_env_get(SCALE_FULL_WEIGHT_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.full_weight = atoi(temp);  
    }
    /*增益校准时的ADC值*/
    temp = device_env_get(SCALE_FULL_ADC_NAME_STR);
    if (temp) {
        digital_scale.scale.nv_param.full_adc = atoi(temp);  
    }
    while (1) {
    if (xQueueReceive(scale_task_msg_q_id, &msg_recv,SCALE_TASK_MSG_WAIT_TIMEOUT) == pdTRUE) {
        /*ADC转换错误*/
        if (msg_recv.head.id == SCALE_TASK_MSG_ADC_ERROR){       
            digital_scale.scale.net_weight_int32 = SCALE_WEIGHT_ERR_VALUE;
            digital_scale.scale.gross_weight_int32 = SCALE_WEIGHT_ERR_VALUE;
            digital_scale.scale.net_weight = SCALE_WEIGHT_ERR_VALUE;
            digital_scale.scale.gross_weight = SCALE_WEIGHT_ERR_VALUE;  
            /*向adc_task回应处理结果*/
            osSignalSet(adc_task_hdl,ADC_TASK_RESTART_SIGNAL);
        }

        /*实时计算毛重和净重*/
        if (msg_recv.head.id == SCALE_TASK_MSG_ADC_COMPLETE){    
            digital_scale.scale.cur_adc = msg_recv.content.adc;
            /*计算毛重和净重*/
            temp_weight = digital_scale.scale.cur_adc * digital_scale.scale.nv_param.a + digital_scale.scale.nv_param.b;
            weight = get_fine_weight(temp_weight);

            /*计算32位净重和毛重值*/
            digital_scale.scale.gross_weight_int32 = weight;
            digital_scale.scale.net_weight_int32 = weight - digital_scale.scale.nv_param.tare_weight;

            /*计算16位毛重值*/
            if (digital_scale.scale.gross_weight_int32 >= SCALE_MAX_WEIGHT_VALUE) {
                digital_scale.scale.gross_weight = SCALE_MAX_WEIGHT_VALUE;
            } else if (digital_scale.scale.gross_weight_int32 <= SCALE_MIN_WEIGHT_VALUE) {
                digital_scale.scale.gross_weight = SCALE_MIN_WEIGHT_VALUE;      
            }else{
                digital_scale.scale.gross_weight = digital_scale.scale.gross_weight_int32;
            }  

            /*计算16位净重值*/
            if (digital_scale.scale.net_weight_int32 >= SCALE_MAX_WEIGHT_VALUE) {
                digital_scale.scale.net_weight = SCALE_MAX_WEIGHT_VALUE;
            } else if (digital_scale.scale.net_weight_int32 <= SCALE_MIN_WEIGHT_VALUE) {
                digital_scale.scale.net_weight = SCALE_MIN_WEIGHT_VALUE;      
            }else{
                digital_scale.scale.net_weight = digital_scale.scale.net_weight_int32;
            } 
 
            /*向adc_task回应处理结果*/
            osSignalSet(adc_task_hdl,ADC_TASK_RESTART_SIGNAL);
        }
   
        /*净重值*/
        if (msg_recv.head.id == SCALE_TASK_MSG_GET_NET_WEIGHT){
            protocol_msg.head.id = PROTOCOL_TASK_MSG_NET_WEIGHT_VALUE;
            protocol_msg.content.net_weight = digital_scale.scale.net_weight;
 
            log_assert_bool_false(xQueueSend(protocol_task_msg_q_id,&protocol_msg,SCALE_TASK_PUT_MSG_TIMEOUT) == pdPASS);
        }
 
 
        /*0点校准*/
        if (msg_recv.head.id == SCALE_TASK_MSG_CALIBRATE_ZERO) {  
            if (digital_scale.scale.net_weight == SCALE_WEIGHT_ERR_VALUE ){
                result = SCALE_TASK_FAILURE; 
                goto calibrate_zero_msg_handle; 
            }

            /*暂存校验参数*/
            zero_weight = msg_recv.content.calibration_weight;
            zero_adc = digital_scale.scale.cur_adc;
            full_weight = digital_scale.scale.nv_param.full_weight;
            full_adc = digital_scale.scale.nv_param.full_adc;
            /*执行校准计算并避免除法错误*/
            if (full_adc == zero_adc) {
                full_adc += 1;
            }
            a = (double)(full_weight - zero_weight) / (double)(full_adc - zero_adc);
            b = (double)zero_weight - a * (double)zero_adc;
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

            snprintf(buffer,SCALE_BUFFER_SIZE,"%d",zero_weight);
            /*预写入*/
            rc = device_env_set(SCALE_ZERO_WEIGHT_NAME_STR,buffer);
            if (rc != 0) {
                result = SCALE_TASK_FAILURE;
                goto calibrate_full_msg_handle;
            }  

            snprintf(buffer,SCALE_BUFFER_SIZE,"%d",zero_adc);
            rc = device_env_set(SCALE_ZERO_ADC_NAME_STR,buffer);
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
    
            digital_scale.scale.nv_param.a = a;
            digital_scale.scale.nv_param.b = b;
            digital_scale.scale.nv_param.tare_weight = tare_weight;
            digital_scale.scale.nv_param.zero_weight = zero_weight;
            digital_scale.scale.nv_param.zero_adc = zero_adc;
            result = SCALE_TASK_SUCCESS;  

calibrate_zero_msg_handle:
            if (result == SCALE_TASK_SUCCESS) {
                log_info("calibrate zero ok.a:%f b:%f.\r\n",a,b);   
            } else {
                log_error("calibrate zero fail.\r\n");
            }
            protocol_msg.head.id = PROTOCOL_TASK_MSG_CALIBRATE_ZERO_RESULT;
            protocol_msg.content.result = result;
            log_assert_bool_false(xQueueSend(protocol_task_msg_q_id,&protocol_msg,SCALE_TASK_PUT_MSG_TIMEOUT) == pdPASS);
        }
    
        /*满量程点校准*/
        if (msg_recv.head.id ==  SCALE_TASK_MSG_CALIBRATE_FULL) {
            if (digital_scale.scale.net_weight == SCALE_WEIGHT_ERR_VALUE ){
                result = SCALE_TASK_FAILURE; 
                goto calibrate_full_msg_handle; 
            }

            /*暂存校验参数*/
            full_weight = msg_recv.content.calibration_weight;
            full_adc = digital_scale.scale.cur_adc;
            zero_weight = digital_scale.scale.nv_param.zero_weight;
            zero_adc = digital_scale.scale.nv_param.zero_adc;
            /*避免除法错误*/
            if (full_adc == zero_adc) {
                full_adc += 1;
            }
            a = (double)(full_weight - zero_weight) / (double)(full_adc - zero_adc);
            b = (double)full_weight - a * (double)full_adc;
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
            snprintf(buffer,SCALE_BUFFER_SIZE,"%d",full_weight);
            /*预写入*/
            rc = device_env_set(SCALE_FULL_WEIGHT_NAME_STR,buffer);
            if (rc != 0) {
                result = SCALE_TASK_FAILURE;
                goto calibrate_full_msg_handle;
            }  

            snprintf(buffer,SCALE_BUFFER_SIZE,"%d",full_adc);
            rc = device_env_set(SCALE_FULL_ADC_NAME_STR,buffer);
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
            digital_scale.scale.nv_param.full_weight = full_weight;
            digital_scale.scale.nv_param.full_adc = full_adc;
            result = SCALE_TASK_SUCCESS;   

calibrate_full_msg_handle:
            if (result == SCALE_TASK_SUCCESS) {
                log_info("calibrate full ok.a:%f b:%f.\r\n",a,b);
            } else {
                log_error("calibrate full fail.\r\n");
            }

            protocol_msg.head.id = PROTOCOL_TASK_MSG_CALIBRATE_FULL_RESULT;
            protocol_msg.content.result = result;
            log_assert_bool_false(xQueueSend(protocol_task_msg_q_id,&protocol_msg,SCALE_TASK_PUT_MSG_TIMEOUT) == pdPASS);
        }
        
        /*去皮*/
        if (msg_recv.head.id ==  SCALE_TASK_MSG_REMOVE_TARE_WEIGHT) {
            if (digital_scale.scale.net_weight == SCALE_WEIGHT_ERR_VALUE ){
                rc = -1;
                result = SCALE_TASK_FAILURE; 
                goto remove_tare_weight_msg_handle; 
            }

            snprintf(buffer,SCALE_BUFFER_SIZE,"%d",digital_scale.scale.gross_weight_int32);
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
        
            digital_scale.scale.nv_param.tare_weight = digital_scale.scale.gross_weight_int32;
            result = SCALE_TASK_SUCCESS;   

remove_tare_weight_msg_handle:
            if (result == SCALE_TASK_SUCCESS) {
                log_info("tare ok.\r\n",a,b);
            } else {
                log_error("tare fail.\r\n");
            }

            protocol_msg.head.id = PROTOCOL_TASK_MSG_REMOVE_TARE_WEIGHT_RESULT;
            protocol_msg.content.result = result;
            log_assert_bool_false(xQueueSend(protocol_task_msg_q_id,&protocol_msg,SCALE_TASK_PUT_MSG_TIMEOUT) == pdPASS);
        }
 
        /*当前地址值*/
        if (msg_recv.head.id ==  SCALE_TASK_MSG_GET_ADDR) {
            protocol_msg.head.id = PROTOCOL_TASK_MSG_ADDR_VALUE;
            protocol_msg.content.addr = digital_scale.scale.nv_param.addr;
            log_assert_bool_false(xQueueSend(protocol_task_msg_q_id,&protocol_msg,SCALE_TASK_PUT_MSG_TIMEOUT) == pdPASS); 
        } 

        /*设置地址*/
        if (msg_recv.head.id ==  SCALE_TASK_MSG_SET_ADDR) {
            if (msg_recv.content.addr_setting > SCALE_ADDR_VALUE_MAX || msg_recv.content.addr_setting < SCALE_ADDR_VALUE_MIN) {
                result = SCALE_TASK_FAILURE ;
                goto set_addr_msg_handle;
            }

            snprintf(buffer,SCALE_BUFFER_SIZE,"%d",msg_recv.content.addr_setting);
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
  
            digital_scale.scale.nv_param.addr = msg_recv.content.addr_setting;
            result = SCALE_TASK_SUCCESS;   

set_addr_msg_handle:
            if (result == SCALE_TASK_SUCCESS) {
                log_info("set addr:%d ok.\r\n", msg_recv.content.addr_setting);
            } else {
                log_error("set addr err.\r\n");
            }
            protocol_msg.head.id = PROTOCOL_TASK_MSG_SET_ADDR_RESULT;
            protocol_msg.content.result = result;
            log_assert_bool_false(xQueueSend(protocol_task_msg_q_id,&protocol_msg,SCALE_TASK_PUT_MSG_TIMEOUT) == pdPASS); 
        }

        /*回应传感器ID*/
        if (msg_recv.head.id ==  SCALE_TASK_MSG_GET_SENSOR_ID){
            protocol_msg.head.id = PROTOCOL_TASK_MSG_SENSOR_ID_VALUE;
            protocol_msg.content.sensor_id = SENSOR_ID;
            log_assert_bool_false(xQueueSend(protocol_task_msg_q_id,&protocol_msg,SCALE_TASK_PUT_MSG_TIMEOUT) == pdPASS);      
        }
 
        /*回应版本号*/
        if (msg_recv.head.id ==  SCALE_TASK_MSG_GET_FW_VERSION){
            protocol_msg.head.id = PROTOCOL_TASK_MSG_FW_VERSION_VALUE;
            protocol_msg.content.fw_version = FIRMWARE_VERSION_HEX;
            log_assert_bool_false(xQueueSend(protocol_task_msg_q_id,&protocol_msg,SCALE_TASK_PUT_MSG_TIMEOUT) == pdPASS);    
        } 


    }
  }
  
}



