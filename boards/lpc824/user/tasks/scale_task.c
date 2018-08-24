#include "board.h"
#include "cpu_utils.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "version.h"
#include "sensor_id.h"
#include "nv.h"
#include "adc_task.h"
#include "protocol_task.h"
#include "scale_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[scale_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


osThreadId   scale_task_hdl;
osMessageQId scale_task_msg_q_id;


#define  SCALE_TASK_DEFAULT_A_VALUE       (0.002)
#define  SCALE_TASK_DEFAULT_B_VALUE       (20.002)

#define  SCALE_ADDR_DEFAULT                0x00



typedef struct
{
float     a;
float     b;
int16_t   zero_weight;
int16_t   full_weight;
uint32_t  zero_adc;
uint32_t  full_adc;
}scale_nv_param_t;

typedef struct
{
uint8_t          nv_addr;
uint8_t          nv_addr_valid;
uint8_t          nv_param_valid;
scale_nv_param_t nv_param;
uint32_t         cur_adc;
int16_t          tar_weight;
int16_t          net_weight;
int16_t          gross_weight;
}scale_t;

static scale_t scale;
static task_msg_t *msg;
static task_msg_t protocol_msg;


void scale_task(void const *argument)
{
 osStatus status;
 osEvent  os_msg;
 uint8_t  result;
 uint8_t  valid;
 int      nv_result;
 float    weight;
 
 osMessageQDef(scale_task_msg_q,5,uint32_t);
 scale_task_msg_q_id = osMessageCreate(osMessageQ(scale_task_msg_q),scale_task_hdl);
 log_assert(scale_task_msg_q_id);  
 
 /*查看保存的地址是否有效*/
 nv_result = nv_read(SCALE_TASK_NV_SCALE_ADDR_VALID_REGION_ADDR,&scale.nv_addr_valid,sizeof(scale.nv_addr_valid));
 
 nv_result = nv_read(SCALE_TASK_NV_SCALE_ADDR_REGION_ADDR,&scale.nv_addr,sizeof(scale.nv_addr));

 
 if(scale.nv_addr_valid != SCALE_TASK_NV_VALID){
   scale.nv_addr_valid = SCALE_TASK_NV_VALID;
   log_info("legacy addr invalid:%d. use default:%d.\r\n",scale.nv_addr,SCALE_ADDR_DEFAULT);
   scale.nv_addr = SCALE_ADDR_DEFAULT;   
 }
 
 nv_result = nv_read(SCALE_TASK_NV_PARAM_VALID_REGION_ADDR,(uint8_t*)&scale.nv_param_valid,sizeof(scale.nv_param_valid));
 
 /*如果上次断电后保存的数据是有效的*/
 if(scale.nv_param_valid == SCALE_TASK_NV_VALID){
 nv_result = nv_read(SCALE_TASK_NV_PARAM_REGION_ADDR,(uint8_t*)&scale.nv_param,sizeof(scale.nv_param));
 }else{
 scale.nv_param_valid = SCALE_TASK_NV_INVALID;
 }

 while(1){
 os_msg = osMessageGet(scale_task_msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t*)os_msg.value.v;
 
 /*实时计算毛重和净重*/
 if(msg->type ==  ADC_SAMPLE_COMPLETED){
  scale.cur_adc = msg->adc;
  /*如果ADC取样是错误值 或者nv保存的参数无效*/
  if(scale.cur_adc == ADC_TASK_SAMPLE_ERR_VALUE ||
     scale.nv_param_valid == SCALE_TASK_NV_INVALID ){
  scale.gross_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
  scale.net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
  }else{
  /*计算毛重和净重*/
  weight = (float)scale.cur_adc * scale.nv_param.a + scale.nv_param.b;
  if(weight >= SCALE_TASK_MAX_WEIGHT_VALUE ||
     weight <= SCALE_TASK_MIN_WEIGHT_VALUE  ){
     scale.gross_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
     scale.net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;     
   }else{
     scale.gross_weight = (int16_t)weight;
     scale.net_weight =  scale.gross_weight - scale.tar_weight; 
   }
  }  
  /*向adc_task回应处理结果*/
  osSignalSet(adc_task_hdl,ADC_TASK_RESTART_SIGNAL);
 }
 
 
  /*向protocol_task回应净重值*/
  if(msg->type ==  REQ_NET_WEIGHT){
  protocol_msg.type = RESPONSE_NET_WEIGHT;
  protocol_msg.net_weight = scale.net_weight;
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
  }
 
 
  /*向protocol_task回应0点校准结果*/
  if(msg->type ==  REQ_CALIBRATE_ZERO){
    if(scale.cur_adc == ADC_TASK_SAMPLE_ERR_VALUE ){
     result = SCALE_TASK_FAILURE; 
     log_error("calibrate zero fail.adc err.\r\n");
     goto calibrate_zero_msg_handle; 
    }
    scale.nv_param.zero_adc = scale.cur_adc;
    scale.nv_param.zero_weight = msg->calibrate_weight;
    /*避免除法错误*/
    if(scale.nv_param.zero_adc == scale.nv_param.full_adc){
      scale.nv_param.full_adc+=1;
    }
    scale.nv_param.a = (float)(scale.nv_param.full_weight - scale.nv_param.zero_weight) / (float)(scale.nv_param.full_adc - scale.nv_param.zero_adc);
    scale.nv_param.b = (float)scale.nv_param.zero_weight - scale.nv_param.a * scale.nv_param.zero_adc;
    
    /*使校准值无效*/
    valid = SCALE_TASK_NV_INVALID;
    nv_result = nv_save(SCALE_TASK_NV_PARAM_VALID_REGION_ADDR,(uint8_t *)&valid,sizeof(valid));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE ;
    log_error("calibrate zero fail.nv invalid err.\r\n");  
    goto calibrate_zero_msg_handle;
    }
       
   nv_result = nv_save(SCALE_TASK_NV_PARAM_REGION_ADDR,(uint8_t *)&scale.nv_param,sizeof(scale.nv_param));
   if(nv_result != 0){
   result = SCALE_TASK_FAILURE;
   log_error("calibrate zero fail.nv param err.\r\n");
   goto calibrate_zero_msg_handle;
   }
   
    /*使校准值有效*/
    valid = SCALE_TASK_NV_VALID;
    nv_result = nv_save(SCALE_TASK_NV_PARAM_VALID_REGION_ADDR,(uint8_t *)&valid,sizeof(valid));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE ;
    log_error("calibrate zero fail.nv valid err.\r\n");  
    goto calibrate_zero_msg_handle;
    } 
    scale.nv_param_valid = SCALE_TASK_NV_VALID;
    result = SCALE_TASK_SUCCESS;   
    log_info("calibrate zero success.nv a:%f b:%f.\r\n",scale.nv_param.a,scale.nv_param.b);
    
calibrate_zero_msg_handle:
    protocol_msg.type = RESPONSE_CALIBRATE_ZERO;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
   }
    
  /*向protocol_task回应满量程点校准结果*/
  if(msg->type ==  REQ_CALIBRATE_FULL){
   if(scale.cur_adc == ADC_TASK_SAMPLE_ERR_VALUE){
   result = SCALE_TASK_FAILURE; 
   log_error("calibrate full fail.adc err.\r\n");
   goto calibrate_full_msg_handle;
   }
   
   scale.nv_param.full_adc = scale.cur_adc;
   scale.nv_param.full_weight = msg->calibrate_weight;
   /*避免除法错误*/
   if(scale.nv_param.zero_adc == scale.nv_param.full_adc){
      scale.nv_param.full_adc+=1;
   }
   scale.nv_param.a = (float)(scale.nv_param.full_weight - scale.nv_param.zero_weight) / (float)(scale.nv_param.full_adc - scale.nv_param.zero_adc);
   scale.nv_param.b = (float)scale.nv_param.full_weight - scale.nv_param.a * scale.nv_param.full_adc;
   /*使校准值无效*/
    valid = SCALE_TASK_NV_INVALID;
    nv_result = nv_save(SCALE_TASK_NV_PARAM_VALID_REGION_ADDR,(uint8_t *)&valid,sizeof(valid));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE ;
    log_error("calibrate full fail.nv invalid err.\r\n");  
    goto calibrate_full_msg_handle;
    }
       
   nv_result = nv_save(SCALE_TASK_NV_PARAM_REGION_ADDR,(uint8_t *)&scale.nv_param,sizeof(scale.nv_param));
   if(nv_result != 0){
   result = SCALE_TASK_FAILURE;
   log_error("calibrate full fail.nv param err.\r\n");
   goto calibrate_full_msg_handle;
   }
   
    /*使校准值有效*/
    valid = SCALE_TASK_NV_VALID;
    nv_result = nv_save(SCALE_TASK_NV_PARAM_VALID_REGION_ADDR,(uint8_t *)&valid,sizeof(valid));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE ;
    log_error("calibrate full fail.nv valid err.\r\n");  
    goto calibrate_full_msg_handle;
    } 
    scale.nv_param_valid = SCALE_TASK_NV_VALID;
    result = SCALE_TASK_SUCCESS;   
    log_info("calibrate full success.nv a:%f b:%f.\r\n",scale.nv_param.a,scale.nv_param.b);
    
calibrate_full_msg_handle:
    protocol_msg.type = RESPONSE_CALIBRATE_FULL;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
 }
 
  /*向protocol_task回应去皮结果*/
  if(msg->type ==  REQ_REMOVE_TAR_WEIGHT){
   if(scale.gross_weight == SCALE_TASK_WEIGHT_ERR_VALUE){
     log_error("remove tar weight fail.weight err.\r\n"); 
     goto remove_tar_weight_msg_handle;
   }
   scale.tar_weight = scale.gross_weight;
   scale.nv_param.b = (float)scale.gross_weight - scale.nv_param.a * scale.cur_adc; 
   
   /*使校准值无效*/
    valid = SCALE_TASK_NV_INVALID;
    nv_result = nv_save(SCALE_TASK_NV_PARAM_VALID_REGION_ADDR,(uint8_t *)&valid,sizeof(valid));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE ;
    log_error("remove tar weight fail.nv invalid err.\r\n");  
    goto remove_tar_weight_msg_handle;
    }
       
   nv_result = nv_save(SCALE_TASK_NV_PARAM_REGION_ADDR,(uint8_t *)&scale.nv_param,sizeof(scale.nv_param));
   if(nv_result != 0){
   result = SCALE_TASK_FAILURE;
   log_error("remove tar weight fail.nv param err.\r\n");
   goto remove_tar_weight_msg_handle;
   }
   
    /*使校准值有效*/
    valid = SCALE_TASK_NV_VALID;
    nv_result = nv_save(SCALE_TASK_NV_PARAM_VALID_REGION_ADDR,(uint8_t *)&valid,sizeof(valid));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE ;
    log_error("remove tar weight fail.nv valid err.\r\n");  
    goto remove_tar_weight_msg_handle;
    } 
    scale.nv_param_valid = SCALE_TASK_NV_VALID;
    result = SCALE_TASK_SUCCESS;   
    log_info("remove tar weight success.nv a:%f b:%f.\r\n",scale.nv_param.a,scale.nv_param.b);
   
remove_tar_weight_msg_handle:
    protocol_msg.type = RESPONSE_CALIBRATE_FULL;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
  }
 
 
  /*向protocol_task回应设置地址结果*/
  if(msg->type ==  REQ_SET_ADDR){
   if(msg->scale_addr > SCALE_TASK_ADDR_VALUE_MAX){
     log_error("set addr fail.addr:%d.\r\n",msg->scale_addr); 
     goto set_addr_msg_handle;
   }
   
   scale.nv_addr = msg->scale_addr;   
   /*使校准值无效*/
    valid = SCALE_TASK_NV_INVALID;
    nv_result = nv_save(SCALE_TASK_NV_SCALE_ADDR_VALID_REGION_ADDR,(uint8_t *)&valid,sizeof(valid));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE ;
    log_error("set scale addr fail.nv invalid err.\r\n");  
    goto set_addr_msg_handle;
    }
       
   nv_result = nv_save(SCALE_TASK_NV_SCALE_ADDR_REGION_ADDR,(uint8_t *)&scale.nv_addr,sizeof(scale.nv_addr));
   if(nv_result != 0){
   result = SCALE_TASK_FAILURE;
   log_error("set scale addr fail.nv addr err.\r\n");
   goto set_addr_msg_handle;
   }
   
    /*使校准值有效*/
    valid = SCALE_TASK_NV_VALID;
    nv_result = nv_save(SCALE_TASK_NV_SCALE_ADDR_VALID_REGION_ADDR,(uint8_t *)&valid,sizeof(valid));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE ;
    log_error("set scale addr fail.nv valid err.\r\n");  
    goto set_addr_msg_handle;
    } 
    scale.nv_addr_valid = SCALE_TASK_NV_VALID;
    result = SCALE_TASK_SUCCESS;   
    log_info("set scale addr success.nv addr:%d.\r\n",scale.nv_addr);
   
set_addr_msg_handle:
    protocol_msg.type = RESPONSE_SET_ADDR;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
  }
 
  /*向protocol_task回应当前地址值*/
  if(msg->type ==  REQ_ADDR){
   protocol_msg.type = RESPONSE_ADDR;
   protocol_msg.version = scale.nv_addr;
   status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
   log_assert(status == osOK);   
  } 
 
 
  /*向protocol_task回应传感器ID*/
  if(msg->type ==  REQ_SENSOR_ID){
   protocol_msg.type = RESPONSE_SENSOR_ID;
   protocol_msg.sensor_id = SENSOR_ID;
   status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
   log_assert(status == osOK);     
  }
 
 
  /*向protocol_task回应版本号*/
  if(msg->type ==  REQ_VERSION){
   protocol_msg.type = RESPONSE_VERSION;
   protocol_msg.version = FIRMWARE_VERSION;
   status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
   log_assert(status == osOK);   
  } 
  }
 
  }  
}



