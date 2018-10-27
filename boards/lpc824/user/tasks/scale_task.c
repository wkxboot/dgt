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
#include "math.h"
#include "log.h"
#define LOG_MODULE_NAME   "[scale_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


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
uint8_t          scale_cnt;
scale_t          scale[ADC_TASK_SAMPLE_CHANNEL_CNT]; 
int16_t          all_net_weight;
}digital_scale_t;


static task_message_t *msg;
static task_message_t protocol_msg;

static digital_scale_t digital_scale;

static const uint8_t scale_nv_param_addr[2] = {
SCALE_TASK_SCALE_1_NV_PARAM_ADDR,
SCALE_TASK_SCALE_2_NV_PARAM_ADDR
};

static void scale_task_param_init()
{
 uint8_t i =0;
 digital_scale.scale_cnt = ADC_TASK_SAMPLE_CHANNEL_CNT;
 digital_scale.nv_addr_addr = SCALE_TASK_NV_ADDR_ADDR;

 for(i=0;i<digital_scale.scale_cnt;i++){
 digital_scale.scale[i].nv_param_addr =  scale_nv_param_addr[i];
 digital_scale.scale[i].net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
 digital_scale.scale[i].gross_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
 }
}


#if  SCALE_TASK_CALCULATE_VARIANCE > 0

#define  MOVE_SAMPLE_CNT            25

typedef struct
{
uint8_t  expired;
uint32_t wr_pos;
int16_t  *buffer;
double   variance;
int16_t  value;
}move_sample_t;

static int16_t sample[MOVE_SAMPLE_CNT]={0};  
static move_sample_t move_sample;

typedef enum
{
STABLE_STATUS_IDEL_WAIT_START=0,
STABLE_STATUS_START_WAIT_IDEL
}stable_status_t;

typedef struct
{
int16_t         change_start_value;
int16_t         change_stop_value;
int16_t         change_value;
uint32_t        change_start_time;
uint32_t        change_stop_time;
uint32_t        change_time;
float           variance;
stable_status_t status;
}stable_t;

stable_t net_weight;

/*定义启动变化阈值 值越小灵敏度要高 */
#define  EVALUATE_TASK_VARIANCE_MAX      3
/*定义停止变化阈值 值越小稳定时间越长，值越精确 */
#define  EVALUATE_TASK_VARIANCE_MIN      0.6

/*计算方差*/
static  int caculate_variance(move_sample_t *ms)
{
uint8_t i;
double sum;
double average;
double variance;

int16_t *buffer;
uint8_t cnt;

buffer = ms->buffer;
cnt = ms->expired;
sum=0;

for(i=0;i<cnt;i++){
sum+=(double)buffer[i];
}
average = sum / (double)cnt;

sum =0;
for(i=0;i<cnt;i++){
sum += pow(((double)buffer[i] - average),2);
}
variance = sum / (double)cnt;
ms->value =(int16_t) average;
ms->variance =variance;

return 0;
}

/*取样数据*/
static int move_sample_put(int16_t value,move_sample_t *ms)
{
 int16_t *buffer;
 buffer = ms->buffer;
 buffer[ms->wr_pos % ms->expired] = value;
 ms->wr_pos++;
 if(ms->wr_pos >= ms->expired){
  return 0;
 }
  return -1;
}

/*取样复位*/
static int move_sample_reset(move_sample_t *ms,uint8_t expired,int16_t *buffer)
{
 ms->wr_pos =0;
 ms->variance =0;
 ms->value =0;
 ms->expired = expired;
 ms->buffer = buffer;
 return 0;
}

#endif



void scale_task(void const *argument)
{
 osStatus status;
 osEvent  os_msg;
 uint8_t  result;
 uint8_t  scale_idx;
 uint32_t all_net_weight;
 int      nv_result;
 float    weight;
 scale_nv_param_t  pre_nv_param;
 scale_nv_addr_t   pre_nv_addr;
 
 
 osMessageQDef(scale_task_msg_q,5,uint32_t);
 scale_task_msg_q_id = osMessageCreate(osMessageQ(scale_task_msg_q),scale_task_hdl);
 log_assert(scale_task_msg_q_id);  
 
 scale_task_param_init();
 
#if SCALE_TASK_CALCULATE_VARIANCE > 0
 move_sample_reset(&move_sample,MOVE_SAMPLE_CNT,sample);
#endif
 
 /*查看保存的地址是否有效*/
 nv_read(digital_scale.nv_addr_addr,(uint8_t*)&digital_scale.nv_addr,sizeof(digital_scale.nv_addr));

 if(digital_scale.nv_addr.valid != SCALE_TASK_NV_VALID){
   digital_scale.nv_addr.valid = SCALE_TASK_NV_VALID;
   log_info("legacy addr invalid:%d. use default:%d.\r\n",digital_scale.nv_addr.addr,SCALE_ADDR_DEFAULT);
   digital_scale.nv_addr.addr = SCALE_ADDR_DEFAULT;   
 }else {
   if(digital_scale.nv_addr.addr <= SCALE_TASK_ADDR_VALUE_MAX && digital_scale.nv_addr.addr > 0){
   log_info("legacy addr valid:%d.\r\n",digital_scale.nv_addr.addr);
   }else{     
   log_info("legacy addr invalid:%d. use default:%d.\r\n",digital_scale.nv_addr.addr,SCALE_ADDR_DEFAULT);
   digital_scale.nv_addr.addr = SCALE_ADDR_DEFAULT;  
   }
 }
 
 for(uint8_t i=0;i< digital_scale.scale_cnt;i++){
 nv_read(digital_scale.scale[i].nv_param_addr,(uint8_t*)&digital_scale.scale[i].nv_param,sizeof(digital_scale.scale[i].nv_param));
 
 /*如果上次断电后保存的数据无效的*/
 if(digital_scale.scale[i].nv_param.valid != SCALE_TASK_NV_VALID){
 digital_scale.scale[i].nv_param.valid = SCALE_TASK_NV_VALID;
 digital_scale.scale[i].nv_param.a = SCALE_TASK_DEFAULT_A_VALUE;
 digital_scale.scale[i].nv_param.b = SCALE_TASK_DEFAULT_B_VALUE;
 }
 }
 
 while(1){
 os_msg = osMessageGet(scale_task_msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_message_t*)os_msg.value.v;
 
 /*实时计算毛重和净重*/
 if(msg->type ==  ADC_SAMPLE_COMPLETED){
  scale_idx = msg->channel;
  digital_scale.scale[scale_idx].cur_adc = msg->adc;
  /*如果ADC取样是错误值 或者nv保存的参数无效*/
  if(digital_scale.scale[scale_idx].cur_adc == ADC_TASK_SAMPLE_ERR_VALUE ){
  digital_scale.scale[scale_idx].gross_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
  digital_scale.scale[scale_idx].net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
  }else{
  /*计算毛重和净重*/
  weight = (float) digital_scale.scale[scale_idx].cur_adc *  digital_scale.scale[scale_idx].nv_param.a +  digital_scale.scale[scale_idx].nv_param.b;
  if(weight >= SCALE_TASK_MAX_WEIGHT_VALUE ||
     weight <= SCALE_TASK_MIN_WEIGHT_VALUE  ){
      digital_scale.scale[scale_idx].gross_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
      digital_scale.scale[scale_idx].net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;      
   }else{
      digital_scale.scale[scale_idx].gross_weight = (int16_t)weight;
      digital_scale.scale[scale_idx].net_weight =   digital_scale.scale[scale_idx].gross_weight -  digital_scale.scale[scale_idx].nv_param.tar_weight; 
   }
  }
  all_net_weight = 0;
  for(uint8_t i=0;i< digital_scale.scale_cnt;i++){
  if(digital_scale.scale[i].net_weight == SCALE_TASK_WEIGHT_ERR_VALUE){
   all_net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
   break;
  }
  all_net_weight +=  digital_scale.scale[i].net_weight; 
  }
  digital_scale.all_net_weight = all_net_weight;
  /*向adc_task回应处理结果*/
  osSignalSet(adc_task_hdl,ADC_TASK_RESTART_SIGNAL);
 }
 
  /*计算稳定时间*/
#if  SCALE_TASK_CALCULATE_VARIANCE > 0
   int rc;
   double variance;

   if(all_net_weight == SCALE_TASK_WEIGHT_ERR_VALUE){
   move_sample_reset(&move_sample,MOVE_SAMPLE_CNT,sample);
   log_error("sample err.\r\n");
   }else{
   rc = move_sample_put(all_net_weight,&move_sample);
   if(rc == 0){
   rc = caculate_variance(&move_sample);
   if(rc == 0){
    //log_debug("variance:%d.",(int)(move_sample.variance*100) );
    variance = move_sample.variance;
    log_one_line("net weight:%d g. v:%d.cpu:%d%%.",all_net_weight,(int32_t)(variance*100000) ,osGetCPUUsage());
    
    if(net_weight.status == STABLE_STATUS_IDEL_WAIT_START && \
    variance >= EVALUATE_TASK_VARIANCE_MAX){
    net_weight.change_start_time = osKernelSysTick();
    net_weight.change_start_value = net_weight.change_stop_value;
    net_weight.status = STABLE_STATUS_START_WAIT_IDEL;
    //log_debug("change start time:%d,start_weight:%dg.\r\n",net_weight.change_start_time,net_weight.change_start_value);
    }else if(net_weight.status == STABLE_STATUS_START_WAIT_IDEL && \
             variance <= EVALUATE_TASK_VARIANCE_MIN){
    net_weight.change_stop_time = osKernelSysTick();
    net_weight.change_time = net_weight.change_stop_time - net_weight.change_start_time;
    net_weight.change_stop_value = all_net_weight;
    net_weight.change_value = net_weight.change_stop_value -net_weight.change_start_value;
    net_weight.variance = variance;    
    net_weight.status = STABLE_STATUS_IDEL_WAIT_START;
    log_debug("change stop time:%d,stop_weight:%dg,change_time:%dms,change_weight:%dg.\r\n"
              ,net_weight.change_stop_time
              ,net_weight.change_stop_value
              ,net_weight.change_time
              ,net_weight.change_value);  
   } 
   }
   }
   }
#endif
   
  /*向protocol_task回应净重值*/
  if(msg->type ==  REQ_NET_WEIGHT){
  protocol_msg.type = RESPONSE_NET_WEIGHT;
  protocol_msg.net_weight = 0;
  for(uint8_t i=0;i< digital_scale.scale_cnt;i++){
  if(digital_scale.scale[i].net_weight == SCALE_TASK_WEIGHT_ERR_VALUE){
   protocol_msg.net_weight = SCALE_TASK_WEIGHT_ERR_VALUE;
   break;
  }
  protocol_msg.net_weight +=  digital_scale.scale[i].net_weight; 
  }
  
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
  }
 
  /*向protocol_task回应0点校准重量值*/
  if(msg->type ==  REQ_CALIBRATE_ZERO_VALUE){
  protocol_msg.type = RESPONSE_CALIBRATE_ZERO_VALUE;
  protocol_msg.calibrate_weight = digital_scale.scale[0].nv_param.zero_weight;
  
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
  }
  
   /*向protocol_task回应增益校准重量值*/
  if(msg->type ==  REQ_CALIBRATE_FULL_VALUE){
  protocol_msg.type = RESPONSE_CALIBRATE_FULL_VALUE;
  protocol_msg.calibrate_weight = digital_scale.scale[0].nv_param.full_weight;;

  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);  
  }
  
 
  /*向protocol_task回应0点校准结果*/
  if(msg->type ==  REQ_CALIBRATE_ZERO){
    for(uint8_t i =0 ;i<digital_scale.scale_cnt;i++){     
    if(digital_scale.scale[i].cur_adc == ADC_TASK_SAMPLE_ERR_VALUE ){
     result = SCALE_TASK_FAILURE; 
     log_error("scale%d calibrate zero fail.adc err.\r\n",i);
     goto calibrate_zero_msg_handle; 
    }
   /*保留先前参数*/
    pre_nv_param = digital_scale.scale[i].nv_param;
    
    digital_scale.scale[i].nv_param.zero_adc = digital_scale.scale[i].cur_adc;
    digital_scale.scale[i].nv_param.zero_weight = msg->calibrate_weight;
    /*避免除法错误*/
    if(digital_scale.scale[i].nv_param.zero_adc == digital_scale.scale[i].nv_param.full_adc){
      digital_scale.scale[i].nv_param.full_adc+=1;
    }
    digital_scale.scale[i].nv_param.a = (float)(digital_scale.scale[i].nv_param.full_weight - digital_scale.scale[i].nv_param.zero_weight) / (float)(digital_scale.scale[i].nv_param.full_adc - digital_scale.scale[i].nv_param.zero_adc);
    digital_scale.scale[i].nv_param.b = (float)digital_scale.scale[i].nv_param.zero_weight - digital_scale.scale[i].nv_param.a * digital_scale.scale[i].nv_param.zero_adc;
    digital_scale.scale[i].nv_param.tar_weight = 0;
       
   nv_result = nv_save(digital_scale.scale[i].nv_param_addr,(uint8_t *)&digital_scale.scale[i].nv_param,sizeof(digital_scale.scale[i].nv_param));
   if(nv_result != 0){
   result = SCALE_TASK_FAILURE;
   /*恢复先前参数*/
   digital_scale.scale[i].nv_param = pre_nv_param ;
   log_error("scale%d  calibrate zero fail.nv param err.\r\n",i);
   goto calibrate_zero_msg_handle;
   }    
   log_info("scale%d calibrate zero success.nv a:%f b:%f.\r\n",i,digital_scale.scale[i].nv_param.a,digital_scale.scale[i].nv_param.b);
   }
    result = SCALE_TASK_SUCCESS;   
calibrate_zero_msg_handle:
    protocol_msg.type = RESPONSE_CALIBRATE_ZERO;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
   }
    
  /*向protocol_task回应满量程点校准结果*/
  if(msg->type ==  REQ_CALIBRATE_FULL){
   for(uint8_t i =0 ;i<digital_scale.scale_cnt;i++){   
   if(digital_scale.scale[i].cur_adc == ADC_TASK_SAMPLE_ERR_VALUE){
   result = SCALE_TASK_FAILURE; 
   log_error("scale%d calibrate  full fail.adc err.\r\n",i);
   goto calibrate_full_msg_handle;
   }
   /*保留先前参数*/
   pre_nv_param = digital_scale.scale[i].nv_param;
   
   digital_scale.scale[i].nv_param.full_adc = digital_scale.scale[i].cur_adc;
   digital_scale.scale[i].nv_param.full_weight = msg->calibrate_weight;
   /*避免除法错误*/
   if(digital_scale.scale[i].nv_param.zero_adc == digital_scale.scale[i].nv_param.full_adc){
      digital_scale.scale[i].nv_param.full_adc+=1;
   }
   digital_scale.scale[i].nv_param.a = (float)(digital_scale.scale[i].nv_param.full_weight - digital_scale.scale[i].nv_param.zero_weight) / (float)(digital_scale.scale[i].nv_param.full_adc - digital_scale.scale[i].nv_param.zero_adc);
   digital_scale.scale[i].nv_param.b = (float)digital_scale.scale[i].nv_param.full_weight - digital_scale.scale[i].nv_param.a * digital_scale.scale[i].nv_param.full_adc;
       
   nv_result = nv_save(digital_scale.scale[i].nv_param_addr,(uint8_t *)&digital_scale.scale[i].nv_param,sizeof(digital_scale.scale[i].nv_param));
   if(nv_result != 0){
   result = SCALE_TASK_FAILURE;
   /*恢复先前参数*/
   digital_scale.scale[i].nv_param = pre_nv_param ;
   log_error("scale%d calibrate full fail.nv param err.\r\n",i);
   goto calibrate_full_msg_handle;
   }
   log_info("scale%d calibrate full success.nv a:%f b:%f.\r\n",i,digital_scale.scale[i].nv_param.a,digital_scale.scale[i].nv_param.b);
   }
    result = SCALE_TASK_SUCCESS;   
calibrate_full_msg_handle:
    protocol_msg.type = RESPONSE_CALIBRATE_FULL;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
 }
 
  /*向protocol_task回应去皮结果*/
  if(msg->type ==  REQ_REMOVE_TAR_WEIGHT){
   for(uint8_t i=0;i<digital_scale.scale_cnt;i++){    
   if(digital_scale.scale[i].gross_weight == SCALE_TASK_WEIGHT_ERR_VALUE){
     result = SCALE_TASK_FAILURE ;
     log_error("scale %d remove tar weight fail.weight err.\r\n",i); 
     goto remove_tar_weight_msg_handle;
   }
    /*保留先前参数*/
    pre_nv_param = digital_scale.scale[i].nv_param;  
    digital_scale.scale[i].nv_param.tar_weight = digital_scale.scale[i].gross_weight;
    //scale.nv_param.b = (float)scale.gross_weight - scale.nv_param.a * scale.cur_adc; 
   
    nv_result = nv_save(digital_scale.scale[i].nv_param_addr,(uint8_t *)&digital_scale.scale[i].nv_param,sizeof(digital_scale.scale[i].nv_param));
    if(nv_result != 0){
    result = SCALE_TASK_FAILURE;
    /*恢复先前参数*/
    digital_scale.scale[i].nv_param = pre_nv_param ;
    log_error("scale%d remove tar weight fail.nv param err.\r\n",i);
    goto remove_tar_weight_msg_handle;
    }  
    log_info("scale %d remove tar weight success.nv a:%f b:%f.\r\n",i,digital_scale.scale[i].nv_param.a,digital_scale.scale[i].nv_param.b);
   }
   
    result = SCALE_TASK_SUCCESS;   
remove_tar_weight_msg_handle:
    protocol_msg.type = RESPONSE_REMOVE_TAR_WEIGHT;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
  }
 
 
  /*向protocol_task回应设置地址结果*/
  if(msg->type ==  REQ_SET_ADDR){
   if(msg->scale_addr > SCALE_TASK_ADDR_VALUE_MAX || msg->scale_addr == 0){
     log_error("set addr fail.addr:%d.\r\n",msg->scale_addr); 
     result = SCALE_TASK_FAILURE ;
     goto set_addr_msg_handle;
   }
   /*保存先前地址*/
   pre_nv_addr = digital_scale.nv_addr;
   
   digital_scale.nv_addr.addr = msg->scale_addr;   
       
   nv_result = nv_save(digital_scale.nv_addr_addr,(uint8_t *)&digital_scale.nv_addr,sizeof(digital_scale.nv_addr));
   if(nv_result != 0){
   result = SCALE_TASK_FAILURE;
   /*恢复先前地址*/
   digital_scale.nv_addr = pre_nv_addr;
   log_error("set scale addr fail.nv addr err.\r\n");
   goto set_addr_msg_handle;
   }
    result = SCALE_TASK_SUCCESS;   
    log_info("set scale addr success.nv addr:%d.\r\n",digital_scale.nv_addr.addr);
set_addr_msg_handle:
    protocol_msg.type = RESPONSE_SET_ADDR;
    protocol_msg.result = result;
    status = osMessagePut(protocol_task_msg_q_id,(uint32_t)(&protocol_msg),SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);
  }
 
  /*向protocol_task回应当前地址值*/
  if(msg->type ==  REQ_ADDR){
   protocol_msg.type = RESPONSE_ADDR;
   protocol_msg.scale_addr = digital_scale.nv_addr.addr;
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



