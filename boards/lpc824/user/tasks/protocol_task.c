#include "board.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "modbus.h"
#include "log.h"
#define LOG_MODULE_NAME   "[protocol]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_ERROR 

extern int modbus_serial_handle;
extern serial_hal_driver_t modbus_serial_driver;

osThreadId protocol_task_hdl;
osMessageQId protocol_task_msg_q_id;

static task_message_t scale_msg;

static modbus_t *ctx;
static modbus_mapping_t *scale_regs_map;
static uint8_t req[64];


#define  REG_WRITE_TRUE      1
#define  REG_WRITE_FALSE     0

typedef struct
{
 uint8_t   write_flag;
 uint16_t  value;
}reg_write_flag;


#define  REGS_CNT                       4
#define  REGS_START_ADDR                0x00
#define  REG_ADDR_OFFSET                0
#define  REG_TAR_OFFSET                 1
#define  REG_CALIBRATE_ZERO_OFFSET      2
#define  REG_CALIBRATE_FULL_OFFSET      3

#define  INPUT_REGS_CNT                 3
#define  INPUT_REGS_START_ADDR          0x100
#define  INPUT_REG_NET_WEIGHT_OFFSET    0
#define  INPUT_REG_SENSOR_ID_OFFSET     1
#define  INPUT_REG_VERSION_OFFSET       2



reg_write_flag regs_flags[REGS_CNT];



static int16_t protocol_get_net_weight()
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 int16_t net_weight =0;
 
 scale_msg.type = REQ_NET_WEIGHT;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg =  (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_NET_WEIGHT){
   net_weight = msg->net_weight;
   break;
  }  
 }
 }
 return net_weight;
}


static int protocol_remove_tar_weight()
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 int        result = -1;
 
 scale_msg.type = REQ_REMOVE_TAR_WEIGHT;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_REMOVE_TAR_WEIGHT){
   if( msg->result == SCALE_TASK_SUCCESS){
     result = 0;
   }else{
     result = -1;
   }  
  break;
  }
 }
 }
 return result;
}

static int protocol_calibrate_weight(int16_t weight)
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 int        result = -1;
 
 if(weight == 0){
 scale_msg.type = REQ_CALIBRATE_ZERO;
 }else{
 scale_msg.type = REQ_CALIBRATE_FULL; 
 } 
 scale_msg.calibrate_weight = weight;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_CALIBRATE_ZERO || msg->type == RESPONSE_CALIBRATE_FULL){
   if( msg->result == SCALE_TASK_SUCCESS){
     result = 0;
   }else{
     result = -1;
   }  
  break;
  }
 }
 }
 return result;
}

static uint8_t protocol_get_sensor_id()
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 uint8_t sensor_id = 0;
 
 scale_msg.type = REQ_SENSOR_ID;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg =  (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_SENSOR_ID){
   sensor_id = msg->sensor_id;
   break;
  }  
 }
 }
 return sensor_id;
}

static uint16_t protocol_get_firmware_version()
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 uint16_t   version = 0;
 
 scale_msg.type = REQ_VERSION;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg =  (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_VERSION){
   version = msg->version;
   break;
  }  
 }
 }
 return version;
}

static uint8_t protocol_get_scale_addr()
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 uint16_t   addr = 0;
 
 scale_msg.type = REQ_ADDR;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg =  (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_ADDR){
   addr = msg->scale_addr;
   break;
  }  
 }
 }
 return addr;
}

int protocol_set_scale_addr(uint8_t addr)
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 int        result = -1;
 
 scale_msg.type = REQ_SET_ADDR;
 scale_msg.scale_addr = addr;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_SET_ADDR){
   if( msg->result == SCALE_TASK_SUCCESS){
     result = 0;
   }else{
     result = -1;
   }  
  break;
  }
 }
 }
 return result;
}


static uint16_t protocol_get_calibrate_zero_value()
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 uint16_t value = 0;
 
 scale_msg.type = REQ_CALIBRATE_ZERO_VALUE;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg =  (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_CALIBRATE_ZERO_VALUE){
   value = msg->calibrate_weight;
   break;
  }  
 }
 }
 return value;
}

static uint16_t protocol_get_calibrate_full_value()
{
 osStatus status;
 osEvent  os_msg;
 task_message_t *msg;
 uint16_t value = 0;
 
 scale_msg.type = REQ_CALIBRATE_FULL_VALUE;
 status = osMessagePut(scale_task_msg_q_id,(uint32_t)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
 log_assert(status == osOK);
 while(1){
 os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg =  (task_message_t *)os_msg.value.v;
 if(msg->type == RESPONSE_CALIBRATE_FULL_VALUE){
   value = msg->calibrate_weight;
   break;
  }  
 }
 }
 return value;
}





/*485操作相关*/
void modbus_rtu_send_pre()
{
 bsp_485_enable_write();  
}
void modbus_rtu_send_after()
{
bsp_485_enable_read();
}
  

/*写寄存器值校验回调*/
int modbus_write_registers_check_callback(modbus_t *ctx,int addr_offset,const uint8_t *value,int cnt)
{
  int rc;
  int value_offset = 0;
  uint16_t addr;
  int16_t  calibrate_weight;
  uint16_t tar;
  
  for(int i = cnt; i > 0;i--){
  regs_flags[addr_offset].value = value[value_offset] << 8|value[value_offset+1];
  regs_flags[addr_offset].write_flag = REG_WRITE_TRUE;
  value_offset+=2;
  }
  
  if(regs_flags[REG_ADDR_OFFSET].write_flag == REG_WRITE_TRUE){
    regs_flags[REG_ADDR_OFFSET].write_flag = REG_WRITE_FALSE;
    addr = regs_flags[REG_ADDR_OFFSET].value;
    if(addr > 0xff){
     return -1;
    }
    rc = protocol_set_scale_addr(addr);
    if(rc !=0){
    return -1; 
    }
    rc = modbus_set_slave(ctx,addr); 
    if(rc !=0){
    return -1; 
   }
       
  }
  
  if(regs_flags[REG_TAR_OFFSET].write_flag == REG_WRITE_TRUE){
   regs_flags[REG_TAR_OFFSET].write_flag = REG_WRITE_FALSE;
   tar = regs_flags[REG_TAR_OFFSET].value;
   if(tar != 0x00){
   return -1;
   }
   rc = protocol_remove_tar_weight();
   if(rc !=0){
   return -1; 
   }
  }
  
  if(regs_flags[REG_CALIBRATE_ZERO_OFFSET].write_flag == REG_WRITE_TRUE){
   regs_flags[REG_CALIBRATE_ZERO_OFFSET].write_flag = REG_WRITE_FALSE;
   calibrate_weight = regs_flags[REG_CALIBRATE_ZERO_OFFSET].value;
   if(calibrate_weight != 0x00){
   return -1;
   }
   rc = protocol_calibrate_weight(calibrate_weight);
   if(rc !=0){
   return -1; 
   }
  } 
  
  if(regs_flags[REG_CALIBRATE_FULL_OFFSET].write_flag == REG_WRITE_TRUE){
   regs_flags[REG_CALIBRATE_FULL_OFFSET].write_flag = REG_WRITE_FALSE;
   calibrate_weight = regs_flags[REG_CALIBRATE_FULL_OFFSET].value;
   if(calibrate_weight == 0x00){
   return -1;
   }
   rc = protocol_calibrate_weight(calibrate_weight);
   if(rc !=0){
   return -1; 
   }

  } 
  
 return 0 ;
}
  


void protocol_task(void const * argument)
{

 int rc; 
 int read_length=0;
 uint8_t  scale_addr;
 uint16_t version;
 uint8_t  sensor_id;
 int16_t  calibrate_value;
 int16_t  net_weight;
 
 
 osMessageQDef(protocol_task_msg_q,6,uint32_t);
 protocol_task_msg_q_id = osMessageCreate(osMessageQ(protocol_task_msg_q),protocol_task_hdl);
 log_assert(protocol_task_msg_q_id); 
 
 ctx = modbus_new_rtu(PROTOCOL_TASK_MODBUS_SERIAL_PORT,
                      PROTOCOL_TASK_MODBUS_SERIAL_BAUDRATES,
                      PROTOCOL_TASK_MODBUS_SERIAL_DATABITS, 
                      PROTOCOL_TASK_MODBUS_SERIAL_STOPBITS,
                      &modbus_serial_driver);
 log_assert(ctx);
 modbus_serial_handle = ctx->s;
 rc = modbus_connect(ctx);
 log_assert(rc == 0);
 
 scale_regs_map = modbus_mapping_new_start_address(0,0,
                                                   0,0,
                                                   REGS_START_ADDR,REGS_CNT,
                                                   INPUT_REGS_START_ADDR,INPUT_REGS_CNT);
 log_assert(scale_regs_map);
 
 /*等待scale_task启动完毕*/
 osDelay(PROTOCOL_TASK_START_DELAY_TIME_VALUE);
 
 /*读取上电后当前地址值*/
 scale_addr = protocol_get_scale_addr();
 scale_regs_map->tab_registers[REG_ADDR_OFFSET] = scale_addr;
 modbus_set_slave(ctx,scale_addr);
 
 /*获取传感器id*/
 sensor_id = protocol_get_sensor_id();
 scale_regs_map->tab_input_registers[INPUT_REG_SENSOR_ID_OFFSET] = sensor_id;
 
 /*获取版本号*/
 version = protocol_get_firmware_version();
 scale_regs_map->tab_input_registers[INPUT_REG_VERSION_OFFSET] = version;
 /*获取0点校准重量值*/
 calibrate_value =protocol_get_calibrate_zero_value();
 scale_regs_map->tab_registers[REG_CALIBRATE_ZERO_OFFSET] = calibrate_value;
 /*获取增益校准重量值*/
 calibrate_value =protocol_get_calibrate_full_value();
 scale_regs_map->tab_registers[REG_CALIBRATE_FULL_OFFSET] = calibrate_value;
 
 serial_flush(modbus_serial_handle);
 /*启动接收*/
 modbus_rtu_send_after();
 while(1){
   
 read_length = modbus_receive(ctx,req);
 if(read_length > 0){
 net_weight = protocol_get_net_weight();
 scale_regs_map->tab_input_registers[INPUT_REG_NET_WEIGHT_OFFSET] = net_weight; 
 rc = modbus_reply(ctx,req,read_length,scale_regs_map);
 if(rc > 0){
 log_debug("modbus process success.\r\n");  
 }else{
 log_error("response err:%d.\r\n",rc);
 serial_flush(modbus_serial_handle); 
 }
 }else{
 serial_flush(modbus_serial_handle); 
 }
 }
}
