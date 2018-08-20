#include "board.h"
#include "cmsis_os.h"
#include "modbus.h"
#include "modbus_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[mb_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 


extern serial_hal_driver_t modbus_serial_driver;

modbus_t *ctx;
modbus_mapping_t* map;


static int slave_id = MODBUS_TASK_SLAVE_ID;

int modbus_task_get_stored_slave_id()
{

  return slave_id;
}


void modbus_task(void const * argument)
{
 int result; 
 int slave;
 uint8_t req[64];
 
 ctx = modbus_new_rtu(MODBUS_TASK_SERIAL_PORT,
                      MODBUS_TASK_SERIAL_BAUDRATE,
                      MODBUS_TASK_SERIAL_DATABITS, 
                      MODBUS_TASK_SERIAL_STOPBITS,
                      &modbus_serial_driver);
 log_assert(ctx);
 slave = modbus_task_get_stored_slave_id();
 result = modbus_set_slave(ctx,slave);
 log_assert(result == 0);
 result = modbus_connect(ctx);
 log_assert(result == 0);
 
 map = modbus_mapping_new_start_address(MODBUS_TASK_BITS_START_ADDR,MODBUS_TASK_BITS_CNT,
                                        MODBUS_TASK_INPUT_BITS_START_ADDR,MODBUS_TASK_INPUT_BITS_CNT,                                     
                                        MODBUS_TASK_REGS_START_ADDR,MODBUS_TASK_REGS_CNT,
                                        MODBUS_TASK_INPUT_REGS_START_ADDR,MODBUS_TASK_INPUT_REGS_CNT);
 log_assert(map);
 
 while(1){
 result = modbus_receive(ctx,req);
 if(result == -1){
 log_error("master wait error.\r\n ");
 continue;
 }
 result = modbus_reply(ctx, req, result, map);
 if(result == -1){
 log_error("master reply error.\r\n ");
 }
 }
  
}