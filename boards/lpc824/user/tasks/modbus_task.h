#ifndef  __MODBUS_TASK_H__
#define  __MODBUS_TASK_H__



#define  MODBUS_TASK_SERIAL_PORT                  0
#define  MODBUS_TASK_SERIAL_BAUDRATE              115200 
#define  MODBUS_TASK_SERIAL_DATABITS              8 
#define  MODBUS_TASK_SERIAL_STOPBITS              1 

#define  MODBUS_TASK_SLAVE_ID                     1

#define  MODBUS_TASK_BITS_START_ADDR              0
#define  MODBUS_TASK_BITS_CNT                     0
#define  MODBUS_TASK_INPUT_BITS_START_ADDR        0
#define  MODBUS_TASK_INPUT_BITS_CNT               0

#define  MODBUS_TASK_INPUT_REGS_START_ADDR        0
#define  MODBUS_TASK_INPUT_REGS_CNT               0
#define  MODBUS_TASK_REGS_START_ADDR              0
#define  MODBUS_TASK_REGS_CNT                     97

void modbus_task(void const * argument);






















#endif