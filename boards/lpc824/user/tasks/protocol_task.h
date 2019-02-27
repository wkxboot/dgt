#ifndef  __PROTOCOL_H__
#define  __PROTOCOL_H__


extern osThreadId   protocol_task_hdl;
extern osMessageQId protocol_task_msg_q_id;
void protocol_task(void const * argument);

#define  PROTOCOL_TASK_START_DELAY_TIME_VALUE      1000

#define  PROTOCOL_TASK_SERIAL_PORT                 1
#define  PROTOCOL_TASK_SERIAL_BAUDRATES            115200
#define  PROTOCOL_TASK_SERIAL_DATABITS             8
#define  PROTOCOL_TASK_SERIAL_STOPBITS             1


#define  PROTOCOL_TASK_RX_BUFFER_SIZE              32
#define  PROTOCOL_TASK_TX_BUFFER_SIZE              32

#endif