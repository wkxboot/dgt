#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__


extern osThreadId   scale_task_hdl;
extern osMessageQId scale_task_msg_q_id;
void scale_task(void const *argument);


#define  SCALE_TASK_SUCCESS                         1
#define  SCALE_TASK_FAILURE                         2

#define  SCALE_TASK_NV_INVALID                      0x55
#define  SCALE_TASK_NV_VALID                        0xaa

#define  SCALE_TASK_NV_ADDR_ADDR                    0x00
#define  SCALE_TASK_SCALE_1_NV_PARAM_ADDR           0x10
#define  SCALE_TASK_SCALE_2_NV_PARAM_ADDR           0x30


#define  SCALE_TASK_ADDR_VALUE_MAX                  0xFE
#define  SCALE_TASK_WEIGHT_ERR_VALUE                0x7FFF
#define  SCALE_TASK_MAX_WEIGHT_VALUE                (32767.0)  /*最大32767  g*/
#define  SCALE_TASK_MIN_WEIGHT_VALUE                (-32767.0) /*最小-32767 g*/

#define  SCALE_TASK_MSG_PUT_TIMEOUT_VALUE           10
#define  SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE          osWaitForever    













#endif