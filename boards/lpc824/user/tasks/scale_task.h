#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__


extern osThreadId   scale_task_hdl;
extern osMessageQId scale_task_msg_q_id;
void scale_task(void const *argument);


#define  SCALE_TASK_CALCULATE_VARIANCE              0

#define  SCALE_BUFFER_SIZE                          16
#define  SCALE_TASK_SUCCESS                         1
#define  SCALE_TASK_FAILURE                         2

#define  SCALE_ADDR_NAME_STR                       "addr"
#define  SCALE_A_NAME_STR                          "a"
#define  SCALE_B_NAME_STR                          "b"
#define  SCALE_TARE_NAME_STR                       "tare"

#define  SCALE_TASK_DEFAULT_A_VALUE                 (6.95875e-3)
#define  SCALE_TASK_DEFAULT_B_VALUE                 (-3.75259e3)
#define  SCALE_TASK_DEFAULT_TARE_VALUE              (0)
#define  SCALE_ADDR_DEFAULT                         1


#define  SCALE_TASK_ADDR_VALUE_MAX                  0xF7
#define  SCALE_TASK_WEIGHT_ERR_VALUE                0x7FFF
#define  SCALE_TASK_MAX_WEIGHT_VALUE                (32767.0)  /*最大32767  g*/
#define  SCALE_TASK_MIN_WEIGHT_VALUE                (-32767.0) /*最小-32767 g*/
#define  SCALE_TASK_DIFF_WEIGHT                     50 /*重力变化阈值*/
#define  SCALE_TASK_MSG_PUT_TIMEOUT_VALUE           10
#define  SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE          osWaitForever    













#endif