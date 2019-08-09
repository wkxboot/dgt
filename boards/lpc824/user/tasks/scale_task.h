#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__


extern osThreadId   scale_task_hdl;
extern osMessageQId scale_task_msg_q_id;
void scale_task(void const *argument);

#define  SCALE_BUFFER_SIZE                          16
#define  SCALE_TASK_SUCCESS                         1
#define  SCALE_TASK_FAILURE                         2

#define  SCALE_ADDR_NAME_STR                       "addr"
#define  SCALE_A_NAME_STR                          "a"
#define  SCALE_B_NAME_STR                          "b"
#define  SCALE_TARE_NAME_STR                       "tare"
#define  SCALE_ZERO_WEIGHT_NAME_STR                "zero_w"
#define  SCALE_ZERO_ADC_NAME_STR                   "zero_adc"
#define  SCALE_FULL_WEIGHT_NAME_STR                "full_w"
#define  SCALE_FULL_ADC_NAME_STR                   "full_adc"

#define  SCALE_DEFAULT_A_VALUE                      (6.95875e-3)/**<默认a值*/
#define  SCALE_DEFAULT_B_VALUE                      (-3.75259e3)/**<默认b值*/
#define  SCALE_DEFAULT_TARE_VALUE                   (0)/**<默认净重值*/
#define  SCALE_DEFAULT_ADDR                         (1)/**<默认地址值*/
#define  SCALE_DEFAULT_ZERO_WEIGHT_VALUE            (0)/**<默认0点校准值0g*/
#define  SCALE_DEFAULT_ZERO_ADC_VALUE               (8500000UL)/**<默认0点校准ADC值*/
#define  SCALE_DEFAULT_FULL_WEIGHT_VALUE            (2000)/**<默认增益校准值2000g*/
#define  SCALE_DEFAULT_FULL_ADC_VALUE               (8800000UL)/**<默认增益校准ADC值*/

#define  SCALE_ADDR_VALUE_MIN                       0x01/**<最小地址值*/
#define  SCALE_ADDR_VALUE_MAX                       0xF7/**<最大地址值*/
#define  SCALE_WEIGHT_ERR_VALUE                     (0x7FFF)/**<重量错误值标志*/
#define  SCALE_MAX_WEIGHT_VALUE                     (32766) /**<最大重量值32766  g*/
#define  SCALE_MIN_WEIGHT_VALUE                     (-32768.0) /**<最小重量值-32768 g*/

#define  SCALE_TASK_PUT_MSG_TIMEOUT                 10
#define  SCALE_TASK_MSG_WAIT_TIMEOUT                osWaitForever    



enum
{
    SCALE_TASK_MSG_ADC_COMPLETE,
    SCALE_TASK_MSG_ADC_ERROR,
    SCALE_TASK_MSG_GET_NET_WEIGHT,
    SCALE_TASK_MSG_CALIBRATE_ZERO,
    SCALE_TASK_MSG_CALIBRATE_FULL,
    SCALE_TASK_MSG_REMOVE_TARE_WEIGHT,
    SCALE_TASK_MSG_SET_ADDR,
    SCALE_TASK_MSG_GET_SENSOR_ID,
    SCALE_TASK_MSG_GET_FW_VERSION,
    SCALE_TASK_MSG_GET_ADDR
};


typedef struct
{
    struct
    {
    uint32_t id;
    uint32_t type;
    }head;
    union 
    {
    uint32_t adc;
    int16_t calibration_weight;
    uint8_t addr_setting;
    }content;
}scale_task_message_t;








#endif