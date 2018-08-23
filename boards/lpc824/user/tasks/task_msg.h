#ifndef  __TASK_MSG_H__
#define  __TASK_MSG_H__





typedef enum
{
ADC_SAMPLE_COMPLETED=0,
REQ_NET_WEIGHT,
REQ_CALIBRATE_ZERO,
REQ_CALIBRATE_FULL,
REQ_REMOVE_TAR_WEIGHT,
REQ_SENSOR_ID,
REQ_VERSION,
REQ_SET_ADDR,
REQ_ADDR,
RESPONSE_NET_WEIGHT,
RESPONSE_CALIBRATE_ZERO,
RESPONSE_CALIBRATE_FULL,
RESPONSE_REMOVE_TAR_WEIGHT,
RESPONSE_SENSOR_ID,
RESPONSE_VERSION,
RESPONSE_SET_ADDR,
RESPONSE_ADDR
}msg_type_t;

typedef struct
{
msg_type_t   type;
union{
uint32_t     adc;
int16_t      net_weight;
int16_t      calibrate_weight;
uint8_t      result;
uint8_t      sensor_id;
uint16_t     version;
uint8_t      scale_addr;
};
}task_msg_t;



















#endif