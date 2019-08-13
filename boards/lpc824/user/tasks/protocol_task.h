#ifndef  __PROTOCOL_H__
#define  __PROTOCOL_H__


extern osThreadId   protocol_task_hdl;
extern osMessageQId protocol_task_msg_q_id;
void protocol_task(void const * argument);

#define  PROTOCOL_TASK_START_DELAY_TIME_VALUE     1000

#define  PROTOCOL_TASK_UART_PORT                  1
#define  PROTOCOL_TASK_UART_BAUD_RATES            115200
#define  PROTOCOL_TASK_UART_DATA_BITS             8
#define  PROTOCOL_TASK_UART_STOP_BITS             1


#define  PROTOCOL_TASK_RX_BUFFER_SIZE             32
#define  PROTOCOL_TASK_TX_BUFFER_SIZE             32


enum {
    PROTOCOL_TASK_MSG_NET_WEIGHT_VALUE,
    PROTOCOL_TASK_MSG_CALIBRATE_ZERO_RESULT,
    PROTOCOL_TASK_MSG_CALIBRATE_FULL_RESULT,
    PROTOCOL_TASK_MSG_REMOVE_TARE_WEIGHT_RESULT,
    PROTOCOL_TASK_MSG_ADDR_VALUE,
    PROTOCOL_TASK_MSG_SET_ADDR_RESULT,
    PROTOCOL_TASK_MSG_SENSOR_ID_VALUE,
    PROTOCOL_TASK_MSG_FW_VERSION_VALUE
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
    uint32_t fw_version;
    int16_t net_weight;
    uint8_t result;
    uint8_t addr;
    uint8_t sensor_id;
    }content;
}protocol_task_message_t;



#endif