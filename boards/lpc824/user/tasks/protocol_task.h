#ifndef  __PROTOCOL_H__
#define  __PROTOCOL_H__


extern osThreadId   protocol_task_hdl;
extern osMessageQId protocol_task_msg_q_id;
void protocol_task(void const * argument);

#define  PROTOCOL_TASK_START_DELAY_TIME_VALUE      1000

#define  PROTOCOL_TASK_MODBUS_SERIAL_PORT                 1
#define  PROTOCOL_TASK_MODBUS_SERIAL_BAUDRATES            115200
#define  PROTOCOL_TASK_MODBUS_SERIAL_DATABITS             8
#define  PROTOCOL_TASK_MODBUS_SERIAL_STOPBITS             1

#define  PROTOCOL_TASK_SEND_TIMEOUT                5

#define  PROTOCOL_TASK_RX_BUFFER_SIZE              32
#define  PROTOCOL_TASK_TX_BUFFER_SIZE              32

#define  PROTOCOL_TASK_FRAME_SIZE_MAX              10
#define  PROTOCOL_TASK_ADU_SIZE_MAX                4
#define  PROTOCOL_TASK_ADU_SIZE_MIN                2


#define  PROTOCOL_TASK_HEADER0_OFFSET              0
#define  PROTOCOL_TASK_HEADER1_OFFSET              1
#define  PROTOCOL_TASK_HEADER_SIZE                 2

#define  PROTOCOL_TASK_SIZE_OFFSET                 2
#define  PROTOCOL_TASK_SIZE_SIZE                   2

#define  PROTOCOL_TASK_ADU_ADDR_OFFSET             4
#define  PROTOCOL_TASK_ADU_ADDR_SIZE               1

#define  PROTOCOL_TASK_ADU_FUNC_OFFSET             5
#define  PROTOCOL_TASK_ADU_FUNC_SIZE               1

#define  PROTOCOL_TASK_ADU_PAYLOAD_OFFSET          6
#define  PROTOCOL_TASK_CRC_SIZE                    2


#define  PROTOCOL_TASK_FRAME_TIMEOUT_VALUE         osWaitForever
#define  PROTOCOL_TASK_CHARACTER_TIMEOUT_VALUE     3

#define  PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE       5
#define  PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE      200


/*协议定义*/
#define  PROTOCOL_TASK_HEADER0_VALUE               'M'
#define  PROTOCOL_TASK_HEADER1_VALUE               'L'

#define  PROTOCOL_TASK_FUNC_READ_NET_WEIGHT        0x00
#define  PROTOCOL_TASK_FUNC_REMOVE_TAR_WEIGHT      0x01
#define  PROTOCOL_TASK_FUNC_CALIBRATE_ZERO         0x02
#define  PROTOCOL_TASK_FUNC_CALIBRATE_FULL         0x03
#define  PROTOCOL_TASK_FUNC_READ_SENSOR_ID         0x04
#define  PROTOCOL_TASK_FUNC_READ_VERSION           0x05
#define  PROTOCOL_TASK_FUNC_SET_ADDR               0x06

#define  PROTOCOL_TASK_SUCCESS_VALUE               0x00
#define  PROTOCOL_TASK_FAILURE_VALUE               0x01

#define  PROTOCOL_TASK_WEIGHT_ERR_VALUE            0x7FFF


#define  PROTOCOL_TASK_READ_NET_WEIGHT_FRAME_LEN                8
#define  PROTOCOL_TASK_REMOVE_TAR_WEIGHT_FRAME_LEN              8
#define  PROTOCOL_TASK_CALIBRATE_ZERO_FRAME_LEN                 10
#define  PROTOCOL_TASK_CALIBRATE_FULL_FRAME_LEN                 10
#define  PROTOCOL_TASK_READ_SENSOR_ID_FRAME_LEN                 8
#define  PROTOCOL_TASK_READ_VERSION_FRAME_LEN                   8
#define  PROTOCOL_TASK_SET_ADDR_FRAME_LEN                       9


#define  PROTOCOL_TASK_RESPONSE_NET_WEIGHT_FRAME_LEN            10
#define  PROTOCOL_TASK_RESPONSE_REMOVE_TAR_WEIGHT_FRAME_LEN     9
#define  PROTOCOL_TASK_RESPONSE_CALIBRATE_ZERO_FRAME_LEN        9
#define  PROTOCOL_TASK_RESPONSE_CALIBRATE_FULL_FRAME_LEN        9
#define  PROTOCOL_TASK_RESPONSE_READ_SENSOR_ID_FRAME_LEN        9
#define  PROTOCOL_TASK_RESPONSE_READ_VERSION_FRAME_LEN          10
#define  PROTOCOL_TASK_RESPONSE_SET_ADDR_FRAME_LEN              9

#endif