#ifndef  __PROTOCOL_H__
#define  __PROTOCOL_H__



#define  PROTOCOL_TASK_SERIAL_PORT                 0
#define  PROTOCOL_TASK_SERIAL_BAUDRATES            115200
#define  PROTOCOL_TASK_SERIAL_DATABITS             8
#define  PROTOCOL_TASK_SERIAL_STOPBITS             1


#define  PROTOCOL_TASK_RX_BUFFER_SIZE              32
#define  PROTOCOL_TASK_TX_BUFFER_SIZE              32

#define  PROTOCOL_TASK_FRAME_SIZE_MAX              7
#define  PROTOCOL_TASK_ADU_SIZE_MAX                3

#define  PROTOCOL_TASK_FRAME_TIMEOUT_VALUE         osWaitForever
#define  PROTOCOL_TASK_CHARACTER_TIMEOUT_VALUE     3

#define  PROTOCOL_TASK_CALIBRATE_TIMEOUT_VALUE            500
#define  PROTOCOL_TASK_REMOVE_TAR_WEIGHT_TIMEOUT_VALUE    500


/*协议定义*/
#define  PROTOCOL_TASK_HEADER0_VALUE               'M'
#define  PROTOCOL_TASK_HEADER1_VALUE               'L'

#define  PROTOCOL_TASK_FUNC_READ_NET_WEIGHT        0x00
#define  PROTOCOL_TASK_FUNC_REMOVE_TAR_WEIGHT      0x01
#define  PROTOCOL_TASK_FUNC_CALIBRATE_ZERO         0x02
#define  PROTOCOL_TASK_FUNC_CALIBRATE_FULL         0x03
#define  PROTOCOL_TASK_FUNC_READ_SENSOR_ID         0x04
#define  PROTOCOL_TASK_FUNC_READ_VERSION           0x05

#define  PROTOCOL_TASK_SUCCESS_VALUE               0x00
#define  PROTOCOL_TASK_FAILURE_VALUE               0x01




#define  PROTOCOL_TASK_READ_NET_WEIGHT_FRAME_LEN      5
#define  PROTOCOL_TASK_REMOVE_TAR_WEIGHT_FRAME_LEN    5
#define  PROTOCOL_TASK_CALIBRATE_ZERO_FRAME_LEN       7
#define  PROTOCOL_TASK_CALIBRATE_FULL_FRAME_LEN       7
#define  PROTOCOL_TASK_READ_SENSOR_ID_FRAME_LEN       5
#define  PROTOCOL_TASK_READ_VERSION_FRAME_LEN         5



#endif