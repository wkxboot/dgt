#include "board.h"
#include "cmsis_os.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "nxp_cm0_uart_hal_driver.h"
#include "xuart.h"
#include "xstring.h"
#include "crc16.h"
#include "log.h"


static xuart_handle_t protocol_uart_handle;
static uint8_t recv_buffer[PROTOCOL_TASK_RX_BUFFER_SIZE];
static uint8_t send_buffer[PROTOCOL_TASK_TX_BUFFER_SIZE];

osThreadId protocol_task_hdl;
osMessageQId protocol_task_msg_q_id;


/*通信协议部分*/
/*ADU*/
#define  ADU_HEAD0_OFFSET              0
#define  ADU_HEAD1_OFFSET              1
#define  ADU_HEAD0_VALUE               'M'
#define  ADU_HEAD1_VALUE               'L'
#define  ADU_HEAD_SIZE                 2

#define  ADU_PDU_SIZE_REGION_OFFSET    2
#define  ADU_PDU_SIZE_REGION_SIZE      1

#define  ADU_PDU_REGION_OFFSET         3
#define  PDU_ADDR_OFFSET               0
#define  PDU_ADDR_SIZE                 1
#define  PDU_CODE_OFFSET               1
#define  PDU_CODE_SIZE                 1
#define  PDU_VALUE_OFFSET              2
#define  PDU_VALUE_SIZE_MAX            2
#define  ADU_CRC_SIZE                  2


#define  ADU_SIZE_MAX                  (ADU_HEAD_SIZE + ADU_PDU_SIZE_REGION_SIZE + PDU_ADDR_SIZE + PDU_CODE_SIZE + PDU_VALUE_SIZE_MAX + ADU_CRC_SIZE)
#define  ADU_SIZE_MIN                  (ADU_HEAD_SIZE + ADU_PDU_SIZE_REGION_SIZE + PDU_ADDR_SIZE + PDU_CODE_SIZE + ADU_CRC_SIZE)
#define  ADU_EXCLUDE_PDU_SIZE          (ADU_HEAD_SIZE + ADU_PDU_SIZE_REGION_SIZE + ADU_CRC_SIZE)

/*PDU*/
#define  PDU_CODE_NET_WEIGHT           0
#define  PDU_CODE_REMOVE_TARE_WEIGHT   1
#define  PDU_CODE_CALIBRATION_ZERO     2
#define  PDU_CODE_CALIBRATION_FULL     3
#define  PDU_CODE_SENSOR_ID            4
#define  PDU_CODE_FIRMWARE_VERSION     5
#define  PDU_CODE_SET_ADDR             6
#define  PDU_CODE_MAX                  PDU_CODE_SET_ADDR

#define  PDU_CONTROLLER_ADDR           1
/*协议错误码*/
#define  PDU_NET_WEIGHT_ERR_VALUE      0x7FFF
#define  PDU_SUCCESS_VALUE             0x00
#define  PDU_FAILURE_VALUE             0x01

/*协议时间*/
#define  PROTOCOL_TASK_ADU_WAIT_TIMEOUT            osWaitForever
#define  PROTOCOL_TASK_ADU_FRAME_TIMEOUT           3
#define  PROTOCOL_TASK_ADU_SEND_TIMEOUT            5

#define  PROTOCOL_TASK_GET_NET_WEIGHT_TIMEOUT      10
#define  PROTOCOL_TASK_GET_FW_VERSION_TIMEOUT      10
#define  PROTOCOL_TASK_GET_ADDR_TIMEOUT            10
#define  PROTOCOL_TASK_GET_SENSOR_ID_TIMEOUT       10
#define  PROTOCOL_TASK_REMOVE_TARE_WEIGHT_TIMEOUT  240
#define  PROTOCOL_TASK_CALIBRATE_ZERO_TIMEOUT      240
#define  PROTOCOL_TASK_CALIBRATE_FULL_TIMEOUT      240
#define  PROTOCOL_TASK_SET_ADDR_TIMEOUT            240

#define  PROTOCOL_TASK_PUT_MSG_TIMEOUT             5

/**
* @brief 计算发送缓存CRC并填充到发送缓存
* @param adu 回应缓存
* @param size 当前数据缓存长度
* @return 加上crc后的数据长度
* @note
*/
static uint8_t adu_add_crc16(uint8_t *adu,uint8_t size)
{
    uint16_t crc16;
    crc16 = calculate_crc16(adu,size);

    adu[size ++] = crc16 & 0xff;
    adu[size ++] = crc16 >> 8;
    return size;
}

/**
* @brief 
* @param
* @param
* @return 
* @note
*/
static int request_net_weight(int16_t *weight)
{
    scale_task_message_t msg_send;
    protocol_task_message_t msg_recv;
 
    msg_send.head.id = SCALE_TASK_MSG_GET_NET_WEIGHT;

    log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&msg_send,PROTOCOL_TASK_PUT_MSG_TIMEOUT) == pdPASS);

    if (xQueueReceive(protocol_task_msg_q_id, &msg_recv,PROTOCOL_TASK_GET_NET_WEIGHT_TIMEOUT) == pdTRUE) {
        if (msg_recv.head.id == PROTOCOL_TASK_MSG_NET_WEIGHT_VALUE){ 
            *weight = msg_recv.content.net_weight;  
            return 0;
        }      
    }

    return -1;
}

/**
* @brief 
* @param
* @param
* @return 
* @note
*/
static int request_remove_tare_weight(void)
{

    scale_task_message_t msg_send;
    protocol_task_message_t msg_recv;
 
    msg_send.head.id = SCALE_TASK_MSG_REMOVE_TARE_WEIGHT;

    log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&msg_send,PROTOCOL_TASK_PUT_MSG_TIMEOUT) == pdPASS);

    if (xQueueReceive(protocol_task_msg_q_id, &msg_recv,PROTOCOL_TASK_REMOVE_TARE_WEIGHT_TIMEOUT) == pdTRUE) {
        if (msg_recv.head.id == PROTOCOL_TASK_MSG_REMOVE_TARE_WEIGHT_RESULT && msg_recv.content.result == SCALE_TASK_SUCCESS){
            return 0;
        }      
    }

    return -1;
}

/**
* @brief 
* @param
* @param
* @return 
* @note
*/

static int request_calibrate_zero_weight(int16_t weight)
{
    scale_task_message_t msg_send;
    protocol_task_message_t msg_recv;
 
    msg_send.head.id = SCALE_TASK_MSG_CALIBRATE_ZERO;
    msg_send.content.calibration_weight = 0;
    log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&msg_send,PROTOCOL_TASK_PUT_MSG_TIMEOUT) == pdPASS);

    if (xQueueReceive(protocol_task_msg_q_id, &msg_recv,PROTOCOL_TASK_CALIBRATE_ZERO_TIMEOUT) == pdTRUE) {
        if (msg_recv.head.id == PROTOCOL_TASK_MSG_CALIBRATE_ZERO_RESULT && msg_recv.content.result == SCALE_TASK_SUCCESS){
            return 0;
        }      
    }

    return -1;
}

/**
* @brief 
* @param
* @param
* @return 
* @note
*/
static int request_calibrate_full_weight(int16_t weight)
{
    scale_task_message_t msg_send;
    protocol_task_message_t msg_recv;
 
    if (weight <= 0) {
        return -1;
    }

    msg_send.head.id = SCALE_TASK_MSG_CALIBRATE_FULL;
    msg_send.content.calibration_weight = weight;
    log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&msg_send,PROTOCOL_TASK_PUT_MSG_TIMEOUT) == pdPASS);

    if (xQueueReceive(protocol_task_msg_q_id, &msg_recv,PROTOCOL_TASK_CALIBRATE_FULL_TIMEOUT) == pdTRUE) {
        if (msg_recv.head.id == PROTOCOL_TASK_MSG_CALIBRATE_FULL_RESULT && msg_recv.content.result == SCALE_TASK_SUCCESS){
            return 0;
        }      
    }

    return -1;
}


/**
* @brief 
* @param
* @param
* @return 
* @note
*/
static int request_sensor_id(uint8_t *sensor_id)
{
    scale_task_message_t msg_send;
    protocol_task_message_t msg_recv;
 
    msg_send.head.id = SCALE_TASK_MSG_GET_SENSOR_ID;
    log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&msg_send,PROTOCOL_TASK_PUT_MSG_TIMEOUT) == pdPASS);

    if (xQueueReceive(protocol_task_msg_q_id, &msg_recv,PROTOCOL_TASK_GET_SENSOR_ID_TIMEOUT) == pdTRUE) {
        if (msg_recv.head.id == PROTOCOL_TASK_MSG_SENSOR_ID_VALUE) {
            *sensor_id = msg_recv.content.sensor_id;
            return 0;
        }      
    }

    return -1;
}

/**
* @brief 
* @param
* @param
* @return 
* @note
*/
static int request_firmware_version(uint32_t *fw_version)
{
    scale_task_message_t msg_send;
    protocol_task_message_t msg_recv;
 
    msg_send.head.id = SCALE_TASK_MSG_GET_FW_VERSION;
    log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&msg_send,PROTOCOL_TASK_PUT_MSG_TIMEOUT) == pdPASS);

    if (xQueueReceive(protocol_task_msg_q_id, &msg_recv,PROTOCOL_TASK_GET_FW_VERSION_TIMEOUT) == pdTRUE) {
        if (msg_recv.head.id == PROTOCOL_TASK_MSG_FW_VERSION_VALUE) {
            *fw_version = msg_recv.content.fw_version;
            return 0;
        }      
    }

    return -1;
}
/**
* @brief 
* @param
* @param
* @return 
* @note
*/
static int request_scale_addr(uint8_t *addr)
{
    scale_task_message_t msg_send;
    protocol_task_message_t msg_recv;
 
    msg_send.head.id = SCALE_TASK_MSG_GET_ADDR;
    log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&msg_send,PROTOCOL_TASK_PUT_MSG_TIMEOUT) == pdPASS);

    if (xQueueReceive(protocol_task_msg_q_id, &msg_recv,PROTOCOL_TASK_GET_ADDR_TIMEOUT) == pdTRUE) {
        if (msg_recv.head.id == PROTOCOL_TASK_MSG_ADDR_VALUE) {
            *addr = msg_recv.content.addr;
            return 0;
        }      
    }

    return -1;
}

/**
* @brief 
* @param
* @param
* @return 
* @note
*/

int request_set_scale_addr(uint8_t addr)
{
    scale_task_message_t msg_send;
    protocol_task_message_t msg_recv;
 
    msg_send.head.id = SCALE_TASK_MSG_SET_ADDR;
    msg_send.content.addr_setting = addr;
    log_assert_bool_false(xQueueSend(scale_task_msg_q_id,&msg_send,PROTOCOL_TASK_PUT_MSG_TIMEOUT) == pdPASS);

    if (xQueueReceive(protocol_task_msg_q_id, &msg_recv,PROTOCOL_TASK_SET_ADDR_TIMEOUT) == pdTRUE) {
        if (msg_recv.head.id == PROTOCOL_TASK_MSG_SET_ADDR_RESULT && msg_recv.content.result == SCALE_TASK_SUCCESS) {
            return 0;
        }      
    }

    return -1;
}


/**
* @brief 串口接收主机ADU
* @param handle 串口句柄
* @param adu 数据缓存指针
* @param wait_timeout 等待超时时间
* @return -1 失败
* @return  0 成功
* @note
*/
static int receive_adu(xuart_handle_t *handle,uint8_t *adu,uint32_t wait_timeout)
{
    uint32_t select_size;
    uint32_t read_size,read_size_total = 0;
    uint32_t timeout;
    timeout = wait_timeout;
    uint8_t buffer[ADU_SIZE_MAX * 2 + 1];

    select_size = xuart_select(handle,timeout);

    if (select_size == 0) {
        log_error("adu recv timeout:%d ms.\r\n",timeout);
        return -1;
    }

    while(select_size != 0) {
        if (read_size_total + select_size > ADU_SIZE_MAX) {
            log_error("adu size total:%d > max:%d. err.\r\n",read_size_total + select_size,ADU_SIZE_MAX);
            return -1;
        }
        read_size = xuart_read(handle,adu + read_size_total,select_size);
        read_size_total += read_size;
        select_size = xuart_select(handle,PROTOCOL_TASK_ADU_FRAME_TIMEOUT);
    }
    
    xstring_hex_to_string((char *)buffer,(char *)adu,read_size_total);
    log_debug("adu recv:[%s]\r\n",buffer);

    return read_size_total;
}


#define  PDU_NET_WEIGHT_VALUE_SIZE                 0
#define  PDU_NET_WEIGHT_SUCCESS_VALUE              0
#define  PDU_NET_WEIGHT_FAIL_VALUE                 1

#define  PDU_REMOVE_TARE_VALUE_SIZE                0
#define  PDU_REMOVE_TARE_SUCCESS_VALUE             0
#define  PDU_REMOVE_TARE_FAIL_VALUE                1

#define  PDU_CALIBRATION_ZERO_VALUE_SIZE           2
#define  PDU_CALIBRATION_ZERO_SUCCESS_VALUE        0
#define  PDU_CALIBRATION_ZERO_FAIL_VALUE           1

#define  PDU_CALIBRATION_FULL_VALUE_SIZE           2
#define  PDU_CALIBRATION_FULL_SUCCESS_VALUE        0
#define  PDU_CALIBRATION_FULL_FAIL_VALUE           1

#define  PDU_SENSOR_ID_VALUE_SIZE                  0


#define  PDU_FW_VERSION_VALUE_SIZE                 0


#define  PDU_SET_ADDR_VALUE_SIZE                   1
#define  PDU_SET_ADDR_SUCCESS_VALUE                0
#define  PDU_SET_ADDR_FAIL_VALUE                   1

/**
* @brief 解析adu
* @param adu 数据缓存指针
* @param size 数据大小
* @param rsp 回应的数据缓存指针
* @param update 回应的是否需要升级
* @return -1 失败 
* @return  > 0 回应的adu大小
* @note
*/
static int parse_adu(uint8_t *adu,uint8_t size,uint8_t *addr,uint8_t *rsp)
{
    int rc;
    uint8_t *pdu;
    uint8_t pdu_size;
    uint8_t pdu_value_size;
    uint8_t code;

    uint8_t addr_recv;
    uint8_t addr_setting;
    uint8_t sensor_id;
    uint32_t fw_verion;
    int16_t calibration_weight;
    uint16_t crc_received,crc_calculated;

    int16_t net_weight;
    uint8_t rsp_size = 0;
    uint8_t rsp_offset = 0;

    if (size < ADU_SIZE_MIN) {
        log_error("adu size:%d < %d err.\r\n",size,ADU_SIZE_MIN);
        return -1;
    }

    /*校验数据头*/
    if (adu[ADU_HEAD0_OFFSET] != ADU_HEAD0_VALUE || \
        adu[ADU_HEAD1_OFFSET] != ADU_HEAD1_VALUE) {
        log_error("recv head0:%d != %d or head1:%d != %d err.\r\n",adu[ADU_HEAD0_OFFSET],ADU_HEAD0_VALUE,adu[ADU_HEAD1_OFFSET],ADU_HEAD1_VALUE);
        return -1;
    }

    /*校验数据长度*/
    pdu_size = adu[ADU_PDU_SIZE_REGION_OFFSET];
    if (pdu_size + ADU_PDU_REGION_OFFSET + ADU_CRC_SIZE != size) {
        log_error("数据域长度%d != %d err.\r\n",pdu_size,size - ADU_CRC_SIZE - ADU_PDU_REGION_OFFSET);
        return -1;
    }

    /*校验CRC*/
    crc_received = (uint16_t)adu[size - 1] << 8 | adu[size - 2];
    crc_calculated = calculate_crc16(adu,size - ADU_CRC_SIZE);
    if (crc_received != crc_calculated) {
        log_error("crc err.claculate:%d receive:%d.\r\n",crc_calculated,crc_received);
        return -1;
    }

    /*解析pdu*/
    pdu = &adu[ADU_PDU_REGION_OFFSET];
    /*校验通信地址*/
    addr_recv = pdu[PDU_ADDR_OFFSET];
    if (addr_recv != *addr) {
        log_error("recv addr:%d != %d err.\r\n",addr_recv,*addr);
        return -1;
    }

    /*校验命令码*/
    code = pdu[PDU_CODE_OFFSET];
    /*回应adu构建*/
    rsp[rsp_offset ++] = ADU_HEAD0_VALUE;
    rsp[rsp_offset ++] = ADU_HEAD1_VALUE;
    rsp[rsp_offset ++] = 0;//数据域长度预留
    rsp[rsp_offset ++] = addr_recv;
    rsp[rsp_offset ++] = code;
    pdu_value_size = pdu_size - PDU_ADDR_SIZE - PDU_CODE_SIZE;

    switch (code) {
        case PDU_CODE_NET_WEIGHT:/*净重*/
            if (pdu_value_size != PDU_NET_WEIGHT_VALUE_SIZE) {
                log_error("net weight value size:%d != %d err.\r\n",pdu_value_size,PDU_NET_WEIGHT_VALUE_SIZE);
                return -1;
            }
            rc = request_net_weight(&net_weight);
            if (rc != 0) {
                return -1;
            }
            rsp[rsp_offset ++] = net_weight & 0xFF;
            rsp[rsp_offset ++] = (net_weight >> 8) & 0xFF;
            break;

        case PDU_CODE_REMOVE_TARE_WEIGHT:/*去皮*/
            if (pdu_value_size != PDU_REMOVE_TARE_VALUE_SIZE) {
                log_error("remove tare value size:%d != %d err.\r\n",pdu_value_size,PDU_REMOVE_TARE_VALUE_SIZE);
                return -1;
            }
            log_debug("remove tare weight...\r\n");
            rc = request_remove_tare_weight();
            if (rc == 0) {
                rsp[rsp_offset ++] = PDU_REMOVE_TARE_SUCCESS_VALUE;
            } else {
                rsp[rsp_offset ++] = PDU_REMOVE_TARE_FAIL_VALUE;
            }
            break;

        case PDU_CODE_CALIBRATION_ZERO:/*0点校准*/
            if (pdu_value_size != PDU_CALIBRATION_ZERO_VALUE_SIZE) {
                log_error("calibration zero value size:%d != %d err.\r\n",pdu_value_size,PDU_CALIBRATION_ZERO_VALUE_SIZE);
                return -1;
            }
            calibration_weight = (uint16_t)pdu[PDU_VALUE_OFFSET + 1] << 8 | pdu[PDU_VALUE_OFFSET + 0];
            if (calibration_weight != 0) {
                log_error("calibration zero weight:%d != 0.err.\r\n",calibration_weight);
                return -1;
            }
            log_debug("calibration zero weight:%d...\r\n",calibration_weight);
            rc = request_calibrate_zero_weight(calibration_weight);
            if (rc == 0) {
                rsp[rsp_offset ++] = PDU_CALIBRATION_ZERO_SUCCESS_VALUE;
            } else {
                rsp[rsp_offset ++] = PDU_CALIBRATION_ZERO_FAIL_VALUE;
            }
            break;

        case PDU_CODE_CALIBRATION_FULL:/*增益校准*/
            if (pdu_value_size != PDU_CALIBRATION_FULL_VALUE_SIZE) {
                log_error("calibration full value size:%d != %d err.\r\n",pdu_value_size,PDU_CALIBRATION_FULL_VALUE_SIZE);
                return -1;
            }
            calibration_weight = (uint16_t)pdu[PDU_VALUE_OFFSET + 1] << 8 | pdu[PDU_VALUE_OFFSET + 0];
            if (calibration_weight <= 0) {
                log_error("calibration full weight:%d <= 0.err.\r\n",calibration_weight);
                return -1;
            }
            log_debug("calibration full weight:%d...\r\n",calibration_weight);
            rc = request_calibrate_full_weight(calibration_weight);
            if (rc == 0) {
                rsp[rsp_offset ++] = PDU_CALIBRATION_FULL_SUCCESS_VALUE;
            } else {
                rsp[rsp_offset ++] = PDU_CALIBRATION_FULL_FAIL_VALUE;
            }
            break;

        case PDU_CODE_SET_ADDR:/*设置地址*/
            if (pdu_value_size != PDU_SET_ADDR_VALUE_SIZE) {
                log_error("set addr value size:%d != %d err.\r\n",pdu_value_size,PDU_SET_ADDR_VALUE_SIZE);
                return -1;
            }
            addr_setting = pdu[PDU_VALUE_OFFSET];
            rc = request_set_scale_addr(addr_setting);
            if (rc == 0) {
                *addr = addr_setting;/*更新地址*/
                rsp[rsp_offset ++] = PDU_SET_ADDR_SUCCESS_VALUE;
            } else {
                rsp[rsp_offset ++] = PDU_SET_ADDR_FAIL_VALUE;
            } 
            break;

        case PDU_CODE_SENSOR_ID:/*传感器代码*/
            if (pdu_value_size != PDU_SENSOR_ID_VALUE_SIZE) {
                log_error("sensor id value size:%d != %d err.\r\n",pdu_value_size,PDU_SENSOR_ID_VALUE_SIZE);
                return -1;
            }
            rc = request_sensor_id(&sensor_id);
            if (rc != 0) {
                return -1;
            }
            rsp[rsp_offset ++] = sensor_id;
            break;

        case PDU_CODE_FIRMWARE_VERSION:/*固件版本*/
            if (pdu_value_size != PDU_FW_VERSION_VALUE_SIZE) {
                log_error("net weight value size:%d != %d err.\r\n",pdu_value_size,PDU_FW_VERSION_VALUE_SIZE);
                return -1;
            }
            rc = request_firmware_version(&fw_verion);
            if (rc != 0) {
                return -1;
            }
            rsp[rsp_offset ++] = fw_verion & 0xFF;
            rsp[rsp_offset ++] = (fw_verion >> 8) & 0xFF;
            break;

        default:
            log_error("invalid code:%d.\r\n",code);
    }

    /*添加CRC16*/
    rsp[ADU_PDU_SIZE_REGION_OFFSET] = rsp_offset - ADU_HEAD_SIZE - ADU_PDU_SIZE_REGION_SIZE;
    rsp_size = adu_add_crc16(rsp,rsp_offset);
    return rsp_size;
}

/*
* @brief 通过串口回应处理结果
* @param handle 串口句柄
* @param adu 结果缓存指针
* @param handle 串口句柄
* @param adu 结果缓存指针
* @param size 结果大小
* @param timeout 发送超时时间
* @return -1 失败 
* @return  0 成功 
* @note
*/
static int send_adu(xuart_handle_t *handle,uint8_t *adu,uint8_t size,uint32_t timeout)
{
    uint32_t write_size;
    char buffer[ADU_SIZE_MAX * 2 + 1];

    write_size = xuart_write(handle,adu,size);

    if (size != write_size){
        log_error("err in write. expect:%d write:%d.\r\n",size,write_size); 
        return -1;      
     }
    if (xuart_complete(handle,timeout) != 0) {
        log_error("err in send.\r\n"); 
        return -1;  
    }

    xstring_hex_to_string(buffer,(char *)adu,size);
    log_debug("adu send[%s]\r\n",buffer);

    return 0;
}

/*串口中断处理*/
void USART1_IRQHandler()
{
    if (protocol_uart_handle.is_port_open) {
        nxp_uart_hal_isr(&protocol_uart_handle);
    }
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

void protocol_task(void const * argument)
{
    int rc; 
    int adu_recv_size;
    int adu_send_size;
    static uint8_t scale_addr;
    uint8_t adu_recv[ADU_SIZE_MAX];
    uint8_t adu_send[ADU_SIZE_MAX];

    xuart_register_hal_driver(&xuart_hal_driver);
    rc = xuart_open(&protocol_uart_handle,PROTOCOL_TASK_UART_PORT,PROTOCOL_TASK_UART_BAUD_RATES,PROTOCOL_TASK_UART_DATA_BITS,PROTOCOL_TASK_UART_STOP_BITS,
                     recv_buffer,PROTOCOL_TASK_RX_BUFFER_SIZE,send_buffer,PROTOCOL_TASK_TX_BUFFER_SIZE);
    log_assert_bool_false(rc == 0);
    log_debug("protocol task ok.\r\n");

    rc = request_scale_addr(&scale_addr);
    log_assert_bool_false(rc == 0);

    /*清空接收缓存*/
    xuart_clear(&protocol_uart_handle);
    while (1) {

        /*接收主机发送的adu*/
        adu_recv_size = receive_adu(&protocol_uart_handle,adu_recv,PROTOCOL_TASK_ADU_WAIT_TIMEOUT);
        if (adu_recv_size < 0) {
            xuart_clear(&protocol_uart_handle);
            continue;
        }
        /*解析处理pdu*/
        adu_send_size = parse_adu(adu_recv,adu_recv_size,&scale_addr,adu_send);
        if (adu_send_size < 0) {
            continue;
        }
        /*回应主机处理结果*/
        send_adu(&protocol_uart_handle,adu_send,adu_send_size,PROTOCOL_TASK_ADU_SEND_TIMEOUT);
    }    
}
