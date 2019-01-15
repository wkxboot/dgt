#include "board.h"
#include "cmsis_os.h"
#include "task_msg.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "nxp_serial_uart_hal_driver.h"
#include "log.h"


int protocol_serial_handle;


osThreadId protocol_task_hdl;
osMessageQId protocol_task_msg_q_id;

typedef enum
{
 PROTOCOL_TASK_ADU_STEP=0,
 PROTOCOL_TASK_CRC_STEP
}protocol_step_t;

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static uint16_t protocol_task_crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    uint32_t i; /* will index into CRC lookup */

   /* calculate the CRC  */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; 
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}
/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static uint16_t protocol_task_prepare_crc16(uint8_t *send_buffer,uint16_t length_to_write)
{
    uint16_t crc_calculated;
    crc_calculated = protocol_task_crc16(send_buffer,length_to_write);

    send_buffer[length_to_write++] = crc_calculated & 0xff;
    send_buffer[length_to_write++] = crc_calculated >> 8;

    return length_to_write;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_get_net_weight(int16_t *net_weight)
{
    int rc = -1;
    osStatus status;
    osEvent  os_msg;
    task_message_t msg,scale_msg;
 
    scale_msg.type = TASK_MSG_REQ_NET_WEIGHT;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);

    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
       msg =  *(task_message_t *)&os_msg.value.v;
       if (msg.type == TASK_MSG_RSP_NET_WEIGHT){
          rc = 0;
          *net_weight = (int16_t)msg.value;        
        }
    }

    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_remove_tar_weight()
{
    int rc = -1;
    osStatus status;
    osEvent  os_msg;
    task_message_t msg,scale_msg;
 
    scale_msg.type = TASK_MSG_REQ_REMOVE_TAR_WEIGHT;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);

    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        msg = *(task_message_t *)&os_msg.value.v;
        if (msg.type == TASK_MSG_RSP_REMOVE_TAR_WEIGHT && msg.value == SCALE_TASK_SUCCESS){
           rc = 0;
        }    
    }

    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_calibrate_weight(int16_t weight)
{
    int rc = -1;
    osStatus status;
    osEvent  os_msg;
    task_message_t msg,scale_msg;
 
    if (weight == 0){
        scale_msg.type = TASK_MSG_REQ_CALIBRATE_ZERO;
    }else{
        scale_msg.type = TASK_MSG_REQ_CALIBRATE_FULL; 
    } 
    scale_msg.value = weight;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);

    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        msg = *(task_message_t *)&os_msg.value.v;
        if((msg.type == TASK_MSG_RSP_CALIBRATE_ZERO || msg.type == TASK_MSG_RSP_CALIBRATE_FULL) && msg.value == SCALE_TASK_SUCCESS){
            rc = 0;
        }
    }

    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_get_sensor_id(uint8_t *sensor_id)
{
    int rc = -1;
    osStatus status;
    osEvent  os_msg;
    task_message_t msg,scale_msg;

    scale_msg.type = TASK_MSG_REQ_SENSOR_ID;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);

    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        msg =  *(task_message_t *)&os_msg.value.v;
        if (msg.type == TASK_MSG_RSP_SENSOR_ID){
            rc = 0;
            *sensor_id = (uint8_t)msg.value;
        }  
    }

    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_get_fireware_version(uint16_t *fw_version)
{
    int rc = -1;
    osStatus status;
    osEvent  os_msg;
    task_message_t msg,scale_msg;

    scale_msg.type = TASK_MSG_REQ_FW_VERSION;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);

    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        msg =  *(task_message_t *)&os_msg.value.v;
        if (msg.type == TASK_MSG_RSP_FW_VERSION){
            rc = 0;
            *fw_version = (uint16_t)msg.value;
        }  
    }

    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_get_scale_addr(uint8_t *addr)
{
    int rc = -1;
    osStatus status;
    osEvent  os_msg;
    task_message_t msg,scale_msg;

    scale_msg.type = TASK_MSG_REQ_ADDR;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);

    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        msg =  *(task_message_t *)&os_msg.value.v;
        if (msg.type == TASK_MSG_RSP_ADDR){
            rc = 0;
            *addr = (uint8_t)msg.value;
        }  
    }

    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

int protocol_set_scale_addr(uint8_t addr)
{
    int rc = -1;
    osStatus status;
    osEvent  os_msg;
    task_message_t msg,scale_msg;

    scale_msg.type = TASK_MSG_REQ_SET_ADDR;
    scale_msg.value = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t*)&scale_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    log_assert(status == osOK);

    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        msg =  *(task_message_t *)&os_msg.value.v;
        if (msg.type == TASK_MSG_RSP_SET_ADDR && msg.value == SCALE_TASK_SUCCESS){
            rc = 0;
        }  
    }

    return rc;
}

/*串口中断处理*/
void USART1_IRQHandler()
{
  nxp_serial_uart_hal_isr(protocol_serial_handle);
}



void protocol_task(void const * argument)
{
    int rc; 
    static uint8_t scale_addr;

    int length_to_read,read_length=0;
    int length_to_write,write_length,remain_length;
    uint8_t  result;
    uint32_t timeout;
    uint16_t crc_calculated;
    uint16_t crc_received;
    int16_t  calibrate_weight;
    int16_t  net_weight;
    uint16_t version;
    uint8_t  sensor_id;
    uint8_t  scale_set_addr;
    protocol_step_t step;
 
    char recv_buffer[PROTOCOL_TASK_FRAME_SIZE_MAX];
    char send_buffer[PROTOCOL_TASK_FRAME_SIZE_MAX];
 
    osMessageQDef(protocol_task_msg_q,6,uint32_t);
    protocol_task_msg_q_id = osMessageCreate(osMessageQ(protocol_task_msg_q),protocol_task_hdl);
    log_assert(protocol_task_msg_q_id); 
 
    rc = serial_create(&protocol_serial_handle,PROTOCOL_TASK_RX_BUFFER_SIZE,PROTOCOL_TASK_TX_BUFFER_SIZE);
    log_assert(rc == 0);
    rc = serial_register_hal_driver(protocol_serial_handle,&nxp_serial_uart_hal_driver);
    log_assert(rc == 0);
 
    rc = serial_open(protocol_serial_handle,
                     PROTOCOL_TASK_SERIAL_PORT,
                     PROTOCOL_TASK_SERIAL_BAUDRATES,
                     PROTOCOL_TASK_SERIAL_DATABITS,
                     PROTOCOL_TASK_SERIAL_STOPBITS);
 
    log_assert(rc == 0); 
 
    /*等待scale_task启动完毕*/
    osDelay(PROTOCOL_TASK_START_DELAY_TIME_VALUE);
 
    /*读取上电后当前地址值*/
    rc = protocol_get_scale_addr(&scale_addr);
    log_assert(rc == 0);

    serial_flush(protocol_serial_handle);
 
    while (1) {

    /*使能485接收*/
    bsp_485_enable_read();
    timeout = PROTOCOL_TASK_FRAME_TIMEOUT_VALUE;
    length_to_read = 4;
    read_length =0;
    length_to_write = 0;
    step = PROTOCOL_TASK_ADU_STEP;

    while (length_to_read != 0) {
        rc = serial_select(protocol_serial_handle,timeout);
        if (rc == -1){
            log_error("protocol select error.\r\n");
            goto err_exit;
        }
        if (rc == 0){
            log_error("protocol select timeout.\r\n");
            goto err_exit;
        }
  
        rc = serial_read(protocol_serial_handle,recv_buffer + read_length,length_to_read);
        if (rc == -1) {
            log_error("protocol read error.\r\n");
            goto err_exit;
        }

        for (int i=0; i < rc; i++){
            log_array("<%2X>\r\n", recv_buffer[read_length + i]);
        }
   
        read_length +=rc;
        length_to_read -=rc;
   
     if(length_to_read == 0){
     switch(step){
     /*接收到了协议头和数据长度域*/
     case PROTOCOL_TASK_ADU_STEP:
     if(recv_buffer[PROTOCOL_TASK_HEADER0_OFFSET] == PROTOCOL_TASK_HEADER0_VALUE &&
        recv_buffer[PROTOCOL_TASK_HEADER1_OFFSET] == PROTOCOL_TASK_HEADER1_VALUE){
         
        length_to_read =*(int16_t*)&recv_buffer[PROTOCOL_TASK_SIZE_OFFSET];
        if(length_to_read < PROTOCOL_TASK_ADU_SIZE_MIN + PROTOCOL_TASK_CRC_SIZE || length_to_read > PROTOCOL_TASK_ADU_SIZE_MAX + PROTOCOL_TASK_CRC_SIZE){
        log_error("protocol err in adu size.\r\n");
        goto err_exit;
        }
        step = PROTOCOL_TASK_CRC_STEP;      
        send_buffer[PROTOCOL_TASK_HEADER0_OFFSET]=PROTOCOL_TASK_HEADER0_VALUE;
        send_buffer[PROTOCOL_TASK_HEADER1_OFFSET]=PROTOCOL_TASK_HEADER1_VALUE;
        length_to_write+=PROTOCOL_TASK_HEADER_SIZE;
        }else{
        log_error("protocol err in header value.%d %d\r\n",recv_buffer[PROTOCOL_TASK_HEADER0_OFFSET],recv_buffer[PROTOCOL_TASK_HEADER1_OFFSET]);
        goto err_exit;
        }
     break;
     /*接收完成了全部的数据*/
     case PROTOCOL_TASK_CRC_STEP:
       crc_calculated = protocol_task_crc16((uint8_t *)recv_buffer,read_length - PROTOCOL_TASK_CRC_SIZE);
       crc_received = recv_buffer[read_length-1]<< 8 | recv_buffer[read_length-2];
       if(crc_calculated != crc_received){
          log_error("protocol err in crc.recv:%d calculate:%d.\r\n",crc_received,crc_calculated);
          goto err_exit;
       }
       
       if(recv_buffer[PROTOCOL_TASK_ADU_ADDR_OFFSET] != scale_addr ){
        log_error("protocol err in addr.recv:%d legacy:%d.\r\n",recv_buffer[PROTOCOL_TASK_ADU_ADDR_OFFSET],scale_addr);
        goto err_exit;  
       }

       /*如果是读取净重值*/
       if(recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET] == PROTOCOL_TASK_FUNC_READ_NET_WEIGHT && \
          read_length == PROTOCOL_TASK_READ_NET_WEIGHT_FRAME_LEN){
          send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_NET_WEIGHT_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE));
          send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_NET_WEIGHT_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE)) >> 8;
          send_buffer[length_to_write++] = scale_addr;  
          send_buffer[length_to_write++] = PROTOCOL_TASK_FUNC_READ_NET_WEIGHT;
          
          rc = protocol_get_net_weight(&net_weight);
          log_assert(rc == 0);
          if (net_weight == SCALE_TASK_WEIGHT_ERR_VALUE){
            net_weight = PROTOCOL_TASK_WEIGHT_ERR_VALUE;
          }

          send_buffer[length_to_write++] = net_weight & 0xff;
          send_buffer[length_to_write++] = net_weight >> 8;        
          length_to_write = protocol_task_prepare_crc16((uint8_t *)send_buffer,length_to_write);         
          break;
       }
       
       /*如果是去皮*/
       if(recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET] == PROTOCOL_TASK_FUNC_REMOVE_TAR_WEIGHT && \
          read_length == PROTOCOL_TASK_REMOVE_TAR_WEIGHT_FRAME_LEN){
         send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_REMOVE_TAR_WEIGHT_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE));
         send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_REMOVE_TAR_WEIGHT_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE)) >> 8; 
         send_buffer[length_to_write++] = scale_addr;
         send_buffer[length_to_write++]=PROTOCOL_TASK_FUNC_REMOVE_TAR_WEIGHT;
         
         /*执行去皮*/
         rc = protocol_remove_tar_weight();
         if(rc == 0){
          result = PROTOCOL_TASK_SUCCESS_VALUE;                
         }else{
         result = PROTOCOL_TASK_FAILURE_VALUE; 
         }
         /*填充操作结果值*/
         send_buffer[length_to_write++] = result;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16((uint8_t *)send_buffer,length_to_write); 
         break;
       }
       
        /*如果是0点校准*/
       if(recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET] == PROTOCOL_TASK_FUNC_CALIBRATE_ZERO &&
          read_length == PROTOCOL_TASK_CALIBRATE_ZERO_FRAME_LEN){
         send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_CALIBRATE_ZERO_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE));
         send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_CALIBRATE_ZERO_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE)) >> 8; 
         send_buffer[length_to_write++] = scale_addr;  
         send_buffer[length_to_write++]=PROTOCOL_TASK_FUNC_CALIBRATE_ZERO;
         
         calibrate_weight = *(int16_t *)&recv_buffer[PROTOCOL_TASK_ADU_PAYLOAD_OFFSET];     
         if(calibrate_weight != 0){
           log_error("protocol err in calibrate zero weight.weight:%d.\r\n",calibrate_weight); 
           goto err_exit;;
         }

         /*执行0点量程校准*/
         rc = protocol_calibrate_weight(calibrate_weight);
         if(rc == 0){
          result = PROTOCOL_TASK_SUCCESS_VALUE;                
         }else{
         result = PROTOCOL_TASK_FAILURE_VALUE; 
         }
         /*填充操作结果值*/
         send_buffer[length_to_write++] = result;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16((uint8_t *)send_buffer,length_to_write); 
         break;
         }                
     
       
       /*如果是full量程校准*/
       if(recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET] == PROTOCOL_TASK_FUNC_CALIBRATE_FULL &&
          read_length == PROTOCOL_TASK_CALIBRATE_FULL_FRAME_LEN){
         send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_CALIBRATE_FULL_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE));
         send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_CALIBRATE_FULL_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE)) >> 8; 
         send_buffer[length_to_write++] = scale_addr;  
         send_buffer[length_to_write++]=PROTOCOL_TASK_FUNC_CALIBRATE_FULL;  
         
         calibrate_weight = *(int16_t *)&recv_buffer[PROTOCOL_TASK_ADU_PAYLOAD_OFFSET];;
         if(calibrate_weight <= 0){
         log_error("protocol err in calibrate full weight.weight:%d.\r\n",calibrate_weight); 
         goto err_exit; ;
         }

         /*执行满量程校准*/
         rc = protocol_calibrate_weight(calibrate_weight);
         if(rc == 0){
          result = PROTOCOL_TASK_SUCCESS_VALUE;                
         }else{
          result = PROTOCOL_TASK_FAILURE_VALUE; 
         }
         /*填充操作结果值*/
         send_buffer[length_to_write++] = result;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16((uint8_t *)send_buffer,length_to_write); 
         break;              
       }
       
       /*如果是读取传感器厂家ID*/
       if(recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET] == PROTOCOL_TASK_FUNC_READ_SENSOR_ID && \
          read_length == PROTOCOL_TASK_READ_SENSOR_ID_FRAME_LEN){
            
          send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_READ_SENSOR_ID_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE));
          send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_READ_SENSOR_ID_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE)) >> 8; 
          send_buffer[length_to_write++] = scale_addr;           
          send_buffer[length_to_write++] = PROTOCOL_TASK_FUNC_READ_SENSOR_ID;
          
          rc = protocol_get_sensor_id(&sensor_id);
          log_assert(rc == 0);
          send_buffer[length_to_write++] = sensor_id;
          /*填充CRC16值*/
          length_to_write = protocol_task_prepare_crc16((uint8_t *)send_buffer,length_to_write); 
          break;               
       }
      
      /*如果是读取固件版本号*/
       if(recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET] == PROTOCOL_TASK_FUNC_READ_VERSION && \
          read_length == PROTOCOL_TASK_READ_VERSION_FRAME_LEN){
            
          send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_READ_VERSION_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE));
          send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_READ_VERSION_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE)) >> 8; 
          send_buffer[length_to_write++] = scale_addr;  
          send_buffer[length_to_write++] = PROTOCOL_TASK_FUNC_READ_VERSION;
          
          rc = protocol_get_fireware_version(&version); 
          log_assert(rc == 0);        
          send_buffer[length_to_write++] = version & 0xff;
          send_buffer[length_to_write++] = version >> 8;
          length_to_write = protocol_task_prepare_crc16((uint8_t *)send_buffer,length_to_write); 
          break;  
      }
      
       /*如果是设置地址值*/
       if(recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET] == PROTOCOL_TASK_FUNC_SET_ADDR && \
          read_length == PROTOCOL_TASK_SET_ADDR_FRAME_LEN){
            
          send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_SET_ADDR_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE));
          send_buffer[length_to_write++] = (PROTOCOL_TASK_RESPONSE_SET_ADDR_FRAME_LEN -(PROTOCOL_TASK_HEADER_SIZE + PROTOCOL_TASK_SIZE_SIZE)) >> 8; 
          send_buffer[length_to_write++] = scale_addr;   
          send_buffer[length_to_write++] = PROTOCOL_TASK_FUNC_SET_ADDR;
          
          scale_set_addr = recv_buffer[PROTOCOL_TASK_ADU_PAYLOAD_OFFSET];
          rc = protocol_set_scale_addr(scale_set_addr);

          if(rc == 0){
            result = PROTOCOL_TASK_SUCCESS_VALUE;  
            /*暂存新地址值*/
            scale_addr = scale_set_addr;
         }else{
            result = PROTOCOL_TASK_FAILURE_VALUE; 
         }
         /*填充操作结果值*/
         send_buffer[length_to_write++] = result;
         /*填充CRC16值*/
         length_to_write = protocol_task_prepare_crc16((uint8_t *)send_buffer,length_to_write); 
         break;  
      }
     
     default :
     log_error("protocol err in  func:%d.\r\n",recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET]); 
     goto err_exit;
     break;
     }
   }
   
   if (length_to_read != 0){   
       timeout = PROTOCOL_TASK_CHARACTER_TIMEOUT_VALUE; 
   }       
  }  
 
    /*解析完毕 回应操作结果*/  
    /*使能485发送*/
    bsp_485_enable_write();
    write_length = serial_write(protocol_serial_handle,send_buffer,length_to_write);
    for (int i=0; i < write_length; i++){
        log_array("[%2X]\r\n", send_buffer[i]);
    }
    if (write_length != length_to_write){
        log_error("protocol err in  serial buffer write. expect:%d write:%d.\r\n",length_to_write,write_length); 
        goto err_exit;      
    }
    
    remain_length = serial_complete(protocol_serial_handle,PROTOCOL_TASK_SEND_TIMEOUT);
    if (remain_length != 0){
        log_error("protocol err in  serial send timeout.\r\n",); 
        goto err_exit;  
    }
    osDelay(1);
    continue;

err_exit:
    serial_flush(protocol_serial_handle);
    log_error("func %d execute err.\r\n",recv_buffer[PROTOCOL_TASK_ADU_FUNC_OFFSET]);    
   }
}
