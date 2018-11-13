#include "board.h"
#include "nv.h"
#include "cmsis_os.h"
#include "log.h"
#define LOG_MODULE_NAME   "[nv]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 

I2C_Type *nv_i2c =NV_DEVICE_I2C;

int nv_init()
{

 i2c_master_config_t masterConfig;
 I2C_MasterGetDefaultConfig(&masterConfig);
 masterConfig.baudRate_Bps = NV_DEVICE_BAUDRATE;

 I2C_MasterInit(nv_i2c, &masterConfig,CLOCK_GetFreq(kCLOCK_CoreSysClk));
 
 return 0;
}


int nv_read(uint8_t addr,uint8_t *buffer,uint16_t size)
{
status_t status;
uint8_t page_cnt;
uint8_t byte_remain;


i2c_master_transfer_t transfer;

page_cnt = size / NV_DEVICE_PAGE_SIZE;
byte_remain = size % NV_DEVICE_PAGE_SIZE;

transfer.subaddress =addr;
transfer.data = buffer;

transfer.flags = kI2C_TransferDefaultFlag;
transfer.direction = kI2C_Read;
transfer.slaveAddress = NV_DEVICE_ADDR ;
transfer.subaddressSize = NV_DEVICE_SUB_ADDR_SIZE;

for(uint8_t cnt = 0;cnt < page_cnt;cnt++){
transfer.dataSize =NV_DEVICE_PAGE_SIZE;
log_debug("nv read.addr:%d, cnt:%d.\r\n",transfer.subaddress,transfer.dataSize);
/*启动页读*/
status = I2C_MasterTransferBlocking(nv_i2c,&transfer);
if(status != kStatus_Success){
return -1;
}
transfer.subaddress +=NV_DEVICE_PAGE_SIZE;
transfer.data = (cnt+1) * NV_DEVICE_PAGE_SIZE + buffer;
}

if(byte_remain > 0){
transfer.dataSize =byte_remain;
log_debug("nv read.addr:%d, cnt:%d.\r\n",transfer.subaddress,transfer.dataSize);
/*启动页读*/
status = I2C_MasterTransferBlocking(nv_i2c,&transfer);
if(status != kStatus_Success){
return -1;
}
}
  
return 0;  
}


int nv_save(uint8_t addr,uint8_t *buffer,uint16_t size)
{
status_t status;
int result;
uint8_t page_cnt;
uint8_t byte_remain;

i2c_master_transfer_t transfer;
uint8_t  read_buffer[0x40];


page_cnt = size / NV_DEVICE_PAGE_SIZE;
byte_remain = size % NV_DEVICE_PAGE_SIZE;

transfer.subaddress = addr;
transfer.data = buffer;

transfer.flags = kI2C_TransferDefaultFlag;
transfer.direction = kI2C_Write;
transfer.slaveAddress = NV_DEVICE_ADDR;
transfer.subaddressSize = NV_DEVICE_SUB_ADDR_SIZE;

for(uint8_t cnt = 0;cnt < page_cnt;cnt++){
transfer.dataSize =NV_DEVICE_PAGE_SIZE;
log_debug("nv program.addr:%d, cnt:%d.\r\n",transfer.subaddress,transfer.dataSize);
/*启动页编程*/
status = I2C_MasterTransferBlocking(nv_i2c,&transfer);
if(status != kStatus_Success){
return -1;
}

transfer.subaddress +=NV_DEVICE_PAGE_SIZE;
transfer.data = (cnt+1) * NV_DEVICE_PAGE_SIZE + buffer;
osDelay(5);
}

if(byte_remain > 0){
transfer.dataSize =byte_remain;
log_debug("nv program.addr:%d, cnt:%d.\r\n",transfer.subaddress,transfer.dataSize);
/*启动页编程*/
status = I2C_MasterTransferBlocking(nv_i2c,&transfer);
if(status != kStatus_Success){
return -1;
}
osDelay(5);
}

/*读回校验是否保存成功*/
/*
result = nv_read(addr,read_buffer,size);
if(result !=0 ){
return -1;
}


for(uint16_t i=0;i<size;i++){
  if(read_buffer[i] != buffer[i]){
    return -1;
  }
}
*/
return 0;
}

