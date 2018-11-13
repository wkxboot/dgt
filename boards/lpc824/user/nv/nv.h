#ifndef  __NV_H__
#define  __NV_H__


#define  NV_DEVICE_I2C                       I2C1
#define  NV_DEVICE_ADDR                      0x50
#define  NV_DEVICE_BAUDRATE                  100000
#define  NV_DEVICE_PAGE_SIZE                 8

/******************************************************************
02 04 08 16型号都可以按照02容量 8byte/page来操作 sub_addr_size == 1
32 64型号按照真实容量 8byte/page来操作 sub_addr_size == 2
*****************************************************************/
#define  NV_DEVICE_SUB_ADDR_SIZE             2

int nv_init();
int nv_read(uint8_t addr,uint8_t *buffer,uint16_t size);
int nv_save(uint8_t addr,uint8_t *buffer,uint16_t size);








#endif