#ifndef  __NV_H__
#define  __NV_H__


#define  NV_DEVICE_I2C                       I2C1
#define  NV_DEVICE_ADDR                      0x50
#define  NV_DEVICE_BAUDRATE                  100000
#define  NV_DEVICE_PAGE_SIZE                 8



int nv_init();
int nv_read(uint8_t addr,uint8_t *buffer,uint16_t size);
int nv_save(uint8_t addr,uint8_t *buffer,uint16_t size);








#endif