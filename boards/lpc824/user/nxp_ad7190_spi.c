#include "board.h"

SPI_Type *ad7190_spi=NULL;


void bsp_ad7190_spi_write_byte(uint8_t byte);
uint8_t bsp_ad7190_spi_read_byte();
void bsp_ad7190_cs_set();
void bsp_ad7190_cs_clr();

ad7190_io_driver_t ad7190_driver={
.cs_set = bsp_ad7190_cs_set,
.cs_clr = bsp_ad7190_cs_clr,
.write_byte = bsp_ad7190_spi_write_byte,
.read_byte = bsp_ad7190_spi_read_byte
};



int bsp_ad7190_spi_int(uint8_t spi_port,uint32_t freq)
{
spi_master_config_t config;
status_t status;

if(spi_port == 1){
ad7190_spi = SPI1;
}else{
ad7190_spi = SPI0;
}

SPI_MasterGetDefaultConfig(&config);
config.baudRate_Bps = freq;
status = SPI_MasterInit(ad7190_spi,&config,CLOCK_GetFreq(kCLOCK_CoreSysClk));
if(status != kStatus_Success){
return -1;
}
SPI_SetDummyData(ad7190_spi,0xffff);

return 0;
}




void bsp_ad7190_cs_set()
{
GPIO_PortSet(BSP_AD7190_CS_GPIO,BSP_AD7190_CS_PORT,BSP_AD7190_CS_PIN);  
}

void bsp_ad7190_cs_clr()
{
GPIO_PortClear(BSP_AD7190_CS_GPIO,BSP_AD7190_CS_PORT,BSP_AD7190_CS_PIN);   
}



void bsp_ad7190_spi_write_byte(uint8_t byte)
{
spi_transfer_t transfer;
uint8_t tx_data[1];
tx_data[0]=byte;
transfer.rxData = NULL;
transfer.txData = tx_data;
transfer.dataSize = 1;
SPI_MasterTransferBlocking(ad7190_spi,&transfer);
}

uint8_t bsp_ad7190_spi_read_byte()
{
spi_transfer_t receive;
uint8_t rx_data[1];
receive.txData = NULL;
receive.rxData = rx_data;
receive.dataSize = 1;
SPI_MasterTransferBlocking(ad7190_spi,&receive);  

return rx_data[0];
}