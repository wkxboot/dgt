#include "board.h"

SPI_Type *ad7190_spi=NULL;


void bsp_ad7190_spi_write_byte(uint8_t byte);
uint8_t bsp_ad7190_spi_read_byte();
void bsp_ad7190_cs_init();
void bsp_ad7190_cs_set();
void bsp_ad7190_cs_clr();
void bsp_ad7190_sync_init();

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
bsp_ad7190_sync_init();
bsp_ad7190_cs_init();


 
if(spi_port == 1){
ad7190_spi = SPI1;
}else{
ad7190_spi = SPI0;
}

SPI_MasterGetDefaultConfig(&config);
config.baudRate_Bps = freq;
config.delayConfig.frameDelay = 0xf;
config.clockPhase = kSPI_ClockPhaseFirstEdge;
status = SPI_MasterInit(ad7190_spi,&config,CLOCK_GetFreq(kCLOCK_MainClk));
if(status != kStatus_Success){
return -1;
}
SPI_SetDummyData(ad7190_spi,0xffff);

return 0;
}


void bsp_ad7190_cs_init()
{
  gpio_pin_config_t config = {
  kGPIO_DigitalOutput, 0,
  };
 GPIO_PortInit(BOARD_INITPINS_AD_SPI_CS_GPIO, BOARD_INITPINS_AD_SPI_CS_PORT);
 GPIO_PinInit(BOARD_INITPINS_AD_SPI_CS_GPIO, BOARD_INITPINS_AD_SPI_CS_PORT, BOARD_INITPINS_AD_SPI_CS_PIN ,&config);
 
}

void bsp_ad7190_sync_init()
{
  gpio_pin_config_t config = {
  kGPIO_DigitalOutput, 1,
  };
 GPIO_PortInit(BOARD_INITPINS_AD_SYNC_CTRL_GPIO, BOARD_INITPINS_AD_SYNC_CTRL_PORT);
 GPIO_PinInit(BOARD_INITPINS_AD_SYNC_CTRL_GPIO, BOARD_INITPINS_AD_SYNC_CTRL_PORT, BOARD_INITPINS_AD_SYNC_CTRL_PIN ,&config);
 
}

void bsp_ad7190_cs_set()
{
GPIO_PortSet(BOARD_INITPINS_AD_SPI_CS_GPIO,BOARD_INITPINS_AD_SPI_CS_PORT,(1<<BOARD_INITPINS_AD_SPI_CS_PIN));  
}

void bsp_ad7190_cs_clr()
{
GPIO_PortClear(BOARD_INITPINS_AD_SPI_CS_GPIO,BOARD_INITPINS_AD_SPI_CS_PORT,(1<<BOARD_INITPINS_AD_SPI_CS_PIN));   
}



void bsp_ad7190_spi_write_byte(uint8_t byte)
{
spi_transfer_t transfer;

uint8_t tx_data[1];
uint8_t rx_data[1];

tx_data[0]=byte;

transfer.txData = tx_data;
transfer.rxData = rx_data;
transfer.dataSize = 1;
transfer.configFlags = kSPI_EndOfTransfer | kSPI_EndOfFrame;
SPI_MasterTransferBlocking(ad7190_spi,&transfer);
}

uint8_t bsp_ad7190_spi_read_byte()
{
spi_transfer_t receive;

uint8_t rx_data[1];
uint8_t tx_data[1];

receive.txData = tx_data;
receive.rxData = rx_data;
receive.dataSize = 1;
receive.configFlags = kSPI_EndOfTransfer | kSPI_EndOfFrame;
SPI_MasterTransferBlocking(ad7190_spi,&receive);  

return rx_data[0];
}