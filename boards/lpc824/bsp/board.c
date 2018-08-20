/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "log.h"
#define LOG_MODULE_NAME   "[board_task]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_DEBUG 
/*******************************************************************************
 * Variables
 ******************************************************************************/
spi_master_handle_t ad7190_spi_handle;
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
}
;
/*******************************************************************************
 * Code
 ******************************************************************************/
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
log_assert(status == kStatus_Success);
status = SPI_MasterTransferCreateHandle(ad7190_spi,&ad7190_spi_handle,NULL,NULL);
log_assert(status == kStatus_Success);
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








