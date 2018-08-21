#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "modbus.h"
#include "serial.h"

MODBUS_BEGIN_DECLS


MODBUS_API modbus_t* modbus_new_rtu(uint8_t port, uint32_t baud, 
                                    uint8_t data_bit, uint8_t stop_bit,serial_hal_driver_t *hal);

#define MODBUS_RTU_RS232                        0
#define MODBUS_RTU_RS485                        1

#define MODBUS_RTU_RS485_HALF_DUPLEX            1
#define MODBUS_RTU_SEND_TIMEOUT                 5

#if MODBUS_RTU_RS485_HALF_DUPLEX > 0
void modbus_rtu_send_pre();
void modbus_rtu_send_after();
#endif

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5
 * RS232 / RS485 ADU = 253 bytes + slave (1 byte) + CRC (2 bytes) = 256 bytes
 */
#define MODBUS_RTU_MAX_ADU_LENGTH               48
#define MODBUS_RTU_RX_BUFFER_SIZE               64
#define MODBUS_RTU_TX_BUFFER_SIZE               64

MODBUS_API int modbus_rtu_set_serial_mode(modbus_t *ctx, int mode);
MODBUS_API int modbus_rtu_get_serial_mode(modbus_t *ctx);



MODBUS_END_DECLS

#endif /* MODBUS_RTU_H */
