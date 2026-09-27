

#ifndef __APP_TWI_H__
#define __APP_TWI_H__
#include "stdint.h"
uint32_t I2C_Transmitreceive(uint8_t* data,uint16_t len,uint8_t* rxdata, uint16_t rx_len);
uint32_t I2C_Transmit(uint8_t* data,uint16_t len);
void twi_init(void);
void read_accel(int16_t *ax, int16_t *ay, int16_t *az);
void twi_deinit(void);
#endif
