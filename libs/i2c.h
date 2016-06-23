/*  Defines necessary for i2c on STM32F103.

K. Sarkies, 22 June 2016
*/

#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

void i2c_initialise(uint32_t i2c);
void i2c_initiate_transmission(uint32_t i2c, uint8_t address, uint8_t rw);
void i2c_transmit_single_byte (uint32_t i2c, uint8_t data);
void i2c_transmit_multiple_byte (uint32_t i2c, uint8_t *data, uint8_t length);
uint8_t i2c_read_single_byte(uint32_t i2c);
uint16_t i2c_read_two_bytes(uint32_t i2c);
uint16_t i2c_read_multiple_bytes(uint32_t i2c);
uint8_t i2c_busy(uint32_t i2c);
uint8_t i2c_start_generated(uint32_t i2c);
uint8_t i2c_address_sent(uint32_t i2c);
uint8_t i2c_byte_transfer_finished(uint32_t i2c);
uint8_t i2c_nack_received(uint32_t i2c);
uint8_t i2c_data_received(uint32_t i2c);

#endif

