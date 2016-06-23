/*  
@mainpage I2C convenience functions.
@version 0.0
@author Ken Sarkies (www.jiggerjuice.info)
@date 22 June 2016


The processor used here is the STM32F103 on the ET-STM32F103 development board.
The hardware library is libopencm3.
*/

#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include "buffer.h"
#include "DHT.h"
#include "i2c.h"

#define  _BV(bit) (1 << (bit))
#define BUFFER_SIZE 128

/*--------------------------------------------------------------------------*/
// Globals

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

/*--------------------------------------------------------------------------*/
/* @brief Initialise I2C

Clock frequencies and characteristics are set, along with own address.
The I2C peripheral is enabled.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
*/

void i2c_initialise(uint32_t i2c)
{
/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(i2c);
/* APB1 is running at 36MHz. */
	i2c_set_clock_frequency(i2c, I2C_CR2_FREQ_36MHZ);
/* 400KHz - I2C Fast Mode */
	i2c_set_fast_mode(i2c);
/* fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
Datasheet suggests 0x1e. */
	i2c_set_ccr(i2c, 0x1e);
/* fclock for I2C is 36MHz -> cycle time 28ns, rise time for
400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
Incremented by 1 -> 11. */
	i2c_set_trise(i2c, 11);
/* This is our slave address - needed only if we want to receive from
other masters. */
	i2c_set_own_7bit_slave_address(i2c, 0x32);
/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(i2c);
}

/*--------------------------------------------------------------------------*/
/* @brief Initiate transmission on I2C

Gets ready for transmission by setting the destination address and indicating
a read or write access.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@param[in] uint8_t address: I2C address of the device.
@param[in] uint8_t rw: I2C_WRITE or I2C_READ.
*/

void i2c_initiate_transmission(uint32_t i2c, uint8_t address, uint8_t rw)
{
	uint32_t reg32 __attribute__((unused));

/* Send START condition. */
	i2c_send_start(i2c);

/* Waiting for START to be sent and switch over to master mode to complete.
This automatically clears the SB bit by reading the SR1 register. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

/* Send destination address. */
	i2c_send_7bit_address(i2c, address, rw);

/* Waiting for address to be transferred.
This automatically reads the SR1 register. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

/* and reading the SR2 register as well is needed to clear the ADDR bit.  */
	reg32 = I2C_SR2(i2c);

}

/*--------------------------------------------------------------------------*/
/* @brief Transmission of Single Byte on I2C

After sending the byte this waits for BTF to be set, at which a stop is
executed.

The i2c_initiate_transmission function must be called first to set the address
to write.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@param[in] uint8_t data: Single data byte to send.
*/

void i2c_transmit_single_byte (uint32_t i2c, uint8_t data)
{
/* Send the data byte. */
	i2c_send_data(i2c, data);

/* Wait for the Byte Transfer Finished BTF flag to be set. */
	while (! i2c_byte_transfer_finished(i2c));

/* Send STOP condition. */
	i2c_send_stop(i2c);
}

/*--------------------------------------------------------------------------*/
/* @brief Transmission of Byte Array on I2C

After sending the byte this waits for BTF to be set, at which a stop is
executed.

The i2c_initiate_transmission function must be called first to set the address
to write.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@param[in] uint8_t *data: Data byte array to send.
*/

void i2c_transmit_multiple_byte (uint32_t i2c, uint8_t *data, uint8_t length)
{
/* Send the data bytes. */
    uint8_t i = 0;
    while (i < length)
    {
    	i2c_send_data(i2c, data[i]);

/* Wait for the TxE flag to be set ready for next byte. */
	    while (!(I2C_SR1(i2c) & I2C_SR1_TxE));
    }

/* Wait for the Byte Transfer Finished BTF flag to be set. */
	while (! i2c_byte_transfer_finished(i2c));

/* Send STOP condition. */
	i2c_send_stop(i2c);
}

/*--------------------------------------------------------------------------*/
/* @brief Check if an error condition occurred on I2C

@returns uint8_t: 0 if no error.
*/

uint8_t i2c_check_error()
{
    return 0;
}

/*--------------------------------------------------------------------------*/
/* @brief Read single byte from I2C

The i2c_initiate_transmission function must be called first to set the address
to read.

Note that two byte word and multiple word messages are handled differently.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@returns uint16_t: data received.
*/

uint8_t i2c_read_single_byte(uint32_t i2c)
{
	uint8_t data = 0;

	return data;
}

/*--------------------------------------------------------------------------*/
/* @brief Read two byte word from I2C

The i2c_initiate_transmission function must be called first to set the address
to read.

Note that single and multiple word messages are handled differently.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@returns uint16_t: data received.
*/

uint16_t i2c_read_two_bytes(uint32_t i2c)
{
	uint16_t data;

/* Clear the ACK bit to prevent an ACK pulse being sent on the second byte. */
	i2c_disable_ack(i2c);

/* Now the slave should begin to send the first byte. Await BTF. */
	while (! i2c_byte_transfer_finished(i2c));

	data = (i2c_get_data(i2c) << 8);        /* MSB */

/* Yes they mean it: we have to generate the STOP condition before
saving the second byte. */
	i2c_send_stop(i2c);

	data |= i2c_get_data(i2c);               /* LSB */

/* NAK this last byte (not sure if this is necessary, not in datasheet). */
	i2c_nack_current(i2c);

	return data;
}

/*--------------------------------------------------------------------------*/
/* @brief Read multiple byte word from I2C

This reads a block of three or more bytes from the I2C.

The i2c_initiate_transmission function must be called first to set the address
to read.

Note that single and two byte word messages are handled differently.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@returns uint16_t: data received.
*/

uint16_t i2c_read_multiple_bytes(uint32_t i2c)
{
	uint16_t data;

/* Clear the ACK bit to prevent an ACK pulse being sent on the second byte. */
	i2c_disable_ack(i2c);

/* Now the slave should begin to send the first byte. Await BTF. */
	while (! i2c_byte_transfer_finished(i2c));

	data = (i2c_get_data(i2c) << 8);        /* MSB */

/* Yes they mean it: we have to generate the STOP condition before
saving the second byte. */
	i2c_send_stop(i2c);

	data |= i2c_get_data(i2c);               /* LSB */

/* NAK this last byte (not sure if this is necessary, not in datasheet). */
	i2c_nack_current(i2c);

	return data;
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check bus busy

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating bus busy.
*/
uint8_t i2c_busy(uint32_t i2c)
{
	return ((I2C_SR2(i2c) & I2C_SR2_BUSY) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check start condition generated

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating start was generated.
*/
uint8_t i2c_start_generated(uint32_t i2c)
{
	return ((I2C_SR1(i2c) & I2C_SR1_SB) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check slave address sent by master

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating address was sent.
*/
uint8_t i2c_address_sent(uint32_t i2c)
{
	if ((I2C_SR1(i2c) & I2C_SR1_ADDR) != 0) {
		//(void)I2C_SR2(i2c);
		return 1;
	}
	return 0;
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check is byte transfer finished (used in transmitter mode)

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating transfer completed.
*/
uint8_t i2c_byte_transfer_finished(uint32_t i2c)
{
	return ((I2C_SR1(i2c) & I2C_SR1_BTF) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check is NACK or ACK received (used in transmitter mode)

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating NAK was received.
*/
uint8_t i2c_nack_received(uint32_t i2c)
{
	return ((I2C_SR1(i2c) & I2C_SR1_AF) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check data received (used receiver mode)

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating data was received.
*/
uint8_t i2c_data_received(uint32_t i2c)
{
	return ((I2C_SR1(i2c) & I2C_SR1_RxNE) != 0);
}

/*--------------------------------------------------------------------------*/

