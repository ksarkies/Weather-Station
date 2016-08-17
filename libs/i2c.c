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
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include "buffer.h"
#include "DHT.h"
#include "i2c.h"
#include "hardware.h"       /* TO BE REMOVED */

#define  _BV(bit) (1 << (bit))
#define BUFFER_SIZE 128
#define OWN_ADDRESS 0x32

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
	i2c_set_own_7bit_slave_address(i2c, OWN_ADDRESS);
/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(i2c);
}

/*--------------------------------------------------------------------------*/
/* @brief Initiate transmission on I2C

Gets ready for transmission by setting the destination address and indicating
a read or write access. Initiation of a start on the bus forces master mode.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@param[in] uint8_t address: I2C address of the device.
@param[in] uint8_t rw: I2C_WRITE or I2C_READ.
*/

void i2c_initiate_transmission_7_bit(uint32_t i2c, uint8_t address, uint8_t rw)
{
	uint32_t reg32 __attribute__((unused));

/* Send START condition. */
	i2c_send_start(i2c);

/* Wait for START to be sent and the change over to master mode to be completed.
The bus should then also be busy having been grabbed by the new master.
This automatically clears the SB bit by reading the SR1 register. */
	while (! (i2c_start_generated(i2c) &&
             (i2c_master_mode(i2c) || i2c_busy(i2c))));

/* Send destination address. */
	i2c_send_7bit_address(i2c, address, rw);

/* Waiting for address to be transferred.
This automatically reads the SR1 register. */
	while (! i2c_address_sent(i2c));

/* and reading the SR2 register as well is needed to cause the ADDR bit to be
cleared.  */
	reg32 = I2C_SR2(i2c);
}

/*--------------------------------------------------------------------------*/
/* @brief Transmission of Byte Array on I2C as Master

After sending the last byte this waits for TxE and/or BTF to be set, at which a
stop is executed.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@param[in] uint8_t *data: Data byte array to send.
*/

void i2c_master_transmit_data(uint32_t i2c, uint8_t address, uint8_t length,
                              uint8_t *data)
{
    i2c_initiate_transmission_7_bit(i2c, address, I2C_WRITE);
/* Send the data bytes. */
    uint8_t i = 0;
    while (i++ < length)
    {
    	i2c_send_data(i2c, data[i]);

/* Wait for the TxE flag to be set ready for next byte. If this is missed then
the BTF flag will be set and must be cleared by reading SR1. */
	    while (! (i2c_byte_transfer_finished(i2c) || i2c_data_sent(i2c)));
    }
/* Wait for the Byte Transfer Finished BTF flag to be set. */
	while (! i2c_byte_transfer_finished(i2c));

/* Send STOP condition. */
	i2c_send_stop(i2c);
}

/*--------------------------------------------------------------------------*/
/* @brief Receive single byte from I2C as Master

Reading a single byte requires a different set of processes to those for the
two and multiple byte cases. The ACK bit must be cleared before the addressing
process is completed.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@returns 8_t: data byte received.
*/

uint8_t i2c_master_read_single_byte(uint32_t i2c, uint8_t address)
{
	uint8_t data = 0;
	uint32_t reg32 __attribute__((unused));

/* Send START condition. */
	i2c_send_start(i2c);

/* Wait for START to be sent and the change over to master mode to be completed.
The bus should then also be busy having been grabbed by the new master.
This automatically clears the SB bit by reading the SR1 register. */
	while (! (i2c_start_generated(i2c) &&
             (i2c_master_mode(i2c) || i2c_busy(i2c))));

/* Send destination address. */
	i2c_send_7bit_address(i2c, address, I2C_READ);

/* Waiting for address to be transferred.
This automatically reads the SR1 register. */
	while (! i2c_address_sent(i2c));

/* Disable the ACK signal. */
	i2c_disable_ack(i2c);

/* Reading the SR2 register as well is needed to cause the ADDR bit to be
cleared.  */
	reg32 = I2C_SR2(i2c);

/* Generate the STOP condition before reading the received byte. */
	i2c_send_stop(i2c);

/* Wait for the transfer to complete and read the data. */
    while (! i2c_data_received(i2c));
	data |= i2c_get_data(i2c);               /* LSB */

	i2c_enable_ack(i2c);

	return data;
}

/*--------------------------------------------------------------------------*/
/* @brief Receive two byte word from I2C as Master

Reading a single byte requires a different set of processes to those for the
two and multiple byte cases. The ACK bit and the NACK bit must be enabled before
the addressing process is completed.

Note that single and multiple word messages are handled differently.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@returns uint16_t: data word received.
*/

uint16_t i2c_master_read_two_bytes(uint32_t i2c, uint8_t address)
{
	uint16_t data;
	uint32_t reg32 __attribute__((unused));

/* Send START condition. */
	i2c_send_start(i2c);

/* Wait for START to be sent and the change over to master mode to be completed.
The bus should then also be busy having been grabbed by the new master.
This automatically clears the SB bit by reading the SR1 register. */
	while (! (i2c_start_generated(i2c) &&
             (i2c_master_mode(i2c) || i2c_busy(i2c))));

/* Send destination address. */
	i2c_send_7bit_address(i2c, address, I2C_READ);

/* Enable the ACK signal and set NACK to occur on the next byte transfer.
This is an exceptional case. */
	i2c_enable_ack(i2c);
    i2c_nack_next(i2c);

/* Waiting for address to be transferred.
This automatically reads the SR1 register. */
	while (! i2c_address_sent(i2c));

/* Reading the SR2 register as well is needed to cause the ADDR bit to be
cleared.  */
	reg32 = I2C_SR2(i2c);

/* Clear the ACK bit to prevent an ACK pulse being sent on the second byte. */
	i2c_disable_ack(i2c);

/* Now the slave should begin to send the first byte. Await BTF as we need to
ensure that when the next byte arrives it holds up the slave. */
	while (! i2c_byte_transfer_finished(i2c));

/* Generate the STOP condition before reading the second byte. */
	i2c_send_stop(i2c);

	data = (i2c_get_data(i2c) << 8);            /* MSB */
    while (! (i2c_byte_transfer_finished(i2c) || i2c_data_received(i2c)));
	data |= i2c_get_data(i2c);                  /* LSB */

/* Set the NACK to occur on the current byte for any future communications. */
	i2c_nack_current(i2c);
	i2c_enable_ack(i2c);

	return data;
}

/*--------------------------------------------------------------------------*/
/* @brief Receive multiple byte array from I2C as Master

This reads a block of three or more bytes from the I2C. The process requires
careful attention to the way the last three bytes are read (see manual).

Note that single and two byte word messages are handled differently.

@param[in] uint32_t i2c: i2c channel to use (I2C1 or I2C2).
@returns uint8_t*: pointer to byte array that stores the data received.
*/

void i2c_master_read_multiple_bytes(uint32_t i2c, uint8_t address,
                                    uint8_t length, uint8_t* data)
{
    uint8_t i = length;

    i2c_initiate_transmission_7_bit(i2c, address, I2C_READ);

    while (i > 2)
    {
	    while (! (i2c_byte_transfer_finished(i2c) || i2c_data_received(i2c)));
    	*data++ = i2c_get_data(i2c);
        i--;
    }
/* Now the slave should send the third last byte. Wait for BTF which will be
set when the second last byte arrives in the shift register. */
	while (! i2c_byte_transfer_finished(i2c));

/* Clear the ACK bit to prevent an ACK pulse being sent on the second byte. */
	i2c_disable_ack(i2c);

/* Get data[n-2]. This will allow the slave to send the last byte. */
	*data++ = i2c_get_data(i2c);

/* Now set the stop condition as required. */
	i2c_send_stop(i2c);

/* Pull in the last two bytes. */
    while (! (i2c_byte_transfer_finished(i2c) || i2c_data_received(i2c)));
	*data++ = i2c_get_data(i2c);
    while (! (i2c_byte_transfer_finished(i2c) || i2c_data_received(i2c)));
	*data++ = i2c_get_data(i2c);

/* Set the NACK to occur on the current byte for any future communications. */
	i2c_nack_current(i2c);
	i2c_enable_ack(i2c);
}

/*--------------------------------------------------------------------------*/
/* @brief Check if an error condition occurred on I2C

* bit 0 bus error
* bit 1 arbitration lost
* bit 2 acknowledge failure
* bit 3 overrun/underrun
* bit 4 PEC error in reception
* bit 5 reserved: always 0
* bit 6 timeout
* bit 7 SMB alert

@returns uint8_t: 0 if no error, otherwise error code as above.
*/

uint8_t i2c_check_error(uint32_t i2c)
{
    return (I2C_SR1(i2c) >> 8);
}

/*---------------------------------------------------------------------------*/
/* Additional API Functions */
/*---------------------------------------------------------------------------*/
/** @brief I2C check the Transmit Buffer is Empty

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating data was sent.
*/
uint8_t i2c_data_sent(uint32_t i2c)
{
	return ((I2C_SR1(i2c) & I2C_SR1_TxE) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check Master Mode has been Established

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating peripheral is in master mode.
*/
uint8_t i2c_master_mode(uint32_t i2c)
{
	return ((I2C_SR2(i2c) & I2C_SR2_MSL) != 0);
}

/*---------------------------------------------------------------------------*/
/* Functions provided by PR 484 KivApple */
/*---------------------------------------------------------------------------*/
/** @brief I2C check bus busy

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating bus busy with communication.
*/
uint8_t i2c_busy(uint32_t i2c)
{
	return ((I2C_SR2(i2c) & I2C_SR2_BUSY) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check start condition generated

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating start generation was completed.
*/
uint8_t i2c_start_generated(uint32_t i2c)
{
	return ((I2C_SR1(i2c) & I2C_SR1_SB) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check slave address sent by master

@param[in] i2c Unsigned int32. I2C register base address @ref i2c_reg_base.
@returns uint8_t: boolean, true indicating address transmission was completed.
*/
uint8_t i2c_address_sent(uint32_t i2c)
{
	return ((I2C_SR1(i2c) & I2C_SR1_ADDR) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief I2C check is byte transfer finished (used in transmitter mode)

The BTF flag is set by hardware usually when a transmission is delayed. This
causes the clock low to be stretched to prevent the slave sending anything else
that may ultimately be lost. The flag must be reset before continuing.

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
@returns uint8_t: boolean, true indicating ACK was not received (hence NACK).
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

