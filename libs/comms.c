/* Various Utility Routines for Communications.

As all communications are made in ASCII form, these routines are provided
for number to string and string manipulation.

Communications have the form of a small two character header (ident) followed by
parameters in ASCII string form separated by commas. Thses are an asynchronous
TxPDOs in CANopen and therefore nominally of fixed length. 

K. Sarkies, 9 December 2016
*/

/*
 * Copyright (C) K. Sarkies <ksarkies@internode.on.net>
 *
 * This project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include "buffer.h"
#include "stringlib.h"
#include "hardware.h"
#include "comms.h"

/* Local Prototypes */

/* Globals */
uint8_t send_buffer[BUFFER_SIZE+3];
uint8_t receive_buffer[BUFFER_SIZE+3];

/* These configuration variables are part of the Object Dictionary. */
/* This is defined in data-acquisition-objdic and is updated in response to
received messages. */
extern union ConfigGroup configData;

/*--------------------------------------------------------------------------*/
/** @brief Initialise the Communications Buffers

Globals: send_buffer, receive_buffer
*/

void init_comms_buffers(void)
{
	buffer_init(send_buffer,BUFFER_SIZE);
	buffer_init(receive_buffer,BUFFER_SIZE);
}

/*--------------------------------------------------------------------------*/
/** @brief Check if received data is available

@returns true if data is present in the receive buffer
*/

bool receive_data_available(void)
{
    return buffer_input_available(receive_buffer);
}

/*--------------------------------------------------------------------------*/
/** @brief Get data from receive buffer

@returns uint8_t: next character from the receive buffer
*/

uint8_t get_from_receive_buffer(void)
{
    return buffer_get(receive_buffer);
}

/*--------------------------------------------------------------------------*/
/** @brief Send data character to the receive buffer

@param[in] uint8_t: character to put to the send buffer
*/

uint16_t put_to_receive_buffer(uint8_t character)
{
    return buffer_put(receive_buffer, character);
}

/*--------------------------------------------------------------------------*/
/** @brief Get data from send buffer

@returns uint8_t: next character from the send buffer
*/

uint16_t get_from_send_buffer(void)
{
    return buffer_get(send_buffer);
}

/*--------------------------------------------------------------------------*/
/** @brief Send a data message with two integer parameters

The response parameters are converted to ASCII integer.

@param ident: char* an identifier string recognized by the receiving program.
@param param1: int32_t first integer parameter.
@param param2: int32_t second integer parameter.
*/

void data_message_send(char* ident, int32_t param1, int32_t param2)
{
    comms_print_string(ident);
    comms_print_string(",");
    comms_print_int(param1);
    comms_print_string(",");
    comms_print_int(param2);
    comms_print_string("\r\n");
}

/*--------------------------------------------------------------------------*/
/** @brief Send a data message with one integer parameter

Use to send a simple response to a command.

@param[in] ident: char* Response identifier string
@param[in] parameter: int32_t Single integer parameter.
*/

void send_response(char* ident, int32_t parameter)
{
    comms_print_string(ident);
    comms_print_string(",");
    comms_print_int(parameter);
    comms_print_string("\r\n");
}

/*--------------------------------------------------------------------------*/
/** @brief Send a data message with one fixed point parameter

Send a fixed point value in which the lower 8 bits are the fraction.

@param[in] ident: char* Response identifier string
@param[in] parameter: int32_t Single fixed point parameter.
*/

void send_fixed_point(char* ident, int32_t parameter)
{
    comms_print_string(ident);
    comms_print_string(",");
    comms_print_fixed_point(parameter);
    comms_print_string("\r\n");
}

/*--------------------------------------------------------------------------*/
/** @brief Send a debug message with one parameter.

Use to send a simple debug response to a command.

@param[in] ident: char* Response identifier string
@param[in] parameter: int32_t Single integer parameter.
*/

void send_debug_response(char* ident, int32_t parameter)
{
    if (ident[0] == 'D')
    {
        comms_print_string(ident);
        comms_print_string(",");
        comms_print_int(parameter);
        comms_print_string("\r\n");
    }
}

/*--------------------------------------------------------------------------*/
/** @brief Send a string

Use to send a single string.

@param[in] ident: char* Response identifier string
@param[in] string: char* Arbitrary length string.
*/

void send_string(char* ident, char* string)
{
    comms_print_string(ident);
    comms_print_string(",");
    comms_print_string(string);
    comms_print_string("\r\n");
}

/*--------------------------------------------------------------------------*/
/** @brief Print out the contents of a register (debug)

@param[in] reg: uint32_t full address of register.

*/

void commsPrintRegister(uint32_t reg)
{
    comms_print_hex((reg >> 16) & 0xFFFF);
    comms_print_hex((reg >> 00) & 0xFFFF);
    comms_print_char(" ");
}

/*--------------------------------------------------------------------------*/
/** @brief Print out an integer value in ASCII decimal form (ack Thomas Otto)

@param[in] value: int32_t integer value to be printed.
*/

void comms_print_int(int32_t value)
{
    uint8_t i=0;
    char buffer[25];
    int_to_ascii(value, buffer);
    while (buffer[i] > 0)
    {
        comms_print_char(&buffer[i]);
        i++;
    }
}

/*--------------------------------------------------------------------------*/
/** @brief Print out an integer value in 16 bit ASCII hex form

@param[in] value: int32_t integer value to be printed.
*/

void comms_print_hex(uint32_t value)
{
    uint8_t i=0;
    char buffer[25];
    hex_to_ascii(value, buffer);
    while (buffer[i] > 0)
    {
        comms_print_char(&buffer[i]);
        i++;
    }
}

/*--------------------------------------------------------------------------*/
/** @brief Print a String

@param[in] ch: char* pointer to string to be printed.
*/

void comms_print_string(char* ch)
{
    while(*ch) comms_print_char(ch++);
}

/*--------------------------------------------------------------------------*/
/** @brief Print a Character

This is where the characters are queued for the ISR to transmit.
The ISR is in the hardware module.

Characters are placed on a queue and picked up by the ISR for transmission.

Blocks if there is no space left on the queue.

@param[in] ch: char* pointer to character to be printed.
*/

void comms_print_char(char* ch)
{
    comms_enable_tx_interrupt(false);
    while (buffer_put(send_buffer, *ch) == 0x100);
    comms_enable_tx_interrupt(true);
}

/*--------------------------------------------------------------------------*/
/* @brief Print out a fixed point value in ASCII decimal form.

Fixed point arithmetic based on 32 bit signed integer of which the first 8 bits
are the fractional part.

@param[in] param: 32 bit signed integer as uint32_t.
*/

void comms_print_fixed_point(uint32_t param)
{
    char buffer[32];
    fixed_point_to_ascii(param,buffer);
    comms_print_string(buffer);
	comms_enable_tx_interrupt(true);
}

