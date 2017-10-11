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

void dataMessageSend(char* ident, int32_t param1, int32_t param2)
{
    commsPrintString(ident);
    commsPrintString(",");
    commsPrintInt(param1);
    commsPrintString(",");
    commsPrintInt(param2);
    commsPrintString("\r\n");
}

/*--------------------------------------------------------------------------*/
/** @brief Send a data message with one integer parameter

Use to send a simple response to a command.

@param[in] ident: char* Response identifier string
@param[in] parameter: int32_t Single integer parameter.
*/

void sendResponse(char* ident, int32_t parameter)
{
    commsPrintString(ident);
    commsPrintString(",");
    commsPrintInt(parameter);
    commsPrintString("\r\n");
}

/*--------------------------------------------------------------------------*/
/** @brief Send a debug message with one parameter.

Use to send a simple debug response to a command.

@param[in] ident: char* Response identifier string
@param[in] parameter: int32_t Single integer parameter.
*/

void sendDebugResponse(char* ident, int32_t parameter)
{
    if (ident[0] == 'D')
    {
        commsPrintString(ident);
        commsPrintString(",");
        commsPrintInt(parameter);
        commsPrintString("\r\n");
    }
}

/*--------------------------------------------------------------------------*/
/** @brief Send a string

Use to send a single string.

@param[in] ident: char* Response identifier string
@param[in] string: char* Arbitrary length string.
*/

void sendString(char* ident, char* string)
{
    commsPrintString(ident);
    commsPrintString(",");
    commsPrintString(string);
    commsPrintString("\r\n");
}

/*--------------------------------------------------------------------------*/
/** @brief Print out the contents of a register (debug)

@param[in] reg: uint32_t full address of register.

*/

void commsPrintRegister(uint32_t reg)
{
    commsPrintHex((reg >> 16) & 0xFFFF);
    commsPrintHex((reg >> 00) & 0xFFFF);
    commsPrintChar(" ");
}

/*--------------------------------------------------------------------------*/
/** @brief Print out an integer value in ASCII decimal form (ack Thomas Otto)

@param[in] value: int32_t integer value to be printed.
*/

void commsPrintInt(int32_t value)
{
    uint8_t i=0;
    char buffer[25];
    intToAscii(value, buffer);
    while (buffer[i] > 0)
    {
        commsPrintChar(&buffer[i]);
        i++;
    }
}

/*--------------------------------------------------------------------------*/
/** @brief Print out an integer value in 16 bit ASCII hex form

@param[in] value: int32_t integer value to be printed.
*/

void commsPrintHex(uint32_t value)
{
    uint8_t i=0;
    char buffer[25];
    hexToAscii(value, buffer);
    while (buffer[i] > 0)
    {
        commsPrintChar(&buffer[i]);
        i++;
    }
}

/*--------------------------------------------------------------------------*/
/** @brief Print a String

@param[in] ch: char* pointer to string to be printed.
*/

void commsPrintString(char* ch)
{
    while(*ch) commsPrintChar(ch++);
}

/*--------------------------------------------------------------------------*/
/** @brief Print a Character

This is where the characters are queued for the ISR to transmit.
The ISR is in the hardware module.

Characters are placed on a queue and picked up by the ISR for transmission.

Blocks if there is no space left on the queue.

@param[in] ch: char* pointer to character to be printed.
*/

void commsPrintChar(char* ch)
{
    comms_enable_tx_interrupt(false);
    while (buffer_put(send_buffer, *ch) == 0x100);
    comms_enable_tx_interrupt(true);
}

/*--------------------------------------------------------------------------*/
/** @brief Convert an ASCII decimal string to an integer

The conversion stops without error when the first non-numerical character
is encountered.

@param[in] string: char* externally defined buffer with the string.
@returns int32_t: integer value to be converted to ASCII form.
*/

int32_t asciiToInt(char* string)
{
    int32_t number = 0;
    while ((*string >= '0') && (*string <= '9'))
    {
        number = number*10+(*string - '0');
        string++;
    }
    return number;
}

/*--------------------------------------------------------------------------*/
/** @brief Convert a 16 bit Integer to ASCII hexadecimal string form

@param[in] value: int16_t integer value to be converted to ASCII form.
@param[in] buffer: char* externally defined buffer to hold the result.
*/

void hexToAscii(int16_t value, char* buffer)
{
    uint8_t i = 0;

    for (i = 0; i < 4; i++)
    {
        buffer[i] = "0123456789ABCDEF"[(value >> 12) & 0xF];
        value <<= 4;
    }
    buffer[4] = 0;
}

/*--------------------------------------------------------------------------*/
/** @brief Convert a 32 bit Integer to ASCII decimal string form

@param[in] value: int32_t integer value to be converted to ASCII form.
@param[in] buffer: char* externally defined buffer to hold the result.
*/

void intToAscii(int32_t value, char* buffer)
{
    uint8_t nr_digits = 0;
    uint8_t i = 0;
    char temp_buffer[25];

/* Add minus sign if negative, and form absolute */
    if (value < 0)
    {
        buffer[nr_digits++] = '-';
        value = value * -1;
    }
/* Stop if value is zero */
    if (value == 0) buffer[nr_digits++] = '0';
    else
    {
/* Build string in reverse */
        while (value > 0)
        {
            temp_buffer[i++] = "0123456789"[value % 10];
            value /= 10;
        }
/* Copy across correcting the order */
        while (i > 0)
        {
            buffer[nr_digits++] = temp_buffer[--i];
        }
    }
    buffer[nr_digits] = 0;
}

/*--------------------------------------------------------------------------*/
/** @brief Append a string to another

@param[in] string: char* original string, returned after appending.
@param[in] appendage: char* string to be appended to end.
*/

void stringAppend(char* string, char* appendage)
{
    uint8_t i=0;
    uint8_t j=stringLength(string);
    while (appendage[i] > 0)
    {
        string[j++] = appendage[i++];
    }
    string[j] = 0;
}

/*--------------------------------------------------------------------------*/
/** @brief String Copy

@param[out] string: char* copied string, returned.
@param[in] original: char* original string to be copied.
*/

void stringCopy(char* string, char* original)
{
    uint8_t i=0;
    while (original[i] > 0)
    {
        string[i] = original[i];
        i++;
    }
    string[i] = 0;
}

/*--------------------------------------------------------------------------*/
/** @brief Compute string length

@param[in] string: char* string to be measured for length.
@returns int16_t: length of string.
*/

uint16_t stringLength(char* string)
{
    uint8_t i=0;
    while (string[i] > 0) i++;
    return i;
}

/*--------------------------------------------------------------------------*/
/** @brief Equality of Strings

@param[in] string1: char* first string
@param[in] string2: char* second string
@returns uint8_t: 1 if strings are equal, 0 otherwise.
*/

uint16_t stringEqual(char* string1,char* string2)
{
    while (*string1 > 0)
    {
        if (*string1 != *string2) return 0;
        string1++;
        string2++;
    }
    return 1;
}

/*--------------------------------------------------------------------------*/
/** @brief Clear String

@param[in] string: char* string
*/

void stringClear(char* string)
{
    string[0] = 0;
}


