/* Various Utility Routines for Strings.

12 October 2017
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
#include "stringlib.h"

/* Local Prototypes */

/* Globals */

/*--------------------------------------------------------------------------*/
/** @brief Convert an ASCII decimal string to an integer

The conversion stops without error when the first non-numerical character
is encountered.

@param[in] string: char* externally defined buffer with the string.
@returns int32_t: integer value to be converted to ASCII form.
*/

int32_t ascii_to_int(char* string)
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

void hex_to_ascii(int16_t value, char* buffer)
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

void int_to_ascii(int32_t value, char* buffer)
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

void string_append(char* string, char* appendage)
{
    uint8_t i=0;
    uint8_t j=string_length(string);
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

void string_copy(char* string, char* original)
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

uint16_t string_length(char* string)
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

uint16_t string_equal(char* string1,char* string2)
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

void string_clear(char* string)
{
    string[0] = 0;
}


