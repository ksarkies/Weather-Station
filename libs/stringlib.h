/*	Various Utility Routines for Strings.

Copyright (C) K. Sarkies <ksarkies@internode.on.net>

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

#ifndef STRINGLIB_H
#define STRINGLIB_H

/*--------------------------------------------------------------------------*/
/* Prototypes */
/*--------------------------------------------------------------------------*/

void hex_to_ascii(int16_t value, char* buffer);
int32_t ascii_to_int(char* buffer);
void int_to_ascii(int32_t value, char* buffer);
void string_append(char* string, char* appendage);
void string_copy(char* string, char* original);
uint16_t string_length(char* string);
uint16_t string_equal(char* string1, char* string2);
void string_clear(char* string);

#endif 

