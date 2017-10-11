/*	Various Utility Routines for Communications.

Copyright (C) K. Sarkies <ksarkies@internode.on.net>

9 December 2016
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

#ifndef COMMS_H
#define COMMS_H

/*--------------------------------------------------------------------------*/
/* Prototypes */
/*--------------------------------------------------------------------------*/

void init_comms_buffers(void);
bool receive_data_available(void);
uint8_t get_from_receive_buffer(void);
uint16_t put_to_receive_buffer(uint8_t character);
uint16_t get_from_send_buffer(void);
void dataMessageSend(char* ident, int32_t parm1, int32_t parm2);
void sendResponse(char* ident, int32_t parameter);
void sendDebugResponse(char* ident, int32_t parameter);
void sendString(char* ident, char* string);
void sendDebugString(char* ident, char* string);
void commsPrintInt(int32_t value);
void commsPrintHex(uint32_t value);
void commsPrintString(char* ch);
void commsPrintChar(char* ch);
void hexToAscii(int16_t value, char* buffer);
int32_t asciiToInt(char* buffer);
void intToAscii(int32_t value, char* buffer);
void stringAppend(char* string, char* appendage);
void stringCopy(char* string, char* original);
uint16_t stringLength(char* string);
uint16_t stringEqual(char* string1, char* string2);
void stringClear(char* string);

#endif 

