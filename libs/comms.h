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
void data_message_send(char* ident, int32_t parm1, int32_t parm2);
void send_response(char* ident, int32_t parameter);
void send_fixed_point(char* ident, int32_t parameter);
void send_debug_response(char* ident, int32_t parameter);
void send_string(char* ident, char* string);
void send_debug_string(char* ident, char* string);
void comms_print_int(int32_t value);
void comms_print_hex(uint32_t value);
void comms_print_string(char* ch);
void comms_print_char(char* ch);
void comms_print_fixed_point(uint32_t value);

#endif 

