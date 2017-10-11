/*	Circular Buffer Management

Copyright (C) K. Sarkies <ksarkies@internode.on.net>

19 September 2012

Intended for libopencm3 but can be adapted to other libraries provided
data types defined.
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

#ifndef BUFFER_H
#define BUFFER_H

void buffer_init(uint8_t buffer[], uint8_t size);
uint16_t buffer_get(uint8_t buffer[]);
uint16_t buffer_put(uint8_t buffer[], uint8_t data);
bool buffer_output_free(uint8_t buffer[]);
bool buffer_input_available(uint8_t buffer[]);

#endif 

