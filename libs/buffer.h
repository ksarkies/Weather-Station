/*	Circular Buffer Management

Copyright (C) K. Sarkies <ksarkies@internode.on.net>

19 September 2012

Intended for libopencm3 but can be adapted to other libraries provided
data types defined.
*/

#ifndef BUFFER_H
#define BUFFER_H

#include <libopencm3/cm3/common.h>

#define BUFFER_EMPTY 0x100
#define BUFFER_FULL 0x200

void buffer_init(uint8_t buffer[], uint8_t size);
uint16_t buffer_get(uint8_t buffer[]);
uint16_t buffer_put(uint8_t buffer[], uint8_t data);
bool buffer_output_free(uint8_t buffer[]);
bool buffer_input_available(uint8_t buffer[]);

#endif 

