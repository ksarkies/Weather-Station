/*	Circular Buffer Management

Copyright (C) K. Sarkies <ksarkies@internode.on.net>

19 September 2012

The buffer is 8 bit and is defined externally. It has the first element
as the buffer size, the second as the buffer head and the third as the
buffer tail. Space allocated must be three more than the buffer contents.

Buffer head points to the last data item put in the buffer (if any).
Buffer tail points to the place behind the next item to be taken.
*/

#include "buffer.h"

/* Initialize the buffer to empty, defining the size */
void buffer_init(uint8_t buffer[], uint8_t size)
{
	buffer[0] = size;
	buffer[1] = 0;		/* Head */
	buffer[2] = 0;		/* Tail */
}

/* Get a byte from the buffer. Returns a byte in the lower 8 bits,
or 0x100 if the buffer has no data. */
uint16_t buffer_get(uint8_t buffer[])
{
    if ( buffer[1] == buffer[2] ) return 0x100;   	/* no data available */
    uint8_t tmptail = (buffer[2] + 1);
	if (tmptail == buffer[0]) tmptail = 0;
    buffer[2] = tmptail; 
    return (uint16_t) buffer[tmptail+3];
}

/* Put a byte to the buffer. Returns 0x100 if the buffer has no space. */
uint16_t buffer_put(uint8_t buffer[], uint8_t data)
{
    uint8_t tmphead  = (buffer[1] + 1);
	if (tmphead == buffer[0]) tmphead = 0;
    if (tmphead == buffer[2]) return 0x100;   	/* no space available */
    buffer[1] = tmphead;
    buffer[tmphead+3] = data;
	return 0;
}

/* Return true if the buffer has space available */
bool buffer_output_free(uint8_t buffer[])
{
    uint8_t tmphead  = (buffer[1] + 1);
	if (tmphead == buffer[0]) tmphead = 0;
    return (tmphead != buffer[2]);
}

/* Return true if the buffer has a byte available */
bool buffer_input_available(uint8_t buffer[])
{
    return (buffer[1] != buffer[2]);
}

