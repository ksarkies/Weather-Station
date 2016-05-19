/* DHT Temperature/Humidity sensor library 

written by Adafruit Industries 21 Jun 2011
*/

/******************************************************************************
    MIT Licence

    Copyright (c) 2011 Adafruit Industries


    Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*******************************************************************************/
#ifndef _DHT_H_
#define _DHT_H_

#include <stdint.h>

// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

/* Data structure for a humidity-temperature sensor */
typedef struct
{
/* I/O pin to which the communication line from the sensor is connected */
    uint8_t pin;
/* Sensor type */
    uint8_t type;
/* Carry persistent variables in case it is used by more than one sensor */
    bool firstreading;
    unsigned long lastreadtime;
/* integer/fraction humidity, integer/fraction temperature, checksum */
    uint8_t data[6];
} DHT;

bool readDHT(DHT *sensor);
void initDHT(DHT *sensor);
float readTemperature(DHT *sensor, bool S);
float convertCtoF(float C);
float readHumidity(DHT *sensor);

#endif

