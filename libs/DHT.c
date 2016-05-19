/* 
@mainpage DHT Temperature/Humidity sensor library 
@version 0.0
@author Adafruit Industries 21 Jun 2011
@author modified for general use by Ken Sarkies (www.jiggerjuice.net)
@date 09 May 2016

Based on a C++ version originally written for Arduino, this adapts to other
hardware configurations. The external file defines.c is provided by the user
to adapt the library to the hardware to be used. See example code.

The low level read/write to the device has been changed to generalise the code
application. Originally the read timing was tied strictly to the Arduino 16MHz
ATMega328 timing. However this is now independent of timing and relies on
comparing the pulse widths within the signal.

Handles the following sensors:

DHT11: this is a low cost device with temperature accuracy to 5C and humidity
       valid 20%-80%

DHT21: Similar to DHT22 but different casing. Code is identical for both.

DHT22: this device provides temperature accurate to 0.5C and humidity over the
       full range 0-100%.
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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "DHT.h"
#include "hardware.h"

//-----------------------------------------------------------------------------
/* @brief Initialization of hardware I/O pin

This is specific to the application. Change defines.c to define the pin used and
configuration.

@param[in] *sensor: DHT for the sensor used.
*/

void initDHT(DHT *sensor)
{
// Code to set pin to digital input and set pullup high.
    pinMode(sensor->pin, INPUT);
    digitalWrite(sensor->pin, HIGH);
    sensor->lastreadtime = 0;
}

//-----------------------------------------------------------------------------
/* @brief Read the Temperature from the sensor

@param[in] *sensor: DHT for the sensor used.
@param[in] boolean S: Scale, True = Farenheit; False = Celcius
@returns float: temperature. NaN if an error occurred.
*/

float readTemperature(DHT *sensor, bool S)
{
    float f;

    if (readDHT(sensor))
    {
        switch (sensor->type)
        {
        case DHT11:
            f = sensor->data[2];
            if (S)
            f = convertCtoF(f);
            return f;
        case DHT22:
        case DHT21:
            f = sensor->data[2] & 0x7F;
            f *= 256;
            f += sensor->data[3];
            f /= 10;
            if (sensor->data[2] & 0x80)
    	        f *= -1;
            if (S)
    	        f = convertCtoF(f);
            return f;
        }
    }
    return NAN;
}

//-----------------------------------------------------------------------------
/* @brief Conversion Celsius to Fahrenheit

@param[in] float c: Celcius temperature
@returns float: Fahrenheit temperature.
*/

float convertCtoF(float C)
{
	return 1.8*C + 32;
}

//-----------------------------------------------------------------------------
/* @brief Read humidity from the sensor

@param[in] *sensor: DHT for the sensor used.
@returns float: Humidity 0 to 100. NaN if an error occurred.
*/

float readHumidity(DHT *sensor)
{
    float f;
    if (readDHT(sensor))
    {
        switch (sensor->type)
        {
        case DHT11:
            f = sensor->data[0];
            return f;
        case DHT22:
        case DHT21:
            f = sensor->data[0];
            f *= 256;
            f += sensor->data[1];
            f /= 10;
            return f;
        }
    }
    return NAN;
}

//-----------------------------------------------------------------------------
/* @brief Read Device

The DHT devices have only a single data line. These send out pulse width coded
signals in response to the data line being pulled low. A 0 is represented by
a high-going pulse (mark) of 25 to 28 microseconds, while a 1 is represented by
a high-going pulse (mark) of 70 microseconds. An initial sync pattern is also
sent.

The original code used a timed loop to sample the signal and identify the length
of the pulse. This loop in C with two function calls and for the 16MHz AVR in
the Arduino appears to take over 5 microseconds to execute. This will require
code changes for other processors and instruction clock rates.

@param[in] *sensor: DHT for the sensor used.
@returns data[]: uint8_t array of 6 bits of data read from device. Global.
@returns bool: true if value read without error.
*/

bool readDHT(DHT *sensor)
{
    uint8_t j = 0, i;
    uint32_t currenttime;

// pull the pin high and wait 250 milliseconds
    digitalWrite(sensor->pin, HIGH);
    delay(250);

/* The specification requires that each collection period be > 2 seconds.
Therefore if the last call time was less than two seconds ago, just return the
previously stored value. */
    currenttime = millis();         // millisecond system clock.
// if there was a rollover in the millis() time.
    if (currenttime < sensor->lastreadtime) sensor->lastreadtime = 0;
    if (!sensor->firstreading && ((currenttime - sensor->lastreadtime) < 2000))
        return true;            // return last correct measurement
//delay(2000 - (currenttime - sensor->lastreadtime));
    sensor->firstreading = false;
    sensor->lastreadtime = millis();

// data array: integer/fraction humidity, integer/fraction temperature, checksum
    sensor->data[0] = sensor->data[1] = sensor->data[2] = sensor->data[3] = sensor->data[4] = 0;
  
/* Now pull it low for ~20 milliseconds, then set it high again.
(the specification only requires 1ms minimum time).
This initiates a reading. */
    pinMode(sensor->pin, OUTPUT);
    digitalWrite(sensor->pin, LOW);
    delay(20);
    cli();                  // Prevent any interrupt interfering with timing.
    digitalWrite(sensor->pin, HIGH);

// Determine lowest space period as sometimes it exceeds the long mark period
// at the beginning of a byte (seems to be just before checksum is issued).
    uint16_t minlowcounter = 255;
// Wait for first space period in sync phase to be established
    delayMicroseconds(40);  // Initial delay for device to respond
/* Change back to input to read data from device */
    pinMode(sensor->pin, INPUT);

    uint16_t highCounts[45];
// read in number of cycle counts that the signal is low and high
// The microsecond delay allows faster processors to limit count magnitude
// For the 16MHz Arduino, the cycle code alone takes about 5 microseconds.
    for ( i=0; i< 42; i++)
    {
// Count space period
        uint16_t lowCounter = 0;
        while (digitalRead(sensor->pin) == 0)
        {
            delayMicroseconds(1);
            if (lowCounter++ > 250) break;
        }
        if (lowCounter > 250) break;
        if (lowCounter < minlowcounter) minlowcounter = lowCounter;
// Count mark period
        uint16_t highCounter = 0;
        while (digitalRead(sensor->pin) == 1)
        {
            delayMicroseconds(1);
            if (highCounter++ > 250) break;
        }
        if (highCounter > 250) break;
        highCounts[i] = highCounter;
    }
    for ( i=0; i< 42; i++)
    {
// shove each bit into the storage bytes
// If the length of a high is smaller than the space, it is 0, otherwise 1
        if (i >= 1)
        {
            sensor->data[j/8] <<= 1;
            if (minlowcounter < highCounts[i]) sensor->data[j/8] |= 1;
            j++;
        }
    }

    sei();
  
// check we read 40 bits and that the checksum matches
    if ((j >= 40) && (sensor->data[4] == ((sensor->data[0] + sensor->data[1]
                                         + sensor->data[2] + sensor->data[3]) & 0xFF)))
        return true;

    return false;

}
