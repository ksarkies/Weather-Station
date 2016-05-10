/* 
@mainpage DHT Temperature/Humidity sensor library 
@version 0.0
@author Adafruit Industries 21 Jun 2011
@author modified for general use by Ken Sarkies (www.jiggerjuice.net)
@date 09 May 2016

Originally written for Arduino, this adapts the code to other hardware
configurations. The external file defines.c is provided by the user to adapt
the library to the hardware to be used. See example code.

Handles the following sensors:

DHT11: this is a low cost device with temperature accuracy to 5C and humidity
       valid 20%-80%

DHT21:

DHT22: this device provides temperature accurate to 0.5C and humidity over the
       full range.
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
*******************************************************************************
*/

#include "DHT.h"
#include "defines.h"

//-----------------------------------------------------------------------------
// Local data types

DHT::DHT(uint8_t pin, uint8_t type)
{
    _pin = pin;
    _type = type;
    firstreading = true;
}

//-----------------------------------------------------------------------------
/* @brief Initialization of hardware I/O pin

This is specific to the application. Change defines.c to define the pin used and
configuration.
*/

void DHT::begin(void)
{
// Code to set pin to digital input and set pullup high.
   _lastreadtime = 0;
}

//-----------------------------------------------------------------------------
/* @brief Read the Temperature from the sensor

Globals: data[] returned by read() function.
         _type: sensor type

@param[in] boolean S: Scale, True = Farenheit; False = Celcius
@returns float: temperature. NaN if an error occurred.
*/

float DHT::readTemperature(bool S)
{
    float f;

    if (read())
    {
        switch (_type)
        {
        case DHT11:
            f = data[2];
            if (S)
            f = convertCtoF(f);
            return f;
        case DHT22:
        case DHT21:
            f = data[2] & 0x7F;
            f *= 256;
            f += data[3];
            f /= 10;
            if (data[2] & 0x80)
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

float DHT::convertCtoF(float c)
{
	return c * 9 / 5 + 32;
}

//-----------------------------------------------------------------------------
/* @brief Read humidity from the sensor

Globals: data[] returned by read() function.
         _type: sensor type

@returns float: Humidity 0 to 100. NaN if an error occurred.
*/

float DHT::readHumidity(void)
{
    float f;
    if (read())
    {
        switch (_type)
        {
        case DHT11:
            f = data[0];
            return f;
        case DHT22:
        case DHT21:
            f = data[0];
            f *= 256;
            f += data[1];
            f /= 10;
            return f;
        }
    }
    return NAN;
}

//-----------------------------------------------------------------------------
/* @brief Read Device

Globals: data[]: filled with result of read.
         _lastreadtime:
         firstreading:

@returns bool: true if value read without error.
*/
boolean DHT::read(void)
{
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;
    unsigned long currenttime;

// pull the pin high and wait 250 milliseconds
/**** code to set pin high */
    delay(250);

    currenttime = millis();
// if there was a rollover
    if (currenttime < _lastreadtime) _lastreadtime = 0;
    if (!firstreading && ((currenttime - _lastreadtime) < 2000))
        return true;            // return last correct measurement
//delay(2000 - (currenttime - _lastreadtime));
    firstreading = false;
    _lastreadtime = millis();

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  
// now pull it low for ~20 milliseconds
/**** code to set pin to output */
/**** code to set pin low */
    delay(20);
    cli();
/**** code to set pin high */
    delayMicroseconds(40);
/**** code to set pin to input */

// read in timings
    for ( i=0; i< MAXTIMINGS; i++)
    {
        counter = 0;
        while (/**** code to read pin */ == laststate)
        {
            counter++;
            delayMicroseconds(1);
            if (counter == 255) break;
        }
        laststate = /**** code to read pin */;

        if (counter == 255) break;

// ignore first 3 transitions
        if ((i >= 4) && (i%2 == 0))  
        {
// shove each bit into the storage bytes
            data[j/8] <<= 1;
            if (counter > 6)
                data[j/8] |= 1;
            j++;
        }

    }

    sei();
  
// check we read 40 bits and that the checksum matches
    if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)))
        return true;

    return false;

}
