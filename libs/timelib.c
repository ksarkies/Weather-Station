/*  Utility Routines for Time/Date

The global time and data counter is set and read.

This counter carries the real time count in seconds for time stamping of
records. Access functions are provided for setting from a UTC string and
reading to a string.

Initial 25 November 2013 from Battery Management System
*/

/*
 * Copyright 2013 K. Sarkies <ksarkies@internode.on.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**@{*/

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "hardware.h"
#include "timelib.h"
#include "comms.h"

/*--------------------------------------------------------------------------*/
/** @brief Return a string containing the time and date

Convert the global time to an ISO 8601 string.

@param[out] timeString char*. Returns pointer to string with formatted date.
*/

void put_time_to_string(char* timeString)
{
    time_t currentTime = (time_t)get_seconds_count();
    struct tm *rtc = localtime(&currentTime);
//    strftime(timeString, sizeof timeString, "%FT%TZ", rtc);
    char buffer[10];
    intToAscii(rtc->tm_year+1900, timeString);
    stringAppend(timeString,"-");
    if (rtc->tm_mon < 9) stringAppend(timeString,"0");
    intToAscii(rtc->tm_mon+1, buffer);
    stringAppend(timeString,buffer);
    stringAppend(timeString,"-");
    if (rtc->tm_mday < 10) stringAppend(timeString,"0");
    intToAscii(rtc->tm_mday, buffer);
    stringAppend(timeString,buffer);
    stringAppend(timeString,"T");
    if (rtc->tm_hour < 10) stringAppend(timeString,"0");
    intToAscii(rtc->tm_hour, buffer);
    stringAppend(timeString,buffer);
    stringAppend(timeString,":");
    if (rtc->tm_min < 10) stringAppend(timeString,"0");
    intToAscii(rtc->tm_min, buffer);
    stringAppend(timeString,buffer);
    stringAppend(timeString,":");
    if (rtc->tm_sec < 10) stringAppend(timeString,"0");
    intToAscii(rtc->tm_sec, buffer);
    stringAppend(timeString,buffer);
}

/*--------------------------------------------------------------------------*/
/** @brief Set the time variable from an ISO 8601 formatted date/time

@param[in] timeString: pointer to string with formatted date.
*/

void set_time_from_string(char* timeString)
{
    struct tm newTime;
    char buffer[5];
    uint8_t i;

    for (i=0; i<4; i++) buffer[i] = timeString[i];
    buffer[4] = 0;
    newTime.tm_year = asciiToInt(buffer)-1900;
    for (i=0; i<2; i++) buffer[i] = timeString[i+5];
    buffer[2] = 0;
    newTime.tm_mon = asciiToInt(buffer)-1;
    for (i=0; i<2; i++) buffer[i] = timeString[i+8];
    newTime.tm_mday = asciiToInt(buffer);
    for (i=0; i<2; i++) buffer[i] = timeString[i+11];
    newTime.tm_hour = asciiToInt(buffer);
    for (i=0; i<2; i++) buffer[i] = timeString[i+14];
    newTime.tm_min = asciiToInt(buffer);
    for (i=0; i<2; i++) buffer[i] = timeString[i+17];
    newTime.tm_sec = asciiToInt(buffer);

    set_seconds_count((uint32_t)mktime(&newTime));
}

/**@}*/

