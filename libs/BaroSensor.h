/* BaroSensor library
 *
 * An Arduino library for the Freetronics BARO sensor module, using
 * the Measurement Specialties MS5637-02BA03 Altimeter/Pressure sensor
 * module.
 *
 * http://www.freetronics.com/baro
 */
/******************************************************************************
 *
 * This file is part of the Weather Station project.
 *
 * Copyright (C)2014 Freetronics Pty Ltd. Licensed under GNU GPLv3
 * http://www.freetronics.com/baro
 * Copyright 2016 K. Sarkies <ksarkies@internode.on.net>
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
 *****************************************************************************/
#ifndef _BAROLIBRARY_H
#define _BAROLIBRARY_H

#include "hardware.h"

/* Module supports a range of lower oversampling levels, for faster
less accurate results. Default is maximum accuracy. */
typedef enum {
    OSR_256, OSR_512, OSR_1024, OSR_2048, OSR_4096, OSR_8192
} BaroOversampleLevel;

typedef enum {
    CELSIUS,
    FAHRENHEIT
} TempUnit;

/* error codes */
#define ERR_NOREPLY -1
#define ERR_BAD_READLEN -2
#define ERR_NEEDS_BEGIN -3

void initBaroSensor();

bool getBaroTempAndPressure(float *temperature,
                            float *pressure,
                            TempUnit tempScale,
                            BaroOversampleLevel level);
bool isBaroOK();
uint8_t getBaroError();

#endif
