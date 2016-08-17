/* 
@mainpage BaroSensor library, for Freetronics BARO module (MS5637-02BA03)
@version 0.0
@author Angus Gratton (angus at freetronics dot com)
@author modified for general use by Ken Sarkies (www.jiggerjuice.info)
@date 04 June 2016
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

#include <stdint.h>
#include <stddef.h>

#include <libopencm3/stm32/i2c.h>
#include "i2c.h"
#include "BaroSensor.h"
#include "hardware.h"

/* i2c address of barometric sensor module */
#define BARO_ADDR 0x76

/* module commands */
#define CMD_RESET 0x1E
/* Offset 0-7 */
#define CMD_PROM_READ(offs) (0xA0+(offs<<1))
/* oversample_level is a parameter in function calls */
#define CMD_START_D1(oversample_level) (0x40 + 2*(int)oversample_level)
#define CMD_START_D2(oversample_level) (0x50 + 2*(int)oversample_level)
#define CMD_READ_ADC 0x00

/*--------------------------------------------------------------------------*/
/* Globals */

static bool initialised;
static int8_t err;
static uint16_t c1,c2,c3,c4,c5,c6;      /* Calibration constants */

/* delay to wait for sampling to complete, on each OSR level */
static const uint8_t SamplingDelayMs[6] = {2,4,6,10,18,34};

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

uint32_t takeBaroReading(uint8_t trigger_cmd, BaroOversampleLevel oversample_level);

/*--------------------------------------------------------------------------*/
/* @brief Initialization of sensor hardware

Resets the module then pulls in calibration constants from the module ROM.
*/

void initBaroSensor(void)
{
    uint8_t command[1];
    uint16_t calibration[7];
    i2c1_setup();
    command[0] = CMD_RESET;
    i2c_master_transmit_data(I2C1, BARO_ADDR, 1, command);
    if (i2c_check_error(I2C1)) return;

/* Pull in the calibration constants */
    for (int i = 0; i < 7; i++) {
        command[0] = CMD_PROM_READ(i);
        i2c_master_transmit_data(I2C1, BARO_ADDR, 1, command);
        if (i2c_check_error(I2C1)) return;
        calibration[i] = i2c_master_read_two_bytes(I2C1, BARO_ADDR);
    }

// TODO verify CRC4 in top 4 bits of prom[0] (follows AN520 but not directly...)

    c1 = calibration[1];
    c2 = calibration[2];
    c3 = calibration[3];
    c4 = calibration[4];
    c5 = calibration[5];
    c6 = calibration[6];
    initialised = true;
}

/*--------------------------------------------------------------------------*/
bool isBaroOK()
{
    return initialised && (err == 0);
}

/*--------------------------------------------------------------------------*/
uint8_t getBaroError()
{
    return initialised ? err : ERR_NEEDS_BEGIN;
}

/*--------------------------------------------------------------------------*/
bool getTempAndPressure(float *temperature, float *pressure, TempUnit tempScale,
                        BaroOversampleLevel level)
{
    if(err || !initialised)
        return false;

    int32_t d2 = takeBaroReading(CMD_START_D2(level), level);
    if(d2 == 0)
        return false;
    int64_t dt = d2 - c5 * (1L<<8);

    int32_t temp = 2000 + (dt * c6) / (1L<<23);

    /* Second order temperature compensation */
    int64_t t2;
    if(temp >= 2000) {
        /* High temperature */
        t2 = 5 * (dt * dt) / (1LL<<38);
    } else {
        /* Low temperature */
        t2 = 3 * (dt * dt) / (1LL<<33);
    }

    if(temperature != NULL) {
        *temperature = (float)(temp - t2) / 100;
        if(tempScale == FAHRENHEIT)
            *temperature = *temperature * 9 / 5 + 32;
    }

    if(pressure != NULL) {
        int32_t d1 = takeBaroReading(CMD_START_D1(level), level);
        if(d1 == 0)
            return false;

        int64_t off = c2 * (1LL<<17) + (c4 * dt) / (1LL<<6);
        int64_t sens = c1 * (1LL<<16) + (c3 * dt) / (1LL<<7);

        /* Second order temperature compensation for pressure */
        if(temp < 2000) {
            /* Low temperature */
            int32_t tx = temp-2000;
            tx *= tx;
            int32_t off2 = 61 * tx / (1<<4);
            int32_t sens2 = 29 * tx / (1<<4);
            if(temp < -1500) {
                /* Very low temperature */
                tx = temp+1500;
                tx *= tx;
                off2 += 17 * tx;
                sens2 += 9 * tx;
            }
            off -= off2;
            sens -= sens2;
        }

        int32_t p = ((int64_t)d1 * sens/(1LL<<21) - off) / (1LL << 15);
        *pressure = (float)p / 100;
    }
    return true;
}

/*--------------------------------------------------------------------------*/
uint32_t takeBaroReading(uint8_t trigger_cmd, BaroOversampleLevel oversample_level)
{
    uint8_t command[1];
    command[0] = trigger_cmd;
    i2c_master_transmit_data(I2C1, BARO_ADDR, 1, command);
    err = i2c_check_error(I2C1);

    if(err)
        return 0;
    uint8_t sampling_delay = SamplingDelayMs[(uint32_t)oversample_level];
    delay(sampling_delay);

    command[0] = CMD_READ_ADC;
    i2c_master_transmit_data(I2C1, BARO_ADDR, 1, command);
    err = i2c_check_error(I2C1);
    if(err)
        return 0;
    uint8_t data[3];
    i2c_master_read_multiple_bytes(I2C1, BARO_ADDR, 3, data);
    uint32_t result = (uint32_t)data[0] << 16;
    result |= (uint32_t)data[1] << 8;
    result |= data[2];
    return result;
}

