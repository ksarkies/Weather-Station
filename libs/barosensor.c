/* 
@mainpage BaroSensor library, for Freetronics BARO module (MS5637-02BA03)
@version 0.0
@author Angus Gratton (angus at freetronics dot com)
@author modified for libopencm3 by Ken Sarkies (www.jiggerjuice.info)
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
#include <stdbool.h>

#include "i2clib.h"
#include "barosensor.h"
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
static uint16_t cal[7];

/* delay to wait for sampling to complete, on each OSR level */
static const uint8_t SamplingDelayMs[6] = {2,4,6,10,18,34};

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

static uint32_t take_baro_reading(uint8_t trigger_cmd,
                                BaroOversampleLevel oversample_level);

/*--------------------------------------------------------------------------*/
/* @brief Initialization of sensor hardware

Resets the module then pulls in calibration constants from the module ROM.
*/

void init_baro_sensor(void)
{
	uint32_t reg32 __attribute__((unused));
    uint8_t command[1];
    i2c_setup(I2C_CHANNEL);
/* Reset Module */
    command[0] = CMD_RESET;
    i2c_master_transmit_data(I2C_CHANNEL, BARO_ADDR, 1, command);
    i2c_terminate(I2C_CHANNEL);

    if (i2c_check_error(I2C_CHANNEL)) return;

    int i = 0;
/* Pull in the calibration constants */
    for (i = 0; i < 7; i++) {
        command[0] = CMD_PROM_READ(i);
        i2c_master_transmit_data(I2C_CHANNEL, BARO_ADDR, 1, command);
        if (i2c_check_error(I2C_CHANNEL)) return;
        cal[i] = i2c_master_read_two_bytes(I2C_CHANNEL, BARO_ADDR);
    }

    initialised = true;
}

/*--------------------------------------------------------------------------*/
bool is_baro_ok()
{
    return initialised && (err == 0);
}

/*--------------------------------------------------------------------------*/
/* @brief Compute Temperature and Pressure

@param[out] *temperature: int32_t value returned as fixed point 8-bit fraction.
@param[out] *pressure: int32_t value returned as fixed point 8-bit fraction.
@param[in] TempUnit: tempScale CELSIUS or FAHRENHEIT.
@param[in] BaroOversampleLevel: oversampling delay level.
*/

bool get_baro_temp_and_pressure(int32_t *temperature, int32_t *pressure,
                            TempUnit tempScale, BaroOversampleLevel level)
{
/* Call to initialise. Global calibration values don't seem to be held over
between calls, despite being declared static */
    init_baro_sensor();
    if(! is_baro_ok()) return false;

/* Temperature computation */
    int32_t d2 = take_baro_reading(CMD_START_D2(level), level);
    if(d2 == 0) return false;

    int64_t dt = d2 - (cal[5]<<8);

    int32_t temp = 2000 + ((dt * cal[6]) >> 23);

/* Second order temperature compensation */
    int64_t t2;
    if(temp >= 2000) {
/* High temperature */
        t2 = 5 * ((dt * dt) >> 38);
    } else {
/* Low temperature */
        t2 = 3 * ((dt * dt) >> 33);
    }

    if(temperature != NULL) {
        *temperature = (int32_t)((temp - t2) * 256 / 100);
        if(tempScale == FAHRENHEIT)
            *temperature = (*temperature * 9 + 160*256) / 5;
    }

/* Pressure computation */
    if(pressure != NULL) {
        int32_t d1 = take_baro_reading(CMD_START_D1(level), level);
        if(d1 == 0) return false;

        int64_t off = ((int64_t)cal[2]<<17) + (((int64_t)cal[4] * dt)>>6);
        int64_t sens = ((int64_t)cal[1]<<16) + (((int64_t)cal[3] * dt)>>7);

/* Second order temperature compensation for pressure */
        if(temp < 2000) {
/* Low temperature */
            int32_t tx = temp-2000;
            tx *= tx;
            int32_t off2 = 61 * (tx>>4);
            int32_t sens2 = 29 * (tx>>4);
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

        int32_t p = (((((int64_t)d1 * sens)>>21) - off)>>15);
        *pressure = (int32_t)p *256 / 100;
    }
    return true;
}

/*--------------------------------------------------------------------------*/
/* @brief Read the sensor

@param[in] trigger_cmd: int8_t choice of reading to make (T or P).
@param[in] BaroOversampleLevel: oversampling delay level.
@return uint32_t value read back.
*/

uint32_t take_baro_reading(uint8_t trigger_cmd,
                         BaroOversampleLevel oversample_level)
{
    uint8_t command[1];
    command[0] = trigger_cmd;
    i2c_master_transmit_data(I2C_CHANNEL, BARO_ADDR, 1, command);
    i2c_terminate(I2C_CHANNEL);

    if(i2c_check_error(I2C_CHANNEL)) return 0;
    uint8_t sampling_delay = SamplingDelayMs[(uint32_t)oversample_level];
    delay(sampling_delay);

    command[0] = CMD_READ_ADC;
    i2c_master_transmit_data(I2C_CHANNEL, BARO_ADDR, 1, command);
    if(i2c_check_error(I2C_CHANNEL)) return 0;
    uint8_t data[3];
    i2c_master_read_multiple_bytes(I2C_CHANNEL, BARO_ADDR, 3, data);
    uint32_t result = (uint32_t)data[0] << 16;
    result |= (uint32_t)data[1] << 8;
    result |= data[2];
    return result;
}

