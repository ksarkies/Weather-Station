/** @brief Definition of CANopen Object Dictionary Variables

These are made available to an external PC and to other processing modules
which may be CANopen devices or tasks running on the same microcontroller.

Note these definitions cannot be placed in the header file.
*/

/*
 * This file is part of the weather station project.
 *
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

#include "weather-station-objdic.h"
#include "../libs/hardware.h"

/* Byte pattern that indicates if a valid NVM config data block is present */
#define VALID_BLOCK                 0xD5

/*--------------------------------------------------------------------------*/
/* Preset the config data block in FLASH to a given pattern to indicate unused. */
union ConfigGroup configDataBlock __attribute__ ((section (".configBlock"))) = {{0xA5}};
union ConfigGroup configData;

/*--------------------------------------------------------------------------*/
/** @brief Initialise Global Configuration Variables

This determines if configuration variables are present in NVM, and if so
reads them in. The first entry is checked against a preprogrammed value to
determine if the block is a valid configuration block. This allows the program
to determine whether to use the block stored in FLASH or to use defaults.
*/

void set_global_defaults(void)
{
    flash_read_data((uint32_t*)configDataBlock.data,
                   configData.data,sizeof(configData.config));
    if (configData.config.validBlock == VALID_BLOCK) return;
/* Set default communications control variables */
    configData.config.measurementSend = true;
    configData.config.debugMessageSend = false;
    configData.config.enableSend = true;
/* Set default recording control variables */
    configData.config.recording = false;
/* Set default measurement variables */
    configData.config.measurementInterval = 1000;   /* 1 second intervals */
    configData.config.numberConversions = 6;        /* number of interfaces plus temperature */
    configData.config.numberSamples = 16;           /* burst of samples for averaging */
}

/*--------------------------------------------------------------------------*/
/** @brief Write Configuration Data Block to Flash

Refer to the linker script for allocation of the config data block on a
page boundary. If this is not done, a page erase may destroy valid data or code.

The current datablock is written to flash with the first entry set to a value
that indicates whether the block is a valid programmed configuration block.

@returns uint32_t result code. 0 success, 1 fail.
*/

uint32_t write_config_block(void)
{
    configData.config.validBlock = VALID_BLOCK;
    return flash_write_data((uint32_t*)configDataBlock.data,
                          configData.data, sizeof(configData.config));
}

/*--------------------------------------------------------------------------*/
/** @brief Get Recording switch

True if recording has been requested, false otherwise.

@returns bool recording setting.
*/

bool is_recording(void)
{
    return configData.config.recording;
}

/*--------------------------------------------------------------------------*/
/** @brief Return a status word showing software controls.

bit  0
bit  1   if recording,
bit  2
bit  3   if measurements are being sent
bit  4   if debug messages are being sent
bits 5-15

@returns uint16_t status of controls
*/
uint16_t get_controls(void)
{
    uint16_t controls = 0;
    if (configData.config.recording) controls |= 1<<1;
    if (configData.config.measurementSend) controls |= 1<<3;
    if (configData.config.debugMessageSend) controls |= 1<<4;
    return controls;
}

/**@}*/

