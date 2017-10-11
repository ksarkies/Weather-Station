/* Weather Station Object Dictionary

This file defines the CANopen object dictionary variables made available to an
external PC and to other processing modules which may be CANopen devices or
tasks running on the same microcontroller.
*/

/*
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
 */

#ifndef _WEATHER_STATION_OBJDIC_H_
#define _WEATHER_STATION_OBJDIC_H_

#define FIRMWARE_VERSION    "1.00"

/*--------------------------------------------------------------------------*/
/****** Object Dictionary Items *******/
/* Configuration items, updated externally, are stored to NVM */
/* Values must be initialized in setGlobalDefaults(). */

/*--------------------------------------------------------------------------*/
struct Config
{
/* Valid data block indicator */
    uint8_t validBlock;
/* Communications Control Variables */
    bool enableSend;            /* Any communications transmission occurs */
    bool measurementSend;       /* Measurements are transmitted */
    bool debugMessageSend;      /* Debug messages are transmitted */
/* Recording Control Variables */
    bool recording;             /* Recording of performance data */
/* Measurement Variables */
    uint32_t measurementInterval;   /* Time between measurements */
    uint8_t numberConversions;  /* Number of channels to be converted */
    uint8_t numberSamples;      /* Number of samples for averaging */
};

/* Map the configuration data also as a block of words.
Block size equal to a FLASH page (2048 bytes) to avoid erase problems.
Needed for reading and writing from Flash. */
#define CONFIG_BLOCK_SIZE       2048
union ConfigGroup
{
    uint8_t data[CONFIG_BLOCK_SIZE];
    struct Config config;
};

/*--------------------------------------------------------------------------*/
/* Prototypes */
/*--------------------------------------------------------------------------*/

void set_global_defaults(void);
uint32_t write_config_block(void);
bool is_recording(void);
uint16_t get_controls(void);

#endif

