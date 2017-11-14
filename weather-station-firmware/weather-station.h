/* Weather Station Firmware

*/

/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* 60 second sleep/stop period */
#define MEASUREMENT_PERIOD         30
/* 5 second light sleep time */
#define LIGHT_SLEEP_TIME            5

/* Time in ms to allow radiance current to settle after switching. */
#define RADIANCE_SETTLE_TIME      100
/* Radiance Current Sense Resistor value times 10 to allow one digit precision */
#define RADIANCE_SENSE_RESISTOR    10
/* Amplification of current sense voltage for radiance */
#define I_AMP_RADIANCE             11

/* Battery Current Sense Resistor value times 10 to allow one digit precision */
#define BATTERY_SENSE_RESISTOR     10
/* Amplification of current sense voltage for radiance */
#define I_AMP_BATTERY              11
/* Amplification of battery voltage */
#define V_AMP                     259

/* Fixed point for battery charge limit for Diamec batteries (7.2V to 7.5V). */
#define CHARGE_LIMIT       72*256/10
/* Fixed point for battery float limit for Diamec batteries (6.5V to 6.8V). */
#define FLOAT_LIMIT        68*256/10

/*--------------------------------------------------------------------------*/

