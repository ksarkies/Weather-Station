/**
@mainpage Weather Station Firmware for ARM based Remote Unit
@version 0.0.0
@author Ken Sarkies (www.jiggerjuice.info)
@date 07 May 2016

This initial development version uses the ET-STM32F103 development board.

- Rainwater gauge. Tipping bucket with reed switch.
- Wind Speed. Reed switch.
- Wind Direction. Reed switch.
- Temperature and Humidity. Freetronics DHT22.
- Solar Radiance (scale dependent on the panel material). Suntech-STP005S12-5W.
- Air Pressure. Freetronics MS-5637-02BA03.

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

#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include "../libs/DHT.h"
#include "../libs/hardware.h"

#define PERIOD 200
#define DEADTIME 30

/*--------------------------------------------------------------------------*/
/* Global Variables */

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

void hardwareSetup(void);

/*--------------------------------------------------------------------------*/

int main(void)
{
    DHT dht = {DHT_PIN,DHT22,false};
    initDHT(&dht);
	hardwareSetup();
    systickSetup();

	while (1) {

	}

	return 0;
}

/*--------------------------------------------------------------------------*/
/* @brief Hardware Setup

*/

void hardwareSetup(void)
{
/* Set the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

/* Enable GPIOA, GPIOB and GPIOC clocks and alternate functions. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);
}

/*--------------------------------------------------------------------------*/

