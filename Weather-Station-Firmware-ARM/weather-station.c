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

Development platform ET-STM32F103 board. LEDS on B8-B15.
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

#include <libopencm3/stm32/gpio.h>
#include "../libs/DHT.h"
#include "../libs/hardware.h"

/* 2 second period */
#define MEASUREMENT_PERIOD 2000

/*--------------------------------------------------------------------------*/
/* Global Variables */

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

void usart_print_fixed_point(uint32_t value);
void usart_print_int(int value);
void usart_print_hex(uint16_t value);
void usart_print_string(char *ch);

/*--------------------------------------------------------------------------*/

int main(void)
{
	hardwareSetup();
    DHT sensor = {DHT_PIN,DHT22,false};
    initDHT(&sensor);

	for (;;)
    {
    	gpio_toggle(GPIOB, GPIO9);      /* LED2 on/off. */
        uint32_t temperature = readTemperature(&sensor, false);
        uint32_t humidity = readHumidity(&sensor);
        usart_print_fixed_point(temperature);
        usart_print_string(" ");
        usart_print_fixed_point(humidity);
        usart_print_string(" ");
        usart_print_int(millis());
        usart_print_string("\n\r");
        delay(MEASUREMENT_PERIOD);
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

