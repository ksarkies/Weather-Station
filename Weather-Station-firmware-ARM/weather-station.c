/**
@mainpage Weather Station Firmware for ARM based Remote Unit
@version 0.0.0
@author Ken Sarkies (www.jiggerjuice.info)
@date 07 May 2016

This initial development version uses the ET-STM32F103 development board. The
port allocations are:

- PA0: Rainwater gauge. Tipping bucket with reed switch.
- PA1: Temperature and Humidity. Freetronics DHT22.
- PA2: Wind Speed. Reed switch.
- PA3: Wind Direction. Reed switch.
- PA4: Solar Radiance (scale dependent on the panel material). Suntech-STP005S12-5W.
- Air Pressure. Freetronics MS-5637-02BA03. I2C-1 on PB6 for SCL and PB7 for SDA.

For battery charging the following are needed:
- PA5 to measure battery voltage on ADC12-IN5.
- PA6 for PWM on TIM3_CH1 to control the battery charging.
- PA7 to switch the panel to the solar radiance current measurement circuit.

USART1 is on PA8-PA12 with Tx on PA9 and Rx on PA10.

Development platform ET-STM32F103 board. LEDS on B8-B15. USART1 is provided.

The final version uses the ET-ARM STAMP board using the same ports.
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

/* 2 second period in milliseconds */
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
    DHT sensorDHT = {DHT_PIN,DHT22,false};
    initDHT(&sensorDHT);

	for (;;)
    {
    	gpio_toggle(GPIOB, GPIO9);      /* LED2 on/off. */
        uint32_t temperature = readTemperature(&sensorDHT, false);
        uint32_t humidity = readHumidity(&sensorDHT);
        usart_print_string("dT,");
        usart_print_fixed_point(temperature);
        usart_print_string("\n\r");
        usart_print_string("dH,");
        usart_print_fixed_point(humidity);
        usart_print_string("\n\r");
        delaySleep(MEASUREMENT_PERIOD);
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

