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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include "../libs/buffer.h"
#include "../libs/DHT.h"
#include "../libs/hardware.h"

/* 2 second period in milliseconds */
#define MEASUREMENT_PERIOD 2000

/*--------------------------------------------------------------------------*/
/* Global Variables */

uint32_t rainfall;                  /* Counter of interrupts from sensor */
uint32_t windSpeed;                 /* Counter of interrupts from sensor */

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

static void hardwareSetup(void);
void i2c1Setup(void);
void adcSetup(void);
void gpioSetup(void);
void dacSetup(void);
void extiSetup(void);

/*--------------------------------------------------------------------------*/

int main(void)
{
	hardwareSetup();
    uint8_t channel[1];                 /* Channel for A/D conversion */
    uint16_t chargeLimit = 0;           /* voltage limit for charging battery */
    DHT sensorDHT = {DHT_PIN,DHT22,false};
    initDHT(&sensorDHT);

	for (;;)
    {
    	gpio_toggle(GPIOB, GPIO9);      /* LED2 on/off. */

/* Read and send Temperature and Humidity from the DTH22. */
        uint32_t temperature = readTemperature(&sensorDHT, false);
        uint32_t humidity = readHumidity(&sensorDHT);
        usart_print_string("dT,");
        usart_print_fixed_point(temperature);
        usart_print_string("\n\r");
        usart_print_string("dH,");
        usart_print_fixed_point(humidity);
        usart_print_string("\n\r");

/* Read and send temperature and barometric pressure over i2c. */

/* Read and send solar panel current. Use simple polling of the ADC. */

        uint16_t radiance = 0;
        channel[0] = ADC_CHANNEL4;         /* channel 4 radiance */
	    adc_set_regular_sequence(ADC1, 1, channel);
        adc_start_conversion_direct(ADC1);
        while (!adc_eoc(ADC1));
        radiance = adc_read_regular(ADC1);
        usart_print_string("dL,");
        usart_print_int(radiance);
        usart_print_string("\n\r");

/* Send rain gauge count. */

        usart_print_string("dR,");
        usart_print_int(rainfall);
        usart_print_string("\n\r");
        cli();
        rainfall = 0;
        sei();

/* Send wind speed count. */

        usart_print_string("dS,");
        usart_print_int(windSpeed);
        usart_print_string("\n\r");
        cli();
        windSpeed = 0;
        sei();

/* Send Battery Voltage. */

        uint16_t voltage = 0;
        channel[0] = ADC_CHANNEL6;         /* channel 6 battery voltage */
	    adc_set_regular_sequence(ADC1, 1, channel);
        adc_start_conversion_direct(ADC1);
        while (!adc_eoc(ADC1));
        voltage = adc_read_regular(ADC1);
        usart_print_string("dV,");
        usart_print_int(voltage);
        usart_print_string("\n\r");

/* Send Battery Current. */

        uint16_t current = 0;
        channel[0] = ADC_CHANNEL7;         /* channel 7 battery current */
	    adc_set_regular_sequence(ADC1, 1, channel);
        adc_start_conversion_direct(ADC1);
        while (!adc_eoc(ADC1));
        current = adc_read_regular(ADC1);
        usart_print_string("dI,");
        usart_print_int(current);
        usart_print_string("\n\r");

/* Control Battery Charging */

        chargeLimit = 4000;                 /* temporary for testing */
        dac_software_trigger(CHANNEL_2);
        dac_load_data_buffer_single(chargeLimit, RIGHT12, CHANNEL_2);

/* Snooze for a while */
        delay(MEASUREMENT_PERIOD);
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
/* @brief      Hardware specific settings  */
/*--------------------------------------------------------------------------*/
/* @brief Hardware Setup.

*/

void hardwareSetup(void)
{
/* Set the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

    gpioSetup();
//    systickSetup(1);            // Set systick to interrupt after 1 millisecond.
    systickSetup(1000);         // Set systick to interrupt after 1 second.
    usart1Setup();
    timer2Setup(0xFFFF);
    adcSetup();
    dacSetup();
    i2c1Setup();
    extiSetup();
    sei();
}

/*--------------------------------------------------------------------------*/
/* @brief GPIO Setup.

This sets the clocks for the GPIO and AFIO ports, and sets the LEDs on
PB8 - PB15 to output and cleared.
*/

void gpioSetup(void)
{
/* Enable GPIOA, GPIOB and GPIOC clocks and alternate functions. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);

/* Set GPIO8-15 (in GPIO port B) to 'output push-pull' for the LEDs. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 |
              GPIO12 | GPIO13 | GPIO14 | GPIO15);
/* All LEDS off */
	gpio_clear(GPIOB, GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
               GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------------*/
/* @brief ADC Setup.

ADC1 is turned on and calibrated.
*/

#define ADC_INPUTS  (GPIO4 | GPIO6 | GPIO7)

void adcSetup(void)
{
/* Set ports on PA for ADC1 to analogue input. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, ADC_INPUTS);
/* Enable the ADC1 clock on APB2 */
    rcc_periph_clock_enable(RCC_ADC1);
/* Setup the ADC */
    adc_power_on(ADC1);
    adc_calibration(ADC1);
}

/*--------------------------------------------------------------------------*/
/* @brief Setup DAC

DAC channel 2 is setup on GPIO PA5.
*/

void dacSetup(void)
{
/* Set port PA5 for DAC1 to 'alternate function'. Output driver mode is ignored. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5);
/* Enable the DAC clock on APB1 */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_DACEN);
/* Setup the DAC, software trigger source. Assume the DAC has
woken up by the time the first interrupt occurs */
	dac_enable(CHANNEL_2);
	dac_trigger_enable(CHANNEL_2);
	dac_set_trigger_source(DAC_CR_TSEL1_SW);
	dac_load_data_buffer_single(0, RIGHT12, CHANNEL_2);
}

/*--------------------------------------------------------------------------*/
/* @brief Setup I2C1

The clocks and GPIO settings are established.
*/

void i2c1Setup(void)
{
/* Enable clocks for I2C1 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);
/* Set alternate functions for the SCL and SDA pins of I2C1. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C1_SCL | GPIO_I2C1_SDA);
}

/*--------------------------------------------------------------------------*/
/* @brief EXTI Setup.

This enables the external interrupts on bits 0, 2 and 3 of the ports.
*/

#define EXTI_ENABLES        (EXTI0 | EXTI2 | EXTI3)
#define PA_DIGITAL_INPUTS   (GPIO0 | GPIO2 | GPIO3)

void extiSetup(void)
{
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
                  PA_DIGITAL_INPUTS);
    gpio_set(GPIOA,PA_DIGITAL_INPUTS);      // Pull up
    exti_select_source(EXTI0, GPIOA);
    exti_select_source(EXTI2, GPIOA);
    exti_select_source(EXTI3, GPIOA);
    exti_set_trigger(EXTI_ENABLES, EXTI_TRIGGER_RISING);
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    nvic_enable_irq(NVIC_EXTI2_IRQ);
    nvic_enable_irq(NVIC_EXTI3_IRQ);
    exti_enable_request(EXTI_ENABLES);
}

/*--------------------------------------------------------------------------*/
/* Interrupt Service Routines */
/*--------------------------------------------------------------------------*/
/* EXT0

Bit 0 of each port used as a pin interrupt. This is for rainfall.
*/

void exti0_isr(void)
{
	gpio_toggle(GPIOB, GPIO10);      /* LED3 on/off. */
    rainfall++;
    exti_reset_request(EXTI0);
}

/*--------------------------------------------------------------------------*/
/* EXT2

Bit 2 of each port used as a pin interrupt. This is for wind speed.
*/

void exti2_isr(void)
{
	gpio_toggle(GPIOB, GPIO11);      /* LED4 on/off. */
    windSpeed++;
    exti_reset_request(EXTI2);
}

/*--------------------------------------------------------------------------*/
/* EXT3

Bit 3 of each port used as a pin interrupt. This is for wind direction.
*/

void exti3_isr(void)
{
	gpio_toggle(GPIOB, GPIO12);      /* LED5 on/off. */
    exti_reset_request(EXTI3);
}

/*--------------------------------------------------------------------------*/

