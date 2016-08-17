/**
@mainpage Weather Station Firmware for ARM based Remote Unit
@version 0.0.0
@author Ken Sarkies (www.jiggerjuice.info)
@date 04 July 2016

This initial development version uses the ET-ARM STAMP board. The port
allocations are:

- PA0: Rainwater gauge. Tipping bucket with reed switch.
- PA1: Temperature and Humidity. Freetronics DHT22.
- PA2: Wind Speed. Reed switch.
- PA3: Wind Direction. Reed switch.
- PA4: Solar Radiance (scale dependent on the panel material).
       Suntech-STP005S12-5W.
- Air Pressure. Freetronics MS-5637-02BA03. I2C-1 on PB6 for SCL, PB7 for SDA.

For battery charging the following are needed:
- PA6 to measure battery voltage on ADC12-IN5.
- PA7 to measure battery current on ADC12-IN6.
- PB2 to switch the panel to charge the battery.
- PB5 to switch the panel to the solar radiance current measurement circuit.

USART1 is on PA8-PA12 with Tx on PA9 and Rx on PA10.

For Memory card SPI interface:
- PD2: for memory card present signal.
- PB12: for memory card select.
- PB13: for memory card SCK.
- PB14: for memory card MISO.
- PB15: for memory card MOSI.

NOTE: The battery must be in place during operation otherwise when the charger
is disabled by the program the processor will power off.

NOTE: the test ET-STM32 STAMP board has a faulty PB4 and PB3.
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

#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include "../libs/buffer.h"
#include "../libs/DHT.h"
#include "../libs/BaroSensor.h"
#include "../libs/i2c.h"
#include "../libs/hardware.h"

/* 10 second sleep/stop period */
#define MEASUREMENT_PERIOD        10
/* Time in ms to allow radiance current to settle after switching. */
#define RADIANCE_SETTLE_TIME     100
/* Resistor values times 10 */
#define BATTERY_SENSE             12        
#define RADIANCE_SENSE            10       
/* Amplification of current sense voltage */
#define I_AMP                    110 
/* Amplification of battery voltage */
#define V_AMP                    259 

/* Fixed point for battery charge limit for Diamec batteries (7.2V to 7.5V). */
#define CHARGE_LIMIT       72*256/10
/* Fixed point for battery float limit for Diamec batteries (6.5V to 6.8V). */
#define FLOAT_LIMIT        68*256/10

/*--------------------------------------------------------------------------*/
/* Global Variables */

static uint32_t rainfall;               /* Counter of interrupts from sensor */
static uint32_t wind_speed;             /* Counter of interrupts from sensor */
static uint8_t charger_is_active;       /* Existing state of charger */
static uint32_t measurement_period;

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

static void enable_radiance_measurement(void);
static void disable_radiance_measurement(void);
static void enable_charging(void);
static void disable_charging(void);
static uint8_t charger_active(void);
static void hardware_setup(void);
static void parse_command(uint8_t* line);

/*--------------------------------------------------------------------------*/

int main(void)
{
	hardware_setup();
    charger_is_active = charger_active();
    uint8_t channel[1];                 /* Channel for A/D conversion */
    uint16_t charge_limit = 0;          /* voltage limit for charging battery */
    uint8_t i=0;
    DHT sensor_DHT = {DHT_PIN,DHT22,false};
    init_DHT(&sensor_DHT);
//    initBaroSensor();
    usart_print_string("Weather Station\n\r");
    uint8_t characterPosition = 0;
    uint8_t line[80];
    measurement_period = MEASUREMENT_PERIOD;
    uint32_t measurement_time = measurement_period;

    for (;;) {
/* Snooze for a while in low power stop mode. */
/* Wait a bit for USART and any other outstanding operations to finish. */
        while (! usart_get_flag(USART1, USART_SR_TC));
		uint32_t cnt;
		for (cnt=0; cnt < 40000; cnt++) {
			asm("nop");
		}

    	rtc_set_alarm_time(measurement_time);
/* Turn off peripherals during stop mode (only ADC and DAC need to do this). */
        peripheral_disable();
/* Set sleep mode and stop */
		pwr_voltage_regulator_low_power_in_stop();
/* Don't set complete power down (otherwise it goes to standby) */
		pwr_set_stop_mode();
/* Just clear the whole bloody lot of exti pending requests */
		exti_reset_request(0xFFFFF);
/* Use light sleep if USART interrupts are to be triggered */
#ifdef DEEPSLEEP
/* Set deep sleep mode bit in SCB to go to stop mode */
		SCB_SCR |= SCB_SCR_SLEEPDEEP;
#endif
		asm volatile("wfi");
/* Repeat setup as clocks seem to have been reset. */

/* Restore hardware clocks and any config that may have been lost. */
        peripheral_enable();

/* Check if the wakeup source was the alarm. If so clear the alarm flag and
reset the alarm value ahead of the RTC. */
		if (rtc_check_flag(RTC_ALR)) {
			rtc_clear_flag(RTC_ALR);
            measurement_time = rtc_get_counter_val() + measurement_period;

/* These tasks are now performed whenever the RTC alarm wakes the processor.
Other interrupts will activate their ISR but are ignored and the processor
will go back into stop mode. */

/* Read and send Battery Voltage. */
            uint16_t voltage_raw = 0;
            channel[0] = ADC_CHANNEL6;          /* channel 6 battery voltage */
	        adc_set_regular_sequence(ADC1, 1, channel);
/* Average over 16 readings */
            for (i=0; i<16; i++) {
                adc_start_conversion_direct(ADC1);
                while (! adc_eoc(ADC1));
                voltage_raw += adc_read_regular(ADC1);
                delay(1);
            }
/* Voltage times 256 for fixed point scaling, with sense resistor (x10)
and amplification (x100) will give results in volts. */
            uint32_t voltage = ((voltage_raw>>4)*256*V_AMP)/(1241*100);/* Volts */
            usart_print_string("dV,");
            usart_print_fixed_point(voltage);
            usart_print_string("\n\r");

/* Read and send Battery Current. */
            uint32_t current_raw = 0;
            channel[0] = ADC_CHANNEL7;          /* channel 7 battery current */
	        adc_set_regular_sequence(ADC1, 1, channel);
/* Average over 16 readings */
            for (i=0; i<16; i++) {
                adc_start_conversion_direct(ADC1);
                while (! adc_eoc(ADC1));
                current_raw += adc_read_regular(ADC1);
            }
/* Current in m_a times 256 for fixed point scaling, with sense resistor (x10)
and current amplification (x10) and scale back to average 16 readings.
Note order of computations to avoid 32 bit overflow. */
            uint32_t current = 
                    ((current_raw*BATTERY_SENSE*1000)/(1241*I_AMP))*16;  /* m_a */
            usart_print_string("dI,");
            usart_print_fixed_point(current);
            usart_print_string("\n\r");

/* Battery Charge Limit Setting */
            charge_limit = 4000;                 /* temporary for testing */
            dac_load_data_buffer_single(charge_limit, RIGHT12, CHANNEL_2);
            dac_software_trigger(CHANNEL_2);

/* Control Battery Charging.
This cycles between the absorption voltage limit and the float voltage limit. */
            if (voltage > CHARGE_LIMIT) {
                disable_charging();              /* Turn off battery charging */
            }
            if (voltage < FLOAT_LIMIT) {
                enable_charging();               /* Turn on battery charging */
            }

/* Offset for time that systick was asleep */
			millis_offset(measurement_period * 1000);
/* Read and send Temperature and Humidity from the DTH22. */
            uint32_t temperature;
            uint32_t humidity;
			bool error = ! read_temperature_humidity(&sensor_DHT, &temperature,
												   &humidity, false);
			if (error)
			{
	            usart_print_string("D");
	            for (i=0; i<5; i++) {
			        usart_print_string(",");
			        usart_print_int(sensor_DHT.data[i]);
				}
	            usart_print_string("\n\r");
			}
            usart_print_string("dT,");
            usart_print_fixed_point(temperature);
            usart_print_string("\n\r");
            usart_print_string("dH,");
            usart_print_fixed_point(humidity);
            usart_print_string("\n\r");

/* Read and send temperature and barometric pressure over i2c. */
            usart_print_string("dP,");
            usart_print_fixed_point(1000*256);
            usart_print_string("\n\r");

/* Read and send solar panel current. Use simple polling of the ADC. */
            enable_radiance_measurement();
            usart_print_string("D");
            for (i=0; i<5; i++) {
		        usart_print_string(",");
		        usart_print_int(sensor_DHT.data[i]);
			}
            usart_print_string("\n\r");
            int32_t radiance_raw = 0;
            channel[0] = ADC_CHANNEL4;          /* channel 4 radiance */
	        adc_set_regular_sequence(ADC1, 1, channel);
/* Average over 16 readings */
            for (i=0; i<16; i++) {
                adc_start_conversion_direct(ADC1);
                while (! adc_eoc(ADC1));
                radiance_raw += adc_read_regular(ADC1);
            }
            disable_radiance_measurement();       /* Puts charging back on */
/* Current in m_a times 256 for fixed point scaling, with sense resistor (x10)
and current amplification (x10) and scale back to average 16 readings.
Note order of computations to avoid 32 bit overflow. */
            uint32_t radiance =
                    ((radiance_raw*RADIANCE_SENSE*1000)/(1241*I_AMP))*16;/* m_a */
            usart_print_string("dL,");
            usart_print_fixed_point(radiance);
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
            usart_print_int(wind_speed);
            usart_print_string("\n\r");
            cli();
            wind_speed = 0;
            sei();
        }
        else {
            uint16_t value = check_receive_buffer();
            char receive_buffer_empty = (value >> 8);
            if (! receive_buffer_empty)
            {
                char character = (value & 0xFF);
                if ((character == 0x0D) || (character == 0x0A) ||
                    (characterPosition > 78))
                {
                    line[characterPosition] = 0;
                    characterPosition = 0;
                    parse_command(line);
                }
                else line[characterPosition++] = character;
            }
        }
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
/* @brief Action a received USART command line.

*/

void parse_command(uint8_t* line)
{
    usart_print_string("D");
    usart_print_string(",");
    usart_print_string(line);
    usart_print_string("\n\r");
}

/*--------------------------------------------------------------------------*/
/* @brief Hardware Setup.

*/

void hardware_setup(void)
{
/* Set the clock to 72MHz from the 8MHz external crystal */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

    systick_setup(1000);         /* Set systick to interrupt after 1 second. */
    gpio_setup();
    usart1_setup();
    timer2_setup(0xFFFF);
    adc_setup();
    dac_setup();
    i2c1_setup();
	rtc_setup();
    exti_setup();
    sei();
}

/*--------------------------------------------------------------------------*/
/* @brief Enable measurement

The measurement MOSFET must be turned on and the charging must be turned off.
Delay for a while to allow settling of currents in the sense resistor.
*/

void enable_radiance_measurement(void)
{
    charger_is_active = charger_active();
    disable_charging();                  /* turn off charger */
    gpio_set(GPIOB, GPIO2);             /* Turn on measurement switch */
    delay(RADIANCE_SETTLE_TIME);
}

/*--------------------------------------------------------------------------*/
/* @brief Disable measurement

The measurement MOSFET must be turned off and the charging must be restored
if it was in a charging state at the time of measurement.
*/

void disable_radiance_measurement(void)
{
    gpio_clear(GPIOB, GPIO2);           /* Turn off measurement switch */
    if (charger_is_active) enable_charging();
    else disable_charging();
}

/*--------------------------------------------------------------------------*/
/* @brief Enable Charging

Output must be low to turn on the MOSFET.
*/

void enable_charging(void)
{
    gpio_clear(GPIOB, GPIO5);
}

/*--------------------------------------------------------------------------*/
/* @brief Disable Charging

Output must be high to turn off the MOSFET.
*/

void disable_charging(void)
{
    gpio_set(GPIOB, GPIO5);
}

/*--------------------------------------------------------------------------*/
/* @brief Return State of Charging

The measurement MOSFET must be turned on and the charging must be turned off.
Delay for a while to allow settling of currents in the sense resistor.

@returns uint8_t: true = charging active, false = charging inactive.
*/

uint8_t charger_active(void)
{
    return (gpio_get(GPIOB, GPIO5) == 0);
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
    wind_speed++;
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

