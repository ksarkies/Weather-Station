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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define PERIOD 200
#define DEADTIME 30

/*--------------------------------------------------------------------------*/
/* Global Variables */

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

void hardware_setup(void);
void timer_setup(void);

/*--------------------------------------------------------------------------*/

int main(void)
{
	hardware_setup();
	timer_setup();

	while (1) {

	}

	return 0;
}

/*--------------------------------------------------------------------------*/
/* @brief Hardware Setup

*/

void hardware_setup(void)
{
/* Set the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

/* Enable GPIOA, GPIOB and GPIOC clocks.
   APB2 (High Speed Advanced Peripheral Bus) peripheral clock enable register (RCC_APB2ENR)
   Set RCC_APB2ENR_IOPBEN for port B, RCC_APB2ENR_IOPAEN for port A and RCC_APB2ENR_IOPAEN
   for Alternate Function clock */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
								 RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN);

/* Set ports PA9 (TIM1_CH2), PA10 (TIM1_CH3), PB14 (TIM1_CH2N), PB15 (TIM1_CH3N)
for PWM, to 'alternate function output push-pull'. */
/*	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9 | GPIO10);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO14 | GPIO15); */

/* Set ports PA9 (TIM1_CH2), PA10 (TIM1_CH3), PB14 (TIM1_CH2N), PB15 (TIM1_CH3N)
for PWM, to 'alternate function output push-pull'. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9 | GPIO10);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------------*/
/* @brief Timer Setup

*/

void timer_setup(void)
{
/* Enable TIM1 clock. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

/* Reset TIM1 peripheral. */
	timer_reset(TIM1);

/* Set Timer global mode:
 * - No division
 * - Alignment centre mode 1 (up/down counting, interrupt on downcount only)
 * - Direction up (when centre mode is set it is read only, changes by hardware)
 */
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);

/* Set Timer output compare mode:
 * - PWM mode 2 (output low when CNT < CCR1, high otherwise)
 * Channel 2 is the one to use for buck synchronous mode */
	timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);
	timer_enable_oc_output(TIM1, TIM_OC2);
	timer_enable_oc_output(TIM1, TIM_OC2N);
/* Channel 3 */
	timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM2);
	timer_enable_oc_output(TIM1, TIM_OC3);
	timer_enable_oc_output(TIM1, TIM_OC3N);
	timer_enable_break_main_output(TIM1);
/* Set the polarity of OC2N to be low to match that of the OC, for switching
the low side MOSFET through an inverting level shifter */
    timer_set_oc_polarity_low(TIM1, TIM_OC2N);
/* Set the deadtime for OC2N. All deadtimes are set to this. */
    timer_set_deadtime(TIM1, DEADTIME);

/* The ARR (auto-preload register) sets the PWM period to 62.5kHz from the
72 MHz clock.*/
	timer_enable_preload(TIM1);
	timer_set_period(TIM1, PERIOD);

/* The CCR1 (capture/compare register 1) sets the PWM duty cycle to default 50% */
	timer_enable_oc_preload(TIM1, TIM_OC2);
	timer_set_oc_value(TIM1, TIM_OC2, (PERIOD*50)/100);
	timer_enable_oc_preload(TIM1, TIM_OC3);
	timer_set_oc_value(TIM1, TIM_OC3, (PERIOD*20)/100);

/* Force an update to load the shadow registers */
	timer_generate_event(TIM1, TIM_EGR_UG);

/* Start the Counter. */
	timer_enable_counter(TIM1);
}

