/**
@mainpage STM32F103 Test of Deep Sleep Modes for power saving
@version 0.0.0
@author Ken Sarkies (www.jiggerjuice.info)
@date 14 July 2016

This is a test of the use of the stop mode in the STM32F103 allowing response
to EXTI and RTC Alarm interrupts. The stop mode is only awakened with an EXTI
(external asynchronous interrupt) while in stop mode, as most clocks are not
running. The RTC, whose low frequency clock LSE is running, has an alarm
interrupt which is directed to EXTI 17. Thus it is the only timing source
available in stop mode.

This program sets up the RTC alarm to interrupt regularly to wake the processor
for other activities, while dealing with asynchronous interrupts on other EXTI
lines.

Interrupt enabled sleep is needed to allow response to interrupts at all times
whether or not the processor is in sleep mode.
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

#include <unistd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/usart.h>

/*--------------------------------------------------------------------------*/
/* Global Variables */

/* interrupt counter */
static uint32_t exti_counter;

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

static void usart1_setup(void);
static void rtc_setup(void);
static void exti_setup(void);
static void usart_print_int(int value);
static void usart_print_string(char *ch);

/*--------------------------------------------------------------------------*/
int main(void)
{
	/* Set the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	usart1_setup();
	usart_print_string("RTC Alarm Test\n\r");
	rtc_setup();
	rtc_set_alarm_time(10);
	exti_setup();
	usart_print_string("RTC Setup Complete\n\r");

	/* Set to stop mode and wait for RTC interrupt. */
	while (1) {

		/* Put in a delay to allow USART to finish. */
		uint32_t delay;
		for (delay = 0; delay < 40000; delay++) {
			asm("nop");
		}

		/* Set sleep mode and stop */
		pwr_voltage_regulator_low_power_in_stop();
		/* Don't set complete power down (else it goes to standby) */
		pwr_set_stop_mode();
		/* Just clear the whole bloody lot of exti pending requests */
		exti_reset_request(0xFFFFF);
		/* Set deep sleep mode bit in SCB to go to stop mode */
		SCB_SCR |= SCB_SCR_SLEEPDEEP;
		asm volatile("wfi");

		/* Repeat setup as clocks seem to have been reset. */
		rcc_clock_setup_in_hse_8mhz_out_72mhz();
		/* Wake up the RTC from the stop condition */
		rtc_auto_awake(RCC_LSE, 0x7FFF);
		/* Check if the wakeup source was the alarm. If so reset the
		RTC counter and set the next alarm. */
		if (rtc_check_flag(RTC_ALR)) {
			rtc_clear_flag(RTC_ALR);
			rtc_set_counter_val(0);
			rtc_set_alarm_time(10);
			/* At this point a whole bunch of other tasks would be
			done, according to the application. */
			usart_print_string("Woken\r\n");
			/* ....... */
		}
		/* Otherwise continue looping. This block just for testing. */
		else {
			usart_print_string("Interrupted ");
			usart_print_int(exti_counter);
			usart_print_string("\r\n");
		}
	}

	return 0;
}
/*--------------------------------------------------------------------------*/
/* @brief Print a String.
*/

void usart_print_string(char *ch)
{
	while (*ch) {
		usart_send_blocking(USART1, (*(ch++) & 0xFF));
	}
}

/*--------------------------------------------------------------------------*/
/* @brief Print out an integer value in ASCII decimal form

(ack Thomas Otto).

@param[in] value: 16 bit signed integer.
*/

void usart_print_int(int value)
{
	uint8_t i;
	uint8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(USART1, '-');
		value = value * -1;
	}
	if (value == 0) {
		buffer[nr_digits++] = '0';
	} else {
		while (value > 0) {
			buffer[nr_digits++] = "0123456789"[value % 10];
			value /= 10;
		}
	}
	for (i = nr_digits; i > 0; i--) {
		usart_send_blocking(USART1, buffer[i-1]);
	}
}

/*--------------------------------------------------------------------------*/
/* @brief Initialise USART 1.

USART 1 is configured for 38400 baud, no flow control and interrupt.
*/

void usart1_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
	/* Setup GPIO pin GPIO_USART1_TX on GPIO port A for transmit only. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 38400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);
	/* Finally enable the USART. */
	usart_enable(USART1);
}

/*--------------------------------------------------------------------------*/
/* @brief RTC Setup.

The RTC is woken up, cleared, set to use the external low frequency LSE clock
(which must be provided on the board used) and set to prescale at 1Hz out.
The LSE clock appears to be already running.
*/

void rtc_setup(void)
{
	/* Wake up and clear RTC registers using the LSE as clock. */
	/* Set prescaler, using value for 1Hz out. */
	rtc_auto_awake(RCC_LSE, 0x7FFF);

	/* Clear RTC counter - some counts will occur before prescale set. */
	rtc_set_counter_val(0);

	/* Set the Alarm to trigger in interrupt mode on EXTI17 for wakeup */
	nvic_enable_irq(NVIC_RTC_ALARM_IRQ);
	EXTI_IMR |= EXTI17;
	exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);
}

/*--------------------------------------------------------------------------*/
/* @brief EXTI Setup.

This enables the external events on bits 0, 2 and 3 of the ports.
*/

#define EXTI_ENABLES		EXTI0
#define PA_DIGITAL_INPUTS   GPIO0

void exti_setup(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
				  PA_DIGITAL_INPUTS);
	gpio_set(GPIOA, PA_DIGITAL_INPUTS);	/* Pull up */
	exti_select_source(EXTI0, GPIOA);
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	exti_set_trigger(EXTI_ENABLES, EXTI_TRIGGER_RISING);
	EXTI_IMR |= EXTI_ENABLES;
}

/*--------------------------------------------------------------------------*/
/* Interrupt Service Routines */
/*--------------------------------------------------------------------------*/
/* EXT0

Bit 0 of each port used as a pin interrupt.
*/

void exti0_isr(void)
{
	exti_counter++;
	exti_reset_request(EXTI0);
}

/*--------------------------------------------------------------------------*/
/* EXT17/RTC Alarm ISR

The RTC alarm appears as EXTI 17 which must be reset independently of the RTC
alarm flag. Do not reset the latter here as it is needed to guide the main
program to activate regular tasks.
*/

void rtc_alarm_isr(void)
{
	exti_reset_request(EXTI17);
}


