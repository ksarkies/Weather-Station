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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include "../libs/buffer.h"
#include "../libs/DHT.h"
#include "../libs/hardware.h"

#define MEASUREMENT_PERIOD 200
#define BUFFER_SIZE 128

/*--------------------------------------------------------------------------*/
/* Global Variables */

bool timerInterruptTriggered;
uint8_t send_buffer[BUFFER_SIZE+3];
uint8_t receive_buffer[BUFFER_SIZE+3];

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

void hardwareSetup(void);
void timer2_setup(uint32_t period);
void usart1_setup(void);
void usart_print_fixed_point(uint32_t value);
void usart_print_int(int value);
void usart_print_hex(uint16_t value);
void usart_print_string(char *ch);

/*--------------------------------------------------------------------------*/

int main(void)
{
    DHT sensor = {DHT_PIN,DHT22,false};
    initDHT(&sensor);
	hardwareSetup();
    systickSetup();
    usart1_setup();
    timer2_setup(MEASUREMENT_PERIOD);
    timerInterruptTriggered = false;

	while (1)
    {
        if (timerInterruptTriggered)
        {
        	gpio_toggle(GPIOB, GPIO8);      /* LED2 on/off. */
            timerInterruptTriggered = false;
            uint32_t temperature = readTemperature(&sensor, false);
            uint32_t humidity = readHumidity(&sensor);
            usart_print_fixed_point(temperature);
            usart_print_string(" ");
            usart_print_fixed_point(humidity);
            usart_print_string("\n\r");
        }
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

/* Set GPIO8-15 (in GPIO port B) to 'output push-pull' for the LEDs. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 |
              GPIO12 | GPIO13 | GPIO14 | GPIO15);
/* All LEDS off */
	gpio_clear(GPIOB, GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
               GPIO14 | GPIO15);
}

/*--------------------------------------------------------------------------*/
/* @brief Initialise USART 1.

USART 1 is configured for 38400 baud, no flow control and interrupt
*/

void usart1_setup(void)
{
/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
/* Setup UART parameters. */
	usart_set_baudrate(USART1, 38400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	/* Enable USART1 receive interrupts. */
	usart_enable_rx_interrupt(USART1);
	usart_disable_tx_interrupt(USART1);
	/* Finally enable the USART. */
	usart_enable(USART1);
}

/*--------------------------------------------------------------------------*/
/* @brief Initialise Timer 2.

Setup timer 2 to run through a period and to interrupt.
*/

void timer2_setup(uint32_t period)
{
	rcc_periph_clock_enable(RCC_TIM2);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_set_priority(NVIC_TIM2_IRQ, 1);
	timer_reset(TIM2);
/* Timer global mode: - No Divider, Alignment edge, Direction up */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM2);
/* Set timer prescaler. 72MHz/1440 => 50000 counts per second. */
	timer_set_prescaler(TIM2, 1440);
/* End timer value. When this is reached an interrupt is generated. */
	timer_set_period(TIM2, period);
/* Update interrupt enable. */
	timer_enable_irq(TIM2, TIM_DIER_UIE);
/* Start timer. */
	timer_enable_counter(TIM2);
}

/*--------------------------------------------------------------------------*/
/* @brief Print out a fixed point value in ASCII decimal form.

Fixed point arithmetic based on 32 bit signed integer of which the first 8 bits
are the fractional part. The integer part is printed first with sign, followed
by the fractional part.

The USART ISR accesses the buffer independently. The USART interrupt must be
re-enabled in the main program after each call to this function.

@param[in] value: 32 bit signed integer as uint32_t.
*/

void usart_print_fixed_point(uint32_t value)
{
    usart_print_int((int) value >> 8);
    buffer_put(send_buffer, '.');

    uint8_t fraction = value & 0xFF;
	if (fraction == 0) buffer_put(send_buffer, '0');
	else while (fraction > 0)
	{
		fraction *= 10;
		buffer_put(send_buffer, "0123456789"[fraction >> 8]);
        fraction &= 0xFF;
	}
}

/*--------------------------------------------------------------------------*/
/* @brief Print out an integer value in ASCII decimal form

(ack Thomas Otto). The USART ISR accesses the buffer independently. The USART
interrupt must be re-enabled in the main program after each call to this
function.

@param[in] value: 16 bit signed integer.
*/

void usart_print_int(int value)
{
	uint8_t i;
	uint8_t nr_digits = 0;
	char buffer[25];

	if (value < 0)
	{
		buffer_put(send_buffer, '-');
		value = value * -1;
	}
	if (value == 0) buffer[nr_digits++] = '0';
	else while (value > 0)
	{
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}
	for (i = nr_digits; i > 0; i--)
	{
		buffer_put(send_buffer, buffer[i-1]);
	}
}

/*--------------------------------------------------------------------------*/
/* @brief Print out a value in ASCII hex form.

The USART ISR accesses the buffer independently. The USART interrupt must be
re-enabled in the main program after each call to this function.

@param[in] value: 16 bit unsigned integer.
*/

void usart_print_hex(uint16_t value)
{
	uint8_t i;
	char buffer[25];

	for (i = 0; i < 4; i++)
	{
		buffer[i] = "0123456789ABCDEF"[value & 0xF];
		value >>= 4;
	}
	for (i = 4; i > 0; i--)
	{
		buffer_put(send_buffer, buffer[i-1]);
	}
	buffer_put(send_buffer, ' ');
}

/*--------------------------------------------------------------------------*/
/* @brief Print a String.
*/

void usart_print_string(char *ch)
{
  	while(*ch)
	{
     	buffer_put(send_buffer, *ch);
     	ch++;
  	}
}

/*--------------------------------------------------------------------------*/
/* Interrupt Service Routines */
/*--------------------------------------------------------------------------*/
/* TIMER

Set a global flag to indicate that the interrupt occurred.
*/

void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_UIF))
        timer_clear_flag(TIM2, TIM_SR_UIF); /* Clear interrrupt flag. */
	timer_get_flag(TIM2, TIM_SR_UIF);	/* Reread to force the previous write */
    timerInterruptTriggered = true;
}

/*--------------------------------------------------------------------------*/
/* USART

Find out what interrupted and get or send data via the FIFO buffers as
appropriate.
*/

void usart1_isr(void)
{
	static uint16_t data;

/* Check if we were called because of RXNE. */
	if (usart_get_flag(USART1,USART_SR_RXNE))
	{
/* If buffer full we'll just drop it */
		buffer_put(receive_buffer, (uint8_t) usart_recv(USART1));
	}
/* Check if we were called because of TXE. */
	if (usart_get_flag(USART1,USART_SR_TXE))
	{
/* If buffer empty, disable the tx interrupt */
		data = buffer_get(send_buffer);
		if ((data & 0xFF00) > 0) usart_disable_tx_interrupt(USART1);
		else usart_send(USART1, (data & 0xFF));
	}
}
/*--------------------------------------------------------------------------*/

