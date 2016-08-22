/*  
@mainpage Defines necessary to specify the processor and board used.
@version 0.0
@author Ken Sarkies (www.jiggerjuice.info)
@date 22 May 2016

This includes code to match the hardware Arduino calls to the particular
processor and board. The reason for doing this is to minimize changes to
Arduino code taken from sensor support websites.

Some hardware setups are provided here as they provide general timing and
communication functionality. These include:

- USART1 for communication.
- TIMER2 for microsecond delays. This must be set to a period of 0xFFFF to
   ensure that the time count wraps around. It must be set to a clock rate of
   1 microsecond. The DTH22 sensor requires microsecond level bit bashing
   interpretation of signals.
- Systick for millisecond delays. It must be set to 1ms ticks.

The processor used here is the STM32F103 on the ET-STM32F103 development board.
The hardware library is libopencm3.

K. Sarkies, 10 May 2016
*/

#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
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
#include <libopencm3/cm3/nvic.h>
#include "buffer.h"
#include "DHT.h"
#include "i2c.h"
#include "hardware.h"

#define  _BV(bit) (1 << (bit))

/*--------------------------------------------------------------------------*/
/* Globals */

volatile uint32_t time2Tick_counter;
volatile uint32_t systick_time;
volatile uint32_t last_reload_value;
static uint8_t send_buffer[BUFFER_SIZE+3];
static uint8_t receive_buffer[BUFFER_SIZE+3];

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

/*--------------------------------------------------------------------------*/
/* @brief   Calls to emulate Arduino hardware calls             */
/*--------------------------------------------------------------------------*/
/*      Derive port number from pin

@param[in] pin: uint8_t. Refers to the Arduino pin used for DHT.
@result uint32_t port specification for the STM32F103
*/

uint32_t gpio_port(uint8_t pin)
{
    switch ((pin >> 4) & 0x0F)
    {
        case 0: return GPIOA;
        case 1: return GPIOB;
        case 2: return GPIOC;
        case 3: return GPIOD;
        case 4: return GPIOE;
        case 5: return GPIOF;
        case 6: return GPIOG;
    }
    return 0;
}

/*--------------------------------------------------------------------------*/
/*      Set mode of pin

For libopencm3 STM32F103 the port is derived from the pin parameter which is
the pin number plus the port number times 16, where 0=GPIOA etc.

@param[in] pin: uint8_t. Refers to the Arduino pin used for DHT.
@param[in] mode: pinmodetype. Type of digital input pin.
*/

void pin_mode(uint8_t pin, enum pinmodetype mode)
{
    switch (mode)
    {
        case INPUT_PULLUP:
            gpio_set_mode(gpio_port(pin), GPIO_MODE_INPUT,
                           GPIO_CNF_INPUT_PULL_UPDOWN, _BV(pin & 0x0F));
            break;
        case INPUT:
            gpio_set_mode(gpio_port(pin), GPIO_MODE_INPUT,
                           GPIO_CNF_INPUT_FLOAT, _BV(pin & 0x0F));
            break;
        case OUTPUT:
            gpio_set_mode(gpio_port(pin), GPIO_MODE_OUTPUT_50_MHZ,
                          GPIO_CNF_OUTPUT_PUSHPULL, _BV(pin & 0x0F));
    }
}

/*--------------------------------------------------------------------------*/
/*      Set Digital Pin value

An output pin is set high or low, while an input pin is set to pullup or
no pull down.

For libopencm3 STM32F103 the port is derived from the pin parameter which is
the pin number plus the port number times 16, where 0=GPIOA etc.

@param[in] pin: uint8_t. Refers to the Arduino pin used for DHT.
@param[in] setting: uint8_t. HIGH (1) or LOW (0). Only bit 0 is used.
*/

void digital_write(uint8_t pin, uint8_t setting)
{
    if ((setting & 0x01) == HIGH)
        gpio_set(gpio_port(pin), _BV(pin & 0x0F));
    else
        gpio_clear(gpio_port(pin), _BV(pin & 0x0F));
}

/*--------------------------------------------------------------------------*/
/*      Read Digital Pin value

For libopencm3 STM32F103 the port is derived from the pin parameter which is
the pin number plus the port number times 16, where 0=GPIOA etc.

@param[in] pin: uint8_t. Refers to the Arduino pin used for DHT.
@returns uint8_t. Pin value HIGH (1) or LOW (0).
*/

uint8_t digital_read(uint8_t pin)
{
    return (uint8_t)(gpio_get(gpio_port(pin), _BV(pin & 0x0F)) >> (pin & 0x0F));
}

/*--------------------------------------------------------------------------*/
/** @Brief Disable Global interrupts
*/

void cli(void)
{
    cm_disable_interrupts();
}

/*--------------------------------------------------------------------------*/
/** @Brief Enable Global interrupts
*/

void sei(void)
{
    cm_enable_interrupts();
}

/*--------------------------------------------------------------------------*/
/*      Millisecond System Time Offset

The milliseconds timer is incremented by a fixed amount. This is useful if the
systick timer stops running for a period, such as during sleep.

@param[in] uint32_t. Time offset in milliseconds.
*/

void millis_offset(uint32_t offset)
{
	last_reload_value += offset * MS_COUNT;
}

/*--------------------------------------------------------------------------*/
/*      Millisecond System Time

The systick time is updated only at the end of a systick timer countdown to 0.
Compute the actual time by adding in the time elapsed since the last event.

@returns uint32_t. Time in milliseconds since rollover or start of counting.
*/

uint32_t millis()
{
    cli();
    uint32_t elapsed_time = systick_time
                           + (last_reload_value - systick_get_value())/MS_COUNT;
    sei();
    return elapsed_time;
}

/*--------------------------------------------------------------------------*/
/* @brief Blocking Delay in Milliseconds

This function provides a basic blocking delay in milliseconds. This differs from
one system to another in libc. Here it makes use of the millis() function to
define the time more accurately and independently of the instruction timing.

@param[in] delay_ms: uint16_t. Delay in milliseconds.
*/

void delay(uint32_t delay_ms)
{
    uint16_t count; 
    for (count = delay_ms; count>0; count--)
    {
        uint32_t last_time = millis();
// As 1ms is an eternity, just spin until the timer changes.
        while (last_time == millis());
    }
}

/*--------------------------------------------------------------------------*/
/* @brief Blocking Delay in Microseconds

This function provides a basic delay in microseconds. This differs from
one system to another in libc. Here it makes use of timer2 to define
the time more accurately and independently of the instruction timing.

NOTE: timer2 must be set to a 1 microsecond clock.

Accuracy depends on the execution time in each loop being less than a
microsecond. Interrupts should be disabled while this is executing.

@note As the timer 2 is 16 bit for the STM32F103, this will fail if delay_us is
greater than 65565.

@param[in] delay_us: uint32_t. Delay in microseconds up to 65565.
*/

void delay_microseconds(uint16_t delay_us)
{
    uint16_t initial_time = timer_get_counter(TIM2);
    uint16_t final_time = initial_time + delay_us;
/* With rollover, count up to rollover first, then continue to final time */
    if (final_time < initial_time)
        while (timer_get_counter(TIM2) > initial_time);
    while (timer_get_counter(TIM2) < final_time);
}

/*--------------------------------------------------------------------------*/
/* @brief Sleeping Delay in Milliseconds

This function provides a basic sleeping delay in milliseconds. The systick
timer defines the time more accurately and independently of the instruction
timing.

The CM3 systick timer is a 24 bits downcounter that can reload to a specified
start value.

Global: systick_time taken from the systick interrupt.

@param[in] delay_ms: uint32_t. Delay in milliseconds to 65,535.
*/

void delay_sleep(uint32_t delay_ms)
{
#define DIVISOR 0xFFFFFF/MS_COUNT

/* If delay_ms is more than 24 bits, split into number of 24 bit full cycles and
the remainder for the last cycle */
    uint16_t cycles = 1+delay_ms/DIVISOR;
    uint32_t last_cycle_count = delay_ms % DIVISOR;
    uint32_t count;
/* Sleep state will awake on any interrupt. Wait for systick before deciding
whether to quit this loop. */
    uint16_t i = 0;
    for (i=0; i < cycles; i++)
    {
        if (i == 0) count = last_cycle_count;
        else count = DIVISOR;
        systick_setup(count);        // Set systick to interrupt after given delay.
        while (! systick_get_countflag())
            asm volatile("wfi");    // sleep until systick fires.
    }
}

/*--------------------------------------------------------------------------*/
/*      Return a character from the receive buffer

@returns uint16_t. Lower 8 bits has character. Upper 8 bits has 0x100 if no
                   character has been received.
*/

uint32_t check_receive_buffer(void)
{
    return buffer_get(receive_buffer);
}

/*--------------------------------------------------------------------------*/
/* @brief Print out a fixed point value in ASCII decimal form.

Fixed point arithmetic based on 32 bit signed integer of which the first 8 bits
are the fractional part. The integer part is printed first with sign, followed
by the fractional part rounded up to a specified precision.

The USART ISR accesses the buffer independently. The USART interrupt must be
re-enabled in the main program after each call to this function.

@param[in] value: 32 bit signed integer as uint32_t.
*/

#define PRECISION 4

void usart_print_fixed_point(uint32_t value)
{
    int i = 0;
    usart_print_int((int) value >> 8);
    buffer_put(send_buffer, '.');
    if ((value & 0x80000000) > 0) value = -value;
    uint16_t fraction = value & 0xFF;
	if (fraction == 0) buffer_put(send_buffer, '0');
	else while (fraction > 0)
	{
		fraction *= 10;
        if ((++i >= PRECISION) && ((fraction & 0xFF) > 128)) fraction += 256;
		buffer_put(send_buffer, "0123456789"[fraction >> 8]);
        if (i >= PRECISION) break;
        fraction &= 0xFF;
	}
	usart_enable_tx_interrupt(USART1);
}

/*--------------------------------------------------------------------------*/
/* @brief Print out an integer value in ASCII decimal form

(ack Thomas Otto). The USART ISR accesses the buffer independently. The USART
interrupt must be re-enabled in the main program after each call to this
function.

@param[in] value: 64 bit signed integer.
*/

void usart_print_int(int64_t value)
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
	usart_enable_tx_interrupt(USART1);
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
	usart_enable_tx_interrupt(USART1);
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
	usart_enable_tx_interrupt(USART1);
}

/*--------------------------------------------------------------------------*/
/** @brief Initialise Systick

Setup Systick Timer for interrupts, enable clock and Systick-Interrupt.
Timing is defined for a 72MHz system clock.

The AHB clock can be divided by 8 to give a 9MHz clock. The period can be up
to 24 bits giving a 1864 ms maximum count. If the period is set beyond this
then a shortened period will occur.

MS_COUNT provides the translation between the systick clock and a millisecond
clock, thus allowing time estimates to be made in milliseconds.

@param[in] period: uint16_t period before interrupt in ms, up to 1864.
*/

void systick_setup(uint16_t period)
{
    systick_counter_disable();
/* 72MHz / 8 => 9,000,000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
/* 9000000/9000 = 1000 overflows per second - every period ms one interrupt */
/* Systick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(MS_COUNT*(uint32_t)period - 1);
    systick_interrupt_enable();
/* Start counting. */
    systick_counter_enable();
    systick_clear();
    systick_get_countflag();        // Clear the flag
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
This must have a 1 microsecond clock for the various delay and time functions.
Its counter is 16 bit so only periods up to 65 milliseconds can be handled.
For longer periods use the systick counter.

@param[in] period: uint32_t tick period in 1 microsecond cycles.
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
/* Set timer prescaler. 72MHz/72 => 1,000,000 counts per second. */
	timer_set_prescaler(TIM2, 72);
/* End timer value. When this is reached an interrupt is generated. */
	timer_set_period(TIM2, period);
/* Update interrupt enable. */
	timer_enable_irq(TIM2, TIM_DIER_UIE);
/* Start timer. */
	timer_enable_counter(TIM2);
}

/*--------------------------------------------------------------------------*/
/* @brief Peripheral Disables.

This turns off power to all peripherals to reduce power drain during sleep.
RTC and EXTI need to remain on.
*/

void peripheral_disable(void)
{
#ifdef DEEPSLEEP
    rcc_periph_clock_disable(RCC_AFIO);
    rcc_periph_clock_disable(RCC_GPIOA);
    rcc_periph_clock_disable(RCC_USART1);
#endif
    rcc_periph_clock_disable(RCC_GPIOB);
    rcc_periph_clock_disable(RCC_GPIOC);
    rcc_periph_clock_disable(RCC_ADC1);
	rcc_periph_clock_disable(RCC_DAC);
	rcc_periph_clock_disable(RCC_I2C1);
	rcc_periph_clock_disable(RCC_TIM2);
    adc_power_off(ADC1);
    dac_disable(CHANNEL_D);
}

/*--------------------------------------------------------------------------*/
/* @brief Peripheral Enables.

This turns on power to all peripherals needed.
*/

void peripheral_enable(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_DAC);
	rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_TIM2);
    adc_power_on(ADC1);
    dac_enable(CHANNEL_2);
}

/*--------------------------------------------------------------------------*/
/* @brief GPIO Setup.

This sets the clocks for the GPIO and AFIO ports, and sets the LEDs on
PB8 - PB15 to output and cleared.

Sets the two MOSFET control outputs to low, which disables the measurement and
enables the charging.
*/

void gpio_setup(void)
{
/* Enable GPIOA, GPIOB and GPIOC clocks and alternate functions. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);

/* Set GPIO8-15 (in GPIO port B) to 'output push-pull' for the LEDs. */
/*    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10 | GPIO11 |
              GPIO12 | GPIO13 | GPIO14 | GPIO15); */
/* All LEDS off */
/*    gpio_clear(GPIOB, GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
               GPIO14 | GPIO15); */

/* Set GPIO2, GPIO5 (in GPIO port B) to 'output push-pull' for the MOSFETs. */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO2 | GPIO5);
/* All MOSFET controls off. */
    gpio_clear(GPIOB, GPIO2 | GPIO5);
}

/*--------------------------------------------------------------------------*/
/* @brief ADC Setup.

ADC1 is turned on and calibrated.
*/

#define ADC_INPUTS  (GPIO4 | GPIO6 | GPIO7)

void adc_setup(void)
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

void dac_setup(void)
{
/* Set port PA5 for DAC1 to 'alternate function'. Output driver mode is ignored. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		          GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5);
/* Enable the DAC clock on APB1 */
	rcc_periph_clock_enable(RCC_DAC);
/* Setup the DAC, software trigger source. Assume the DAC has
woken up by the time the first interrupt occurs */
	dac_enable(CHANNEL_2);
	dac_trigger_enable(CHANNEL_2);
	dac_set_trigger_source(DAC_CR_TSEL1_SW);
}

/*--------------------------------------------------------------------------*/
/* @brief Setup I2C1

The clocks and GPIO settings are established.
*/

void i2c1_setup(void)
{
/* Enable clocks for I2C1 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);
/* Set alternate functions for the SCL and SDA pins of I2C1. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C1_SCL | GPIO_I2C1_SDA);
    i2c_initialise(I2C1);
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
	rtc_auto_awake(RCC_LSE,0x7FFF);

	/* Clear the RTC counter - some counts will occur before prescale is set. */
	rtc_set_counter_val(0);

	/* Set the Alarm to trigger in interrupt mode on EXTI17 for wakeup */
	nvic_enable_irq(NVIC_RTC_ALARM_IRQ);
	EXTI_IMR |= EXTI17;
	exti_set_trigger(EXTI17,EXTI_TRIGGER_RISING);
}

/*--------------------------------------------------------------------------*/
/* @brief EXTI Setup.

This enables the external interrupts on bits 0, 2 and 3 of the ports.
*/

#define EXTI_ENABLES        (EXTI0 | EXTI2 | EXTI3)
#define PA_DIGITAL_INPUTS   (GPIO0 | GPIO2 | GPIO3)

void exti_setup(void)
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

/*--------------------------------------------------------------------------*/
/** @Brief Systick Interrupt Handler

Just update a counter for extended time use. As it is only updated on an
interrupt occurring, the last reload value that resulted in the countdown
is read and used to update the time before being reset to the existing reload
value.

Globals: systick_time: running count of milliseconds from an arbitrary time.
         last_reload_value: reload value used for the countdown just finished.
*/

void systick_handler(void)
{
//    systick_time++;
    systick_time += last_reload_value/MS_COUNT;
    last_reload_value = (systick_get_reload() & 0xFFFFFF);
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
/* TIMER

The tick counter is incremented on each interrupt.
*/

void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_UIF))
        timer_clear_flag(TIM2, TIM_SR_UIF); /* Clear interrrupt flag. */
	timer_get_flag(TIM2, TIM_SR_UIF);	/* Reread to force the previous write */
    time2Tick_counter++;
}

/*--------------------------------------------------------------------------*/

