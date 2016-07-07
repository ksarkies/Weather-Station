#include <unistd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/usart.h>
#include "buffer.h"

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

static void usart_print_string(char *ch);
static void usart1_setup(void);
static void rtc_setup(void);

/*--------------------------------------------------------------------------*/
int main(void)
{
    uint32_t time = 0;
    uint32_t previousTime = 0;

/* Set the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	usart1_setup();
    usart_print_string("RTC Alarm Test\n\r");
	rtc_setup();
	rtc_set_alarm_time(10);
    usart_print_string("RTC Setup Complete\n\r");

/* Set to stop mode and wait for RTC interrupt. */
	while (1)
    {
/* Put in a delay to allow USART to finish */
        while (time == previousTime)
        {
            previousTime = time;
            time = rtc_get_counter_val();
        }
        previousTime = time;

/* Set sleep mode and sleep */
        pwr_voltage_regulator_low_power_in_stop();
        pwr_set_stop_mode();                /* This only sets the power down */
        exti_reset_request(0xFFFFF);        /* Clear the whole bloody lot */
//        SCB_SCR |= SCB_SCR_SLEEPDEEP;       /* Set the deep sleep mode */
        asm volatile("wfe");

/* Identify */
/* Put in a delay to allow wakeup to finish */
        while (time == previousTime)
        {
            previousTime = time;
            time = rtc_get_counter_val();
        }
        previousTime = time;

/* Repeat setup as clocks may have been reset */
	    rcc_clock_setup_in_hse_8mhz_out_72mhz();
	    usart1_setup();
        usart_print_string("Woken?\r\n");
	    rtc_setup();
	    rtc_set_alarm_time(10);
	}

	return 0;
}
/*--------------------------------------------------------------------------*/
/* @brief Print a String.
*/

void usart_print_string(char *ch)
{
  	while(*ch)
	{
     	usart_send_blocking(USART1, (*(ch++) & 0xFF));
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
	rtc_awake_from_off(RCC_LSE);
	rtc_set_prescale_val(0x7FFF);

    rtc_set_counter_val(0);     /* Didn't reset in rtc_awake_from_off ?? */

/* Set the Alarm to trigger in event mode on EXTI17 for wakeup */
    exti_enable_request(EXTI17);
    exti_set_trigger(EXTI17,EXTI_TRIGGER_RISING);
}


