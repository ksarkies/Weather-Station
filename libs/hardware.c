/*  Defines necessary to specify the processor and board used.

This includes code to match the hardware Arduino calls to the particular
processor and board.

The systick timer must be set to 1ms ticks, and timer 2 set to a clock rate of
1 microsecond.

The processor used here is the STM32F103 on the ET-STM32F103 development board.
libopencm3 is the hardware library.

K. Sarkies, 10 May 2016
*/

#include <stdint.h>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include "hardware.h"

#define  _BV(bit) (1 << (bit))

//-----------------------------------------------------------------------------
// Globals
uint32_t systickTime;

//-----------------------------------------------------------------------------
/* @brief   Calls to match Arduino hardware calls to processor
*/
//-----------------------------------------------------------------------------
/*      Derive port number from pin

@param[in] pin: uint8_t. Refers to the Arduino pin used for DHT.
@result uint32_t port specification for the STM32F103
*/

uint32_t gpioPort(uint8_t pin)
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

//-----------------------------------------------------------------------------
/*      Set mode of pin

For libopencm3 STM32F103 the port is derived from the pin parameter which is
the pin number plus the port number times 16, where 0=GPIOA etc.

@param[in] pin: uint8_t. Refers to the Arduino pin used for DHT.
@param[in] mode: pinmodetype. Type of digital input pin.
*/

void pinMode(uint8_t pin, enum pinmodetype mode)
{
    switch (mode)
    {
        case INPUT_PULLUP:
        case INPUT:
            gpio_set_mode(gpioPort(pin), GPIO_MODE_INPUT,
                           GPIO_CNF_INPUT_PULL_UPDOWN, _BV(pin & 0x0F));
            break;
        case OUTPUT:
            gpio_set_mode(gpioPort(pin), GPIO_MODE_OUTPUT_50_MHZ,
                          GPIO_CNF_OUTPUT_PUSHPULL, _BV(pin & 0x0F));
    }
}

//-----------------------------------------------------------------------------
/*      Set Digital Pin value

An output pin is set high or low, while an input pin is set to pullup high or
no pullup.

For libopencm3 STM32F103 the port is derived from the pin parameter which is
the pin number plus the port number times 16, where 0=GPIOA etc.

@param[in] pin: uint8_t. Refers to the Arduino pin used for DHT.
@param[in] setting: uint8_t. HIGH (1) or LOW (0).
*/

inline void digitalWrite(uint8_t pin, uint8_t setting)
{
    gpio_port_write(gpioPort(pin), _BV(setting));
}

//-----------------------------------------------------------------------------
/*      Read Digital Pin value

For libopencm3 STM32F103 the port is derived from the pin parameter which is
the pin number plus the port number times 16, where 0=GPIOA etc.

@param[in] pin: uint8_t. Refers to the Arduino pin used for DHT.
@returns uint8_t. Pin value HIGH (1) or LOW (0).
*/

inline uint8_t digitalRead(uint8_t pin)
{
    return (uint8_t)(gpio_get(gpioPort(pin), _BV(pin & 0x0F)) >> (pin & 0x0F));
}

//-----------------------------------------------------------------------------
/*      Millisecond System Time

@returns uint32_t. Time in milliseconds since rollover or start of counting.
*/

inline uint32_t millis()
{
    return systickTime;
}

//-----------------------------------------------------------------------------
/*      Delay in milliseconds

This function provides a basic delay in milliseconds. This differs from
one system to another in libc. Here it makes use of a timer to define
the time more accurately and independently of the instruction timing.

Global: systickTime taken from the systick interrupt.

@param[in] delayMs: uint16_t. Delay in milliseconds to 65.536 seconds.
*/

void delay(uint16_t delayMs)
{
    uint16_t lastTime = systickTime;
    uint16_t count; 
    for (count = delayMs; count>0; count--)
    {
// As 1ms is an eternity, just spin until the systick timer changes.
        while (lastTime == systickTime);
        lastTime = systickTime;
    }
}

//-----------------------------------------------------------------------------
/*      Delay in microseconds

This function provides a basic delay in microseconds. This differs from
one system to another in libc. Here it makes use of timer2 to define
the time more accurately and independently of the instruction timing.

Accuracy depends on the execution time in each loop being less than a
microsecond. Interrupts should be disabled while this is executing.

@note As the timer 2 is 16 bit for the STM32F103, this will fail if delayUs is
greater than 32768.

@param[in] delayUs: uint32_t. Delay in microseconds up to 32768.
*/

 void delayMicroseconds(uint32_t delayUs)
{
    uint16_t initialTime = timer_get_counter(TIM2);
    uint16_t finalTime = initialTime+delayUs; 
    if (finalTime > initialTime)
        while (timer_get_counter(TIM2) < finalTime);
    else
        while ((timer_get_counter(TIM2) - delayUs) < initialTime);
}

/*--------------------------------------------------------------------------*/
/** @brief Systick Setup

Setup SysTick Timer for 1 millisecond interrupts, also enables Systick and
Systick-Interrupt. Timing is defined for a 72MHz system clock.
*/

void systickSetup()
{
/* 72MHz / 8 => 9,000,000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
/* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(MS_COUNT);

    systick_interrupt_enable();

/* Start counting. */
    systick_counter_enable();
}

/*--------------------------------------------------------------------------*/
/** @brief Timer 2 Setup for Microsecond Timing

Timer 2 is set to run at a frequency that allows times to be measured at
microsecond accuracy. Timing is defined for a 72MHz system clock.
Interrupts are not used; the counter simply runs continuously.
*/

void timer2Setup(void)
{
	rcc_periph_clock_enable(RCC_TIM2);
	timer_reset(TIM2);
/* Timer global mode: - No Divider, Alignment edge, Direction up */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM2);
/* Set timer prescaler. 72MHz/72 => 1,000,000 counts per second. */
	timer_set_prescaler(TIM2, 73);
/* Start timer. */
	timer_enable_counter(TIM2);
}

/*-----------------------------------------------------------*/
/** @Brief Systick Interrupt Handler

Just update some counters for general use.
*/

void sys_tick_handler(void)
{
    systickTime++;
/* updated every 1s if systick is used for real-time clock. */
    static uint16_t cnttime=0;
    cnttime++;
    if (cnttime >= 1000)
    {
        cnttime = 0;
//        updateTimeCount();
    }
}

/*-----------------------------------------------------------*/
/** @Brief Disable Global interrupts
*/

inline void cli(void)
{
    cm_disable_interrupts();
}

/*-----------------------------------------------------------*/
/** @Brief Enable Global interrupts
*/

inline void sei(void)
{
    cm_enable_interrupts();
}

//-----------------------------------------------------------------------------

