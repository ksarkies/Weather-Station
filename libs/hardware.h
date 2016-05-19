/*  Defines necessary to specify the processor and board used.

This includes code to match the hardware Arduino calls to the particular
processor and board.

The processor used here is the STM32F103 on the ET-STM32F103 development board.

K. Sarkies, 10 May 2016
*/

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define HIGH        1
#define LOW         0

/* 72MHz clock rate divided by 8 and 1000 to set ms count period for systick */
#define MS_COUNT    8999

enum pinmodetype {INPUT, OUTPUT, INPUT_PULLUP};

/* STM32F103 definitions */
#define DHT_PORT    GPIOA
#define DHT_PIN     1

void digitalWrite(uint8_t pin, uint8_t setting);
uint8_t digitalRead(uint8_t pin);
uint32_t millis();
void delay(uint16_t delayMs);
void delayMicroseconds(uint32_t delayUs);
void systickSetup();
void timer2Setup(void);
void cli(void);
void sei(void);
void pinMode(uint8_t pin, enum pinmodetype mode);

#endif

