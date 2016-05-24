/*  Defines necessary to specify the processor and board used.

This includes code to match the hardware Arduino calls to the particular
processor and board.

The processor used here is the STM32F103 on the ET-STM32F103 development board.

K. Sarkies, 10 May 2016
*/

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include <stdint.h>

#define HIGH        1
#define LOW         0

/* 72MHz clock rate divided by 8 and 1000 to set ms count period for systick */
#define MS_COUNT    9000

enum pinmodetype {INPUT, OUTPUT, INPUT_PULLUP};

/* STM32F103 definitions. Define pin as pin number in first four bits,
with the port number in the next 4 bits (GPIOA = 0 etc). */
#define DHT_PORT    GPIOA
#define DHT_PIN     1

uint32_t elapsed();
void hardwareSetup(void);
void pinMode(uint8_t pin, enum pinmodetype mode);
void digitalWrite(uint8_t pin, uint8_t setting);
uint8_t digitalRead(uint8_t pin);
uint32_t millis();
void delaySleep(uint32_t delayMs);
void delay(uint32_t delaySec);
void delayMicroseconds(uint16_t delayUs);
void cli(void);
void sei(void);
void usart_print_fixed_point(uint32_t value);
void usart_print_int(int value);
void usart_print_hex(uint16_t value);
void usart_print_string(char *ch);

#endif

