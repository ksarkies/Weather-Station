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

#define BUFFER_SIZE 128

/* 72MHz clock rate divided by 8 and 1000 to set ms count period for systick */
#define MS_COUNT    9000

/* Flash. Largest page size compatible with most families used.
(note only STM32F1xx,  STM32F05x have compatible memory organization). */
#define FLASH_PAGE_SIZE 2048

/* Use first I2C channel */
#define I2C_CHANNEL 0

enum pinmodetype {INPUT, OUTPUT, INPUT_PULLUP};

/* STM32F103 definitions. Define pin as pin number in first four bits,
with the port number in the next 4 bits (GPIOA = 0 etc). */
#define DHT_PORT    GPIOA
#define DHT_PIN     1

/* Arduino emulated calls */
uint32_t gpio_port(uint8_t pin);
void pin_mode(uint8_t pin, enum pinmodetype mode);
void digital_write(uint8_t pin, uint8_t setting);
uint8_t digital_read(uint8_t pin);

/* Convenience functions */
void hardware_init(void);
void cli(void);
void sei(void);
void millis_offset(uint32_t offset);
uint32_t millis();
void delay(uint32_t delay_sec);
void delay_microseconds(uint16_t delay_us);
uint32_t get_seconds_count();
void set_seconds_count(uint32_t time);
void delay_sleep(uint32_t delay_ms);
void flash_read_data(uint32_t *flashBlock, uint8_t *dataBlock, uint16_t size);
uint32_t flash_write_data(uint32_t *flashBlock, uint8_t *dataBlock, uint16_t size);
void comms_enable_tx_interrupt(uint8_t enable);
void peripheral_enable(void);
void peripheral_disable(void);
/* Setup procedures */
void clock_setup(void);
void systick_setup(uint16_t period);
void usart1_setup(void);
void timer2_setup(uint32_t period);
void i2c_setup(uint8_t i2c);
void adc_setup(void);
void gpio_setup(void);
void dac_setup(void);
void rtc_setup(void);
void exti_setup(void);

#endif

