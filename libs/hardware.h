/*  Defines necessary to specify the processor and board used.

This includes code to match the hardware Arduino calls to the particular
processor and board.

The processor used here is the STM32F103 on the ET-STM32F103 development board.

K. Sarkies, 10 May 2016
*/

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include <stdint.h>

#define DEEPSLEEP

#define RTC_SOURCE  RTC

#define HIGH        1
#define LOW         0

/* 72MHz clock rate divided by 8 and 1000 to set ms count period for systick */
#define MS_COUNT    9000

/* Use first I2C channel */
#define I2C_CHANNEL 0

/* DHT Temperature/Humidity sensor */
#define DHT_PORT    GPIOA
#define DHT_PIN     1

/* DAC set of charge limit */
#define DAC_PORT    GPIOA
#define DAC_PIN     5

/* Size of communications receive and transmit buffers. */
#define BUFFER_SIZE 250

/* USART */
#define BAUDRATE        38400

/* Watchdog Timer Timeout Period in ms */
#define IWDG_TIMEOUT_MS 1500

/* Flash. Largest page size compatible with most families used.
(note only STM32F1xx,  STM32F05x have compatible memory organization). */
#define FLASH_PAGE_SIZE 2048

/* Number of A/D converter channels available (STM32F103) */
#define NUM_DEVICES     3
#define NUM_LOADS       2
#define NUM_SOURCES     1
#define NUM_INTERFACES  6
#define NUM_CHANNEL     1 + 2*NUM_INTERFACES

/* A/D Converter Channels
For A/D conversion on the STM32F103RET6 the A/D ports are:
PA 0-7 is ADC 0-7
PB 0-1 is ADC 8-9
PC 0-5 is ADC 10-15 */

#define EXTI_ENABLES        (EXTI0 | EXTI2 | EXTI3)

/* GPIO Port Settings */
#define PA_ANALOGUE_INPUTS          GPIO4 | GPIO6 | GPIO7
#define PA_DIGITAL_INPUTS           GPIO0 | GPIO1 | GPIO2
#define PB_DIGITAL_OUTPUTS          GPIO4 | GPIO5 | GPIO10 | GPIO11 | GPIO12
#define PC_DIGITAL_OUTPUTS          GPIO0 | GPIO1

/* Arduino emulated calls */
enum pinmodetype {INPUT, OUTPUT, INPUT_PULLUP};

uint32_t gpio_port(uint8_t pin);
void pin_mode(uint8_t pin, enum pinmodetype mode);
void digital_write(uint8_t pin, uint8_t setting);
uint8_t digital_read(uint8_t pin);

/* Convenience functions */
void hardware_init(void);
void cli(void);
void sei(void);
uint32_t millis();
void millis_offset(uint32_t offset);
void delay(uint32_t delay_sec);
void delay_microseconds(uint16_t delay_us);
uint32_t get_seconds_count();
void set_seconds_count(uint32_t time);
uint32_t get_delay_count(void);
void set_delay_count(uint32_t time);
void delay_sleep(uint32_t delay_ms);
void flash_read_data(uint32_t *flashBlock, uint8_t *dataBlock, uint16_t size);
uint32_t flash_write_data(uint32_t *flashBlock, uint8_t *dataBlock, uint16_t size);
void comms_enable_tx_interrupt(uint8_t enable);
void peripheral_enable(void);
void peripheral_disable(void);
/* Setup procedures */
void clock_setup(void);
void systick_setup();
void usart1_setup(void);
void timer2_setup(uint32_t period);
void i2c_setup(uint8_t i2c);
void adc_setup(void);
void gpio_setup(void);
void dac_setup(void);
void rtc_setup(void);
void exti_setup(void);

#endif

