/**
@mainpage Weather Station Firmware for ARM based Remote Unit
@version 0.0.0
@author Ken Sarkies (www.jiggerjuice.info)
@date 04 July 2016

This initial development version uses the ET-ARM STAMP board. The port
allocations are:

- PA0: Rainwater gauge. Tipping bucket with reed switch.
- PA1: Temperature and Humidity. Freetronics DHT22.
- PA2: Wind Speed. Reed switch.
- PA3: Wind Direction. Reed switch.
- PA4: Solar Radiance (scale dependent on the panel material). Suntech-STP005S12-5W.
- Air Pressure. Freetronics MS-5637-02BA03. I2C-1 on PB6 for SCL and PB7 for SDA.

For battery charging the following are needed:
- PA6 to measure battery voltage on ADC12-IN5.
- PA7 to measure battery current on ADC12-IN6.
- PB2 to switch the panel to charge the battery.
- PB5 to switch the panel to the solar radiance current measurement circuit.

USART1 is on PA8-PA12 with Tx on PA9 and Rx on PA10.

NOTE: The battery must be in place otherwise when the charger is disabled by
the program the processor will power off.

NOTE: the test ET-STM32 STAMP board has a faulty PB4 and PB3.
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
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include "../libs/buffer.h"
#include "../libs/DHT.h"
#include "../libs/hardware.h"

/* 2 second period in milliseconds */
#define MEASUREMENT_PERIOD      2000
#define RADIANCE_SETTLE_TIME     100
/* Resistor values times 10 */
#define BATTERY_SENSE             12        
#define RADIANCE_SENSE            10       
/* Amplification of current sense voltage */
#define I_AMP                    110 
/* Amplification of battery voltage */
#define V_AMP                    259 

/* Fixed point for battery charge limit for Diamec batteries (7.2V to 7.5V). */
#define CHARGE_LIMIT       72*256/10
/* Fixed point for battery float limit for Diamec batteries (6.5V to 6.8V). */
#define FLOAT_LIMIT        68*256/10

/*--------------------------------------------------------------------------*/
/* Global Variables */

static uint32_t rainfall;               /* Counter of interrupts from sensor */
static uint32_t windSpeed;              /* Counter of interrupts from sensor */
static uint8_t chargerIsActive;         /* Existing state of charger */

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

static void enableRadianceMeasurement(void);
static void disableRadianceMeasurement(void);
static void enableCharging(void);
static void disableCharging(void);
static uint8_t chargerActive(void);
static void peripheralEnable(void);
static void peripheralDisable(void);
static void peripheralSetup(void);
static void hardwareSetup(void);
static void i2c1Setup(void);
static void adcSetup(void);
static void gpioSetup(void);
static void dacSetup(void);
static void extiSetup(void);

/*--------------------------------------------------------------------------*/

int main(void)
{
	hardwareSetup();
    chargerIsActive = chargerActive();
    uint8_t channel[1];                 /* Channel for A/D conversion */
    uint16_t chargeLimit = 0;           /* voltage limit for charging battery */
    uint8_t i=0;
    DHT sensorDHT = {DHT_PIN,DHT22,false};
    initDHT(&sensorDHT);

	for (;;)
    {

/* Read and send Battery Voltage. */

        uint16_t voltageRaw = 0;
        channel[0] = ADC_CHANNEL6;          /* channel 6 battery voltage */
	    adc_set_regular_sequence(ADC1, 1, channel);
        for (i=0; i<16; i++)                /* Average over 16 readings */
        {
            adc_start_conversion_direct(ADC1);
            while (! adc_eoc(ADC1));
            voltageRaw += adc_read_regular(ADC1);
            delay(1);
        }
/* Voltage times 256 for fixed point scaling, with sense resistor (x10)
and amplification (x100) will give results in volts. */
        uint32_t voltage = ((voltageRaw>>4)*256*V_AMP)/(1241*100);/* Volts */
        usart_print_string("dV,");
        usart_print_fixed_point(voltage);
        usart_print_string("\n\r");

/* Read and send Battery Current. */

        uint32_t currentRaw = 0;
        channel[0] = ADC_CHANNEL7;          /* channel 7 battery current */
	    adc_set_regular_sequence(ADC1, 1, channel);
        for (i=0; i<16; i++)                /* Average over 16 readings */
        {
            adc_start_conversion_direct(ADC1);
            while (! adc_eoc(ADC1));
            currentRaw += adc_read_regular(ADC1);
        }
/* Current in mA times 256 for fixed point scaling, with sense resistor (x10)
and current amplification (x10) and scale back to average 16 readings.
Note order of computations to avoid 32 bit overflow. */
        uint32_t current = 
                ((currentRaw*BATTERY_SENSE*1000)/(1241*I_AMP))*16;  /* mA */
        usart_print_string("dI,");
        usart_print_fixed_point(current);
        usart_print_string("\n\r");

/* Battery Charge Limit Setting */

        chargeLimit = 4000;                 /* temporary for testing */
        dac_load_data_buffer_single(chargeLimit, RIGHT12, CHANNEL_2);
        dac_software_trigger(CHANNEL_2);

/* Control Battery Charging.
This cycles between the absorption voltage limit and the float voltage limit. */

        if (voltage > CHARGE_LIMIT)
        {
            disableCharging();              /* Turn off battery charging */
        }
        if (voltage < FLOAT_LIMIT)
        {
            enableCharging();               /* Turn on battery charging */
        }

/* Read and send Temperature and Humidity from the DTH22. */
        uint32_t temperature = readTemperature(&sensorDHT, false);
        uint32_t humidity = readHumidity(&sensorDHT);
        usart_print_string("dT,");
        usart_print_fixed_point(temperature);
        usart_print_string("\n\r");
        usart_print_string("dH,");
        usart_print_fixed_point(humidity);
        usart_print_string("\n\r");

/* Read and send temperature and barometric pressure over i2c. */

/* Read and send solar panel current. Use simple polling of the ADC. */

        enableRadianceMeasurement();
        uint32_t radianceRaw = 0;
        channel[0] = ADC_CHANNEL4;          /* channel 4 radiance */
	    adc_set_regular_sequence(ADC1, 1, channel);
        for (i=0; i<16; i++)                /* Average over 16 readings */
        {
            adc_start_conversion_direct(ADC1);
            while (! adc_eoc(ADC1));
            radianceRaw += adc_read_regular(ADC1);
        }
        disableRadianceMeasurement();       /* Puts charging back on */
/* Current in mA times 256 for fixed point scaling, with sense resistor (x10)
and current amplification (x10) and scale back to average 16 readings.
Note order of computations to avoid 32 bit overflow. */
        uint32_t radiance =
                ((radianceRaw*RADIANCE_SENSE*1000)/(1241*I_AMP))*16;/* mA */
        usart_print_string("dL,");
        usart_print_fixed_point(radiance);
        usart_print_string("\n\r");

/* Send rain gauge count. */

        usart_print_string("dR,");
        usart_print_int(rainfall);
        usart_print_string("\n\r");
        cli();
        rainfall = 0;
        sei();

/* Send wind speed count. */

        usart_print_string("dS,");
        usart_print_int(windSpeed);
        usart_print_string("\n\r");
        cli();
        windSpeed = 0;
        sei();

/* Snooze for a while */
        delay(20);                      /* Wait for USART to complete */
        peripheralDisable();
        delaySleep(MEASUREMENT_PERIOD);
        peripheralEnable();
        delay(50);
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
/* @brief Enable measurement

The measurement MOSFET must be turned on and the charging must be turned off.
Delay for a while to allow settling of currents in the sense resistor.
*/

void enableRadianceMeasurement(void)
{
    chargerIsActive = chargerActive();
    disableCharging();                  /* turn off charger */
    gpio_set(GPIOB, GPIO2);             /* Turn on measurement switch */
    delay(RADIANCE_SETTLE_TIME);
}

/*--------------------------------------------------------------------------*/
/* @brief Disable measurement

The measurement MOSFET must be turned off and the charging must be restored
if it was in a charging state at the time of measurement.
*/

void disableRadianceMeasurement(void)
{
    gpio_clear(GPIOB, GPIO2);           /* Turn off measurement switch */
    if (chargerIsActive) enableCharging();
    else disableCharging();
}

/*--------------------------------------------------------------------------*/
/* @brief Enable Charging

Output must be low to turn on the MOSFET.
*/

void enableCharging(void)
{
    gpio_clear(GPIOB, GPIO5);
}

/*--------------------------------------------------------------------------*/
/* @brief Disable Charging

Output must be high to turn off the MOSFET.
*/

void disableCharging(void)
{
    gpio_set(GPIOB, GPIO5);
}

/*--------------------------------------------------------------------------*/
/* @brief Return State of Charging

The measurement MOSFET must be turned on and the charging must be turned off.
Delay for a while to allow settling of currents in the sense resistor.

@returns uint8_t: true = charging active, false = charging inactive.
*/

uint8_t chargerActive(void)
{
    return (gpio_get(GPIOB, GPIO5) == 0);
}

/*--------------------------------------------------------------------------*/
/* @brief      Hardware specific settings  */
/*--------------------------------------------------------------------------*/
/* @brief Hardware Setup.

*/

void hardwareSetup(void)
{
/* Set the clock to 72MHz from the 8MHz external crystal */

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

//    systickSetup(1);            // Set systick to interrupt after 1 millisecond.
    systickSetup(1000);         // Set systick to interrupt after 1 second.
    peripheralSetup();
    sei();
}

/*--------------------------------------------------------------------------*/
/* @brief Peripheral Disables.

This turns off power to all peripherals to reduce power drain during sleep.
Systick and EXTI need to remain on.
*/

void peripheralSetup(void)
{
    gpioSetup();
    usart1Setup();
    timer2Setup(0xFFFF);
    adcSetup();
    dacSetup();
    i2c1Setup();
    extiSetup();
}

/*--------------------------------------------------------------------------*/
/* @brief Peripheral Disables.

This turns off power to all peripherals to reduce power drain during sleep.
Systick and EXTI need to remain on.
*/

void peripheralDisable(void)
{
    rcc_periph_clock_disable(RCC_AFIO);
    rcc_periph_clock_disable(RCC_GPIOA);
    rcc_periph_clock_disable(RCC_GPIOB);
    rcc_periph_clock_disable(RCC_GPIOC);
    rcc_periph_clock_disable(RCC_ADC1);
	rcc_periph_clock_disable(RCC_DAC);
	rcc_periph_clock_disable(RCC_I2C1);
    rcc_periph_clock_disable(RCC_USART1);
	rcc_periph_clock_disable(RCC_TIM2);
    adc_power_off(ADC1);
    dac_disable(CHANNEL_D);
}

/*--------------------------------------------------------------------------*/
/* @brief Peripheral Enables.

This turns on power to all peripherals needed.
*/

void peripheralEnable(void)
{
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

void gpioSetup(void)
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

void adcSetup(void)
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

void dacSetup(void)
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

void i2c1Setup(void)
{
/* Enable clocks for I2C1 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);
/* Set alternate functions for the SCL and SDA pins of I2C1. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C1_SCL | GPIO_I2C1_SDA);
}

/*--------------------------------------------------------------------------*/
/* @brief EXTI Setup.

This enables the external interrupts on bits 0, 2 and 3 of the ports.
*/

#define EXTI_ENABLES        (EXTI0 | EXTI2 | EXTI3)
#define PA_DIGITAL_INPUTS   (GPIO0 | GPIO2 | GPIO3)

void extiSetup(void)
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
/* EXT0

Bit 0 of each port used as a pin interrupt. This is for rainfall.
*/

void exti0_isr(void)
{
	gpio_toggle(GPIOB, GPIO10);      /* LED3 on/off. */
    rainfall++;
    exti_reset_request(EXTI0);
}

/*--------------------------------------------------------------------------*/
/* EXT2

Bit 2 of each port used as a pin interrupt. This is for wind speed.
*/

void exti2_isr(void)
{
	gpio_toggle(GPIOB, GPIO11);      /* LED4 on/off. */
    windSpeed++;
    exti_reset_request(EXTI2);
}

/*--------------------------------------------------------------------------*/
/* EXT3

Bit 3 of each port used as a pin interrupt. This is for wind direction.
*/

void exti3_isr(void)
{
	gpio_toggle(GPIOB, GPIO12);      /* LED5 on/off. */
    exti_reset_request(EXTI3);
}

/*--------------------------------------------------------------------------*/

