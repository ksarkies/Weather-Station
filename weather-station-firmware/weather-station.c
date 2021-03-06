/**
@mainpage Weather Station Firmware for ARM based Remote Unit
@version 0.0.0
@author Ken Sarkies (www.jiggerjuice.info)
@date 04 July 2016

The program will place the processor into sleep mode for a given period of time
measured by the RTC alarm setting, which is the only clock still running in the
deep sleep (stop) mode. When expired, a scan of peripherals is made and the
results transmitted and recorded. During sleep, counting peripherals such as
rainfall and wind speed will temporarily wake the processor until the ISR is
complete and it will be placed back to sleep.

If the processor is placed into the lowest power stop mode, it cannot be
contacted by USART. In order to allow access by the base station, the processor
is first placed into sleep mode for a short time before being placed into stop
mode.

This initial development version uses the ET-ARM STAMP board. The port
allocations are:

- PA0: Rainwater gauge. Tipping bucket with reed switch.
- PA1: Temperature and Humidity. Freetronics DHT22.
- PA2: Wind Speed. Reed switch.
- PA3: Wind Direction. Reed switch.
- PA4: Solar Radiance (scale dependent on the panel material).
       Suntech-STP005S12-5W.
- Air Pressure. Freetronics MS-5637-02BA03. I2C-1 on PB6 for SCL, PB7 for SDA.

For battery charging the following are needed:
- PA6 to measure battery voltage on ADC12-IN5.
- PA7 to measure battery current on ADC12-IN6.
- PB2 to switch the panel to charge the battery.
- PB5 to switch the panel to the solar radiance current measurement circuit.

USART1 is on PA8-PA12 with Tx on PA9 and Rx on PA10.

For Memory card SPI interface:
- PD2: for memory card present signal.
- PB12: for memory card select.
- PB13: for memory card SCK.
- PB14: for memory card MISO.
- PB15: for memory card MOSI.

NOTE: The battery must be in place during operation otherwise when the charger
is disabled by the program the processor will power off.

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
#include <stdbool.h>

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
#include "../libs/comms.h"
#include "../libs/stringlib.h"
#include "../libs/file.h"
#include "../libs/timelib.h"
#include "../libs/dht.h"
#include "../libs/barosensor.h"
#include "../libs/i2clib.h"
#include "../libs/hardware.h"
#include "weather-station.h"
#include "weather-station-objdic.h"

/*--------------------------------------------------------------------------*/
/* Local Prototypes */

static void enable_radiance_measurement(void);
static void disable_radiance_measurement(void);
static void enable_charging(void);
static void disable_charging(void);
static uint8_t charger_active(void);
static void parse_command(uint8_t* line);

/*--------------------------------------------------------------------------*/
/* Global Variables */

static uint32_t rainfall;               /* Counter of interrupts from sensor */
static uint32_t wind_speed;             /* Counter of interrupts from sensor */
static uint8_t charger_is_active;       /* Existing state of charger */
static uint32_t measurement_period;
static uint8_t writeFileHandle;
static uint8_t readFileHandle;
static char writeFileName[12];
static char readFileName[12];
static bool deepSleep;                  /* Put processor into stop mode */
static bool lightSleep;                 /* Short term sleep mode in progress */
static bool scanNow;                    /* Request an immediate scan */
static uint32_t measurement_time;

/* These configuration variables are part of the Object Dictionary. */
/* This is defined in data-acquisition-objdic and is updated in response to
received messages. */
extern union ConfigGroup configData;

/*--------------------------------------------------------------------------*/

int main(void)
{
    set_global_defaults();
	hardware_init();
    DHT sensor_DHT = {DHT_PIN,DHT22,false};
    init_DHT(&sensor_DHT);
    init_comms_buffers();

    init_file_system();
    writeFileHandle = 0xFF;
    readFileHandle = 0xFF;
    writeFileName[0] = 0;
    readFileName[0] = 0;

    deepSleep = true;                   /* Start off in deep sleep */
    lightSleep = false;
    scanNow = true;

    charger_is_active = charger_active();
    uint8_t channel[1];                 /* Channel for A/D conversion */
    uint16_t charge_limit = 0;          /* voltage limit for charging battery */
    uint8_t i=0;
    comms_print_string("Weather Station\n\r");
    measurement_period = MEASUREMENT_PERIOD;
    measurement_time = rtc_get_counter_val() + measurement_period;
    uint32_t cnt;

    for (;;)
    {
/* -------- CLI ------------*/
/* Command Line Interface receiver interpretation.
 Process incoming commands as they appear on the serial input.
These will be fully processed before the processor goes to sleep mode.*/
        static uint8_t line[80];
        static uint8_t characterPosition = 0;
        if (receive_data_available())
        {
            uint8_t character = get_from_receive_buffer();
            if ((character == 0x0D) || (character == 0x0A) || (characterPosition > 78))
            {
                line[characterPosition] = 0;
                characterPosition = 0;
                parse_command(line);
            }
            else line[characterPosition++] = character;
        }
        if (! scanNow)
        {
/* Wait a bit for USART and any other outstanding operations to finish. */
            while (! usart_get_flag(USART1, USART_SR_TC)) {}
            for (cnt=0; cnt < 40000; cnt++) asm("nop");

/* Snooze for a while in low power stop mode. */
/* Possible power down modes are SLEEP or STOP modes only. These can be woken
by EXTI interrupts and so can register external events as well as RTC alarm for
periodical servicing of peripherals. If the USART is to be active during low
power mode then only SLEEP mode can be used. */

/* Just clear the whole bloody lot of exti pending requests */
            exti_reset_request(0xFFFFF);

/* -------- Deep Sleep ------------*/
/* The following would provide Deep Sleep (STOP) mode but at present this is
incompatible with USART usage. */
/* Use DEEPSLEEP mode if USART interrupts are not needed during sleep periods.
USART is powered off when the 1.8V regulator is powered down. */
#ifdef DEEPSLEEP
            if (deepSleep && ! lightSleep)
            {
/* Turn off peripherals during sleep (only ADC and DAC need to do this in STOP
mode if regulator low power is used). */
                peripheral_disable();
/* Don't set complete power down (otherwise it goes to standby and memory
contents are lost) */
                pwr_set_stop_mode();
/* Set deep sleep mode bit in SCB to go to stop mode */
                SCB_SCR |= SCB_SCR_SLEEPDEEP;
/* Set the 1.8V regulator to low power to preserve registers and SRAM but
power off core and digital peripherals. */
                pwr_voltage_regulator_low_power_in_stop();
            }
            else
#endif
            {
/* Clear power down bit */
                pwr_set_stop_mode();
/* Clear deep sleep mode bit in SCB in case it was previously set */
                SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
/* Leave the 1.8V regulator on. */
                pwr_voltage_regulator_on_in_stop();
            }

            rtc_set_alarm_time(measurement_time);
            asm volatile("wfi");

/* Check if the wakeup source was the alarm. If so clear the alarm flag and
set the new alarm value ahead of the RTC. Signal a scan to take place. */
		    if (rtc_check_flag(RTC_ALR))
            {
                scanNow = true;
/* After light sleep mode period, go to main sleep period without scanning. */
                if (lightSleep)
                {
                    measurement_period = MEASUREMENT_PERIOD;
                    scanNow = false;
                    lightSleep = false;
                }
			    rtc_clear_flag(RTC_ALR);
                measurement_time = rtc_get_counter_val() + measurement_period;
            }

#ifdef DEEPSLEEP
/* Restore hardware clocks and any config that may have been lost. */
            if (deepSleep && ! lightSleep)
            {
                peripheral_enable();
                usart1_setup();
            }
#endif
        }

		if (scanNow)
        {
            scanNow = false;

/* Send out a time string */
            char timeString[20];
            put_time_to_string(timeString);
            send_string("pH",timeString);
            if (is_recording()) record_string("pH",timeString,writeFileHandle);

/* Send status message. Bit 0 represents deep sleep (stop) mode */
            uint8_t status = 0;
            if (deepSleep) status |= 0x01;
            send_response("ds",status);

/* These tasks are now performed whenever the RTC alarm wakes the processor.
Other interrupts will activate their ISR but are ignored and the processor
will go back into stop mode. */

/* Read and send Battery Voltage. */
            uint16_t voltage_raw = 0;
            channel[0] = ADC_CHANNEL6;          /* channel 6 battery voltage */
	        adc_set_regular_sequence(ADC1, 1, channel);
/* Average over 16 readings */
            for (i=0; i<16; i++)
            {
                adc_start_conversion_direct(ADC1);
                while (! adc_eoc(ADC1));
                voltage_raw += adc_read_regular(ADC1);
                delay(1);
            }
/* Voltage times 256 for fixed point scaling, with sense resistor (x10)
and amplification (x100) will give results in volts. */
            uint32_t voltage = ((voltage_raw>>4)*256*V_AMP)/(1241*100);/* Volts */
/* Read Battery Current. */
            uint32_t current_raw = 0;
            channel[0] = ADC_CHANNEL7;          /* channel 7 battery current */
	        adc_set_regular_sequence(ADC1, 1, channel);
/* Average over 16 readings */
            for (i=0; i<16; i++)
            {
                adc_start_conversion_direct(ADC1);
                while (! adc_eoc(ADC1));
                current_raw += adc_read_regular(ADC1);
            }
/* Current in mA times 256 for fixed point scaling, with sense resistor times 10
and current amplification and scale back to average 16 readings.
Note order of computations to avoid 32 bit overflow. */
            uint32_t current = 
                    ((current_raw*BATTERY_SENSE_RESISTOR*(1000/10))
                                /(1241*I_AMP_BATTERY))*(256/16);
            data_message_send("dB", current, voltage);
            if (is_recording()) record_dual("dB", current, voltage, writeFileHandle);
            delay(5);

/* Battery Charge Limit Setting */
            charge_limit = 4000;                 /* temporary for testing */
            dac_load_data_buffer_single(charge_limit, RIGHT12, CHANNEL_2);
            dac_software_trigger(CHANNEL_2);

/* Control Battery Charging.
This cycles between the absorption voltage limit and the float voltage limit. */
            if (voltage > CHARGE_LIMIT) {
                disable_charging();              /* Turn off battery charging */
            }
            if (voltage < FLOAT_LIMIT) {
                enable_charging();               /* Turn on battery charging */
            }

/* Offset for time that systick was asleep */
			millis_offset(measurement_period * 1000);
/* Read and send (Temperature and) Humidity from the DTH22. */
            uint32_t temperature;
            uint32_t humidity;
			bool error = ! read_temperature_humidity(&sensor_DHT, &temperature,
												   &humidity, false);
			if (! error)
            {
                send_response("dT", temperature);
                if (is_recording()) record_single("dT", temperature, writeFileHandle);
                send_response("dH", humidity);
                if (is_recording()) record_single("dH", humidity, writeFileHandle);
                delay(5);
            }

/* Read and send temperature and barometric pressure over i2c. */
            int32_t temp;
            int32_t pressure;
            error = ! get_baro_temp_and_pressure(&temp,&pressure,CELSIUS,OSR_4096);
			if (! error)
            {
//                comms_print_string("dT,");
//                comms_print_fixed_point(temp);
//                comms_print_string("\n\r");
                send_response("dP", pressure);
                if (is_recording()) record_single("dP", pressure, writeFileHandle);
                delay(5);
            }

/* Read and send solar panel current. Use simple polling of the ADC. */
            enable_radiance_measurement();
            int32_t radiance_raw = 0;
            channel[0] = ADC_CHANNEL4;          /* channel 4 radiance */
	        adc_set_regular_sequence(ADC1, 1, channel);
/* Average over 16 readings */
            for (i=0; i<16; i++) {
                adc_start_conversion_direct(ADC1);
                while (! adc_eoc(ADC1));
                radiance_raw += adc_read_regular(ADC1);
            }
            disable_radiance_measurement();       /* Puts charging back on */
/* Current in mA times 256 for fixed point scaling, with sense resistor times 10
and current amplification and scale back to average 16 readings.
Note order of computations to avoid 32 bit overflow. */
            uint32_t radiance =
                    ((radiance_raw*RADIANCE_SENSE_RESISTOR*(1000/10))
                                /(1241*I_AMP_RADIANCE))*(256/16);
            send_response("dL", radiance);
            if (is_recording()) record_single("dL", radiance, writeFileHandle);
            delay(5);

/* Send rain gauge count. */
            send_response("dR", rainfall);
            if (is_recording()) record_single("dR", rainfall, writeFileHandle);
            delay(5);
            cli();
            rainfall = 0;
            sei();

/* Send wind speed count. */
            send_response("dS", wind_speed);
            if (is_recording()) record_single("dS", wind_speed, writeFileHandle);
            delay(5);
            cli();
            wind_speed = 0;
            sei();

/* Set to a short light sleep period in case base station attempts to contact */
            lightSleep = true;
            measurement_time = rtc_get_counter_val() + LIGHT_SLEEP_TIME;
        }

/* Wait a bit for USART and any other outstanding operations to finish. */
        while (! usart_get_flag(USART1, USART_SR_TC)) {}
        for (cnt=0; cnt < 40000; cnt++) asm("nop");
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
/* @brief Action a received USART command line.

*/

void parse_command(uint8_t* line)
{
/* ======================== Action commands ========================  */
/** Action Commands */
    if (line[0] == 'a')
    {
        switch (line[1])
        {
/* W Write the current configuration block to FLASH */
        case 'W':
            {
                write_config_block();
                break;
            }
/* Request identification string with version sent back.  */
        case 'E':
            {
                char ident[35] = "Weather Station,";
                string_append(ident,FIRMWARE_VERSION);
                send_string("dE",ident);
                break;
            }
/** Perform a scan. */
        case 'S':
            {
                if (! deepSleep) scanNow = true;
                break;
            }
        }
    }
/* ======================== Data request commands ================  */
/** Data Request Commands */
    else if (line[0] == 'd')
    {
        switch (line[1])
        {
/** Return the internal time. */
        case 'H':
            {
                char timeString[20];
                put_time_to_string(timeString);
                send_string("pH",timeString);
                break;
            }
        }
    }
/* ======================== Parameter commands ================  */
/**
Parameter Setting Commands */
    else if (line[0] == 'p')
    {
        switch (line[1])
        {
/* c-, c+ Turn communications sending on or off */
        case 'c':
            {
                if (line[2] == '-') configData.config.enableSend = false;
                else if (line[2] == '+') configData.config.enableSend = true;
                break;
            }
/* w-, w+ Turn deep sleep mode on or off (to allow USART communications) */
        case 'w':
            {
                if (line[2] == '-') deepSleep = false;
                else if (line[2] == '+') deepSleep = true;
                break;
            }
/* d-, d+ Turn on debug messages */
        case 'd':
            {
                if (line[2] == '+') configData.config.debugMessageSend = true;
                if (line[2] == '-') configData.config.debugMessageSend = false;
                break;
            }
/* Hxxxx Set time from an ISO 8601 formatted string. Echo back for GUI. */
        case 'H':
            {
                set_time_from_string((char*)line+2);
                char timeString[20];
                put_time_to_string(timeString);
                send_string("pH",timeString);
                measurement_time = rtc_get_counter_val() + measurement_period;
                break;
            }
/* M-, M+ Turn on/off data messaging (mainly for debug) */
        case 'M':
            {
                if (line[2] == '-') configData.config.measurementSend = false;
                else if (line[2] == '+') configData.config.measurementSend = true;
                break;
            }
/* r-, r+ Turn recording on or off */
        case 'r':
            {
                if (line[2] == '-') configData.config.recording = false;
                else if ((line[2] == '+') && (writeFileHandle < 0x0FF))
                    configData.config.recording = true;
                break;
            }
        }
    }

/* ======================== File commands ================ */
/*
F           - get free clusters
Wfilename   - Open file for read/write. Filename is 8.3 string style. Returns handle.
Rfilename   - Open file read only. Filename is 8.3 string style. Returns handle.
Xfilename   - Delete the file. Filename is 8.3 string style.
Cxx         - Close file. x is the file handle.
Gxx         - Read a record from read or write file.
Ddirname    - Get a directory listing. Directory name is 8.3 string style.
d[dirname]  - Get the first (if dirname present) or next entry in directory.
s           - Get status of open files and configData.config.recording flag
M           - Mount the SD card.
All commands return an error status byte at the end.
Only one file for writing and a second for reading is possible.
Data is not written to the file externally. */

/* File Commands */
    else if (line[0] == 'f')
    {
        switch (line[1])
        {
/* F Return number of free clusters followed by the cluster size in sectors. */
            case 'F':
            {
                uint32_t freeClusters = 0;
                uint32_t sectorsPerCluster = 0;
                uint8_t fileStatus = get_free_clusters(&freeClusters, &sectorsPerCluster);
                data_message_send("fF",freeClusters,sectorsPerCluster);
                send_response("fE",(uint8_t)fileStatus);
                break;
            }
/* d[d] Directory listing, d is the d=directory name. Get the first (if d
present) or next entry in the directory. If the name has a zero in the first
position, return the next entry in the directory listing. Returns the type,
size and name preceded by a comma. If there are no further entries found in the
directory, then size and name are not sent back. The type character can be:
 f = file, d = directory, n = error e = end */
            case 'd':
            {
                if (! file_system_usable()) break;
                char fileName[20];
                char type;
                uint32_t size;
                uint8_t fileStatus =
                    read_directory_entry((char*)line+2, &type, &size, fileName);
                char dirInfo[20];
                dirInfo[0] = type;
                dirInfo[1] = 0;
                if (type != 'e')
                {
                    char fileSize[5];
                    hex_to_ascii((size >> 16) & 0xFFFF,fileSize);
                    string_append(dirInfo,fileSize);
                    hex_to_ascii(size & 0xFFFF,fileSize);
                    string_append(dirInfo,fileSize);
                    string_append(dirInfo,fileName);
                }
                send_string("fd",dirInfo);
                send_response("fE",(uint8_t)fileStatus);
                break;
            }
/* Wf Open a file f=filename for writing less than 12 characters.
Parameter is a filename, 8 character plus dot plus 3 character extension.
Returns a file handle. On error, file handle is 0xFF. */
            case 'W':
            {
                if (! file_system_usable()) break;
                if (string_length((char*)line+2) < 12)
                {
                    uint8_t fileStatus =
                        open_write_file((char*)line+2, &writeFileHandle);
                    if (fileStatus == 0)
                    {
                        string_copy(writeFileName,(char*)line+2);
                        send_response("fW",writeFileHandle);
                    }
                    send_response("fE",(uint8_t)fileStatus);
                }
                break;
            }
/* Rf Open a file f=filename for reading less than 12 characters.
Parameter is a filename, 8 character plus dot plus 3 character extension.
Returns a file handle. On error, file handle is 0xFF. */
            case 'R':
            {
                if (! file_system_usable()) break;
                if (string_length((char*)line+2) < 12)
                {
                    uint8_t fileStatus = 
                        open_read_file((char*)line+2, &readFileHandle);
                    if (fileStatus == 0)
                    {
                        string_copy(readFileName,(char*)line+2);
                        send_response("fR",readFileHandle);
                    }
                    send_response("fE",(uint8_t)fileStatus);
                }
                break;
            }
/* Gf Read a record from the file specified by f=file handle. Return as a comma
separated list. */
            case 'G':
            {
                if (! file_system_usable()) break;
                uint8_t fileHandle = ascii_to_int((char*)line+2);
                if (valid_file_handle(fileHandle))
                {
                    char string[80];
                    uint8_t fileStatus = 
                        read_line_from_file(fileHandle,string);
                    send_string("fG",string);
                    send_response("fE",(uint8_t)fileStatus);
                }
                break;
            }
/* s Send a status message containing: software switches and names of open
files, with open write filehandle and filename first followed by read filehandle
and filename, or blank if any file is not open. */
            case 's':
            {
                comms_print_string("fs,");
                comms_print_int((int)get_controls());
                comms_print_string(",");
                uint8_t writeStatus;
                comms_print_int(writeFileHandle);
                comms_print_string(",");
                if (writeFileHandle < 0xFF)
                {
                    comms_print_string(writeFileName);
                    comms_print_string(",");
                }
                comms_print_int(readFileHandle);
                if (readFileHandle < 0xFF)
                {
                    comms_print_string(",");
                    comms_print_string(readFileName);
                }
                comms_print_string("\r\n");
                break;
            }
/* Cf Close File specified by f=file handle. */
            case 'C':
            {
                if (! file_system_usable()) break;
                uint8_t fileHandle = ascii_to_int((char*)line+2);
                uint8_t fileStatus = close_file(&fileHandle);
                if (fileStatus == 0) writeFileHandle = 0xFF;
                send_response("fE",(uint8_t)fileStatus);
                break;
            }
/* X Delete File. */
            case 'X':
            {
                if (! file_system_usable()) break;
                uint8_t fileStatus = delete_file((char*)line+2);
                send_response("fE",(uint8_t)fileStatus);
                break;
            }
/* M Reinitialize the memory card. */
            case 'M':
            {
                uint8_t fileStatus = init_file_system();
                send_response("fE",(uint8_t)fileStatus);
                break;
            }
/* Z Create a standard file system on the memory volume */
            case 'Z':
            {
                send_string("D","Creating Filesystem");
                uint8_t fileStatus = make_filesystem();
                send_response("fE",(uint8_t)fileStatus);
                break;
            }
        }
    }
}

/*--------------------------------------------------------------------------*/
/* @brief Enable measurement

The measurement MOSFET must be turned on and the charging must be turned off.
Delay for a while to allow settling of currents in the sense resistor.
*/

void enable_radiance_measurement(void)
{
    charger_is_active = charger_active();
    disable_charging();                  /* turn off charger */
    gpio_set(GPIOB, GPIO4);             /* Turn on measurement switch */
    delay(RADIANCE_SETTLE_TIME);
}

/*--------------------------------------------------------------------------*/
/* @brief Disable measurement

The measurement MOSFET must be turned off and the charging must be restored
if it was in a charging state at the time of measurement.
*/

void disable_radiance_measurement(void)
{
    gpio_clear(GPIOB, GPIO4);           /* Turn off measurement switch */
    if (charger_is_active) enable_charging();
    else disable_charging();
}

/*--------------------------------------------------------------------------*/
/* @brief Enable Charging

Output must be low to turn on the MOSFET.
*/

void enable_charging(void)
{
    gpio_clear(GPIOB, GPIO5);
}

/*--------------------------------------------------------------------------*/
/* @brief Disable Charging

Output must be high to turn off the MOSFET.
*/

void disable_charging(void)
{
    gpio_set(GPIOB, GPIO5);
}

/*--------------------------------------------------------------------------*/
/* @brief Return State of Charging

@returns uint8_t: true = charging active, false = charging inactive.
*/

uint8_t charger_active(void)
{
    return (gpio_get(GPIOB, GPIO5) == 0);
}

/*--------------------------------------------------------------------------*/
/* Interrupt Service Routines */
/*--------------------------------------------------------------------------*/
/* EXT0

Bit 0 of each port used as a pin interrupt. This is for rainfall.
*/

void exti0_isr(void)
{
    rainfall++;
    exti_reset_request(EXTI0);
}

/*--------------------------------------------------------------------------*/
/* EXT2

Bit 2 of each port used as a pin interrupt. This is for wind speed.
*/

void exti2_isr(void)
{
    wind_speed++;
    exti_reset_request(EXTI2);
}

/*--------------------------------------------------------------------------*/
/* EXT3

Bit 3 of each port used as a pin interrupt. This is for wind direction.
*/

void exti3_isr(void)
{
    exti_reset_request(EXTI3);
}

/*--------------------------------------------------------------------------*/

