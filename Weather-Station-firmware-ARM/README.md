Weather Station Firmware
--------------------------

The firmware for the remote weather station unit provides interfacing to
the sensors in its vicinity. Communication with a base station or external PC
is done over a serial cable. It is possible that a wireless interface could be
added in the future.

The firmware powers down the unit to sleep, and wakes on interrupt from a
sensor, or at regular preset times to take measurements with non-interrupting
sensors and to make connection with any attached base station.

The firmware also controls charging of the backup battery from the solar
panel. This is done by monitoring the battery voltage during bulk charge and
applying a limit when it reaches absorption phase. Charging is then stopped
when the current falls below the 0.02C level (C being the battery capacity).

The Makefile currently includes the load script for the STM32F103RET6 used on
the ET Stamp module. Change this to STM32F103RBT6 if the ET-STM32F103
development board is used.

The sensors are allocated to the following GPIO ports:
* PA0 Rainfall Gauge using EXTI interrupt.
* PA1 Temperature and Humidity from DTH22.
* PA2 Wind speed using EXTI interrupt.
* PA3 Wind direction using EXTI interrupt.
* PA4 Solar radiance current using ADC12-IN4.
* Air pressure. This is I2C and needs I2C1_SCL on PB6 and I2C1_SDA on PB7.

For battery charging the following are needed:
* PA5 from a D/A converter DAC_OUT2 to control the battery charging voltage.
* PA6 to measure the battery voltage on ADC12-IN6.
* PA7 to measure the battery current on ADC12-IN7.
* PB4 to switch the panel to the solar radiance current measurement circuit.
* PB5 to disable battery charging.

This allocation allows ports to be used directly without remap on the STM32F103.

(c) K. Sarkies 07/05/2016

