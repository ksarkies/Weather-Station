Test of Deep Sleep mode on STM32F103
------------------------------------

This is a test program to investigate the use of deep sleep modes, notably stop
mode, for teh STM32F103. The RTC Alarm is needed for wakeup. The sleep mode
used is Wait for Interrupt (wfi instruction) so that other asynchronous EXTI
interrupts can be recognised and handled during the sleep period.

It is noted that on wakeup the system clock must be reconfigured back to the
original frequency setting, and the RTC needs to be woken up. All other clocks
and peripheral settings are maintained during sleep.

The RTC Alarm flag must be checked on wakeup to determine is wakeup occurred due
to the alarm or another interrupt. In the former case the flag must be cleared
and the alarm counter reset to zero if more sleep periods are to occur.

ADC and DAC must be powered down before sleep to conserve power.

(c) K. Sarkies 16/07/2016

