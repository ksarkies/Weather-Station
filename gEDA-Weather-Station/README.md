Weather Station Interface Board
-------------------------------

This board provides the interface between the processor and the solar panel.
The panel is used to measure solar irradiation by measuring the (almost) short
circuit current which has a quite linear relationship with the incident
illuminance.

The panel is also used to charge the battery that powers the station. This
charging is done between measurements and is independent of them. A measurement
is taken by activating a digital output from the processor, which turns on a
MOSFET to the current measurement circuit. This effectively shorts out the
panel. At the same time the battery charger circuit is disabled to avoid any
possible interference with the measurements.

A small FM640A 6V 4AH Lead Acid battery is used for powering the system. 
The circuit manages battery charging by limiting the battery voltage through
a linear regulator with an output voltage controlled from the processor. The
battery voltage and charging current is passed to the processor for management
battery health.

A 3.3V power is supplied for the processor, and 5V power for the operational
amplifier. The LM324 has a 3.5V output limit with this supply voltage. The
LM324 also draws only about 0.7mA from the battery. The processor can be placed
into sleep mode between measurements to reduce power drain.

Design Notes
------------

The LM324 has been given a supply voltage VDD of 5V. The outputs can reach
VDD-1.5V which limits them to 3.5V; within the range of the processor input
voltage limits.

The variable voltage regulator based on the 7805 has a 100 ohm voltage setting
resistor. The 7805 has a quiescent current of 60mA which flows through this
resistor and sets a minimum voltage of 0.6V. The LM324 can source as much as
60mA which gives a maximum voltage of 3.6V. This provides an output voltage
range between 5.6V and 8.6V. The actual maximum can be somewhat lower but as
long as it is more than 7.5V it will provide a valuable natural limit on the
battery charging voltage.

A Schottky diode prevents the battery discharging back through the circuit when
measurements are being taken (as both MOSFETs may be on simultaneously for a
brief period as they are switched).

(c) K. Sarkies 10/06/2016

