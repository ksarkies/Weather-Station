Weather Station Interface Board: gEDA Version
---------------------------------------------

This PCB version will not be updated. The Kicad version will be used instead.

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

The variable voltage regulator based on the 7805 has a voltage setting 
resistor. The 7805 has a quiescent current of a few milliamps which flows
through this resistor and sets a minimum voltage. After some experimentation
it was found that a 470 ohm resistor would set a minimum voltage of 0.8V and a
maximum of 3.3V. The LM324 can source as much as 60mA which is sufficient to
maintain this maximum voltage. This provides an output voltage range between
5.5V and 8.0V. As long as it is more than 7.5V it will provide a valuable
natural limit on the battery charging voltage. Using a D/A converter to drive
this voltage will allow accurate setting of voltages according to battery
temperature. An option for testing purposes allows a fixed voltage provided by
jumper lead as any selected processor may not have a D/A converter. Once the
charging current falls below a given threshold the processor must terminate
charging.

A Schottky diode prevents the battery discharging back through the circuit when
measurements are being taken (as both MOSFETs may be on simultaneously for a
brief period when they are switched).

(c) K. Sarkies 10/06/2016

