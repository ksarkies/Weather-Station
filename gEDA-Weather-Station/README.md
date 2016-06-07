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

The circuit manages battery charging by limiting the battery voltage through
a linear regulator with an output voltage controlled from the processor. The
battery voltage and charging current is passed to the processor for management
battery health.

A 3.3V power is supplied for the processor, and 5V power for the operational
amplifier. The LM324 has a 3.5V output limit with this supply voltage. The
LM324 also draws only about 0.7mA from the battery. The processor can be placed
into sleep mode between measurements to reduce power drain.

(c) K. Sarkies 07/06/2016

