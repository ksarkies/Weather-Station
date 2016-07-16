Weather Station
---------------

This project aims to produce a weather station consisting of a weatherproofed
processing unit for control and data capture from sensors. This would be
remotely mounted in a suitable location with the sensors and will communicate
with a base station over a serial cable or wirelessly.

The remote unit is ARM Cortex CM3 based and is powered either from the serial
link (if present) and/or from a solar panel with battery backup.

The base station collects and stores data and permit configuration of the
system via a user interface. It consists of a Linux based PC or microcomputer
unit.

Some of the sensors initially are taken from a defunct low cost commercial
weather station. Later they may be replaced with scientific grade sensors.

The sensors to be used are:

1. Temperature and humidity (indoor and outdoor),
2. Wind Speed,
3. Wind Direction,
4. Rainfall,
5. Barometric Pressure,
6. Solar Radiance.

The solar panel provided for power is also used to measure the incident
solar radiance. This is a measure specific to the panel type and is intended
only for providing records of incident radiance for use in design of solar
power generators having similar panel types.

(c) K. Sarkies 16/07/2016

