EESchema Schematic File Version 2
LIBS:Kicad-Weather-Station-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:DataStorage
LIBS:KB1LQC
LIBS:MiscellaneousDevices
LIBS:RF_OEM_Parts
LIBS:Sensors
LIBS:TransistorParts
LIBS:Regulators
LIBS:Kicad-Weather-Station-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Q_NPN_EBC Q1
U 1 1 579469B0
P 3300 1700
F 0 "Q1" H 3600 1750 50  0000 R CNN
F 1 "MMBT3904" H 3500 1350 50  0000 R CNN
F 2 "WeatherStation:SOT23EBC" H 3500 1800 50  0001 C CNN
F 3 "" H 3300 1700 50  0000 C CNN
	1    3300 1700
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_EBC Q2
U 1 1 57946B6D
P 4100 1100
F 0 "Q2" H 4400 1150 50  0000 R CNN
F 1 "MMBT3904" H 4350 1400 50  0000 R CNN
F 2 "WeatherStation:SOT23EBC" H 4300 1200 50  0001 C CNN
F 3 "" H 4100 1100 50  0000 C CNN
	1    4100 1100
	1    0    0    -1  
$EndComp
$Comp
L Q_PNP_EBC Q3
U 1 1 57946B8E
P 4100 1500
F 0 "Q3" H 4400 1550 50  0000 R CNN
F 1 "MMBT3906" H 4400 1900 50  0000 R CNN
F 2 "WeatherStation:SOT23EBC" H 4300 1600 50  0001 C CNN
F 3 "" H 4100 1500 50  0000 C CNN
	1    4100 1500
	1    0    0    1   
$EndComp
$Comp
L Q_PMOS_GDS Q4
U 1 1 57946BC6
P 4650 1300
F 0 "Q4" H 4950 1350 50  0000 R CNN
F 1 "SUD45P03-12-GE3" H 5300 800 50  0000 R CNN
F 2 "WeatherStation:TO-252-2Lead" H 4850 1400 50  0001 C CNN
F 3 "" H 4650 1300 50  0000 C CNN
	1    4650 1300
	1    0    0    1   
$EndComp
$Comp
L R R8
U 1 1 57946E13
P 3400 1350
F 0 "R8" V 3480 1350 50  0000 C CNN
F 1 "10K" V 3400 1350 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 3330 1350 50  0001 C CNN
F 3 "" H 3400 1350 50  0000 C CNN
	1    3400 1350
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 57946ED8
P 2950 1700
F 0 "R5" V 3030 1700 50  0000 C CNN
F 1 "100K" V 2950 1700 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 2880 1700 50  0001 C CNN
F 3 "" H 2950 1700 50  0000 C CNN
	1    2950 1700
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 57946F81
P 2500 1850
F 0 "R1" V 2580 1850 50  0000 C CNN
F 1 "10K" V 2500 1850 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 2430 1850 50  0001 C CNN
F 3 "" H 2500 1850 50  0000 C CNN
	1    2500 1850
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR01
U 1 1 579472CF
P 3400 1900
F 0 "#PWR01" H 3400 1650 50  0001 C CNN
F 1 "Earth" H 3400 1750 50  0001 C CNN
F 2 "" H 3400 1900 50  0000 C CNN
F 3 "" H 3400 1900 50  0000 C CNN
	1    3400 1900
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR02
U 1 1 579472F1
P 4200 1700
F 0 "#PWR02" H 4200 1450 50  0001 C CNN
F 1 "Earth" H 4200 1550 50  0001 C CNN
F 2 "" H 4200 1700 50  0000 C CNN
F 3 "" H 4200 1700 50  0000 C CNN
	1    4200 1700
	1    0    0    -1  
$EndComp
Text GLabel 2250 1700 0    60   Input ~ 0
PB4
$Comp
L BARREL_JACK CON1
U 1 1 57947748
P 2250 1000
F 0 "CON1" H 2250 1250 50  0000 C CNN
F 1 "Solar Panel" H 2250 800 50  0000 C CNN
F 2 "WeatherStation:BARREL_JACK" H 2250 1000 50  0001 C CNN
F 3 "" H 2250 1000 50  0000 C CNN
	1    2250 1000
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR03
U 1 1 57947821
P 2550 1250
F 0 "#PWR03" H 2550 1000 50  0001 C CNN
F 1 "Earth" H 2550 1100 50  0001 C CNN
F 2 "" H 2550 1250 50  0000 C CNN
F 3 "" H 2550 1250 50  0000 C CNN
	1    2550 1250
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 57947CC0
P 4750 3900
F 0 "R9" V 4830 3900 50  0000 C CNN
F 1 "1" V 4750 3900 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 4680 3900 50  0001 C CNN
F 3 "" H 4750 3900 50  0000 C CNN
	1    4750 3900
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR04
U 1 1 57947D3D
P 4750 4050
F 0 "#PWR04" H 4750 3800 50  0001 C CNN
F 1 "Earth" H 4750 3900 50  0001 C CNN
F 2 "" H 4750 4050 50  0000 C CNN
F 3 "" H 4750 4050 50  0000 C CNN
	1    4750 4050
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_EBC Q5
U 1 1 5794871D
P 5450 2550
F 0 "Q5" H 5400 2650 50  0000 R CNN
F 1 "MMBT3904" H 5700 2050 50  0000 R CNN
F 2 "WeatherStation:SOT23EBC" H 5650 2650 50  0001 C CNN
F 3 "" H 5450 2550 50  0000 C CNN
	1    5450 2550
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_EBC Q7
U 1 1 57948723
P 6250 1100
F 0 "Q7" H 6550 1150 50  0000 R CNN
F 1 "MMBT3904" H 6350 1400 50  0000 R CNN
F 2 "WeatherStation:SOT23EBC" H 6450 1200 50  0001 C CNN
F 3 "" H 6250 1100 50  0000 C CNN
	1    6250 1100
	1    0    0    -1  
$EndComp
$Comp
L Q_PNP_EBC Q8
U 1 1 57948729
P 6250 1500
F 0 "Q8" H 6550 1550 50  0000 R CNN
F 1 "MMBT3906" H 6500 1900 50  0000 R CNN
F 2 "WeatherStation:SOT23EBC" H 6450 1600 50  0001 C CNN
F 3 "" H 6250 1500 50  0000 C CNN
	1    6250 1500
	1    0    0    1   
$EndComp
$Comp
L Q_PMOS_GDS Q9
U 1 1 5794872F
P 6650 1300
F 0 "Q9" H 6950 1350 50  0000 R CNN
F 1 "SUD45P03-12-GE3" H 7200 800 50  0000 R CNN
F 2 "WeatherStation:TO-252-2Lead" H 6850 1400 50  0001 C CNN
F 3 "" H 6650 1300 50  0000 C CNN
	1    6650 1300
	1    0    0    1   
$EndComp
$Comp
L R R16
U 1 1 57948735
P 5550 1200
F 0 "R16" V 5630 1200 50  0000 C CNN
F 1 "10K" V 5550 1200 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 5480 1200 50  0001 C CNN
F 3 "" H 5550 1200 50  0000 C CNN
	1    5550 1200
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5794873B
P 2950 2550
F 0 "R6" V 3030 2550 50  0000 C CNN
F 1 "100K" V 2950 2550 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 2880 2550 50  0001 C CNN
F 3 "" H 2950 2550 50  0000 C CNN
	1    2950 2550
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR05
U 1 1 57948746
P 5550 2750
F 0 "#PWR05" H 5550 2500 50  0001 C CNN
F 1 "Earth" H 5550 2600 50  0001 C CNN
F 2 "" H 5550 2750 50  0000 C CNN
F 3 "" H 5550 2750 50  0000 C CNN
	1    5550 2750
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR06
U 1 1 5794874C
P 6350 1700
F 0 "#PWR06" H 6350 1450 50  0001 C CNN
F 1 "Earth" H 6350 1550 50  0001 C CNN
F 2 "" H 6350 1700 50  0000 C CNN
F 3 "" H 6350 1700 50  0000 C CNN
	1    6350 1700
	1    0    0    -1  
$EndComp
$Comp
L R R22
U 1 1 5794875F
P 7850 3500
F 0 "R22" V 7930 3500 50  0000 C CNN
F 1 "470" V 7850 3500 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 7780 3500 50  0001 C CNN
F 3 "" H 7850 3500 50  0000 C CNN
	1    7850 3500
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR07
U 1 1 57948765
P 7850 3650
F 0 "#PWR07" H 7850 3400 50  0001 C CNN
F 1 "Earth" H 7850 3500 50  0001 C CNN
F 2 "" H 7850 3650 50  0000 C CNN
F 3 "" H 7850 3650 50  0000 C CNN
	1    7850 3650
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D1
U 1 1 57948A62
P 7700 1500
F 0 "D1" H 7700 1600 50  0000 C CNN
F 1 "MBR0520LT1G" H 7700 1400 50  0000 C CNN
F 2 "WeatherStation:SOD-123" H 7700 1500 50  0001 C CNN
F 3 "" H 7700 1500 50  0000 C CNN
	1    7700 1500
	-1   0    0    1   
$EndComp
$Comp
L TPS77033 U3
U 1 1 5794A170
P 9700 1550
F 0 "U3" H 9850 1154 60  0000 C CNN
F 1 "TPS77033" H 9900 1750 60  0000 C CNN
F 2 "WeatherStation:SOT23-5" H 9700 1550 60  0001 C CNN
F 3 "" H 9700 1550 60  0000 C CNN
	1    9700 1550
	1    0    0    -1  
$EndComp
$Comp
L R R24
U 1 1 5794A35E
P 9050 1650
F 0 "R24" V 9130 1650 50  0000 C CNN
F 1 "100K" V 9050 1650 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 8980 1650 50  0001 C CNN
F 3 "" H 9050 1650 50  0000 C CNN
	1    9050 1650
	1    0    0    -1  
$EndComp
$Comp
L R R25
U 1 1 5794A407
P 9050 2950
F 0 "R25" V 9130 2950 50  0000 C CNN
F 1 "68K" V 9050 2950 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 8980 2950 50  0001 C CNN
F 3 "" H 9050 2950 50  0000 C CNN
	1    9050 2950
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR08
U 1 1 5794A9D3
P 9050 3100
F 0 "#PWR08" H 9050 2850 50  0001 C CNN
F 1 "Earth" H 9050 2950 50  0001 C CNN
F 2 "" H 9050 3100 50  0000 C CNN
F 3 "" H 9050 3100 50  0000 C CNN
	1    9050 3100
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR09
U 1 1 5794AA56
P 9400 1750
F 0 "#PWR09" H 9400 1500 50  0001 C CNN
F 1 "Earth" H 9400 1600 50  0001 C CNN
F 2 "" H 9400 1750 50  0000 C CNN
F 3 "" H 9400 1750 50  0000 C CNN
	1    9400 1750
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR010
U 1 1 5794AAA3
P 9700 2000
F 0 "#PWR010" H 9700 1750 50  0001 C CNN
F 1 "Earth" H 9700 1850 50  0001 C CNN
F 2 "" H 9700 2000 50  0000 C CNN
F 3 "" H 9700 2000 50  0000 C CNN
	1    9700 2000
	1    0    0    -1  
$EndComp
Text GLabel 10500 1650 2    60   Input ~ 0
+3.3V-M
Text GLabel 2250 2550 0    60   Input ~ 0
PB5
Text GLabel 8850 2500 0    60   Input ~ 0
PA6
Text GLabel 2250 3250 0    60   Input ~ 0
PA4
Text GLabel 6350 3350 0    60   Input ~ 0
PA5
$Comp
L LM324 U1
U 2 1 57948CB0
P 7250 2850
F 0 "U1" H 7300 3050 50  0000 C CNN
F 1 "LMV324" H 7400 2650 50  0000 C CNN
F 2 "WeatherStation:SOIC-14_N" H 7200 2950 50  0001 C CNN
F 3 "" H 7300 3050 50  0000 C CNN
	2    7250 2850
	1    0    0    -1  
$EndComp
$Comp
L LM324 U1
U 3 1 57948DC1
P 7500 5400
F 0 "U1" H 7550 5600 50  0000 C CNN
F 1 "LMV324" H 7800 5650 50  0000 C CNN
F 2 "WeatherStation:SOIC-14_N" H 7450 5500 50  0001 C CNN
F 3 "" H 7550 5600 50  0000 C CNN
	3    7500 5400
	-1   0    0    -1  
$EndComp
$Comp
L LM324 U1
U 1 1 57948E7E
P 2800 3250
F 0 "U1" H 2850 3450 50  0000 C CNN
F 1 "LMV324" H 3050 3400 50  0000 C CNN
F 2 "WeatherStation:SOIC-14_N" H 2750 3350 50  0001 C CNN
F 3 "" H 2850 3450 50  0000 C CNN
	1    2800 3250
	-1   0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5794941C
P 2500 3400
F 0 "R3" V 2580 3400 50  0000 C CNN
F 1 "100K" V 2500 3400 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 2430 3400 50  0001 C CNN
F 3 "" H 2500 3400 50  0000 C CNN
	1    2500 3400
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 57949512
P 2500 3950
F 0 "R4" V 2580 3950 50  0000 C CNN
F 1 "10K" V 2500 3950 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 2430 3950 50  0001 C CNN
F 3 "" H 2500 3950 50  0000 C CNN
	1    2500 3950
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR011
U 1 1 579495D7
P 2500 4100
F 0 "#PWR011" H 2500 3850 50  0001 C CNN
F 1 "Earth" H 2500 3950 50  0001 C CNN
F 2 "" H 2500 4100 50  0000 C CNN
F 3 "" H 2500 4100 50  0000 C CNN
	1    2500 4100
	1    0    0    -1  
$EndComp
$Comp
L R R20
U 1 1 579498DB
P 7200 5550
F 0 "R20" V 7280 5550 50  0000 C CNN
F 1 "100K" V 7200 5550 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 7130 5550 50  0001 C CNN
F 3 "" H 7200 5550 50  0000 C CNN
	1    7200 5550
	1    0    0    -1  
$EndComp
$Comp
L R R21
U 1 1 579498E1
P 7200 6100
F 0 "R21" V 7280 6100 50  0000 C CNN
F 1 "10K" V 7200 6100 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 7130 6100 50  0001 C CNN
F 3 "" H 7200 6100 50  0000 C CNN
	1    7200 6100
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR012
U 1 1 579498E7
P 7200 6250
F 0 "#PWR012" H 7200 6000 50  0001 C CNN
F 1 "Earth" H 7200 6100 50  0001 C CNN
F 2 "" H 7200 6250 50  0000 C CNN
F 3 "" H 7200 6250 50  0000 C CNN
	1    7200 6250
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 57949AE3
P 6650 3500
F 0 "R18" V 6730 3500 50  0000 C CNN
F 1 "100K" V 6650 3500 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 6580 3500 50  0001 C CNN
F 3 "" H 6650 3500 50  0000 C CNN
	1    6650 3500
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR013
U 1 1 57949D65
P 6650 3650
F 0 "#PWR013" H 6650 3400 50  0001 C CNN
F 1 "Earth" H 6650 3500 50  0001 C CNN
F 2 "" H 6650 3650 50  0000 C CNN
F 3 "" H 6650 3650 50  0000 C CNN
	1    6650 3650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P7
U 1 1 5794A4D6
P 8250 2500
F 0 "P7" H 8250 2650 50  0000 C CNN
F 1 "Battery" H 8300 2300 50  0000 C CNN
F 2 "WeatherStation:PINHEAD1-2" H 8250 2500 50  0001 C CNN
F 3 "" H 8250 2500 50  0000 C CNN
	1    8250 2500
	1    0    0    -1  
$EndComp
$Comp
L R R23
U 1 1 5794A692
P 8050 6100
F 0 "R23" V 8130 6100 50  0000 C CNN
F 1 "1" V 8050 6100 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 7980 6100 50  0001 C CNN
F 3 "" H 8050 6100 50  0000 C CNN
	1    8050 6100
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR014
U 1 1 5794A698
P 8050 6250
F 0 "#PWR014" H 8050 6000 50  0001 C CNN
F 1 "Earth" H 8050 6100 50  0001 C CNN
F 2 "" H 8050 6250 50  0000 C CNN
F 3 "" H 8050 6250 50  0000 C CNN
	1    8050 6250
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_EBC Q6
U 1 1 5794F0C3
P 5450 2150
F 0 "Q6" H 5700 2250 50  0000 R CNN
F 1 "MMBT3904" H 6150 2200 50  0000 R CNN
F 2 "WeatherStation:SOT23EBC" H 5650 2250 50  0001 C CNN
F 3 "" H 5450 2150 50  0000 C CNN
	1    5450 2150
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5794F6E9
P 2500 2400
F 0 "R2" V 2580 2400 50  0000 C CNN
F 1 "10K" V 2500 2400 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 2430 2400 50  0001 C CNN
F 3 "" H 2500 2400 50  0000 C CNN
	1    2500 2400
	-1   0    0    1   
$EndComp
$Comp
L D D2
U 1 1 579521C9
P 8350 6100
F 0 "D2" H 8350 6200 50  0000 C CNN
F 1 "1N4148" H 8350 6000 50  0000 C CNN
F 2 "WeatherStation:SOD-523" H 8350 6100 50  0001 C CNN
F 3 "" H 8350 6100 50  0000 C CNN
	1    8350 6100
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR015
U 1 1 57952374
P 8350 6250
F 0 "#PWR015" H 8350 6000 50  0001 C CNN
F 1 "Earth" H 8350 6100 50  0001 C CNN
F 2 "" H 8350 6250 50  0000 C CNN
F 3 "" H 8350 6250 50  0000 C CNN
	1    8350 6250
	1    0    0    -1  
$EndComp
$Comp
L CP C3
U 1 1 57953173
P 8700 2000
F 0 "C3" H 8725 2100 50  0000 L CNN
F 1 "100uF" H 8725 1900 50  0000 L CNN
F 2 "WeatherStation:SM1206POL" H 8738 1850 50  0001 C CNN
F 3 "" H 8700 2000 50  0000 C CNN
	1    8700 2000
	1    0    0    -1  
$EndComp
$Comp
L CP C4
U 1 1 57953264
P 10250 2000
F 0 "C4" H 10275 2100 50  0000 L CNN
F 1 "10uF" H 10275 1900 50  0000 L CNN
F 2 "WeatherStation:SM1206POL" H 10288 1850 50  0001 C CNN
F 3 "" H 10250 2000 50  0000 C CNN
	1    10250 2000
	1    0    0    -1  
$EndComp
$Comp
L CP C2
U 1 1 57954A85
P 3100 1050
F 0 "C2" H 3125 1150 50  0000 L CNN
F 1 "100uF" H 3125 950 50  0000 L CNN
F 2 "WeatherStation:SM1206POL" H 3138 900 50  0001 C CNN
F 3 "" H 3100 1050 50  0000 C CNN
	1    3100 1050
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 57954AF6
P 2800 1050
F 0 "C1" H 2825 1150 50  0000 L CNN
F 1 "0.1uF" H 2825 950 50  0000 L CNN
F 2 "WeatherStation:SM0603_Capa" H 2838 900 50  0001 C CNN
F 3 "" H 2800 1050 50  0000 C CNN
	1    2800 1050
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR016
U 1 1 57955EC5
P 8700 2150
F 0 "#PWR016" H 8700 1900 50  0001 C CNN
F 1 "Earth" H 8700 2000 50  0001 C CNN
F 2 "" H 8700 2150 50  0000 C CNN
F 3 "" H 8700 2150 50  0000 C CNN
	1    8700 2150
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X25 P1
U 1 1 5795879B
P 2500 5350
F 0 "P1" H 2500 6650 50  0000 C CNN
F 1 "CONN_01X25" V 2600 5350 50  0000 C CNN
F 2 "WeatherStation:Pin_Header_Straight_1x25" H 2500 5350 50  0001 C CNN
F 3 "" H 2500 5350 50  0000 C CNN
	1    2500 5350
	0    1    1    0   
$EndComp
$Comp
L CONN_01X25 P2
U 1 1 579588A6
P 2550 5900
F 0 "P2" H 2550 7200 50  0000 C CNN
F 1 "CONN_01X25" V 2650 5900 50  0000 C CNN
F 2 "WeatherStation:Pin_Header_Straight_1x25" H 2550 5900 50  0001 C CNN
F 3 "" H 2550 5900 50  0000 C CNN
	1    2550 5900
	0    -1   -1   0   
$EndComp
$Comp
L Earth #PWR017
U 1 1 57958AFF
P 3750 6100
F 0 "#PWR017" H 3750 5850 50  0001 C CNN
F 1 "Earth" H 3750 5950 50  0001 C CNN
F 2 "" H 3750 6100 50  0000 C CNN
F 3 "" H 3750 6100 50  0000 C CNN
	1    3750 6100
	1    0    0    -1  
$EndComp
Text GLabel 1300 5150 1    60   Input ~ 0
+3.3V-M
Text GLabel 3300 5150 1    60   Input ~ 0
PA4
Text GLabel 3200 5150 1    60   Input ~ 0
PA5
Text GLabel 3100 5150 1    60   Input ~ 0
PA6
Text GLabel 3000 5150 1    60   Input ~ 0
PA7
Text GLabel 1500 5150 1    60   Input ~ 0
PB6
Text GLabel 1600 5150 1    60   Input ~ 0
PB5
Text GLabel 1700 5150 1    60   Input ~ 0
PB4
$Comp
L CONN_01X02 P3
U 1 1 5795907D
P 4450 7100
F 0 "P3" H 4450 7250 50  0000 C CNN
F 1 "Rain Gauge" V 4550 7100 50  0000 C CNN
F 2 "WeatherStation:PINHEAD1-2" H 4450 7100 50  0001 C CNN
F 3 "" H 4450 7100 50  0000 C CNN
	1    4450 7100
	0    1    1    0   
$EndComp
$Comp
L CONN_01X02 P5
U 1 1 57959159
P 5150 7100
F 0 "P5" H 5150 7250 50  0000 C CNN
F 1 "Wind Speed" V 5250 7100 50  0000 C CNN
F 2 "WeatherStation:PINHEAD1-2" H 5150 7100 50  0001 C CNN
F 3 "" H 5150 7100 50  0000 C CNN
	1    5150 7100
	0    1    1    0   
$EndComp
$Comp
L CONN_01X03 P4
U 1 1 579591CF
P 5600 7100
F 0 "P4" H 5600 7300 50  0000 C CNN
F 1 "Temperature/Humidity" V 5800 7150 50  0000 C CNN
F 2 "WeatherStation:PINHEAD1-3" H 5600 7100 50  0001 C CNN
F 3 "" H 5600 7100 50  0000 C CNN
	1    5600 7100
	0    1    1    0   
$EndComp
$Comp
L CONN_01X04 P6
U 1 1 57959288
P 6300 7100
F 0 "P6" H 6300 7350 50  0000 C CNN
F 1 "Air Pressure" V 6400 7100 50  0000 C CNN
F 2 "WeatherStation:PINHEAD1-4" H 6300 7100 50  0001 C CNN
F 3 "" H 6300 7100 50  0000 C CNN
	1    6300 7100
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR018
U 1 1 5795D90E
P 6150 6900
F 0 "#PWR018" H 6150 6650 50  0001 C CNN
F 1 "Earth" H 6150 6750 50  0001 C CNN
F 2 "" H 6150 6900 50  0000 C CNN
F 3 "" H 6150 6900 50  0000 C CNN
	1    6150 6900
	-1   0    0    1   
$EndComp
$Comp
L Earth #PWR019
U 1 1 5795D9A3
P 5500 6900
F 0 "#PWR019" H 5500 6650 50  0001 C CNN
F 1 "Earth" H 5500 6750 50  0001 C CNN
F 2 "" H 5500 6900 50  0000 C CNN
F 3 "" H 5500 6900 50  0000 C CNN
	1    5500 6900
	-1   0    0    1   
$EndComp
$Comp
L Earth #PWR020
U 1 1 5795DA38
P 5100 6900
F 0 "#PWR020" H 5100 6650 50  0001 C CNN
F 1 "Earth" H 5100 6750 50  0001 C CNN
F 2 "" H 5100 6900 50  0000 C CNN
F 3 "" H 5100 6900 50  0000 C CNN
	1    5100 6900
	-1   0    0    1   
$EndComp
$Comp
L Earth #PWR021
U 1 1 5795DACD
P 4400 6900
F 0 "#PWR021" H 4400 6650 50  0001 C CNN
F 1 "Earth" H 4400 6750 50  0001 C CNN
F 2 "" H 4400 6900 50  0000 C CNN
F 3 "" H 4400 6900 50  0000 C CNN
	1    4400 6900
	-1   0    0    1   
$EndComp
$Comp
L R R15
U 1 1 5795E069
P 4650 6500
F 0 "R15" V 4730 6500 50  0000 C CNN
F 1 "100K" V 4650 6500 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 4580 6500 50  0001 C CNN
F 3 "" H 4650 6500 50  0000 C CNN
	1    4650 6500
	0    1    1    0   
$EndComp
$Comp
L R R19
U 1 1 5795E4A4
P 5050 6500
F 0 "R19" V 5130 6500 50  0000 C CNN
F 1 "100K" V 5050 6500 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 4980 6500 50  0001 C CNN
F 3 "" H 5050 6500 50  0000 C CNN
	1    5050 6500
	0    1    1    0   
$EndComp
Text GLabel 6350 6900 1    60   Input ~ 0
PB6
Text GLabel 1350 6100 3    60   Input ~ 0
PB7
Text GLabel 6250 6900 1    60   Input ~ 0
PB7
Text GLabel 3500 5150 1    60   Input ~ 0
PA2
Text GLabel 3600 5150 1    60   Input ~ 0
PA1
Text GLabel 3700 5150 1    60   Input ~ 0
PA0
Text GLabel 4500 6050 1    60   Input ~ 0
PA0
Text GLabel 5600 6900 1    60   Input ~ 0
PA1
Text GLabel 5200 6050 1    60   Input ~ 0
PA2
Text Notes 6700 5700 0    60   ~ 0
Battery\nCurrent
Text GLabel 7050 5400 0    60   Input ~ 0
PA7
Text Notes 6050 3750 0    60   ~ 0
Charge\nVoltage\nLimit
Text Notes 1950 3650 0    60   ~ 0
Solar\nPanel\nCurrent
Text Notes 1900 2800 0    60   ~ 0
Charge\nEnable
Text Notes 1750 2050 0    60   ~ 0
Solar\nMeasurement\nEnable
$Comp
L Earth #PWR022
U 1 1 5796D696
P 7150 3150
F 0 "#PWR022" H 7150 2900 50  0001 C CNN
F 1 "Earth" H 7150 3000 50  0001 C CNN
F 2 "" H 7150 3150 50  0000 C CNN
F 3 "" H 7150 3150 50  0000 C CNN
	1    7150 3150
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR023
U 1 1 5796D731
P 7600 5700
F 0 "#PWR023" H 7600 5450 50  0001 C CNN
F 1 "Earth" H 7600 5550 50  0001 C CNN
F 2 "" H 7600 5700 50  0000 C CNN
F 3 "" H 7600 5700 50  0000 C CNN
	1    7600 5700
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR024
U 1 1 5796D7CC
P 2900 3550
F 0 "#PWR024" H 2900 3300 50  0001 C CNN
F 1 "Earth" H 2900 3400 50  0001 C CNN
F 2 "" H 2900 3550 50  0000 C CNN
F 3 "" H 2900 3550 50  0000 C CNN
	1    2900 3550
	1    0    0    -1  
$EndComp
$Comp
L JUMPER3 JP1
U 1 1 57973F71
P 6650 2750
F 0 "JP1" H 6700 2650 50  0000 L CNN
F 1 "JUMPER3" H 6650 2850 50  0000 C BNN
F 2 "WeatherStation:PINHEAD1-3" H 6650 2750 50  0001 C CNN
F 3 "" H 6650 2750 50  0000 C CNN
	1    6650 2750
	0    -1   1    0   
$EndComp
$Comp
L POT RV1
U 1 1 579744A7
P 6650 2350
F 0 "RV1" V 6500 2150 50  0000 C CNN
F 1 "100K" V 6500 2400 50  0000 C CNN
F 2 "WeatherStation:Potentiometer_Bourns_3339P_Angular_ScrewUp" H 6650 2350 50  0001 C CNN
F 3 "" H 6650 2350 50  0000 C CNN
	1    6650 2350
	0    -1   1    0   
$EndComp
$Comp
L Earth #PWR025
U 1 1 57974586
P 6900 2350
F 0 "#PWR025" H 6900 2100 50  0001 C CNN
F 1 "Earth" H 6900 2200 50  0001 C CNN
F 2 "" H 6900 2350 50  0000 C CNN
F 3 "" H 6900 2350 50  0000 C CNN
	1    6900 2350
	0    -1   1    0   
$EndComp
Text Notes 1950 5650 0    60   ~ 0
ET-STM32F103 STAMP Module
Text Notes 7500 7500 0    60   ~ 0
Weather Station
Text Notes 8350 7650 0    60   ~ 0
25/07/2016
Text Notes 10750 7650 0    60   ~ 0
0.0
NoConn ~ 3550 6100
NoConn ~ 3450 6100
NoConn ~ 3350 6100
NoConn ~ 3250 6100
NoConn ~ 3150 6100
NoConn ~ 3050 6100
NoConn ~ 2950 6100
NoConn ~ 2850 6100
NoConn ~ 2750 6100
NoConn ~ 2650 6100
NoConn ~ 2550 6100
NoConn ~ 2450 6100
NoConn ~ 2350 6100
NoConn ~ 2250 6100
NoConn ~ 1750 6100
NoConn ~ 1650 6100
NoConn ~ 1550 6100
NoConn ~ 1450 6100
NoConn ~ 2800 5150
NoConn ~ 2700 5150
NoConn ~ 2600 5150
NoConn ~ 2500 5150
NoConn ~ 2400 5150
NoConn ~ 2300 5150
NoConn ~ 2200 5150
NoConn ~ 2100 5150
NoConn ~ 2000 5150
NoConn ~ 1900 5150
NoConn ~ 1400 5150
NoConn ~ 10000 1500
NoConn ~ 3400 5150
$Comp
L PWR_FLAG #FLG026
U 1 1 5797995C
P 9400 1500
F 0 "#FLG026" H 9400 1595 50  0001 C CNN
F 1 "PWR_FLAG" H 9400 1680 50  0000 C CNN
F 2 "" H 9400 1500 50  0000 C CNN
F 3 "" H 9400 1500 50  0000 C CNN
	1    9400 1500
	1    0    0    -1  
$EndComp
$Comp
L MicroSD_Amph_114_00841_68 J1
U 1 1 5798C59E
P 1950 7200
F 0 "J1" H 1750 7900 60  0000 C CNN
F 1 "MicroSD" H 1750 6250 60  0000 C CNN
F 2 "WeatherStation:Conn_uSDcard_Amph_114_00841_68" H 2050 7350 60  0001 C CNN
F 3 "" H 2050 7350 60  0000 C CNN
	1    1950 7200
	0    -1   -1   0   
$EndComp
Text GLabel 2150 6100 3    60   Input ~ 0
PB15
Text GLabel 2050 6100 3    60   Input ~ 0
PB14
Text GLabel 1950 6100 3    60   Input ~ 0
PB13
Text GLabel 1850 6100 3    60   Input ~ 0
PB12
Text GLabel 1450 7000 1    60   Input ~ 0
PB12
Text GLabel 1750 7000 1    60   Input ~ 0
PB13
Text GLabel 1950 7000 1    60   Input ~ 0
PB14
Text GLabel 1550 7000 1    60   Input ~ 0
PB15
Text GLabel 3750 6850 0    60   Input ~ 0
PB12
Text GLabel 3750 7000 0    60   Input ~ 0
PB13
Text GLabel 3750 7150 0    60   Input ~ 0
PB14
Text GLabel 3750 7300 0    60   Input ~ 0
PB15
$Comp
L Earth #PWR027
U 1 1 579905E5
P 1850 7000
F 0 "#PWR027" H 1850 6750 50  0001 C CNN
F 1 "Earth" H 1850 6850 50  0001 C CNN
F 2 "" H 1850 7000 50  0000 C CNN
F 3 "" H 1850 7000 50  0000 C CNN
	1    1850 7000
	-1   0    0    1   
$EndComp
$Comp
L Earth #PWR028
U 1 1 57990837
P 2250 7000
F 0 "#PWR028" H 2250 6750 50  0001 C CNN
F 1 "Earth" H 2250 6850 50  0001 C CNN
F 2 "" H 2250 7000 50  0000 C CNN
F 3 "" H 2250 7000 50  0000 C CNN
	1    2250 7000
	-1   0    0    1   
$EndComp
$Comp
L Earth #PWR029
U 1 1 579908DE
P 2350 7000
F 0 "#PWR029" H 2350 6750 50  0001 C CNN
F 1 "Earth" H 2350 6850 50  0001 C CNN
F 2 "" H 2350 7000 50  0000 C CNN
F 3 "" H 2350 7000 50  0000 C CNN
	1    2350 7000
	-1   0    0    1   
$EndComp
$Comp
L Earth #PWR030
U 1 1 57990985
P 2450 7000
F 0 "#PWR030" H 2450 6750 50  0001 C CNN
F 1 "Earth" H 2450 6850 50  0001 C CNN
F 2 "" H 2450 7000 50  0000 C CNN
F 3 "" H 2450 7000 50  0000 C CNN
	1    2450 7000
	-1   0    0    1   
$EndComp
$Comp
L Earth #PWR031
U 1 1 57990A2C
P 2550 7000
F 0 "#PWR031" H 2550 6750 50  0001 C CNN
F 1 "Earth" H 2550 6850 50  0001 C CNN
F 2 "" H 2550 7000 50  0000 C CNN
F 3 "" H 2550 7000 50  0000 C CNN
	1    2550 7000
	-1   0    0    1   
$EndComp
Text GLabel 2050 7000 1    60   Input ~ 0
RSRVD
Text GLabel 3750 7450 0    60   Input ~ 0
RSRVD
$Comp
L 78M05 U2
U 1 1 5796D7C3
P 7150 1550
F 0 "U2" H 7300 1354 50  0000 C CNN
F 1 "78M05" H 7150 1750 50  0000 C CNN
F 2 "WeatherStation:TO-252-2Lead" H 7150 1550 50  0001 C CNN
F 3 "" H 7150 1550 50  0000 C CNN
	1    7150 1550
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 579728DF
P 3900 6850
F 0 "R10" V 3980 6850 50  0000 C CNN
F 1 "100K" V 3900 6850 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 3830 6850 50  0001 C CNN
F 3 "" H 3900 6850 50  0000 C CNN
	1    3900 6850
	0    1    1    0   
$EndComp
$Comp
L R R11
U 1 1 57972AB3
P 3900 7000
F 0 "R11" V 3980 7000 50  0000 C CNN
F 1 "100K" V 3900 7000 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 3830 7000 50  0001 C CNN
F 3 "" H 3900 7000 50  0000 C CNN
	1    3900 7000
	0    1    1    0   
$EndComp
$Comp
L R R12
U 1 1 57972B68
P 3900 7150
F 0 "R12" V 3980 7150 50  0000 C CNN
F 1 "100K" V 3900 7150 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 3830 7150 50  0001 C CNN
F 3 "" H 3900 7150 50  0000 C CNN
	1    3900 7150
	0    1    1    0   
$EndComp
$Comp
L R R13
U 1 1 57972C20
P 3900 7300
F 0 "R13" V 3980 7300 50  0000 C CNN
F 1 "100K" V 3900 7300 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 3830 7300 50  0001 C CNN
F 3 "" H 3900 7300 50  0000 C CNN
	1    3900 7300
	0    1    1    0   
$EndComp
$Comp
L R R14
U 1 1 57972CDF
P 3900 7450
F 0 "R14" V 3980 7450 50  0000 C CNN
F 1 "100K" V 3900 7450 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 3830 7450 50  0001 C CNN
F 3 "" H 3900 7450 50  0000 C CNN
	1    3900 7450
	0    1    1    0   
$EndComp
NoConn ~ 1350 7000
$Comp
L Earth #PWR032
U 1 1 57C198FD
P 2750 7000
F 0 "#PWR032" H 2750 6750 50  0001 C CNN
F 1 "Earth" H 2750 6850 50  0001 C CNN
F 2 "" H 2750 7000 50  0000 C CNN
F 3 "" H 2750 7000 50  0000 C CNN
	1    2750 7000
	-1   0    0    1   
$EndComp
$Comp
L R R26
U 1 1 57C19F76
P 3900 6700
F 0 "R26" V 3980 6700 50  0000 C CNN
F 1 "100K" V 3900 6700 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 3830 6700 50  0001 C CNN
F 3 "" H 3900 6700 50  0000 C CNN
	1    3900 6700
	0    1    1    0   
$EndComp
Text GLabel 3650 6100 3    60   Input ~ 0
PD2
Text GLabel 2650 7000 1    60   Input ~ 0
PD2
Text GLabel 3750 6700 0    60   Input ~ 0
PD2
$Comp
L Earth #PWR033
U 1 1 59EB8994
P 5700 4600
F 0 "#PWR033" H 5700 4350 50  0001 C CNN
F 1 "Earth" H 5700 4450 50  0001 C CNN
F 2 "" H 5700 4600 50  0000 C CNN
F 3 "" H 5700 4600 50  0000 C CNN
	1    5700 4600
	1    0    0    -1  
$EndComp
$Comp
L R R27
U 1 1 59EBCD84
P 5100 4450
F 0 "R27" V 5180 4450 50  0000 C CNN
F 1 "10K" V 5100 4450 50  0000 C CNN
F 2 "WeatherStation:SM0603_Resistor" V 5030 4450 50  0001 C CNN
F 3 "" H 5100 4450 50  0000 C CNN
	1    5100 4450
	1    0    0    -1  
$EndComp
Text GLabel 2250 4300 0    60   Input ~ 0
PB3
Text Notes 1850 4600 0    60   ~ 0
Sensor Power\nEnable
$Comp
L Earth #PWR034
U 1 1 59EBCEAA
P 5100 4600
F 0 "#PWR034" H 5100 4350 50  0001 C CNN
F 1 "Earth" H 5100 4450 50  0001 C CNN
F 2 "" H 5100 4600 50  0000 C CNN
F 3 "" H 5100 4600 50  0000 C CNN
	1    5100 4600
	1    0    0    -1  
$EndComp
Text GLabel 1800 5150 1    60   Input ~ 0
PB3
$Comp
L TPS77033 U4
U 1 1 59F0317C
P 5700 4100
F 0 "U4" H 5850 3704 60  0000 C CNN
F 1 "TPS77033" H 5900 4300 60  0000 C CNN
F 2 "WeatherStation:SOT23-5" H 5700 4100 60  0001 C CNN
F 3 "" H 5700 4100 60  0000 C CNN
	1    5700 4100
	1    0    0    -1  
$EndComp
Text GLabel 6450 6150 1    60   Input ~ 0
+3.3V
Text GLabel 5700 6150 1    60   Input ~ 0
+3.3V
Text GLabel 6650 4450 2    60   Input ~ 0
+3.3V
NoConn ~ 6000 4050
Text GLabel 8350 1500 3    60   Input ~ 0
Power
Text GLabel 5400 4050 0    60   Input ~ 0
Power
$Comp
L CP C5
U 1 1 59F0BDCB
P 6200 4450
F 0 "C5" H 6225 4550 50  0000 L CNN
F 1 "10uF" H 6225 4350 50  0000 L CNN
F 2 "WeatherStation:SM1206POL" H 6238 4300 50  0001 C CNN
F 3 "" H 6200 4450 50  0000 C CNN
	1    6200 4450
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR035
U 1 1 59F0C357
P 6200 4600
F 0 "#PWR035" H 6200 4350 50  0001 C CNN
F 1 "Earth" H 6200 4450 50  0001 C CNN
F 2 "" H 6200 4600 50  0000 C CNN
F 3 "" H 6200 4600 50  0000 C CNN
	1    6200 4600
	1    0    0    -1  
$EndComp
Text Notes 8600 2800 0    60   ~ 0
Battery\nVoltage
$Comp
L JUMPER3 JP2
U 1 1 59F80C61
P 6550 4450
F 0 "JP2" V 6650 4100 50  0000 L CNN
F 1 "JUMPER3" V 6750 4100 50  0000 C BNN
F 2 "WeatherStation:PINHEAD1-3" H 6550 4450 50  0001 C CNN
F 3 "" H 6550 4450 50  0000 C CNN
	1    6550 4450
	0    -1   1    0   
$EndComp
Text GLabel 6550 4700 3    60   Input ~ 0
+3.3V-M
$Comp
L LM324 U1
U 4 1 59F88D92
P 9900 5750
F 0 "U1" H 9950 5950 50  0000 C CNN
F 1 "LMV324" H 10200 6000 50  0000 C CNN
F 2 "WeatherStation:SOIC-14_N" H 9850 5850 50  0001 C CNN
F 3 "" H 9950 5950 50  0000 C CNN
	4    9900 5750
	-1   0    0    -1  
$EndComp
$Comp
L Earth #PWR036
U 1 1 59F89224
P 10000 6050
F 0 "#PWR036" H 10000 5800 50  0001 C CNN
F 1 "Earth" H 10000 5900 50  0001 C CNN
F 2 "" H 10000 6050 50  0000 C CNN
F 3 "" H 10000 6050 50  0000 C CNN
	1    10000 6050
	1    0    0    -1  
$EndComp
NoConn ~ 9600 5750
$Comp
L Earth #PWR037
U 1 1 5A067E16
P 2800 1250
F 0 "#PWR037" H 2800 1000 50  0001 C CNN
F 1 "Earth" H 2800 1100 50  0001 C CNN
F 2 "" H 2800 1250 50  0000 C CNN
F 3 "" H 2800 1250 50  0000 C CNN
	1    2800 1250
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR038
U 1 1 5A06817B
P 3100 1250
F 0 "#PWR038" H 3100 1000 50  0001 C CNN
F 1 "Earth" H 3100 1100 50  0001 C CNN
F 2 "" H 3100 1250 50  0000 C CNN
F 3 "" H 3100 1250 50  0000 C CNN
	1    3100 1250
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR039
U 1 1 5A0A7600
P 10250 2150
F 0 "#PWR039" H 10250 1900 50  0001 C CNN
F 1 "Earth" H 10250 2000 50  0001 C CNN
F 2 "" H 10250 2150 50  0000 C CNN
F 3 "" H 10250 2150 50  0000 C CNN
	1    10250 2150
	1    0    0    -1  
$EndComp
Text GLabel 7150 2550 2    60   Input ~ 0
+3.3V-M
Text GLabel 6400 2350 0    60   Input ~ 0
+3.3V-M
Text GLabel 10000 5450 1    60   Input ~ 0
+3.3V-M
Text GLabel 7600 5100 1    60   Input ~ 0
+3.3V-M
Text GLabel 2900 2950 2    60   Input ~ 0
+3.3V-M
Text GLabel 4050 6500 1    60   Input ~ 0
+3.3V-M
Text GLabel 4850 6150 1    60   Input ~ 0
+3.3V-M
Text GLabel 1650 7000 1    60   Input ~ 0
+3.3V-M
$Comp
L Earth #PWR040
U 1 1 5A430765
P 10200 6050
F 0 "#PWR040" H 10200 5800 50  0001 C CNN
F 1 "Earth" H 10200 5900 50  0001 C CNN
F 2 "" H 10200 6050 50  0000 C CNN
F 3 "" H 10200 6050 50  0000 C CNN
	1    10200 6050
	1    0    0    -1  
$EndComp
NoConn ~ 2900 5150
$Comp
L Earth #PWR042
U 1 1 57947726
P 2500 2000
F 0 "#PWR042" H 2500 1750 50  0001 C CNN
F 1 "Earth" H 2500 1850 50  0001 C CNN
F 2 "" H 2500 2000 50  0000 C CNN
F 3 "" H 2500 2000 50  0000 C CNN
	1    2500 2000
	1    0    0    -1  
$EndComp
Connection ~ 7200 5400
Wire Wire Line
	7050 5400 7200 5400
Connection ~ 8050 5300
Wire Wire Line
	7800 5300 8050 5300
Wire Wire Line
	8050 2550 8050 5300
Wire Wire Line
	8050 5300 8050 5800
Wire Wire Line
	8050 5800 8050 5950
Connection ~ 8050 1500
Wire Wire Line
	8050 2450 8050 1500
Connection ~ 2500 3250
Wire Wire Line
	2250 3250 2500 3250
Connection ~ 4750 3150
Wire Wire Line
	3100 3150 4750 3150
Connection ~ 6650 3350
Wire Wire Line
	6750 2750 6950 2750
Connection ~ 7550 2850
Connection ~ 7850 2850
Wire Wire Line
	7550 2850 7850 2850
Wire Wire Line
	7800 5500 7800 5850
Connection ~ 7200 5850
Wire Wire Line
	7800 5850 7200 5850
Wire Wire Line
	7200 5700 7200 5850
Wire Wire Line
	7200 5850 7200 5950
Connection ~ 2500 3700
Wire Wire Line
	3100 3700 2500 3700
Wire Wire Line
	3100 3350 3100 3700
Wire Wire Line
	2500 3550 2500 3700
Wire Wire Line
	2500 3700 2500 3800
Wire Wire Line
	6950 3350 6950 2950
Wire Wire Line
	7550 3350 6950 3350
Wire Wire Line
	7550 2850 7550 3350
Wire Wire Line
	10000 1650 10250 1650
Wire Wire Line
	10250 1650 10500 1650
Wire Wire Line
	7850 2050 7850 2850
Wire Wire Line
	7850 2850 7850 3350
Connection ~ 4750 900 
Connection ~ 6350 900 
Wire Wire Line
	6750 900  6750 1100
Wire Wire Line
	2250 2550 2500 2550
Wire Wire Line
	2500 2550 2800 2550
Wire Wire Line
	6450 1300 6350 1300
Connection ~ 6050 1500
Wire Wire Line
	6050 1100 6050 1500
Connection ~ 6350 1300
Connection ~ 4200 900 
Wire Wire Line
	4750 1500 4750 3150
Wire Wire Line
	4750 3150 4750 3750
Connection ~ 3400 900 
Wire Wire Line
	3400 900  3400 1200
Wire Wire Line
	2550 900  2800 900 
Wire Wire Line
	2800 900  3100 900 
Wire Wire Line
	3100 900  3400 900 
Wire Wire Line
	3400 900  4200 900 
Wire Wire Line
	4200 900  4750 900 
Wire Wire Line
	4750 900  5550 900 
Wire Wire Line
	5550 900  6350 900 
Wire Wire Line
	6350 900  6750 900 
Connection ~ 2550 1100
Wire Wire Line
	2550 1000 2550 1100
Wire Wire Line
	2550 1100 2550 1250
Connection ~ 2500 1700
Wire Wire Line
	2250 1700 2500 1700
Wire Wire Line
	2500 1700 2800 1700
Wire Wire Line
	3400 1500 3700 1500
Wire Wire Line
	3700 1500 3900 1500
Wire Wire Line
	4450 1300 4200 1300
Connection ~ 3900 1500
Wire Wire Line
	3900 1500 3900 1100
Connection ~ 3400 1500
Connection ~ 4200 1300
Wire Wire Line
	8350 5950 8350 5800
Wire Wire Line
	8350 5800 8050 5800
Connection ~ 8050 5800
Wire Wire Line
	10250 1850 10250 1650
Connection ~ 10250 1650
Wire Wire Line
	8700 1500 8700 1850
Connection ~ 8700 1500
Wire Wire Line
	5550 1050 5550 900 
Connection ~ 5550 900 
Wire Wire Line
	5250 2550 3100 2550
Wire Wire Line
	4500 6050 4500 6500
Wire Wire Line
	4500 6500 4500 6900
Wire Wire Line
	5200 6050 5200 6500
Wire Wire Line
	5200 6500 5200 6900
Connection ~ 4850 6500
Connection ~ 5200 6500
Connection ~ 4500 6500
Wire Wire Line
	4750 900  4750 1100
Wire Wire Line
	6650 3000 6650 3350
Wire Wire Line
	6650 3350 6350 3350
Wire Wire Line
	9050 1800 9050 2500
Wire Wire Line
	9050 2500 9050 2800
Wire Wire Line
	8850 2500 9050 2500
Connection ~ 9050 2500
Wire Wire Line
	6500 2350 6400 2350
Wire Wire Line
	6800 2350 6900 2350
Connection ~ 2500 2550
Connection ~ 3100 900 
Connection ~ 2800 900 
Connection ~ 9050 1500
Connection ~ 9400 1500
Wire Wire Line
	4050 6500 4050 6700
Wire Wire Line
	4050 6700 4050 6850
Wire Wire Line
	4050 6850 4050 7000
Wire Wire Line
	4050 7000 4050 7150
Wire Wire Line
	4050 7150 4050 7300
Wire Wire Line
	4050 7300 4050 7450
Connection ~ 4050 6850
Connection ~ 4050 7000
Connection ~ 4050 7150
Connection ~ 4050 7300
Connection ~ 4050 6700
Connection ~ 5100 4300
Wire Wire Line
	2250 4300 5100 4300
Wire Wire Line
	5100 4300 5400 4300
Wire Wire Line
	6450 6150 6450 6900
Wire Wire Line
	4800 6500 4850 6500
Wire Wire Line
	4850 6500 4900 6500
Wire Wire Line
	4850 6500 4850 6150
Wire Wire Line
	7150 1800 7150 2050
Wire Wire Line
	7150 2050 7850 2050
Wire Wire Line
	5700 6150 5700 6900
Wire Wire Line
	6000 4200 6200 4200
Wire Wire Line
	6200 4200 6550 4200
Wire Wire Line
	6200 4300 6200 4200
Connection ~ 6200 4200
Wire Wire Line
	5700 4600 5700 4550
Wire Wire Line
	10200 5650 10200 5850
Wire Wire Line
	10200 5850 10200 6050
Connection ~ 10200 5850
Wire Wire Line
	7850 1500 8050 1500
Wire Wire Line
	8050 1500 8700 1500
Wire Wire Line
	8700 1500 9050 1500
Wire Wire Line
	9050 1500 9400 1500
Wire Wire Line
	2800 1250 2800 1200
Wire Wire Line
	3100 1250 3100 1200
Connection ~ 5550 2350
Wire Wire Line
	5250 2150 3700 2150
Wire Wire Line
	3700 2150 3700 1500
Connection ~ 3700 1500
Wire Wire Line
	5550 1950 5550 1500
Wire Wire Line
	5550 1500 5550 1350
Wire Wire Line
	6050 1500 5550 1500
Connection ~ 5550 1500
Text GLabel 2500 2250 2    60   Input ~ 0
+3.3V-M
$EndSCHEMATC
