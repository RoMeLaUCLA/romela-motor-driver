EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 3 5
Title "RoMeLa Motor Gen2 Logic Board"
Date "10/27/2022"
Rev "1.0"
Comp "Robotics & Mechanisms Labratory(RoMeLa) UCLA"
Comment1 "Author: Tym Zhu"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Motor_Gen2L-rescue:MAX31855KASA-Sensor_Temperature U?
U 1 1 582A0766
P 2800 2300
AR Path="/582A0766" Ref="U?"  Part="1" 
AR Path="/5829AC99/582A0766" Ref="U9"  Part="1" 
F 0 "U9" H 2450 2700 50  0000 L CNN
F 1 "MAX31855KASA" H 2900 1950 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2800 2300 50  0001 C CIN
F 3 "" H 2800 2300 50  0000 C CNN
	1    2800 2300
	1    0    0    -1  
$EndComp
Text HLabel 1450 2100 0    60   Input ~ 0
T+
Text HLabel 1450 2500 0    60   Input ~ 0
T-
$Comp
L Motor_Gen2L-rescue:C_Small-Device C52
U 1 1 582A07EC
P 2050 2300
F 0 "C52" H 2060 2370 50  0000 L CNN
F 1 "103" H 2060 2220 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2050 2300 50  0001 C CNN
F 3 "" H 2050 2300 50  0000 C CNN
	1    2050 2300
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:C_Small-Device C50
U 1 1 582A0842
P 3000 1600
F 0 "C50" H 3010 1670 50  0000 L CNN
F 1 "104" H 3010 1520 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3000 1600 50  0001 C CNN
F 3 "" H 3000 1600 50  0000 C CNN
	1    3000 1600
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:GND-power #PWR080
U 1 1 582A0891
P 3000 1700
F 0 "#PWR080" H 3000 1450 50  0001 C CNN
F 1 "GND" H 3000 1550 50  0000 C CNN
F 2 "" H 3000 1700 50  0000 C CNN
F 3 "" H 3000 1700 50  0000 C CNN
	1    3000 1700
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:GND-power #PWR082
U 1 1 582A08AC
P 2800 2700
F 0 "#PWR082" H 2800 2450 50  0001 C CNN
F 1 "GND" H 2800 2550 50  0000 C CNN
F 2 "" H 2800 2700 50  0000 C CNN
F 3 "" H 2800 2700 50  0000 C CNN
	1    2800 2700
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:+3.3V-power #PWR079
U 1 1 582A08C8
P 2800 1450
F 0 "#PWR079" H 2800 1300 50  0001 C CNN
F 1 "+3.3V" H 2800 1590 50  0000 C CNN
F 2 "" H 2800 1450 50  0000 C CNN
F 3 "" H 2800 1450 50  0000 C CNN
	1    2800 1450
	1    0    0    -1  
$EndComp
Text HLabel 3200 2400 2    60   Input ~ 0
TC_nCS
Text HLabel 3200 2100 2    60   Input ~ 0
TC_SCK
Text HLabel 3200 2200 2    60   Output ~ 0
TC_SO
Wire Wire Line
	2800 1900 2800 1450
Wire Wire Line
	2800 1450 3000 1450
Wire Wire Line
	3000 1450 3000 1500
$Comp
L Motor_Gen2L-rescue:Ferrite_Bead-Device FB?
U 1 1 58324B84
P 1700 2100
AR Path="/58324B84" Ref="FB?"  Part="1" 
AR Path="/5829AC99/58324B84" Ref="FB1"  Part="1" 
F 0 "FB1" V 1800 2100 50  0000 L CNN
F 1 "Ferrite_Bead" V 1800 2050 50  0001 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1700 2100 50  0001 C CNN
F 3 "" H 1700 2100 50  0000 C CNN
	1    1700 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1450 2500 1550 2500
Wire Wire Line
	1550 2100 1450 2100
Connection ~ 2800 1450
$Comp
L Motor_Gen2L-rescue:C_Small-Device C53
U 1 1 5E1EF5FC
P 2050 2700
F 0 "C53" H 2060 2770 50  0000 L CNN
F 1 "103" H 2060 2620 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2050 2700 50  0001 C CNN
F 3 "" H 2050 2700 50  0000 C CNN
	1    2050 2700
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:C_Small-Device C51
U 1 1 5E1EF984
P 2050 1900
F 0 "C51" H 2060 1970 50  0000 L CNN
F 1 "103" H 2060 1820 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2050 1900 50  0001 C CNN
F 3 "" H 2050 1900 50  0000 C CNN
	1    2050 1900
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:Ferrite_Bead-Device FB?
U 1 1 58324C3A
P 1700 2500
AR Path="/58324C3A" Ref="FB?"  Part="1" 
AR Path="/5829AC99/58324C3A" Ref="FB2"  Part="1" 
F 0 "FB2" V 1800 2500 50  0000 L CNN
F 1 "Ferrite_Bead" V 1800 2450 50  0001 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1700 2500 50  0001 C CNN
F 3 "" H 1700 2500 50  0000 C CNN
	1    1700 2500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2050 2400 2050 2500
Wire Wire Line
	2400 2400 2350 2400
Wire Wire Line
	2350 2400 2350 2500
Wire Wire Line
	2350 2500 2050 2500
Connection ~ 2050 2500
Wire Wire Line
	2050 2500 2050 2600
Wire Wire Line
	1850 2500 2050 2500
Wire Wire Line
	1850 2100 2050 2100
Wire Wire Line
	2350 2100 2350 2200
Wire Wire Line
	2350 2200 2400 2200
Wire Wire Line
	2050 2200 2050 2100
Connection ~ 2050 2100
Wire Wire Line
	2050 2100 2350 2100
Wire Wire Line
	2050 2000 2050 2100
$Comp
L Motor_Gen2L-rescue:GND-power #PWR083
U 1 1 5E1F5B47
P 2050 2800
F 0 "#PWR083" H 2050 2550 50  0001 C CNN
F 1 "GND" H 2050 2650 50  0000 C CNN
F 2 "" H 2050 2800 50  0000 C CNN
F 3 "" H 2050 2800 50  0000 C CNN
	1    2050 2800
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:GND-power #PWR081
U 1 1 5E1F6041
P 2050 1800
F 0 "#PWR081" H 2050 1550 50  0001 C CNN
F 1 "GND" H 2050 1650 50  0000 C CNN
F 2 "" H 2050 1800 50  0000 C CNN
F 3 "" H 2050 1800 50  0000 C CNN
	1    2050 1800
	-1   0    0    1   
$EndComp
$EndSCHEMATC
