EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 4 4
Title "RoMeLa Motor Gen2 Power Board"
Date "10/21/2022"
Rev "1.0"
Comp "Robotics & Mechanisms Labratory(RoMeLa) UCLA"
Comment1 "Author: Tym Zhu"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 5500 3400 0    60   BiDi ~ 0
I2C_SDA
Text HLabel 5500 3550 0    60   Input ~ 0
I2C_SCL
Text Label 6350 3400 2    60   ~ 0
SDA
Text Label 6350 3550 2    60   ~ 0
SCL
$Comp
L Motor_Gen2P-rescue:GND-power #PWR0103
U 1 1 582AA1F9
P 6900 4100
F 0 "#PWR0103" H 6900 3850 50  0001 C CNN
F 1 "GND" H 6900 3950 50  0000 C CNN
F 2 "" H 6900 4100 50  0000 C CNN
F 3 "" H 6900 4100 50  0000 C CNN
	1    6900 4100
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:GND-power #PWR0104
U 1 1 582AA270
P 7500 3900
F 0 "#PWR0104" H 7500 3650 50  0001 C CNN
F 1 "GND" H 7500 3750 50  0000 C CNN
F 2 "" H 7500 3900 50  0000 C CNN
F 3 "" H 7500 3900 50  0000 C CNN
	1    7500 3900
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:+3.3V-power #PWR0105
U 1 1 582AA2D4
P 7750 3400
F 0 "#PWR0105" H 7750 3250 50  0001 C CNN
F 1 "+3.3V" H 7750 3540 50  0000 C CNN
F 2 "" H 7750 3400 50  0000 C CNN
F 3 "" H 7750 3400 50  0000 C CNN
	1    7750 3400
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:C_Small-Device C54
U 1 1 582AA2EB
P 7750 3500
F 0 "C54" H 7760 3570 50  0000 L CNN
F 1 "104" H 7760 3420 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7750 3500 50  0001 C CNN
F 3 "" H 7750 3500 50  0000 C CNN
	1    7750 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3400 7750 3400
$Comp
L Motor_Gen2P-rescue:GND-power #PWR0106
U 1 1 582AA3A4
P 7750 3600
F 0 "#PWR0106" H 7750 3350 50  0001 C CNN
F 1 "GND" H 7750 3450 50  0000 C CNN
F 2 "" H 7750 3600 50  0000 C CNN
F 3 "" H 7750 3600 50  0000 C CNN
	1    7750 3600
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:TMP112-RoMeLa_Motor U10
U 1 1 5E1F7DC4
P 6900 3550
F 0 "U10" H 6600 3950 60  0000 C CNN
F 1 "TMP112" H 7050 3950 60  0000 C CNN
F 2 "RoMeLa_Motor:SOT-563" H 6900 3550 60  0001 C CNN
F 3 "" H 6900 3550 60  0001 C CNN
	1    6900 3550
	1    0    0    -1  
$EndComp
NoConn ~ 6350 3700
Wire Wire Line
	7450 3700 7500 3700
Wire Wire Line
	7500 3700 7500 3900
Connection ~ 7750 3400
Text Notes 7600 3950 0    60   ~ 0
ADDR: 1001000
Wire Wire Line
	5500 3550 6350 3550
Wire Wire Line
	5500 3400 6350 3400
$EndSCHEMATC
