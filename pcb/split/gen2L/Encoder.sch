EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 4 5
Title "RoMeLa Motor Gen2 Logic Board"
Date "10/27/2022"
Rev "1.0"
Comp "Robotics & Mechanisms Labratory(RoMeLa) UCLA"
Comment1 "Author: Tym Zhu"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 4250 3750 0    60   Input ~ 0
CSn
Text HLabel 4250 3350 0    60   Input ~ 0
CLK
Text HLabel 4250 3550 0    60   Output ~ 0
MISO
Text HLabel 4250 3650 0    60   Input ~ 0
MOSI
$Comp
L Motor_Gen2L-rescue:GND-power #PWR094
U 1 1 5E23DD79
P 2700 4250
F 0 "#PWR094" H 2700 4000 50  0001 C CNN
F 1 "GND" H 2700 4100 50  0000 C CNN
F 2 "" H 2700 4250 50  0000 C CNN
F 3 "" H 2700 4250 50  0000 C CNN
	1    2700 4250
	1    0    0    -1  
$EndComp
Text HLabel 1850 3950 0    60   Input ~ 0
BATT+
Text HLabel 1850 4200 0    60   Input ~ 0
BATT-
Wire Wire Line
	2700 4250 2700 4200
Text Label 4650 4800 2    60   ~ 0
SCLK
$Comp
L Motor_Gen2L-rescue:Conn_01x06-Connector_Generic P?
U 1 1 5E99C8B7
P 4850 4900
AR Path="/5827F7AC/5E99C8B7" Ref="P?"  Part="1" 
AR Path="/5829B004/5E99C8B7" Ref="P10"  Part="1" 
F 0 "P10" H 4850 5250 50  0000 C CNN
F 1 "Port_Encoder" V 4950 4900 50  0000 C CNN
F 2 "RoMeLa_Motor:JST_SH_BM06B-SRSS-TB_1x06-1MP_P1.00mm_Vertical" H 4850 4900 50  0001 C CNN
F 3 "" H 4850 4900 50  0000 C CNN
	1    4850 4900
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:GND-power #PWR?
U 1 1 5E99C8BD
P 4150 4900
AR Path="/5827F7AC/5E99C8BD" Ref="#PWR?"  Part="1" 
AR Path="/5829B004/5E99C8BD" Ref="#PWR099"  Part="1" 
F 0 "#PWR099" H 4150 4650 50  0001 C CNN
F 1 "GND" H 4150 4750 50  0000 C CNN
F 2 "" H 4150 4900 50  0000 C CNN
F 3 "" H 4150 4900 50  0000 C CNN
	1    4150 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4600 4450 4700
Wire Wire Line
	4450 4700 4650 4700
Text Label 4650 5200 2    60   ~ 0
NCS
Text Label 4650 5100 2    60   ~ 0
MISO
Wire Wire Line
	4150 4900 4650 4900
Text Label 4650 5000 2    60   ~ 0
MOSI
Wire Wire Line
	1850 4200 2700 4200
$Comp
L Motor_Gen2L-rescue:+5V-power #PWR0121
U 1 1 5E2BEDC3
P 4450 4600
F 0 "#PWR0121" H 4450 4450 50  0001 C CNN
F 1 "+5V" H 4465 4773 50  0000 C CNN
F 2 "" H 4450 4600 50  0001 C CNN
F 3 "" H 4450 4600 50  0001 C CNN
	1    4450 4600
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2L-rescue:Conn_01x08-Connector_Generic J2
U 1 1 63533FC5
P 4850 3650
F 0 "J2" H 4800 4050 50  0000 L CNN
F 1 "Port_MU" H 4930 3551 50  0000 L CNN
F 2 "RoMeLa_Motor:FFC_8POS_0.5mm_BackFlip" H 4850 3650 50  0001 C CNN
F 3 "~" H 4850 3650 50  0001 C CNN
	1    4850 3650
	1    0    0    1   
$EndComp
$Comp
L Motor_Gen2L-rescue:+5V-power #PWR0124
U 1 1 635376DF
P 4450 3200
F 0 "#PWR0124" H 4450 3050 50  0001 C CNN
F 1 "+5V" H 4465 3373 50  0000 C CNN
F 2 "" H 4450 3200 50  0001 C CNN
F 3 "" H 4450 3200 50  0001 C CNN
	1    4450 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 3200 4450 3250
Wire Wire Line
	4450 3250 4650 3250
Text Label 4650 3350 2    60   ~ 0
SCLK
$Comp
L Motor_Gen2L-rescue:GND-power #PWR?
U 1 1 6353884F
P 3800 3450
AR Path="/5827F7AC/6353884F" Ref="#PWR?"  Part="1" 
AR Path="/5829B004/6353884F" Ref="#PWR0125"  Part="1" 
F 0 "#PWR0125" H 3800 3200 50  0001 C CNN
F 1 "GND" H 3800 3300 50  0000 C CNN
F 2 "" H 3800 3450 50  0000 C CNN
F 3 "" H 3800 3450 50  0000 C CNN
	1    3800 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 3450 4650 3450
Text Label 4650 3550 2    60   ~ 0
MISO
Text Label 4650 3650 2    60   ~ 0
MOSI
Text Label 4650 3750 2    60   ~ 0
NCS
Text Label 4650 3850 2    60   ~ 0
PRESET
Text Label 4650 3950 2    60   ~ 0
VBATT
Wire Wire Line
	4650 3750 4250 3750
Wire Wire Line
	4250 3350 4650 3350
Wire Wire Line
	4250 3650 4650 3650
Wire Wire Line
	4250 3550 4650 3550
Text Notes 4800 2950 0    60   ~ 0
MOLEX 5034800800\nFlipped from Encoder board!
Wire Wire Line
	1850 3950 4650 3950
$EndSCHEMATC
