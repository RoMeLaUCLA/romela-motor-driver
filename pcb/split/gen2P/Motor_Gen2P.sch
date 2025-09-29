EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 4
Title "RoMeLa Motor Gen2 Power Board"
Date "10/21/2022"
Rev "1.0"
Comp "Robotics & Mechanisms Labratory(RoMeLa) UCLA"
Comment1 "Author: Tym Zhu"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 6400 1900 1500 1750
U 58280BEC
F0 "Power_Stage" 60
F1 "Power_Stage.sch" 60
F2 "ENGATE" I L 6400 2150 60 
F3 "H1" I L 6400 2300 60 
F4 "H2" I L 6400 2400 60 
F5 "H3" I L 6400 2500 60 
F6 "SCS" I L 6400 2700 60 
F7 "SCLK" I L 6400 2800 60 
F8 "SDI" I L 6400 2900 60 
F9 "SDO" O L 6400 3000 60 
F10 "FAULT" O L 6400 3150 60 
F11 "MOTOR_A" O R 7900 2150 60 
F12 "MOTOR_B" O R 7900 2350 60 
F13 "MOTOR_C" O R 7900 2550 60 
F14 "Shunt_A_P" O R 7900 3050 60 
F15 "Shunt_A_N" O R 7900 3150 60 
F16 "Shunt_C_P" O R 7900 3350 60 
F17 "Shunt_C_N" O R 7900 3450 60 
F18 "PVDDSENSE" O L 6400 3400 60 
F19 "DRV_ENABLE" I L 6400 2050 60 
$EndSheet
$Sheet
S 6400 3950 1500 1300
U 5828F3AC
F0 "Current_ADC" 60
F1 "Current_ADC.sch" 60
F2 "AIN+" I R 7900 4450 60 
F3 "AIN-" I R 7900 4350 60 
F4 "CIN+" I R 7900 4200 60 
F5 "CIN-" I R 7900 4100 60 
F6 "DOUT" O L 6400 4400 60 
F7 "DIN" I L 6400 4200 60 
F8 "CLK" I L 6400 4600 60 
F9 "~CS" I L 6400 4800 60 
$EndSheet
$Comp
L Motor_Gen2P-rescue:TestPoint-Connector P3
U 1 1 582AB0F8
P 7900 2150
F 0 "P3" V 7800 2300 50  0000 C CNN
F 1 "MT_A" V 7900 2450 50  0000 C CNN
F 2 "RoMeLa_Motor:motor_winding_oval_3mm" H 8100 2150 50  0001 C CNN
F 3 "" H 8100 2150 50  0000 C CNN
	1    7900 2150
	0    1    1    0   
$EndComp
$Comp
L Motor_Gen2P-rescue:TestPoint-Connector P4
U 1 1 582ABC63
P 7900 2350
F 0 "P4" V 7800 2500 50  0000 C CNN
F 1 "MT_B" V 7900 2650 50  0000 C CNN
F 2 "RoMeLa_Motor:motor_winding_oval_3mm" H 8100 2350 50  0001 C CNN
F 3 "" H 8100 2350 50  0000 C CNN
	1    7900 2350
	0    1    1    0   
$EndComp
$Comp
L Motor_Gen2P-rescue:TestPoint-Connector P2
U 1 1 582ABE29
P 7900 2550
F 0 "P2" V 7800 2700 50  0000 C CNN
F 1 "MT_C" V 7900 2850 50  0000 C CNN
F 2 "RoMeLa_Motor:motor_winding_oval_3mm" H 8100 2550 50  0001 C CNN
F 3 "" H 8100 2550 50  0000 C CNN
	1    7900 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 2150 6400 2150
Wire Wire Line
	6000 2300 6400 2300
Wire Wire Line
	6400 2400 6000 2400
Wire Wire Line
	6000 2500 6400 2500
Wire Wire Line
	6400 2700 6000 2700
Wire Wire Line
	6000 2800 6400 2800
Wire Wire Line
	6400 2900 6000 2900
Wire Wire Line
	6000 3000 6400 3000
Wire Wire Line
	6000 3150 6400 3150
Wire Wire Line
	5950 3400 6400 3400
Wire Wire Line
	7900 3150 8100 3150
Wire Wire Line
	8100 3150 8100 4450
Wire Wire Line
	8150 4350 8150 3050
Wire Wire Line
	8150 3050 7900 3050
Wire Wire Line
	7900 4100 7950 4100
Wire Wire Line
	7950 4100 7950 3450
Wire Wire Line
	7950 3450 7900 3450
Wire Wire Line
	7900 3350 8000 3350
Wire Wire Line
	8000 3350 8000 4200
Wire Wire Line
	8000 4200 7900 4200
Wire Wire Line
	6400 4400 6000 4400
Wire Wire Line
	6000 4600 6400 4600
$Comp
L Motor_Gen2P-rescue:+48V-power #PWR01
U 1 1 5832C1E8
P 8900 1450
F 0 "#PWR01" H 8900 1300 50  0001 C CNN
F 1 "+48V" H 8900 1590 50  0000 C CNN
F 2 "" H 8900 1450 50  0000 C CNN
F 3 "" H 8900 1450 50  0000 C CNN
	1    8900 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4350 7900 4350
Wire Wire Line
	8100 4450 7900 4450
$Comp
L Motor_Gen2P-rescue:GNDPWR-power #PWR02
U 1 1 5832E244
P 8900 1550
F 0 "#PWR02" H 8900 1350 50  0001 C CNN
F 1 "GNDPWR" H 8900 1420 50  0000 C CNN
F 2 "" H 8900 1500 50  0000 C CNN
F 3 "" H 8900 1500 50  0000 C CNN
	1    8900 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4200 6400 4200
Wire Wire Line
	6000 4800 6400 4800
$Comp
L Motor_Gen2P-rescue:GNDPWR-power #PWR0113
U 1 1 5E0E9B26
P 2050 6300
F 0 "#PWR0113" H 2050 6100 50  0001 C CNN
F 1 "GNDPWR" H 2050 6170 50  0000 C CNN
F 2 "" H 2050 6250 50  0000 C CNN
F 3 "" H 2050 6250 50  0000 C CNN
	1    2050 6300
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:MountingHole_Pad-Mechanical H1
U 1 1 5E0FA87A
P 2050 5550
F 0 "H1" H 2150 5599 50  0000 L CNN
F 1 "MT" H 2150 5508 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via" H 2050 5550 50  0001 C CNN
F 3 "~" H 2050 5550 50  0001 C CNN
	1    2050 5550
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:TestPoint-Connector P1
U 1 1 5E2D6D9C
P 8900 1450
F 0 "P1" V 8850 1650 50  0000 C CNN
F 1 "+IN" V 8900 1750 50  0000 C CNN
F 2 "RoMeLa_Motor:Motor_winding_SMT" H 9100 1450 50  0001 C CNN
F 3 "" H 9100 1450 50  0000 C CNN
	1    8900 1450
	0    1    1    0   
$EndComp
$Comp
L Motor_Gen2P-rescue:TestPoint-Connector P11
U 1 1 5E2D7645
P 8900 1550
F 0 "P11" V 8950 1750 50  0000 C CNN
F 1 "-IN" V 8900 1850 50  0000 C CNN
F 2 "RoMeLa_Motor:Motor_winding_SMT" H 9100 1550 50  0001 C CNN
F 3 "" H 9100 1550 50  0000 C CNN
	1    8900 1550
	0    1    1    0   
$EndComp
$Comp
L Motor_Gen2P-rescue:MountingHole_Pad-Mechanical H2
U 1 1 5E2E12CF
P 2350 5550
F 0 "H2" H 2450 5599 50  0000 L CNN
F 1 "MT" H 2450 5508 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via" H 2350 5550 50  0001 C CNN
F 3 "~" H 2350 5550 50  0001 C CNN
	1    2350 5550
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:MountingHole_Pad-Mechanical H3
U 1 1 5E2E15B2
P 2650 5600
F 0 "H3" H 2750 5649 50  0000 L CNN
F 1 "MT" H 2750 5558 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via" H 2650 5600 50  0001 C CNN
F 3 "~" H 2650 5600 50  0001 C CNN
	1    2650 5600
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:MountingHole_Pad-Mechanical H4
U 1 1 5E2E1805
P 2950 5550
F 0 "H4" H 3050 5599 50  0000 L CNN
F 1 "MT" H 3050 5508 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via" H 2950 5550 50  0001 C CNN
F 3 "~" H 2950 5550 50  0001 C CNN
	1    2950 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 5700 2650 5700
Connection ~ 2650 5700
$Comp
L Motor_Gen2P-rescue:C_Small-Device C?
U 1 1 5E2FC9E6
P 2050 6000
AR Path="/58280BEC/5E2FC9E6" Ref="C?"  Part="1" 
AR Path="/5E2FC9E6" Ref="C26"  Part="1" 
F 0 "C26" H 2150 6050 50  0000 L CNN
F 1 "4.7nF 1kv" H 2150 5950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2050 6000 50  0001 C CNN
F 3 "" H 2050 6000 50  0000 C CNN
	1    2050 6000
	1    0    0    -1  
$EndComp
Text Notes 2150 6200 0    60   ~ 0
C0805W472KDRACTU
$Comp
L Motor_Gen2P-rescue:Earth_Protective-power #PWR0118
U 1 1 5E1D66A0
P 2950 5700
F 0 "#PWR0118" H 3200 5450 50  0001 C CNN
F 1 "Earth_Protective" H 3400 5550 50  0001 C CNN
F 2 "" H 2950 5600 50  0001 C CNN
F 3 "~" H 2950 5600 50  0001 C CNN
	1    2950 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 5700 2050 5900
Connection ~ 2050 5700
$Comp
L Motor_Gen2P-rescue:R-Device R38
U 1 1 5E1EE096
P 1700 6000
F 0 "R38" H 1770 6046 50  0000 L CNN
F 1 "1M" H 1770 5955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1630 6000 50  0001 C CNN
F 3 "~" H 1700 6000 50  0001 C CNN
	1    1700 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5850 1700 5700
Wire Wire Line
	1700 5700 2050 5700
Wire Wire Line
	1700 6150 1700 6250
Wire Wire Line
	1700 6250 2050 6250
Connection ~ 2050 6250
Wire Wire Line
	2050 6250 2050 6300
Wire Wire Line
	2050 6100 2050 6250
Wire Wire Line
	5900 2050 6400 2050
$Comp
L Motor_Gen2P-rescue:GND-power #PWR0101
U 1 1 633EBE00
P 4200 4350
F 0 "#PWR0101" H 4200 4100 50  0001 C CNN
F 1 "GND" H 4350 4300 50  0000 C CNN
F 2 "" H 4200 4350 50  0001 C CNN
F 3 "" H 4200 4350 50  0001 C CNN
	1    4200 4350
	1    0    0    -1  
$EndComp
$Comp
L Motor_Gen2P-rescue:+3.3V-power #PWR0102
U 1 1 633EC400
P 3850 4550
F 0 "#PWR0102" H 3850 4400 50  0001 C CNN
F 1 "+3.3V" H 4000 4600 50  0000 C CNN
F 2 "" H 3850 4550 50  0001 C CNN
F 3 "" H 3850 4550 50  0001 C CNN
	1    3850 4550
	1    0    0    -1  
$EndComp
$Sheet
S 6400 5550 900  650 
U 635D2F02
F0 "Thermal" 50
F1 "Thermal.sch" 50
F2 "I2C_SDA" B L 6400 5750 50 
F3 "I2C_SCL" I L 6400 5950 50 
$EndSheet
$Comp
L Motor_Gen2P-rescue:Conn_01x21-Connector_Generic J1
U 1 1 636885CE
P 3500 3650
F 0 "J1" H 3418 2425 50  0000 C CNN
F 1 "Conn_01x21" H 3418 2516 50  0000 C CNN
F 2 "RoMeLa_Motor:Molex_5019122190_2Rows-21Pins_P0.30mm" H 3500 3650 50  0001 C CNN
F 3 "~" H 3500 3650 50  0001 C CNN
	1    3500 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	2050 5700 2650 5700
Text Label 6000 3150 0    50   ~ 0
FAULT
Text Label 3700 2850 0    50   ~ 0
FAULT
Text Label 5900 2050 0    50   ~ 0
DRV_ENABLE
Text Label 6000 2150 0    50   ~ 0
ENGATE
Text Label 6000 2300 0    50   ~ 0
H1
Text Label 6000 2400 0    50   ~ 0
H2
Text Label 6000 2500 0    50   ~ 0
H3
Text Label 6000 2700 0    50   ~ 0
SCS
Text Label 6000 2800 0    50   ~ 0
SCLK
Text Label 6000 2900 0    50   ~ 0
SDI
Text Label 6000 3000 0    50   ~ 0
SDO
Text Label 5950 3400 0    50   ~ 0
PVDDSENSE
Text Label 6000 4200 0    50   ~ 0
DIN
Text Label 6000 4400 0    50   ~ 0
DOUT
Text Label 6000 4600 0    50   ~ 0
CLK
Text Label 6000 4800 0    50   ~ 0
~CS
Text Label 6050 5750 0    50   ~ 0
I2C_SDA
Text Label 6050 5950 0    50   ~ 0
I2C_SCL
Wire Wire Line
	6050 5950 6400 5950
Wire Wire Line
	6400 5750 6050 5750
Text Label 3700 3050 0    50   ~ 0
SDO
Text Label 3700 3250 0    50   ~ 0
SDI
Text Label 3700 3450 0    50   ~ 0
SCLK
Text Label 3700 3650 0    50   ~ 0
SCS
Text Label 3700 3850 0    50   ~ 0
DRV_ENABLE
Text Label 3700 4650 0    50   ~ 0
ENGATE
Text Label 3700 4050 0    50   ~ 0
H1
Text Label 3700 4250 0    50   ~ 0
H2
Text Label 3700 4450 0    50   ~ 0
H3
Text Label 3700 2650 0    50   ~ 0
PVDDSENSE
Text Label 3700 3950 0    50   ~ 0
DIN
Text Label 3700 3350 0    50   ~ 0
DOUT
Text Label 3700 3550 0    50   ~ 0
CLK
Text Label 3700 3750 0    50   ~ 0
~CS
Text Label 3700 3150 0    50   ~ 0
I2C_SDA
Text Label 3700 2950 0    50   ~ 0
I2C_SCL
$Comp
L Motor_Gen2P-rescue:LOGO_Small-RoMeLa_Motor G1
U 1 1 63626CD5
P 1000 7400
F 0 "G1" H 1000 7540 50  0001 C CNN
F 1 "ARTEMIS" H 1000 7250 50  0000 C CNN
F 2 "RoMeLa_Motor:artemis_logo_small" H 1000 7225 50  0001 C CNN
F 3 "~" H 1030 7200 50  0001 C CNN
	1    1000 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 4550 3850 4550
Wire Wire Line
	3700 4350 4200 4350
$Comp
L Motor_Gen2P-rescue:+5V-power #PWR0107
U 1 1 6350D26C
P 3850 4150
F 0 "#PWR0107" H 3850 4000 50  0001 C CNN
F 1 "+5V" H 3950 4250 50  0000 C CNN
F 2 "" H 3850 4150 50  0001 C CNN
F 3 "" H 3850 4150 50  0001 C CNN
	1    3850 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 4150 3700 4150
$EndSCHEMATC
