EESchema Schematic File Version 4
LIBS:PCB_C-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
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
L SamacSys_Parts:BMX055 IC5
U 1 1 5BB71FEC
P 5250 3050
F 0 "IC5" H 5850 3315 50  0000 C CNN
F 1 "BMX055" H 5850 3224 50  0000 C CNN
F 2 "SamacSys_Parts:LGA_PACKAGE_20_PINS" H 6300 3150 50  0001 L CNN
F 3 "http://www.mouser.com/ds/2/621/BST-BMX055-DS000-02-586421.pdf" H 6300 3050 50  0001 L CNN
F 4 "Small versatile 9-axis sensor module" H 6300 2950 50  0001 L CNN "Description"
F 5 "" H 6300 2850 50  0001 L CNN "Height"
F 6 "Bosch Sensortec" H 6300 2750 50  0001 L CNN "Manufacturer_Name"
F 7 "BMX055" H 6300 2650 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 6300 2550 50  0001 L CNN "RS Part Number"
F 9 "" H 6300 2450 50  0001 L CNN "RS Price/Stock"
F 10 "BMX055" H 6300 2350 50  0001 L CNN "Arrow Part Number"
F 11 "" H 6300 2250 50  0001 L CNN "Arrow Price/Stock"
	1    5250 3050
	1    0    0    -1  
$EndComp
Text HLabel 6600 3050 2    50   BiDi ~ 0
SDA
Text HLabel 5050 3850 0    50   Input ~ 0
SCL
Text HLabel 5050 3050 0    50   Output ~ 0
ACC_INT
Text HLabel 5050 3950 0    50   Output ~ 0
MAG_INT
Text HLabel 6500 3350 2    50   Output ~ 0
GYRO_INT
$Comp
L power:GND #PWR073
U 1 1 5BB72101
P 4300 3550
F 0 "#PWR073" H 4300 3300 50  0001 C CNN
F 1 "GND" H 4305 3377 50  0000 C CNN
F 2 "" H 4300 3550 50  0001 C CNN
F 3 "" H 4300 3550 50  0001 C CNN
	1    4300 3550
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR072
U 1 1 5BB72133
P 4300 3050
F 0 "#PWR072" H 4300 2900 50  0001 C CNN
F 1 "+3V3" H 4315 3223 50  0000 C CNN
F 2 "" H 4300 3050 50  0001 C CNN
F 3 "" H 4300 3050 50  0001 C CNN
	1    4300 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3050 5250 3050
Wire Wire Line
	4300 3050 4300 3250
Connection ~ 4300 3250
Wire Wire Line
	4300 3250 4300 3300
Wire Wire Line
	4300 3500 4300 3550
Wire Wire Line
	5250 3350 5200 3350
Wire Wire Line
	4650 3350 4650 3550
$Comp
L power:GND #PWR074
U 1 1 5BB7221A
P 4650 3550
F 0 "#PWR074" H 4650 3300 50  0001 C CNN
F 1 "GND" H 4655 3377 50  0000 C CNN
F 2 "" H 4650 3550 50  0001 C CNN
F 3 "" H 4650 3550 50  0001 C CNN
	1    4650 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3550 5200 3550
Wire Wire Line
	5200 3550 5200 3350
Connection ~ 5200 3350
Wire Wire Line
	5200 3350 4650 3350
$Comp
L power:+3V3 #PWR075
U 1 1 5BB72267
P 5000 3600
F 0 "#PWR075" H 5000 3450 50  0001 C CNN
F 1 "+3V3" H 5015 3773 50  0000 C CNN
F 2 "" H 5000 3600 50  0001 C CNN
F 3 "" H 5000 3600 50  0001 C CNN
	1    5000 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3650 5000 3650
Wire Wire Line
	5000 3650 5000 3600
Wire Wire Line
	5250 3850 5050 3850
Wire Wire Line
	5050 3950 5250 3950
$Comp
L power:GND #PWR080
U 1 1 5BB7256E
P 7400 4050
F 0 "#PWR080" H 7400 3800 50  0001 C CNN
F 1 "GND" H 7405 3877 50  0000 C CNN
F 2 "" H 7400 4050 50  0001 C CNN
F 3 "" H 7400 4050 50  0001 C CNN
	1    7400 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4050 7400 3150
$Comp
L power:+3V3 #PWR078
U 1 1 5BB726DE
P 7050 3050
F 0 "#PWR078" H 7050 2900 50  0001 C CNN
F 1 "+3V3" H 7065 3223 50  0000 C CNN
F 2 "" H 7050 3050 50  0001 C CNN
F 3 "" H 7050 3050 50  0001 C CNN
	1    7050 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3050 7050 3250
$Comp
L power:GND #PWR079
U 1 1 5BB72CBF
P 7050 4050
F 0 "#PWR079" H 7050 3800 50  0001 C CNN
F 1 "GND" H 7055 3877 50  0000 C CNN
F 2 "" H 7050 4050 50  0001 C CNN
F 3 "" H 7050 4050 50  0001 C CNN
	1    7050 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4050 7050 3450
Wire Wire Line
	6450 3050 6600 3050
Wire Wire Line
	6450 3250 7050 3250
Wire Wire Line
	6500 3350 6450 3350
$Comp
L power:GND #PWR077
U 1 1 5BB7368C
P 6850 4050
F 0 "#PWR077" H 6850 3800 50  0001 C CNN
F 1 "GND" H 6855 3877 50  0000 C CNN
F 2 "" H 6850 4050 50  0001 C CNN
F 3 "" H 6850 4050 50  0001 C CNN
	1    6850 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 4050 6850 3650
Wire Wire Line
	6850 3650 6450 3650
Wire Wire Line
	6450 3950 6650 3950
Wire Wire Line
	6650 3950 6650 4050
$Comp
L power:GND #PWR076
U 1 1 5BB741F0
P 6650 4050
F 0 "#PWR076" H 6650 3800 50  0001 C CNN
F 1 "GND" H 6655 3877 50  0000 C CNN
F 2 "" H 6650 4050 50  0001 C CNN
F 3 "" H 6650 4050 50  0001 C CNN
	1    6650 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3250 5250 3250
$Comp
L Device:C_Small C?
U 1 1 5BCC83DC
P 4300 3400
AR Path="/5BB122CA/5BCC83DC" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5BCC83DC" Ref="C24"  Part="1" 
F 0 "C24" H 4392 3446 50  0000 L CNN
F 1 "100nF" H 4392 3355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4300 3400 50  0001 C CNN
F 3 "~" H 4300 3400 50  0001 C CNN
F 4 "EMK107B7104KAHT" H 4300 3400 50  0001 C CNN "Manufacturer Part Number"
	1    4300 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5BCC861B
P 7050 3350
AR Path="/5BB122CA/5BCC861B" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5BCC861B" Ref="C25"  Part="1" 
F 0 "C25" H 7142 3396 50  0000 L CNN
F 1 "100nF" H 7142 3305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7050 3350 50  0001 C CNN
F 3 "~" H 7050 3350 50  0001 C CNN
F 4 "EMK107B7104KAHT" H 7050 3350 50  0001 C CNN "Manufacturer Part Number"
	1    7050 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3150 7400 3150
Connection ~ 7050 3250
$EndSCHEMATC
