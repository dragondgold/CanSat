EESchema Schematic File Version 4
LIBS:PCB_B-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 6
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
L Regulator_Switching:ADP2108AUJ-3.3 U?
U 1 1 5BBA6889
P 4650 4400
AR Path="/5BC13C37/5BBA6889" Ref="U?"  Part="1" 
AR Path="/5BC14C3C/5BBA6889" Ref="U?"  Part="1" 
F 0 "U?" H 4650 4725 50  0000 C CNN
F 1 "ADP2108AUJ-3.3" H 4650 4634 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TSOT-23-5" H 4700 4150 50  0001 L CNN
F 3 "http://www.analog.com/media/en/technical-documentation/data-sheets/ADP2108.pdf" H 4400 4050 50  0001 C CNN
	1    4650 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4300 3550 4300
Wire Wire Line
	3550 4300 3550 4450
$Comp
L power:GND #PWR?
U 1 1 5BBA8A01
P 3550 4800
AR Path="/5BC13C37/5BBA8A01" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5BBA8A01" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3550 4550 50  0001 C CNN
F 1 "GND" H 3555 4627 50  0000 C CNN
F 2 "" H 3550 4800 50  0001 C CNN
F 3 "" H 3550 4800 50  0001 C CNN
	1    3550 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5BBA8A27
P 4650 4800
AR Path="/5BC13C37/5BBA8A27" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5BBA8A27" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4650 4550 50  0001 C CNN
F 1 "GND" H 4655 4627 50  0000 C CNN
F 2 "" H 4650 4800 50  0001 C CNN
F 3 "" H 4650 4800 50  0001 C CNN
	1    4650 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4650 3550 4800
Wire Wire Line
	4650 4800 4650 4700
$Comp
L Device:L_Core_Iron L?
U 1 1 5BBA8BBD
P 5150 4300
AR Path="/5BC13C37/5BBA8BBD" Ref="L?"  Part="1" 
AR Path="/5BC14C3C/5BBA8BBD" Ref="L?"  Part="1" 
F 0 "L?" V 5375 4300 50  0000 C CNN
F 1 "1uH" V 5284 4300 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5150 4300 50  0001 C CNN
F 3 "~" H 5150 4300 50  0001 C CNN
F 4 "LQM21PN1R0MC0D" V 5150 4300 50  0001 C CNN "Manufacturer Part Number"
	1    5150 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5000 4300 4950 4300
Wire Wire Line
	6100 4300 6100 4400
$Comp
L power:GND #PWR?
U 1 1 5BBA8CC8
P 6100 4800
AR Path="/5BC13C37/5BBA8CC8" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5BBA8CC8" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6100 4550 50  0001 C CNN
F 1 "GND" H 6105 4627 50  0000 C CNN
F 2 "" H 6100 4800 50  0001 C CNN
F 3 "" H 6100 4800 50  0001 C CNN
	1    6100 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4600 6100 4800
Text HLabel 3400 4300 0    50   Input ~ 0
VBAT
Text HLabel 7050 4300 2    50   Output ~ 0
3V3
Wire Wire Line
	3550 4300 3400 4300
Connection ~ 3550 4300
Text HLabel 4100 4400 0    50   Input ~ 0
CTRL
Wire Wire Line
	4100 4400 4350 4400
Wire Wire Line
	5850 4500 5850 4300
Connection ~ 5850 4300
Wire Wire Line
	5850 4300 6100 4300
Wire Wire Line
	5300 4300 5350 4300
Wire Wire Line
	4950 4500 5850 4500
Connection ~ 6100 4300
$Comp
L Amplifier_Current:INA194 U?
U 1 1 5BBAC7D2
P 5600 2500
AR Path="/5BC13C37/5BBAC7D2" Ref="U?"  Part="1" 
AR Path="/5BC14C3C/5BBAC7D2" Ref="U?"  Part="1" 
F 0 "U?" V 5646 2159 50  0000 R CNN
F 1 "INA194" V 5555 2159 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 5600 2500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ina193.pdf" H 5600 2500 50  0001 C CNN
	1    5600 2500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5BBACA67
P 5600 4300
AR Path="/5BC13C37/5BBACA67" Ref="R?"  Part="1" 
AR Path="/5BC14C3C/5BBACA67" Ref="R?"  Part="1" 
F 0 "R?" V 5303 4300 50  0000 C CNN
F 1 "120m" V 5394 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5530 4300 50  0001 C CNN
F 3 "~" H 5600 4300 50  0001 C CNN
F 4 "1%" V 5485 4300 50  0000 C CNN "Tolerance"
F 5 "KRL1632E-M-R120-F-T5" V 5600 4300 50  0001 C CNN "Manufacturer Part Number"
	1    5600 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 4300 5850 4300
$Comp
L Device:R R?
U 1 1 5BBACF91
P 5350 3700
AR Path="/5BC13C37/5BBACF91" Ref="R?"  Part="1" 
AR Path="/5BC14C3C/5BBACF91" Ref="R?"  Part="1" 
F 0 "R?" H 5280 3609 50  0000 R CNN
F 1 "51R" H 5280 3700 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5280 3700 50  0001 C CNN
F 3 "~" H 5350 3700 50  0001 C CNN
F 4 "1%" H 5280 3791 50  0000 R CNN "Tolerance"
F 5 "AC0603FR-0751RL" V 5350 3700 50  0001 C CNN "Manufacturer Part Number"
	1    5350 3700
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 5BBACFF0
P 5850 3700
AR Path="/5BC13C37/5BBACFF0" Ref="R?"  Part="1" 
AR Path="/5BC14C3C/5BBACFF0" Ref="R?"  Part="1" 
F 0 "R?" H 5780 3609 50  0000 R CNN
F 1 "51R" H 5780 3700 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5780 3700 50  0001 C CNN
F 3 "~" H 5850 3700 50  0001 C CNN
F 4 "1%" H 5780 3791 50  0000 R CNN "Tolerance"
F 5 "AC0603FR-0751RL" V 5850 3700 50  0001 C CNN "Manufacturer Part Number"
	1    5850 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	5350 3850 5350 4300
Connection ~ 5350 4300
Wire Wire Line
	5350 4300 5450 4300
Wire Wire Line
	5850 3850 5850 4300
Wire Wire Line
	5700 3200 5850 3200
Wire Wire Line
	5850 3200 5850 3550
Wire Wire Line
	5500 3200 5350 3200
Wire Wire Line
	5350 3200 5350 3550
Wire Wire Line
	5350 3200 5350 2900
Wire Wire Line
	5350 2900 5500 2900
Wire Wire Line
	5500 2900 5500 2800
Connection ~ 5350 3200
Wire Wire Line
	5700 2800 5700 2900
Wire Wire Line
	5700 2900 5850 2900
Wire Wire Line
	5850 2900 5850 3200
Connection ~ 5850 3200
$Comp
L AXTEC_IC:+3V3_Internal #PWR?
U 1 1 5BBB2E44
P 4450 2500
AR Path="/5BC13C37/5BBB2E44" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5BBB2E44" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4450 2900 50  0001 C CNN
F 1 "+3V3_Internal" H 4435 2673 50  0000 C CNN
F 2 "" H 4450 2500 50  0001 C CNN
F 3 "" H 4450 2500 50  0001 C CNN
	1    4450 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2500 4450 2600
Wire Wire Line
	4450 2600 4600 2600
Wire Wire Line
	4950 2700 4950 2600
Connection ~ 4950 2600
Wire Wire Line
	4950 2600 5300 2600
Wire Wire Line
	4600 2700 4600 2600
Connection ~ 4600 2600
Wire Wire Line
	4600 2600 4950 2600
$Comp
L power:GND #PWR?
U 1 1 5BBB3B4A
P 4600 2900
AR Path="/5BC13C37/5BBB3B4A" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5BBB3B4A" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4600 2650 50  0001 C CNN
F 1 "GND" H 4605 2727 50  0000 C CNN
F 2 "" H 4600 2900 50  0001 C CNN
F 3 "" H 4600 2900 50  0001 C CNN
	1    4600 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5BBB3B6B
P 4950 2900
AR Path="/5BC13C37/5BBB3B6B" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5BBB3B6B" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4950 2650 50  0001 C CNN
F 1 "GND" H 4955 2727 50  0000 C CNN
F 2 "" H 4950 2900 50  0001 C CNN
F 3 "" H 4950 2900 50  0001 C CNN
	1    4950 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5BBB3D93
P 6150 2850
AR Path="/5BC13C37/5BBB3D93" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5BBB3D93" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6150 2600 50  0001 C CNN
F 1 "GND" H 6155 2677 50  0000 C CNN
F 2 "" H 6150 2850 50  0001 C CNN
F 3 "" H 6150 2850 50  0001 C CNN
	1    6150 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 2850 6150 2600
Wire Wire Line
	6150 2600 5900 2600
Text HLabel 5850 1950 2    50   Output ~ 0
ISENSE
Wire Wire Line
	5850 1950 5600 1950
Wire Wire Line
	5600 1950 5600 2200
$Comp
L Device:R R?
U 1 1 5C270D6B
P 6700 4500
AR Path="/5BC13C37/5C270D6B" Ref="R?"  Part="1" 
AR Path="/5BC14C3C/5C270D6B" Ref="R?"  Part="1" 
F 0 "R?" H 6630 4408 50  0000 R CNN
F 1 "2k" H 6630 4500 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6630 4500 50  0001 C CNN
F 3 "~" H 6700 4500 50  0001 C CNN
F 4 "1%" H 6630 4592 50  0000 R CNN "Tolerance"
F 5 "RC0603FR-102KL" V 6700 4500 50  0001 C CNN "Manufacturer Part Number"
	1    6700 4500
	-1   0    0    1   
$EndComp
Wire Wire Line
	6100 4300 6700 4300
Wire Wire Line
	6700 4350 6700 4300
Connection ~ 6700 4300
Wire Wire Line
	6700 4300 7050 4300
$Comp
L Device:R R?
U 1 1 5C2719F0
P 6700 5000
AR Path="/5BC13C37/5C2719F0" Ref="R?"  Part="1" 
AR Path="/5BC14C3C/5C2719F0" Ref="R?"  Part="1" 
F 0 "R?" H 6630 4908 50  0000 R CNN
F 1 "130k" H 6630 5000 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6630 5000 50  0001 C CNN
F 3 "~" H 6700 5000 50  0001 C CNN
F 4 "1%" H 6630 5092 50  0000 R CNN "Tolerance"
F 5 "RC0603FR-07150KL" V 6700 5000 50  0001 C CNN "Manufacturer Part Number"
	1    6700 5000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C272082
P 6700 5200
AR Path="/5BC13C37/5C272082" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5C272082" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6700 4950 50  0001 C CNN
F 1 "GND" H 6705 5027 50  0000 C CNN
F 2 "" H 6700 5200 50  0001 C CNN
F 3 "" H 6700 5200 50  0001 C CNN
	1    6700 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5150 6700 5200
Text HLabel 8800 4850 2    50   Output ~ 0
VSENSE
$Comp
L Amplifier_Operational:MCP6001-OT U?
U 1 1 5C273454
P 8000 4850
AR Path="/5BC13C37/5C273454" Ref="U?"  Part="1" 
AR Path="/5BC14C3C/5C273454" Ref="U?"  Part="1" 
F 0 "U?" H 8050 4650 50  0000 L CNN
F 1 "MCP6001-OT" H 8050 4550 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 7900 4650 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 8000 5050 50  0001 C CNN
	1    8000 4850
	1    0    0    -1  
$EndComp
$Comp
L AXTEC_IC:+3V3_Internal #PWR?
U 1 1 5C2735D2
P 7900 4050
AR Path="/5BC13C37/5C2735D2" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5C2735D2" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7900 4450 50  0001 C CNN
F 1 "+3V3_Internal" H 7885 4223 50  0000 C CNN
F 2 "" H 7900 4050 50  0001 C CNN
F 3 "" H 7900 4050 50  0001 C CNN
	1    7900 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C273865
P 8100 4250
AR Path="/5BB122CA/5C273865" Ref="C?"  Part="1" 
AR Path="/5BB12371/5C273865" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5C273865" Ref="C?"  Part="1" 
AR Path="/5BB6C3A1/5C273865" Ref="C?"  Part="1" 
AR Path="/5BC13C37/5C273865" Ref="C?"  Part="1" 
AR Path="/5BC14C3C/5C273865" Ref="C?"  Part="1" 
F 0 "C?" H 8008 4204 50  0000 R CNN
F 1 "1uF" H 8008 4295 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8100 4250 50  0001 C CNN
F 3 "~" H 8100 4250 50  0001 C CNN
F 4 "10V" H 8100 4250 50  0001 C CNN "Voltage"
F 5 "LMK107B7105MA-T" H 8100 4250 50  0001 C CNN "Manufacturer Part Number"
	1    8100 4250
	-1   0    0    1   
$EndComp
Wire Wire Line
	7700 4950 7400 4950
Wire Wire Line
	7400 4950 7400 5400
Wire Wire Line
	7400 5400 8650 5400
Wire Wire Line
	8650 5400 8650 4850
Wire Wire Line
	8650 4850 8300 4850
$Comp
L power:GND #PWR?
U 1 1 5C274292
P 7900 5150
AR Path="/5BC13C37/5C274292" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5C274292" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7900 4900 50  0001 C CNN
F 1 "GND" H 7905 4977 50  0000 C CNN
F 2 "" H 7900 5150 50  0001 C CNN
F 3 "" H 7900 5150 50  0001 C CNN
	1    7900 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 4050 7900 4150
Wire Wire Line
	8450 4150 8100 4150
Connection ~ 7900 4150
Wire Wire Line
	7900 4150 7900 4550
Connection ~ 8100 4150
Wire Wire Line
	8100 4150 7900 4150
$Comp
L power:GND #PWR?
U 1 1 5C275DA7
P 8100 4350
AR Path="/5BC13C37/5C275DA7" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5C275DA7" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 8100 4100 50  0001 C CNN
F 1 "GND" H 8105 4177 50  0000 C CNN
F 2 "" H 8100 4350 50  0001 C CNN
F 3 "" H 8100 4350 50  0001 C CNN
	1    8100 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C275DD2
P 8450 4350
AR Path="/5BC13C37/5C275DD2" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5C275DD2" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 8450 4100 50  0001 C CNN
F 1 "GND" H 8455 4177 50  0000 C CNN
F 2 "" H 8450 4350 50  0001 C CNN
F 3 "" H 8450 4350 50  0001 C CNN
	1    8450 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 4850 8650 4850
Connection ~ 8650 4850
Wire Wire Line
	6700 4650 6700 4750
Wire Wire Line
	6700 4750 7050 4750
Connection ~ 6700 4750
Wire Wire Line
	6700 4750 6700 4850
Wire Wire Line
	7050 4900 7050 4750
Connection ~ 7050 4750
Wire Wire Line
	7050 4750 7700 4750
$Comp
L power:GND #PWR?
U 1 1 5C28697B
P 7050 5200
AR Path="/5BC13C37/5C28697B" Ref="#PWR?"  Part="1" 
AR Path="/5BC14C3C/5C28697B" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7050 4950 50  0001 C CNN
F 1 "GND" H 7055 5027 50  0000 C CNN
F 2 "" H 7050 5200 50  0001 C CNN
F 3 "" H 7050 5200 50  0001 C CNN
	1    7050 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 5200 7050 5100
Text Notes 6600 5550 0    50   ~ 0
fc = 795 Hz
$Comp
L Device:C_Small C?
U 1 1 5C68D48B
P 3550 4550
AR Path="/5BB122CA/5C68D48B" Ref="C?"  Part="1" 
AR Path="/5BB12371/5C68D48B" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5C68D48B" Ref="C?"  Part="1" 
AR Path="/5BB6C3A1/5C68D48B" Ref="C?"  Part="1" 
AR Path="/5BB6C83B/5C68D48B" Ref="C?"  Part="1" 
AR Path="/5BC14C3C/5C68D48B" Ref="C?"  Part="1" 
AR Path="/5C68D48B" Ref="C?"  Part="1" 
AR Path="/5BC13C37/5C68D48B" Ref="C?"  Part="1" 
F 0 "C?" H 3458 4504 50  0000 R CNN
F 1 "10uF" H 3458 4595 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3550 4550 50  0001 C CNN
F 3 "~" H 3550 4550 50  0001 C CNN
F 4 "10V" H 3550 4550 50  0001 C CNN "Voltage"
F 5 "C2012X7R1A106M125AC" H 3550 4550 50  0001 C CNN "Manufacturer Part Number"
	1    3550 4550
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C68E0D1
P 6100 4500
AR Path="/5BB122CA/5C68E0D1" Ref="C?"  Part="1" 
AR Path="/5BB12371/5C68E0D1" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5C68E0D1" Ref="C?"  Part="1" 
AR Path="/5BB6C3A1/5C68E0D1" Ref="C?"  Part="1" 
AR Path="/5BB6C83B/5C68E0D1" Ref="C?"  Part="1" 
AR Path="/5BC14C3C/5C68E0D1" Ref="C?"  Part="1" 
AR Path="/5C68E0D1" Ref="C?"  Part="1" 
AR Path="/5BC13C37/5C68E0D1" Ref="C?"  Part="1" 
F 0 "C?" H 6008 4454 50  0000 R CNN
F 1 "10uF" H 6008 4545 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6100 4500 50  0001 C CNN
F 3 "~" H 6100 4500 50  0001 C CNN
F 4 "10V" H 6100 4500 50  0001 C CNN "Voltage"
F 5 "C2012X7R1A106M125AC" H 6100 4500 50  0001 C CNN "Manufacturer Part Number"
	1    6100 4500
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C68E3B8
P 7050 5000
AR Path="/5BB122CA/5C68E3B8" Ref="C?"  Part="1" 
AR Path="/5BB12371/5C68E3B8" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5C68E3B8" Ref="C?"  Part="1" 
AR Path="/5BB6C3A1/5C68E3B8" Ref="C?"  Part="1" 
AR Path="/5BB6C83B/5C68E3B8" Ref="C?"  Part="1" 
AR Path="/5BC14C3C/5C68E3B8" Ref="C?"  Part="1" 
AR Path="/5C68E3B8" Ref="C?"  Part="1" 
AR Path="/5BC13C37/5C68E3B8" Ref="C?"  Part="1" 
F 0 "C?" H 6958 4954 50  0000 R CNN
F 1 "100nF" H 6958 5045 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7050 5000 50  0001 C CNN
F 3 "~" H 7050 5000 50  0001 C CNN
F 4 "10V" H 7050 5000 50  0001 C CNN "Voltage"
F 5 "EMK107B7104KAHT" H 7050 5000 50  0001 C CNN "Manufacturer Part Number"
	1    7050 5000
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C69023C
P 5600 3200
AR Path="/5BB122CA/5C69023C" Ref="C?"  Part="1" 
AR Path="/5BB12371/5C69023C" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5C69023C" Ref="C?"  Part="1" 
AR Path="/5BB6C3A1/5C69023C" Ref="C?"  Part="1" 
AR Path="/5BC13C37/5C69023C" Ref="C?"  Part="1" 
AR Path="/5BC14C3C/5C69023C" Ref="C?"  Part="1" 
F 0 "C?" H 5508 3154 50  0000 R CNN
F 1 "1uF" H 5508 3245 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5600 3200 50  0001 C CNN
F 3 "~" H 5600 3200 50  0001 C CNN
F 4 "10V" H 5600 3200 50  0001 C CNN "Voltage"
F 5 "LMK107B7105MA-T" H 5600 3200 50  0001 C CNN "Manufacturer Part Number"
	1    5600 3200
	0    1    -1   0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C6903C3
P 4950 2800
AR Path="/5BB122CA/5C6903C3" Ref="C?"  Part="1" 
AR Path="/5BB12371/5C6903C3" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5C6903C3" Ref="C?"  Part="1" 
AR Path="/5BB6C3A1/5C6903C3" Ref="C?"  Part="1" 
AR Path="/5BC13C37/5C6903C3" Ref="C?"  Part="1" 
AR Path="/5BC14C3C/5C6903C3" Ref="C?"  Part="1" 
F 0 "C?" H 4858 2754 50  0000 R CNN
F 1 "1uF" H 4858 2845 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4950 2800 50  0001 C CNN
F 3 "~" H 4950 2800 50  0001 C CNN
F 4 "10V" H 4950 2800 50  0001 C CNN "Voltage"
F 5 "LMK107B7105MA-T" H 4950 2800 50  0001 C CNN "Manufacturer Part Number"
	1    4950 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C6933E3
P 8450 4250
AR Path="/5BB122CA/5C6933E3" Ref="C?"  Part="1" 
AR Path="/5BB12371/5C6933E3" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5C6933E3" Ref="C?"  Part="1" 
AR Path="/5BB6C3A1/5C6933E3" Ref="C?"  Part="1" 
AR Path="/5BB6C83B/5C6933E3" Ref="C?"  Part="1" 
AR Path="/5BC14C3C/5C6933E3" Ref="C?"  Part="1" 
AR Path="/5C6933E3" Ref="C?"  Part="1" 
AR Path="/5BC13C37/5C6933E3" Ref="C?"  Part="1" 
F 0 "C?" H 8358 4204 50  0000 R CNN
F 1 "100nF" H 8358 4295 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8450 4250 50  0001 C CNN
F 3 "~" H 8450 4250 50  0001 C CNN
F 4 "10V" H 8450 4250 50  0001 C CNN "Voltage"
F 5 "EMK107B7104KAHT" H 8450 4250 50  0001 C CNN "Manufacturer Part Number"
	1    8450 4250
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C693F77
P 4600 2800
AR Path="/5BB122CA/5C693F77" Ref="C?"  Part="1" 
AR Path="/5BB12371/5C693F77" Ref="C?"  Part="1" 
AR Path="/5BB123AC/5C693F77" Ref="C?"  Part="1" 
AR Path="/5BB6C3A1/5C693F77" Ref="C?"  Part="1" 
AR Path="/5BB6C83B/5C693F77" Ref="C?"  Part="1" 
AR Path="/5BC14C3C/5C693F77" Ref="C?"  Part="1" 
AR Path="/5C693F77" Ref="C?"  Part="1" 
AR Path="/5BC13C37/5C693F77" Ref="C?"  Part="1" 
F 0 "C?" H 4508 2754 50  0000 R CNN
F 1 "100nF" H 4508 2845 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4600 2800 50  0001 C CNN
F 3 "~" H 4600 2800 50  0001 C CNN
F 4 "10V" H 4600 2800 50  0001 C CNN "Voltage"
F 5 "EMK107B7104KAHT" H 4600 2800 50  0001 C CNN "Manufacturer Part Number"
	1    4600 2800
	1    0    0    1   
$EndComp
$EndSCHEMATC
