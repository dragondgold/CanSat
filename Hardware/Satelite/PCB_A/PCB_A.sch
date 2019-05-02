EESchema Schematic File Version 4
LIBS:PCB_A-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title "PCB_A"
Date "2019-01-19"
Rev "1.0.0"
Comp "AXTEC Ingeniería"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x03 J5
U 1 1 5BA1A8A6
P 9600 5050
F 0 "J5" H 9680 5092 50  0000 L CNN
F 1 "Servo Paracaídas" H 9680 5001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 9600 5050 50  0001 C CNN
F 3 "~" H 9600 5050 50  0001 C CNN
F 4 "61300311121" H 9600 5050 50  0001 C CNN "Manufacturer Part Number"
	1    9600 5050
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 5BA1A8E2
P 8150 5050
F 0 "J3" H 8230 5092 50  0000 L CNN
F 1 "Servo Globo" H 8230 5001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 8150 5050 50  0001 C CNN
F 3 "~" H 8150 5050 50  0001 C CNN
F 4 "61300311121" H 8150 5050 50  0001 C CNN "Manufacturer Part Number"
	1    8150 5050
	0    -1   -1   0   
$EndComp
$Sheet
S 1150 5350 550  200 
U 5BA1A7F9
F0 "GPS" 50
F1 "GPS.sch" 50
F2 "TX" O R 1700 5450 50 
$EndSheet
Wire Wire Line
	1550 3350 2000 3350
Wire Wire Line
	1550 3450 2000 3450
Wire Wire Line
	1550 3550 2000 3550
Wire Wire Line
	1700 5450 2450 5450
Text Label 2000 3350 2    50   ~ 0
SDO
Text Label 2000 3450 2    50   ~ 0
SDI
Text Label 2000 3550 2    50   ~ 0
SCLK
Text Label 2100 5450 0    50   ~ 0
GPS_TX
Text Label 7300 3250 0    50   ~ 0
SDO
Text Label 7300 3350 0    50   ~ 0
SDI
Text Label 7300 3550 0    50   ~ 0
SCLK
Text Label 9650 2750 2    50   ~ 0
GPS_TX
$Sheet
S 1000 3200 550  700 
U 5BA17F9C
F0 "CC1101" 50
F1 "CC1101.sch" 50
F2 "ANT" B R 1550 3800 50 
F3 "SCLK" I R 1550 3550 50 
F4 "SO" O R 1550 3350 50 
F5 "SI" I R 1550 3450 50 
F6 "CS" O R 1550 3250 50 
$EndSheet
$Comp
L power:GND #PWR012
U 1 1 5BAE96AC
P 8250 5250
F 0 "#PWR012" H 8250 5000 50  0001 C CNN
F 1 "GND" H 8255 5077 50  0000 C CNN
F 2 "" H 8250 5250 50  0001 C CNN
F 3 "" H 8250 5250 50  0001 C CNN
	1    8250 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5BAE9E59
P 9700 5250
F 0 "#PWR015" H 9700 5000 50  0001 C CNN
F 1 "GND" H 9705 5077 50  0000 C CNN
F 2 "" H 9700 5250 50  0001 C CNN
F 3 "" H 9700 5250 50  0001 C CNN
	1    9700 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 5250 8150 5550
Wire Wire Line
	9600 5250 9600 5550
Wire Wire Line
	8050 5250 8050 6000
Wire Wire Line
	9500 5250 9500 6000
Text Label 9500 5900 1    50   ~ 0
S_PARACAIDAS
Text Label 8050 5900 1    50   ~ 0
S_GLOBO
Text Label 9650 2650 2    50   ~ 0
P1
Text Label 7300 3450 0    50   ~ 0
CS
Wire Wire Line
	1550 3250 2000 3250
Text Label 2000 3250 2    50   ~ 0
CS
$Comp
L power:+BATT #PWR07
U 1 1 5BC131BC
P 7150 2450
F 0 "#PWR07" H 7150 2300 50  0001 C CNN
F 1 "+BATT" H 7164 2625 50  0000 C CNN
F 2 "" H 7150 2450 50  0001 C CNN
F 3 "" H 7150 2450 50  0001 C CNN
	1    7150 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2750 7750 2750
Wire Wire Line
	7750 2750 7750 2650
Connection ~ 7750 2650
Wire Wire Line
	7750 2650 7800 2650
Text Label 7300 3050 0    50   ~ 0
S_GLOBO
Text Label 7300 3150 0    50   ~ 0
S_PARACAIDAS
Wire Wire Line
	8600 5550 8600 5350
Wire Wire Line
	8150 5550 8600 5550
Wire Wire Line
	10050 5550 10050 5350
Wire Wire Line
	9600 5550 10050 5550
$Comp
L AXTEC_IC:+3V3_Internal #PWR18
U 1 1 5BCA5312
P 9350 2450
F 0 "#PWR18" H 9350 2850 50  0001 C CNN
F 1 "+3V3_Internal" H 9335 2623 50  0000 C CNN
F 2 "" H 9350 2450 50  0001 C CNN
F 3 "" H 9350 2450 50  0001 C CNN
	1    9350 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5BCC3FF5
P 3250 3950
F 0 "#PWR06" H 3250 3700 50  0001 C CNN
F 1 "GND" H 3255 3777 50  0000 C CNN
F 2 "" H 3250 3950 50  0001 C CNN
F 3 "" H 3250 3950 50  0001 C CNN
	1    3250 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3800 1550 3800
Text Notes 8100 850  0    157  ~ 0
Conector PCB A-B\n
Wire Notes Line
	11100 6350 6950 6350
Wire Notes Line
	6950 6350 6950 3800
Wire Notes Line
	6950 3800 11100 3800
Wire Notes Line
	11100 3800 11100 6350
Wire Notes Line
	11100 3750 6950 3750
Wire Notes Line
	6950 3750 6950 550 
Wire Notes Line
	6950 550  11100 550 
Wire Notes Line
	11100 550  11100 3750
Wire Notes Line
	700  2350 4100 2350
Text Notes 1400 850  0    157  ~ 0
Conector Bateria\n
Wire Notes Line
	700  2400 3600 2400
Wire Notes Line
	3600 2400 3600 4500
Wire Notes Line
	3600 4500 700  4500
Wire Notes Line
	700  4500 700  2400
Text Notes 1600 3050 0    157  ~ 0
CC1101\n\n
Text Notes 7350 4100 0    157  ~ 0
SERVOS GLOBO Y PARACAIDA\n
Wire Notes Line
	700  4600 3600 4600
Wire Notes Line
	3600 4600 3600 6200
Wire Notes Line
	3600 6200 700  6200
Wire Notes Line
	700  6200 700  4600
Text Notes 1850 5350 0    157  ~ 0
GPS\n\n\n
$Comp
L AXTEC_IC:MH_CanSat MH1
U 1 1 5BC7973F
P 1200 7200
F 0 "MH1" H 1478 7315 50  0000 L CNN
F 1 "MH_CanSat" H 1478 7224 50  0000 L CNN
F 2 "AXTEC:MH_CanSat" H 1200 7200 50  0001 C CNN
F 3 "" H 1200 7200 50  0001 C CNN
	1    1200 7200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5BC798E1
P 1200 7400
F 0 "#PWR04" H 1200 7150 50  0001 C CNN
F 1 "GND" H 1205 7227 50  0000 C CNN
F 2 "" H 1200 7400 50  0001 C CNN
F 3 "" H 1200 7400 50  0001 C CNN
	1    1200 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 7300 1050 7350
Wire Wire Line
	1050 7350 1200 7350
Wire Wire Line
	1350 7350 1350 7300
Wire Wire Line
	1200 7400 1200 7350
Connection ~ 1200 7350
Wire Wire Line
	1200 7350 1350 7350
Wire Wire Line
	1200 7300 1200 7350
Wire Notes Line
	700  6250 700  7750
Wire Notes Line
	700  7750 3600 7750
Wire Notes Line
	3600 7750 3600 6250
Wire Notes Line
	3600 6250 700  6250
Text Notes 800  6550 0    157  ~ 0
AGUJEROS DE MONTAJE
$Comp
L SamacSys_Parts:CONUFL001-SMD J2
U 1 1 5BC6411A
P 2750 3800
F 0 "J2" H 2775 4065 50  0000 C CNN
F 1 "CONUFL001-SMD" H 2775 3974 50  0000 C CNN
F 2 "SamacSys_Parts:CONUFL001-SMD" H 3050 3900 50  0001 L CNN
F 3 "https://componentsearchengine.com/Datasheets/1/CONUFL001-SMD.pdf" H 3050 3800 50  0001 L CNN
F 4 "RF Connectors / Coaxial Connectors U.FL Straight Surface Mount Jack" H 3050 3700 50  0001 L CNN "Description"
F 5 "Linx Technologies" H 3050 3500 50  0001 L CNN "Manufacturer_Name"
F 6 "CONUFL001-SMD" H 3050 3400 50  0001 L CNN "Manufacturer Part Number"
F 7 "CONUFL001-SMD" H 3050 3100 50  0001 L CNN "Arrow Part Number"
	1    2750 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 3800 3250 3800
Wire Wire Line
	3250 3800 3250 3900
Wire Wire Line
	3200 3900 3250 3900
Connection ~ 3250 3900
Wire Wire Line
	3250 3900 3250 3950
Wire Wire Line
	7300 3050 7800 3050
Wire Wire Line
	7300 3150 7800 3150
Wire Wire Line
	7300 3250 7800 3250
Wire Wire Line
	7300 3350 7800 3350
Wire Wire Line
	7300 3450 7800 3450
Wire Wire Line
	7800 3550 7300 3550
Wire Wire Line
	9200 2650 9650 2650
Wire Wire Line
	9200 2750 9650 2750
$Comp
L SamacSys_Parts:500SSP1S1M6QEA S1
U 1 1 5BC7E376
P 10500 2900
F 0 "S1" V 10900 2900 50  0000 L CNN
F 1 "500SSP1S1M6QEA" V 10800 2600 50  0000 L CNN
F 2 "SamacSys_Parts:500SSP1S1M6QEA" H 11150 3000 50  0001 L CNN
F 3 "https://www.e-switch.com/system/asset/product_line/data_sheet/115/500.pdf" H 11150 2900 50  0001 L CNN
F 4 "Slide Switches SPDT RA SLIDE PC MNT" H 11150 2800 50  0001 L CNN "Description"
F 5 "6" H 11150 2700 50  0001 L CNN "Height"
F 6 "E-Switch" H 11150 2600 50  0001 L CNN "Manufacturer_Name"
F 7 "500SSP1S1M6QEA" H 11150 2500 50  0001 L CNN "Manufacturer Part Number"
F 8 "500SSP1S1M6QEA" H 11150 2200 50  0001 L CNN "Arrow Part Number"
	1    10500 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5BC844C6
P 7150 2850
F 0 "#PWR011" H 7150 2600 50  0001 C CNN
F 1 "GND" H 7155 2677 50  0000 C CNN
F 2 "" H 7150 2850 50  0001 C CNN
F 3 "" H 7150 2850 50  0001 C CNN
	1    7150 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 2850 7800 2850
Wire Wire Line
	7600 2950 7600 2450
Wire Wire Line
	7600 2950 7800 2950
$Comp
L power:GND #PWR017
U 1 1 5BC7DA4F
P 9750 3300
F 0 "#PWR017" H 9750 3050 50  0001 C CNN
F 1 "GND" H 9755 3127 50  0000 C CNN
F 2 "" H 9750 3300 50  0001 C CNN
F 3 "" H 9750 3300 50  0001 C CNN
	1    9750 3300
	1    0    0    -1  
$EndComp
$Comp
L AXTEC_IC:Conector_PCB_A-B J4
U 1 1 5BC7F6BA
P 8350 2950
F 0 "J4" H 8500 3515 50  0000 C CNN
F 1 "Conector_PCB_A-B" H 8500 3424 50  0000 C CNN
F 2 "AXTEC:Conector_PCB_A-B" H 8250 3400 50  0001 C CNN
F 3 "" H 8250 3400 50  0001 C CNN
F 4 "22-28-5034" H 8350 2950 50  0001 C CNN "Manufacturer Part Number"
F 5 "3" H 8350 2950 50  0001 C CNN "Quantity"
F 6 "22-28-4041" H 8350 2950 50  0001 C CNN "Manufacturer Part Number 2"
F 7 "2" H 8350 2950 50  0001 C CNN "Quantity 2"
	1    8350 2950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 5BC7FC2A
P 7600 2450
F 0 "#PWR0101" H 7600 2300 50  0001 C CNN
F 1 "+5V" H 7615 2623 50  0000 C CNN
F 2 "" H 7600 2450 50  0001 C CNN
F 3 "" H 7600 2450 50  0001 C CNN
	1    7600 2450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0102
U 1 1 5BC7FC87
P 8600 5350
F 0 "#PWR0102" H 8600 5200 50  0001 C CNN
F 1 "+5V" H 8615 5523 50  0000 C CNN
F 2 "" H 8600 5350 50  0001 C CNN
F 3 "" H 8600 5350 50  0001 C CNN
	1    8600 5350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5BC7FCED
P 10050 5350
F 0 "#PWR0103" H 10050 5200 50  0001 C CNN
F 1 "+5V" H 10065 5523 50  0000 C CNN
F 2 "" H 10050 5350 50  0001 C CNN
F 3 "" H 10050 5350 50  0001 C CNN
	1    10050 5350
	1    0    0    -1  
$EndComp
$Comp
L AXTEC_IC:Spacer M1
U 1 1 5C2735B7
P 2100 7200
F 0 "M1" H 2179 7193 50  0000 L CNN
F 1 "Spacer" H 2179 7101 50  0000 L CNN
F 2 "AXTEC:Empty_Footprint" H 2000 7350 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/R30-301-1135805.pdf" H 2100 7450 50  0001 C CNN
F 4 "R30-3011002" H 2179 7055 50  0001 L CNN "Manufacturer Part Number"
	1    2100 7200
	1    0    0    -1  
$EndComp
$Comp
L AXTEC_IC:Spacer M2
U 1 1 5C2736D4
P 2550 7200
F 0 "M2" H 2629 7193 50  0000 L CNN
F 1 "Spacer" H 2629 7101 50  0000 L CNN
F 2 "AXTEC:Empty_Footprint" H 2450 7350 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/R30-301-1135805.pdf" H 2550 7450 50  0001 C CNN
F 4 "R30-3011002" H 2629 7055 50  0001 L CNN "Manufacturer Part Number"
	1    2550 7200
	1    0    0    -1  
$EndComp
$Comp
L AXTEC_IC:Spacer M3
U 1 1 5C273720
P 3000 7200
F 0 "M3" H 3079 7193 50  0000 L CNN
F 1 "Spacer" H 3079 7101 50  0000 L CNN
F 2 "AXTEC:Empty_Footprint" H 2900 7350 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/R30-301-1135805.pdf" H 3000 7450 50  0001 C CNN
F 4 "R30-3011002" H 3079 7055 50  0001 L CNN "Manufacturer Part Number"
	1    3000 7200
	1    0    0    -1  
$EndComp
$Comp
L Graphic:Logo_Open_Hardware_Large #LOGO1
U 1 1 5C33AF12
P 5250 7150
F 0 "#LOGO1" H 5250 7650 50  0001 C CNN
F 1 "Logo_Open_Hardware_Large" H 5250 6750 50  0001 C CNN
F 2 "" H 5250 7150 50  0001 C CNN
F 3 "~" H 5250 7150 50  0001 C CNN
	1    5250 7150
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR014
U 1 1 5C2D3F1A
P 4950 1100
F 0 "#PWR014" H 4950 950 50  0001 C CNN
F 1 "+BATT" H 4964 1275 50  0000 C CNN
F 2 "" H 4950 1100 50  0001 C CNN
F 3 "" H 4950 1100 50  0001 C CNN
	1    4950 1100
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP10
U 1 1 5C2D567F
P 5950 2000
F 0 "TP10" H 6103 2102 50  0000 L CNN
F 1 "TestPoint_GND" H 6103 2011 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 6150 2000 50  0001 C CNN
F 3 "~" H 6150 2000 50  0001 C CNN
	1    5950 2000
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP6
U 1 1 5C2D56BD
P 5950 1200
F 0 "TP6" H 6103 1302 50  0000 L CNN
F 1 "TestPoint_CS" H 6103 1211 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 6150 1200 50  0001 C CNN
F 3 "~" H 6150 1200 50  0001 C CNN
	1    5950 1200
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP7
U 1 1 5C2D56FB
P 5950 1400
F 0 "TP7" H 6103 1502 50  0000 L CNN
F 1 "TestPoint_SDO" H 6103 1411 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 6150 1400 50  0001 C CNN
F 3 "~" H 6150 1400 50  0001 C CNN
	1    5950 1400
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Probe TP8
U 1 1 5C2D575A
P 5950 1600
F 0 "TP8" H 6103 1702 50  0000 L CNN
F 1 "TestPoint_SDI" H 6103 1611 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 6150 1600 50  0001 C CNN
F 3 "~" H 6150 1600 50  0001 C CNN
	1    5950 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 1200 5600 1200
Wire Wire Line
	5950 1400 5600 1400
Wire Wire Line
	5950 1600 5600 1600
$Comp
L power:GND #PWR016
U 1 1 5C2DB21A
P 5950 2050
F 0 "#PWR016" H 5950 1800 50  0001 C CNN
F 1 "GND" H 5955 1877 50  0000 C CNN
F 2 "" H 5950 2050 50  0001 C CNN
F 3 "" H 5950 2050 50  0001 C CNN
	1    5950 2050
	1    0    0    -1  
$EndComp
Text Label 5600 1200 0    50   ~ 0
CS
Text Label 5600 1400 0    50   ~ 0
SDO
Text Label 5600 1600 0    50   ~ 0
SDI
$Comp
L Connector:TestPoint_Probe TP9
U 1 1 5C2DD179
P 5950 1800
F 0 "TP9" H 6103 1902 50  0000 L CNN
F 1 "TestPoint_SCLK" H 6103 1811 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 6150 1800 50  0001 C CNN
F 3 "~" H 6150 1800 50  0001 C CNN
	1    5950 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 1800 5600 1800
Text Label 5600 1800 0    50   ~ 0
SCLK
$Comp
L Connector:TestPoint_Probe TP2
U 1 1 5C2DEC1A
P 4950 1400
F 0 "TP2" H 5700 1550 50  0000 R CNN
F 1 "TestPoint_GPSTX" H 5700 1450 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 5150 1400 50  0001 C CNN
F 3 "~" H 5150 1400 50  0001 C CNN
	1    4950 1400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4950 1400 5350 1400
Text Label 5350 1400 2    50   ~ 0
GPS_TX
$Comp
L Connector:TestPoint_Probe TP4
U 1 1 5C2E7A24
P 4950 1600
F 0 "TP4" H 5700 1750 50  0000 R CNN
F 1 "TestPoint_P1" H 5700 1650 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 5150 1600 50  0001 C CNN
F 3 "~" H 5150 1600 50  0001 C CNN
	1    4950 1600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4950 1600 5350 1600
$Comp
L Connector:TestPoint_Probe TP5
U 1 1 5C2E94B0
P 4950 1800
F 0 "TP5" H 5700 1950 50  0000 R CNN
F 1 "TestPoint_P1" H 5700 1850 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 5150 1800 50  0001 C CNN
F 3 "~" H 5150 1800 50  0001 C CNN
	1    4950 1800
	-1   0    0    -1  
$EndComp
Text Label 5350 1600 2    50   ~ 0
S_GLOBO
Text Label 5500 1800 2    50   ~ 0
S_PARACAIDAS
Wire Wire Line
	4950 1800 5500 1800
Wire Wire Line
	5950 2050 5950 2000
Wire Wire Line
	4950 1100 4950 1200
$Comp
L Connector:TestPoint_Probe TP1
U 1 1 5C2FCED2
P 4950 1200
F 0 "TP1" H 5700 1350 50  0000 R CNN
F 1 "TestPoint_BATT" H 5700 1250 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 5150 1200 50  0001 C CNN
F 3 "~" H 5150 1200 50  0001 C CNN
	1    4950 1200
	-1   0    0    -1  
$EndComp
Wire Notes Line
	4150 2350 6900 2350
Wire Notes Line
	6900 2350 6900 550 
Wire Notes Line
	6900 550  4150 550 
Wire Notes Line
	4150 550  4150 2350
Wire Notes Line
	700  550  4100 550 
Wire Notes Line
	4100 550  4100 2350
Wire Notes Line
	700  550  700  2350
Text Notes 4450 850  0    157  ~ 0
Puntos de prueba\n
$Bitmap
Pos 8050 6750
Scale 1.000000
Data
89 50 4E 47 0D 0A 1A 0A 00 00 00 0D 49 48 44 52 00 00 02 71 00 00 00 AA 08 06 00 00 00 C2 EB 19 
FD 00 00 00 04 73 42 49 54 08 08 08 08 7C 08 64 88 00 00 00 09 70 48 59 73 00 00 0A F0 00 00 0A 
F0 01 42 AC 34 98 00 00 0A 3D 49 44 41 54 78 9C ED DD ED 71 DC D6 15 06 E0 73 33 FE 2F 77 20 75 
20 75 40 76 20 76 E0 ED C0 EC 40 4C 05 8E 2B 30 54 41 E8 0A B2 25 58 1D 28 1D 44 15 20 3F 08 C6 
CC 8A 30 89 BB 17 1F 07 FB 3C 33 9E B1 A5 C5 E2 8E 21 91 2F 2F DE B3 28 7D DF 07 00 00 B9 FC 6D 
ED 05 00 00 30 9D 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 
07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 
90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 
10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 
00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 
90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 
07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 90 10 07 00 90 50 89 88 BB B5 17 C1 59 8E 
7D DF 1F A7 1C 50 4A B9 9B 67 29 2C C4 35 BF 3C AE F9 E5 71 CD 2F CF E4 6B FE 43 44 7C 9A 67 2D 
2C E8 38 F1 F5 AE 79 7E C7 89 AF 77 CD F3 3B 4E 7C BD 6B 9E DF 71 E2 EB 5D F3 FC 8E 53 5E EC 76 
2A 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 
40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 
42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 
00 40 42 42 1C 00 40 42 42 1C 00 40 42 42 1C 00 40 42 3F AC BD 80 86 FE 3E F1 F5 D7 11 71 35 C3 
3A 78 F0 39 22 BE 56 1E FB 21 22 3E B6 5B CA 24 DF 22 E2 1F 67 1C 7F 1B 11 6F 1A AD 25 9B A9 7F 
07 F7 E2 3A 2E F7 6B 89 6B 7E 79 5C F3 0D D9 4B 88 FB D2 F7 FD DD 94 03 4A 29 37 B1 C1 0B B2 23 
D7 11 F1 A1 EF FB FF 4C 3D B0 94 F2 63 44 1C 23 E2 7D E3 35 BD C6 6D DF F7 5D CD 81 A5 94 4B 0E 
70 31 F5 EF E0 5E 94 52 EE E2 42 BF 96 B8 E6 97 C7 35 DF 96 BD DC 4E 9D BC 73 D2 F7 FD 7D 44 FC 
7B 86 B5 F0 E0 6D 44 74 35 07 0E C1 EF 10 0F BB 62 4B FA 7C 46 80 FB 10 11 BF B4 5D 0E 00 8C DB 
43 88 FB 16 11 F7 95 C7 76 0D D7 C1 F7 3E 0E BB 53 93 F5 7D FF 47 3C DC 9A 5C CA 97 DA F3 0D 3B 
87 B5 7F 06 01 A0 CA 1E 42 DC 7D CD 2D BB 41 D7 72 21 3C EB 97 61 97 6A B2 61 57 EC 73 DB E5 3C 
EB 5B 44 1C CE FC 73 F4 B6 DD 72 00 E0 65 7B 08 71 D5 25 F4 BE EF BF 46 C4 EF ED 96 C2 88 FB 61 
B7 AA C6 6D 3C EC 92 CD E9 76 D8 F9 9B 6C D8 69 5C 6B 08 03 80 0B 96 3D C4 7D A9 FD E6 FB 44 D7 
62 21 FC A5 2D F7 E3 F4 E0 00 48 29 7B 88 3B E7 A3 20 22 C2 80 C3 82 B6 D8 8F D3 83 03 20 AD CC 
21 EE 9C 81 86 53 5D A3 F7 E1 AF 6D A9 1F A7 07 07 40 6A 99 43 DC 39 03 0D A7 BA 46 EF C3 CB B6 
D2 8F D3 83 03 20 B5 CC 21 EE EC 5B A9 8F 0C 38 2C 6A 0B FD 38 3D 38 00 D2 CB 1A E2 5A 0C 34 9C 
EA 1A BF 1F E3 D6 EC C7 E9 C1 01 B0 0B 59 43 5C B3 5D B8 47 06 1C 16 B7 46 3F 4E 0F 0E 80 DD C8 
F8 EC D4 96 03 0D A7 BA 88 F8 34 D3 7B F3 BD FB 52 4A D5 F3 55 E3 61 37 ED 43 4C 7B BE AA 1E DC 
8C 4A 29 FD DC E7 E8 FB BE 4C 79 FD F0 BC 43 7F A7 37 C2 F5 B8 3C AE F9 BC 32 EE C4 B5 1C 68 38 
D5 CD F4 BE 3C 6F C9 7E 9C 1E 1C 00 BB 92 31 C4 35 BF 95 FA C8 80 C3 2A 96 E8 C7 E9 C1 01 B0 3B 
D9 42 DC 1C 03 0D A7 BA 99 DF 9F EF CD D9 8F D3 83 03 60 97 B2 85 B8 D9 76 E1 1E 19 70 58 CD 5C 
9F 1F A7 07 07 C0 2E 65 0A 71 73 0E 34 9C EA 16 3A 0F 7F 9A A3 1F A7 07 07 C0 6E 65 0A 71 93 07 
1A 4A 29 3F 56 F6 AD BA 8A 63 38 5F CB 7E 9C 1E 1C 00 BB 96 29 C4 D5 DC 4A BD 89 8A 6F E4 06 1C 
56 D5 A2 1F A7 07 07 C0 EE 65 09 71 B5 03 0D B7 11 F1 B6 94 72 53 71 6C 57 71 0C 6D 9C DB 8F 3B 
E8 C1 01 B0 77 59 42 DC E4 5D B8 61 37 E7 F1 83 60 0F 53 8F 37 E0 B0 AA B3 FA 71 C3 B5 9B 4C 0F 
0E 80 4C 32 84 B8 DA 81 86 C3 93 7F FF 58 B9 B3 D3 55 1C 43 1B D5 FD B8 1A 7A 70 00 64 93 21 C4 
D5 3E A1 E1 F0 C2 7F BF 46 57 71 0C ED 54 F7 E3 2A 74 A1 07 07 40 22 19 9E 9D 5A 73 2B F5 10 11 
6F 4E 7E F9 76 EA 7B F5 7D FF B5 94 F2 7B E8 48 AD E9 9C E7 AB BE 8A 1E 1C 2C E6 B8 F6 02 46 78 
B6 27 29 6D 3D C4 D5 0E 34 1C 9E F9 B5 B7 A5 94 EB BE EF 8F 13 DF AB 0B DF E0 D7 F4 D8 8F AB 19 
4E 79 91 1E 1C 2C 67 F8 FA 7B 5C 79 19 DF 29 A5 08 71 A4 B4 F5 DB A9 35 BB 70 EF 22 E2 6A E4 B7 
0F 53 DF CF 80 C3 26 CC D2 8F D3 83 03 20 B3 2D 87 B8 16 03 0D A7 6E 0C 38 A4 35 47 3F AE 0B 3D 
38 00 92 DA 72 88 6B 35 D0 F0 D4 9B A8 BB 2D D7 55 1C 43 7B E7 7C 7E DC FF D1 83 03 20 BB 2D 87 
B8 9A 5B A9 37 F1 F2 CE 8A 27 38 E4 55 FD F9 71 4F E9 C1 01 B0 07 5B 0D 71 2D 07 1A 4E BD 1F 7A 
73 53 75 15 C7 D0 DE C7 61 FA F8 1C 7A 70 00 A4 B7 D5 10 57 B3 0B F7 63 BC FE F6 58 CD 6E 9C 01 
87 6D F8 16 E7 4F B7 09 71 00 A4 B7 C5 10 37 C7 40 C3 39 AF 7D AA AB 3C 8E 76 0E C3 ED ED 6A 7D 
DF DF 46 C4 97 36 CB 01 80 75 6C 31 C4 D5 0E 34 4C D9 5D 7B 53 79 4B AE AB 38 86 76 7E AD 7D 2E 
EA 33 6E E2 E1 07 06 00 48 69 8B 21 AE E6 56 EA 75 4C FF A8 88 C9 53 AA 06 1C 56 F5 65 D8 41 6B 
62 B8 96 87 56 EF 07 00 4B DB 5A 88 9B 73 A0 E1 D4 47 03 0E 69 7C 8B 19 9E D8 30 EC EA FD DA FA 
7D 01 60 09 5B 0B 71 B5 03 0D B5 DF E0 0F 53 0F 30 E0 B0 8A B3 7B 70 63 F4 E3 00 C8 6A 4B 21 AE 
76 A0 E1 26 BE 7F D8 FD 6B 1D 2A 8F EB 2A 8F 63 BA 96 3D B8 31 FA 71 B0 80 52 CA 5D 29 A5 DF DA 
3F 6B FF 7F 81 5A 5B 0A 71 4B 0C 34 9C 7A 3B F4 E9 A6 EA CE 38 27 AF D7 B4 07 37 46 3F 0E 80 8C 
B6 14 E2 6A 1F 76 FF FE CC F3 1E A6 1E 60 C0 61 11 B3 F4 E0 C6 E8 C7 01 90 CD 56 42 5C ED 40 43 
8B 5D 9A 9F 2A 9F C7 D9 35 38 37 E3 66 EB C1 8D D1 8F 03 20 93 AD 84 B8 C9 BB 70 83 43 A3 F3 4F 
7E 1F 03 0E B3 AA EE C1 95 52 AE 4B 29 C7 33 CE AD 1F 07 40 0A 5B 08 71 55 03 0D C3 87 F5 D6 0E 
34 9C 3A 54 1E D7 35 3A 3F 7F AA EE C1 0D 3B AA F7 11 71 55 4A B9 AB 79 0F FD 38 00 B2 D8 42 88 
AB 1D 68 68 D9 97 7A 5F 4A F9 50 71 5C D7 70 0D 9C DF 83 BB 8F 3F 83 FD A7 CA A1 15 FD 38 00 52 
D8 42 88 AB 1D 68 78 ED C3 EE 5F 6B F2 EE 8F 01 87 E6 AA 7B 70 C3 CE DB D5 C9 2F DF 57 F6 1D F5 
E3 00 D8 BC B5 43 DC 92 4F 68 78 C9 8D 01 87 55 9D D5 83 8B 88 4F CF FC D6 9B A8 FB EC C1 47 FA 
71 00 6C D6 DA 21 6E ED 81 86 A7 DE 44 DD F3 54 0D 38 9C AF 45 0F 6E 8C 7E 1C 00 BB B4 66 88 AB 
1D 68 B8 89 E9 0F BB 7F AD 43 E5 71 5D C3 35 5C 9A 96 3D B8 31 FA 71 00 EC CE 9A 21 6E 0B 03 0D 
A7 AE 86 BE DD 54 5D DB 65 5C 94 D6 3D B8 31 FA 71 00 EC CA 0F 2B 9E BB F6 61 F7 3F 4D 3C EC 73 
44 7C 9D F0 FA 77 13 5F 1F 7D DF 7F 2D A5 FC 1E ED 87 2D F6 6E 8E 1E DC 98 C7 7E DC 75 CD F9 E2 
E1 87 87 3F A2 DD C7 DA C0 25 3A AE BD 80 46 AE E3 F5 3F 40 5E BA E3 DA 0B 68 E4 3A 36 78 CD D7 
0A 71 4B 0E 34 74 7D DF 1F 2B 8E 9B 7C 9E 10 E2 A6 98 B3 07 37 E6 AA 94 72 D7 F7 FD DD D4 03 87 
A0 7E 88 88 7F 56 9C 17 88 88 E1 6B F1 71 E5 65 9C 6D E2 5D 80 8B E6 9A CF 6B AD DB A9 B5 03 0D 
B3 3F 0C BD 96 01 87 49 96 E8 C1 8D D1 8F 03 60 17 D6 08 71 B5 03 0D 1F 62 BE 81 86 56 BA B5 17 
90 C4 52 3D B8 31 FA 71 00 A4 B7 46 88 AB 1D 68 D8 EC 2E DC 13 DD DA 0B 48 60 C9 1E DC 18 9F 1F 
07 40 7A 6B 84 B8 DA 81 86 39 A7 52 9B F0 04 87 17 AD D1 83 1B E3 F3 E3 00 48 6D E9 10 57 3B D0 
70 13 79 A6 02 BB B5 17 B0 51 6B F6 E0 C6 E8 C7 01 90 D6 D2 21 6E 77 03 0D A7 0C 38 8C 5A BB 07 
37 46 3F 0E 80 94 96 0C 71 B5 03 0D EF 22 E2 7D EB C5 CC AC 5B 7B 01 1B B3 85 1E DC 18 FD 38 00 
52 5A 32 C4 ED 79 A0 E1 54 B7 F6 02 36 64 4B 3D B8 31 FA 71 00 A4 B3 64 88 DB D2 C3 EE 67 65 C0 
E1 7F B6 D8 83 1B A3 1F 07 40 2A 4B 85 B8 AA 81 86 E1 61 F7 59 06 1A 4E 75 6B 2F 60 03 B6 DA 83 
1B A3 1F 07 40 1A 4B 85 B8 8B D9 85 7B 64 C0 61 D3 3D B8 31 FA 71 00 A4 B1 D4 B3 53 7F 2B A5 FC 
B6 D0 B9 4E FD AB 94 B2 D2 A9 2F DA CF A5 94 9F D7 5E 44 85 AB 52 4A BF F6 22 78 9E 6B B3 2D AE 
C7 E5 71 CD B7 65 AD 67 A7 02 00 70 06 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 
21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 
0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 
20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 21 21 0E 00 20 A1 
D2 F7 FD DA 6B 00 00 60 22 3B 71 00 00 09 09 71 00 00 09 09 71 00 00 09 09 71 00 00 09 09 71 00 
00 09 09 71 00 00 09 09 71 00 00 09 09 71 00 00 09 09 71 00 00 09 09 71 00 00 09 09 71 00 00 09 
09 71 00 00 09 FD 17 3C 6D 9D 90 5F F2 D3 C3 00 00 00 00 49 45 4E 44 AE 42 60 82 
EndData
$EndBitmap
Wire Wire Line
	7150 2450 7150 2650
Wire Wire Line
	7150 2650 7750 2650
$Comp
L Device:R R?
U 1 1 5C266CED
P 10100 1250
AR Path="/5BA17F9C/5C266CED" Ref="R?"  Part="1" 
AR Path="/5BA1A7F9/5C266CED" Ref="R?"  Part="1" 
AR Path="/5BA1A77A/5C266CED" Ref="R?"  Part="1" 
AR Path="/5C266CED" Ref="R11"  Part="1" 
F 0 "R11" V 10200 1250 50  0000 L CNN
F 1 "1k" V 10000 1250 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 10030 1250 50  0001 C CNN
F 3 "~" H 10100 1250 50  0001 C CNN
F 4 "RC0603JR-071KL" V 10100 1250 50  0001 C CNN "Manufacturer Part Number"
	1    10100 1250
	-1   0    0    1   
$EndComp
$Comp
L AXTEC_IC:+3V3_Internal #PWR8
U 1 1 5C266CBF
P 10100 1100
F 0 "#PWR8" H 10100 1500 50  0001 C CNN
F 1 "+3V3_Internal" H 10085 1273 50  0000 C CNN
F 2 "" H 10100 1100 50  0001 C CNN
F 3 "" H 10100 1100 50  0001 C CNN
	1    10100 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5C265A0A
P 10100 2100
F 0 "#PWR013" H 10100 1850 50  0001 C CNN
F 1 "GND" H 10105 1927 50  0000 C CNN
F 2 "" H 10100 2100 50  0001 C CNN
F 3 "" H 10100 2100 50  0001 C CNN
	1    10100 2100
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC817 Q2
U 1 1 5C258860
P 10200 1900
F 0 "Q2" H 10391 1946 50  0000 L CNN
F 1 "BC817" H 10391 1854 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10400 1825 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC817.pdf" H 10200 1900 50  0001 L CNN
F 4 "BC817-16LT1G" H 10200 1900 50  0001 C CNN "Manufacturer Part Number"
	1    10200 1900
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5C01D74F
P 7350 1700
F 0 "D2" V 7295 1778 50  0000 L CNN
F 1 "Verde" V 7386 1778 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7350 1700 50  0001 C CNN
F 3 "~" H 7350 1700 50  0001 C CNN
F 4 "150060VS75000" V 7350 1700 50  0001 C CNN "Manufacturer Part Number"
	1    7350 1700
	0    -1   -1   0   
$EndComp
Text Label 10950 1900 2    50   ~ 0
P1
Wire Wire Line
	10700 1900 10950 1900
$Comp
L Device:R R?
U 1 1 5BAFD8C8
P 10550 1900
AR Path="/5BA17F9C/5BAFD8C8" Ref="R?"  Part="1" 
AR Path="/5BA1A7F9/5BAFD8C8" Ref="R?"  Part="1" 
AR Path="/5BA1A77A/5BAFD8C8" Ref="R?"  Part="1" 
AR Path="/5BAFD8C8" Ref="R1"  Part="1" 
F 0 "R1" V 10650 1900 50  0000 L CNN
F 1 "1k" V 10450 1900 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 10480 1900 50  0001 C CNN
F 3 "~" H 10550 1900 50  0001 C CNN
F 4 "RC0603JR-071KL" V 10550 1900 50  0001 C CNN "Manufacturer Part Number"
	1    10550 1900
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5BAFD8C1
P 10100 1550
F 0 "D1" V 10045 1628 50  0000 L CNN
F 1 "Rojo" V 10136 1628 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 10100 1550 50  0001 C CNN
F 3 "~" H 10100 1550 50  0001 C CNN
F 4 "150060RS75000" V 10100 1550 50  0001 C CNN "Manufacturer Part Number"
	1    10100 1550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5BAF7898
P 7350 1850
F 0 "#PWR010" H 7350 1600 50  0001 C CNN
F 1 "GND" H 7355 1677 50  0000 C CNN
F 2 "" H 7350 1850 50  0001 C CNN
F 3 "" H 7350 1850 50  0001 C CNN
	1    7350 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 2850 10200 2850
Wire Wire Line
	10200 2850 10200 2700
Wire Wire Line
	10200 2700 10350 2700
Wire Wire Line
	9200 2950 10200 2950
Wire Wire Line
	10200 2950 10200 3100
Wire Wire Line
	10200 3100 10450 3100
$Comp
L power:GND #PWR046
U 1 1 5CCAD052
P 10850 3250
F 0 "#PWR046" H 10850 3000 50  0001 C CNN
F 1 "GND" H 10855 3077 50  0000 C CNN
F 2 "" H 10850 3250 50  0001 C CNN
F 3 "" H 10850 3250 50  0001 C CNN
	1    10850 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 2850 10850 2850
Wire Wire Line
	10850 2850 10850 2950
Wire Wire Line
	10750 2950 10850 2950
Connection ~ 10850 2950
Wire Wire Line
	10850 2950 10850 3250
Wire Wire Line
	9350 2450 9350 3150
Wire Wire Line
	9350 3150 9200 3150
Wire Wire Line
	9750 3050 9750 3300
Wire Wire Line
	9200 3050 9750 3050
Wire Wire Line
	9650 3250 9200 3250
Text Label 9650 3250 2    50   ~ 0
nCHG
Wire Wire Line
	7350 1550 7350 1450
$Comp
L Device:R R?
U 1 1 5C01D115
P 7350 1300
AR Path="/5BA17F9C/5C01D115" Ref="R?"  Part="1" 
AR Path="/5BA1A7F9/5C01D115" Ref="R?"  Part="1" 
AR Path="/5BA1A77A/5C01D115" Ref="R?"  Part="1" 
AR Path="/5C01D115" Ref="R2"  Part="1" 
F 0 "R2" V 7450 1300 50  0000 L CNN
F 1 "1k" V 7250 1300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7280 1300 50  0001 C CNN
F 3 "~" H 7350 1300 50  0001 C CNN
F 4 "RC0603JR-071KL" V 7350 1300 50  0001 C CNN "Manufacturer Part Number"
	1    7350 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 1150 7350 1100
$Comp
L AXTEC_IC:+3V3_Internal #PWR9
U 1 1 5CC81380
P 7350 1100
F 0 "#PWR9" H 7350 1500 50  0001 C CNN
F 1 "+3V3_Internal" H 7335 1273 50  0000 C CNN
F 2 "" H 7350 1100 50  0001 C CNN
F 3 "" H 7350 1100 50  0001 C CNN
	1    7350 1100
	1    0    0    -1  
$EndComp
Text Label 8050 1950 3    50   ~ 0
nCHG
$Comp
L Device:LED D3
U 1 1 5CCDBC03
P 8050 1700
F 0 "D3" V 7995 1778 50  0000 L CNN
F 1 "Verde" V 8086 1778 50  0000 L CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8050 1700 50  0001 C CNN
F 3 "~" H 8050 1700 50  0001 C CNN
F 4 "150060VS75000" V 8050 1700 50  0001 C CNN "Manufacturer Part Number"
	1    8050 1700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8050 1550 8050 1450
$Comp
L Device:R R?
U 1 1 5CCDBC11
P 8050 1300
AR Path="/5BA17F9C/5CCDBC11" Ref="R?"  Part="1" 
AR Path="/5BA1A7F9/5CCDBC11" Ref="R?"  Part="1" 
AR Path="/5BA1A77A/5CCDBC11" Ref="R?"  Part="1" 
AR Path="/5CCDBC11" Ref="R12"  Part="1" 
F 0 "R12" V 8150 1300 50  0000 L CNN
F 1 "1k" V 7950 1300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7980 1300 50  0001 C CNN
F 3 "~" H 8050 1300 50  0001 C CNN
F 4 "RC0603JR-071KL" V 8050 1300 50  0001 C CNN "Manufacturer Part Number"
	1    8050 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 1150 8050 1100
Wire Wire Line
	8050 1850 8050 1950
$Comp
L power:+BATT #PWR047
U 1 1 5CCE0678
P 8050 1100
F 0 "#PWR047" H 8050 950 50  0001 C CNN
F 1 "+BATT" H 8064 1275 50  0000 C CNN
F 2 "" H 8050 1100 50  0001 C CNN
F 3 "" H 8050 1100 50  0001 C CNN
	1    8050 1100
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:AP9211SA-AC-HAC-7 U4
U 1 1 5D050D68
P 2450 1650
F 0 "U4" H 3050 1915 50  0000 C CNN
F 1 "AP9211SA-AF-HAC-7" H 3050 1824 50  0000 C CNN
F 2 "SamacSys_Parts:AP9211SAACHAC7" H 3500 1750 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/AP9211.pdf" H 3500 1650 50  0001 L CNN
F 4 "Battery Management Multicell Batt MGR PMIC Protection Int" H 3500 1550 50  0001 L CNN "Description"
F 5 "0" H 3500 1450 50  0001 L CNN "Height"
F 6 "AP9211SA-AF-HAC-7" H 3500 1250 50  0001 L CNN "Manufacturer Part Number"
	1    2450 1650
	-1   0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR01
U 1 1 5BC12477
P 950 1150
F 0 "#PWR01" H 950 1000 50  0001 C CNN
F 1 "+BATT" H 964 1325 50  0000 C CNN
F 2 "" H 950 1150 50  0001 C CNN
F 3 "" H 950 1150 50  0001 C CNN
	1    950  1150
	1    0    0    -1  
$EndComp
$Comp
L SamacSys_Parts:SSM3J338R,LF Q1
U 1 1 5BAE3EF9
P 2850 1700
F 0 "Q1" V 3417 1800 50  0000 C CNN
F 1 "SSM3J338R,LF" V 3326 1800 50  0000 C CNN
F 2 "SamacSys_Parts:SSM3J332RLFT" H 3300 1650 50  0001 L CNN
F 3 "http://toshiba.semicon-storage.com/info/docget.jsp?did=30384&prodName=SSM3J338R" H 3300 1550 50  0001 L CNN
F 4 "MOSFET Small-signal MOSFET Vdss= -12V, ID= -6A" H 3300 1450 50  0001 L CNN "Description"
F 5 "Toshiba" H 3300 1250 50  0001 L CNN "Manufacturer_Name"
F 6 "SSM3J338R,LF" H 3300 1150 50  0001 L CNN "Manufacturer Part Number"
F 7 "SSM3J338R,LF" H 3300 850 50  0001 L CNN "Arrow Part Number"
F 8 "https://www.arrow.com/en/products/ssm3j338rlf/toshiba" H 3300 750 50  0001 L CNN "Arrow Price/Stock"
	1    2850 1700
	0    1    -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5BAE15D7
P 3850 1400
F 0 "J1" H 3800 1500 50  0000 L CNN
F 1 "Batería LiPo (Soldada)" V 4000 950 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3850 1400 50  0001 C CNN
F 3 "~" H 3850 1400 50  0001 C CNN
	1    3850 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1500 3350 1700
Wire Wire Line
	3350 1500 3650 1500
Wire Wire Line
	3350 1700 2850 1700
Connection ~ 2850 1700
Wire Wire Line
	2450 1650 2450 1700
Wire Wire Line
	2850 1700 2450 1700
Connection ~ 2450 1700
Wire Wire Line
	2450 1700 2450 1750
$Comp
L Device:C_Small C?
U 1 1 5D09E78F
P 2600 2000
AR Path="/5BA17F9C/5D09E78F" Ref="C?"  Part="1" 
AR Path="/5BA1A7F9/5D09E78F" Ref="C?"  Part="1" 
AR Path="/5D09E78F" Ref="C1"  Part="1" 
F 0 "C1" H 2400 1950 50  0000 L CNN
F 1 "100nF" H 2350 1850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2600 2000 50  0001 C CNN
F 3 "~" H 2600 2000 50  0001 C CNN
F 4 "GCM155R71C104KA55D" H 2600 2000 50  0001 C CNN "Manufacturer Part Number"
	1    2600 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 1850 2600 1850
$Comp
L Device:R_Small R14
U 1 1 5D0AEF9A
P 2600 1550
F 0 "R14" H 2541 1504 50  0000 R CNN
F 1 "330" H 2541 1595 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2600 1550 50  0001 C CNN
F 3 "~" H 2600 1550 50  0001 C CNN
F 4 "RC0603FR-07330RL" H 2600 1550 50  0001 C CNN "Manufacturer Part Number"
	1    2600 1550
	-1   0    0    1   
$EndComp
Wire Wire Line
	2600 1650 2600 1850
Wire Wire Line
	2600 1450 2600 1400
Wire Wire Line
	2600 1400 2650 1400
Wire Wire Line
	2600 1900 2600 1850
Connection ~ 2600 1850
Wire Wire Line
	2600 2100 2600 2200
Wire Wire Line
	2600 2200 2850 2200
Wire Wire Line
	2850 2200 2850 1700
Wire Wire Line
	2600 1400 2600 1200
Connection ~ 2600 1400
Wire Wire Line
	950  1200 950  1150
Wire Wire Line
	950  1200 2600 1200
Wire Wire Line
	1250 1750 950  1750
Wire Wire Line
	950  1750 950  1800
$Comp
L power:GND #PWR02
U 1 1 5D0C4FD0
P 950 1800
F 0 "#PWR02" H 950 1550 50  0001 C CNN
F 1 "GND" H 955 1627 50  0000 C CNN
F 2 "" H 950 1800 50  0001 C CNN
F 3 "" H 950 1800 50  0001 C CNN
	1    950  1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R13
U 1 1 5D0C92B7
P 1150 1650
F 0 "R13" V 1346 1650 50  0000 C CNN
F 1 "2.7k" V 1255 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1150 1650 50  0001 C CNN
F 3 "~" H 1150 1650 50  0001 C CNN
F 4 "RC0603FR-072K7L" V 1150 1650 50  0001 C CNN "Manufacturer Part Number"
	1    1150 1650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1050 1650 950  1650
Wire Wire Line
	950  1650 950  1750
Connection ~ 950  1750
Wire Wire Line
	3250 1400 3650 1400
$EndSCHEMATC
