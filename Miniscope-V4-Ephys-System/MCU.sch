EESchema Schematic File Version 4
EELAYER 30 0
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
Text Label 1825 1650 0    50   ~ 0
SWDIO
Text Label 1825 1800 0    50   ~ 0
SWDCLK
$Comp
L power:GND #PWR0115
U 1 1 5EEBC0C6
P 1150 2950
F 0 "#PWR0115" H 1150 2700 50  0001 C CNN
F 1 "GND" H 1155 2777 50  0000 C CNN
F 2 "" H 1150 2950 50  0001 C CNN
F 3 "" H 1150 2950 50  0001 C CNN
	1    1150 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5EEBC0CC
P 1950 5375
F 0 "#PWR0116" H 1950 5125 50  0001 C CNN
F 1 "GND" H 1955 5202 50  0000 C CNN
F 2 "" H 1950 5375 50  0001 C CNN
F 3 "" H 1950 5375 50  0001 C CNN
	1    1950 5375
	1    0    0    -1  
$EndComp
Text Label 900  5000 0    50   ~ 0
RED_LED
$Comp
L power:GND #PWR0117
U 1 1 5EEBC0D4
P 9575 4725
F 0 "#PWR0117" H 9575 4475 50  0001 C CNN
F 1 "GND" H 9580 4552 50  0000 C CNN
F 2 "" H 9575 4725 50  0001 C CNN
F 3 "" H 9575 4725 50  0001 C CNN
	1    9575 4725
	1    0    0    -1  
$EndComp
Text Label 9575 4625 0    50   ~ 0
TK
Text Label 9575 4525 0    50   ~ 0
TD
Text Label 9575 4425 0    50   ~ 0
TF
Wire Wire Line
	10125 4725 9575 4725
Wire Wire Line
	10125 4625 9575 4625
Wire Wire Line
	10125 4525 9575 4525
Wire Wire Line
	10125 4425 9575 4425
$Comp
L .Connector:Conn_01x01 J16
U 1 1 5EEBC0ED
P 10325 5150
F 0 "J16" H 10405 5146 50  0000 L CNN
F 1 "Conn_01x01" H 10405 5146 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 10405 5101 50  0001 L CNN
F 3 "~" H 10325 5150 50  0001 C CNN
	1    10325 5150
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J15
U 1 1 5EEBC0F3
P 10325 5050
F 0 "J15" H 10405 5046 50  0000 L CNN
F 1 "Conn_01x01" H 10405 5046 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 10325 5050 50  0001 C CNN
F 3 "~" H 10325 5050 50  0001 C CNN
	1    10325 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10125 5050 9675 5050
Wire Wire Line
	10125 5150 9675 5150
Text Label 9675 5050 0    50   ~ 0
SWDIO
Text Label 9675 5150 0    50   ~ 0
SWDCLK
Wire Wire Line
	10275 3350 10275 3450
Connection ~ 10275 3350
$Comp
L power:GND #PWR0118
U 1 1 5EEBC0FF
P 10275 3750
F 0 "#PWR0118" H 10275 3500 50  0001 C CNN
F 1 "GND" H 10280 3577 50  0000 C CNN
F 2 "" H 10275 3750 50  0001 C CNN
F 3 "" H 10275 3750 50  0001 C CNN
	1    10275 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10275 2700 10275 2800
Connection ~ 10275 2700
Wire Wire Line
	9775 3350 10275 3350
$Comp
L .Inductor:BLM18PG471SN1D L2
U 1 1 5EEBC110
P 9675 3350
F 0 "L2" V 9860 3350 50  0000 C CNN
F 1 "BLM18PG471SN1D" H 9725 3200 50  0001 L CNN
F 2 ".Inductor:L_0603_1608Metric_L" H 9725 3650 50  0001 C CNN
F 3 "https://www.murata.com/en-us/products/productdata/8796738650142/ENFA0003.pdf" H 9675 3350 50  0001 C CNN
F 4 "470 Ohms @ 100MHz" V 9769 3350 50  0000 C CNN "Note"
F 5 "0603" H 9825 3100 50  0001 C CNN "Size"
	1    9675 3350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9775 2700 10275 2700
$Comp
L .Inductor:BLM18PG471SN1D L1
U 1 1 5EEBC119
P 9675 2700
F 0 "L1" V 9860 2700 50  0000 C CNN
F 1 "BLM18PG471SN1D" H 9725 2550 50  0001 L CNN
F 2 ".Inductor:L_0603_1608Metric_L" H 9725 3000 50  0001 C CNN
F 3 "https://www.murata.com/en-us/products/productdata/8796738650142/ENFA0003.pdf" H 9675 2700 50  0001 C CNN
F 4 "470 Ohms @ 100MHz" V 9769 2700 50  0000 C CNN "Note"
F 5 "0603" H 9825 2450 50  0001 C CNN "Size"
	1    9675 2700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8775 2700 9575 2700
Wire Wire Line
	8775 3350 9575 3350
Text Label 8775 2700 0    50   ~ 0
VDDOUT
Text Label 8775 3350 0    50   ~ 0
VDDOUT
Wire Wire Line
	10275 2700 10775 2700
Wire Wire Line
	10275 3350 10775 3350
Text Label 10475 2700 0    50   ~ 0
VDDPLL
Text Label 10475 3350 0    50   ~ 0
VDDUTMIC
Text Label 9375 1800 0    50   ~ 0
VDDOUT
Text Label 10300 1800 0    50   ~ 0
VDDCORE
$Comp
L power:GND #PWR0119
U 1 1 5EEBC18B
P 6875 1350
F 0 "#PWR0119" H 6875 1100 50  0001 C CNN
F 1 "GND" H 6880 1177 50  0000 C CNN
F 2 "" H 6875 1350 50  0001 C CNN
F 3 "" H 6875 1350 50  0001 C CNN
	1    6875 1350
	1    0    0    -1  
$EndComp
Text Label 8325 1775 0    50   ~ 0
VDDIO
Wire Wire Line
	9350 950  9350 1050
Text Label 9475 950  0    50   ~ 0
VDDIN
Text Label 8350 1000 0    50   ~ 0
VDDPLLUSB
$Comp
L power:GND #PWR0120
U 1 1 5EEBC1B3
P 7425 1200
F 0 "#PWR0120" H 7425 950 50  0001 C CNN
F 1 "GND" H 7430 1027 50  0000 C CNN
F 2 "" H 7425 1200 50  0001 C CNN
F 3 "" H 7425 1200 50  0001 C CNN
	1    7425 1200
	1    0    0    -1  
$EndComp
Text Label 7575 1000 0    50   ~ 0
VDDUTMII
Wire Wire Line
	10125 4325 9575 4325
Text Label 9950 950  0    50   ~ 0
VDDOUT
$Comp
L power:+1V8 #PWR0121
U 1 1 5EEBC1DB
P 1400 1050
F 0 "#PWR0121" H 1400 900 50  0001 C CNN
F 1 "+1V8" H 1415 1223 50  0000 C CNN
F 2 "" H 1400 1050 50  0001 C CNN
F 3 "" H 1400 1050 50  0001 C CNN
	1    1400 1050
	1    0    0    -1  
$EndComp
$Comp
L .Device:R_Small_US R5
U 1 1 5EEBC1E1
P 1400 1400
F 0 "R5" H 1468 1446 50  0000 L CNN
F 1 "100K" H 1468 1355 50  0000 L CNN
F 2 ".Resistor:R_0201_0603Metric_ERJ_L" H 1400 1400 50  0001 C CNN
F 3 "~" H 1400 1400 50  0001 C CNN
	1    1400 1400
	1    0    0    -1  
$EndComp
$Comp
L .Device:R_Small_US R6
U 1 1 5EEBC1E7
P 1725 1400
F 0 "R6" H 1793 1446 50  0000 L CNN
F 1 "100K" H 1793 1355 50  0000 L CNN
F 2 ".Resistor:R_0201_0603Metric_ERJ_L" H 1725 1400 50  0001 C CNN
F 3 "~" H 1725 1400 50  0001 C CNN
	1    1725 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 1050 1400 1200
Wire Wire Line
	1400 1200 1725 1200
Wire Wire Line
	1725 1200 1725 1300
Connection ~ 1400 1200
Wire Wire Line
	1400 1200 1400 1300
Wire Wire Line
	1725 1650 1725 1500
Wire Wire Line
	1400 1500 1400 1800
Wire Wire Line
	1400 1800 2100 1800
Wire Wire Line
	1725 1650 2100 1650
$Comp
L .Device:R_Small_US R7
U 1 1 5EEBC1F6
P 1950 4225
F 0 "R7" H 2018 4271 50  0000 L CNN
F 1 "1.5K" H 2018 4180 50  0000 L CNN
F 2 ".Resistor:R_0201_0603Metric_ERJ_L" H 1950 4225 50  0001 C CNN
F 3 "~" H 1950 4225 50  0001 C CNN
	1    1950 4225
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0122
U 1 1 5EEBC1FE
P 9575 4325
F 0 "#PWR0122" H 9575 4175 50  0001 C CNN
F 1 "+5V" H 9590 4498 50  0000 C CNN
F 2 "" H 9575 4325 50  0001 C CNN
F 3 "" H 9575 4325 50  0001 C CNN
	1    9575 4325
	1    0    0    -1  
$EndComp
$Comp
L power:+1V8 #PWR0123
U 1 1 5EEBC204
P 7250 1000
F 0 "#PWR0123" H 7250 850 50  0001 C CNN
F 1 "+1V8" H 7265 1173 50  0000 C CNN
F 2 "" H 7250 1000 50  0001 C CNN
F 3 "" H 7250 1000 50  0001 C CNN
	1    7250 1000
	1    0    0    -1  
$EndComp
$Comp
L power:+1V8 #PWR0124
U 1 1 5EEBC20A
P 8125 1000
F 0 "#PWR0124" H 8125 850 50  0001 C CNN
F 1 "+1V8" H 8140 1173 50  0000 C CNN
F 2 "" H 8125 1000 50  0001 C CNN
F 3 "" H 8125 1000 50  0001 C CNN
	1    8125 1000
	1    0    0    -1  
$EndComp
$Comp
L power:+1V8 #PWR0125
U 1 1 5EEBC210
P 7250 1775
F 0 "#PWR0125" H 7250 1625 50  0001 C CNN
F 1 "+1V8" H 7265 1948 50  0000 C CNN
F 2 "" H 7250 1775 50  0001 C CNN
F 3 "" H 7250 1775 50  0001 C CNN
	1    7250 1775
	1    0    0    -1  
$EndComp
$Comp
L power:+1V8 #PWR0126
U 1 1 5EEBC216
P 6875 1000
F 0 "#PWR0126" H 6875 850 50  0001 C CNN
F 1 "+1V8" H 6890 1173 50  0000 C CNN
F 2 "" H 6875 1000 50  0001 C CNN
F 3 "" H 6875 1000 50  0001 C CNN
	1    6875 1000
	1    0    0    -1  
$EndComp
$Comp
L power:+1V8 #PWR0127
U 1 1 5EEBC21C
P 9175 950
F 0 "#PWR0127" H 9175 800 50  0001 C CNN
F 1 "+1V8" H 9190 1123 50  0000 C CNN
F 2 "" H 9175 950 50  0001 C CNN
F 3 "" H 9175 950 50  0001 C CNN
	1    9175 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 950  9175 950 
Wire Wire Line
	6875 1000 6875 1100
Wire Wire Line
	6875 1300 6875 1350
Connection ~ 9350 950 
Wire Wire Line
	9350 950  9700 950 
Wire Notes Line
	8950 4075 10800 4075
Wire Notes Line
	10800 4075 10800 5875
Wire Notes Line
	10825 6400 8975 6400
Wire Notes Line
	8950 5875 8950 4075
Text Notes 8975 4175 0    50   ~ 0
Connectors
$Comp
L .MCU:ATSAME70N21A-CN U4
U 1 1 5EEBC239
P 5375 4475
F 0 "U4" H 6250 1700 50  0000 C CNN
F 1 "ATSAME70N21A-CN" H 6300 1575 50  0000 C CNN
F 2 ".Package_BGA:BGA_100_CP80_10X10_900X900X110B40L" H 5275 4375 50  0001 C CNN
F 3 "http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en589960" H 5375 4475 50  0001 C CNN
	1    5375 4475
	1    0    0    -1  
$EndComp
Wire Wire Line
	1475 2675 1150 2675
Wire Wire Line
	1575 2800 1575 2950
Wire Wire Line
	1575 2950 1150 2950
Wire Wire Line
	1150 2950 1150 2875
Wire Wire Line
	2000 2675 1675 2675
Wire Wire Line
	2000 2875 2000 2950
Wire Wire Line
	2000 2950 1575 2950
Connection ~ 1575 2950
Wire Wire Line
	1575 2550 2325 2550
Wire Wire Line
	2325 2550 2325 2950
Wire Wire Line
	2325 2950 2000 2950
Connection ~ 2000 2950
Connection ~ 1150 2950
Wire Wire Line
	4925 1175 4825 1175
Connection ~ 4825 1175
Wire Wire Line
	4825 1175 4725 1175
Wire Wire Line
	4725 1175 4725 650 
Connection ~ 4725 1175
Wire Wire Line
	5325 1175 5225 1175
Connection ~ 5225 1175
Wire Wire Line
	5225 1175 5125 1175
Wire Wire Line
	5125 1175 5125 650 
Connection ~ 5125 1175
Wire Wire Line
	5575 1175 5575 650 
Wire Wire Line
	5675 1175 5675 650 
Wire Wire Line
	5775 1175 5775 650 
Wire Wire Line
	5875 1175 5875 650 
Wire Wire Line
	5975 1175 5975 650 
Wire Wire Line
	6075 1175 6075 650 
Text Label 4725 1025 1    50   ~ 0
VDDCORE
Text Label 5125 975  1    50   ~ 0
VDDIO
Text Label 5575 975  1    50   ~ 0
VDDOUT
Text Label 5875 975  1    50   ~ 0
VDDUTMIC
Wire Wire Line
	4125 5575 3650 5575
Wire Wire Line
	4125 5475 3650 5475
Text Label 3650 5475 0    50   ~ 0
XOUT
Text Label 3650 5575 0    50   ~ 0
XIN
Text Label 6075 975  1    50   ~ 0
VDDPLLUSB
Text Label 5975 975  1    50   ~ 0
VDDUTMII
Text Label 5675 975  1    50   ~ 0
VDDIN
Text Label 5775 975  1    50   ~ 0
VDDPLL
Wire Wire Line
	5875 7375 5775 7375
Connection ~ 5125 7375
Wire Wire Line
	5125 7375 5025 7375
Connection ~ 5225 7375
Wire Wire Line
	5225 7375 5125 7375
Connection ~ 5325 7375
Wire Wire Line
	5325 7375 5225 7375
Connection ~ 5575 7375
Wire Wire Line
	5575 7375 5450 7375
Connection ~ 5675 7375
Wire Wire Line
	5675 7375 5575 7375
Connection ~ 5775 7375
Wire Wire Line
	5775 7375 5675 7375
$Comp
L power:GND #PWR0128
U 1 1 5EEBC275
P 5450 7375
F 0 "#PWR0128" H 5450 7125 50  0001 C CNN
F 1 "GND" H 5455 7202 50  0000 C CNN
F 2 "" H 5450 7375 50  0001 C CNN
F 3 "" H 5450 7375 50  0001 C CNN
	1    5450 7375
	1    0    0    -1  
$EndComp
Connection ~ 5450 7375
Wire Wire Line
	5450 7375 5325 7375
Wire Wire Line
	4125 4475 3650 4475
Wire Wire Line
	4125 4275 3650 4275
Text Label 3650 4275 0    50   ~ 0
RED_LED
Text Label 3175 4500 0    50   ~ 0
SPI0_NPCS1
Wire Wire Line
	4125 4675 3650 4675
Wire Wire Line
	4125 4775 3650 4775
Wire Wire Line
	4125 4875 3650 4875
Wire Wire Line
	4125 5075 3650 5075
Wire Wire Line
	4125 5175 3650 5175
Wire Wire Line
	4125 5275 3650 5275
Wire Wire Line
	4125 5375 3650 5375
Text Label 3650 4675 0    50   ~ 0
TF
Text Label 3650 4775 0    50   ~ 0
TK
Text Label 3650 4875 0    50   ~ 0
SPI0_NPCS0
Text Label 3650 5075 0    50   ~ 0
SDA_I2C
Text Label 3650 5175 0    50   ~ 0
SCL_I2C
Text Label 3650 5275 0    50   ~ 0
SWDIO
Text Label 3650 5375 0    50   ~ 0
SWDCLK
Wire Wire Line
	4125 6975 4125 6875
Wire Wire Line
	4125 6975 4125 7375
Wire Wire Line
	4125 7375 5025 7375
Connection ~ 4125 6975
Connection ~ 5025 7375
Wire Wire Line
	4125 6375 3650 6375
Text Label 3650 6375 0    50   ~ 0
NRESET
Wire Wire Line
	7000 3775 6525 3775
Wire Wire Line
	7000 3975 6525 3975
Wire Wire Line
	7000 4775 6525 4775
Wire Wire Line
	7000 4875 6525 4875
Wire Wire Line
	7000 4975 6525 4975
Text Label 7000 3775 2    50   ~ 0
TD
Text Label 7475 4000 2    50   ~ 0
SPI0_NPCS2
Text Label 7000 4775 2    50   ~ 0
SPI0_MISO
Text Label 7000 4875 2    50   ~ 0
SPI0_MOSI
Text Label 7000 4975 2    50   ~ 0
SPI0_SPCK
Wire Wire Line
	7000 5175 6525 5175
Wire Wire Line
	7000 5375 6525 5375
Text Label 7500 5200 2    50   ~ 0
SPI0_NPCS1
Text Label 7500 5400 2    50   ~ 0
SPI0_NPCS3
$Comp
L .Device:C_Small C9
U 1 1 5EEBC2A4
P 6875 1200
F 0 "C9" H 6967 1246 50  0000 L CNN
F 1 "4.7uF" H 6967 1155 50  0000 L CNN
F 2 ".Capacitor:C_0402_1005Metric_L" H 6875 1200 50  0001 C CNN
F 3 "~" H 6875 1200 50  0001 C CNN
	1    6875 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 1000 7425 1000
$Comp
L .Device:C_Small C5
U 1 1 5EEBC2AB
P 7425 1100
F 0 "C5" H 7517 1146 50  0000 L CNN
F 1 "4.7uF" H 7517 1055 50  0000 L CNN
F 2 ".Capacitor:C_0402_1005Metric_L" H 7425 1100 50  0001 C CNN
F 3 "~" H 7425 1100 50  0001 C CNN
	1    7425 1100
	1    0    0    -1  
$EndComp
Connection ~ 7425 1000
Wire Wire Line
	7425 1000 7925 1000
$Comp
L .Device:LED_Small_ALT D1
U 1 1 5EEBC2B3
P 1950 4600
F 0 "D1" V 1996 4532 50  0000 R CNN
F 1 "LED_Small_ALT" V 1905 4532 50  0000 R CNN
F 2 "LED_SMD:LED_0402_1005Metric" V 1950 4600 50  0001 C CNN
F 3 "~" V 1950 4600 50  0001 C CNN
	1    1950 4600
	0    -1   -1   0   
$EndComp
$Comp
L .Regulator_Linear:MCP1711 U?
U 1 1 5EE8F7BA
P 1575 6975
AR Path="/5EE8F7BA" Ref="U?"  Part="1" 
AR Path="/5EEA60FC/5EE8F7BA" Ref="U5"  Part="1" 
F 0 "U5" H 1750 7200 50  0000 C CNN
F 1 "MCP1711T-18I/5X" H 1750 7109 50  0000 C CNN
F 2 ".Package_QFN:UQFN_MCP1711" H 1575 6975 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20005415D.pdf" H 1575 6975 50  0001 C CNN
	1    1575 6975
	1    0    0    -1  
$EndComp
Wire Wire Line
	1875 7375 1875 7425
Wire Wire Line
	1875 7425 1975 7425
Wire Wire Line
	1975 7425 1975 7375
Wire Wire Line
	2125 6975 2200 6975
$Comp
L power:+5V #PWR?
U 1 1 5EE8F7C4
P 925 6975
AR Path="/5EE8F7C4" Ref="#PWR?"  Part="1" 
AR Path="/5EEA60FC/5EE8F7C4" Ref="#PWR0129"  Part="1" 
F 0 "#PWR0129" H 925 6825 50  0001 C CNN
F 1 "+5V" H 940 7148 50  0000 C CNN
F 2 "" H 925 6975 50  0001 C CNN
F 3 "" H 925 6975 50  0001 C CNN
	1    925  6975
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EE8F7CA
P 1875 7425
AR Path="/5EE8F7CA" Ref="#PWR?"  Part="1" 
AR Path="/5EEA60FC/5EE8F7CA" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0130" H 1875 7175 50  0001 C CNN
F 1 "GND" H 1880 7252 50  0000 C CNN
F 2 "" H 1875 7425 50  0001 C CNN
F 3 "" H 1875 7425 50  0001 C CNN
	1    1875 7425
	1    0    0    -1  
$EndComp
Connection ~ 1875 7425
$Comp
L power:+1V8 #PWR?
U 1 1 5EE8F7D1
P 2375 6975
AR Path="/5EE8F7D1" Ref="#PWR?"  Part="1" 
AR Path="/5EEA60FC/5EE8F7D1" Ref="#PWR0131"  Part="1" 
F 0 "#PWR0131" H 2375 6825 50  0001 C CNN
F 1 "+1V8" H 2390 7148 50  0000 C CNN
F 2 "" H 2375 6975 50  0001 C CNN
F 3 "" H 2375 6975 50  0001 C CNN
	1    2375 6975
	1    0    0    -1  
$EndComp
$Comp
L .Device:C C?
U 1 1 5EE8F7D7
P 1000 7125
AR Path="/5EE8F7D7" Ref="C?"  Part="1" 
AR Path="/5EEA60FC/5EE8F7D7" Ref="C20"  Part="1" 
F 0 "C20" H 1115 7171 50  0000 L CNN
F 1 "0.1uF" H 1115 7080 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 1038 6975 50  0001 C CNN
F 3 "~" H 1000 7125 50  0001 C CNN
	1    1000 7125
	1    0    0    -1  
$EndComp
$Comp
L .Device:C C?
U 1 1 5EE8F7DD
P 2200 7125
AR Path="/5EE8F7DD" Ref="C?"  Part="1" 
AR Path="/5EEA60FC/5EE8F7DD" Ref="C21"  Part="1" 
F 0 "C21" H 2315 7171 50  0000 L CNN
F 1 "0.1uF" H 2315 7080 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 2238 6975 50  0001 C CNN
F 3 "~" H 2200 7125 50  0001 C CNN
	1    2200 7125
	1    0    0    -1  
$EndComp
Connection ~ 2200 6975
Wire Wire Line
	2200 6975 2375 6975
Connection ~ 1000 6975
Wire Wire Line
	1000 6975 925  6975
Wire Wire Line
	1000 6975 1375 6975
Wire Wire Line
	1375 6975 1375 7175
Connection ~ 1375 6975
Wire Wire Line
	1000 7275 1000 7425
Wire Wire Line
	1000 7425 1875 7425
Wire Wire Line
	2200 7275 2200 7425
Wire Wire Line
	2200 7425 1975 7425
Connection ~ 1975 7425
$Comp
L Connector_Generic:Conn_01x05 J14
U 1 1 5EFE1647
P 10325 4525
F 0 "J14" H 10405 4567 50  0000 L CNN
F 1 "Conn_01x05" H 10405 4476 50  0000 L CNN
F 2 ".Connector:B2B_Flex_05_Dual_Row_38milx24mil_Pad_20mil_copy" H 10325 4525 50  0001 C CNN
F 3 "~" H 10325 4525 50  0001 C CNN
	1    10325 4525
	1    0    0    -1  
$EndComp
$Comp
L .Oscillator:SMD_CRYSTAL Y1
U 1 1 5F019C81
P 1575 2675
F 0 "Y1" H 1375 2775 50  0000 L CNN
F 1 "SMD_CRYSTAL" H 1625 2850 50  0000 L CNN
F 2 ".Oscillator:Epson_Crystal_2_5x2" H 1525 2750 50  0001 C CNN
F 3 "" H 1575 2675 50  0001 C CNN
	1    1575 2675
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J17
U 1 1 5F03F7CC
P 10325 5450
F 0 "J17" H 10405 5446 50  0000 L CNN
F 1 "Conn_01x01" H 10405 5446 50  0001 L CNN
F 2 ".Connector:Spring_2306654_3" H 10325 5450 50  0001 C CNN
F 3 "~" H 10325 5450 50  0001 C CNN
	1    10325 5450
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J18
U 1 1 5F041FF0
P 10325 5550
F 0 "J18" H 10405 5546 50  0000 L CNN
F 1 "Conn_01x01" H 10405 5546 50  0001 L CNN
F 2 ".Connector:Spring_2306654_3" H 10325 5550 50  0001 C CNN
F 3 "~" H 10325 5550 50  0001 C CNN
	1    10325 5550
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J19
U 1 1 5F0422AB
P 10325 5650
F 0 "J19" H 10405 5646 50  0000 L CNN
F 1 "Conn_01x01" H 10405 5646 50  0001 L CNN
F 2 ".Connector:Spring_2306654_3" H 10325 5650 50  0001 C CNN
F 3 "~" H 10325 5650 50  0001 C CNN
	1    10325 5650
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J20
U 1 1 5F04250E
P 10325 5750
F 0 "J20" H 10405 5746 50  0000 L CNN
F 1 "Conn_01x01" H 10405 5746 50  0001 L CNN
F 2 ".Connector:Spring_2306654_3" H 10325 5750 50  0001 C CNN
F 3 "~" H 10325 5750 50  0001 C CNN
	1    10325 5750
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J21
U 1 1 5F04281A
P 10325 5850
F 0 "J21" H 10405 5846 50  0000 L CNN
F 1 "Conn_01x01" H 10405 5846 50  0001 L CNN
F 2 ".Connector:Spring_2306654_3" H 10325 5850 50  0001 C CNN
F 3 "~" H 10325 5850 50  0001 C CNN
	1    10325 5850
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J22
U 1 1 5F042B28
P 10325 5950
F 0 "J22" H 10405 5946 50  0000 L CNN
F 1 "Conn_01x01" H 10405 5946 50  0001 L CNN
F 2 ".Connector:Spring_2306654_3" H 10325 5950 50  0001 C CNN
F 3 "~" H 10325 5950 50  0001 C CNN
	1    10325 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10125 5650 9575 5650
Wire Wire Line
	10125 5550 9575 5550
Wire Wire Line
	10125 5450 9575 5450
Wire Wire Line
	10125 5950 9575 5950
Wire Wire Line
	10125 5850 9575 5850
Wire Wire Line
	10125 5750 9575 5750
$Comp
L power:+5V #PWR0132
U 1 1 5F054EB1
P 9575 5450
F 0 "#PWR0132" H 9575 5300 50  0001 C CNN
F 1 "+5V" H 9590 5623 50  0000 C CNN
F 2 "" H 9575 5450 50  0001 C CNN
F 3 "" H 9575 5450 50  0001 C CNN
	1    9575 5450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 5F055572
P 9575 5950
F 0 "#PWR0133" H 9575 5700 50  0001 C CNN
F 1 "GND" H 9580 5777 50  0000 C CNN
F 2 "" H 9575 5950 50  0001 C CNN
F 3 "" H 9575 5950 50  0001 C CNN
	1    9575 5950
	1    0    0    -1  
$EndComp
Text Label 9575 5550 0    50   ~ 0
SPI0_MISO
Text Label 9575 5650 0    50   ~ 0
SPI0_MOSI
Text Label 9575 5750 0    50   ~ 0
SPI0_SPCK
Text Notes 4975 7700 0    50   Italic 10
NEED TO ACTUALLY USE THE SAMS VARIANT
Text Label 9575 5850 0    50   ~ 0
SPI0_NPCS0
Text Notes 9225 6300 0    50   Italic 10
Make sure NPCS0 is the correct one
$Comp
L power:GND #PWR0134
U 1 1 5EEBC107
P 10275 3100
F 0 "#PWR0134" H 10275 2850 50  0001 C CNN
F 1 "GND" H 10280 2927 50  0000 C CNN
F 2 "" H 10275 3100 50  0001 C CNN
F 3 "" H 10275 3100 50  0001 C CNN
	1    10275 3100
	1    0    0    -1  
$EndComp
$Comp
L .Device:C C18
U 1 1 5F0DB9E4
P 10275 2950
F 0 "C18" H 10390 2996 50  0000 L CNN
F 1 "0.1uF" H 10390 2905 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 10313 2800 50  0001 C CNN
F 3 "~" H 10275 2950 50  0001 C CNN
	1    10275 2950
	1    0    0    -1  
$EndComp
$Comp
L .Device:C C19
U 1 1 5F0E6E88
P 10275 3600
F 0 "C19" H 10390 3646 50  0000 L CNN
F 1 "0.1uF" H 10390 3555 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 10313 3450 50  0001 C CNN
F 3 "~" H 10275 3600 50  0001 C CNN
	1    10275 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9375 1800 9675 1800
$Comp
L .Device:C C14
U 1 1 5F0ED1AA
P 9675 1950
F 0 "C14" H 9790 1996 50  0000 L CNN
F 1 "0.1uF" H 9790 1905 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 9713 1800 50  0001 C CNN
F 3 "~" H 9675 1950 50  0001 C CNN
	1    9675 1950
	1    0    0    -1  
$EndComp
Connection ~ 9675 1800
Wire Wire Line
	9675 1800 10150 1800
$Comp
L .Device:C C15
U 1 1 5F0ED59D
P 10150 1950
F 0 "C15" H 10265 1996 50  0000 L CNN
F 1 "0.1uF" H 10265 1905 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 10188 1800 50  0001 C CNN
F 3 "~" H 10150 1950 50  0001 C CNN
	1    10150 1950
	1    0    0    -1  
$EndComp
Connection ~ 10150 1800
Wire Wire Line
	10150 1800 10650 1800
Wire Wire Line
	10150 2100 9675 2100
$Comp
L power:GND #PWR0135
U 1 1 5F0F7943
P 10150 2100
F 0 "#PWR0135" H 10150 1850 50  0001 C CNN
F 1 "GND" H 10155 1927 50  0000 C CNN
F 2 "" H 10150 2100 50  0001 C CNN
F 3 "" H 10150 2100 50  0001 C CNN
	1    10150 2100
	1    0    0    -1  
$EndComp
Connection ~ 10150 2100
$Comp
L .Device:C C7
U 1 1 5F0F8513
P 10650 1100
F 0 "C7" H 10765 1146 50  0000 L CNN
F 1 "0.1uF" H 10765 1055 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 10688 950 50  0001 C CNN
F 3 "~" H 10650 1100 50  0001 C CNN
	1    10650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 950  10250 950 
$Comp
L .Device:C C6
U 1 1 5F0FFEFC
P 10250 1100
F 0 "C6" H 10365 1146 50  0000 L CNN
F 1 "1uF" H 10365 1055 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 10288 950 50  0001 C CNN
F 3 "~" H 10250 1100 50  0001 C CNN
	1    10250 1100
	1    0    0    -1  
$EndComp
Connection ~ 10250 950 
Wire Wire Line
	10250 950  10650 950 
$Comp
L .Device:C C10
U 1 1 5F1054F4
P 9350 1200
F 0 "C10" H 9465 1246 50  0000 L CNN
F 1 "0.1uF" H 9465 1155 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 9388 1050 50  0001 C CNN
F 3 "~" H 9350 1200 50  0001 C CNN
	1    9350 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 1250 10650 1250
$Comp
L power:GND #PWR0136
U 1 1 5F10A644
P 10650 1250
F 0 "#PWR0136" H 10650 1000 50  0001 C CNN
F 1 "GND" H 10655 1077 50  0000 C CNN
F 2 "" H 10650 1250 50  0001 C CNN
F 3 "" H 10650 1250 50  0001 C CNN
	1    10650 1250
	1    0    0    -1  
$EndComp
Connection ~ 10650 1250
$Comp
L power:GND #PWR0137
U 1 1 5F10A9D2
P 9350 1350
F 0 "#PWR0137" H 9350 1100 50  0001 C CNN
F 1 "GND" H 9355 1177 50  0000 C CNN
F 2 "" H 9350 1350 50  0001 C CNN
F 3 "" H 9350 1350 50  0001 C CNN
	1    9350 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8125 1000 8275 1000
$Comp
L .Device:C C8
U 1 1 5F111378
P 8275 1150
F 0 "C8" H 8390 1196 50  0000 L CNN
F 1 "0.1uF" H 8390 1105 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 8313 1000 50  0001 C CNN
F 3 "~" H 8275 1150 50  0001 C CNN
	1    8275 1150
	1    0    0    -1  
$EndComp
Connection ~ 8275 1000
Wire Wire Line
	8275 1000 8800 1000
Wire Wire Line
	7250 1775 7350 1775
$Comp
L .Device:C C11
U 1 1 5F11AA4A
P 7350 1925
F 0 "C11" H 7465 1971 50  0000 L CNN
F 1 "0.1uF" H 7465 1880 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 7388 1775 50  0001 C CNN
F 3 "~" H 7350 1925 50  0001 C CNN
	1    7350 1925
	1    0    0    -1  
$EndComp
Connection ~ 7350 1775
Wire Wire Line
	7350 1775 7775 1775
$Comp
L .Device:C C12
U 1 1 5F11AF96
P 7775 1925
F 0 "C12" H 7890 1971 50  0000 L CNN
F 1 "0.1uF" H 7890 1880 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 7813 1775 50  0001 C CNN
F 3 "~" H 7775 1925 50  0001 C CNN
	1    7775 1925
	1    0    0    -1  
$EndComp
Connection ~ 7775 1775
Wire Wire Line
	7775 1775 8200 1775
$Comp
L .Device:C C13
U 1 1 5F11B2F1
P 8200 1925
F 0 "C13" H 8315 1971 50  0000 L CNN
F 1 "0.1uF" H 8315 1880 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 8238 1775 50  0001 C CNN
F 3 "~" H 8200 1925 50  0001 C CNN
	1    8200 1925
	1    0    0    -1  
$EndComp
Connection ~ 8200 1775
Wire Wire Line
	8200 1775 8575 1775
Wire Wire Line
	8200 2075 7775 2075
Connection ~ 7775 2075
Wire Wire Line
	7775 2075 7350 2075
$Comp
L power:GND #PWR0138
U 1 1 5F120124
P 8200 2075
F 0 "#PWR0138" H 8200 1825 50  0001 C CNN
F 1 "GND" H 8205 1902 50  0000 C CNN
F 2 "" H 8200 2075 50  0001 C CNN
F 3 "" H 8200 2075 50  0001 C CNN
	1    8200 2075
	1    0    0    -1  
$EndComp
Connection ~ 8200 2075
$Comp
L power:GND #PWR0139
U 1 1 5F120598
P 8275 1300
F 0 "#PWR0139" H 8275 1050 50  0001 C CNN
F 1 "GND" H 8280 1127 50  0000 C CNN
F 2 "" H 8275 1300 50  0001 C CNN
F 3 "" H 8275 1300 50  0001 C CNN
	1    8275 1300
	1    0    0    -1  
$EndComp
$Comp
L .Device:C_Small C17
U 1 1 5F123563
P 2000 2775
F 0 "C17" H 2092 2821 50  0000 L CNN
F 1 "12pF" H 2092 2730 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 2000 2775 50  0001 C CNN
F 3 "~" H 2000 2775 50  0001 C CNN
	1    2000 2775
	1    0    0    -1  
$EndComp
$Comp
L .Device:C_Small C16
U 1 1 5F1240BC
P 1150 2775
F 0 "C16" H 1242 2821 50  0000 L CNN
F 1 "12pF" H 1242 2730 50  0000 L CNN
F 2 ".Capacitor:C_0201_0603Metric_L" H 1150 2775 50  0001 C CNN
F 3 "~" H 1150 2775 50  0001 C CNN
	1    1150 2775
	1    0    0    -1  
$EndComp
$Comp
L .Transistor:MMBT3904LP-7B Q1
U 1 1 5F16F091
P 1850 5000
F 0 "Q1" H 2041 5046 50  0000 L CNN
F 1 "MMBT3904LP-7B" H 2041 4955 50  0000 L CNN
F 2 ".Transistor:TRXDFN3_101X61X53L25X15N" H 2050 4925 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds31835.pdf" H 1850 5000 50  0001 L CNN
	1    1850 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 5200 1950 5375
Wire Wire Line
	1950 4125 1950 3950
Wire Wire Line
	1650 5000 1525 5000
$Comp
L .Device:R_Small_US R8
U 1 1 5F17F583
P 1425 5000
F 0 "R8" H 1493 5046 50  0000 L CNN
F 1 "1.5K" H 1493 4955 50  0000 L CNN
F 2 ".Resistor:R_0201_0603Metric_ERJ_L" H 1425 5000 50  0001 C CNN
F 3 "~" H 1425 5000 50  0001 C CNN
	1    1425 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	1325 5000 900  5000
Wire Wire Line
	1950 4700 1950 4800
Wire Wire Line
	1950 4325 1950 4500
$Comp
L power:+5V #PWR?
U 1 1 5F19718E
P 1950 3950
AR Path="/5F19718E" Ref="#PWR?"  Part="1" 
AR Path="/5EEA60FC/5F19718E" Ref="#PWR03"  Part="1" 
F 0 "#PWR03" H 1950 3800 50  0001 C CNN
F 1 "+5V" H 1965 4123 50  0000 C CNN
F 2 "" H 1950 3950 50  0001 C CNN
F 3 "" H 1950 3950 50  0001 C CNN
	1    1950 3950
	1    0    0    -1  
$EndComp
Text Label 1225 2675 0    50   ~ 0
XOUT
Text Label 1800 2675 0    50   ~ 0
XIN
$EndSCHEMATC
