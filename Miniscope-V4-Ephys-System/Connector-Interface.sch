EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 4
Title "E-Scope System"
Date "2020-07-15"
Rev ""
Comp ""
Comment1 "Daniel Aharoni"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F12E1C6
P 5225 3525
AR Path="/5EEA60FC/5F12E1C6" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F12E1C6" Ref="J3"  Part="1" 
F 0 "J3" H 5305 3521 50  0000 L CNN
F 1 "Conn_01x01" H 5305 3521 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 5225 3525 50  0001 C CNN
F 3 "~" H 5225 3525 50  0001 C CNN
	1    5225 3525
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F12E1CC
P 5225 3625
AR Path="/5EEA60FC/5F12E1CC" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F12E1CC" Ref="J5"  Part="1" 
F 0 "J5" H 5305 3621 50  0000 L CNN
F 1 "Conn_01x01" H 5305 3621 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 5225 3625 50  0001 C CNN
F 3 "~" H 5225 3625 50  0001 C CNN
	1    5225 3625
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F12E1D2
P 5225 3725
AR Path="/5EEA60FC/5F12E1D2" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F12E1D2" Ref="J7"  Part="1" 
F 0 "J7" H 5305 3721 50  0000 L CNN
F 1 "Conn_01x01" H 5305 3721 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 5225 3725 50  0001 C CNN
F 3 "~" H 5225 3725 50  0001 C CNN
	1    5225 3725
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F12E1D8
P 5225 3825
AR Path="/5EEA60FC/5F12E1D8" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F12E1D8" Ref="J9"  Part="1" 
F 0 "J9" H 5305 3821 50  0000 L CNN
F 1 "Conn_01x01" H 5305 3821 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 5225 3825 50  0001 C CNN
F 3 "~" H 5225 3825 50  0001 C CNN
	1    5225 3825
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F12E1DE
P 5225 3925
AR Path="/5EEA60FC/5F12E1DE" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F12E1DE" Ref="J11"  Part="1" 
F 0 "J11" H 5305 3921 50  0000 L CNN
F 1 "Conn_01x01" H 5305 3921 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 5225 3925 50  0001 C CNN
F 3 "~" H 5225 3925 50  0001 C CNN
	1    5225 3925
	1    0    0    -1  
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F12E1E4
P 5225 4025
AR Path="/5EEA60FC/5F12E1E4" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F12E1E4" Ref="J13"  Part="1" 
F 0 "J13" H 5305 4021 50  0000 L CNN
F 1 "Conn_01x01" H 5305 4021 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 5225 4025 50  0001 C CNN
F 3 "~" H 5225 4025 50  0001 C CNN
	1    5225 4025
	1    0    0    -1  
$EndComp
Wire Wire Line
	5025 3525 4475 3525
Wire Wire Line
	5025 4025 4475 4025
$Comp
L power:+5V #PWR?
U 1 1 5F12E1F0
P 4475 3525
AR Path="/5EEA60FC/5F12E1F0" Ref="#PWR?"  Part="1" 
AR Path="/5EEA5765/5F12E1F0" Ref="#PWR0113"  Part="1" 
F 0 "#PWR0113" H 4475 3375 50  0001 C CNN
F 1 "+5V" H 4490 3698 50  0000 C CNN
F 2 "" H 4475 3525 50  0001 C CNN
F 3 "" H 4475 3525 50  0001 C CNN
	1    4475 3525
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F12E1F6
P 4475 4025
AR Path="/5EEA60FC/5F12E1F6" Ref="#PWR?"  Part="1" 
AR Path="/5EEA5765/5F12E1F6" Ref="#PWR0114"  Part="1" 
F 0 "#PWR0114" H 4475 3775 50  0001 C CNN
F 1 "GND" H 4480 3852 50  0000 C CNN
F 2 "" H 4475 4025 50  0001 C CNN
F 3 "" H 4475 4025 50  0001 C CNN
	1    4475 4025
	1    0    0    -1  
$EndComp
Text Label 4475 3625 0    50   ~ 0
MISO_MCU
Text Label 4475 3725 0    50   ~ 0
MOSI_MCU
Text Label 4475 3825 0    50   ~ 0
SPCK_MCU
Text Label 4475 3925 0    50   ~ 0
CS_MCU
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F13234F
P 4075 3525
AR Path="/5EEA60FC/5F13234F" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F13234F" Ref="J2"  Part="1" 
F 0 "J2" H 4155 3521 50  0000 L CNN
F 1 "Conn_01x01" H 4155 3521 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 4075 3525 50  0001 C CNN
F 3 "~" H 4075 3525 50  0001 C CNN
	1    4075 3525
	-1   0    0    1   
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F1332E7
P 4075 3625
AR Path="/5EEA60FC/5F1332E7" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F1332E7" Ref="J4"  Part="1" 
F 0 "J4" H 4155 3621 50  0000 L CNN
F 1 "Conn_01x01" H 4155 3621 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 4075 3625 50  0001 C CNN
F 3 "~" H 4075 3625 50  0001 C CNN
	1    4075 3625
	-1   0    0    1   
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F13346A
P 4075 3725
AR Path="/5EEA60FC/5F13346A" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F13346A" Ref="J6"  Part="1" 
F 0 "J6" H 4155 3721 50  0000 L CNN
F 1 "Conn_01x01" H 4155 3721 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 4075 3725 50  0001 C CNN
F 3 "~" H 4075 3725 50  0001 C CNN
	1    4075 3725
	-1   0    0    1   
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F13362C
P 4075 3825
AR Path="/5EEA60FC/5F13362C" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F13362C" Ref="J8"  Part="1" 
F 0 "J8" H 4155 3821 50  0000 L CNN
F 1 "Conn_01x01" H 4155 3821 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 4075 3825 50  0001 C CNN
F 3 "~" H 4075 3825 50  0001 C CNN
	1    4075 3825
	-1   0    0    1   
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F133825
P 4075 3925
AR Path="/5EEA60FC/5F133825" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F133825" Ref="J10"  Part="1" 
F 0 "J10" H 4155 3921 50  0000 L CNN
F 1 "Conn_01x01" H 4155 3921 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 4075 3925 50  0001 C CNN
F 3 "~" H 4075 3925 50  0001 C CNN
	1    4075 3925
	-1   0    0    1   
$EndComp
$Comp
L .Connector:Conn_01x01 J?
U 1 1 5F133A13
P 4075 4025
AR Path="/5EEA60FC/5F133A13" Ref="J?"  Part="1" 
AR Path="/5EEA5765/5F133A13" Ref="J12"  Part="1" 
F 0 "J12" H 4155 4021 50  0000 L CNN
F 1 "Conn_01x01" H 4155 4021 50  0001 L CNN
F 2 ".Connector:Conn_1x1_250x750_Pad" H 4075 4025 50  0001 C CNN
F 3 "~" H 4075 4025 50  0001 C CNN
	1    4075 4025
	-1   0    0    1   
$EndComp
Wire Wire Line
	4475 4025 4275 4025
Connection ~ 4475 4025
Wire Wire Line
	4275 3925 5025 3925
Wire Wire Line
	4275 3825 5025 3825
Wire Wire Line
	4275 3725 5025 3725
Wire Wire Line
	4275 3625 5025 3625
Wire Wire Line
	4475 3525 4275 3525
Connection ~ 4475 3525
Text Notes 4850 4275 0    50   Italic 10
Do not populate these pads
$EndSCHEMATC
