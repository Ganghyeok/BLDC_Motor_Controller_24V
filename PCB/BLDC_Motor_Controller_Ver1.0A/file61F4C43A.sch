EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr User 9843 7087
encoding utf-8
Sheet 3 3
Title "USB to UART Converter"
Date "2020-11-05"
Rev "Ver1.0A"
Comp "ArkX"
Comment1 "Designed by Ganghyeok Lim"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:USB_B_Mini J6
U 1 1 61F4CEE5
P 1400 2150
F 0 "J6" H 1457 2617 50  0000 C CNN
F 1 "USB_B_Mini" H 1457 2526 50  0000 C CNN
F 2 "" H 1550 2100 50  0001 C CNN
F 3 "~" H 1550 2100 50  0001 C CNN
	1    1400 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 2550 1300 2600
Wire Wire Line
	1300 2600 1350 2600
Wire Wire Line
	1400 2600 1400 2550
Wire Wire Line
	1350 2650 1350 2600
Connection ~ 1350 2600
Wire Wire Line
	1350 2600 1400 2600
$Comp
L power:GND #PWR0151
U 1 1 61F4D6C6
P 1350 2650
F 0 "#PWR0151" H 1350 2400 50  0001 C CNN
F 1 "GND" H 1355 2477 50  0000 C CNN
F 2 "" H 1350 2650 50  0001 C CNN
F 3 "" H 1350 2650 50  0001 C CNN
	1    1350 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 1950 1950 1950
Wire Wire Line
	1950 1950 1950 1750
$Comp
L power:VBUS #PWR0152
U 1 1 61F4DC4C
P 1950 1750
F 0 "#PWR0152" H 1950 1600 50  0001 C CNN
F 1 "VBUS" H 1965 1923 50  0000 C CNN
F 2 "" H 1950 1750 50  0001 C CNN
F 3 "" H 1950 1750 50  0001 C CNN
	1    1950 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 2150 2000 2150
Wire Wire Line
	1700 2250 2400 2250
NoConn ~ 1700 2350
$Comp
L Device:C C36
U 1 1 61F4E2CD
P 2000 2550
F 0 "C36" H 2115 2596 50  0000 L CNN
F 1 "15pF" H 2115 2505 50  0000 L CNN
F 2 "" H 2038 2400 50  0001 C CNN
F 3 "~" H 2000 2550 50  0001 C CNN
	1    2000 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C37
U 1 1 61F4E53F
P 2400 2550
F 0 "C37" H 2515 2596 50  0000 L CNN
F 1 "15pF" H 2515 2505 50  0000 L CNN
F 2 "" H 2438 2400 50  0001 C CNN
F 3 "~" H 2400 2550 50  0001 C CNN
	1    2400 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2400 2000 2150
Connection ~ 2000 2150
Wire Wire Line
	2000 2150 2800 2150
Wire Wire Line
	2400 2400 2400 2250
Connection ~ 2400 2250
Wire Wire Line
	2400 2250 2800 2250
Text GLabel 2800 2150 2    50   BiDi ~ 0
D+
Text GLabel 2800 2250 2    50   BiDi ~ 0
D-
$Comp
L power:GND #PWR0153
U 1 1 61F4F333
P 2000 2850
F 0 "#PWR0153" H 2000 2600 50  0001 C CNN
F 1 "GND" H 2005 2677 50  0000 C CNN
F 2 "" H 2000 2850 50  0001 C CNN
F 3 "" H 2000 2850 50  0001 C CNN
	1    2000 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0154
U 1 1 61F4F4DC
P 2400 2850
F 0 "#PWR0154" H 2400 2600 50  0001 C CNN
F 1 "GND" H 2405 2677 50  0000 C CNN
F 2 "" H 2400 2850 50  0001 C CNN
F 3 "" H 2400 2850 50  0001 C CNN
	1    2400 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2700 2000 2850
Wire Wire Line
	2400 2700 2400 2850
$Comp
L power:VBUS #PWR0155
U 1 1 61F50E71
P 3650 1650
F 0 "#PWR0155" H 3650 1500 50  0001 C CNN
F 1 "VBUS" H 3665 1823 50  0000 C CNN
F 2 "" H 3650 1650 50  0001 C CNN
F 3 "" H 3650 1650 50  0001 C CNN
	1    3650 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D20
U 1 1 61F51213
P 3650 2050
F 0 "D20" V 3689 1932 50  0000 R CNN
F 1 "LED" V 3598 1932 50  0000 R CNN
F 2 "" H 3650 2050 50  0001 C CNN
F 3 "~" H 3650 2050 50  0001 C CNN
	1    3650 2050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R33
U 1 1 61F51832
P 3650 2600
F 0 "R33" H 3718 2646 50  0000 L CNN
F 1 "330" H 3718 2555 50  0000 L CNN
F 2 "" V 3690 2590 50  0001 C CNN
F 3 "~" H 3650 2600 50  0001 C CNN
	1    3650 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0156
U 1 1 61F51BED
P 3650 3000
F 0 "#PWR0156" H 3650 2750 50  0001 C CNN
F 1 "GND" H 3655 2827 50  0000 C CNN
F 2 "" H 3650 3000 50  0001 C CNN
F 3 "" H 3650 3000 50  0001 C CNN
	1    3650 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 1650 3650 1900
Wire Wire Line
	3650 2200 3650 2450
Wire Wire Line
	3650 2750 3650 3000
$Comp
L My_Libraries:CP2102 U8
U 1 1 61F53DBF
P 7500 3100
F 0 "U8" H 7500 4265 50  0000 C CNN
F 1 "CP2102" H 7500 4174 50  0000 C CNN
F 2 "" H 7450 3350 50  0001 C CNN
F 3 "" H 7450 3350 50  0001 C CNN
	1    7500 3100
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0157
U 1 1 61F55ED0
P 5200 1400
F 0 "#PWR0157" H 5200 1250 50  0001 C CNN
F 1 "VBUS" H 5215 1573 50  0000 C CNN
F 2 "" H 5200 1400 50  0001 C CNN
F 3 "" H 5200 1400 50  0001 C CNN
	1    5200 1400
	1    0    0    -1  
$EndComp
Text GLabel 8600 1300 2    50   Input ~ 0
+3.3V
$Comp
L Device:CP1 C39
U 1 1 61F56BB0
P 8550 2550
F 0 "C39" H 8665 2596 50  0000 L CNN
F 1 "1uF" H 8665 2505 50  0000 L CNN
F 2 "" H 8550 2550 50  0001 C CNN
F 3 "~" H 8550 2550 50  0001 C CNN
	1    8550 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C38
U 1 1 61F56D03
P 5200 3100
F 0 "C38" H 5315 3146 50  0000 L CNN
F 1 "1uF" H 5315 3055 50  0000 L CNN
F 2 "" H 5200 3100 50  0001 C CNN
F 3 "~" H 5200 3100 50  0001 C CNN
	1    5200 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R36
U 1 1 61F5726F
P 6600 2200
F 0 "R36" V 6395 2200 50  0000 C CNN
F 1 "4.7k" V 6486 2200 50  0000 C CNN
F 2 "" V 6640 2190 50  0001 C CNN
F 3 "~" H 6600 2200 50  0001 C CNN
	1    6600 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	7000 2200 6750 2200
Wire Wire Line
	6450 2200 6100 2200
Wire Wire Line
	6100 2200 6100 1800
Wire Wire Line
	6100 1800 8550 1800
Wire Wire Line
	7000 2300 6100 2300
Wire Wire Line
	6100 2300 6100 2200
Connection ~ 6100 2200
NoConn ~ 8000 2500
NoConn ~ 8000 2600
NoConn ~ 8000 2700
NoConn ~ 8000 2800
NoConn ~ 8000 2900
NoConn ~ 8000 3000
NoConn ~ 8000 3100
NoConn ~ 8000 3200
NoConn ~ 8000 3300
NoConn ~ 8000 3400
NoConn ~ 8000 3500
NoConn ~ 7000 3200
NoConn ~ 7000 3300
NoConn ~ 7000 3400
NoConn ~ 7000 3500
NoConn ~ 7000 3600
NoConn ~ 7000 3700
NoConn ~ 7000 3900
NoConn ~ 7000 4000
$Comp
L power:GND #PWR0158
U 1 1 61F5D411
P 8100 4000
F 0 "#PWR0158" H 8100 3750 50  0001 C CNN
F 1 "GND" H 8105 3827 50  0000 C CNN
F 2 "" H 8100 4000 50  0001 C CNN
F 3 "" H 8100 4000 50  0001 C CNN
	1    8100 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 3900 8100 3900
Wire Wire Line
	8100 3900 8100 4000
Wire Wire Line
	8000 2200 8550 2200
Wire Wire Line
	8550 1800 8550 2200
Wire Wire Line
	8550 2200 8550 2400
Connection ~ 8550 2200
Wire Wire Line
	8600 1300 8550 1300
Wire Wire Line
	8550 1300 8550 1800
Connection ~ 8550 1800
Wire Wire Line
	5200 1400 5200 2400
$Comp
L power:GND #PWR0159
U 1 1 61F6249D
P 8550 2900
F 0 "#PWR0159" H 8550 2650 50  0001 C CNN
F 1 "GND" H 8555 2727 50  0000 C CNN
F 2 "" H 8550 2900 50  0001 C CNN
F 3 "" H 8550 2900 50  0001 C CNN
	1    8550 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 2700 8550 2900
Text GLabel 6650 2600 0    50   BiDi ~ 0
D+
Text GLabel 6650 2700 0    50   BiDi ~ 0
D-
Wire Wire Line
	6650 2600 7000 2600
Wire Wire Line
	6650 2700 7000 2700
$Comp
L Device:LED D22
U 1 1 61F652D5
P 6550 3950
F 0 "D22" V 6497 4030 50  0000 L CNN
F 1 "LED" V 6588 4030 50  0000 L CNN
F 2 "" H 6550 3950 50  0001 C CNN
F 3 "~" H 6550 3950 50  0001 C CNN
	1    6550 3950
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R35
U 1 1 61F65794
P 6550 3400
F 0 "R35" H 6618 3446 50  0000 L CNN
F 1 "150" H 6618 3355 50  0000 L CNN
F 2 "" V 6590 3390 50  0001 C CNN
F 3 "~" H 6550 3400 50  0001 C CNN
	1    6550 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R34
U 1 1 61F65BE5
P 6100 3400
F 0 "R34" H 6168 3446 50  0000 L CNN
F 1 "150" H 6168 3355 50  0000 L CNN
F 2 "" V 6140 3390 50  0001 C CNN
F 3 "~" H 6100 3400 50  0001 C CNN
	1    6100 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D21
U 1 1 61F66312
P 6100 3950
F 0 "D21" V 6047 4030 50  0000 L CNN
F 1 "LED" V 6138 4030 50  0000 L CNN
F 2 "" H 6100 3950 50  0001 C CNN
F 3 "~" H 6100 3950 50  0001 C CNN
	1    6100 3950
	0    1    1    0   
$EndComp
Text GLabel 5850 2900 0    50   Input ~ 0
TXD
Text GLabel 5850 3000 0    50   Output ~ 0
RXD
Wire Wire Line
	6100 3550 6100 3800
Wire Wire Line
	6550 3550 6550 3800
Wire Wire Line
	5850 2900 6100 2900
Wire Wire Line
	5850 3000 6550 3000
Wire Wire Line
	6100 3250 6100 2900
Connection ~ 6100 2900
Wire Wire Line
	6100 2900 7000 2900
Wire Wire Line
	6550 3250 6550 3000
Connection ~ 6550 3000
Wire Wire Line
	6550 3000 7000 3000
Wire Wire Line
	5200 2400 7000 2400
$Comp
L power:GND #PWR0160
U 1 1 61F71315
P 5200 3800
F 0 "#PWR0160" H 5200 3550 50  0001 C CNN
F 1 "GND" H 5205 3627 50  0000 C CNN
F 2 "" H 5200 3800 50  0001 C CNN
F 3 "" H 5200 3800 50  0001 C CNN
	1    5200 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2400 5200 2950
Connection ~ 5200 2400
Wire Wire Line
	5200 3250 5200 3800
Wire Wire Line
	6550 4100 6550 4500
Wire Wire Line
	6550 4500 6100 4500
Wire Wire Line
	6100 4100 6100 4500
Connection ~ 6100 4500
Wire Wire Line
	6100 4500 5600 4500
Text GLabel 5600 4500 0    50   Input ~ 0
+3.3V
$EndSCHEMATC
