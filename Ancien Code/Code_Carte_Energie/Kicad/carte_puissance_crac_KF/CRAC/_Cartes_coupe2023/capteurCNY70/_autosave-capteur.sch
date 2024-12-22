EESchema Schematic File Version 5
EELAYER 36 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
Connection ~ 4350 2000
Connection ~ 4400 3550
Connection ~ 4700 2900
Wire Wire Line
	3150 2500 3350 2500
Wire Wire Line
	3150 2600 3450 2600
Wire Wire Line
	3150 2700 3450 2700
Wire Wire Line
	4100 2000 4100 2200
Wire Wire Line
	4100 2000 4350 2000
Wire Wire Line
	4100 2700 4100 3550
Wire Wire Line
	4100 3550 4400 3550
Wire Wire Line
	4400 3550 4700 3550
Wire Wire Line
	4500 2900 4700 2900
Wire Wire Line
	4700 2000 4350 2000
Wire Wire Line
	4700 2500 4700 2000
Wire Wire Line
	4700 2700 4700 2900
Wire Wire Line
	4700 2900 4700 2950
Wire Wire Line
	4700 3250 4700 3550
Text Label 3450 2600 0    50   ~ 0
Capt
Text Label 4500 2900 0    50   ~ 10
Capt
$Comp
L capteur-rescue:+5V-power #PWR0101
U 1 1 00000000
P 3450 2700
F 0 "#PWR0101" H 3450 2550 50  0001 C CNN
F 1 "+5V" H 3465 2873 50  0000 C CNN
F 2 "" H 3450 2700 50  0001 C CNN
F 3 "" H 3450 2700 50  0001 C CNN
	1    3450 2700
	0    1    1    0   
$EndComp
$Comp
L capteur-rescue:+5V-power #PWR0106
U 1 1 61619A62
P 4350 2000
F 0 "#PWR0106" H 4350 1850 50  0001 C CNN
F 1 "+5V" H 4365 2173 50  0000 C CNN
F 2 "" H 4350 2000 50  0001 C CNN
F 3 "" H 4350 2000 50  0001 C CNN
	1    4350 2000
	1    0    0    -1  
$EndComp
$Comp
L capteur-rescue:GND-power #PWR0112
U 1 1 615BE43C
P 3350 2500
F 0 "#PWR0112" H 3350 2250 50  0001 C CNN
F 1 "GND" V 3355 2372 50  0000 R CNN
F 2 "" H 3350 2500 50  0001 C CNN
F 3 "" H 3350 2500 50  0001 C CNN
	1    3350 2500
	0    -1   -1   0   
$EndComp
$Comp
L capteur-rescue:GND-power #PWR0107
U 1 1 616203E8
P 4400 3550
F 0 "#PWR0107" H 4400 3300 50  0001 C CNN
F 1 "GND" H 4405 3377 50  0000 C CNN
F 2 "" H 4400 3550 50  0001 C CNN
F 3 "" H 4400 3550 50  0001 C CNN
	1    4400 3550
	1    0    0    -1  
$EndComp
$Comp
L SymbGamelGe2:R R1
U 1 1 6155C049
P 4100 2350
F 0 "R1" H 4170 2396 50  0000 L CNN
F 1 "100" H 4170 2305 50  0000 L CNN
F 2 "FootprintGamelGe2:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4030 2350 50  0001 C CNN
F 3 "~" H 4100 2350 50  0001 C CNN
	1    4100 2350
	1    0    0    -1  
$EndComp
$Comp
L SymbGamelGe2:R R5
U 1 1 6155DF8A
P 4700 3100
F 0 "R5" H 4770 3146 50  0000 L CNN
F 1 "5.6k" H 4770 3055 50  0000 L CNN
F 2 "FootprintGamelGe2:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4630 3100 50  0001 C CNN
F 3 "~" H 4700 3100 50  0001 C CNN
	1    4700 3100
	1    0    0    -1  
$EndComp
$Comp
L SymbGamelGe2:Mouting_Screw_3mm J5
U 1 1 615D7EE6
P 2500 1400
F 0 "J5" H 2648 1446 50  0000 L CNN
F 1 "Mouting_Screw_3mm" H 2648 1355 50  0000 L CNN
F 2 "FootprintGamelGe2:Mouting_Screw_3mm" H 2550 1400 50  0001 C CNN
F 3 "~" H 2550 1400 50  0001 C CNN
	1    2500 1400
	-1   0    0    1   
$EndComp
$Comp
L SymbGamelGe2:Mouting_Screw_3mm J6
U 1 1 615DA206
P 2600 5200
F 0 "J6" H 2748 5246 50  0000 L CNN
F 1 "Mouting_Screw_3mm" H 2748 5155 50  0000 L CNN
F 2 "FootprintGamelGe2:Mouting_Screw_3mm" H 2650 5200 50  0001 C CNN
F 3 "~" H 2650 5200 50  0001 C CNN
	1    2600 5200
	-1   0    0    1   
$EndComp
$Comp
L 0_SymbGamelGe2:Autocom3 J1
U 1 1 00000000
P 2950 2600
F 0 "J1" H 2950 2250 50  0000 C CNN
F 1 "Autocom3" H 2950 2350 50  0000 C CNN
F 2 "FootprintGamelGe2:Autocom3" H 2950 2600 50  0001 C CNN
F 3 "~" H 2950 2600 50  0001 C CNN
	1    2950 2600
	-1   0    0    1   
$EndComp
$Comp
L SymbGamelGe2:CNY70 U1
U 1 1 6154AC2A
P 4400 2600
F 0 "U1" H 4400 2917 50  0000 C CNN
F 1 "CNY70" H 4400 2826 50  0000 C CNN
F 2 "FootprintGamelGe2:Vishay_CNY70" H 4400 2400 50  0001 C CNN
F 3 "https://www.vishay.com/docs/83751/cny70.pdf" H 4370 2510 50  0001 C CNN
	1    4400 2600
	1    0    0    -1  
$EndComp
$EndSCHEMATC
