# Esp32Projects
This project contains basic examples of ESP32 programming, each in a different file but under the same solution.
Simply uncomment (or copy) the code of the file you want to test.

I have an ESP-WROOM-32, and the project is configure to use it (see file platformio.ini). 
Remember to install the PlatformIO extension in VSCode in order to use this project as it is.

## Examples
* __Ex1_BlinkLED.cpp__ - The "hello world" of ESP32 porograming: Blink a LED!
* __Ex2_Potenciometre_a_AdafruitIO.cpp__
	- Upload the values read from a potentiometer into an AdafruitIO feed 
	- Use the value read from the potentiometer to blink a led faster or slower.
* __Ex3_BME280_a_AdafruitIO.cpp__ 
	- Upload temperature/humidity/atmospheric pressure read from BME280 sensor into 3 AdafruitIO feeds. 
	- Turn on a led everytime we upload data into AdafruitIO.
	- Read toggle from AdafruitIO feed to turn on a different LED
	- Read slider from AdafruitIO feed to define the intensity of a LED using PWM
	<img src="https://github.com/Vegethalia/Esp32Projects/blob/master/images/Ex3_Schema.jpg" alt="Fast & Ugly Circuit Schema" width="300"/>
	<img src="https://github.com/Vegethalia/Esp32Projects/blob/master/images/Ex3_AdafruitDashboardExample.png" alt="Example of AdafruitIO dashboard" width="400"/>

* __Ex4_ScanWifiNetworks.cpp__
	- Scan available wifi networks and show them in the terminal (including 5GHz).
	- Also show how to read from terminal (`Serial.available()`, `Serial.read()`) and wait until 'R' is pressed to scan again.
	
	```
	scan start    
	scan done       
	9 networks found
 		0: 😡😷🙈🙉🙊😜 (-52db) Channel=6 *
 		1: Picadero (-53db) Channel=6 *
 		2: Picadero (-76db) Channel=1 *
 		3: DIRECT-AE-HP DeskJet 2600 series (-77db) Channel=11 *
 		4: MOVISTAR_3413 (-81db) Channel=1 *
 		5: Red Wi-Fi de ruben (-86db) Channel=11 *
 		6: MOVISTAR_DEE0 (-88db) Channel=6 *
 		7: Orange-F621 (-89db) Channel=6 *
 		8: JORDI_14 (-90db) Channel=1 *

	Connecting to WIFI...
	..
	Connected!! IP=[192.168.1.165]  Gateway=[192.168.1.1] DNS=[212.230.135.2]
	Press 'R' to scan again.
	```
* __Ex5_ReadPartitionTable.cpp__
	- Prints the partition table present in the Esp32
	- Prints the total/free heap memory and the total/free PSRAM (if available)
	<img src="https://github.com/Vegethalia/Esp32Projects/blob/master/images/Ex5_PartitionInfoExample.png" alt="Example of output" width="600"/>
	
* __Ex6_SH1106Screen.cpp__
	- Basic example just to show how to use the U8G2 library with the SH1106 128x64 module screen
	<img src="https://github.com/Vegethalia/Esp32Projects/blob/master/images/Ex6_Schema.jpg" alt="Fast & Ugly Circuit Schema" width="300"/>
	<img src="https://github.com/Vegethalia/Esp32Projects/blob/master/images/Ex6_WorkingScreen.jpg" alt="SH1106 showing text" width="350"/>

