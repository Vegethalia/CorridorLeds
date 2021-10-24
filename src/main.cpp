/*
//THIS FILE CAN BE USED AS A TEMPLATE TO CREATE A NEW Ex_file.cpp!!!
#include <Arduino.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define PIN_LED 32
#define PIN_BUTTON 35

void setup()
{
 	Serial.begin(115200);
  // wait for serial monitor to open
  while(!Serial);

	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_BUTTON, INPUT);
}

void loop()
{
	Serial.print("Reading button... ");
	if(digitalRead(PIN_BUTTON)==HIGH) {
		Serial.println("HIGH");
		digitalWrite(PIN_LED, HIGH);
	}
	else {
		Serial.println("LOW");
		digitalWrite(PIN_LED, LOW);
	}
	delay(400);
}
*/