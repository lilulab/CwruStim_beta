#include "CwruStim.h"

Stim stim(STIM_CHANNEL_UART0);


void setup() {


	// initialize digital pin 13 as an output.
	Serial.begin(9600);
	//Serial.println("Start CwruStim Program Setup");
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	delay(2000);
	/*
	while (!Serial) {
		digitalWrite(13, HIGH);    // turn the LED off by making the voltage LOW  
		delay(10);
		digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
		delay(10);
	}
  	*/


  stim.init(STIM_MODE_DEFAULT);
  stim.config(STIM_SETTING_DEFAULT);
}

void loop() {
	//Serial.println("Start CwruStim Main Loop");
	digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(100);              // wait for a second

	stim.update(STIM_COMMAND_DEFAULT);

	digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
	delay(100);              // wait for a second
	
}


