/*
  CwruStim_beta.ino - Beta Code for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

#include "CwruStim.h"

Stim stim(STIM_CHANNEL_UART1);

//static int led_pin = 13; // for official Arduino boards
static int led_pin = 40; // for official N-TREK boards

// Pulse width and Amplitude value
uint8_t pulse_width[8] = {0,0,0,0,0,0,0,0};
uint8_t amplitude[8] = {0,0,0,0,0,0,0,0};

void setup() {


	// initialize digital pin 13 as an output.
	Serial.begin(9600);
	//Serial.println("Start CwruStim Program Setup");
	pinMode(led_pin, OUTPUT);

	digitalWrite(led_pin, HIGH);	// turn the LED on

	delay(2000); // delay 2 sec

	/*
	// // Wait for Serial
	// while (!Serial) {
	// 	digitalWrite(led_pin, HIGH);    // turn the LED on by making the voltage HIGH  
	// 	delay(10);
	// 	digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW
	// 	delay(10);
	// }
  	*/

	// Setup CwruStim Lib
	stim.init(STIM_MODE_DEFAULT); // Initialize the Stim board and delete old schedule
	stim.config(STIM_SETTING_DEFAULT); // Setup channels, schedule, and events
	stim.start(UECU_SYNC_MSG); // Send start command (Sync message)

	// Stim Event update
	stim.update(STIM_COMMAND_ZERO_ALL); // Set pulse width and amplitude to 0 for all four channels. 
}

void loop() {
	//Serial.println("Start CwruStim Main Loop");
	digitalWrite(led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(100);              // wait for 100ms, , not requirement

	// Stim Event update
	// Change pulse width and amplitude on the fly

	// Do Something here to change the pw and amp value
	// pulse_width[x] = ???;
	// amplitude[x] = ???;

	// stim.cmd_set_evnt( event_id, pulse_width, amplitude, zone);
	stim.cmd_set_evnt( 1, pulse_width[0], amplitude[0], 0); // Change Event 1 for port_chn_id 0 in sched_id 1
	stim.cmd_set_evnt( 2, pulse_width[1], amplitude[1], 0);	// Change Event 2 for port_chn_id 1 in sched_id 1
	stim.cmd_set_evnt( 3, pulse_width[2], amplitude[2], 0);	// Change Event 3 for port_chn_id 2 in sched_id 1
	stim.cmd_set_evnt( 4, pulse_width[3], amplitude[3], 0); // Change Event 4 for port_chn_id 3 in sched_id 1

	//delay(50); // delay 50ms, not requirement

	digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW
	delay(100);              // wait for 100ms, , not requirement
	
}


