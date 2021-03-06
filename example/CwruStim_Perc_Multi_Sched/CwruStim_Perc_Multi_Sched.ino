/*
  CwruStim_beta.ino - Beta Code for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

#include <CwruStim.h>

Stim stim(STIM_CHANNEL_UART1);
// Stim stim(STIM_CHANNEL_UART0); //only use for USB-UART debugging.

//static int led_pin = 13; // for official Arduino boards
static int led_pin = 40; // for official N-TREK boards

// Pulse width and Amplitude value
uint8_t pulse_width[8] = {25,50,100,150,25,50,100,150};
uint8_t amplitude[8] = {38,38,38,38,38,38,38,38};
uint16_t ipi[8] = {60, 50, 60, 50, 60, 50, 60, 50};

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

	// stim.init(STIM_MODE_PERC); // Initialize the Stim board and delete old schedule
	// stim.config(STIM_MODE_PERC); // Setup channels, schedule, and events
	// //stim.start_multi_schedule();
	// stim.start(UECU_SYNC_MSG);

	stim.init(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Initialize the Stim board and delete old schedule
	stim.config(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Setup channels, schedule, and events
	//stim.start_multi_schedule();
	stim.start(UECU_SYNC_MSG);
	
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


	for (int i=0; i<8; i++) {	
		//stim.cmd_set_sched( sched_id, sync_signal, duration);
		stim.cmd_set_sched(i+1, UECU_SYNC_MSG, ipi[i]);
		delay(ipi[i]);
		// stim.cmd_set_evnt( event_id, pulse_width, amplitude, zone);
		stim.cmd_set_evnt(i+1, pulse_width[i], amplitude[i], 0); // Change Event i+1 for port_chn_id i in sched_id i+1
		delay(1); // TODO: ask Jeremy about the mini interval for set event.
	}



	//delay(50); // delay 50ms, not requirement

	digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW
	delay(900);              // wait for 100ms, , not requirement
	
}


