/*
  CwruStim_beta.ino - Beta Code for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

#include "CwruStim.h" // Stim board driver
#include <TimerOne.h>

// Stim board 1 @ UART1
Stim stimBrd1(STIM_CHANNEL_UART1);

// Stim board 2 @ UART3
Stim stimBrd2(STIM_CHANNEL_UART3);

static int led_pin = 40; // for N-TREK board LEDs

uint16_t time_test = 0;
bool led_state;

void setup() {


  // initialize Serial0 for debug.
  Serial.begin(115200);
  Serial.print("Program Start ...");
  //Serial.println("Start CwruStim Program Setup");

  // LED pin
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);  // turn the LED on

  delay(1000); // delay 2 sec

  // Setup CwruStim Lib
  stimBrd1.init(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Initialize the Stim board and delete old schedule
  stimBrd1.config(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Setup channels, schedule, and events
  stimBrd1.start(UECU_SYNC_MSG);

  stimBrd2.init(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Initialize the Stim board and delete old schedule
  stimBrd2.config(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Setup channels, schedule, and events
  stimBrd2.start(UECU_SYNC_MSG);

  //Serial.println((*LUT_B1_PP)[1][5]);

  Timer1.initialize(30000); // set a timer of length 1000 microseconds (or 0.001 sec - or 1kHz) 
  Timer1.attachInterrupt( timerOneIsr ); // attach the service routine here
  //Timer1.start();


}


void loop() {

  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_LSETP, time_test);
  stimBrd1.update(VCK5_BRD1|AMP, PATTERN_LSETP, time_test);

  time_test = 0;
  
  while (1) {
	  //Serial.println("Start CwruStim Main Loop");
	  //digitalWrite(led_pin, !led_state);   // turn the LED on (HIGH is the voltage level)
	  //delay(100);              // wait for 100ms, , not requirement

	  Serial.println(" ");
	  Serial.print("cycle_percentage = ");
	  Serial.print(time_test);
	  Serial.print("\t");

	  // for debug only, set all board/param/pattern combo
	  //test_all_update();

	  // Test the board/pattern in PW mode

	  //stimBrd2.debug_print_states(2);

	  //digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW

	  //delay(50); // delay 50ms, not requirement
	  stimBrd1.update(VCK5_BRD1|PW, PATTERN_LSETP, time_test);
	  stimBrd1.debug_print_states(1);
	  //delay(100);              // wait for 100ms, , not requirement
  } // end while 1
}

void timerOneIsr()
{
	// 30ms timer ISR
	digitalWrite( led_pin, digitalRead( led_pin ) ^ 1 );
	time_test +=235;

	if (time_test > 20000) {
		time_test = 0;
	}
}

void test_all_update(void) {
  // Test all the board/pattern in IPI mode
  // Stim::update(type, pattern, cycle_percentage)
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_NO_STIM, time_test);
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_STAND, time_test);
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_SIT, time_test);
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_LSETP, time_test);
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_RSETP, time_test);

  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_NO_STIM, time_test);
  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_STAND, time_test);
  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_SIT, time_test);
  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_LSETP, time_test);
  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_RSETP, time_test);

  // Test all the board/pattern in AMP mode
  // Stim::update(type, pattern, cycle_percentage)
  stimBrd1.update(VCK5_BRD1|AMP, PATTERN_NO_STIM, time_test);
  stimBrd1.update(VCK5_BRD1|AMP, PATTERN_STAND, time_test);
  stimBrd1.update(VCK5_BRD1|AMP, PATTERN_SIT, time_test);
  stimBrd1.update(VCK5_BRD1|AMP, PATTERN_LSETP, time_test);
  stimBrd1.update(VCK5_BRD1|AMP, PATTERN_RSETP, time_test);

  stimBrd2.update(VCK5_BRD2|AMP, PATTERN_NO_STIM, time_test);
  stimBrd2.update(VCK5_BRD2|AMP, PATTERN_STAND, time_test);
  stimBrd2.update(VCK5_BRD2|AMP, PATTERN_SIT, time_test);
  stimBrd2.update(VCK5_BRD2|AMP, PATTERN_LSETP, time_test);
  stimBrd2.update(VCK5_BRD2|AMP, PATTERN_RSETP, time_test);
}


