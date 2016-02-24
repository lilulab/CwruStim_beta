/*
  CwruStim_beta.ino - Beta Code for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

// #include "StimPattern/Gait_IST07.h" // IST-07's walking pattern
#include "CwruStim.h" // Stim board driver
#include <TimerOne.h>

// #define DEBUG_ON 1 //Comment this out if want to disenable all debug serial printing.

// Stim board 1 @ UART1
Stim stimBrd1(STIM_CHANNEL_UART1);

// Stim board 2 @ UART3
Stim stimBrd2(STIM_CHANNEL_UART3);

static int led_pin = 40; // for N-TREK board LEDs

bool led_state;

uint16_t cycle_percentage_value = 0;
float step_duration_ms = gait_walk_R_duration*1000;
float PP_step = (step_duration_ms/30);
uint16_t PP_increment = (uint16_t)(10000/PP_step);

void setup() {

  // // initialize Serial0 for debug.
  Serial.begin(115200);
  // #if defined(DEBUG_ON)
    Serial.println("Program Start ...");
  // #endif  
  // //Serial.println("Start CwruStim Program Setup");

  // LED pin
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);  // turn the LED on

  // delay(1000); // delay 2 sec

  // // Setup CwruStim Lib
  stimBrd1.init(STIM_MODE_ICM|IST); // Initialize the Stim board and delete old schedule
  stimBrd1.config(STIM_MODE_ICM|IST); // Setup channels, schedule, and events
  stimBrd1.start(UECU_SYNC_MSG);

  stimBrd2.init(STIM_MODE_SURF); // Initialize the Stim board and delete old schedule
  stimBrd2.config(STIM_MODE_SURF); // Setup channels, schedule, and events
  stimBrd2.start(UECU_SYNC_MSG);

  // //Serial.println((*LUT_B1_PP)[1][5]);

  Timer1.initialize(30000); // set a timer of length 1000 microseconds (or 0.001 sec - or 1kHz) 
  Timer1.attachInterrupt( timerOneIsr ); // attach the service routine here
  Timer1.stop();


}


void loop() {
  #if defined(DEBUG_ON)
    Serial.println("Start CwruStim Main Loop");
  #endif

  cycle_percentage_value = 0;

  // Change this to test different pattern
  int test_pattern = PATTERN_LSETP;

  // update IPI and AMP at the begining of gait
  stimBrd1.update(BRD1|IPI, test_pattern, cycle_percentage_value);
  stimBrd1.update(BRD1|AMP, test_pattern, cycle_percentage_value);
  stimBrd1.update(BRD1|PW, test_pattern, cycle_percentage_value);
  stimBrd1.start(UECU_SYNC_MSG); // After set IPI, board need a Sync msg to line up the pulses.
  delay(10);

  stimBrd2.update(BRD2|IPI, test_pattern, cycle_percentage_value);
  stimBrd2.update(BRD2|AMP, test_pattern, cycle_percentage_value);
  stimBrd1.update(BRD2|PW, test_pattern, cycle_percentage_value);
  stimBrd2.start(UECU_SYNC_MSG); // After set IPI, board need a Sync msg to line up the pulses.
  delay(10);

  // start timer to simulate gait 
  Timer1.resume();

  while (1) {
    // test PW ramping.
    digitalWrite(led_pin, LOW);  // turn the LED on
    stimBrd1.update(BRD1|PW, test_pattern, cycle_percentage_value);
    delay(10);
    digitalWrite(led_pin, HIGH);  // turn the LED on
    stimBrd2.update(BRD2|PW, test_pattern, cycle_percentage_value);
    delay(10);
  }


} //end loop()

void timerOneIsr()
{
  // 30ms timer ISR
  //digitalWrite( led_pin, digitalRead( led_pin ) ^ 1 );
  cycle_percentage_value += PP_increment;

  if (cycle_percentage_value > 20000) {
    cycle_percentage_value = 0;
  }
}

// void test_all_update(void) {
//   // Test all the board/pattern in IPI mode
//   // Stim::update(type, pattern, cycle_percentage)
//   stimBrd1.update(BRD1|IPI, PATTERN_NO_STIM, cycle_percentage_value);
//   stimBrd1.update(BRD1|IPI, PATTERN_STAND, cycle_percentage_value);
//   stimBrd1.update(BRD1|IPI, PATTERN_SIT, cycle_percentage_value);
//   stimBrd1.update(BRD1|IPI, PATTERN_LSETP, cycle_percentage_value);
//   stimBrd1.update(BRD1|IPI, PATTERN_RSETP, cycle_percentage_value);

//   stimBrd2.update(BRD2|IPI, PATTERN_NO_STIM, cycle_percentage_value);
//   stimBrd2.update(BRD2|IPI, PATTERN_STAND, cycle_percentage_value);
//   stimBrd2.update(BRD2|IPI, PATTERN_SIT, cycle_percentage_value);
//   stimBrd2.update(BRD2|IPI, PATTERN_LSETP, cycle_percentage_value);
//   stimBrd2.update(BRD2|IPI, PATTERN_RSETP, cycle_percentage_value);

//   // Test all the board/pattern in AMP mode
//   // Stim::update(type, pattern, cycle_percentage)
//   stimBrd1.update(BRD1|AMP, PATTERN_NO_STIM, cycle_percentage_value);
//   stimBrd1.update(BRD1|AMP, PATTERN_STAND, cycle_percentage_value);
//   stimBrd1.update(BRD1|AMP, PATTERN_SIT, cycle_percentage_value);
//   stimBrd1.update(BRD1|AMP, PATTERN_LSETP, cycle_percentage_value);
//   stimBrd1.update(BRD1|AMP, PATTERN_RSETP, cycle_percentage_value);

//   stimBrd2.update(BRD2|AMP, PATTERN_NO_STIM, cycle_percentage_value);
//   stimBrd2.update(BRD2|AMP, PATTERN_STAND, cycle_percentage_value);
//   stimBrd2.update(BRD2|AMP, PATTERN_SIT, cycle_percentage_value);
//   stimBrd2.update(BRD2|AMP, PATTERN_LSETP, cycle_percentage_value);
//   stimBrd2.update(BRD2|AMP, PATTERN_RSETP, cycle_percentage_value);
// }


