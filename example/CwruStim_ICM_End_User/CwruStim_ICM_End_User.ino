/*
  CwruStim_beta.ino - Beta Code for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

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
float step_duration_ms = VCK5_walk_L_duration*1000;
float PP_step = (step_duration_ms/30);
uint16_t PP_increment = (uint16_t)(10000/PP_step);

void setup() {

  // // initialize Serial0 for debug.
  // Serial.begin(115200);
  // #if defined(DEBUG_ON)
  //   Serial.print("Program Start ...");
  // #endif  
  // //Serial.println("Start CwruStim Program Setup");

  // // LED pin
  // pinMode(led_pin, OUTPUT);
  // digitalWrite(led_pin, HIGH);  // turn the LED on

  // delay(1000); // delay 2 sec

  // // Setup CwruStim Lib
  // stimBrd1.init(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Initialize the Stim board and delete old schedule
  // stimBrd1.config(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Setup channels, schedule, and events
  // stimBrd1.start(UECU_SYNC_MSG);

  // stimBrd2.init(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Initialize the Stim board and delete old schedule
  // stimBrd2.config(STIM_MODE_PERC_8CH_MULTI_SCHEDULE); // Setup channels, schedule, and events
  // stimBrd2.start(UECU_SYNC_MSG);

  // //Serial.println((*LUT_B1_PP)[1][5]);

  // Timer1.initialize(30000); // set a timer of length 1000 microseconds (or 0.001 sec - or 1kHz) 
  // Timer1.attachInterrupt( timerOneIsr ); // attach the service routine here
  // //Timer1.start();


}


void loop() {
  // #if defined(DEBUG_ON)
  //   Serial.println("Start CwruStim Main Loop");
  // #endif

  // stimBrd1.update(VCK5_BRD1|IPI, PATTERN_LSETP, cycle_percentage_value);
  // stimBrd1.update(VCK5_BRD1|AMP, PATTERN_LSETP, cycle_percentage_value);
  // stimBrd1.start(UECU_SYNC_MSG); // After set IPI, board need a Sync msg to line up the pulses.

  // stimBrd2.update(VCK5_BRD2|IPI, PATTERN_LSETP, cycle_percentage_value);
  // stimBrd2.update(VCK5_BRD2|AMP, PATTERN_LSETP, cycle_percentage_value);
  // stimBrd2.start(UECU_SYNC_MSG); // After set IPI, board need a Sync msg to line up the pulses.

  // cycle_percentage_value = 0;
  
  // while (1) {
  //   //digitalWrite(led_pin, !led_state);   // turn the LED on (HIGH is the voltage level)
  //   //delay(100);              // wait for 100ms, , not requirement


  //   // for debug only, set all board/param/pattern combo
  //   //test_all_update();

  //   stimBrd1.update(VCK5_BRD1|PW, PATTERN_LSETP, cycle_percentage_value);

  //   stimBrd2.update(VCK5_BRD2|PW, PATTERN_LSETP, cycle_percentage_value);

  //   #if defined(DEBUG_ON)
  //     //Serial.println(" ");
  //     Serial.print("cycle_PP = ");
  //     Serial.print(cycle_percentage_value);
  //     Serial.print("\t PP_step = ");
  //     Serial.print(PP_step);
  //     Serial.print("\t PP_increment = ");
  //     Serial.print(PP_increment);
  //     Serial.print("\t");

  //     // Print out Stim PW
  //     //stimBrd1.debug_print_states(1);
  //     //stimBrd2.debug_print_states(1);
  //   #endif

  // } // end while 1
}

// void timerOneIsr()
// {
//   // 30ms timer ISR
//   digitalWrite( led_pin, digitalRead( led_pin ) ^ 1 );
//   cycle_percentage_value += PP_increment;

//   if (cycle_percentage_value > 20000) {
//     cycle_percentage_value = 0;
//   }
// }

// void test_all_update(void) {
//   // Test all the board/pattern in IPI mode
//   // Stim::update(type, pattern, cycle_percentage)
//   stimBrd1.update(VCK5_BRD1|IPI, PATTERN_NO_STIM, cycle_percentage_value);
//   stimBrd1.update(VCK5_BRD1|IPI, PATTERN_STAND, cycle_percentage_value);
//   stimBrd1.update(VCK5_BRD1|IPI, PATTERN_SIT, cycle_percentage_value);
//   stimBrd1.update(VCK5_BRD1|IPI, PATTERN_LSETP, cycle_percentage_value);
//   stimBrd1.update(VCK5_BRD1|IPI, PATTERN_RSETP, cycle_percentage_value);

//   stimBrd2.update(VCK5_BRD2|IPI, PATTERN_NO_STIM, cycle_percentage_value);
//   stimBrd2.update(VCK5_BRD2|IPI, PATTERN_STAND, cycle_percentage_value);
//   stimBrd2.update(VCK5_BRD2|IPI, PATTERN_SIT, cycle_percentage_value);
//   stimBrd2.update(VCK5_BRD2|IPI, PATTERN_LSETP, cycle_percentage_value);
//   stimBrd2.update(VCK5_BRD2|IPI, PATTERN_RSETP, cycle_percentage_value);

//   // Test all the board/pattern in AMP mode
//   // Stim::update(type, pattern, cycle_percentage)
//   stimBrd1.update(VCK5_BRD1|AMP, PATTERN_NO_STIM, cycle_percentage_value);
//   stimBrd1.update(VCK5_BRD1|AMP, PATTERN_STAND, cycle_percentage_value);
//   stimBrd1.update(VCK5_BRD1|AMP, PATTERN_SIT, cycle_percentage_value);
//   stimBrd1.update(VCK5_BRD1|AMP, PATTERN_LSETP, cycle_percentage_value);
//   stimBrd1.update(VCK5_BRD1|AMP, PATTERN_RSETP, cycle_percentage_value);

//   stimBrd2.update(VCK5_BRD2|AMP, PATTERN_NO_STIM, cycle_percentage_value);
//   stimBrd2.update(VCK5_BRD2|AMP, PATTERN_STAND, cycle_percentage_value);
//   stimBrd2.update(VCK5_BRD2|AMP, PATTERN_SIT, cycle_percentage_value);
//   stimBrd2.update(VCK5_BRD2|AMP, PATTERN_LSETP, cycle_percentage_value);
//   stimBrd2.update(VCK5_BRD2|AMP, PATTERN_RSETP, cycle_percentage_value);
// }


