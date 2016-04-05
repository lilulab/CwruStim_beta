/*
  CwruStim_beta.ino - Beta Code for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

#include "CwruStim.h" // Stim board driver
#include <TimerOne.h>

#define DEBUG_ON 1 //Comment this out if want to disenable all debug serial printing.

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

int current_pattern = PATTERN_NO_STIM;
int last_pattern = current_pattern;

void setup() {

  // initialize Serial0 for debug.
  Serial.begin(115200);
  #if defined(DEBUG_ON)
    Serial.print("Program Start ...");
  #endif  
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

  Timer1.initialize(30000); // set a timer of length 30 ms
  Timer1.attachInterrupt( timerOneIsr ); // attach the service routine here
  //Timer1.start();


}


void loop() {
  #if defined(DEBUG_ON)
    Serial.println("Start CwruStim Main Loop");
  #endif

  current_pattern = PATTERN_LSETP;

  stimBrd1.update(VCK5_BRD1|IPI, current_pattern, cycle_percentage_value);
  stimBrd1.update(VCK5_BRD1|AMP, current_pattern, cycle_percentage_value);
  stimBrd1.start(UECU_SYNC_MSG); // After set IPI, board need a Sync msg to line up the pulses.

  stimBrd2.update(VCK5_BRD2|IPI, current_pattern, cycle_percentage_value);
  stimBrd2.update(VCK5_BRD2|AMP, current_pattern, cycle_percentage_value);
  stimBrd2.start(UECU_SYNC_MSG); // After set IPI, board need a Sync msg to line up the pulses.

  cycle_percentage_value = 0;
  
  while (1) {
    //digitalWrite(led_pin, !led_state);   // turn the LED on (HIGH is the voltage level)
    //delay(100);              // wait for 100ms, , not requirement


    // for debug only, set all board/param/pattern combo
    //test_all_update();


    // check if switch pattern
    if (current_pattern!=last_pattern) {

      // print current pattern
      #if defined(DEBUG_ON)
        Serial.print("current pattern = ");
        Serial.print(current_pattern);
        Serial.print("(");
        Serial.print(last_pattern);
        Serial.print(")");
        // Serial.print("cycle_PP = ");
        // Serial.print(cycle_percentage_value);
        Serial.println(".");

      #endif

      // update new pattern IPI and AMP
      stimBrd1.update(VCK5_BRD1|IPI, current_pattern, cycle_percentage_value);
      stimBrd1.update(VCK5_BRD1|AMP, current_pattern, cycle_percentage_value);
      stimBrd1.start(UECU_SYNC_MSG); // After set IPI, board need a Sync msg to line up the pulses.

      stimBrd2.update(VCK5_BRD2|IPI, current_pattern, cycle_percentage_value);
      stimBrd2.update(VCK5_BRD2|AMP, current_pattern, cycle_percentage_value);
      stimBrd2.start(UECU_SYNC_MSG); // After set IPI, board need a Sync msg to line up the pulses.

      // after finish update IPI and AMP, update pattern history, to clean the flag
      last_pattern = current_pattern;

    } else {

      // still the same pattern, then only update PW

      // Dynamic pulse width control!!!
      demo_dynamic_pw_control();

      stimBrd1.update(VCK5_BRD1|PW, current_pattern, cycle_percentage_value);

      stimBrd2.update(VCK5_BRD2|PW, current_pattern, cycle_percentage_value);

      // #if defined(DEBUG_ON)
      //   //Serial.println(" ");
      //   Serial.print("cycle_PP = ");
      //   Serial.print(cycle_percentage_value);
      //   Serial.print("\t PP_step = ");
      //   Serial.print(PP_step);
      //   Serial.print("\t PP_increment = ");
      //   Serial.print(PP_increment);
      //   Serial.print("\t");

      //   // Print out Stim PW
      //   //stimBrd1.debug_print_states(1);
      //   //stimBrd2.debug_print_states(1);
      // #endif
    } // end if else

  } // end while 1
}

// Dynamic pulse width control!!!
void demo_dynamic_pw_control(void){

  // call this function right before update the pw.
  // this will set a gain factor for each channel's pw.
  // Output_PW = Original_PW * gain; 
  // (range of Output_PW limited by PW_MAX_PERC@CwruStimConst.h)
  
  // Stim.set_chan_pw_gain(uint8_t channel_id, float gain)
  // Board 1
  stimBrd1.set_chan_pw_gain(0,1.0);   // Channel 0, pw remain the same
  stimBrd1.set_chan_pw_gain(1,0.75);  // Channel 1, pw reduce to 3/4 value
  stimBrd1.set_chan_pw_gain(2,0.5);   // Channel 2, pw reduce to 1/2 value
  stimBrd1.set_chan_pw_gain(3,0.25);  // Channel 3, pw reduce to 1/4 value

  stimBrd1.set_chan_pw_gain(4,1.25);  // Channel 4, pw increase 1/4 value
  stimBrd1.set_chan_pw_gain(5,1.5);   // Channel 5, pw increase 1/2 value
  stimBrd1.set_chan_pw_gain(6,2.0);   // Channel 6, pw x2 times larger than original
  stimBrd1.set_chan_pw_gain(7,3.0);   // Channel 7, pw x3 times larger than original

  // Board 2
  stimBrd2.set_chan_pw_gain(0,1.0);   // Channel 0, pw remain the same
  stimBrd2.set_chan_pw_gain(1,0.75);  // Channel 1, pw reduce to 3/4 value
  stimBrd2.set_chan_pw_gain(2,0.5);   // Channel 2, pw reduce to 1/2 value
  stimBrd2.set_chan_pw_gain(3,0.25);  // Channel 3, pw reduce to 1/4 value

  stimBrd2.set_chan_pw_gain(4,1.25);  // Channel 4, pw increase 1/4 value
  stimBrd2.set_chan_pw_gain(5,1.5);   // Channel 5, pw increase 1/2 value
  stimBrd2.set_chan_pw_gain(6,2.0);   // Channel 6, pw x2 times larger than original
  stimBrd2.set_chan_pw_gain(7,3.0);   // Channel 7, pw x3 times larger than original
}

void timerOneIsr()
{
  // 30ms timer ISR
  digitalWrite( led_pin, digitalRead( led_pin ) ^ 1 );
  cycle_percentage_value += PP_increment;

  // check if past 3 sec
  if (cycle_percentage_value > 3000) {
    cycle_percentage_value = 0; //reset counter

    // update pattern history
    last_pattern = current_pattern;
    // increment pattern id
    current_pattern++;

    if (current_pattern >= NUM_PATTERN) {
      current_pattern = PATTERN_NO_STIM; //reset pattern id
    }
  }
}

void test_all_update(void) {
  // Test all the board/pattern in IPI mode
  // Stim::update(type, pattern, cycle_percentage)
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_NO_STIM, cycle_percentage_value);
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_STAND, cycle_percentage_value);
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_SIT, cycle_percentage_value);
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_LSETP, cycle_percentage_value);
  stimBrd1.update(VCK5_BRD1|IPI, PATTERN_RSETP, cycle_percentage_value);

  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_NO_STIM, cycle_percentage_value);
  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_STAND, cycle_percentage_value);
  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_SIT, cycle_percentage_value);
  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_LSETP, cycle_percentage_value);
  stimBrd2.update(VCK5_BRD2|IPI, PATTERN_RSETP, cycle_percentage_value);

  // Test all the board/pattern in AMP mode
  // Stim::update(type, pattern, cycle_percentage)
  // stimBrd1.update(VCK5_BRD1|AMP, PATTERN_NO_STIM, cycle_percentage_value);
  // stimBrd1.update(VCK5_BRD1|AMP, PATTERN_STAND, cycle_percentage_value);
  // stimBrd1.update(VCK5_BRD1|AMP, PATTERN_SIT, cycle_percentage_value);
  // stimBrd1.update(VCK5_BRD1|AMP, PATTERN_LSETP, cycle_percentage_value);
  // stimBrd1.update(VCK5_BRD1|AMP, PATTERN_RSETP, cycle_percentage_value);

  // stimBrd2.update(VCK5_BRD2|AMP, PATTERN_NO_STIM, cycle_percentage_value);
  // stimBrd2.update(VCK5_BRD2|AMP, PATTERN_STAND, cycle_percentage_value);
  // stimBrd2.update(VCK5_BRD2|AMP, PATTERN_SIT, cycle_percentage_value);
  // stimBrd2.update(VCK5_BRD2|AMP, PATTERN_LSETP, cycle_percentage_value);
  // stimBrd2.update(VCK5_BRD2|AMP, PATTERN_RSETP, cycle_percentage_value);
}


