/*
  CwruStim.cpp - Library for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

#include "Arduino.h"
#include "CwruStim.h"

// #define DEBUG_ON 1 //Comment this out if want to disenable all debug serial printing.
// #define DEBUG_STIM_UPDATE 1;
// #define DEBUG_STIM_RAMPING 1;

// Stim constructor and UART selector
Stim::Stim(int uart_channel_id) {
  // Initialize the NTREK ECB to connect the Stim Board
  // Check which UART channel is needed
  // Save to private var
  _uart_channel_id = uart_channel_id;
  
  //Serial.println("Enter Stim constructor");
  switch (_uart_channel_id) {

    case STIM_CHANNEL_UART0:
      //Use UART1(TX1&RX1) port to connect Stim Board
      Serial.begin(9600);
      while (!Serial) {_stim_error |= STIM_ERROR_SERIAL_ERROR;} //Set Error bit
      if (Serial) _stim_error &= ~STIM_NO_ERROR; //Clear Error bit
      break;

    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    //Code in here will only be compiled if an Arduino Mega is used.

      case STIM_CHANNEL_UART1:
        //Use UART1(TX1&RX1) port to connect Stim Board
        Serial1.begin(9600);
        while (!Serial1) {_stim_error |= STIM_ERROR_SERIAL_ERROR;} //Set Error bit
        if (Serial1) _stim_error &= ~STIM_NO_ERROR; //Clear Error bit
        break;

      case STIM_CHANNEL_UART3:
        //Use UART3(TX3&RX3) port to connect Stim Board
        Serial3.begin(9600);
        while (!Serial3) {_stim_error |= STIM_ERROR_SERIAL_ERROR;} //Set Error bit
        if (Serial3) {_stim_error &= ~STIM_NO_ERROR;} //Clear Error bit
        break;
    #endif

    default:
      _stim_error |= STIM_ERROR_SERIAL_ERROR;
      break;
    //Serial.print("_stim_error=");//Serial.print(_stim_error,BIN);//Serial.println(".");
    //Serial.println("exit Stim constructor");
  }

  // Pattern init
  for (uint8_t i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
    // Sync messages
    _PERC_8CH_SYNC_MSG[i] = 0xAA;

    // Inter phase interval
    // CHECKLIST: Need to change this later according to gait pattern file!
    _current_ipi[i] = 30;

    // Pulse width
    _current_pulse_width[i] = 0;

    // Amplitude
    _current_amplitude[i] = 0;
  }

}

// Initialize Stim Board via UART
int Stim::init(int mode) {

  //Serial.println("Enter Stim Initialization");
    // Commend Holder

  _mode = mode;
  //Serial.println("Exit Stim Initialization");

  delay(50); //wait UECU power up

  switch (mode) {
    case STIM_MODE_SURF:
      // Set message destination address
      MSG_DES_ADDR = MSG_DES_ADDR_SURF;
        // Reset Halt Message to reset Stim board
      this->cmd_halt_rset(UECU_RESET);

      // Delete Schedule
      this->cmd_del_sched(1); // Delete schedule 1
      break;
    case STIM_MODE_PERC:
      // Do nothing, since Perc board don't need to channel setup
      // Set message destination address
      MSG_DES_ADDR = MSG_DES_ADDR_PERC;
      break;

    case STIM_MODE_PERC_8CH_MULTI_SCHEDULE:   
      // same as Perc normal
      MSG_DES_ADDR = MSG_DES_ADDR_PERC;
      break;

    case STIM_MODE_DEFAULT:
      return -1;
      break;
    default:
      return -1;
      break;
    }

  return 1;
}

// Configure Stim Board via UART
int Stim::config(int setting) {

  // //Serial.println("Enter Stim Config");

  _setting = setting;
  // //Serial.println("Exit Stim Config");

  // Serial.print("_mode =  ");
  // Serial.println(_mode);

  // Serial.print("_setting =  ");
  // Serial.println(_setting);

  // For Surface board
  switch (setting) {

    case STIM_MODE_SURF:
      // Channels setup
      //cmd_chan_set(port_chn_id, amp_limit, pw_limit, ip_delay, asp_ratio, anode_cathode);
      
      // Bipolar 01
      this->cmd_chan_set( 0,    // port_chn_id =  0
                100,  // amp_limit = 100mA
                0xFA, // pw_limit = 250usec
                100,  // ip_delay = 100usec
                0x11,   // asp_ratio = 1:1
                0x01);  //anode = 0, cathode = 1, for bipolar mode);

      // Bipolar 23
      this->cmd_chan_set( 1,    // port_chn_id =  0
                100,  // amp_limit = 100mA
                0xFA, // pw_limit = 250usec
                100,  // ip_delay = 100usec
                0x11,   // asp_ratio = 1:1
                0x23);  //anode = 2, cathode = 3, for bipolar mode);

      // Bipolar 45
      this->cmd_chan_set( 2,    // port_chn_id =  0
                100,  // amp_limit = 100mA
                0xFA, // pw_limit = 250usec
                100,  // ip_delay = 100usec
                0x11,   // asp_ratio = 1:1
                0x45);  //anode = 0, cathode = 1, for bipolar mode);

      // Bipolar 67
      this->cmd_chan_set( 3,    // port_chn_id =  0
                100,  // amp_limit = 100mA
                0xFA, // pw_limit = 250usec
                100,  // ip_delay = 100usec
                0x11,   // asp_ratio = 1:1
                0x67);  //anode = 0, cathode = 1, for bipolar mode);



      // Create Schedule
      // this->cmd_crt_sched(sync_signal, duration);
      this->cmd_crt_sched(UECU_SYNC_MSG, 29); // Sync signal = 0xAA, duration 29msec.

      // Create Events
      // this->cmd_crt_evnt(sched_id, delay, priority, event_type, port_chn_id);

      // Create Event 1 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0,  // delay = 0msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                0,  // port_chn_id = 0;
                0,  // pulse_width set to 0,
                0,  // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 2 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0,  // delay = 0msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                1,  // port_chn_id = 1;
                0,  // pulse_width set to 0,
                0,  // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 3 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0,  // delay = 0msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                2,  // port_chn_id = 2;
                0,  // pulse_width set to 0,
                0,  // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 4 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0,  // delay = 0msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                3,  // port_chn_id = 3;
                0,  // pulse_width set to 0,
                0,  // amplitude set to 0,
                0); // zone not implemented;
      return 1;
      break;

    // For Perc board
    case STIM_MODE_PERC:

        // Serial.print("EXE STIM_MODE_PERC");
      // Create Schedule ------------------------------
      // crt_sched  : 01 80 10 03 AA 00 1D A3 

      // this->cmd_crt_sched(sync_signal, duration);
      this->cmd_crt_sched(UECU_SYNC_MSG, 29); // Sync signal = 0xAA, duration 29msec (0d29 = 0x1D).

      // Create Event 1-12  ----------------------------------
      // crt_evnt1  : 01 80 15 09 01 00 00 00 03 00 50 10 00 FB
      // crt_evnt2  : 01 80 15 09 01 00 05 00 03 01 80 15 00 C0
      // ...
      // crt_evnt12 : 01 80 15 09 01 00 0F 00 03 0B F0 26 00 2B

      // this->cmd_crt_evnt(sched_id, delay, priority, event_type, port_chn_id);
      // Create Event 1 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0,  // delay = 0msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                0,  // port_chn_id = 0;
                0x50, // pulse_width set to 0,
                0x10, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 2 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x05, // delay = 5msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                1,  // port_chn_id = 1;
                0x80, // pulse_width set to 0,
                0x15, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 3 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x0A, // delay = 10msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                2,  // port_chn_id = 2;
                0xB0, // pulse_width set to 0,
                0x20, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 4 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x0F, // delay = 15msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                3,  // port_chn_id = 3;
                0xF0, // pulse_width set to 0,
                0x26, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 5 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0,  // delay = 0msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                4,  // port_chn_id = 4;
                0x50, // pulse_width set to 0,
                0x10, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 6 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x05, // delay = 5msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                5,  // port_chn_id = 5;
                0x80, // pulse_width set to 0,
                0x15, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 7 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x0A, // delay = 10msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                6,  // port_chn_id = 6;
                0xB0, // pulse_width set to 0,
                0x20, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 8 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x0F, // delay = 15msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                7,  // port_chn_id = 7;
                0xF0, // pulse_width set to 0,
                0x26, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 9 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0,  // delay = 0msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                8,  // port_chn_id = 8;
                0x50, // pulse_width set to 0,
                0x10, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 10 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x05, // delay = 5msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                9,  // port_chn_id = 9;
                0x80, // pulse_width set to 0,
                0x15, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 11 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x0A, // delay = 10msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                10, // port_chn_id = 10;
                0xB0, // pulse_width set to 0,
                0x20, // amplitude set to 0,
                0); // zone not implemented;

      // Create Event 12 for port_chn_id 0 in sched_id 1 
      this->cmd_crt_evnt( 
                1,  // sched_id = 1
                0x0F, // delay = 15msec
                0,  // priority = 0
                3,  // event_type = 3, for for Stimulus Event
                11, // port_chn_id = 11;
                0xF0, // pulse_width set to 0,
                0x26, // amplitude set to 0,
                0); // zone not implemented;

      // Create Sync Message ----------------------------------
      // sync_msg1  : 01 80 1B 01 AA B7
      return 1;
      break;

    // multi scheduler for percutaneous stimulation board
    case STIM_MODE_PERC_8CH_MULTI_SCHEDULE:

        // Serial.print("EXE STIM_MODE_PERC_8CH_MULTI_SCHEDULE");

      // Create Schedule ------------------------------
      // crt_sched  : 01 80 10 03 AA 00 1D A3 

      // this->cmd_crt_sched(sync_signal, duration);
      // Create 8 schedules
      for (uint8_t i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
        this->cmd_crt_sched(UECU_SYNC_MSG, _current_ipi[i]);  // Sync signal, duration 30msec.
        delay(UECU_DELAY_SETUP);
      }

      for (uint8_t i=0; i<STIM_CHANNEL_MAX_PERC; i++) {

        // QUESTION: setup 8 schedules than 8 events, or do it like below?

        // Create schedule
        // CHECKLIST: Need to set IPI array first!
        //this->cmd_crt_sched(_PERC_8CH_SYNC_MSG[i], _current_ipi[i]);  // Sync signal, duration 30msec.


        // Create event 
        // this->cmd_crt_evnt(sched_id, delay, priority, event_type, port_chn_id);
        // Create Event 1 for port_chn_id 0 in sched_id 1 
        this->cmd_crt_evnt( 
                    i+1,  // sched_id 1 to 8
                  (i+1)*2,  // delay every 2ms. (2,4,6, ...)
                  0,  // priority = 0
                  3,  // event_type = 3, for for Stimulus Event
                  i,  // port_chn_id = 0;
                  // 0x00,  // pulse_width set to 0,
                  0,  // pulse_width set to 0,
                  0x26, // amplitude set to 0x26,
                  0); // zone not implemented;

        // Serial.print("In config loop, ");
        // Serial.print("i = ");
        // Serial.print(i,HEX);
        // Serial.print(";\t IPI = ");
        // Serial.println(_current_ipi[i],HEX);
        // Serial.println(" ");
        delay(UECU_DELAY_SETUP);

      } // end for loop

      delay(UECU_DELAY_SETUP);

      // TODO: Send 8 Sync msgs here or modify start func.

      break;

    default: 
      return -1;
    } //end switch case
}

// Start Stim board using sync signal command
int Stim::start(uint8_t sync_signal) {

  // Send Sync to start
  //this->cmd_sync_msg(0xAA); // Sent Sync_message 0xAA to start schedule.
  this->cmd_sync_msg(sync_signal); // Sent Sync_message to start schedule.
}

// Start multiple scheduler
int Stim::start_multi_schedule(void) {

  // Loop through 8 schedules
  for (int i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
    // Send Sync to start
    this->cmd_sync_msg(_PERC_8CH_SYNC_MSG[i]); // Sent Sync_message to start schedule.

    // Delay duration need to be save as IPI.
    delay(_current_ipi[i]);
  }

}

// Update Stim pattern via UART
// Stim::update(type, pattern, cycle_percentage)
int Stim::update(int type, int pattern, uint16_t cycle_percentage) {

  // Pattern look up table temp holder.
  const uint16_t (*LUT_PP)[12][8]; 
  const uint8_t (*LUT_PW)[12][8]; 
  const uint8_t (*LUT_IPI)[12];  
  const uint8_t (*LUT_AMP)[12];  


  #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
    Serial.println(" ");
    Serial.println(" ");
    Serial.print("_uart_channel_id = ");
    Serial.print(_uart_channel_id);
    Serial.print("\ttype = 0x");
    Serial.print(type,HEX);  
    Serial.print("\tpattern = ");
    Serial.print(pattern);
    Serial.print("\tcycle_percentage = ");
    Serial.print(cycle_percentage);
    Serial.println(". ");
    
    //   //Test the pattern data.
    //   // print test pattern
    //   LUT_PP = &VCK5_walk_L_B1_PP; 
    //   LUT_PW = &VCK5_walk_L_B1_PW; 
    //   LUT_IPI = &VCK5_walk_L_B1_IPI;
    //   LUT_AMP = &VCK5_B1_AMP;

    //   // print PP
    //   for (int i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
    //     Serial.print("(*LUT_PP)[");
    //     Serial.print(i);
    //     Serial.print("][j] = {");
    //     for (int j=0; j<GAIT_LUT_RES; j++) {
    //       Serial.print((*LUT_PP)[i][j]);
    //       Serial.print(",\t");
    //     }
    //     Serial.println(" ");  
    //   }
    //   Serial.println(" ");

    //   // print PW
    //   for (int i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
    //     Serial.print("(*LUT_PW)[");
    //     Serial.print(i);
    //     Serial.print("][j] = {");
    //     for (int j=0; j<GAIT_LUT_RES; j++) {
    //       Serial.print((*LUT_PW)[i][j]);
    //       Serial.print(",\t");
    //     }
    //     Serial.println(" ");  
    //   }
    //   Serial.println(" ");

    //   // print IPI
    //   Serial.println("VCK5_B1_IPI array = ");
    //   Serial.print("{");  
    //   for (int i=0; i<12; i++) {
    //       Serial.print((*LUT_IPI)[i]);
    //       Serial.print(",");
    //   }
    //   Serial.println("}");

    //   // print AMP
    //   Serial.println("VCK5_B1_AMP array = ");
    //   Serial.print("{");  
    //   for (int i=0; i<12; i++) {
    //       Serial.print((*LUT_AMP)[i]);
    //       Serial.print(",");
    //   }
    //   Serial.println("}");
  #endif

  // type mask
  int board = type & 0xFF00;
  int param = type & 0x00FF;

  int need_update = 0;

  #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
    Serial.print("board = 0x");
    Serial.print(board,HEX);
    Serial.print("\t param = 0x");
    Serial.println(param,HEX);    
  #endif

  // Select update type
  switch (param) {
    // if need to update IPI
    case IPI:
      #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
        Serial.print("IPI MODE: ");
      #endif
      // Select which pattern
      need_update = 1;
      switch (pattern) {
        case PATTERN_NO_STIM:
          need_update = 0;
          break;

        // IPI - Stand
        case PATTERN_STAND:
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print("-- IPI - Stand ");
          #endif
          if (board == VCK5_BRD1) {
            // #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            //   Serial.print("-- -- VCK5_BRD1 ");
            // #endif
            LUT_IPI = &VCK5_stand_B1_IPI;
          } else if (board == VCK5_BRD2) {
            // #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            //   Serial.print("-- -- VCK5_BRD2 ");
            // #endif
            LUT_IPI = &VCK5_stand_B2_IPI;
          } else {
            _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
          }
          break;

        // IPI - SIT
        case PATTERN_SIT:
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print("-- IPI - SIT ");
          #endif
          if (board == VCK5_BRD1) {
            LUT_IPI = &VCK5_sit_B1_IPI;
          } else if (board == VCK5_BRD2) {
            LUT_IPI = &VCK5_sit_B2_IPI;
          } else {
            _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
          }
          break;

        // IPI - LSTEP
        case PATTERN_LSETP:
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print("-- IPI - LSTEP ");
          #endif
          if (board == VCK5_BRD1) {
            LUT_IPI = &VCK5_walk_L_B1_IPI;
          } else if (board == VCK5_BRD2) {
            LUT_IPI = &VCK5_walk_L_B2_IPI;
          } else {
            _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
          }
          break;

        // IPI - RSTEP
        case PATTERN_RSETP:
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print("-- IPI - RSTEP ");
          #endif
          if (board == VCK5_BRD1) {
            LUT_IPI = &VCK5_walk_R_B1_IPI;
          } else if (board == VCK5_BRD2) {
            LUT_IPI = &VCK5_walk_R_B2_IPI;
          } else {
            _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
          }
          break;
      } // end switch (pattern)

      // Now update IPI 
      if (need_update == 1) {

        #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
          Serial.print("IPI updating.\t");
        #endif

        for (int i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
          _current_ipi[i] = (*LUT_IPI)[i];
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print(_current_ipi[i]);
            Serial.print(",\t");
          #endif
          this->cmd_set_sched(i+1, UECU_SYNC_MSG, _current_ipi[i]);
          //delay(_current_ipi[i]); //Do not need this delay
        } // end for
      } // end if

      break; // end case IPI

    // if need to update AMP
    case AMP:
      #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
        Serial.print("AMP MODE: ");
      #endif

      if (board == VCK5_BRD1) {
        LUT_AMP = &VCK5_B1_AMP;
      } else if (board == VCK5_BRD2) {
        LUT_AMP = &VCK5_B2_AMP;
      } else {
        _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
      }        

      // Now update AMP 
      for (int i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
        if (pattern == PATTERN_NO_STIM) {
          _current_amplitude[i] = 0;
        } else {
          _current_amplitude[i] = (*LUT_AMP)[i];
        }
        #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
          Serial.print(_current_amplitude[i]);
          Serial.print(",\t");
        #endif

        // do not set event here, since need to set after PW
        //this->cmd_set_evnt(i+1, _current_pulse_width[i], _current_amplitude[i], 0); // Change Event i for port_chn_id i in sched_id 1  
      } // end for   
      break; // end case AMP

    // if need to update PW
    case PW:
      #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
        Serial.print("PW MODE: ");
      #endif
      // Select which pattern
      need_update = 1;
      switch (pattern) {
        case PATTERN_NO_STIM:
          // do nothing here, set all 0 at the cmd_set_evnt in the end
          break;

        // PW - Stand
        case PATTERN_STAND:
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print("-- PW - Stand ");
          #endif
          if (board == VCK5_BRD1) {
            LUT_PP = &VCK5_stand_B1_PP;
            LUT_PW = &VCK5_stand_B1_PW;
          } else if (board == VCK5_BRD2) {
            LUT_PP = &VCK5_stand_B2_PP;
            LUT_PW = &VCK5_stand_B2_PW;
          } else {
            _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
          }
          break;

        // PW - SIT
        case PATTERN_SIT:
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print("-- PW - SIT ");
          #endif
          if (board == VCK5_BRD1) {
            LUT_PP = &VCK5_sit_B1_PP;
            LUT_PW = &VCK5_sit_B1_PW;
          } else if (board == VCK5_BRD2) {
            LUT_PP = &VCK5_sit_B2_PP;
            LUT_PW = &VCK5_sit_B2_PW;
          } else {
            _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
          }
          break;

        // PW - LSTEP
        case PATTERN_LSETP:
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print("-- PW - LSTEP ");
          #endif
          if (board == VCK5_BRD1) {
            LUT_PP = &VCK5_walk_L_B1_PP;
            LUT_PW = &VCK5_walk_L_B1_PW;
          } else if (board == VCK5_BRD2) {
            LUT_PP = &VCK5_walk_L_B2_PP;
            LUT_PW = &VCK5_walk_L_B2_PW;
          } else {
            _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
          }
          break;

        // PW - RSTEP
        case PATTERN_RSETP:
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print("-- PW - RSTEP ");
          #endif
          if (board == VCK5_BRD1) {
            LUT_PP = &VCK5_walk_R_B1_PP;
            LUT_PW = &VCK5_walk_R_B1_PW;
          } else if (board == VCK5_BRD2) {
            LUT_PP = &VCK5_walk_R_B2_PP;
            LUT_PW = &VCK5_walk_R_B2_PW;
          } else {
            _stim_error |= STIM_ERROR_UPDATE_PATTERN_ERROR;
          }
          break;
      } // end switch (pattern)

      // Now update PW 
      if (need_update == 1) {
        for (int i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
          // save the previous PW value
          uint8_t _last_pulse_width = _current_pulse_width[i];

          if (pattern == PATTERN_NO_STIM) {
            _current_pulse_width[i] = 0;
          } else {
            // ramping function
            _current_pulse_width[i] = get_PW_ramping(i, LUT_PP, LUT_PW, cycle_percentage);
            //_current_pulse_width[i] = (*LUT_PW)[i][STIM_CHANNEL_MAX_PERC-1];
          }
          #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
            Serial.print(_current_pulse_width[i]);
            Serial.print(",\t");
          #endif
          // publish if only it is diff than the previous value
          // if ((_last_pulse_width != _current_pulse_width[i]) || (cycle_percentage == 0)) {
          if (_last_pulse_width != _current_pulse_width[i]) {
            this->cmd_set_evnt(i+1, _current_pulse_width[i], _current_amplitude[i], 0); // Change Event i for port_chn_id i in sched_id 1  
          }
        } // end for
      } // end if

      break; // end case PW

    // Error 
    default: 
      _stim_error |= STIM_ERROR_UPDATE_TYPE_ERROR;
      return -1;
      break;
  } // end switch (type)

  #if defined(DEBUG_STIM_UPDATE) && defined(DEBUG_ON)
    Serial.print("_stim_error = ");
    Serial.println(_stim_error);
  #endif
  

  return 1;
}

uint8_t Stim::get_PW_ramping( int channel_i,
                        const uint16_t (*LUT_PP_t)[12][8],
                        const uint8_t (*LUT_PW_t)[12][8],
                        uint16_t cycle_pp_t) 
{
  // search in LUT_PP_t[i][:] find where cycle_pp_t belongs to
  // find the upper limit of ramp

  // calulated ramping value, set to 0 for safety
  uint8_t pw_ramping_val = 0;

  // search until found the correct ramping value
  int found_ramp = 0;
  
  // prevent pp greater than 100%
  if (cycle_pp_t>10000) {
    cycle_pp_t = 10000;
  }

  #if defined(DEBUG_STIM_RAMPING) && defined(DEBUG_ON)
    Serial.print("get_PW_ramping(): ");
    Serial.print("channel_i = ");
    Serial.print(channel_i);
    Serial.print("\t cycle_pp_t = ");
    Serial.println(cycle_pp_t);

    Serial.println("(*LUT_PP_t)[12][8] = {");
    for (int i=0; i < STIM_CHANNEL_MAX_PERC; i++) {
      for (int j=0; j < GAIT_LUT_RES; j++) {
        Serial.print((*LUT_PP_t)[i][j]);
        Serial.print(", \t");
      }
      Serial.println("}");
    }

    Serial.println(" ");

    Serial.println("(*LUT_PW_t)[12][8] = {");
    for (int i=0; i < STIM_CHANNEL_MAX_PERC; i++) {
      for (int j=0; j < GAIT_LUT_RES; j++) {
        Serial.print((*LUT_PW_t)[i][j]);
        Serial.print(", \t");
      }
      Serial.println("}");
    }

  #endif
  
  int j=0;
  while (found_ramp == 0) {
    // LUT resolution step iterator

    #if defined(DEBUG_STIM_RAMPING) && defined(DEBUG_ON)
      Serial.print("found_ramp = ");
      Serial.print(found_ramp);
      Serial.print(".\t j = ");
      Serial.println(j);
    #endif

    // only search 0 to (RES-1)
    if (j < GAIT_LUT_RES) {

      #if defined(DEBUG_STIM_RAMPING) && defined(DEBUG_ON)
          Serial.print("Search loop j: ");
      #endif

      //(*LUT_PP_t)[channel_i][j]
      if (cycle_pp_t == (*LUT_PP_t)[channel_i][j]) {
        // if equal, then use PP to directly find PW value, no need for ramping
        // direct return the ramped PW value;
        pw_ramping_val =  (*LUT_PW_t)[channel_i][j];

        // and exit search loop
        found_ramp = 1;

        #if defined(DEBUG_STIM_RAMPING) && defined(DEBUG_ON)
          Serial.print("cycle_pp_t = (*LUT_PP_t)[channel_i][j]");
          Serial.print(", pw_ramping_val = ");
          Serial.println(pw_ramping_val);
        #endif

      } else if (cycle_pp_t < (*LUT_PP_t)[channel_i][j]) {

        // if cycle_pp_t is smaller then this LUT_PP_t
        // then cycle_pp_t belongs to this slot
        // that means the previous LUT_PP_t is the correct lower limit
        // and do calculation in here, then break the search loop

        // Calculate the ramping value
        //                              time_diff 
        // pw_ramping_val = pw_diff * (-----------) + PW_low
        //                              time_div  
        // then,
        //                                     PP_now - PP_low 
        // pw_ramping_val = (PW_up-PW_low) * (-----------------) + PW_low
        //                                     PP_up - PP_low  
        uint16_t time_diff = cycle_pp_t - (*LUT_PP_t)[channel_i][j-1];
        uint16_t time_base = (*LUT_PP_t)[channel_i][j] - (*LUT_PP_t)[channel_i][j-1];
        float time_div = (float)time_diff / (float)time_base;
        float pw_diff = (float)((*LUT_PW_t)[channel_i][j] - (*LUT_PW_t)[channel_i][j-1]);
        uint8_t pw_base;
        if (j==0) {
          pw_base = (*LUT_PW_t)[channel_i][j];
        } else {
          pw_base = (*LUT_PW_t)[channel_i][j-1];
        }
        pw_ramping_val = (uint8_t) round(pw_diff * time_div) + pw_base;

        // and exit search loop
        found_ramp = 1;

        #if defined(DEBUG_STIM_RAMPING) && defined(DEBUG_ON)
          Serial.print("cycle_pp_t > (*LUT_PP_t)[channel_i][j]");
          Serial.print(", pw_ramping_val = ");
          Serial.println(pw_ramping_val);
          Serial.print("time_diff = ");
          Serial.print(time_diff);
          Serial.print("time_base = ");
          Serial.print(time_base);
          Serial.print("time_div = ");
          Serial.print(time_div);
          Serial.print("pw_diff = ");
          Serial.println(pw_diff);
        #endif

      } else {
        // this means either cycle_pp_t should belong to this slot,
        // or we still need to keep serching.
        // so keep increment j, or keep the search loop
        found_ramp = 0;
        j++;

        #if defined(DEBUG_STIM_RAMPING) && defined(DEBUG_ON)
          Serial.print("Else");
          Serial.print(", pw_ramping_val = ");
          Serial.println(pw_ramping_val);
        #endif
      } // end if else ..
    }// end if j<
  } // end while (found_ramp)

  #if defined(DEBUG_STIM_RAMPING) && defined(DEBUG_ON)
          Serial.print("Found Ramp!");
          Serial.print("j = ");
          Serial.println(j);
          Serial.print(", pw_ramping_val = ");
          Serial.println(pw_ramping_val);
  #endif
  // return the ramped PW value;
  return pw_ramping_val;

}

//int _Gait_LUT_VCK5[2][4];
int Stim::gait_LUT_builder(void) {

  // Board 1
  // STAND
  // _Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_STAND][PATTERN_PARAM_PP] = (uint16_t) &VCK5_stand_B1_PP;
  // _Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_STAND][PATTERN_PARAM_PW] = &VCK5_stand_B1_PW;
  // _Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_STAND][PATTERN_PARAM_AMP] = &VCK5_B1_AMP; //Same cross board
  // _Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_STAND][PATTERN_PARAM_IPI] = &VCK5_stand_B1_IPI;

 //  // SIT
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_SIT][PATTERN_PARAM_PP] = &VCK5_sit_B1_PP;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_SIT][PATTERN_PARAM_PW] = &VCK5_sit_B1_PW;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_SIT][PATTERN_PARAM_AMP] = &VCK5_B1_AMP; //Same cross board
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_SIT][PATTERN_PARAM_IPI] = &VCK5_sit_B1_IPI;  

  // // LSTEP
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_LSETP][PATTERN_PARAM_PP] = &VCK5_walk_L_B1_PP;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_LSETP][PATTERN_PARAM_PW] = &VCK5_walk_L_B1_PW;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_LSETP][PATTERN_PARAM_AMP] = &VCK5_B1_AMP; //Same cross board
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_LSETP][PATTERN_PARAM_IPI] = &VCK5_walk_L_B1_IPI;

  // // RSTEP
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_RSETP][PATTERN_PARAM_PP] = &VCK5_walk_R_B1_PP;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_RSETP][PATTERN_PARAM_PW] = &VCK5_walk_R_B1_PW;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_RSETP][PATTERN_PARAM_AMP] = &VCK5_B1_AMP; //Same cross board
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD1][PATTERN_RSETP][PATTERN_PARAM_IPI] = &VCK5_walk_R_B1_IPI;

 //  // Board 2
 //  // STAND
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_STAND][PATTERN_PARAM_PP] = &VCK5_stand_B2_PP;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_STAND][PATTERN_PARAM_PW] = &VCK5_stand_B2_PW;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_STAND][PATTERN_PARAM_AMP] = &VCK5_B2_AMP; //Same cross board
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_STAND][PATTERN_PARAM_IPI] = &VCK5_stand_B2_IPI;

 //  // SIT
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_SIT][PATTERN_PARAM_PP] = &VCK5_sit_B2_PP;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_SIT][PATTERN_PARAM_PW] = &VCK5_sit_B2_PW;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_SIT][PATTERN_PARAM_AMP] = &VCK5_B2_AMP; //Same cross board
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_SIT][PATTERN_PARAM_IPI] = &VCK5_sit_B2_IPI;  

  // // LSTEP
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_LSETP][PATTERN_PARAM_PP] = &VCK5_walk_L_B2_PP;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_LSETP][PATTERN_PARAM_PW] = &VCK5_walk_L_B2_PW;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_LSETP][PATTERN_PARAM_AMP] = &VCK5_B2_AMP; //Same cross board
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_LSETP][PATTERN_PARAM_IPI] = &VCK5_walk_L_B2_IPI;

  // // RSTEP
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_RSETP][PATTERN_PARAM_PP] = &VCK5_walk_R_B2_PP;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_RSETP][PATTERN_PARAM_PW] = &VCK5_walk_R_B2_PW;
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_RSETP][PATTERN_PARAM_AMP] = &VCK5_B2_AMP; //Same cross board
  // *_Gait_LUT_VCK5[GAIT_VCK5_BOARD2][PATTERN_RSETP][PATTERN_PARAM_IPI] = &VCK5_walk_R_B2_IPI;

}

// UART write array
int Stim::serial_write_array(uint8_t buf[], int length) {
  
  // for(int i = 0; i<length; i++){
  //  Serial.write(buf[i]);
  //  //Serial.print(buf[i],HEX);
  //  //Serial.print(" ");
  // }

  // Select UART Channel
  switch (_uart_channel_id) {

    case STIM_CHANNEL_UART0:
      for(int i = 0; i<length; i++){
        Serial.write(buf[i]);
      }
      break;

    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    //Code in here will only be compiled if an Arduino Mega is used.
      case STIM_CHANNEL_UART1:
        for(int i = 0; i<length; i++){
          Serial1.write(buf[i]);
        }
        break;

      case STIM_CHANNEL_UART3:
        for(int i = 0; i<length; i++){
          Serial3.write(buf[i]);
        }
        break;
    #endif

    default:
      _stim_error |= STIM_ERROR_SERIAL_ERROR;
      break;
  }

  //delay(50);
  return Serial;
}

// Use Serial0 to print debug messages.
int Stim::debug_print_states(int id) {
  if (Serial) {

    Serial.println(" ");

    Serial.print("stimBrd"); Serial.print(id); Serial.print(".");
    Serial.print("_uart_channel_id = ");
    Serial.print(_uart_channel_id);
    Serial.print(".\t _stim_error = ");
    Serial.print(_stim_error);
    Serial.println(".");

    Serial.print("_current_pulse_width = {");
    for (uint8_t i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
      Serial.print(_current_pulse_width[i]);
      Serial.print(",\t");
    }
    Serial.println("}.");

    Serial.print("_current_amplitude = {");
    for (uint8_t i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
      Serial.print(_current_amplitude[i]);
      Serial.print(",\t");
    }
    Serial.println("}.");

    Serial.print("_current_ipi = {");
    for (uint8_t i=0; i<STIM_CHANNEL_MAX_PERC; i++) {
      Serial.print(_current_ipi[i]);
      Serial.print(",\t");
    }
    Serial.println("}.");

  }
}


// Retun check sum byte
uint8_t Stim::checksum(uint8_t vals[], int length){
  uint16_t csum = 0;
  for(int i=0; i<length-1; i++) {
    csum += (uint16_t)vals[i];
  }
  csum = ((0x00FF & csum) + (csum >> 8))^0xFF;
  return csum;
} 

// uint8_t Stim::_chan_seti[12] = {0x04, 0x80, 0x47, 0x07, 0x00, 0x64, 0xFA, 0x00, 0x64, 0x11, 0x01, 0x57};
// UECU command sets
//UECU Halt
int Stim::cmd_halt_rset(uint8_t halt_flag) {
  // calculate message size
  int size = HALT_MSG_LEN + UECU_MSG_EXTRAL_LEN;
  // build message content
  uint8_t msg[HALT_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
  {   MSG_DES_ADDR,
    MSG_SRC_ADDR,
    HALT_MSG,
    HALT_MSG_LEN,
    halt_flag,
    0x00
  };

  // Insert checksum byte
  msg[size-1] = this->checksum(msg,size);

  // Send message
  return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));

}

// UECU Delete Schedule
int Stim::cmd_del_sched(uint8_t sched_id) {
  // calculate message size
  int size = DELETE_SCHEDULE_MSG_LEN + UECU_MSG_EXTRAL_LEN;
  // build message content
  uint8_t msg[DELETE_SCHEDULE_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
  {   MSG_DES_ADDR,
    MSG_SRC_ADDR,
    DELETE_SCHEDULE_MSG,
    DELETE_SCHEDULE_MSG_LEN,
    sched_id,
    0x00
  };

  // Insert checksum byte
  msg[size-1] = this->checksum(msg,size);

  // Send message
  return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));
}

// UECU Create Schedule
int Stim::cmd_crt_sched(uint8_t sync_signal, uint16_t duration) {
  // calculate message size
  int size = CREATE_SCHEDULE_MSG_LEN + UECU_MSG_EXTRAL_LEN;
  // build message content
  uint8_t msg[CREATE_SCHEDULE_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
  {   MSG_DES_ADDR,
    MSG_SRC_ADDR,
    CREATE_SCHEDULE_MSG,
    CREATE_SCHEDULE_MSG_LEN,
    sync_signal,
    (uint8_t)((duration >> 8) & 0x00FF),
    (uint8_t)(duration & 0x00FF),
    0x00
  };

  // Insert checksum byte
  msg[size-1] = this->checksum(msg,size);

  // Send message
  return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));
}

// UECU Channel Setup 
int Stim::cmd_chan_set( uint8_t port_chn_id, 
                        uint8_t amp_limit,
                        uint8_t pw_limit,
                        uint16_t ip_delay, 
                        uint8_t asp_ratio, 
                        uint8_t anode_cathode) {
  // calculate message size
  int size = CHANNEL_SETUP_MSG_LEN + UECU_MSG_EXTRAL_LEN;
  // build message content
  uint8_t msg[CHANNEL_SETUP_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
  {   MSG_DES_ADDR,
    MSG_SRC_ADDR,
    CHANNEL_SETUP_MSG,
    CHANNEL_SETUP_MSG_LEN,
    port_chn_id,
    amp_limit,
    pw_limit,
    (uint8_t)((ip_delay >> 8) & 0x00FF),
    (uint8_t)(ip_delay & 0x00FF),
    asp_ratio,
    anode_cathode,
    0x00
  };

  // Insert checksum byte
  msg[size-1] = this->checksum(msg,size);

  // Send message
  return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));
}

// UECU Create Event
int Stim::cmd_crt_evnt( uint8_t sched_id, 
                        uint16_t delay, 
                        uint8_t priority, 
                        uint8_t event_type, 
                        uint8_t port_chn_id,
                        uint8_t pulse_width,
                        uint8_t amplitude,
                        uint8_t zone) {
  // calculate message size
  int size = CREATE_EVENT_MSG_LEN + UECU_MSG_EXTRAL_LEN;
  // build message content
  uint8_t msg[CREATE_EVENT_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
  {   MSG_DES_ADDR,
    MSG_SRC_ADDR,
    CREATE_EVENT_MSG,
    CREATE_EVENT_MSG_LEN,
    sched_id,
    (uint8_t)((delay >> 8) & 0x00FF),
    (uint8_t)(delay & 0x00FF),
    priority,
    event_type,
    port_chn_id,
    pulse_width, //Param[1]
    amplitude, //Param[2]
    zone, //Param[3] not implemented
    0x00
  };

  // Insert checksum byte
  msg[size-1] = this->checksum(msg,size);

  // Send message
  return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));
}
// UECU Change Event Parameter Command
int Stim::cmd_set_evnt( uint8_t event_id,
                        uint8_t pulse_width,
                        uint8_t amplitude,
                        uint8_t zone) {
  // calculate message size
  int size = CHANGE_EVENT_PARAMS_MSG_LEN + UECU_MSG_EXTRAL_LEN;
  // build message content
  uint8_t msg[CHANGE_EVENT_PARAMS_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
  {   MSG_DES_ADDR,
    MSG_SRC_ADDR,
    CHANGE_EVENT_PARAMS_MSG,
    CHANGE_EVENT_PARAMS_MSG_LEN,
    event_id,
    pulse_width,
    amplitude,
    zone, //Param[3] not implemented
    0x00
  };

  // Insert checksum byte
  msg[size-1] = this->checksum(msg,size);

  // Serial.print("In set event loop, ");
  // Serial.print("event_id = ");
  // Serial.print(event_id,HEX);
  // Serial.print(";\t pulse_width = ");
  // Serial.print(pulse_width,HEX);
  // Serial.print(";\t amplitude = ");
  // Serial.println(amplitude,HEX);


  // Send message
  return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));
}
// UECU Change Schedule Message
int Stim::cmd_set_sched( uint8_t sched_id,
                         uint8_t sync_signal,
                         uint16_t duration) {
  // calculate message size
  int size = CHANGE_SCHEDULE_MSG_LEN + UECU_MSG_EXTRAL_LEN;
  // build message content
  uint8_t msg[CHANGE_SCHEDULE_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
  {   MSG_DES_ADDR,
    MSG_SRC_ADDR,
    CHANGE_SCHEDULE_MSG,
    CHANGE_SCHEDULE_MSG_LEN,
    sched_id,
    sync_signal,
    (uint8_t)((duration >> 8) & 0x00FF),
    (uint8_t)(duration & 0x00FF),
    0x00
  };

  // Insert checksum byte
  msg[size-1] = this->checksum(msg,size);

  // Send message
  return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));
}

// UECU Sync Message
int Stim::cmd_sync_msg( uint8_t sync_signal) {
  // calculate message size
  int size = SYNC_MSG_LEN + UECU_MSG_EXTRAL_LEN;
  // build message content
  uint8_t msg[SYNC_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
  {   MSG_DES_ADDR,
    MSG_SRC_ADDR,
    SYNC_MSG,
    SYNC_MSG_LEN,
    sync_signal,
    0x00
  };

  // Insert checksum byte
  msg[size-1] = this->checksum(msg,size);

  // Send message
  return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));
}