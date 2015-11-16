/*
  CwruStim.cpp - Library for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Aug, 2015.
  Version 1.1
  Online Doc: https://goo.gl/s20iH4
  Repo: https://github.com/lilulab/CwruStim_beta
*/

#include "Arduino.h"
#include "CwruStim.h"

int Stim::_uart_channel_id = 0;
int Stim::_mode = 0;
int Stim::_setting = 0;
int Stim::_stim_error = 0;
int Stim::_command = 0;
 
// Sync messages
uint8_t Stim::_PERC_8CH_SYNC_MSG[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};

// Inter phase interval
// CHECKLIST: Need to update this later according to gait pattern file!
uint16_t Stim::_PERC_8CH_IPI[8] = {30, 60, 30, 60, 30, 30, 30, 30};

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
			this->cmd_chan_set(	0,		// port_chn_id =  0
								100,	// amp_limit = 100mA
								0xFA,	// pw_limit = 250usec
								100,	// ip_delay = 100usec
								0x11, 	// asp_ratio = 1:1
								0x01);	//anode = 0, cathode = 1, for bipolar mode);

			// Bipolar 23
			this->cmd_chan_set(	1,		// port_chn_id =  0
								100,	// amp_limit = 100mA
								0xFA,	// pw_limit = 250usec
								100,	// ip_delay = 100usec
								0x11, 	// asp_ratio = 1:1
								0x23);	//anode = 2, cathode = 3, for bipolar mode);

			// Bipolar 45
			this->cmd_chan_set(	2,		// port_chn_id =  0
								100,	// amp_limit = 100mA
								0xFA,	// pw_limit = 250usec
								100,	// ip_delay = 100usec
								0x11, 	// asp_ratio = 1:1
								0x45);	//anode = 0, cathode = 1, for bipolar mode);

			// Bipolar 67
			this->cmd_chan_set(	3,		// port_chn_id =  0
								100,	// amp_limit = 100mA
								0xFA,	// pw_limit = 250usec
								100,	// ip_delay = 100usec
								0x11, 	// asp_ratio = 1:1
								0x67);	//anode = 0, cathode = 1, for bipolar mode);



			// Create Schedule
			// this->cmd_crt_sched(sync_signal, duration);
			this->cmd_crt_sched(UECU_SYNC_MSG, 29);	// Sync signal = 0xAA, duration 29msec.

			// Create Events
			// this->cmd_crt_evnt(sched_id, delay, priority, event_type, port_chn_id);

			// Create Event 1 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0,	// delay = 0msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								0,	// port_chn_id = 0;
								0,	// pulse_width set to 0,
		            0,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 2 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0,	// delay = 0msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								1,	// port_chn_id = 1;
								0,	// pulse_width set to 0,
		            0,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 3 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0,	// delay = 0msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								2,	// port_chn_id = 2;
								0,	// pulse_width set to 0,
		            0,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 4 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0,	// delay = 0msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								3,	// port_chn_id = 3;
								0,	// pulse_width set to 0,
		            0,	// amplitude set to 0,
		            0);	// zone not implemented;
			return 1;
			break;

		// For Perc board
		case STIM_MODE_PERC:

				// Serial.print("EXE STIM_MODE_PERC");
			// Create Schedule ------------------------------
			// crt_sched	: 01 80 10 03 AA 00 1D A3 

			// this->cmd_crt_sched(sync_signal, duration);
			this->cmd_crt_sched(UECU_SYNC_MSG, 29);	// Sync signal = 0xAA, duration 29msec (0d29 = 0x1D).

			// Create Event 1-12  ----------------------------------
			// crt_evnt1	: 01 80 15 09 01 00 00 00 03 00 50 10 00 FB
			// crt_evnt2	: 01 80 15 09 01 00 05 00 03 01 80 15 00 C0
			// ...
			// crt_evnt12	: 01 80 15 09 01 00 0F 00 03 0B F0 26 00 2B

			// this->cmd_crt_evnt(sched_id, delay, priority, event_type, port_chn_id);
			// Create Event 1 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0,	// delay = 0msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								0,	// port_chn_id = 0;
								0x50,	// pulse_width set to 0,
		            0x10,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 2 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x05,	// delay = 5msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								1,	// port_chn_id = 1;
								0x80,	// pulse_width set to 0,
		            0x15,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 3 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x0A,	// delay = 10msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								2,	// port_chn_id = 2;
								0xB0,	// pulse_width set to 0,
		            0x20,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 4 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x0F,	// delay = 15msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								3,	// port_chn_id = 3;
								0xF0,	// pulse_width set to 0,
		            0x26,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 5 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0,	// delay = 0msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								4,	// port_chn_id = 4;
								0x50,	// pulse_width set to 0,
		            0x10,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 6 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x05,	// delay = 5msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								5,	// port_chn_id = 5;
								0x80,	// pulse_width set to 0,
		            0x15,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 7 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x0A,	// delay = 10msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								6,	// port_chn_id = 6;
								0xB0,	// pulse_width set to 0,
		            0x20,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 8 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x0F,	// delay = 15msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								7,	// port_chn_id = 7;
								0xF0,	// pulse_width set to 0,
		            0x26,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 9 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0,	// delay = 0msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								8,	// port_chn_id = 8;
								0x50,	// pulse_width set to 0,
		            0x10,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 10 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x05,	// delay = 5msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								9,	// port_chn_id = 9;
								0x80,	// pulse_width set to 0,
		            0x15,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 11 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x0A,	// delay = 10msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								10,	// port_chn_id = 10;
								0xB0,	// pulse_width set to 0,
		            0x20,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Event 12 for port_chn_id 0 in sched_id 1 
			this->cmd_crt_evnt( 
							  1,	// sched_id = 1
								0x0F,	// delay = 15msec
								0,	// priority = 0
								3,	// event_type = 3, for for Stimulus Event
								11,	// port_chn_id = 11;
								0xF0,	// pulse_width set to 0,
		            0x26,	// amplitude set to 0,
		            0);	// zone not implemented;

			// Create Sync Message ----------------------------------
			// sync_msg1	: 01 80 1B 01 AA B7
			return 1;
			break;

		// multi scheduler for percutaneous stimulation board
		case STIM_MODE_PERC_8CH_MULTI_SCHEDULE:

				// Serial.print("EXE STIM_MODE_PERC_8CH_MULTI_SCHEDULE");

			// Create Schedule ------------------------------
			// crt_sched	: 01 80 10 03 AA 00 1D A3 

			// this->cmd_crt_sched(sync_signal, duration);
			// Create 8 schedules
			for (uint8_t i=0; i<8; i++) {

				// QUESTION: setup 8 schedules than 8 events, or do it like below?

				// Create schedule
				// CHECKLIST: Need to set IPI array first!
				//this->cmd_crt_sched(_PERC_8CH_SYNC_MSG[i], _PERC_8CH_IPI[i]);	// Sync signal, duration 30msec.
				this->cmd_crt_sched(UECU_SYNC_MSG, _PERC_8CH_IPI[i]);	// Sync signal, duration 30msec.

				delay(UECU_DELAY_SETUP);

				// Create event 
				// this->cmd_crt_evnt(sched_id, delay, priority, event_type, port_chn_id);
				// Create Event 1 for port_chn_id 0 in sched_id 1 
				this->cmd_crt_evnt( 
								  i+1,	// sched_id 1 to 8
									(i+1)*2,	// delay every 2ms. (2,4,6, ...)
									0,	// priority = 0
									3,	// event_type = 3, for for Stimulus Event
									i,	// port_chn_id = 0;
									// 0x00,	// pulse_width set to 0,
									50,	// pulse_width set to 0,
			            0x26,	// amplitude set to 0x26,
			            0);	// zone not implemented;

				Serial.print("In config loop, ");
				Serial.print("i = ");
				Serial.print(i,HEX);
				Serial.print(";\t IPI = ");
				Serial.println(_PERC_8CH_IPI[i],HEX);
				Serial.println(" ");



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
	for (int i=0; i<8; i++) {
		// Send Sync to start
		this->cmd_sync_msg(_PERC_8CH_SYNC_MSG[i]); // Sent Sync_message to start schedule.

		// Delay duration need to be save as IPI.
		delay(_PERC_8CH_IPI[i]);
	}

}

// Update Stim pattern via UART
int Stim::update(int command) {

	// STIM_COMMAND_DEMO
	switch (command) {

		case STIM_COMMAND_ZERO_ALL:

			// Pulse width 0us, Amplitude 0mA

			// this->cmd_set_evnt( event_id, pulse_width, amplitude, zone);
			// Change Event 1 for port_chn_id 0 in sched_id 1 
			this->cmd_set_evnt(	1,	//event_id
								0,//pulse_width = 100us
								0,	//amplitude = 60mA
								0);	// zone not implemented

			// Change Event 2 for port_chn_id 1 in sched_id 1 
			this->cmd_set_evnt(	2,	//event_id
								0,//pulse_width = 100us
								0,	//amplitude = 60mA
								0);	// zone not implemented	

			// Change Event 3 for port_chn_id 2 in sched_id 1 
			this->cmd_set_evnt(	3,	//event_id
								0,//pulse_width = 100us
								0,	//amplitude = 60mA
								0);	// zone not implemented	

			// Change Event 4 for port_chn_id 3 in sched_id 1 
			this->cmd_set_evnt(	4,	//event_id
								0,//pulse_width = 100us
								0,	//amplitude = 60mA
								0);	// zone not implemented

			delay(50); // delay 50ms
			break;

		case STIM_COMMAND_DEMO:

			// Pulse width 100us, Amplitude 60mA

			// this->cmd_set_evnt( event_id, pulse_width, amplitude, zone);
			// Change Event 1 for port_chn_id 0 in sched_id 1 
			this->cmd_set_evnt(	1,	//event_id
								100,//pulse_width = 100us
								60,	//amplitude = 60mA
								0);	// zone not implemented

			// Change Event 2 for port_chn_id 1 in sched_id 1 
			this->cmd_set_evnt(	2,	//event_id
								100,//pulse_width = 100us
								60,	//amplitude = 60mA
								0);	// zone not implemented	

			// Change Event 3 for port_chn_id 2 in sched_id 1 
			this->cmd_set_evnt(	3,	//event_id
								100,//pulse_width = 100us
								60,	//amplitude = 60mA
								0);	// zone not implemented	

			// Change Event 4 for port_chn_id 3 in sched_id 1 
			this->cmd_set_evnt(	4,	//event_id
								100,//pulse_width = 100us
								60,	//amplitude = 60mA
								0);	// zone not implemented

			//delay(50); // delay 50ms
			break;

		default:
			_stim_error |= STIM_ERROR_COMMAND_ERROR;
			break;
	}

	_command = command;
	return 1;
}

// UART write array
int Stim::serial_write_array(uint8_t buf[], int length) {
	
	// for(int i = 0; i<length; i++){
	// 	Serial.write(buf[i]);
	// 	//Serial.print(buf[i],HEX);
	// 	//Serial.print(" ");
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
	{ 	MSG_DES_ADDR,
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
	{ 	MSG_DES_ADDR,
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
	{ 	MSG_DES_ADDR,
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
int Stim::cmd_chan_set(	uint8_t port_chn_id, 
                        uint8_t amp_limit,
                        uint8_t pw_limit,
                        uint16_t ip_delay, 
                        uint8_t asp_ratio, 
                        uint8_t anode_cathode) {
	// calculate message size
	int size = CHANNEL_SETUP_MSG_LEN + UECU_MSG_EXTRAL_LEN;
	// build message content
	uint8_t msg[CHANNEL_SETUP_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
	{ 	MSG_DES_ADDR,
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
	{ 	MSG_DES_ADDR,
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
	{ 	MSG_DES_ADDR,
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

	Serial.print("In set event loop, ");
	Serial.print("event_id = ");
	Serial.print(event_id,HEX);
	Serial.print(";\t pulse_width = ");
	Serial.print(pulse_width,HEX);
	Serial.print(";\t amplitude = ");
	Serial.println(amplitude,HEX);


	// Send message
	return this->serial_write_array (msg,sizeof(msg)/sizeof(uint8_t));
}
// UECU Change Schedule Message
int Stim::cmd_set_sched( uint8_t sched_id,
                         uint8_t sync_signal,
                         uint16_t duration) {
	// calculate message size
	int size = CHANGE_EVENT_SCHED_MSG_LEN + UECU_MSG_EXTRAL_LEN;
	// build message content
	uint8_t msg[CHANGE_EVENT_SCHED_MSG_LEN + UECU_MSG_EXTRAL_LEN] = 
	{ 	MSG_DES_ADDR,
		MSG_SRC_ADDR,
		CHANGE_EVENT_SCHED_MSG,
		CHANGE_EVENT_SCHED_MSG_LEN,
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
	{ 	MSG_DES_ADDR,
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