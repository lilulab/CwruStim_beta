/*
  CwruStim.cpp - Library for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Jul, 2015.
  Version 1.0
*/

#include "Arduino.h"
#include "CwruStim.h"

int Stim::_uart_channel_id = 0;
int Stim::_mode = 0;
int Stim::_setting = 0;
int Stim::_stim_error = 0;
int Stim::_command = 0;
 
int Stim::_dir = 1;
int Stim::_min = 0x00;
int Stim::_max = 0xFA;
//String _serlInput = "12345678", serlError = "0234AA06";
uint8_t Stim::_halt_rset[6] = {0x04, 0x80, 0x04, 0x01, 0x01, 0x75};
uint8_t Stim::_del_sched[6] = {0x04, 0x80, 0x12, 0x01, 0x01, 0x67};
uint8_t Stim::_chan_set1[12] = {0x04, 0x80, 0x47, 0x07, 0x00, 0x64, 0xFA, 0x00, 0x64, 0x11, 0x01, 0x57};
uint8_t Stim::_chan_set2[12] = {0x04, 0x80, 0x47, 0x07, 0x00, 0x64, 0xFA, 0x00, 0x64, 0x11, 0x23, 0x35};
uint8_t Stim::_chan_set3[12] = {0x04, 0x80, 0x47, 0x07, 0x00, 0x64, 0xFA, 0x00, 0x64, 0x11, 0x45, 0x13};

uint8_t Stim::_crt_sched[8] = {0x04, 0x80, 0x10, 0x03, 0xAA, 0x00, 0x1D, 0xA0};
uint8_t Stim::_crt_evnt1[14] = {0x04, 0x80, 0x15, 0x09, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x59}; 
uint8_t Stim::_crt_evnt2[14] = {0x04, 0x80, 0x15, 0x09, 0x01, 0x00, 0x05, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x53}; 
uint8_t Stim::_crt_evnt3[14] = {0x04, 0x80, 0x15, 0x09, 0x01, 0x00, 0x0A, 0x00, 0x03, 0x02, 0x00, 0x00, 0x00, 0x4D};
uint8_t Stim::_sync_msg1[6] = {0x04, 0x80, 0x1B, 0x01, 0xAA, 0xB4};

uint8_t Stim::_chngevnt1[9] = {0x04, 0x80, 0x19, 0x04, 0x01, 0x00, 0x3C, 0x00, 0x00};

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
			//Serial.begin(9600);
			while (!Serial) {_stim_error |= STIM_ERROR_CANNOT_OPEN_SERIAL;} //Set Error bit
			if (Serial) _stim_error &= ~STIM_NO_ERROR; //Clear Error bit
			break;

		#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		//Code in here will only be compiled if an Arduino Mega is used.

			case STIM_CHANNEL_UART1:
				//Use UART1(TX1&RX1) port to connect Stim Board
				Serial1.begin(9600);
				while (!Serial1) {_stim_error |= STIM_ERROR_CANNOT_OPEN_SERIAL;} //Set Error bit
				if (Serial1) _stim_error &= ~STIM_NO_ERROR; //Clear Error bit
				break;

			case STIM_CHANNEL_UART3:
				//Use UART3(TX3&RX3) port to connect Stim Board
				Serial3.begin(9600);
				while (!Serial3) {_stim_error |= STIM_ERROR_CANNOT_OPEN_SERIAL;} //Set Error bit
				if (Serial3) {_stim_error &= ~STIM_NO_ERROR;} //Clear Error bit
				break;
		#endif

		default:
			_stim_error |= STIM_ERROR_CANNOT_OPEN_SERIAL;
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

	return 1;
}

// Configure Stim Board via UART
int Stim::config(int setting) {

	//Serial.println("Enter Stim Config");

	delay(50);
	this->serial_write_array (_halt_rset,sizeof(_halt_rset)/sizeof(uint8_t));
	//this->serial_write_array (del_sched);
	
	this->serial_write_array (_chan_set1,sizeof(_chan_set1)/sizeof(uint8_t));
	this->serial_write_array (_chan_set2,sizeof(_chan_set2)/sizeof(uint8_t));
	this->serial_write_array (_chan_set3,sizeof(_chan_set3)/sizeof(uint8_t));
	this->serial_write_array (_crt_sched,sizeof(_crt_sched)/sizeof(uint8_t));
	this->serial_write_array (_crt_evnt1,sizeof(_crt_evnt1)/sizeof(uint8_t));
	this->serial_write_array (_crt_evnt2,sizeof(_crt_evnt2)/sizeof(uint8_t));
	this->serial_write_array (_crt_evnt3,sizeof(_crt_evnt3)/sizeof(uint8_t));
	this->serial_write_array (_sync_msg1,sizeof(_sync_msg1)/sizeof(uint8_t));

	_chngevnt1[5] = _min + 1;

	_setting = setting;
	//Serial.println("Exit Stim Config");
	return 1;
}

// Update Stim pattern via UART
int Stim::update(int command) {
	if((_chngevnt1[5] + _dir) > _max | (_chngevnt1[5] + _dir) < _min) { _dir = -_dir; }
	_chngevnt1[5]+=_dir;
	//chngevnt1[chngevnt1.length-1] = checksum(chngevnt1);
	this->serial_write_array (_chngevnt1,sizeof(_chngevnt1)/sizeof(uint8_t));

	delay(50);

	_command = command;
	return 1;
}

int Stim::serial_write_array(uint8_t buf[], int length) {
	//Serial.write (buf,sizeof(buf)/sizeof(uint8_t));
	//Serial.write (buf,sizeof(buf)/sizeof( int));
	//Serial.write (buf,4);
	//Serial.println(buf);

	// ToDo: switch different UART channel.
	
	for(int i = 0; i<length; i++){
		Serial.write(buf[i]);
		//Serial.print(buf[i],HEX);
		//Serial.print(" ");
	}
	//Serial.println(".");

	// Serial.print("sizeof(buf)=");Serial.print(sizeof(buf));Serial.println(".");
	// Serial.print("sizeof(int)=");Serial.print(sizeof(uint8_t));Serial.println(".");
	// Serial.print("sizeof_rest=");Serial.print(sizeof(buf)/sizeof(uint8_t));Serial.println(".");
	// int size1 = sizeof buf;
	// Serial.print("sizeof_2ndm=");Serial.print(size1);Serial.println(".");

	delay(50);
	return Serial;
}

uint8_t Stim::checksum(uint8_t vals[], int length){
  uint8_t csum = 0;
  for(int i=0; i<length-1; i++) {
    csum += vals[i];
  }
  csum = ((0x00FF & csum) + (csum >> 8))^0xFF;
  return csum;
} 

// uint8_t Stim::_chan_seti[12] = {0x04, 0x80, 0x47, 0x07, 0x00, 0x64, 0xFA, 0x00, 0x64, 0x11, 0x01, 0x57};
// UECU command sets
//UECU Halt
int Stim::cmd_halt_rset(int halt_flag) {
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
int Stim::cmd_del_sched(int sched_id) {

}

// UECU Create Schedule
int Stim::cmd_crt_sched(int sync_signal, int duration) {

}

// UECU Channel Setup 
int Stim::cmd_chan_set(	int port_chn_id, 
                        int amp_limit,
                        int ip_delay, 
                        int asp_ratio, 
                        int anode_cathode) {

}

// UECU Create Event
int Stim::cmd_crt_evnt( int sched_id, 
                        int delay, 
                      	int priority, 
                      	int event_type, 
                      	int port_chn_id) {

}
// UECU Change Event Parameter Command
int Stim::cmd_set_evnt( int event_id,
                        int pulse_width,
                        int amplitude) {

}
// UECU Change Schedule Message
int Stim::cmd_set_sched( int sched_id,
                         int sync_signal,
                         int duration) {

}