/*
  CwruStim.h - Library for muscle stim board for HNPv2 Project.
  Created by Lu Li (lxl361@case), Jul, 2015.
  Version 1.0
*/

#ifndef CwruStim_h
#define CwruStim_h

#include "Arduino.h"

/*
  HNPv2 NTREK Embedded Controller Board(ECB) 
  Support two channels for muscle stimulation board UART communication.
  UART1 (TX1 and RX1) for stim board 1
  UART3 (TX3 and RX3) fpr stim board 3
 */ 
#define STIM_CHANNEL_UART0 0
#define STIM_CHANNEL_UART1 1
#define STIM_CHANNEL_UART3 3

#define STIM_MODE_DEFAULT 0

#define STIM_SETTING_DEFAULT 0

#define STIM_COMMAND_DEFAULT 0

// Error Flag (bitwise encoded)
#define STIM_NO_ERROR 0<<0
#define STIM_ERROR_CANNOT_OPEN_SERIAL 1<<1

// UECU Message Header
#define MSG_DES_ADDR 0x04
#define MSG_SRC_ADDR 0x80

// UECU Message Type
#define TRIGGER_SETUP_MSG 0x03
#define HALT_MSG 0x04
#define ERROR_REPORT_MSG 0x05
#define EVENT_ERROR_MSG 0x06
#define CREATE_SCHEDULE_MSG 0x10
#define CREATE_SCHEDULE_REPLY_MSG 0x11
#define DELETE_SCHEDULE_MSG 0x12
#define CHANGE_SCHEDULE_MSG 0x13
#define CHANGE_SCHEDULE_STATE_MSG 0x14
#define CREATE_EVENT_MSG 0x15
#define CREATE_EVENT_REPLY_MSG 0x16
#define DELETE_EVENT_MSG 0x17
#define CHANGE_EVENT_SCHED_MSG 0x18
#define CHANGE_EVENT_PARAMS_MSG 0x19
#define SYNC_MSG 0x1B
#define EVENT_COMMAND_MSG 0x1C
#define CHANNEL_SETUP_MSG 0x47
#define EVENT_COMMAND_REPLY 0x49

class Stim
{
  public:
    Stim(int channel_id); // Stim constructor and UART selector
    int init(int mode); // Initialize Stim Board via UART
    int config(int setting); // Configurate Stim Board via UART
    int update(int command); // Update Stim pattern via UART
    int serial_write_array(uint8_t buf[],int length);

  private:
    static int _channel_id;
    static int _mode;
    static int _setting;
    static int _stim_error;
    static int _command;

    // Commend Holder
  	static int _dir;
    static int _min;
    static int _max;

  	//String _serlInput = "12345678", serlError = "0234AA06";
  	static uint8_t _halt_rset[6];
  	static uint8_t _del_sched[6];
  	static uint8_t _chan_set1[12];
  	static uint8_t _chan_set2[12];
  	static uint8_t _chan_set3[12];
  	static uint8_t _crt_sched[8];
  	static uint8_t _crt_evnt1[14]; 
  	static uint8_t _crt_evnt2[14]; 
  	static uint8_t _crt_evnt3[14];
  	static uint8_t _sync_msg1[6];

  	static uint8_t _chngevnt1[9];

    int checksum(int vals[], int length);
};

#endif



