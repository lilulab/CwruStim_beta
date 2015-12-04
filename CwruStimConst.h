#ifndef CwruStimConst_h
#define CwruStimConst_h

// UART Channel Setting
static const int STIM_CHANNEL_UART0 = 0;
static const int STIM_CHANNEL_UART1 = 1;
static const int STIM_CHANNEL_UART3 = 3;

static const int STIM_PORT_UART0 = 0;
static const int STIM_PORT_UART1 = 1;
static const int STIM_PORT_UART3 = 3;

// Stim settings
static const int STIM_MODE_DEFAULT = 0; // Not used
static const int STIM_MODE_SURF = 1;  // For surface stimulation board
static const int STIM_MODE_PERC = 2;  // For percutaneous stimulation board
static const int STIM_MODE_PERC_8CH_MULTI_SCHEDULE = 3;  // multi scheduler for percutaneous stimulation board

// Upper limit of channels
static const int STIM_CHANNEL_MAX_PERC = 8;

static const int STIM_SETTING_DEFAULT = 0;

// Error Flag (bitwise encoded)
static const int STIM_NO_ERROR = 0<<0;
static const int STIM_ERROR_SERIAL_ERROR = 1<<1;
static const int STIM_ERROR_UPDATE_TYPE_ERROR = 1<<2;
static const int STIM_ERROR_UPDATE_PATTERN_ERROR = 1<<3;

// UECU Message Header
static int MSG_DES_ADDR; 
static const int MSG_DES_ADDR_SURF = 0x04;
static const int MSG_DES_ADDR_PERC = 0x01;
static const int MSG_SRC_ADDR = 0x80;

// UECU Message Type
static const int TRIGGER_SETUP_MSG = 0x03;
static const int HALT_MSG = 0x04;
static const int ERROR_REPORT_MSG = 0x05;
static const int EVENT_ERROR_MSG = 0x06;
static const int CREATE_SCHEDULE_MSG = 0x10;
static const int CREATE_SCHEDULE_REPLY_MSG = 0x11;
static const int DELETE_SCHEDULE_MSG = 0x12;
static const int CHANGE_SCHEDULE_MSG = 0x13;
static const int CHANGE_SCHEDULE_STATE_MSG = 0x14;
static const int CREATE_EVENT_MSG = 0x15;
static const int CREATE_EVENT_REPLY_MSG = 0x16;
static const int DELETE_EVENT_MSG = 0x17;
static const int CHANGE_EVENT_SCHED_MSG = 0x18;
static const int CHANGE_EVENT_PARAMS_MSG = 0x19;
static const int SYNC_MSG = 0x1B;
static const int EVENT_COMMAND_MSG = 0x1C;
static const int CHANNEL_SETUP_MSG = 0x47;
static const int EVENT_COMMAND_REPLY = 0x49;

// UECU Message Length
static const int TRIGGER_SETUP_MSG_LEN = 4;
static const int HALT_MSG_LEN = 1;
static const int ERROR_REPORT_MSG_LEN = 2;
static const int EVENT_ERROR_MSG_LEN = 4;
static const int CREATE_SCHEDULE_MSG_LEN = 3;
static const int CREATE_SCHEDULE_REPLY_MSG_LEN = 1;
static const int DELETE_SCHEDULE_MSG_LEN = 1;
static const int CHANGE_SCHEDULE_MSG_LEN = 4;
static const int CHANGE_SCHEDULE_STATE_MSG_LEN = 2;
static const int CREATE_EVENT_MSG_LEN = 9;
static const int CREATE_EVENT_REPLY_MSG_LEN = 4;
static const int DELETE_EVENT_MSG_LEN = 1;
static const int CHANGE_EVENT_SCHED_MSG_LEN = 5;
static const int CHANGE_EVENT_PARAMS_MSG_LEN = 4;
static const int SYNC_MSG_LEN = 1;
static const int EVENT_COMMAND_MSG_LEN = 0;
static const int CHANNEL_SETUP_MSG_LEN = 7;
static const int EVENT_COMMAND_REPLY_LEN = 0;

static const int UECU_MSG_EXTRAL_LEN = 5; // 2 Header + 1 msg_type + 1 msg_length +1 Check_sum = 5

// UECU Halt
static const int UECU_HALT = 0x00;
static const int UECU_RESET = 0x01;

// UECU Sync Message
static const int UECU_SYNC_MSG = 0xAA;

// Delay for setup
static const int UECU_DELAY_SETUP = 25;

// Gait Params
static const int NUM_GAIT_VCK5_BOARD = 2;
static const int GAIT_VCK5_BOARD1 = 0;
static const int GAIT_VCK5_BOARD2 = 1;

static const int VCK5_BRD1 = 0<<8;
static const int VCK5_BRD2 = 1<<8;

static const int IPI = 1;
static const int AMP = 2;
static const int PW = 3;

static const int NUM_PATTERN = 5;
static const int PATTERN_NO_STIM = 0;
static const int PATTERN_STAND = 1;
static const int PATTERN_SIT = 2;
static const int PATTERN_LSETP = 3;
static const int PATTERN_RSETP = 4;


static const int NUM_PATTERN_PARAM = 4;
static const int PATTERN_PARAM_PP = 0;
static const int PATTERN_PARAM_PW = 1;
static const int PATTERN_PARAM_AMP = 2;
static const int PATTERN_PARAM_IPI = 3;

// gait look up table resolution.
static const int GAIT_LUT_RES = 8;




#endif