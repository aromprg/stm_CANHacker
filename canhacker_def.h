#ifndef CANHACKER_DEF_H_
#define CANHACKER_DEF_H_

#define HW_VER        0x10  // hardware version
#define SW_VER        0x10  // software version
#define SW_VER_MAJOR  0x01  // software major version
#define SW_VER_MINOR  0x09  // software minor version
#define SERIAL        "0001"// device serial number

#if !defined(CMD_END_CR)
#define CMD_END_CR    '\r'	// command end tag (ASCII CR)
#endif

#if !defined(ERROR_BEL)
#define ERROR_BEL     '\a'	// error tag (ASCII BEL)
#endif

#define SET_BITRATE     'S'	// set CAN bit rate
#define SET_BTR         's'	// set CAN bit rate via
#define OPEN_CAN_CHAN   'O'	// open CAN channel
#define CLOSE_CAN_CHAN  'C'	// close CAN channel
#define SEND_11BIT_ID   't'	// send CAN message with 11bit ID
#define SEND_29BIT_ID   'T'	// send CAN message with 29bit ID
#define SEND_R11BIT_ID  'r'	// send CAN remote message with 11bit ID
#define SEND_R29BIT_ID  'R'	// send CAN remote message with 29bit ID
#define READ_STATUS     'F'	// read status flag byte
#define SET_ACR         'M'	// set Acceptance Code Register
#define SET_AMR         'm'	// set Acceptance Mask Register
#define GET_VERSION     'V'	// get hardware and software version
#define GET_SW_VERSION  'v' // get software version only
#define GET_SERIAL      'N'	// get device serial number
#define TIME_STAMP      'Z'	// toggle time stamp setting
#define READ_ECR        'E'	// read Error Capture Register
#define READ_ALCR       'A'	// read Arbritation Lost Capture Register
#define READ_REG_SJ     'G'	// read register conten from SJA1000
#define WRITE_REG_SJ    'W'	// write register content to SJA1000
#define LISTEN_ONLY     'L'	// switch to listen only mode

#define TIME_STAMP_TICK 1000	// microseconds

/*
 define command receive buffer length
 minimum length is define as follow:
 1 byte	: command identifier
 8 byte	: CAN identifier (for both 11bit and 29bit ID)
 1 byte	: CAN data length (0-8 byte)
 2*8 byte: CAN data (one data byte is send as two ASCII chars)
 1 byte  : [CR] command end tag
 ---
 27 byte
 */
#define CMD_BUFFER_LENGTH  30


#endif /* CANHACKER_DEF_H_ */
