#ifndef __VERIFY_H__
#define __VERIFY_H__

#include "config.h"
#include "serial.h"

#define K_SIGN_OFFSET			0x00 // 0x0C
#define SIGNATURE_SIZE			16
#define ENCRYPTION_KEY_SIZE		24

#define Direction_Flag_Right 0x01
#define Direction_Flag_Left  0x02

#define STEP_PER_MM  186
#define Cliff_Limit         (int16_t)20

#define Two_Hours         7200

#define DUMMY_DOWNLINK_OFFSET		2
#define KEY_DOWNLINK_OFFSET			9
#define SEQUENCE_DOWNLINK_OFFSET	7

#define DUMMY_DOWNLINK_LENGTH		5
#define SEQUENCE_DOWNLINK_LENGTH	2
#define KEY_DOWNLINK_LENGTH			8

#define KEY_UPLINK_OFFSET			16
#define KEY_UPLINK_LENGTH			16

#define CMD_UPLINK_OFFSET			REC_KEY_VALIDATION

#define CMD_KEY1					0x40
#define CMD_KEY2					0x41
#define CMD_KEY3					0x42
#define CMD_ID						0x43

#define CMD_ACK						0x23
#define CMD_NCK						0x25

enum {
	K_SIGN_CORRECT	= 1,
	K_SIGN_WR		= 0,
	K_ERR_INPUT		= -1,
	K_ERR_FILE		= -2,
	K_ERR_HEADER	= -3,
	K_ERR_SIGN		= -4
};

#if VERIFY_CPU_ID

int verify_cpu_id(void);

#endif

#if VERIFY_KEY

int verify_key(void);

#endif

#endif
