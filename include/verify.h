#ifndef __VERIFY_H__
#define __VERIFY_H__

#include "config.h"

#define K_SIGN_OFFSET			0x00 // 0x0C
#define SIGNATURE_SIZE			16
#define ENCRYPTION_KEY_SIZE		24

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
