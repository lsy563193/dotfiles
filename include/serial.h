#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>

typedef enum {
	SERIAL_NONE = 0,
	SERIAL_RESET,
	SERIAL_START,
	SERIAL_UPDATE,
} SerialCommandType;

void serial_init();
void serial_close();

int serial_write(uint8_t len, uint8_t *buf);

//SerialCommandType  serial_read();

#endif
