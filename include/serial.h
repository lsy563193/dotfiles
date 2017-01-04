#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>

typedef enum {
	SERIAL_NONE = 0,
	SERIAL_RESET,
	SERIAL_START,
	SERIAL_UPDATE,
} SerialCommandType;

void serial_init(const char* port,int baudrate); 
int serial_close();
int serial_flush();
bool is_serial_ready();
int serial_write(uint8_t len, uint8_t *buf);
int serial_read(int len,uint8_t *buf);
//SerialCommandType  serial_read();

#endif
