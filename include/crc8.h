#ifndef __CRC8_H__
#define __CRC8_H__

#include <stdint.h>

void init_crc8(void);
void crc8(unsigned char *crc, unsigned char m);

uint8_t calcBufCrc8(char *inBuf, uint32_t inBufSz);

#endif
