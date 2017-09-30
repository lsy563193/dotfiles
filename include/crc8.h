#ifndef __CRC8_H__
#define __CRC8_H__

#include <stdint.h>

void init_crc8(void);
void crc8(uint8_t *crc, const uint8_t m);

uint8_t calc_buf_crc8(const uint8_t *inBuf, uint32_t inBufSz);

#endif
