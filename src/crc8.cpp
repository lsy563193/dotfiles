#include "crc8.h"

#define GP		0x107	/*crc: x^8 + x^2 + x + 1 */
#define DI		0x07

static int made_table = 0;
static unsigned char crc8_table[256];	/* 8-bit table */

void init_crc8(void)
{
	int				i, j;
	unsigned char	crc;
  
	if (!made_table) {
		for (i = 0; i < 256; i++) {
			crc = i;
			for (j = 0; j < 8; j++)
				crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
			crc8_table[i] = crc & 0xFF;
		}
		made_table = 1;
	}
}

/*
 * For a byte array whose accumulated crc value is stored in *crc, computes
 * resultant crc obtained by appending m to the byte array
 */
void crc8(unsigned char *crc, unsigned char m)
{
	if (!made_table) {
		init_crc8();
	}

	*crc = crc8_table[(*crc) ^ m];
	*crc &= 0xFF;
}

uint8_t calc_buf_crc8(char *inBuf, uint32_t inBufSz)
{
	uint8_t		crc_base = 0;
	uint32_t	i;
    
	for (i = 0; i < inBufSz; i++) {
		crc8(&crc_base, inBuf[i]);
	}
	return crc_base;
}

#if 0
int main(int argc, char** argv)
{
	const char* buf1 = "<ctl td='WIFIStat' st='c'/>";
	const char* buf2 = "<ctl td='GetDeviceCap'/>";
	uint8_t ret = calcBufCrc8(buf1, strlen(buf1)+1);

	printf("buf1 crc8: 0x%x\n", ret);
	ret = calcBufCrc8(buf2, strlen(buf2)+1);
	printf("buf2 crc8: 0x%x\n", ret);
}
#endif
