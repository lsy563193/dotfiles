#include <stdio.h>

#define PROGVER			1 // Program Version
#define PROGSUBVER		0 // Program Sub-Version
#define PROGREVISE		0 // Program Revise number

const unsigned int K_ILIFE_SECTION[9] __attribute__ ((section ("ilife"))) =
{
	(unsigned int)((0xF8) | (0x8E << 8) | (0xC9 << 16) | (0xEA << 24)),
	(unsigned int)((0xC8) | (0x3D << 8) | (0xA5 << 16) | (0x67 << 24)),
	(unsigned int)((0x6B) | (0xC1 << 8) | (0xAE << 16) | (0x9A << 24)),
	(unsigned int)((0x12) | (0xF0 << 8) | (0xB4 << 16) | (0x7F << 24)),

	//Product Name
	(unsigned int)(('I') | ('L' << 8) | ('I' << 16) | ('F' << 24)),
	(unsigned int)(('E') | (0 << 8) | (0 << 16) | (0 << 24)),
	(unsigned int)((PROGVER) |  (PROGSUBVER << 8) | ('r' << 16) | (PROGREVISE << 24)),
	(unsigned int)(('i') | ('l' << 8) | ('i' << 16) | ('f' << 24)),
	(unsigned int)(('e') | ('c' << 8) | ('o' << 16) | ('m' << 24)),
};
