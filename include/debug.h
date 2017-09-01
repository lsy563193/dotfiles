#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "config.h"

#include "map.h"

#if  defined(DEBUG_MAP)  || defined(DEBUG_SM_MAP)
#if COLOR_DEBUG_MAP
void color_print(char *outString,int16_t ,int16_t);
#endif
void debug_map(uint8_t id, int16_t endx, int16_t endy);
#else

#define debug_map(a, b, c) { do {;} while(0) }
#endif

#endif
