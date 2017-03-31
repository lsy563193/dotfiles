#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "config.h"

#include "map.h"

#ifdef  DEBUG_MAP
void debug_map(uint8_t id, int16_t endx, int16_t endy);
#endif

void debug_WF_map(uint8_t id, int16_t endx, int16_t endy);

#ifdef  DEBUG_SM_MAP
void debug_sm_map(uint8_t id, int16_t endx, int16_t endy);
#endif

#endif
