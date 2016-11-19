#ifndef __ROUNDING_H__
#define __ROUNDING_H__

#include "map.h"

typedef enum {
	ROUNDING_LEFT = 0,
	ROUNDING_RIGHT,
} RoundingType;

uint8_t rounding(RoundingType type, Point32_t target);

#endif
