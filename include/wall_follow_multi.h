#ifndef __WALLFOLLOWMULTI_H__
#define __WALLFOLLOWMULTI_H__

#include "config.h"

#include "mathematics.h"
#include "debug.h"

typedef enum {
	Map_Wall_Follow_None = 0,
	Map_Wall_Follow_Escape_Trapped,
} MapWallFollowType;

typedef enum {
	Map_Escape_Trapped_Escaped,
	Map_Escape_Trapped_Trapped,
	Map_Escape_Trapped_Timeout,
} MapEscapeTrappedType;

typedef struct {
	int32_t front_obs_val;
	int32_t left_bumper_val;
	int32_t right_bumper_val;
} MapWallFollowSetting;

uint8_t Map_Wall_Follow(MapWallFollowType follow_type);
uint8_t Wall_Follow(MapWallFollowType follow_type);

#endif
