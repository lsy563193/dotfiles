#ifndef __WALLFOLLOWMULTI_H__
#define __WALLFOLLOWMULTI_H__

#include "config.h"

#include "mathematics.h"
#include "debug.h"
#include <functional>
#include <future>
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
void Wall_Follow_Init_Slam(void);
void Wall_Follow_Stop_Slam(void);
void WF_update_position(void);
void WF_Check_Loop_Closed(uint16_t heading);
bool WF_Is_Reach_Cleaned(void);
int8_t WF_Push_Point(int32_t x, int32_t y, int16_t th);
uint8_t WF_End_Wall_Follow(void);
uint8_t WF_Break_Wall_Follow(void);
void WFM_move_back(uint16_t dist);
bool WF_check_isolate(void);
void WF_Mark_Home_Point(void);
bool WF_Check_Angle(void);
#endif
