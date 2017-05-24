#ifndef __WALL_FOLLOW_SLAM_H__
#define __WALL_FOLLOW_SLAM_H__

#include "config.h"

#include "mathematics.h"
#include "debug.h"
#include <functional>
#include <future>
#include "wall_follow_trapped.h"

uint8_t Wall_Follow(MapWallFollowType follow_type);
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
