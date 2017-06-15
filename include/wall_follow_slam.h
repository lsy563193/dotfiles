#ifndef __WALL_FOLLOW_SLAM_H__
#define __WALL_FOLLOW_SLAM_H__

#include "config.h"

#include "mathematics.h"
#include "debug.h"
#include <functional>
#include <future>
#include "wall_follow_trapped.h"

uint8_t wall_follow(MapWallFollowType follow_type);
void wf_update_position(void);
void wf_check_loop_closed(uint16_t heading);
bool wf_is_reach_cleaned(void);
int8_t wf_push_point(int32_t x, int32_t y, int16_t th);
uint8_t wf_end_wall_follow(void);
uint8_t wf_break_wall_follow(void);
void WFM_move_back(uint16_t dist);
bool wf_check_isolate(void);
void wf_mark_home_point(void);
bool wf_check_angle(void);
#endif
