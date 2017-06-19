#ifndef __WALL_FOLLOW_SLAM_H__
#define __WALL_FOLLOW_SLAM_H__

#include "config.h"

#include "mathematics.h"
#include "debug.h"
#include <functional>
#include <future>
#include "wall_follow_trapped.h"

uint8_t wall_follow(MapWallFollowType follow_type);
void wf_check_loop_closed(uint16_t heading);
void wf_update_position(void);
bool is_start_cell(void);
bool is_new_cell();

/*
 * Check whether the robot is trapped or not. The robot is trapped if there
 * is no path to (0, 0) or home.
 *
 * @param
 *
 * @return	0 if the robot is trapped
 * 		1 if the robot is not trapped.
 */
int16_t WF_path_escape_trapped(void);
void wf_turn_right(uint16_t speed, int16_t angle);

int8_t wf_push_point(int32_t x, int32_t y, int16_t th);
bool wf_is_reach_cleaned(void);
uint8_t wf_end_wall_follow(void);
uint8_t wf_break_wall_follow(void);
void WFM_move_back(uint16_t dist);
bool is_check_isolate(void);
void wf_mark_home_point(void);

/**************************************************************
Function:WF_Check_Check_Angle
Description:
 *It mainly for checking whether the angle is same when wall
 *follow end. When it check is loop closed, and is not isolated,
 *it will check whether the angle of last 10 poses in WF_Point
 *is same as the other same pose in the WF_Point except the last
 *10 point. It can prevent from the case that the robot is in the
 *narrow and long space when wall follow, and it will be checked
 *as loop closed.
 ***************************************************************/
bool wf_check_angle(void);
#endif
