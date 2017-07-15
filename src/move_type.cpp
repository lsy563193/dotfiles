//
// Created by lsy563193 on 7/7/17.
//

#include "ros/ros.h"
#include <mathematics.h>
#include <movement.h>
#include <core_move.h>
#include <event_manager.h>
#include "move_type.h"

CMMoveType g_cm_move_type;

bool mt_is_left()
{
	return g_cm_move_type == CM_FOLLOW_LEFT_WALL;
}

bool mt_is_right()
{
	return g_cm_move_type == CM_FOLLOW_RIGHT_WALL;
}

bool mt_is_follow_wall()
{
	return g_cm_move_type == CM_FOLLOW_LEFT_WALL || g_cm_move_type == CM_FOLLOW_RIGHT_WALL;
}

bool mt_is_linear()
{
	return g_cm_move_type == CM_LINEARMOVE;
}

CMMoveType mt_get()
{
	return g_cm_move_type ;
}
void mt_update(Point32_t *next_point, Point32_t target_point, uint16_t dir) {
	g_cm_move_type = CM_LINEARMOVE;
	ROS_DEBUG("mt_update, 2_left_3_right, dir,obs,bumper(%d,%d,%d)",dir,g_obs_triggered, g_bumper_triggered);
	if (!IS_X_AXIS(dir) || (g_obs_triggered == 0 && g_bumper_triggered == 0) ||next_point->Y == map_get_y_count())
		return;

	auto delta_y = count_to_cell(next_point->Y) - map_get_y_cell();

	ROS_DEBUG("mt_update, 2_left_3_right, obs,bumper(%d,%d)",g_obs_triggered, g_bumper_triggered);
	if ( delta_y != 0 && std::abs(delta_y) <= 2 ) {
		g_cm_move_type = (dir == POS_X) ^ (delta_y > 0) ? CM_FOLLOW_LEFT_WALL: CM_FOLLOW_RIGHT_WALL;
		ROS_INFO("follow wall to new line, 2_left_3_right(%d)",g_cm_move_type);
	} else if(delta_y == 0){
		if (!(count_to_cell(next_point->X) == SHRT_MAX || count_to_cell(next_point->X) == SHRT_MIN)) {
			delta_y = count_to_cell(target_point.Y) - map_get_y_cell();
			if (delta_y != 0 && std::abs(delta_y) <= 2) {
				next_point->Y = target_point.Y;
				g_cm_move_type = ((dir == POS_X ^ delta_y > 0 ) ? CM_FOLLOW_LEFT_WALL : CM_FOLLOW_RIGHT_WALL);
				ROS_INFO("follow wall to new line, 2_left_3_right(%d)",g_cm_move_type);
			}
		}
	}
}

void mt_set(CMMoveType mt)
{
	g_cm_move_type = mt;
}
