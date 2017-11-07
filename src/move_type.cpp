//
// Created by lsy563193 on 7/7/17.
//

#include "ros/ros.h"
#include <mathematics.h>
#include <movement.h>
#include <core_move.h>
#include <event_manager.h>
#include <path_planning.h>
#include "move_type.h"
#include "spot.h"

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

bool mt_is_go_to_charger()
{
	return g_cm_move_type == CM_GO_TO_CHARGER;
}

CMMoveType mt_get()
{
	return g_cm_move_type ;
}

void mt_update(const Cell_t& curr, PPTargetType& path) {
	//	g_old_dir = g_new_dir;
	//set move_type
	if (g_go_home || cm_is_exploration())
		mt_set(CM_LINEARMOVE);
	else if (cm_is_navigation()) {
		mt_set(CM_LINEARMOVE);
		auto dir = curr.TH;
		ROS_WARN("%s,%d: dir(%d),obs(%d),laser(%d), bumper(%d)", __FUNCTION__, __LINE__, dir, g_obs_triggered,
						 g_laser_triggered, g_bumper_triggered);
		if (!IS_X_AXIS(dir) || (g_obs_triggered == 0 && g_laser_triggered == 0 && g_bumper_triggered == 0) ||
				g_trapped_mode == 1 || path.size() > 2)
			return;

		if (g_tilt_triggered)
			return;

		auto delta_y = path.back().Y - curr.Y;
		if (delta_y == 0) return;
		auto delta_x = path.back().X - curr.X;
		CMMoveType move_type_tmp = ((dir == POS_X ^ delta_y > 0) ? CM_FOLLOW_LEFT_WALL : CM_FOLLOW_RIGHT_WALL);
		if (std::abs(delta_x) > 1 && (IS_POS_AXIS(dir) ^ (delta_x < 0))) {
			auto is_right = (g_obs_triggered == BLOCK_RIGHT || g_bumper_triggered == BLOCK_RIGHT ||
											 g_laser_triggered == BLOCK_RIGHT);
			auto is_left = (g_obs_triggered == BLOCK_LEFT || g_bumper_triggered == BLOCK_LEFT ||
											g_laser_triggered == BLOCK_LEFT);
			if ((move_type_tmp == CM_FOLLOW_LEFT_WALL && is_right) ||
					(move_type_tmp == CM_FOLLOW_RIGHT_WALL && is_left)) {
				ROS_WARN(
								"%s,%d: move_type_tmp same side with block(%d),g_obs_triggered(%d),g_laser_triggered(%d), g_bumper_triggered(%d)",
								__FUNCTION__, __LINE__, move_type_tmp, g_obs_triggered, g_laser_triggered, g_bumper_triggered);
				return;
			}
		}

//	ROS_ERROR("%s,%d: target delta_y(%d)",__FUNCTION__,__LINE__,delta_y);
		if (std::abs(delta_y) <= 2) {
//		path.push_front(g_target_cell);
//		g_next_cell = g_target_cell;
			g_cm_move_type = move_type_tmp;
			ROS_INFO("\033[31m""%s,%d: target:, 2_left_3_right(%d)""\033[0m", __FUNCTION__, __LINE__, g_cm_move_type);
		}
	}
	if (g_trapped_mode == 1)
		mt_set(CM_FOLLOW_LEFT_WALL);
}

void mt_set(CMMoveType mt)
{
	g_cm_move_type = mt;
}
