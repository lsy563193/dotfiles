//
// Created by lsy563193 on 7/7/17.
//

#include "ros/ros.h"
#include <mathematics.h>
#include <core_move.h>
#include <event_manager.h>
#include <path_planning.h>
#include <clean_state.h>
#include <pp.h>
#include "move_type.h"
#include "spot.h"
#include "clean_mode.h"

MoveType move_type;

bool mt_is_left()
{
	return move_type == MT_FOLLOW_LEFT_WALL;
}

bool mt_is_right()
{
	return move_type == MT_FOLLOW_RIGHT_WALL;
}

bool mt_is_follow_wall()
{
	return move_type == MT_FOLLOW_LEFT_WALL || move_type == MT_FOLLOW_RIGHT_WALL;
}

bool mt_is_linear()
{
	return move_type == MT_LINEARMOVE;
}

bool mt_is_go_to_charger()
{
	return move_type == MT_GO_TO_CHARGER;
}

MoveType mt_get()
{
	return move_type ;
}

void mt_update(const Cell_t& curr, PPTargetType& path) {
	//set move_type
	mt_set(MT_LINEARMOVE);
	if (cm_is_follow_wall()) {
		if(path.empty())
			mt_set(MT_FOLLOW_LEFT_WALL);
	}
	else if (cm_is_navigation()) {
		auto dir = g_old_dir;
		ROS_WARN("%s,%d: dir(%d),tilt(%d), rcon(%d)", __FUNCTION__, __LINE__, dir, ev.tilt_triggered, ev.rcon_triggered);
		if (!IS_X_AXIS(dir) || path.size() > 3 || ev.tilt_triggered || ev.rcon_triggered)
			return;

		auto delta_y = path.back().Y - curr.Y;
		if (delta_y == 0)
			return;
		//auto delta_x = path.back().X - curr.X;
		MoveType move_type_tmp = ((dir == POS_X ^ delta_y > 0) ? MT_FOLLOW_LEFT_WALL : MT_FOLLOW_RIGHT_WALL);
/*		if (std::abs(delta_x) > 1 && (IS_POS_AXIS(dir) ^ (delta_x < 0))) {
			auto is_right = (ev.obs_triggered == BLOCK_RIGHT || ev.bumper_triggered == BLOCK_RIGHT ||
											 ev.laser_triggered == BLOCK_RIGHT);
			auto is_left = (ev.obs_triggered == BLOCK_LEFT || ev.bumper_triggered == BLOCK_LEFT ||
											ev.laser_triggered == BLOCK_LEFT);
			if ((move_type_tmp == MT_FOLLOW_LEFT_WALL && is_right) || (move_type_tmp == MT_FOLLOW_RIGHT_WALL && is_left)) {
				ROS_WARN(
								"%s,%d: move_type_tmp same side with block(%d),ev.obs_triggered(%d),ev.laser_triggered(%d), ev.bumper_triggered(%d)",
								__FUNCTION__, __LINE__, move_type_tmp, ev.obs_triggered, ev.laser_triggered, ev.bumper_triggered);
				return;
			}
		}*/
		if (std::abs(delta_y) <= 2) {
			mt_set(move_type_tmp);
			ROS_INFO("\033[31m""%s,%d: target:, 2_left_3_right(%d)""\033[0m", __FUNCTION__, __LINE__, mt_get());
		}
	}
}

void mt_set(MoveType mt)
{
	move_type = mt;
	ROS_WARN("%s %d: mt is set to %d.", __FUNCTION__, __LINE__, mt_get());
}
