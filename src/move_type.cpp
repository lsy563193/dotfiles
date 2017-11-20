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

void mt_set(MoveType mt)
{
	move_type = mt;
	//ROS_WARN("%s %d: mt is set to %d.", __FUNCTION__, __LINE__, mt_get());
}

void mt_update(const Cell_t& curr, PPTargetType& path) {
	//set move_type
	mt_set(MT_LINEARMOVE);
	if (cm_is_follow_wall()) {
		if(path.empty())
			mt_set(MT_FOLLOW_LEFT_WALL);
	}
	else if (cm_is_navigation())
	{
		if (mt_should_follow_wall(g_old_dir, curr, path))
		{
			auto delta_y = path.back().Y - curr.Y;
			MoveType move_type_tmp = (g_old_dir == POS_X ^ delta_y > 0) ? MT_FOLLOW_LEFT_WALL : MT_FOLLOW_RIGHT_WALL;
			mt_set(move_type_tmp);
			ROS_INFO("\033[31m""%s,%d: target:, 1_left_2_right(%d)""\033[0m", __FUNCTION__, __LINE__, mt_get());
		}
	}
}

bool mt_should_follow_wall(const int16_t dir, const Cell_t& curr, PPTargetType& path)
{
	bool ret = true;
	auto delta_y = path.back().Y - curr.Y;
	ROS_INFO("%s,%d: path size(%u), dir(%d), g_check_path_in_advance(%d), bumper(%d), cliff(%d), laser(%d), delta_y(%d)",
			 __FUNCTION__, __LINE__, path.size(), dir, g_check_path_in_advance, ev.bumper_triggered, ev.cliff_triggered, ev.laser_triggered, delta_y);

	if (!IS_X_AXIS(dir) // If last movement is not x axis linear movement, should not follow wall.
		|| path.size() > 2 || (!g_check_path_in_advance && !ev.bumper_triggered && !ev.cliff_triggered && !ev.laser_triggered)
		|| delta_y == 0 || std::abs(delta_y) > 2)
		ret = false;

	return ret;
}
