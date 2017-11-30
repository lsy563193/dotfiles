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

MoveType mt;

bool MoveType::is_left()
{
	return mt_ == MT_FOLLOW_LEFT_WALL;
}

bool MoveType::is_right()
{
	return mt_ == MT_FOLLOW_RIGHT_WALL;
}

bool MoveType::is_follow_wall()
{
	return mt_ == MT_FOLLOW_LEFT_WALL || mt_ == MT_FOLLOW_RIGHT_WALL;
}

bool MoveType::is_linear()
{
	return mt_ == MT_LINEARMOVE;
}

bool MoveType::is_go_to_charger()
{
	return mt_ == MT_GO_TO_CHARGER;
}

int MoveType::get()
{
	return mt_ ;
}

void MoveType::set(int mt)
{
	mt_ = mt;
	//ROS_WARN("%s %d: mt is set to %d.", __FUNCTION__, __LINE__, MoveType::get());
}

void MoveType::update(const Cell_t& curr, PPTargetType& path) {
	//set mt_
	set(MT_LINEARMOVE);
	if (cm_is_follow_wall()) {
		if(path.empty())
			set(MT_FOLLOW_LEFT_WALL);
	}
	else if (cm_is_navigation())
	{
		if (should_follow_wall(g_old_dir, curr, path))
		{
			auto delta_y = path.back().Y - curr.Y;
			int move_type_tmp = ((CostMap::isPositiveDirection(g_old_dir) && CostMap::isXDirection(g_old_dir)) ^ delta_y > 0) ? MT_FOLLOW_LEFT_WALL : MT_FOLLOW_RIGHT_WALL;
			set(move_type_tmp);
			ROS_INFO("\033[31m""%s,%d: target:, 1_left_2_right(%d)""\033[0m", __FUNCTION__, __LINE__, get());
		}
	}
}

bool MoveType::should_follow_wall(const MapDirection dir, const Cell_t& curr, PPTargetType& path)
{
	bool ret = true;
	auto delta_y = path.back().Y - curr.Y;
	ROS_INFO("%s,%d: path size(%u), dir(%d), g_check_path_in_advance(%d), bumper(%d), cliff(%d), lidar(%d), delta_y(%d)",
			 __FUNCTION__, __LINE__, path.size(), dir, g_check_path_in_advance, ev.bumper_triggered, ev.cliff_triggered, ev.lidar_triggered, delta_y);

	if (!CostMap::isXDirection(dir) // If last movement is not x axis linear movement, should not follow wall.
		|| path.size() > 2 || (!g_check_path_in_advance && !ev.bumper_triggered && !ev.cliff_triggered && !ev.lidar_triggered)
		|| delta_y == 0 || std::abs(delta_y) > 2)
		ret = false;

	return ret;
}
