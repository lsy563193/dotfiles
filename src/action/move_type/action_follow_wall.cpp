//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"

//bool ActionFollowWall::isFinish() {
//	return false;
//}

bool ActionFollowWall::is_left_ = true;

ActionFollowWall::ActionFollowWall(bool is_left) {

	is_left_ = is_left;
	auto cur = GridMap::getCurrPoint();
	auto tar = nav_map.cellToPoint(plan_path_.back());
	//			ROS_INFO("%s,%d: mt_is_fw",__FUNCTION__, __LINE__);
	if (LIDAR_FOLLOW_WALL)
		if (!lidar_turn_angle(g_turn_angle))
			g_turn_angle = ranged_angle(course_to_dest(cur.X, cur.Y, tar.X, tar.Y) -
																	robot::instance()->getPoseAngle());
	turn_target_angle_ = ranged_angle(robot::instance()->getPoseAngle() + g_turn_angle);
	ROS_INFO("g_turn_angle(%d)cur(%d,%d,%d),tar(%d,%d,%d)", g_turn_angle, cur.X, cur.Y, cur.TH, tar.X, tar.Y, tar.TH);

//	if (action_i_ == ac_back) {
//		PP_INFO();
//		g_time_straight = 0.2;
//		g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
//	}
//	else if (action_i_ == ac_turn) {
//		PP_INFO();
//		g_time_straight = 0;
//		g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
//	}
}

//IAction *ActionFollowWall::setNextAction() {
//	return nullptr;
//}

