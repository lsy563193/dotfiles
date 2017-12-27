//
// Created by lsy563193 on 12/5/17.
//

#include "pp.h"
#include <arch.hpp>
#include "wheel.hpp"
IMoveType* IMovement::sp_mt_  = nullptr;

//Point32_t IMovement::s_target_p = {0,0};
//Point32_t IMovement::s_start_p = {0,0};

float IMovement::s_pos_x = 0;
float IMovement::s_pos_y = 0;
//CellPath IMovement::path_ = {};

void IMovement::run() {
//	PP_INFO();
	int32_t l_speed{},r_speed{};
	adjustSpeed(l_speed,r_speed);
	wheel.setPidTargetSpeed(l_speed, r_speed);
}

bool IMovement::is_near()
{
	auto curr_p = GridMap::getCurrPoint();
	bool is_decrease_blocked = decrease_map.isFrontBlocked();
//	auto distance = two_points_distance(curr_p.X, curr_p.Y, s_target_p.X, s_target_p.Y);
	auto obstacle_distance_front = lidar.getObstacleDistance(0,ROBOT_RADIUS);
	return obs.getStatus() > 0 || /*(distance < SLOW_DOWN_DISTANCE) ||*/ nav_map.isFrontBlockBoundary(3) || (obstacle_distance_front < 0.25) || is_decrease_blocked;
}
