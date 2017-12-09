//
// Created by lsy563193 on 11/29/17.
//
#include "pp.h"

ForwardMovement::ForwardMovement(Point32_t target, const PPTargetType& path):
				integrated_(0),base_speed_(LINEAR_MIN_SPEED),integration_cycle_(0),tick_(0),turn_speed_(4),odom_y_start(0.0),odom_x_start(0.0)
{
//	g_is_should_follow_wall = false;
//	s_target = target;
//	path_ = path;
	//ROS_INFO("%s %d: current cell(%d,%d), target cell(%d,%d) ", __FUNCTION__, __LINE__, nav_map.get_x_cell(),nav_map.get_y_cell(), count_to_cell(s_target.X), count_to_cell(s_target.Y));
}

bool ForwardMovement::isCellReach()
{
	// Checking if robot has reached target cell.
	auto curr = (GridMap::isXDirection(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = nav_map.cellToPoint(g_plan_path.back());
	auto target = (GridMap::isXDirection(g_new_dir)) ? target_p.X : target_p.Y;
	if (std::abs(s_curr_p.X - target_p.X) < CELL_COUNT_MUL_1_2 &&
		std::abs(s_curr_p.Y - target_p.Y) < CELL_COUNT_MUL_1_2)
	{
		ROS_INFO("\033[1m""%s, %d: ForwardMovement, reach the target cell (%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
						 g_plan_path.back().X, g_plan_path.back().Y);
		g_turn_angle = ranged_angle(g_new_dir - robot::instance()->getPoseAngle());
		return true;
	}

	return false;
}

bool ForwardMovement::isPoseReach()
{
	// Checking if robot has reached target cell and target angle.
	auto target_angle = g_plan_path.back().TH;
	if (isCellReach() && std::abs(ranged_angle(robot::instance()->getPoseAngle() - target_angle)) < 200)
	{
		ROS_INFO("\033[1m""%s, %d: ForwardMovement, reach the target cell and pose(%d,%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
				 g_plan_path.back().X, g_plan_path.back().Y, g_plan_path.back().TH);
		return true;
	}
	return false;
}

bool ForwardMovement::isNearTarget()
{
	auto curr = (GridMap::isXDirection(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = nav_map.cellToPoint(g_plan_path.front());
	auto &target = (GridMap::isXDirection(g_new_dir)) ? target_p.X : target_p.Y;
	//ROS_INFO("%s %d: s_curr_p(%d, %d), target_p(%d, %d), dir(%d)",
	//		 __FUNCTION__, __LINE__, s_curr_p.X, s_curr_p.Y, target_p.X, target_p.Y, new_dir_);
	if ((GridMap::isPositiveDirection(g_new_dir) && (curr > target - 1.5 * CELL_COUNT_MUL)) ||
		(!GridMap::isPositiveDirection(g_new_dir) && (curr < target + 1.5 * CELL_COUNT_MUL))) {
		if(g_plan_path.size() > 1)
		{
			// Switch to next target for smoothly turning.
			g_new_dir = static_cast<MapDirection>(g_plan_path.front().TH);
			g_plan_path.pop_front();
			ROS_INFO("%s %d: Curr(%d, %d), switch next cell(%d, %d), new dir(%d).", __FUNCTION__, __LINE__,
					 nav_map.getXCell(),
					 nav_map.getYCell(), g_plan_path.front().X, g_plan_path.front().Y, g_new_dir);
		}
		else if(g_plan_path.front() != g_zero_home && g_allow_check_path_in_advance)
		{
			g_check_path_in_advance = true;
			ROS_INFO("%s %d: Curr(%d, %d), target(%d, %d), dir(%d), g_check_path_in_advance(%d)", __FUNCTION__, __LINE__,
					 nav_map.getXCell(),
					 nav_map.getYCell(), g_plan_path.front().X, g_plan_path.front().Y, g_new_dir, g_check_path_in_advance);
			return true;
		}
	}
	return false;
}

bool ForwardMovement::shouldMoveBack()
{
	// Robot should move back for these cases.
	ev.bumper_triggered = bumper.get_status();
	ev.cliff_triggered = cliff.get_status();
	ev.tilt_triggered = gyro.getTiltCheckingStatus();

	if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || g_robot_slip)
	{
		ROS_WARN("%s, %d,ev.bumper_triggered(%d) ev.cliff_triggered(%d) ev.tilt_triggered(%d) g_robot_slip(%d)."
				, __FUNCTION__, __LINE__,ev.bumper_triggered,ev.cliff_triggered,ev.tilt_triggered,g_robot_slip);
		return true;
	}

	return false;
}

bool ForwardMovement::isRconStop()
{
	ev.rcon_triggered = c_rcon.getTrig();
	if(ev.rcon_triggered)
	{
		g_turn_angle = rcon_turn_angle();
		ROS_INFO("%s, %d: ev.rcon_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.rcon_triggered, g_turn_angle);
		return true;
	}

	return false;
}

bool ForwardMovement::isOBSStop()
{
	// Now OBS sensor is just for slowing down.
	return false;
/*
	ev.obs_triggered = obs.get_status(200, 1700, 200);
	if(ev.obs_triggered)
	{
		g_turn_angle = obs_turn_angle();
		ROS_INFO("%s, %d: ev.obs_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.obs_triggered, g_turn_angle);
		return true;
	}

	return false;*/
}

bool ForwardMovement::isLidarStop()
{
	ev.lidar_triggered = lidar_get_status();
	if (ev.lidar_triggered)
	{
		// Temporary use OBS to get angle.
		ev.obs_triggered = ev.lidar_triggered;
		g_turn_angle = obs_turn_angle();
		ROS_WARN("%s, %d: ev.lidar_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.lidar_triggered, g_turn_angle);
		return true;
	}

	return false;
}

bool ForwardMovement::isBoundaryStop()
{
	if (nav_map.isFrontBlockBoundary(2))
	{
		ROS_INFO("%s, %d: ForwardMovement, Blocked boundary.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

bool ForwardMovement::isPassTargetStop()
{
	// Checking if robot has reached target cell.
	auto curr = (GridMap::isXDirection(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = nav_map.cellToPoint(g_plan_path.back());
	auto target = (GridMap::isXDirection(g_new_dir)) ? target_p.X : target_p.Y;
	if ((GridMap::isPositiveDirection(g_new_dir) && (curr > target + CELL_COUNT_MUL / 4)) ||
		(!GridMap::isPositiveDirection(g_new_dir) && (curr < target - CELL_COUNT_MUL / 4)))
	{
		ROS_INFO("%s, %d: ForwardMovement, pass target: new_dir_(\033[32m%d\033[0m),is_x_axis(\033[32m%d\033[0m),is_pos(\033[32m%d\033[0m),curr(\033[32m%d\033[0m),target(\033[32m%d\033[0m)",
				 __FUNCTION__, __LINE__, g_new_dir, GridMap::isXDirection(g_new_dir), GridMap::isPositiveDirection(g_new_dir), curr, target);
		return true;
	}
	return false;
}

void ForwardMovement::setTarget()
{
//	g_turn_angle = ranged_angle(
//						course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - robot::instance()->getPoseAngle());
	s_target_p = nav_map.cellToPoint(g_plan_path.back());
//	path_ = g_plan_path;
}

void ForwardMovement::setBaseSpeed()
{
	base_speed_ = LINEAR_MIN_SPEED;
}

void ForwardMovement::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	ROS_WARN("%s,%d: g_path_size(%d)",__FUNCTION__, __LINE__,g_plan_path.size());
	wheel.setDirectionForward();
	auto curr = (GridMap::isXDirection(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = nav_map.cellToPoint(g_plan_path.front());
	auto &target = (GridMap::isXDirection(g_new_dir)) ? target_p.X : target_p.Y;


	int16_t angle_diff = 0;
	int16_t dis = std::min(std::abs(curr - target), (int32_t) (1.5 * CELL_COUNT_MUL));
	if (!GridMap::isPositiveDirection(g_new_dir))
		dis *= -1;
	target = curr + dis;

	angle_diff = ranged_angle(
					course_to_dest(s_curr_p.X, s_curr_p.Y, target_p.X, target_p.Y) - robot::instance()->getPoseAngle());

//	ROS_WARN("curr(%d),x?(%d),pos(%d),dis(%d), target_p(%d,%d)", curr, GridMap::isXDirection(new_dir_), GridMap::isPositiveDirection(new_dir_), dis, target_p.X, target_p.Y);
//	auto dis_diff = GridMap::isXDirection(new_dir_) ? s_curr_p.Y - s_target_p.Y : s_curr_p.X - s_target_p.X;
//	dis_diff = GridMap::isPositiveDirection(new_dir_) ^ GridMap::isXDirection(new_dir_) ? dis_diff :  -dis_diff;

	if (integration_cycle_++ > 10) {
		auto t = nav_map.pointToCell(target_p);
		robot::instance()->pubCleanMapMarkers(nav_map, g_plan_path, &t);
		integration_cycle_ = 0;
		integrated_ += angle_diff;
		check_limit(integrated_, -150, 150);
	}
	auto distance = two_points_distance(s_curr_p.X, s_curr_p.Y, target_p.X, target_p.Y);
	auto obstalce_distance_front = lidar.getObstacleDistance(0,ROBOT_RADIUS);
	uint8_t obs_state = obs.getStatus();
	bool is_decrease_blocked = decrease_map.isFrontBlocked();
	if (obs_state > 0 || (distance < SLOW_DOWN_DISTANCE) || nav_map.isFrontBlockBoundary(3) || (obstalce_distance_front < 0.25) || is_decrease_blocked)
	{
//		ROS_WARN("decelarate");
		if (distance < SLOW_DOWN_DISTANCE)
			angle_diff = 0;
		integrated_ = 0;
		if (base_speed_ > (int32_t) LINEAR_MIN_SPEED){
			base_speed_--;
			/*if(obstalce_distance_front > 0.025 && obstalce_distance_front < 0.125 && (left_speed > 20 || right_speed > 20)) {
				base_speed_ -= 2;
			}
			else if(obs_state & BLOCK_FRONT)
				base_speed_ -=2;
			else if(obs_state & (BLOCK_LEFT | BLOCK_RIGHT))
				base_speed_ --;*/
		}
	}
	else if (base_speed_ < (int32_t) LINEAR_MAX_SPEED) {
		if (tick_++ > 1) {
			tick_ = 0;
			base_speed_++;
		}
		integrated_ = 0;
	}

	left_speed =
					base_speed_ - angle_diff / 20 - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
	right_speed =
					base_speed_ + angle_diff / 20 + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
//		ROS_ERROR("left_speed(%d),right_speed(%d),angle_diff(%d), intergrated_(%d)",left_speed, right_speed, angle_diff, integrated_);

#if LINEAR_MOVE_WITH_PATH
	check_limit(left_speed, 0, LINEAR_MAX_SPEED);
	check_limit(right_speed, 0, LINEAR_MAX_SPEED);
#else
	check_limit(left_speed, LINEAR_MIN_SPEED, LINEAR_MAX_SPEED);
	check_limit(right_speed, LINEAR_MIN_SPEED, LINEAR_MAX_SPEED);
#endif
	base_speed_ = (left_speed + right_speed) / 2;
}
