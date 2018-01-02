//
// Created by lsy563193 on 12/5/17.
//

#include "pp.h"
#include "arch.hpp"



// Back distance for go to charger regulator
bool g_go_to_charger_back_30cm = false;
bool g_go_to_charger_back_10cm = false;
bool g_go_to_charger_back_0cm = false;

MovementBack::MovementBack(float back_distance, uint8_t max_speed)
{
	back_distance_ = back_distance;
	max_speed_ = max_speed;
	speed_ = max_speed_;
	bumper_jam_cnt_ = 0;
	cliff_jam_cnt_ = 0;
	updateStartPose();
	ROS_INFO("%s %d: Set back distance: %f.", __FUNCTION__, __LINE__, back_distance_);
}

void MovementBack::updateStartPose()
{
	s_pos_x = odom.getX();
	s_pos_y = odom.getY();
}

bool MovementBack::isLidarStop()
{
	auto obstacle_distance = lidar.getObstacleDistance(1, ROBOT_RADIUS);
	if (back_distance_ >= 0.05 && obstacle_distance < 0.03)
	{
		ROS_WARN("%s, %d: obstacle_distance:%f.", __FUNCTION__, __LINE__, obstacle_distance);
		return true;
	}

	return false;
}

void MovementBack::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
//	ROS_INFO("MovementBack::adjustSpeed");
	wheel.setDirectionBackward();
	speed_ = (speed_ > max_speed_) ? max_speed_ : speed_;
	wheel.resetStep();
	l_speed = r_speed = speed_;
}

bool MovementBack::isFinish()
{
	float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
	ROS_DEBUG("%s, %d: MovementBack distance %f", __FUNCTION__, __LINE__, distance);

	if(fabsf(distance) >= back_distance_)
	{

		bumper_jam_cnt_ = bumper.getStatus() == 0 ? 0 : bumper_jam_cnt_+1 ;
		cliff_jam_cnt_ = cliff.getStatus() == 0 ? 0 : cliff_jam_cnt_+1 ;
		ev.tilt_triggered = gyro.getTiltCheckingStatus();
		//g_lidar_bumper_cnt = robot::instance()->getLidarBumper() == 0? 0:g_lidar_bumper_cnt+1;

		if (bumper_jam_cnt_ == 0 && cliff_jam_cnt_ == 0 && !ev.tilt_triggered)
		{
			ROS_INFO("%s, %d: MovementBack reach target.", __FUNCTION__, __LINE__);
			return true;
		}
		if (cliff_jam_cnt_ >= 2)
		{
			ev.cliff_jam = true;
			ROS_WARN("%s, %d: Cliff jam.", __FUNCTION__, __LINE__);
			return true;
		}
		else if (bumper_jam_cnt_ >= 2)
		{
			ev.bumper_jam = true;
			ROS_WARN("%s, %d: Bumper jam.", __FUNCTION__, __LINE__);
			return true;
		}
		//else if (g_lidar_bumper_cnt >= 2)
		//{
		//	g_lidar_bumper_jam = true;
		//	return false;
		//}
		else
			updateStartPose();
	}
	return false;
}
