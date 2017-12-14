//
// Created by lsy563193 on 12/5/17.
//

#include "pp.h"
#include "arch.hpp"


float g_back_distance = 0.01;

// Back distance for go to charger regulator
bool g_go_to_charger_back_30cm = false;
bool g_go_to_charger_back_10cm = false;
bool g_go_to_charger_back_0cm = false;

MovementBack::MovementBack(float back_distance) : counter_(0), speed_(BACK_MAX_SPEED)
{
//	ROS_INFO("%s, %d: ", __FUNCTION__, __LINE__);
	back_distance_ = back_distance;
	bumper_jam_cnt_ = 0;
	cliff_jam_cnt_ = 0;
	updateStartPose();
	ROS_INFO("%s %d: Set back distance: %f.", __FUNCTION__, __LINE__, back_distance_);
}

void MovementBack::updateStartPose()
{
	s_pos_x = odom.getX();
	s_pos_y = odom.getY();
	if (g_robot_slip){
		back_distance_ = 0.30;
		g_slip_backward= true;
		g_robot_slip = false;
	}

}

bool MovementBack::isReach()
{
	float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
	ROS_DEBUG("%s, %d: MovementBack distance %f", __FUNCTION__, __LINE__, distance);
	/*---------slip detect------*/
	if(g_robot_slip && g_slip_cnt >= 2){
		g_robot_slip = false;
		g_slip_backward = false;
		ROS_WARN("%s,%d,\033[1mrobot slip %d times!!\033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
		beeper.play_for_command(false);
		return true;
	}
	if(fabsf(distance) >= back_distance_)
	{
		if(g_slip_backward){
			ROS_WARN("%s,%d,\033[1mrobot slip backward reach!! distance(%f),back_distance(%f)\033[0m",__FUNCTION__,__LINE__,distance,back_distance_);
			g_slip_backward= false;
			return true;
		}

		bumper_jam_cnt_ = bumper.get_status() == 0 ? 0 : bumper_jam_cnt_+1 ;
		cliff_jam_cnt_ = cliff.get_status() == 0 ? 0 : cliff_jam_cnt_+1 ;
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
	wheel.setDirBackward();
	if (!cm_is_follow_wall())
	{
		speed_ += ++counter_;
		speed_ = (speed_ > BACK_MAX_SPEED) ? BACK_MAX_SPEED : speed_;
	}
	wheel.resetStep();
	/*if (fabsf(distance) >= back_distance_ * 0.8)
	{
		l_speed = r_speed = speed_--;
		check_limit(l_speed, BACK_MIN_SPEED, BACK_MAX_SPEED);
		check_limit(r_speed, BACK_MIN_SPEED, BACK_MAX_SPEED);
	}
	else*/
		l_speed = r_speed = speed_;
}

bool MovementBack::isFinish() {
	return isReach();
}
