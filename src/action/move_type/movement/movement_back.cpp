//
// Created by lsy563193 on 12/5/17.
//

#include <event_manager.h>
#include <movement.hpp>
#include <robot.hpp>
#include <wheel.hpp>
#include <cliff.h>
#include <bumper.h>
#include <lidar.hpp>
#include <gyro.h>

MovementBack::MovementBack(float back_distance, uint8_t max_speed)
{
	back_distance_ = back_distance;
	max_speed_ = max_speed;
	speed_ = max_speed_;
	bumper_jam_cnt_ = 0;
	lidar_bumper_jam_cnt_ = 0;
	cliff_jam_cnt_ = 0;
	robot_stuck_cnt_ = 0;
	tilt_cnt_ = 0;
	updateStartPose();
	ROS_WARN("%s %d: Distance: %.2f.", __FUNCTION__, __LINE__, back_distance_);
}

MovementBack::~MovementBack() {
	robot::instance()->unlockScanCtrl();
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

void MovementBack::updateStartPose()
{
	s_pos_x = odom.getOriginX();
	s_pos_y = odom.getOriginY();
}
//
//bool MovementBack::isLidarStop()
//{
//	auto obstacle_distance = lidar.getObstacleDistance(1, ROBOT_RADIUS);
//	if (back_distance_ >= 0.05 && obstacle_distance < 0.03)
//	{
//		ROS_WARN("%s, %d: obstacle_distance:%f.", __FUNCTION__, __LINE__, obstacle_distance);
//		return true;
//	}
//
//	return false;
//}

void MovementBack::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
//	ROS_INFO("MovementBack::adjustSpeed");
	wheel.setDirectionBackward();
	speed_ = (speed_ > max_speed_) ? max_speed_ : speed_;
	cliff_status_ |= cliff.getStatus();
//	ROS_INFO("%s %d: cliff_status:%x.", __FUNCTION__, __LINE__, cliff_status_);
	if ((cliff_status_ & BLOCK_LEFT) && ((cliff_status_ & BLOCK_RIGHT) == 0))
	{
//		ROS_INFO("%s %d: Left cliff triggered while right cliff is alright.", __FUNCTION__, __LINE__);
		l_speed = speed_;
		r_speed = speed_ / 2;
	}
	else if ((cliff_status_ & BLOCK_RIGHT) && ((cliff_status_ & BLOCK_LEFT) == 0))
	{
//		ROS_INFO("%s %d: Right cliff triggered while left cliff is alright.", __FUNCTION__, __LINE__);
		l_speed = speed_ / 2;
		r_speed = speed_;
	}
	else
		l_speed = r_speed = speed_;

	//For cliff turn
	if(is_left_cliff_triggered_ || is_right_cliff_triggered_)
		l_speed = r_speed = 0;
}

bool MovementBack::isFinish()
{
	//Check cliff turn
	checkCliffTurn();
	if(is_left_cliff_triggered_ || is_right_cliff_triggered_)
	{
		wheel.stop();
		return false;
	}

	robot::instance()->lockScanCtrl();
	robot::instance()->pubScanCtrl(true, true);
	bool ret{false};
	float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
//	ROS_INFO("%s, %d: MovementBack distance %f", __FUNCTION__, __LINE__, distance);

	if (std::abs(distance) >= back_distance_ || isLidarStop()) {
		auto tmp_bumper_status = bumper.getStatus();
		if (tmp_bumper_status == BLOCK_ALL || tmp_bumper_status == BLOCK_LEFT || tmp_bumper_status == BLOCK_RIGHT) {
			bumper_jam_cnt_++;
		} else if (tmp_bumper_status != BLOCK_ALL && tmp_bumper_status != BLOCK_LEFT && tmp_bumper_status != BLOCK_RIGHT) {
			bumper_jam_cnt_ = 0;
		}
		if (tmp_bumper_status == BLOCK_LIDAR_BUMPER) {
			lidar_bumper_jam_cnt_++;
		} else if (tmp_bumper_status != BLOCK_LIDAR_BUMPER) {
			lidar_bumper_jam_cnt_ = 0;
		}
		cliff_jam_cnt_ = static_cast<uint8_t>(cliff.getStatus() == 0 ? 0 : cliff_jam_cnt_ + 1);
		robot_stuck_cnt_ = static_cast<uint8_t>(robot::instance()->isRobotSlip() == 0 ? 0 : robot_stuck_cnt_ + 1);
//		ROS_INFO("tilt_cnt_(%d)", tilt_cnt_);
		tilt_cnt_ = static_cast<uint8_t>(gyro.getTiltCheckingStatus() == 0 ? 0 : tilt_cnt_ + 1);
//		ROS_INFO("tilt_cnt_2(%d)", tilt_cnt_);
		//g_lidar_bumper_cnt = robot::instance()->getLidarBumper() == 0? 0:g_lidar_bumper_cnt+1;

		ROS_WARN("%s, %d: MovementBack reach target, bumper_jam_cnt_(%d), lidar_bumper_jam_cnt_(%d), cliff_jam_cnt_(%d), robot_stuck_cnt_(%d), tilt_cnt_(%d).",
						 __FUNCTION__, __LINE__, bumper_jam_cnt_, lidar_bumper_jam_cnt_, cliff_jam_cnt_, robot_stuck_cnt_,
						 tilt_cnt_);
		if (bumper_jam_cnt_ == 0 && lidar_bumper_jam_cnt_ == 0 && cliff_jam_cnt_ == 0 && robot_stuck_cnt_ == 0 &&
				tilt_cnt_ == 0)
			ret = true;
//			return true;
		else if (cliff_jam_cnt_ >= 2) {
			ev.cliff_jam = true;
			ROS_WARN("%s, %d: Cliff jam.", __FUNCTION__, __LINE__);
			ret = false;
//			return false;
		} else if (bumper_jam_cnt_ >= 2) {
			ev.bumper_jam = true;
			ROS_WARN("%s, %d: Bumper jam.", __FUNCTION__, __LINE__);
			ret = false;
//			return false;
		} else if (lidar_bumper_jam_cnt_ >= 2) {
			ev.lidar_bumper_jam = true;
			ROS_WARN("%s, %d: Lidar Bumper jam.", __FUNCTION__, __LINE__);
			ret = false;
//			return false;
		} else if (robot_stuck_cnt_ >= 2) {
			ev.robot_stuck = true;
			ROS_WARN("%s, %d: Robot stuck.", __FUNCTION__, __LINE__);
			ret = false;
//			return false;
		} else if (tilt_cnt_ >= 2) {
			ev.tilt_jam = true;
			ROS_WARN("%s, %d: Tilt jam.", __FUNCTION__, __LINE__);
			ret = false;
//			return false;
		} else {
			updateStartPose();
//			ret = false;
			ROS_INFO("%s %d: Move back again.", __FUNCTION__, __LINE__);
		}
	} else {
		ret = false;
	}
	if (ret) {
		wheel.stop();
	}
	return ret;
}

bool MovementBack::isLidarStop()
{
	if (bumper.getStatus() || cliff.getStatus() || robot::instance()->isRobotSlip())
		return false;

	if (lidar.getObstacleDistance(1, 0.15) < 0.02)
	{
		ROS_WARN("%s %d: Stop by lidar.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

void MovementBack::checkCliffTurn()
{
	if (wheel.getLeftWheelActualSpeed() < 0 && wheel.getRightWheelActualSpeed() < 0)
	{
		if (!has_mark_original_cliff_)
		{
			original_left_cliff_triggered_ = cliff.getLeft();
			original_right_cliff_triggered_ = cliff.getRight();
			has_mark_original_cliff_ = true;
		}
	}
	else
		return;

	if (!original_left_cliff_triggered_ && !original_right_cliff_triggered_)
	{
		//For left
		if (!is_left_cliff_triggered_)
		{
			if (cliff.getLeft())
			{
				ROS_WARN("%s,%d: Cliff left!", __FUNCTION__, __LINE__);
				is_left_cliff_triggered_ = true;
				left_cliff_triggered_start_time_ = ros::Time::now().toSec();
			}
		}
		else
		{
			if (cliff.getLeft())
			{
				if (ros::Time::now().toSec() - left_cliff_triggered_start_time_ > 0.2)
				{
					ROS_WARN("%s,%d: Cliff turn left", __FUNCTION__, __LINE__);
					ev.cliff_turn |= BLOCK_CLIFF_TURN_LEFT;
				}
			}
			else
			{
				is_left_cliff_triggered_ = false;
				left_cliff_triggered_start_time_ = 0;
			}
		}

		//For right
		if (!is_right_cliff_triggered_)
		{
			if (!original_right_cliff_triggered_ && cliff.getRight())
			{
				ROS_WARN("%s,%d: Cliff right!", __FUNCTION__, __LINE__);
				is_right_cliff_triggered_ = true;
				right_cliff_triggered_start_time_ = ros::Time::now().toSec();
			}
		}
		else
		{
			if (cliff.getRight())
			{
				if (ros::Time::now().toSec() - right_cliff_triggered_start_time_ > 0.2)
				{
					ROS_WARN("%s,%d: Cliff turn right", __FUNCTION__, __LINE__);
					ev.cliff_turn |= BLOCK_CLIFF_TURN_RIGHT;
				}
			}
			else
			{
				is_right_cliff_triggered_ = false;
				right_cliff_triggered_start_time_ = 0;
			}
		}
	}

}
