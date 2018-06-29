//
// Created by lsy563193 on 11/29/17.
//

#include <event_manager.h>
#include <movement.hpp>
#include <move_type.hpp>
#include <wheel.hpp>
#include <cliff.h>
#include "robot.hpp"


MovementTurn::MovementTurn(double slam_target, uint8_t max_speed) : speed_(ROTATE_LOW_SPEED)
{
//	auto rad_diff = getPosition().th - slam_target + odom.getRadian();
	is_left_cliff_trigger_in_start_ = cliff.getLeft();
	is_right_cliff_trigger_in_start_ = cliff.getRight();
	turn_radian_ = fabs(ranged_radian(slam_target - getPosition().th));
	target_radian_ = ranged_radian(slam_target  - getPosition().th + odom.getRadian());//odom_target = slam_target-slam_start + odom_start
	max_speed_ = max_speed;
	accurate_ = max_speed_ > ROTATE_TOP_SPEED ? degree_to_radian(3) : degree_to_radian(1);
	timeout_interval_ = 10;
	ROS_WARN("%s, %d: target_radian_: %.2lf (in degree), current radian: %.2lf (in degree), timeout:(%.2f)s."/*\ntarget_radian_(%.2lf) = ranged_radian(slam_target(%.2lf)  - getPosition().th(%.2lf) + odom.getRadian(%.2lf))"*/,
			 __FUNCTION__, __LINE__, radian_to_degree(target_radian_), radian_to_degree(odom.getRadian()),
			 timeout_interval_/*, radian_to_degree(target_radian_), radian_to_degree(slam_target),
			 radian_to_degree(getPosition().th), radian_to_degree(odom.getRadian())*/);
}

MovementTurn::~MovementTurn() {
	gyro.resetCheckRobotSlipByGyro();
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool MovementTurn::isReach()
{
//	ROS_WARN("%s, %d: MovementTurn finish, target_radian_: \033[32m%f (in degree)\033[0m, current radian: \033[32m%f (in degree)\033[0m."
//	, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(odom.getRadian()));
	if (std::abs(ranged_radian(odom.getRadian() - target_radian_)) < accurate_){
		ROS_WARN("%s, %d: MovementTurn, target_radian_: \033[32m%.2f (in degree)\033[0m, current radian: \033[32m%.2f (in degree)\033[0m."
		, __FUNCTION__, __LINE__, radian_to_degree(target_radian_), radian_to_degree(odom.getRadian()));
		return true;
	}

	return false;
}

void MovementTurn::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
//	auto diff = ranged_radian(target_radian_ - getPosition().th);
	auto diff = ranged_radian(target_radian_ - odom.getRadian());
//	ROS_INFO("%s %d: MovementTurn diff: %f, cm_target_p_.th: %f, current angle: %f.", __FUNCTION__, __LINE__, diff, target_radian_, robot::instance()->getWorldPoseRadian());
	auto angle_diff = radian_to_degree(diff);
	(diff >= 0) ? wheel.setDirectionLeft() : wheel.setDirectionRight();

//	ROS_INFO("MovementTurn::adjustSpeed");
	auto turn_angle =radian_to_degree(turn_radian_);
//	ROS_WARN("turn_angle(%lf)", turn_angle);
	if (turn_angle > 40) {
		if (std::abs(diff) > degree_to_radian(40)){
			speed_ += 1;
			speed_ = std::min(speed_, max_speed_);
//			ROS_INFO("%s %d: angle > 20, speed = %d angle_diff = %lf.", __FUNCTION__, __LINE__, speed_, radian_to_degree(diff));
		}
		else if (std::abs(diff) > degree_to_radian(8)){
			speed_ = static_cast<uint8_t>(fabs(radian_to_degree(diff) / 2));//2
//		speed_ = static_cast<uint8_t>(fabs(angle_diff * angle_diff / 45));
//		speed_ = static_cast<uint8_t>(fabs(sqrt(angle_diff -1) * 4));
			speed_ = std::min(max_speed_, speed_);
			speed_ = std::max(speed_, (uint8_t)8);
//			ROS_INFO("%s %d: 10 - 20, speed = %d angle_diff = %lf.", __FUNCTION__, __LINE__, speed_, radian_to_degree(diff));
		}
		else
			speed_ = 5;
	} else {
		if (std::abs(diff) > degree_to_radian(20)){
			speed_ += 1;
			speed_ = std::min(speed_, max_speed_);
//			ROS_INFO("%s %d: angle > 20, speed = %d angle_diff = %lf.", __FUNCTION__, __LINE__, speed_, radian_to_degree(diff));
		}
		else if (std::abs(diff) > degree_to_radian(10)){
#if 1
		speed_ -= 1;
		uint8_t low_speed = static_cast<uint8_t>((ROTATE_LOW_SPEED + max_speed_) / 2);
//		ROS_INFO("low_speed = %d", low_speed);
		speed_ = std::max(speed_, low_speed);
#endif
		}
		else{
			speed_ -= 1;
			speed_ = std::max(speed_, ROTATE_LOW_SPEED);
//			ROS_INFO("%s %d: 0 - 10, speed = %d angle_diff = %lf.", __FUNCTION__, __LINE__, speed_, radian_to_degree(diff));
		}

	}

	if(is_left_cliff_trigger_ || is_right_cliff_trigger_)
		speed_ = 0;

	l_speed = r_speed = speed_;
}

bool MovementTurn::isFinish()
{
	//For cliff turn
	checkCliffTurn();

	if(is_left_cliff_trigger_ || is_right_cliff_trigger_)
	{
		wheel.stop();
		return false;
	}

	// Check slip by gyro
	gyro.checkRobotSlipByGyro();

	auto ret = isReach() || sp_mt_->isFinishForward() || ev.cliff_turn;

	if(isTimeUp()){
		ROS_WARN("%s %d: Robot maybe slip but not detect in checkSlip, curr_degree(%lf)", __FUNCTION__, __LINE__,radian_to_degree(getPosition().th));
		ev.slip_triggered = true;
		ret = true;
	}

	if (ret) {
		wheel.stop();
	}
	return ret;
}

void MovementTurn::checkCliffTurn()
{
	//For left
	if (!is_left_cliff_trigger_)
	{
		if (!is_left_cliff_trigger_in_start_ && cliff.getLeft() && wheel.getDirection() == DIRECTION_LEFT)
		{
			ROS_WARN("%s,%d: Cliff left!", __FUNCTION__, __LINE__);
			is_left_cliff_trigger_ = true;
			left_cliff_trigger_start_time_ = ros::Time::now().toSec();
		}
	}
	else
	{
		if (cliff.getLeft())
		{
			if (ros::Time::now().toSec() - left_cliff_trigger_start_time_ > 0.2)
			{
				ROS_WARN("%s,%d: Cliff turn left", __FUNCTION__, __LINE__);
				ev.cliff_turn |= BLOCK_CLIFF_TURN_LEFT;
			}
		} else
		{
			is_left_cliff_trigger_ = false;
			left_cliff_trigger_start_time_ = 0;
		}
	}

	//For right
	if (!is_right_cliff_trigger_)
	{
		if (!is_right_cliff_trigger_in_start_ && cliff.getRight() && wheel.getDirection() == DIRECTION_RIGHT)
		{
			ROS_WARN("%s,%d: Cliff right!", __FUNCTION__, __LINE__);
			is_right_cliff_trigger_ = true;
			right_cliff_trigger_start_time_ = ros::Time::now().toSec();
		}
	}
	else
	{
		if (cliff.getRight())
		{
			if (ros::Time::now().toSec() - right_cliff_trigger_start_time_ > 0.2)
			{
				ROS_WARN("%s,%d: Cliff turn right", __FUNCTION__, __LINE__);
				ev.cliff_turn |= BLOCK_CLIFF_TURN_RIGHT;
			}
		} else
		{
			is_right_cliff_trigger_ = false;
			right_cliff_trigger_start_time_ = 0;
		}
	}
}
