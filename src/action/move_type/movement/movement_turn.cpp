//
// Created by lsy563193 on 11/29/17.
//

#include <movement.hpp>
#include <move_type.hpp>
#include "dev.h"
#include "robot.hpp"


MovementTurn::MovementTurn(double radian, uint8_t max_speed) : speed_(ROTATE_LOW_SPEED)
{
	accurate_ = ROTATE_TOP_SPEED > 30 ? degree_to_radian(3) : degree_to_radian(1.5);
	target_radian_ = radian;
	max_speed_ = max_speed;
	timeout_interval_ = (fabs(radian) * 107/* distance between wheel and robot center*/) / (speed_ * SPEED_ALF);
	ROS_INFO("%s, %d: MovementTurn init, target_radian_: \033[32m%f (in degree)\033[0m, current radian: \033[32m%f (in degree)\033[0m, timeout:(%f)s."
			, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(getPosition().th), timeout_interval_);
}

bool MovementTurn::isReach()
{
	if (std::abs(ranged_radian(getPosition().th - target_radian_)) < accurate_){
		ROS_INFO("%s, %d: MovementTurn finish, target_radian_: \033[32m%f (in degree)\033[0m, current radian: \033[32m%f (in degree)\033[0m."
		, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(getPosition().th));
		return true;
	}
	return false;
}

void MovementTurn::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	auto diff = ranged_radian(target_radian_ - getPosition().th);
//	ROS_INFO("%s %d: MovementTurn diff: %f, cm_target_p_.th: %f, current angle: %f.", __FUNCTION__, __LINE__, diff, target_radian_, robot::instance()->getWorldPoseRadian());
	(diff >= 0) ? wheel.setDirectionLeft() : wheel.setDirectionRight();

//	ROS_INFO("MovementTurn::adjustSpeed");
	if (std::abs(diff) > degree_to_radian(20)){
		speed_ += 1;
		speed_ = std::min(speed_, max_speed_);
		ROS_DEBUG("%s %d: angle > 20, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}
	else if (std::abs(diff) > degree_to_radian(10)){
		speed_ -= 1;
		uint8_t low_speed = static_cast<uint8_t>((ROTATE_LOW_SPEED + max_speed_) / 2);
		speed_ = std::max(speed_, low_speed);
		ROS_DEBUG("%s %d: 10 - 20, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}
	else{
		speed_ -= 1;
		speed_ = std::max(speed_, ROTATE_LOW_SPEED);
		ROS_DEBUG("%s %d: 0 - 10, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}

	l_speed = r_speed = speed_;
}

bool MovementTurn::isFinish()
{
	return isReach() || sp_mt_->shouldMoveBack();
}
