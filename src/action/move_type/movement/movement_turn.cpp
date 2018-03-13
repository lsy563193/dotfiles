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
//	auto rad_diff = getPosition().th - radian + odom.getRadian();
	target_radian_ = radian  - getPosition().th + odom.getRadian();
	max_speed_ = max_speed;
	ROS_INFO("%s, %d: MovementTurn init, target_radian_: \033[32m%f (in degree)\033[0m, current radian: \033[32m%f (in degree)\033[0m."
			, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(getPosition().th));
}

bool MovementTurn::isReach()
{
	ROS_WARN("%s, %d: MovementTurn finish, target_radian_: \033[32m%f (in degree)\033[0m, current radian: \033[32m%f (in degree)\033[0m."
	, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(odom.getRadian()));
	if (std::abs(ranged_radian(odom.getRadian() - target_radian_)) < accurate_){
		ROS_ERROR("%s, %d: MovementTurn finish, target_radian_: \033[32m%f (in degree)\033[0m, current radian: \033[32m%f (in degree)\033[0m."
		, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(odom.getRadian()));
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
	}
	else if (std::abs(diff) > degree_to_radian(10)){
		speed_ -= 1;
		uint8_t low_speed = (ROTATE_LOW_SPEED + ROTATE_TOP_SPEED) / 2;
		speed_ = std::max(speed_, low_speed);
		ROS_DEBUG("%s %d: 100 - 200, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}
	else{
		speed_ -= 1;
		speed_ = std::max(speed_, ROTATE_LOW_SPEED);
		ROS_DEBUG("%s %d: 0 - 100, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}

	l_speed = r_speed = speed_;
}

bool MovementTurn::isFinish()
{
	return isReach() || sp_mt_->shouldMoveBack();
}
