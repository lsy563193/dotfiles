//
// Created by lsy563193 on 11/29/17.
//

#include <movement.hpp>
#include <move_type.hpp>
#include "dev.h"
#include "robot.hpp"

MovementTurn::MovementTurn(int16_t angle, uint8_t max_speed) : speed_(ROTATE_LOW_SPEED)
{
	accurate_ = ROTATE_TOP_SPEED > 30 ? 30 : 15;
	target_angle_ = angle;
	max_speed_ = max_speed;
	ROS_INFO("%s %d: Init, \033[32mtarget_angle_: %d\033[0m", __FUNCTION__, __LINE__, angle);
}

bool MovementTurn::isReach()
{
	if (abs(getPosition().angleDiff(target_angle_)) < accurate_){
			ROS_INFO("%s, %d: MovementTurn finish, target_angle_: \033[32m%d\033[0m, current angle: \033[32m%d\033[0m."
					, __FUNCTION__, __LINE__, target_angle_, robot::instance()->getWorldPoseAngle());
		return true;
	}
	return false;
}

void MovementTurn::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	auto diff = getPosition().angleDiff(target_angle_);
//	ROS_DEBUG("%s %d: MovementTurn diff: %d, cm_target_p_.th: %d, current angle: %d.", __FUNCTION__, __LINE__, diff, target_angle_ robot::instance()->getWorldPoseAngle());
	(diff >= 0) ? wheel.setDirectionLeft() : wheel.setDirectionRight();

//	ROS_INFO("MovementTurn::adjustSpeed");
	if (std::abs(diff) > 200){
		speed_ += 1;
		speed_ = std::min(speed_, max_speed_);
	}
	else if (std::abs(diff) > 100){
		speed_ -= 2;
		uint8_t low_speed = ROTATE_LOW_SPEED + 5;
		speed_ = std::max(speed_, low_speed);
		ROS_DEBUG("%s %d: 100 - 200, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}
	else{
		speed_ -= 2;
		speed_ = std::max(speed_, ROTATE_LOW_SPEED);
		ROS_DEBUG("%s %d: 0 - 100, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}

	l_speed = r_speed = speed_;

}

bool MovementTurn::isFinish()
{
	return isReach() || sp_mt_->shouldMoveBack();
}
