//
// Created by austin on 17-12-13.
//

#include <robot.hpp>
#include "config.h"
#include "speed_governor.hpp"
#include "dev.h"
#include "mathematics.h"

SpeedGovernorBack::SpeedGovernorBack()
{
	speed_ = BACK_MIN_SPEED;
}

void SpeedGovernorBack::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	speed_++;
	speed_ = (speed_ > BACK_MAX_SPEED) ? BACK_MAX_SPEED : speed_;
	left_speed = right_speed = speed_;
	wheel.setDirectionBackward();
}

SpeedGovernorTurn::SpeedGovernorTurn(int16_t target_angle)
{
	target_angle_ = target_angle;
}

void SpeedGovernorTurn::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{

	auto diff = getPosition().angleDiff(target_angle_);
//	ROS_INFO("SpeedGovernorTurn::adjustSpeed diff(%d),(%d,%d)", diff, target_angle_, robot::instance()->getWorldPoseYaw());
	ROS_DEBUG("%s %d: SpeedGovernorTurn diff: %f, target_angle_: %f, current angle: %f.",
			  __FUNCTION__, __LINE__, diff, target_angle_, robot::instance()->getWorldPoseYaw());

	(diff >= 0) ? wheel.setDirectionLeft() : wheel.setDirectionRight();

	if (std::abs(diff) > 200){
		speed_ += 1;
		speed_ = std::min(speed_, static_cast<int32_t>(ROTATE_TOP_SPEED));
	}
	else if (std::abs(diff) > 100){
		speed_ -= 2;
		int32_t low_speed = ROTATE_LOW_SPEED + 5;
		speed_ = std::max(speed_, low_speed);
		ROS_DEBUG("%s %d: 100 - 200, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}
	else{
		speed_ -= 2;
		speed_ = std::max(speed_, static_cast<int32_t>(ROTATE_LOW_SPEED));
		ROS_DEBUG("%s %d: 0 - 100, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}

	left_speed = right_speed = speed_;
}
