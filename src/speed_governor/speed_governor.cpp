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

	auto diff = getPosition().th - target_angle_;
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

//void SpeedGovernorRcon::adjustSpeed(int32_t &left_speed, int32_t &right_speed) {
//	auto rcon_status = c_rcon.getAll();
//	/*---only use a part of the Rcon signal---*/
//	rcon_status &= (RconFL2_HomeT|RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT);
//	if(rcon_status)
//	{
////		g_rcon_triggered = get_rcon_trig();
////		map_set_rcon();
//		int32_t linear_speed = 24;
//		/* angular speed notes						*
//		 * larger than 0 means move away from wall	*
//		 * less than 0 means move close to wall		*/
//		int32_t angular_speed = 0;
//
//		seen_charger_counter_ = 30;
//		if(rcon_status & (RconFR_HomeT|RconFL_HomeT))
//		{
//			angular_speed = 12;
//		}
//		else if(rcon_status & RconFR2_HomeT)
//		{
//			if(is_left_)
//				angular_speed = 15;
//			else
//				angular_speed = 10;
//		}
//		else if(rcon_status & RconFL2_HomeT)
//		{
//			if(is_left_)
//				angular_speed = 10;
//			else
//				angular_speed = 15;
//		}
//		/*---check if should eloud the charger---*/
//		if(seen_charger_counter_)
//		{
////			same_speed = linear_speed + angular_speed;
////			diff_speed = linear_speed - angular_speed;
//			left_speed = is_left_ ? linear_speed + angular_speed : linear_speed - angular_speed;
//			right_speed = is_left_ ? linear_speed - angular_speed : linear_speed + angular_speed;
//			left_speed_ = left_speed;
//			right_speed_ = right_speed;
//			return ;
//		}
//	}
//	if(seen_charger_counter_)
//	{
//		seen_charger_counter_--;
//		left_speed = left_speed_;
//		right_speed = right_speed_;
//		return ;
//	}
//}

