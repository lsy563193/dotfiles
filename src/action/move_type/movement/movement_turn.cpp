//
// Created by lsy563193 on 11/29/17.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <event_manager.h>
#include <mode.hpp>
#include "dev.h"
#include "robot.hpp"


MovementTurn::MovementTurn(double radian, uint8_t max_speed) : speed_(ROTATE_LOW_SPEED)
{
//	auto rad_diff = getPosition().th - radian + odom.getRadian();
	target_radian_ = ranged_radian(radian  - getPosition().th + odom.getRadian());
	max_speed_ = max_speed;
	accurate_ = max_speed_ > ROTATE_TOP_SPEED ? degree_to_radian(3) : degree_to_radian(1);
	auto diff = ranged_radian(target_radian_ - odom.getRadian());
	auto isUseTimeOut = (sp_mt_->sp_mode_->mode_i_ != sp_mt_->sp_mode_->md_go_to_charger) && (sp_mt_->sp_mode_->mode_i_ != sp_mt_->sp_mode_->md_remote);
	timeout_interval_ = isUseTimeOut ? 5 : 100;
	ROS_INFO("%s, %d: MovementTurn init, target_radian_: \033[32m%.1f (in degree)\033[0m, current radian: \033[32m%.1f (in degree)\033[0m, timeout:(%.2f)s."
			, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(getPosition().th), timeout_interval_);
}

bool MovementTurn::isReach()
{
//	ROS_WARN("%s, %d: MovementTurn finish, target_radian_: \033[32m%f (in degree)\033[0m, current radian: \033[32m%f (in degree)\033[0m."
//	, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(odom.getRadian()));
	if (std::abs(ranged_radian(odom.getRadian() - target_radian_)) < accurate_){
		ROS_INFO("%s, %d: MovementTurn finish, target_radian_: \033[32m%f (in degree)\033[0m, current radian: \033[32m%f (in degree)\033[0m."
		, __FUNCTION__, __LINE__, radian_to_degree(ranged_radian(target_radian_)), radian_to_degree(odom.getRadian()));
		return true;
	}
	if(isTimeUp()){
		ROS_WARN("%s %d: Robot maybe slip but not detect in checkSlip", __FUNCTION__, __LINE__);
		ev.robot_slip = true;
		return true;
	}
	return false;
}

void MovementTurn::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
//	auto diff = ranged_radian(target_radian_ - getPosition().th);
	auto diff = ranged_radian(target_radian_ - odom.getRadian());
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
	return isReach() || sp_mt_->isFinishForward();
}
