//
// Created by lsy563193 on 12/25/17.
//

#include <movement.hpp>
#include <move_type.hpp>
#include "dev.h"
#include "robot.hpp"
#include "mode.hpp"

//Point_t AMovementFollowPoint::tmp_target_{};

void AMovementFollowPoint::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	PP_INFO();
//	ROS_WARN("%s,%d: g_p_clean_mode->plan_path_size(%d)",__FUNCTION__, __LINE__,p_clean_mode->tmp_plan_path_.size());
	wheel.setDirectionForward();

	if (integration_cycle_++ > 10) {
		integration_cycle_ = 0;
		integrated_ += yaw_diff;
		check_limit(integrated_, -150, 150);
	}

//	ROS_INFO("diff(%f), angle_forward_to_turn_(%f)",yaw_diff, angle_forward_to_turn_);

		if (is_near()) {
			if (base_speed_ > (int32_t) min_speed_) {
				base_speed_--;
			}
		}
		else if (base_speed_ < (int32_t) max_speed_) {
			if (tick_++ > tick_limit_) {
				tick_ = 0;
				base_speed_++;
			}
			integrated_ = 0;
		}
		auto angle_diff = static_cast<int32_t>(yaw_diff * 180 / PI );
		auto speed_diff = angle_diff / kp_;
//		auto speed_diff = static_cast<int32_t>(yaw_diff * 180 / PI ) / kp_;
		left_speed = (base_speed_ - speed_diff - integrated_ / 150); // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
		right_speed = base_speed_ + speed_diff + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;

	check_limit(left_speed, 0, max_speed_);
	check_limit(right_speed, 0, max_speed_);
//	ROS_INFO("speed(%d,%d),base(%d),angle_diff(%d), speed_diff(%d)", left_speed, right_speed,base_speed_,angle_diff, speed_diff);

	base_speed_ = (left_speed + right_speed) / 2;

}

bool AMovementFollowPoint::isFinish() {
//	ROS_WARN("curr target th(%f,%f)", getPosition().th, calcTmpTarget().th);
	yaw_diff = getPosition().courseToDest(calcTmpTarget());
//	ROS_WARN("yaw_diff(%f)", yaw_diff);
	if(std::abs(yaw_diff) > angle_forward_to_turn_)
	{
		ROS_INFO_FL();
		ROS_WARN("yaw_diff(%f)", yaw_diff);
#if DEBUG_ENABLE
		if (std::abs(yaw_diff) > 140*PI/180) {
			ROS_ERROR_COND(DEBUG_ENABLE, "LASER WALL FOLLOW ERROR! PLEASE CALL ALVIN AND RESTART THE ROBOT.");
			while(ros::ok()){
				beeper.play_for_command(VALID);
				wheel.setPidTargetSpeed(0, 0);
			}
		}
#endif
		sp_mt_->state_turn = true;
		return true;
	}
	return false;
}
