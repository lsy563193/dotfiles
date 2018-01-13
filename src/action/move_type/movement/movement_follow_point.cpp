//
// Created by lsy563193 on 12/25/17.
//
#include "pp.h"
#include "arch.hpp"

//Point32_t AMovementFollowPoint::tmp_target_{};

void AMovementFollowPoint::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	PP_INFO();
//	ROS_WARN("%s,%d: g_p_clean_mode->plan_path_size(%d)",__FUNCTION__, __LINE__,p_clean_mode->tmp_plan_path_.size());
	wheel.setDirectionForward();

	if (integration_cycle_++ > 10) {
		integration_cycle_ = 0;
		integrated_ += angle_diff;
		check_limit(integrated_, -150, 150);
	}

	ROS_INFO("diff(%d), angle_forward_to_turn_(%d)",angle_diff, angle_forward_to_turn_);

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

		left_speed = base_speed_ - angle_diff / kp_ -
								 integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
		right_speed = base_speed_ + angle_diff / kp_ +
									integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;

	check_limit(left_speed, 0, max_speed_);
	check_limit(right_speed, 0, max_speed_);
//	ROS_INFO("speed(%d,%d)", left_speed, right_speed);

	base_speed_ = (left_speed + right_speed) / 2;

}

bool AMovementFollowPoint::isFinish() {
	angle_diff = getPosition().angleDiff(calcTmpTarget());
	INFO_BLUE("angle_diff");
	ROS_INFO("angle_diff(%d)", angle_diff);
	if(std::abs(angle_diff) > angle_forward_to_turn_)
	{
		INFO_RED("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
		sp_mt_->state_turn = true;
		return true;
	}
	return false;
}

AMovementFollowPoint::AMovementFollowPoint() {

}
