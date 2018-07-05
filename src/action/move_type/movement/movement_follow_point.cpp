//
// Created by lsy563193 on 12/25/17.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <wheel.hpp>
#include <mode.hpp>

//Point_t AMovementFollowPoint::tmp_target_{};

void AMovementFollowPoint::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	PP_INFO();
//	ROS_WARN("%s,%d: g_p_clean_mode->plan_path_size(%d)",__FUNCTION__, __LINE__,p_clean_mode->tmp_plan_path_.size());
	wheel.setDirectionForward();
	auto angle_diff = static_cast<int32_t>(radian_to_degree(radian_diff));
	if (integration_cycle_++ > 10) {
		integration_cycle_ = 0;
		integrated_ += angle_diff;
		check_limit(integrated_, -15, 15);
	}

//	ROS_INFO("diff(%f), angle_forward_to_turn_(%f)",radian_diff, angle_forward_to_turn_);
	auto deceleration_level = isNear();
	uint8_t speed_limit;
	if (deceleration_level == 1) {
		speed_limit = min_speed_;
	} else if (deceleration_level == 2) {
		speed_limit = min_speed_ + 5;
	}
		if (deceleration_level /*|| sp_mt_->radian_diff_count < 10*/) {
				if (base_speed_ > (int32_t) speed_limit) {
					base_speed_--;
//					integrated_ = 0;
//					angle_diff = 0;
			}
//			ROS_WARN_COND(sp_mt_->sp_mode_->action_i_ == sp_mt_->sp_mode_->ac_linear, "slow down");
		}
		else if (base_speed_ < (int32_t) max_speed_) {
			if (tick_++ > tick_limit_) {
				tick_ = 0;
				base_speed_++;
			}
			integrated_ = 0;
		}
//		auto speed_diff = static_cast<int32_t>(radian_to_degree(radian_diff)) / kp_;
		left_speed = (base_speed_ - angle_diff / kp_ - integrated_ / 15); // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
		right_speed = base_speed_ + angle_diff / kp_ + integrated_ / 15; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;

	check_limit(left_speed, 0, max_speed_);
	check_limit(right_speed, 0, max_speed_);
	if (!is_out_corner) {
//		beeper.beepForCommand(INVALID);
		// todo: Does this logic belong here?? by Austin.
		if (sp_mt_->sp_mode_->action_i_ == sp_mt_->sp_mode_->ac_follow_wall_left) {
			check_limit(left_speed,0.210 * right_speed,max_speed_)
		} else if (sp_mt_->sp_mode_->action_i_ == sp_mt_->sp_mode_->ac_follow_wall_right) {
			check_limit(right_speed,0.210 * left_speed,max_speed_)
		}
	} else {
//		beeper.beepForCommand(VALID);
	}
//	ROS_INFO_COND(sp_mt_->sp_mode_->action_i_ == sp_mt_->sp_mode_->ac_linear, "speed(%d,%d),base(%d),angle_diff(%d), speed_diff(%d)", left_speed, right_speed,base_speed_,angle_diff, speed_diff);
	left_speed_ = left_speed;
	right_speed_ = right_speed;
//	ROS_INFO_COND(sp_mt_->sp_mode_->action_i_ == sp_mt_->sp_mode_->ac_linear, "speed(%d,%d),base(%d),angle_diff(%d)", left_speed, right_speed,base_speed_,angle_diff);

	base_speed_ = (left_speed + right_speed) / 2;

}

bool AMovementFollowPoint::isFinish() {
//	ROS_WARN("curr target th(%f,%f)", getPosition().th, calcTmpTarget().th);
//	ROS_WARN("radian_diff(%f)", radian_diff);
	bool is_wf = sp_mt_->sp_mode_->action_i_ == sp_mt_->sp_mode_->ac_follow_wall_left
							 || sp_mt_->sp_mode_->action_i_ == sp_mt_->sp_mode_->ac_follow_wall_right;
	bool is_left = sp_mt_->sp_mode_->action_i_ == sp_mt_->sp_mode_->ac_follow_wall_left;
	if (is_wf) {
		if (is_left){
			if (radian_diff > 0)
				return false;
		} else {
			if (radian_diff < 0 )
				return false;
		}
		if(std::abs(radian_diff) > angle_forward_to_turn_)
		{
			ROS_INFO_FL();
			ROS_WARN("radian_diff(%f)", radian_diff);
#if DEBUG_ENABLE
			if (std::abs(radian_diff) > degree_to_radian(140)) {
				ROS_ERROR_COND(DEBUG_ENABLE, "LASER WALL FOLLOW ERROR! PLEASE CALL ALVIN AND RESTART THE ROBOT.");
//			while(ros::ok()){
//				beeper.beepForCommand(VALID);
//				wheel.setPidTargetSpeed(0, 0);
//			}
			}
#endif
			sp_mt_->state_turn = true;
			return true;
		}
		return false;
	} else {
		return false;
	}
}

void AMovementFollowPoint::getLRSpeed(int32_t& left, int32_t& right) {
	left = left_speed_;
	right = right_speed_;
}
