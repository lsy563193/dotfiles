//
// Created by lsy563193 on 12/5/17.
//

#include <event_manager.h>
#include <movement.hpp>
#include <move_type.hpp>

void MovementRcon::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
//	ROS_INFO("rcon_status = %x ev.rcon_status = %x", rcon_status, ev.rcon_status);
	rcon_status = ev.rcon_status;
//	ROS_INFO("rcon_status = %x", rcon_status);
	/*---only use a part of the Rcon signal---*/
	rcon_status &= (RconFL2_HomeT|RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT);
//	ROS_INFO("rcon_status = %x", rcon_status);
	if(rcon_status) {
		seen_charger_counter_ = 80;
//		g_rcon_triggered = get_rcon_trig();
//		map_set_rcon();
		int32_t linear_speed = 20;
		/* angular speed notes						*
		 * larger than 0 means move away from wall	*
		 * less than 0 means move close to wall		*/
		int32_t angular_speed = 0;

		if (rcon_status & (RconFR_HomeT | RconFL_HomeT)) {
			angular_speed = 12;
		}
		else if (rcon_status & RconFR2_HomeT) {
			if (is_left_)
				angular_speed = 15;
			else
				angular_speed = 10;
		}
		else if (rcon_status & RconFL2_HomeT) {
			if (is_left_)
				angular_speed = 10;
			else
				angular_speed = 15;
		}
		if (seen_charger_counter_) {
			l_speed = is_left_ ? linear_speed + angular_speed : linear_speed - angular_speed;
			r_speed = is_left_ ? linear_speed - angular_speed : linear_speed + angular_speed;
			left_speed_ = l_speed;
			right_speed_ = r_speed;
//			ROS_INFO("speed(%d, %d)", l_speed, r_speed);
			return;
		} else {
			l_speed = left_speed_;
			r_speed = right_speed_;
//			ROS_WARN("speed(%d, %d)", l_speed, r_speed);
			return;
		}
	}
	if (seen_charger_counter_ > 0){
		seen_charger_counter_--;
		l_speed = left_speed_;
		r_speed = right_speed_;
//		ROS_WARN("speed(%d, %d)", l_speed, r_speed);
	}
//	ROS_INFO("seen_charger_counter_ = %d", seen_charger_counter_);
}

bool MovementRcon::isFinish() {
	sp_mt_->RconTrigger();
	if(rcon_status == 0 && seen_charger_counter_ == 0)
		rcon_disappear_count++;
	else
		rcon_disappear_count = 0;
	if(rcon_disappear_count >= 3)
		ROS_INFO("%s,%d rcon_disappear_count:%d",__FUNCTION__,__LINE__,rcon_disappear_count);
	return rcon_disappear_count >= 3 || sp_mt_->isFinishForward();
}

MovementRcon::MovementRcon(bool is_left) {
	ROS_WARN("%s %d: Enter.", __FUNCTION__, __LINE__);
	is_left_ = is_left;
	rcon_status = ev.rcon_status;
//	ROS_INFO("ev.rcon_status = %x", ev.rcon_status);
}

MovementRcon::~MovementRcon() {
//	ev.rcon_status = 0;
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}
