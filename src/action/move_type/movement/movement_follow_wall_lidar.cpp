//
// Created by lsy563193 on 12/19/17.
//
#include "pp.h"
#include "arch.hpp"

#define WF_SPEED						((int32_t) 20)
#define WF_SCAN_TYPE						(2)
Points g_wf_path{};
MovementFollowWallLidar::MovementFollowWallLidar(bool is_left) : MovementFollowWallInfrared(is_left)
{
	tmp_target_ = calcTmpTarget();
}

Point32_t MovementFollowWallLidar::calcTmpTarget()
{
	auto curr = nav_map.getCurrPoint();
	extern boost::mutex scan2_mutex_;
	PPTargetType path;
	scan2_mutex_.lock();
	if (is_sp_turn)
	{
	}
	else
	{
		if (g_wf_path.empty())
		{
			is_no_target = true;
		}
		else
		{
			if (std::abs(curr.X - tmp_target_.X) < 30 && std::abs(curr.Y - tmp_target_.Y) < 30) {
				if (g_wf_path.size() > 0) {
					g_wf_path.pop_front();
					tmp_target_ = g_wf_path.front();
				}
				ROS_ERROR("reach! g_wf_path.points.pop_front(), size = %d", g_wf_path.size());
				is_no_target = g_wf_path.empty();
			}

			is_no_target = false;
			if (tmp_target_.X != g_wf_path.front().X && tmp_target_.Y != g_wf_path.front().Y) {
				tmp_target_ = g_wf_path.front();
				ROS_INFO("tmp_target_ = g_wf_path.points.front()");
			}
		}
	}
	scan2_mutex_.unlock();
	return tmp_target_;
}

void MovementFollowWallLidar::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	ROS_INFO("MovementFollowWallLidar::adjustSpeed");
//	ROS_WARN("s_curr_p.X = %d, s_curr_p.Y = %d, temp_target.X = %d, temp_target.Y = %d",s_curr_p.X, s_curr_p.Y, temp_target.X, temp_target.Y);
	wheel.setDirectionForward();
	//ROS_INFO("1 : time_start_straight = %lf", time_start_straight);
	if (ros::Time::now().toSec() - start_timer_  < g_time_straight)
	{
		auto tmp = ros::Time::now().toSec() - start_timer_;
		if(tmp < (g_time_straight / 3)) {
			if(left_speed < 8 )
				left_speed = right_speed += 1;
			else
				left_speed = right_speed = 8;
		}
		else if(tmp < (2 * g_time_straight / 3)) {
			if(left_speed < 8)
				left_speed = right_speed = 8;
			if(left_speed < 13)
				left_speed = right_speed += 1;
			else
				left_speed = right_speed = 13;
		}
		else {
			if (left_speed < 13)
				left_speed = right_speed = 13;
			if(left_speed < 18)
				left_speed = right_speed += 1;
			else
				left_speed = right_speed = 18;
		}
	} else {
		if (is_no_target) {
			if (is_left_) {
				left_speed = 8;
				right_speed = 31;
			} else {
				left_speed = 31;
				right_speed = 8;
			}
			ROS_ERROR("not target:left_speed(%d),right_speed(%d)",left_speed, right_speed);
		} else {
			ROS_ERROR("find target:left_speed(%d),right_speed(%d)",left_speed, right_speed);
			auto curr = GridMap::getCurrPoint();
			auto angle_diff = ranged_angle(
							course_to_dest(curr.X, curr.Y, tmp_target_.X, tmp_target_.Y) - GridMap::getCurrPoint().TH);

			if (integration_cycle_++ > 10)
			{
				integration_cycle_ = 0;
				integrated_ += angle_diff;
				check_limit(integrated_, -150, 150);
			}

			int kp;
			auto tmp_cond = is_left_ ? (angle_diff < -600 || is_sp_turn) : (angle_diff > 600 || is_sp_turn);
			auto tmp_cond_1 = is_left_ ? (angle_diff > 450) : (angle_diff < -450);
			if (tmp_cond) {
				//beep_for_command(VALID);
				if (!is_sp_turn) {
					tmp_target_ = calcTmpTarget();
					ROS_INFO("fresh tmp_target_!");
				}
				auto tmp_angle_diff = ranged_angle(course_to_dest(curr.X, curr.Y, tmp_target_.X, tmp_target_.Y) - GridMap::getCurrPoint().TH);
				ROS_INFO("tmp_angle_diff = %d angle_diff = %d", tmp_angle_diff, angle_diff);
				auto tmp_cond_2 = is_left_ ? (tmp_angle_diff > -10) : (tmp_angle_diff < 10);
				is_sp_turn = !tmp_cond_2;
				kp = 40;//
				int speed_lim  = 0;
				if (base_speed_ > speed_lim) {
					base_speed_--;
				}
				/*
				left_speed = base_speed_ - angle_diff / kp - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
				right_speed = base_speed_ + angle_diff / kp + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
				*/
				if (is_left_) {
					left_speed = 10;
					right_speed = 0 - left_speed;
				} else {
					right_speed = 10;
					left_speed = 0 - right_speed;
				}
				base_speed_ = (left_speed + right_speed) / 2;
			} else if (tmp_cond_1) {
				kp = 40;
				/*
				left_speed = base_speed_ - angle_diff / kp - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
				right_speed = base_speed_ + angle_diff / kp + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
				*/
				if (is_left_) {
					left_speed = 8;
					right_speed = 31;
				} else {
					left_speed = 31;
					right_speed = 8;
				}
				base_speed_ = (left_speed + right_speed) / 2;
			} else {
				//kp = 20;
				kp = 20;//20
				if (base_speed_ < WF_SPEED) {
					base_speed_++;
				}
				if (is_left_) {
					left_speed = base_speed_ - angle_diff / kp - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
					right_speed = base_speed_ + angle_diff / kp + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
				} else {
					left_speed = base_speed_ - angle_diff / kp - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
					right_speed = base_speed_ + angle_diff / kp + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
				}
				base_speed_ = (left_speed + right_speed) / 2;
			}
		}
	}
}

