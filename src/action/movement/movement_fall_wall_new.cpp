//
// Created by lsy563193 on 12/5/17.
//

//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "arch.hpp"

double robot_to_wall_distance = 0.8;

MovementFollowWall::MovementFollowWall(bool is_left) : previous_(0), seen_charger_counter(0), is_left_(is_left)
{
		fw_map.reset(CLEAN_MAP);
}

bool MovementFollowWall::isClosure(uint8_t closure_cnt)
{
	if (g_wf_reach_count >= closure_cnt) {
		ROS_WARN("%s %d: Trapped wall follow is loop closed. reach_count(%d) ", __FUNCTION__, __LINE__, g_wf_reach_count);
		return true;
	}
	return false;
}

bool MovementFollowWall::isIsolate()
{
	return false;
}

bool MovementFollowWall::isTimeUp()
{
	if (fw_is_time_up()) {
		ROS_WARN("%s %d: curr(%d),start(%d),diff(%d)",__FUNCTION__, __LINE__, time(NULL), g_wf_start_timer, g_wf_diff_timer);
		ev.fatal_quit = true;
		ROS_INFO("%s %d: curr(%d),start(%d),diff(%d)",__FUNCTION__, __LINE__, time(NULL), g_wf_start_timer, g_wf_diff_timer);
		ev.fatal_quit = true;
		return true;
	}

	return false;
}

bool MovementFollowWall::shouldMoveBack()
{
	ev.bumper_triggered = bumper.get_status();
	if (ev.bumper_triggered) {
		g_turn_angle = bumper_turn_angle();
		ROS_WARN("%s %d: Bumper triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
	ev.cliff_triggered = cliff.get_status();
	if (ev.cliff_triggered) {
		g_turn_angle = cliff_turn_angle();
		ROS_WARN("%s %d: Cliff triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
	ev.tilt_triggered = gyro.getTiltCheckingStatus();
	if (ev.tilt_triggered) {
		g_turn_angle = tilt_turn_angle();
		ROS_WARN("%s %d: Tilt triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	if(g_robot_slip)
	{
		// Temporary use obs as lidar triggered.
		ev.obs_triggered = BLOCK_FRONT;
		g_turn_angle = obs_turn_angle();
		ROS_WARN("%s %d: slip triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	return false;
}

bool MovementFollowWall::shouldTurn()
{
	ev.lidar_triggered = lidar_get_status();
	if (ev.lidar_triggered)
	{
		// Temporary use bumper as lidar triggered.
		ev.bumper_triggered = ev.lidar_triggered;
		g_turn_angle = bumper_turn_angle();
		ev.bumper_triggered = 0;
		ROS_WARN("%s %d: Lidar triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	ev.obs_triggered = (obs.getFront() > obs.getFrontTrigValue() + 1700);
	if (ev.obs_triggered)
	{
		ev.obs_triggered = BLOCK_FRONT;
		g_turn_angle = obs_turn_angle();
		ROS_WARN("%s %d: OBS triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	return false;
}

bool MovementFollowWall::isBlockCleared()
{
	if (!nav_map.isBlockAccessible(nav_map.getXCell(), nav_map.getYCell())) // Robot has step on blocks.
	{
		ROS_WARN("%s %d: Lidar triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	return false;
}

void MovementFollowWall::setTarget()
{
	// No need to set target here, it is set in path_next().
}

void MovementFollowWall::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	ROS_DEBUG("%s %d: MovementFollowWall.", __FUNCTION__, __LINE__);
	wheel.setDirectionForward();
//	uint32_t same_dist = (wheel.get_right_step() / 100) * 11 ;
	uint32_t rcon_status = 0;
	auto _l_step = wheel.getLeftStep();
	auto _r_step = wheel.getRightStep();
	auto &same_dist = (is_left_) ? _l_step : _r_step;
	auto &diff_dist = (is_left_) ? _r_step : _l_step;
	auto &same_speed = (is_left_) ? l_speed : r_speed;
	auto &diff_speed = (is_left_) ? r_speed : l_speed;
	wall_buffer[2]=wall_buffer[1];
	wall_buffer[1]=wall_buffer[0];
	wall_buffer[0]=(is_left_) ? wall.getLeft() : wall.getRight();

	rcon_status = c_rcon.getStatus();
	/*---only use a part of the Rcon signal---*/
	rcon_status &= (RconFL2_HomeT|RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT);
	if(rcon_status)
	{
//		ev.rcon_triggered = c_rcon.get_trig();
//		nav_map.set_rcon();
		int32_t linear_speed = 24;
		/* angular speed notes						*
		 * larger than 0 means move away from wall	*
		 * less than 0 means move close to wall		*/
		int32_t angular_speed = 0;

		seen_charger_counter = 30;
		if(rcon_status & (RconFR_HomeT|RconFL_HomeT))
		{
			angular_speed = 12;
		}
		else if(rcon_status & RconFR2_HomeT)
		{
			if(is_left_)
				angular_speed = 15;
			else
				angular_speed = 10;
		}
		else if(rcon_status & RconFL2_HomeT)
		{
			if(is_left_)
				angular_speed = 10;
			else
				angular_speed = 15;
		}
		c_rcon.resetStatus();
		/*---check if should aloud the charger---*/
		if(seen_charger_counter)
		{
			same_speed = linear_speed + angular_speed;
			diff_speed = linear_speed - angular_speed;
			same_speed_ = same_speed;
			diff_speed_ = diff_speed;
			return ;
		}
	}
	if(seen_charger_counter)
	{
		seen_charger_counter--;
		same_speed = same_speed_;
		diff_speed = diff_speed_;
		return ;
	}

	if (ros::Time::now().toSec() - time_start_straight < g_time_straight)
	{
		auto tmp = ros::Time::now().toSec() - time_start_straight;
		if(tmp < (g_time_straight / 3)) {
			if(same_speed < 8 )
				same_speed = diff_speed += 1;
			else
				same_speed = diff_speed = 8;
		}
		else if(tmp < (2 * g_time_straight / 3)) {
			if(same_speed < 8)
				same_speed = diff_speed = 8;
			if(same_speed < 13)
				same_speed = diff_speed += 1;
			else
				same_speed = diff_speed = 13;
		}
		else {
			if (same_speed < 13)
				same_speed = diff_speed = 13;
			if(same_speed < 18)
			same_speed = diff_speed += 1;
			else
				same_speed = diff_speed = 18;
		}
	}
	else
	{
		auto wheel_speed_base = 17 + diff_dist / 150;
		if (wheel_speed_base > 28)wheel_speed_base = 28;

		auto adc_value = (is_left_) ? wall.getLeft() : wall.getRight();

		auto proportion = (adc_value - g_wall_distance) * 100 / g_wall_distance;

		auto delta = proportion - previous_;

		previous_ = proportion;
		if (robot_to_wall_distance > 0.8 || abs(adc_value - g_wall_distance) > 150 )
		{//over left
			same_speed = wheel_speed_base + proportion / 7 + delta/2; //
			diff_speed = wheel_speed_base - proportion / 7 - delta/2; //

			if (wheel_speed_base < 26)
			{
				reset_sp_turn_count();
					if (diff_speed > wheel_speed_base + 6)
				{
					diff_speed = 32;
					same_speed = 6;
//				ROS_INFO("Wf_2, same_speed = %d, diff_speed = %d", same_speed, diff_speed);
				}
				else if (same_speed > wheel_speed_base + 10)
				{
					diff_speed = 6;
					same_speed = 28;
//					ROS_INFO("Wf_3, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
				}
			}
			else
			{
				if (diff_speed > 35)
				{
					add_sp_turn_count();
					diff_speed = 34;
					same_speed = 6;
//					ROS_INFO("Wf_4, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
//					ROS_INFO("get_sp_turn_count() = %d", get_sp_turn_count());
				}
				else
				{
					reset_sp_turn_count();
				}
			}
		}
		else
		{
			same_speed = wheel_speed_base + proportion / 11 + delta/2;//16
			diff_speed = wheel_speed_base - proportion / 11 - delta/2; //11
//			ROS_INFO("Wf_4.1, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
			if (wheel_speed_base < 26)
			{
				reset_sp_turn_count();
				if (diff_speed > wheel_speed_base + 4)
				{
					diff_speed = 32;
					same_speed = 5;
//					ROS_INFO("Wf_5, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
				}
			}
			else
			{
				if (diff_speed > 32)
				{
					add_sp_turn_count();
					diff_speed = 35;
					same_speed = 6;
//					ROS_INFO("Wf_6, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
//					ROS_INFO("g_sp_turn_count() = %d",get_sp_turn_count());
				}
				else
				{
					reset_sp_turn_count();
				}
			}
		}

		/****************************************************turn a right angular***************************************************/
		if (wall_buffer[0] < 100)
		{
			if(g_wall_distance > 250)
				turn_right_angle_factor = 50;				//white
			else
				turn_right_angle_factor = 13;				//black

			if ((wall_buffer[1] - wall_buffer[0]) >= g_wall_distance / turn_right_angle_factor &&
					(wall_buffer[2] - wall_buffer[1]) >= g_wall_distance / turn_right_angle_factor &&
					same_dist > 200 &&
					(diff_speed-same_speed) >= -3) {
				is_right_angle = true;
			}
		}

		if(is_right_angle)
		{
			if(time_right_angle == 0) {
				time_right_angle = ros::Time::now().toSec();
				ROS_WARN("%s,%d: delay_sec(0.44) to walk straight", __FUNCTION__, __LINE__);
			}
			if(obs.frontTriggered() || nav_map.isFrontBlockBoundary(3) ) {
				if(ros::Time::now().toSec() - time_right_angle < 0.4) {
					same_speed = 2 * 300 * (wall_follow_detect_distance - 0.167) + (20 - 15) / 2;
					diff_speed = 2 * 300 * (wall_follow_detect_distance - 0.167) - (20 - 15) / 2;
					return;
				}else {
					time_right_angle = 0;
					is_right_angle = 0;
				}
			}else {
				if(ros::Time::now().toSec() - time_right_angle < 0.44) {
					same_speed = 15;
					diff_speed = 15;
					return;
				}else{
					time_right_angle = 0;
					is_right_angle = 0;
					ROS_INFO("reset time_right_angle,is_right_angle");
				}
			}
		}
		/****************************************************END**************************************************************/
//		ROS_ERROR("same_speed:%d,diff_speed:%d",same_speed,diff_speed);

		if (same_speed > 39)same_speed = 39;
		if (same_speed < 0)same_speed = 0;
		if (diff_speed > 35)diff_speed = 35;
		if (diff_speed < 5)diff_speed = 5;

		if (obs.frontTriggered() || nav_map.isFrontBlockBoundary(3)) {
//			ROS_WARN("decelarate");
			old_same_speed = same_speed;
			old_diff_speed = diff_speed;
			if (next_linear_speed > (300 * (wall_follow_detect_distance - 0.167))){
				if(next_linear_speed == INT_MAX)
					next_linear_speed = (old_same_speed + old_diff_speed) / 2 - 1;
				same_speed = (2 * next_linear_speed + next_linear_speed * (old_same_speed - old_diff_speed) / (old_same_speed + old_diff_speed)) / 2;
				diff_speed = (2 * next_linear_speed - next_linear_speed * (old_same_speed - old_diff_speed) / (old_same_speed + old_diff_speed)) / 2;
				next_linear_speed = (same_speed + diff_speed) / 2 - 1;
//			ROS_ERROR("decelerate:same_speed:%d,diff_speed:%d,next_linear_speed:%d",same_speed,diff_speed,next_linear_speed);
			} else{
				//the first parameter 300 must below 638 to ensure the linear velocity below to the calculating linear velocity
				same_speed = (2 * (300 * (wall_follow_detect_distance - 0.167)) + old_same_speed - old_diff_speed) / 2;
				diff_speed = (2 * (300 * (wall_follow_detect_distance - 0.167)) + old_diff_speed - old_same_speed) / 2;
//				ROS_ERROR("continue:same_speed:%d,diff_speed:%d,linear_speed:%d",same_speed,diff_speed,(same_speed + diff_speed) / 2);
			}
			if(same_speed < 0) {
				diff_speed -= same_speed;
				same_speed = 0;
//			ROS_ERROR("below zero by same_speed:same_speed:%d,diff_speed:%d",same_speed,diff_speed);
			}
			else if(diff_speed < 0)
			{
				same_speed -= diff_speed;
				diff_speed = 0;
//			ROS_ERROR("below zero by diff_speed:same_speed:%d,diff_speed:%d",same_speed,diff_speed);
			}
		} else{
			next_linear_speed = INT_MAX;
		}

		if(same_speed > diff_speed && diff_speed < (0.210 * same_speed)) {
			diff_speed = 0.210 * same_speed;
		}if(same_speed < diff_speed && same_speed < (0.210 * diff_speed)) {
			same_speed = 0.210 * diff_speed;
		}
	}
}

bool MovementFollowWall::sp_turn_over(const Cell_t &curr) {
		ROS_INFO("  %s %d:?? curr(%d,%d,%d)", __FUNCTION__, __LINE__, curr.X, curr.Y, curr.TH);
		/*check if spot turn*/
		if (get_sp_turn_count() > 400) {
			reset_sp_turn_count();
			ROS_WARN("  yes! sp_turn over 400");
			return true;
		}
		return false;
	}

bool MovementFollowWall::isFinish() {
	return sp_cm_->MovementFollowWallisFinish();
//	return isNewLineReach() || /*isClosure(1) ||*/ shouldMoveBack() || shouldTurn()
//					|| isBlockCleared() || isOverOriginLine();
}
