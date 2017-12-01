//
// Created by root on 11/29/17.
//

#include "pp.h"

#define TURN_REGULATOR_WAITING_FOR_LIDAR 1
#define TURN_REGULATOR_TURNING 2

TurnMovement::TurnMovement(int16_t angle) : speed_(ROTATE_LOW_SPEED), stage_(TURN_REGULATOR_WAITING_FOR_LIDAR), waiting_finished_(false)
{
	accurate_ = ROTATE_TOP_SPEED > 30 ? 30 : 15;
	waiting_start_sec_ = ros::Time::now().toSec();
	s_target_angle = angle;
	if (mt.is_follow_wall())
	{
		wait_sec_ = 1;
		stage_ = TURN_REGULATOR_WAITING_FOR_LIDAR;
		skip_lidar_turn_angle_cnt_ = 2;
	}
	else
	{
		wait_sec_ = 0;
		stage_ = TURN_REGULATOR_TURNING;
		skip_lidar_turn_angle_cnt_ = 0;
	}
	ROS_INFO("%s %d: Init, \033[32ms_target_angle: %d\033[0m", __FUNCTION__, __LINE__, s_target_angle);
}
bool TurnMovement::isReach()
{
	if (stage_ == TURN_REGULATOR_WAITING_FOR_LIDAR)
		setTarget();
	else if (abs(ranged_angle(s_target_angle - robot::instance()->getPoseAngle())) < accurate_){

		/*********************************************For wall follow**********************************************/
		if(line_is_found)
		{
			g_wall_distance = (mt.is_left()) ? wall.getLeft() : wall.getRight();
/*			if(g_wall_distance < 10)	//set g_wall_distance in U round
			{
				g_wall_distance=last_g_wall_distance;
				ROS_ERROR("g_wall_distance: %d",g_wall_distance);
				return true;
			}*/

			ROS_INFO("%s, %d: TurnMovement target angle: \033[32m%d\033[0m, current angle: \033[32m%d\033[0m, g_wall_distance:%d."
					, __FUNCTION__, __LINE__, s_target_angle, robot::instance()->getPoseAngle(), g_wall_distance);
			if(g_wall_distance < 150)  //150 is the experience value by testing in the closest position to black wall
			{
				g_wall_distance += (150 - g_wall_distance) / 4 * 3;
				if(g_wall_distance < WALL_DISTANCE_BLACK_MIN)
					g_wall_distance = WALL_DISTANCE_BLACK_MIN;
				ROS_INFO("g_wall_distance_distance_adjust: %d",g_wall_distance);
			} else if(g_wall_distance > 250 && g_wall_distance < 320) // boardline invagination
			{
				g_wall_distance += (400 - g_wall_distance) / 5 * 4;
				ROS_INFO("boradline invagination，g_wall_distance: %d",g_wall_distance);
			} else if(g_wall_distance > 320 && g_wall_distance < 620)  //620 is the experience value by testing in the closest position to white wall
			{
				g_wall_distance += (620 - g_wall_distance) / 4 * 3;
				if(g_wall_distance < WALL_DISTANCE_WHITE_MIN)
					g_wall_distance = WALL_DISTANCE_WHITE_MIN;
				ROS_INFO("g_wall_distance_distance_adjust: %d",g_wall_distance);
			}else if(g_wall_distance > WALL_DISTANCE_WHITE_MAX)
			{
				g_wall_distance = WALL_DISTANCE_WHITE_MAX + 0.3 * (g_wall_distance - WALL_DISTANCE_WHITE_MAX);
				ROS_INFO("g_wall_distance_distance_adjust: %d",g_wall_distance);
			}
//			last_g_wall_distance=g_wall_distance;
			line_is_found = false;
		}
		else
			ROS_INFO("%s, %d: TurnMovement target angle: \033[32m%d\033[0m, current angle: \033[32m%d\033[0m, line is not found."
					, __FUNCTION__, __LINE__, s_target_angle, robot::instance()->getPoseAngle());
		time_start_straight = ros::Time::now().toSec();
		return true;
	}
		/**********************************************END**********************************************************/
	return false;
}

bool TurnMovement::shouldMoveBack()
{
	// Robot should move back for these cases.
	ev.bumper_triggered = bumper.get_status();
	ev.cliff_triggered = cliff.get_status();
	ev.tilt_triggered = gyro.getTiltCheckingStatus();

	if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || g_robot_slip)
	{
		ROS_WARN("%s, %d,TurnMovement, ev.bumper_triggered(\033[32m%d\033[0m) ev.cliff_triggered(\033[32m%d\033[0m) ev.tilt_triggered(\033[32m%d\033[0m) g_robot_slip(\033[32m%d\033[0m)."
				, __FUNCTION__, __LINE__,ev.bumper_triggered,ev.cliff_triggered,ev.tilt_triggered,g_robot_slip);
		return true;
	}

	return false;

}

void TurnMovement::setTarget()
{
	if(cs.is_going_home() && cost_map.pointToCell(s_curr_p) == g_zero_home)
	{
		s_target_angle = g_home_point.TH;
	}
	else if(LIDAR_FOLLOW_WALL && !cs.is_trapped() && mt.is_follow_wall() && skip_lidar_turn_angle_cnt_ >= 2)
	{
		// Use lidar data for generating target angle in every 3 times of turning.
		if (waiting_finished_) {
			stage_ = TURN_REGULATOR_WAITING_FOR_LIDAR;
			waiting_finished_ = false;
			waiting_start_sec_ = ros::Time::now().toSec();
			wait_sec_ = 0.33;
			s_target_angle = robot::instance()->getPoseAngle();
			ROS_INFO("%s %d: TurnMovement, start waiting for %fs.", __FUNCTION__, __LINE__, wait_sec_);
		}
		else
		{
			// Wait for specific time, for a new scan and gyro dynamic adjustment.
			double tmp_sec = ros::Time::now().toSec() - waiting_start_sec_;
			//ROS_INFO("%s %d: Has been wait for %f sec.", __FUNCTION__, __LINE__, tmp_sec);
			if (tmp_sec > wait_sec_) {
				waiting_finished_ = true;
				stage_ = TURN_REGULATOR_TURNING;
				lidar_turn_angle(g_turn_angle);
				s_target_angle = ranged_angle(robot::instance()->getPoseAngle() + g_turn_angle);
				// Reset the speed.
				speed_ = ROTATE_LOW_SPEED;
				ROS_INFO("%s %d: TurnMovement, current angle:%d, \033[33ms_target_angle: \033[32m%d\033[0m, after %fs waiting."
						, __FUNCTION__, __LINE__, robot::instance()->getPoseAngle(), s_target_angle, tmp_sec);
				skip_lidar_turn_angle_cnt_ = 0;
			}
		}
	}
	else
	{
		s_target_angle = ranged_angle(robot::instance()->getPoseAngle() + g_turn_angle);
		stage_ = TURN_REGULATOR_TURNING;
		// Reset the speed.
		speed_ = ROTATE_LOW_SPEED;
		skip_lidar_turn_angle_cnt_++;
		ROS_INFO("%s %d: TurnMovement, \033[33ms_target_angle: \033[32m%d\033[0m, skip_lidar_turn_angle_cnt_: %d.", __FUNCTION__, __LINE__, s_target_angle, skip_lidar_turn_angle_cnt_);
	}
}

void TurnMovement::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	if (stage_ == TURN_REGULATOR_WAITING_FOR_LIDAR)
	{
		l_speed = r_speed = 0;
		return;
	}

	auto diff = ranged_angle(s_target_angle - robot::instance()->getPoseAngle());
//	ROS_INFO("TurnMovement::adjustSpeed diff(%d),(%d,%d)", diff,s_target_angle, robot::instance()->getPoseAngle());
	ROS_DEBUG("%s %d: TurnMovement diff: %d, s_target_angle: %d, current angle: %d.", __FUNCTION__, __LINE__, diff, s_target_angle, robot::instance()->getPoseAngle());
	(diff >= 0) ? wheel.setDirectionLeft() : wheel.setDirectionRight();

//	ROS_INFO("TurnMovement::adjustSpeed");
	if (std::abs(diff) > 200){
		speed_ += 1;
		speed_ = std::min(speed_, ROTATE_TOP_SPEED);
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

bool TurnSpeedRegulator::adjustSpeed(int16_t diff, uint8_t& speed)
{
	if ((diff >= 0) && (diff <= 1800))
		wheel.setDirectionLeft();
	else if ((diff <= 0) && (diff >= (-1800)))
		wheel.setDirectionRight();

	tick_++;
	if (tick_ > 2)
	{
		tick_ = 0;
		if (std::abs(diff) > 350){
			speed_ = std::min(++speed_, speed_max_);
		}
		else{
			--speed_;
			speed_ = std::max(--speed_, speed_min_);
		}
	}
	speed = speed_;
	return true;
}
