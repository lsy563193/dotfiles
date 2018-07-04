//
// Created by lsy563193 on 11/29/17.
//

#include <event_manager.h>
#include <movement.hpp>
#include <error.h>
#include <robot.hpp>
#include <wheel.hpp>
#include <bumper.h>
#include <cliff.h>
#include <brush.h>
#include <vacuum.h>
#include <water_tank.hpp>
#include <lidar.hpp>
#include <gyro.h>
#include <beeper.h>

double MovementExceptionResume::stuck_start_turn_time_ = 0;
bool MovementExceptionResume::is_stuck_last_turn_right_ = false;
MovementExceptionResume::MovementExceptionResume(int last_action)
{
	ROS_WARN("%s %d: Enter.", __FUNCTION__, __LINE__);

	last_action_i_ = last_action;
	// Save current position for moving back detection.
	s_pos_x = odom.getOriginX();
	s_pos_y = odom.getOriginY();

	//For slip
	if(ros::Time::now().toSec() - stuck_start_turn_time_ < 3){
		if(last_action_i_ == sp_mt_->sp_mode_->ac_follow_wall_left)
			robot_stuck_flag_ = 1;
		else if(last_action_i_ == sp_mt_->sp_mode_->ac_follow_wall_right)
			robot_stuck_flag_ = 2;
		else
			robot_stuck_flag_ = static_cast<uint8_t>(is_stuck_last_turn_right_ ? 2 : 1);
		stuck_start_turn_time_ = ros::Time::now().toSec();
	}else{
		robot_stuck_flag_ = 0;
		stuck_start_turn_time_ = 0;
	}

	//For wheel cliff
	if(sp_mt_->sp_mode_->is_wheel_cliff_triggered)
	{
		is_up_tilt_ = gyro.getAngleR() > 4;
		wheel_cliff_back_distance_ = is_up_tilt_ ? 0.05 : 0.02;
		ROS_INFO("%s,%d wheel_cliff_back_distance(%lf)",__FUNCTION__,__LINE__,wheel_cliff_back_distance_);
	}

	resetResumeTime();
}

MovementExceptionResume::~MovementExceptionResume()
{
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

void MovementExceptionResume::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	ROS_INFO("self rescuing!");


	if (ev.oc_wheel_left || ev.oc_wheel_right)
	{
		if (ev.oc_wheel_right)
			wheel.setDirectionRight();
		else
			wheel.setDirectionLeft();
		left_speed = 30;
		right_speed = 30;
	}
	else if (ev.cliff_jam || ev.cliff_all_triggered)
	{
		wheel.setDirectionBackward();
		left_speed = right_speed = BACK_MAX_SPEED;
	}
	else if (sp_mt_->sp_mode_->is_wheel_cliff_triggered)//new wheel cliff rescue
	{
		switch (wheel_cliff_state_)
		{
			case 1:
			case 2:
			case 3:
			{
				// Quickly move back for a distance.
				wheel.setDirectionBackward();
				left_speed = right_speed = BACK_MAX_SPEED;
				break;
			}
			case 4:
			{
				// Quickly move back for a distance.
				wheel.setDirectionBackward();
				left_speed = right_speed = BACK_MAX_SPEED;
				break;
			}
			case 5:
			{
				// Quickly turn right for 90 degrees.
				wheel.setDirectionRight();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 6:
			{
				// Quickly turn left for 180 degrees.
				wheel.setDirectionLeft();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;

			}
		}
	}
	else if (ev.cliff_turn)
	{
		switch (ev.cliff_turn)
		{
			case BLOCK_CLIFF_TURN_LEFT:
			{ //turn should right
				wheel.setDirectionForward();
				left_speed = RUN_TOP_SPEED * 2 / 3;
				right_speed = RUN_TOP_SPEED / 8;
				break;
			}
			case BLOCK_CLIFF_TURN_RIGHT:
			{ //should turn left
				wheel.setDirectionForward();
				left_speed = RUN_TOP_SPEED / 8;
				right_speed = RUN_TOP_SPEED * 2 / 3;
				break;
			}
			case BLOCK_CLIFF_TURN_ALL:
			{
				wheel.setDirectionForward();
				left_speed = right_speed = RUN_TOP_SPEED / 4;
				break;
			}
		}
//		ROS_INFO("%s %d: left(%d), right(%d)", __FUNCTION__, __LINE__, left_speed, right_speed);
	}
	else if (ev.bumper_jam)
	{
		switch (bumper_jam_state_)
		{
			case 1:
			case 2:
			case 3:
			{
				// Quickly move back for a distance.
				wheel.setDirectionBackward();
				left_speed = right_speed = BACK_MAX_SPEED;
				break;
			}
			case 4:
			{
				// Quickly turn right for 90 degrees.
				wheel.setDirectionRight();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 5:
			{
				// Quickly turn left for 180 degrees.
				wheel.setDirectionLeft();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
		}
	}
	else if (ev.lidar_bumper_jam)
	{
		switch (lidar_bumper_jam_state_)
		{
			case 1:
			case 2:
			case 3:
			{
				// Quickly move back for a distance.
				wheel.setDirectionBackward();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 4:
			{
				// Quickly turn right for 90 degrees.
				wheel.setDirectionRight();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 5:
			{
				// Quickly turn left for 180 degrees.
				wheel.setDirectionLeft();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
		}
	}
	else if (ev.tilt_jam)
	{
		switch (tilt_jam_state_)
		{
			case 1:
			case 2:
			case 3:
			{
				// Quickly move back for a distance.
				wheel.setDirectionBackward();
				left_speed = right_speed = BACK_MAX_SPEED;
				break;
			}
			case 4:
			{
				// Quickly turn right for 90 degrees.
				wheel.setDirectionRight();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 5:
			{
				// Quickly turn left for 180 degrees.
				wheel.setDirectionLeft();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
		}
	}
	else if (ev.oc_vacuum)
		left_speed = right_speed = 0;
	else if (ev.oc_brush_main)
	{
		if (main_brush_resume_state_ == 1)
		{
			wheel.setDirectionBackward();
			left_speed = right_speed = BACK_MAX_SPEED;
		}
		else
			left_speed = right_speed = 0;
	}
	else if (ev.lidar_stuck)
	{
//		ROS_INFO("lidar stucking");
		if (lidar_resume_cnt_ >= 5)
		{
			wheel.stop();
		}
		else
		{
			wheel.setDirectionBackward();
			left_speed = right_speed = BACK_MAX_SPEED;
		}
	}
	else if (ev.robot_stuck)
	{
//		ROS_INFO("slipping");
		switch (robot_stuck_flag_)
		{
			case 0:
			{
				wheel.setDirectionBackward();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 1:
			{
				wheel.setDirectionRight();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 2:
			{
				wheel.setDirectionLeft();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
		}
	}
	else if (ev.gyro_error)
	{
		left_speed = right_speed = 0;
	}
//	ROS_INFO("speed(%d, %d)!", left_speed, right_speed);}
}

bool MovementExceptionResume::isFinish() {
	updatePosition();
//	printf("%s %d: cliff left(%d), front(%d), right(%d).\n", __FUNCTION__, __LINE__, cliff.getLeft(),
//		   cliff.getFront(), cliff.getRight());
	if (!(ev.bumper_jam || ev.lidar_bumper_jam || ev.cliff_jam || ev.tilt_jam || ev.cliff_all_triggered ||
				ev.oc_wheel_left || ev.oc_wheel_right
				|| ev.oc_vacuum || ev.lidar_stuck || ev.robot_stuck || ev.oc_brush_main || ev.gyro_error
				|| sp_mt_->sp_mode_->is_wheel_cliff_triggered || ev.cliff_turn)) {
		ROS_WARN("%s %d: All exception cleared.", __FUNCTION__, __LINE__);
		return true;
	}

	if (ev.oc_wheel_left || ev.oc_wheel_right)
	{// Check for right wheel.
		/*--- init start time ---*/
		if (resume_wheel_start_time_ == 0)
			resume_wheel_start_time_ = ros::Time::now().toSec();
		if (brush.isSideBrushOn())
			brush.stop();
		vacuum.stop();
		water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);
		if (ros::Time::now().toSec() - resume_wheel_start_time_ >= 2) {
			if (ev.oc_wheel_left) {
				ROS_WARN("%s %d: Left wheel resume succeeded.", __FUNCTION__, __LINE__);
				ev.oc_wheel_left = false;
			} else {
				ROS_WARN("%s %d: Right wheel resume succeeded.", __FUNCTION__, __LINE__);
				ev.oc_wheel_right = false;
			}
			return true;
		} else if (wheel.getLeftWheelOc() || wheel.getRightWheelOc()) {
			if (++wheel_resume_cnt_ >= 30) {
				wheel.stop();
				brush.stop();
				vacuum.stop();
				if (ev.oc_wheel_left) {
					ROS_ERROR("%s,%d Left wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
					robot_error.set(ERROR_CODE_LEFTWHEEL);
				} else {
					ROS_ERROR("%s,%d Right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
					robot_error.set(ERROR_CODE_RIGHTWHEEL);
				}
				ev.fatal_quit = true;
				return true;
			} else {
				ROS_INFO("%s %d: Failed to resume for %d times.", __FUNCTION__, __LINE__, wheel_resume_cnt_);
			}
		}
	}
	else if (ev.cliff_all_triggered)
	{
		if (wheel.getRightWheelCliffStatus() && wheel.getLeftWheelCliffStatus()) {
			ev.fatal_quit = true;
			ROS_WARN("%s %d: All cliff and wheel cliff  triggered.", __FUNCTION__, __LINE__);
		}
		if (cliff.getStatus() != BLOCK_ALL) {
			ROS_WARN("%s %d: Cliff all resume succeeded.", __FUNCTION__, __LINE__);
			ev.cliff_all_triggered = false;
			ev.cliff_triggered = 0;
			g_cliff_cnt = 0;
			return true;
		} else if (cliff_all_resume_cnt_ < 2) {
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
			if (std::abs(distance) > 0.02f) {
				wheel.stop();
				cliff_all_resume_cnt_++;
				if (cliff_all_resume_cnt_ <= 2)
					ROS_INFO("%s %d: Resume failed, try cliff all resume for the %d time.",
									 __FUNCTION__, __LINE__, cliff_all_resume_cnt_);
				s_pos_x = odom.getOriginX();
				s_pos_y = odom.getOriginY();
			}
		} else {
			ROS_ERROR("%s %d: Cliff all triggered.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
		}
	}
	else if (ev.cliff_jam)
	{
		if (!cliff.getStatus()) {
			ROS_WARN("%s %d: Cliff resume succeeded.", __FUNCTION__, __LINE__);
			ev.cliff_jam = false;
			ev.cliff_triggered = 0;
			g_cliff_cnt = 0;
			return true;
		} else if (cliff_resume_cnt_ < 5) {
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
			if (std::abs(distance) > 0.01f) {
				wheel.stop();
				cliff_resume_cnt_++;
				if (cliff_resume_cnt_ <= 3)
					ROS_INFO("%s %d: Resume failed, try cliff resume for the %d time.",
									 __FUNCTION__, __LINE__, cliff_resume_cnt_);
				s_pos_x = odom.getOriginX();
				s_pos_y = odom.getOriginY();
			}
		} else {
			ROS_ERROR("%s %d: Cliff jamed.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
			robot_error.set(ERROR_CODE_CLIFF);
		}
	}
		//new cliff rescue
	else if (sp_mt_->sp_mode_->is_wheel_cliff_triggered)
	{
		bool right_wheel_and_cliff{false};
		bool left_wheel_and_cliff{false};

		left_wheel_and_cliff = cliff.getLeft() & ev.left_wheel_cliff;
		right_wheel_and_cliff = cliff.getRight() & ev.right_wheel_cliff;

		//bumper temporary
		if (!wheel.getLeftWheelCliffStatus() &&
				!wheel.getRightWheelCliffStatus())//for checking wheel cliff is still tirggered
		{
			ROS_WARN("%s %d: wheel cliff resume succeeded.", __FUNCTION__, __LINE__);
			sp_mt_->sp_mode_->is_wheel_cliff_triggered = false;
			ev.right_wheel_cliff = false;
			ev.left_wheel_cliff = false;
			return true;
		} else {
			switch (wheel_cliff_state_) {
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				case 3: // Move back for the third time.
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
//					ROS_INFO("distance(%lf), pos(%lf, %lf)", distance, s_pos_x, s_pos_y);
					if (std::abs(distance) > wheel_cliff_back_distance_) {
						wheel.stop();
						// If cliff jam during bumper self resume.
//						if (cliff.getStatus() && ++g_cliff_cnt > 2)
						if (left_wheel_and_cliff || right_wheel_and_cliff) {
							wheel_cliff_state_++;
							if (wheel_cliff_state_ == 4)
								wheel_cliff_state_ = 7;
							else
								ROS_INFO("%s %d: Try wheel cliff resume the %d times.", __FUNCTION__, __LINE__, wheel_cliff_state_);
						} else {
							ROS_INFO("%s %d: Triggered cliff jam during resuming wheel cliff.", __FUNCTION__, __LINE__);
							wheel_cliff_state_ = 4;
							wheel_resume_cnt_ = 0;
							g_cliff_cnt = 0;
						}
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				}
				case 4: {
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) > 0.25f || lidar.getObstacleDistance(1, ROBOT_RADIUS) < 0.06) {
						wheel.stop();
						// If cliff jam during bumper self resume.
//						if (cliff.getStatus() && ++g_cliff_cnt > 2)
						if (left_wheel_and_cliff || right_wheel_and_cliff) {
							wheel_cliff_state_ = 1;
							ROS_INFO("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
						} else {
							ROS_INFO("%s %d: Triggered cliff jam during resuming wheel cliff.", __FUNCTION__, __LINE__);
							wheel_cliff_state_ = left_wheel_and_cliff ? 5 : 6;
							wheel_resume_cnt_ = 0;
							wheel_cliff_resume_start_radian_ = odom.getRadian();
						}
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				}
				case 5:
				case 6 : {
//					ROS_DEBUG("%s %d: robot::instance()->getWorldPoseRadian(): %d", __FUNCTION__, __LINE__,
//							  robot::instance()->getWorldPoseRadian());
					if (fabs(ranged_radian(odom.getRadian() - wheel_cliff_resume_start_radian_)) > degree_to_radian(90)) {
						if (left_wheel_and_cliff || right_wheel_and_cliff) {
							wheel_cliff_state_ = 1;
						} else {
							wheel_cliff_resume_cnt_++;
							if (wheel_cliff_resume_cnt_ > 3) {
								wheel_cliff_state_ = 7;
							} else {
								wheel_cliff_state_ = 4;
							}
						}
					}
					break;
				}
				default: //case 7:
				{
					ROS_ERROR("%s %d: Wheel cliff jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					robot_error.set(ERROR_CODE_CLIFF);
					break;
				}
			}
		}
	}
	else if (ev.cliff_turn)
	{
		if (cliff.getFront())
		{
			ev.cliff_turn = 0;
			ev.cliff_jam = true;
			ROS_WARN("%s %d: Cliff front triggered in cliff turn.", __FUNCTION__, __LINE__);
			beeper.debugBeep(VALID);
		}
		else
		{
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
			if (resume_cliff_turn_start_time_ == 0)
				resume_cliff_turn_start_time_ = ros::Time::now().toSec();
			if (ros::Time::now().toSec() - resume_cliff_turn_start_time_ > 3)
			{
				ev.fatal_quit = true;
				robot_error.set(ERROR_CODE_CLIFF);
				ROS_ERROR("%s %d: Cliff turn resume failed.", __FUNCTION__, __LINE__);
			}

			switch (ev.cliff_turn)
			{
				case BLOCK_CLIFF_TURN_LEFT:
				{
					if (cliff.getRight())
					{
						ev.cliff_turn = 0;
						ev.cliff_jam = true;
						ROS_WARN("%s %d: Cliff right triggered in cliff turn left.", __FUNCTION__, __LINE__);
						beeper.debugBeep(VALID);
					}
					else if (distance > 0.08f && !cliff.getLeft())
					{
						ROS_INFO("%s %d: Resume cliff turn left.", __FUNCTION__, __LINE__);
						ev.cliff_turn = 0;
					}
					break;
				}
				case BLOCK_CLIFF_TURN_RIGHT:
				{
					if (cliff.getLeft())
					{
						ev.cliff_turn = 0;
						ev.cliff_jam = true;
						ROS_WARN("%s %d: Cliff left triggered in cliff turn right.", __FUNCTION__, __LINE__);
						beeper.debugBeep(VALID);
					}
					else if (distance > 0.08f && !cliff.getRight())
					{
						ROS_INFO("%s %d: Resume cliff turn right.", __FUNCTION__, __LINE__);
						ev.cliff_turn = 0;
					}
					break;
				}
				default: //case BLOCK_CLIFF_TURN_ALL:
				{
					if (distance > 0.08f && !cliff.getStatus())
					{
						ROS_INFO("%s %d: Resume cliff turn all.", __FUNCTION__, __LINE__);
						ev.cliff_turn = 0;
					}
					break;
				}
			}
		}
	}
	else if (ev.bumper_jam)
	{
		if (bumper.getStatus() != BLOCK_LEFT && bumper.getStatus() != BLOCK_RIGHT && bumper.getStatus() != BLOCK_ALL) {
			ROS_WARN("%s %d: Bumper resume succeeded.", __FUNCTION__, __LINE__);
			ev.bumper_jam = false;
			ev.bumper_triggered = 0;
			g_bumper_cnt = 0;
			return true;
		} else if (bumper.getStatus() == BLOCK_LEFT || bumper.getStatus() == BLOCK_RIGHT ||
							 bumper.getStatus() == BLOCK_ALL) {
			switch (bumper_jam_state_) {
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				case 3: // Move back for the third time.
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) > 0.05f) {
						wheel.stop();
						// If cliff jam during bumper self resume.
						if (cliff.getStatus() && ++g_cliff_cnt > 2) {
							ROS_WARN("%s %d: Triggered cliff jam during resuming bumper.", __FUNCTION__, __LINE__);
							ev.cliff_jam = true;
							resetResumeTime();
							bumper_jam_state_ = 1;
							wheel_resume_cnt_ = 0;
							g_cliff_cnt = 0;
						} else {
							bumper_jam_state_++;
							ROS_INFO("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
							if (bumper_jam_state_ == 4)
								bumper_resume_start_radian_ = odom.getRadian();
						}
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				}
				case 4:
				case 5: {
//					ROS_DEBUG("%s %d: robot::instance()->getWorldPoseRadian(): %d", __FUNCTION__, __LINE__,
//							  robot::instance()->getWorldPoseRadian());
					// If cliff jam during bumper self resume.
					if (cliff.getStatus() && ++g_cliff_cnt > 2) {
						ROS_WARN("%s %d: Triggered cliff jam during resuming bumper.", __FUNCTION__, __LINE__);
						ev.cliff_jam = true;
						resetResumeTime();
						bumper_jam_state_ = 1;
						wheel_resume_cnt_ = 0;
						g_cliff_cnt = 0;
					} else if (fabs(ranged_radian(odom.getRadian() - bumper_resume_start_radian_)) > degree_to_radian(90)) {
						bumper_jam_state_++;
						bumper_resume_start_radian_ = odom.getRadian();
						ROS_INFO("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
					}
					break;
				}
				default: //case 6:
				{
					ROS_ERROR("%s %d: Bumper jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					robot_error.set(ERROR_CODE_BUMPER);
					break;
				}
			}
		}
	}
	else if (ev.lidar_bumper_jam)
	{
		if (bumper.getStatus() != BLOCK_LIDAR_BUMPER) {
			ROS_WARN("%s %d: Bumper resume succeeded.", __FUNCTION__, __LINE__);
			ev.lidar_bumper_jam = false;
			ev.bumper_triggered = 0;
			g_bumper_cnt = 0;
			return true;
		} else if (bumper.getStatus() == BLOCK_LIDAR_BUMPER) {
			switch (lidar_bumper_jam_state_) {
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				case 3: // Move back for the third time.
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) > 0.05f) {
						wheel.stop();
						// If cliff jam during bumper self resume.
						if (cliff.getStatus() && ++g_cliff_cnt > 2) {
							ROS_WARN("%s %d: Triggered cliff jam during resuming bumper.", __FUNCTION__, __LINE__);
							ev.cliff_jam = true;
							resetResumeTime();
							lidar_bumper_jam_state_ = 1;
							wheel_resume_cnt_ = 0;
							g_cliff_cnt = 0;
						} else {
							lidar_bumper_jam_state_++;
							ROS_INFO("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
							if (lidar_bumper_jam_state_ == 4)
								bumper_resume_start_radian_ = odom.getRadian();
						}
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				}
				case 4:
				case 5: {
//					ROS_DEBUG("%s %d: robot::instance()->getWorldPoseRadian(): %d", __FUNCTION__, __LINE__,
//							  robot::instance()->getWorldPoseRadian());
					// If cliff jam during bumper self resume.
					if (cliff.getStatus() && ++g_cliff_cnt > 2) {
						ROS_WARN("%s %d: Triggered cliff jam during resuming bumper.", __FUNCTION__, __LINE__);
						ev.cliff_jam = true;
						resetResumeTime();
						lidar_bumper_jam_state_ = 1;
						wheel_resume_cnt_ = 0;
						g_cliff_cnt = 0;
					} else if (fabs(ranged_radian(odom.getRadian() - bumper_resume_start_radian_)) > degree_to_radian(90)) {
						lidar_bumper_jam_state_++;
						bumper_resume_start_radian_ = odom.getRadian();
						ROS_INFO("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
					}
					break;
				}
				default: //case 6:
				{
					ROS_ERROR("%s %d: Lidar Bumper jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					robot_error.set(ERROR_CODE_LIDAR);
					break;
				}
			}
		}
	}
	else if (ev.tilt_jam)
	{
		if (!gyro.getTiltCheckingStatus()) {
			ROS_ERROR("%s %d: Tilt resume succeeded.", __FUNCTION__, __LINE__);
			ev.tilt_jam = false;
			ev.tilt_triggered = false;
			return true;
		} else {
			switch (tilt_jam_state_) {
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				case 3: // Move back for the third time.
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) > 0.05f) {
						wheel.stop();
						// If cliff jam during bumper self resume.
						if (cliff.getStatus() && ++g_cliff_cnt > 2) {
							ROS_WARN("%s %d: Triggered cliff jam during resuming tilt.", __FUNCTION__, __LINE__);
							ev.cliff_jam = true;
							resetResumeTime();
							tilt_jam_state_ = 1;
							wheel_resume_cnt_ = 0;
							g_cliff_cnt = 0;
						} else {
							tilt_jam_state_++;
							ROS_INFO("%s %d: Try tilt resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
							if (tilt_jam_state_ == 4)
								tilt_resume_start_radian_ = odom.getRadian();
						}
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				}
				case 4:
				case 5: {
//					ROS_DEBUG("%s %d: robot::instance()->getWorldPoseRadian(): %d", __FUNCTION__, __LINE__,
//							  robot::instance()->getWorldPoseRadian());
					// If cliff jam during bumper self resume.
					if (cliff.getStatus() && ++g_cliff_cnt > 2) {
						ROS_WARN("%s %d: Triggered cliff jam during resuming tilt.", __FUNCTION__, __LINE__);
						ev.cliff_jam = true;
						resetResumeTime();
						tilt_jam_state_ = 1;
						wheel_resume_cnt_ = 0;
						g_cliff_cnt = 0;
					} else if (fabs(ranged_radian(odom.getRadian() - tilt_resume_start_radian_)) > degree_to_radian(90))//90
					{
						tilt_jam_state_++;
						tilt_resume_start_radian_ = odom.getRadian();
						ROS_INFO("%s %d: Try tilt resume state %d.", __FUNCTION__, __LINE__, tilt_jam_state_);
					}
					break;
				}
				default: //case 6:
				{
					ROS_ERROR("%s %d: Tilt jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					robot_error.set(ERROR_CODE_STUCK);
					break;
				}
			}
		}
	}
	else if (ev.oc_vacuum)
	{
		/*--- init start time ---*/
		if (resume_vacuum_start_time_ == 0)
			resume_vacuum_start_time_ = ros::Time::now().toSec();
		if (!vacuum.getOc()) {
			ROS_WARN("%s %d: Vacuum over current resume succeeded!", __FUNCTION__, __LINE__);
			vacuum.resetExceptionResume();
			ev.oc_vacuum = false;
			return true;
		} else if (ros::Time::now().toSec() - resume_vacuum_start_time_ > 10) {
			ROS_ERROR("%s %d: Vacuum resume failed..", __FUNCTION__, __LINE__);
			ev.oc_vacuum = false;
			ev.fatal_quit = true;
			vacuum.resetExceptionResume();
			robot_error.set(ERROR_CODE_VACUUM);
		} else if (oc_vacuum_resume_cnt_ == 0) {
			brush.stop();
			vacuum.stop();
			vacuum.startExceptionResume();
			oc_vacuum_resume_cnt_++;
		}
	}
	else if (ev.oc_brush_main)
	{
		/*--- init start time ---*/
		if (resume_main_bursh_start_time_ == 0)
			resume_main_bursh_start_time_ = ros::Time::now().toSec();
		if (oc_main_brush_cnt_ < 1) {
			switch (main_brush_resume_state_) {
				case 1: {
					if (brush.isSideBrushOn())
						brush.stopForMainBrushResume();
					if (vacuum.isOn())
						vacuum.stop();
					if (water_tank.getStatus(WaterTank::operate_option::swing_motor))
						water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) >= CELL_SIZE * ROBOT_SIZE / 2) {
						ROS_INFO("%s %d: Move back finish!", __FUNCTION__, __LINE__);
						brush.mainBrushResume();
						main_brush_resume_state_++;
						resume_main_bursh_start_time_ = ros::Time::now().toSec();
					}
					break;
				}
				case 2: {
					if ((ros::Time::now().toSec() - resume_main_bursh_start_time_) >= 1.5 && brush.getMainOc()) {
						oc_main_brush_cnt_++;
						main_brush_resume_state_ = 1;
					}

					if ((ros::Time::now().toSec() - resume_main_bursh_start_time_) >= 3) {
						ROS_WARN("%s %d: main brush over current resume succeeded!", __FUNCTION__, __LINE__);
						if (brush.isMainBrushSlowOperate())
							brush.blockMainBrushSlowOperation();
						brush.stop();
						brush.slowOperate();
						ev.oc_brush_main = false;
						return true;
					}
					break;
				}
				default:
					main_brush_resume_state_ = 1;
					break;
			}
		} else {
			ROS_ERROR("%s %d: Main brush stuck.", __FUNCTION__, __LINE__);
			ev.oc_brush_main = false;
			ev.fatal_quit = true;
			robot_error.set(ERROR_CODE_MAINBRUSH);
		}
	}
	else if (ev.robot_stuck)
	{
		/*--- init start time ---*/
		if (resume_stuck_start_time_ == 0)
			resume_stuck_start_time_ = ros::Time::now().toSec();
		CellState cell_state;
		auto is_follow_wall_left = last_action_i_ == sp_mt_->sp_mode_->ac_follow_wall_left;
		auto is_follow_wall_right = last_action_i_ == sp_mt_->sp_mode_->ac_follow_wall_right;
		if (sp_mt_->sp_mode_->mode_i_ != sp_mt_->sp_mode_->md_go_to_charger &&
				sp_mt_->sp_mode_->mode_i_ != sp_mt_->sp_mode_->md_remote) {
			ACleanMode *p_mode = dynamic_cast<ACleanMode *>(sp_mt_->sp_mode_);
			cell_state = p_mode->clean_map_.getCost(getPosition().toCell().x, getPosition().toCell().y);
		}

		if (ros::Time::now().toSec() - resume_stuck_start_time_ > 60) {
			ev.robot_stuck = false;
			ev.slip_triggered = false;
			ev.fatal_quit = true;
			robot_error.set(ERROR_CODE_STUCK);
		}
		switch (robot_stuck_flag_) {
			case 0: {
				float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
				if ((cell_state != BLOCKED_SLIP && std::abs(distance) > 0.15f) ||
						lidar.getObstacleDistance(1, ROBOT_RADIUS) < 0.06) {
					ROS_INFO("%s,%d Robot slip to go straight finished", __FUNCTION__, __LINE__);
					if (!robot::instance()->isRobotSlip()) {
						ev.robot_stuck = false;
						ev.slip_triggered = false;
						stuck_start_turn_time_ = ros::Time::now().toSec();//in this place,stuck_start_turn_time_ record the slip end time
						return true;
					} else {
						if (is_follow_wall_left)
							robot_stuck_flag_ = 1;
						else if (is_follow_wall_right)
							robot_stuck_flag_ = 2;
						else
							robot_stuck_flag_ = static_cast<uint8_t>(is_stuck_last_turn_right_ ? 2 : 1);
						stuck_start_turn_time_ = ros::Time::now().toSec();
					}
				}
				break;
			}
			case 1: {
				if (ros::Time::now().toSec() - stuck_start_turn_time_ > 1) {
					ROS_INFO("%s,%d Robot slip to turn right finished", __FUNCTION__, __LINE__);
					s_pos_x = odom.getOriginX();
					s_pos_y = odom.getOriginY();
					is_stuck_last_turn_right_ = true;
					robot_stuck_flag_ = 0;
				}
				break;
			}
			case 2: {
				if (ros::Time::now().toSec() - stuck_start_turn_time_ > 1) {
					ROS_INFO("%s,%d Robot slip to turn left finished", __FUNCTION__, __LINE__);
					s_pos_x = odom.getOriginX();
					s_pos_y = odom.getOriginY();
					is_stuck_last_turn_right_ = false;
					robot_stuck_flag_ = 0;
				}
				break;
			}
		}
	}
	else if (ev.lidar_stuck)
	{
		/*--- init start time ---*/
		if (resume_lidar_start_time_ == 0)
			resume_lidar_start_time_ = ros::Time::now().toSec();
		if (lidar_resume_cnt_ < 5) {
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
			if (std::abs(distance) > 0.02f) {
				wheel.stop();
				lidar_resume_cnt_++;
				if (lidar_resume_cnt_ <= 5)
					ROS_INFO("%s %d: Resume failed, try lidar resume for the %d time.",
									 __FUNCTION__, __LINE__, lidar_resume_cnt_);
				s_pos_x = odom.getOriginX();
				s_pos_y = odom.getOriginY();
			}
		} else if (lidar_resume_cnt_ == 5) {
			if (ros::Time::now().toSec() - resume_lidar_start_time_ > 30) {//stop for 10 seconds for rescue the lidar
				lidar_resume_cnt_ = 6;
			}
		} else {
			ROS_ERROR("%s %d: lidar jamed.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
			robot_error.set(ERROR_CODE_LIDAR);
		}
		if (!robot::instance()->checkLidarStuck()) {
			ROS_WARN("%s %d: Lidar resume succeeded.", __FUNCTION__, __LINE__);
			ev.lidar_stuck = false;
			return true;
		}
	}
	else if (ev.gyro_error)
	{
		/*--- init start time ---*/
		if (resume_gyro_start_time_ == 0)
			resume_gyro_start_time_ = ros::Time::now().toSec();
		if (should_init_for_gyro_exception_) {
			should_init_for_gyro_exception_ = false;
			if (brush.isSideBrushOn())
				brush.stop();
			vacuum.stop();
			water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);
			odom.setRadianOffset(odom.getRadian());
			gyro.setOff();
		}
		if (++debug_print_counter_ > 50) {
			ROS_WARN("%s, %d: gyro error: %d", __FUNCTION__, __LINE__, gyro.error());
			debug_print_counter_ = 0;
		}
		if (!gyro.error()) {
			if (p_action_open_gyro_ == nullptr) {
				ROS_WARN("%s, %d: gyro.error is false, restart gyro", __FUNCTION__, __LINE__);
				p_action_open_gyro_ = new ActionOpenGyro();
			}

			if (p_action_open_gyro_->isFinish()) {
				ROS_WARN("%s, %d: Gyro resume success", __FUNCTION__, __LINE__);
				ev.gyro_error = false;
				delete p_action_open_gyro_;
				return true;
				return true;
			} else
				p_action_open_gyro_->run();
		} else if (ros::Time::now().toSec() - resume_gyro_start_time_ > 30) {
			ROS_ERROR("%s, %d: Gyro resume fail!", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
			ev.gyro_error = false;
			robot_error.set(ERROR_CODE_GYRO);
		}
	}
//	if (ev.fatal_quit)
//		ROS_INFO("%s %d: ev.fatal_quit is set to %d.", __FUNCTION__, __LINE__, ev.fatal_quit);
	return ev.fatal_quit;
}

void MovementExceptionResume::resetResumeTime(void)
{
	resume_wheel_start_time_ = 0;
	resume_main_bursh_start_time_ = 0;
	resume_vacuum_start_time_ = 0;
	resume_lidar_start_time_ = 0;
	resume_stuck_start_time_ = 0;
	resume_gyro_start_time_ = 0;
	resume_cliff_turn_start_time_ = 0;
}
