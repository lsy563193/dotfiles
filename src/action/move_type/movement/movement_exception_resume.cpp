//
// Created by lsy563193 on 11/29/17.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <error.h>
#include <event_manager.h>
#include <robot.hpp>
#include <mode.hpp>
#include "dev.h"

double MovementExceptionResume::slip_start_turn_time_ = 0;
bool MovementExceptionResume::is_slip_last_turn_left_ = false;
MovementExceptionResume::MovementExceptionResume()
{
	ROS_INFO("%s %d: Entering movement exception resume.", __FUNCTION__, __LINE__);

	// Save current position for moving back detection.
	s_pos_x = odom.getOriginX();
	s_pos_y = odom.getOriginY();

	//For slip
	if(ros::Time::now().toSec() - slip_start_turn_time_ < 5){
		robot_slip_flag_ = static_cast<uint8_t>(is_slip_last_turn_left_ ? 2 : 1);
		slip_start_turn_time_ = ros::Time::now().toSec();
	}else{
		robot_slip_flag_ = 0;
		slip_start_turn_time_ = 0;
	}

	resume_wheel_start_time_ = ros::Time::now().toSec();
	resume_main_bursh_start_time_ = ros::Time::now().toSec();
	resume_vacuum_start_time_ = ros::Time::now().toSec();
	resume_lidar_start_time_ = ros::Time::now().toSec();
	resume_slip_start_time_ = ros::Time::now().toSec();
}

MovementExceptionResume::~MovementExceptionResume()
{
	ROS_INFO("%s %d: Exiting movement exception resume.", __FUNCTION__, __LINE__);
}

void MovementExceptionResume::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	ROS_INFO("self rescuing!");
	if (ev.oc_vacuum)
		left_speed = right_speed = 0;
	else if (ev.oc_wheel_left || ev.oc_wheel_right)
	{
		if (ev.oc_wheel_right)
			wheel.setDirectionRight();
		else
			wheel.setDirectionLeft();
		left_speed = 30;
		right_speed = 30;
	}
	else if (ev.robot_stuck || ev.cliff_jam || ev.cliff_all_triggered )
	{
		wheel.setDirectionBackward();
		left_speed = right_speed = RUN_TOP_SPEED;
	}
	else if(sp_mt_->sp_mode_->is_wheel_cliff_triggered)//new wheel cliff rescue
	{
		switch (wheel_cliff_state_)
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
#if 0
	else if(ev.left_wheel_cliff || ev.right_wheel_cliff)
	{
		switch(wheel_cliff_state_)
		{
			case 1:
				wheel.setDirectionBackward();
				left_speed = right_speed = BACK_MAX_SPEED;
				break;
			case 2:
				wheel.setDirectionLeft();
				left_speed = 30;
				right_speed = 30;
				break;
			case 3:
				wheel.setDirectionRight();
				left_speed = 30;
				right_speed = 30;
				break;
		}
	}
#endif
	else if(ev.oc_brush_main)
	{
		if(main_brush_resume_state_ == 1){
			wheel.setDirectionBackward();
			left_speed = right_speed = BACK_MAX_SPEED;
		}
		else
			left_speed = right_speed = 0;
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
	else if (ev.lidar_stuck)
	{
		ROS_INFO("lidar stucking");
		if (lidar_resume_cnt_ >= 5) {
			wheel.stop();
		} else {
			wheel.setDirectionBackward();
			left_speed = right_speed = BACK_MAX_SPEED;
		}
	}
	else if (ev.robot_slip)
	{
		ROS_INFO("slipping");
		switch(robot_slip_flag_){
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
//	ROS_INFO("speed(%d, %d)!", left_speed, right_speed);
}

bool MovementExceptionResume::isFinish()
{
	updatePosition();
	if (!(ev.bumper_jam || ev.lidar_bumper_jam || ev.cliff_jam || ev.cliff_all_triggered || ev.oc_wheel_left || ev.oc_wheel_right
		  || ev.oc_vacuum || ev.lidar_stuck || ev.robot_stuck || ev.oc_brush_main || ev.robot_slip
			|| sp_mt_->sp_mode_->is_wheel_cliff_triggered))
	{
		ROS_INFO("%s %d: All exception cleared.", __FUNCTION__, __LINE__);
		return true;
	}

	// Check for right wheel.
	if (ev.oc_wheel_left || ev.oc_wheel_right)
	{
		if (brush.isOn())
			brush.stop();
		vacuum.stop();
		water_tank.stop(WaterTank::tank_pump);
		if (ros::Time::now().toSec() - resume_wheel_start_time_ >= 1)
		{
			if (wheel.getLeftWheelOc() || wheel.getRightWheelOc())
			{
				if (wheel_resume_cnt_ >= 3)
				{
					wheel.stop();
					brush.stop();
					vacuum.stop();
					if (ev.oc_wheel_left)
					{
						ROS_WARN("%s,%d Left wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
						error.set(ERROR_CODE_LEFTWHEEL);
					} else
					{
						ROS_WARN("%s,%d Right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
						error.set(ERROR_CODE_RIGHTWHEEL);
					}
					ev.fatal_quit = true;
					return true;
				}
				else
				{
					resume_wheel_start_time_ = time(NULL);
					wheel_resume_cnt_++;
					ROS_WARN("%s %d: Failed to resume for %d times.", __FUNCTION__, __LINE__, wheel_resume_cnt_);
				}
			}
			else
			{
				if (ev.oc_wheel_left)
				{
					ROS_WARN("%s %d: Left wheel resume succeeded.", __FUNCTION__, __LINE__);
					ev.oc_wheel_left = false;
					if (!water_tank.checkEquipment(true))
						vacuum.setCleanState();
//					brush.normalOperate();

				} else
				{
					ROS_WARN("%s %d: Right wheel resume succeeded.", __FUNCTION__, __LINE__);
					ev.oc_wheel_right = false;
					if (!water_tank.checkEquipment(true))
						vacuum.setCleanState();
//					brush.normalOperate();
				}
			}
		}
	}
	else if (ev.oc_brush_main)
	{
		if (oc_main_brush_cnt_ < 1)
		{
			switch (main_brush_resume_state_)
			{
				case 1:
				{
					if (brush.isOn())
						brush.stop();
					vacuum.stop();
					water_tank.stop(WaterTank::tank_pump);
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) >= CELL_SIZE * ROBOT_SIZE / 2)
					{
						ROS_INFO("%s %d: Move back finish!", __FUNCTION__, __LINE__);
						brush.mainBrushResume();
						main_brush_resume_state_++;
						resume_main_bursh_start_time_ = ros::Time::now().toSec();
					}
					break;
				}
				case 2:
				{
					if ((ros::Time::now().toSec() - resume_main_bursh_start_time_) >= 3)
					{
						if (!brush.getMainOc())
						{
							ROS_INFO("%s %d: main brush over current resume succeeded!", __FUNCTION__, __LINE__);
							brush.normalOperate();
							ev.oc_brush_main = false;
							if (!water_tank.checkEquipment(true))
								vacuum.setCleanState();
						}
						else
						{
							oc_main_brush_cnt_++;
							main_brush_resume_state_ = 1;
						}
					}
					break;
				}
				default:
					main_brush_resume_state_ = 1;
					break;
			}
		}
		else
		{
			ROS_WARN("%s %d: Main brush stuck.", __FUNCTION__, __LINE__);
			ev.oc_brush_main = false;
			ev.fatal_quit = true;
			error.set(ERROR_CODE_MAINBRUSH);
		}
	}
	else if (ev.robot_stuck)
	{
		if (!lidar.isRobotSlip())
		{
			ROS_INFO("%s %d: Cliff resume succeeded.", __FUNCTION__, __LINE__);
			ev.robot_slip = false;
			ev.robot_stuck = false;
		}
		else if (robot_stuck_resume_cnt_ < 5)
		{
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
			if (std::abs(distance) > 0.05f)
			{
				wheel.stop();
				robot_stuck_resume_cnt_++;
				ROS_WARN("%s %d: Try robot stuck resume for the %d time.", __FUNCTION__, __LINE__, robot_stuck_resume_cnt_);
				s_pos_x = odom.getOriginX();
				s_pos_y = odom.getOriginY();
			}
		}
		else
		{
			ROS_WARN("%s %d: Robot stuck.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
			error.set(ERROR_CODE_STUCK);
		}
	}
	else if (ev.cliff_all_triggered)
	{
		if(wheel.getRightWheelCliffStatus() && wheel.getLeftWheelCliffStatus())
		{
			ev.fatal_quit = true;
			ROS_WARN("%s %d: All cliff and wheel cliff  triggered.", __FUNCTION__, __LINE__);
		}
		if (cliff.getStatus() != BLOCK_ALL)
		{
			ROS_INFO("%s %d: Cliff all resume succeeded.", __FUNCTION__, __LINE__);
			ev.cliff_all_triggered = false;
			ev.cliff_triggered = 0;
			g_cliff_cnt = 0;
		}
		else if (cliff_all_resume_cnt_ < 2)
		{
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
			if (std::abs(distance) > 0.02f)
			{
				wheel.stop();
				cliff_all_resume_cnt_++;
				if (cliff_all_resume_cnt_ <= 2)
					ROS_WARN("%s %d: Resume failed, try cliff all resume for the %d time.",
							 __FUNCTION__, __LINE__, cliff_all_resume_cnt_);
				s_pos_x = odom.getOriginX();
				s_pos_y = odom.getOriginY();
			}
		}
		else
		{
			ROS_WARN("%s %d: Cliff all triggered.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
		}
	}
	else if (ev.cliff_jam)
	{
		if (!cliff.getStatus())
		{
			ROS_INFO("%s %d: Cliff resume succeeded.", __FUNCTION__, __LINE__);
			ev.cliff_jam = false;
			ev.cliff_triggered = 0;
			g_cliff_cnt = 0;
		}
		else if (cliff_resume_cnt_ < 5)
		{
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
			if (std::abs(distance) > 0.02f)
			{
				wheel.stop();
				cliff_resume_cnt_++;
				if (cliff_resume_cnt_ <= 5)
					ROS_WARN("%s %d: Resume failed, try cliff resume for the %d time.",
							 __FUNCTION__, __LINE__, cliff_resume_cnt_);
				s_pos_x = odom.getOriginX();
				s_pos_y = odom.getOriginY();
			}
		}
		else
		{
			ROS_WARN("%s %d: Cliff jamed.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
			error.set(ERROR_CODE_CLIFF);
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
		if(!wheel.getLeftWheelCliffStatus() && !wheel.getRightWheelCliffStatus())//for checking wheel cliff is still tirggered
		{
			ROS_INFO("%s %d: wheel cliff resume succeeded.", __FUNCTION__, __LINE__);
			sp_mt_->sp_mode_->is_wheel_cliff_triggered = false;
			ev.right_wheel_cliff = false;
			ev.left_wheel_cliff = false;
		}
		else
		{
			switch (wheel_cliff_state_)
			{
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				case 3: // Move back for the third time.
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
//					ROS_INFO("distance(%lf), pos(%lf, %lf)", distance, s_pos_x, s_pos_y);
					if (std::abs(distance) > 0.02f)
					{
						wheel.stop();
						// If cliff jam during bumper self resume.
//						if (cliff.getStatus() && ++g_cliff_cnt > 2)
						if (left_wheel_and_cliff || right_wheel_and_cliff)
						{
							wheel_cliff_state_++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
							if (wheel_cliff_state_ == 4)
								wheel_cliff_state_ = 7;
						} else
						{
							ROS_WARN("%s %d: Triggered cliff jam during resuming wheel cliff.", __FUNCTION__, __LINE__);
							wheel_cliff_state_ = 4;
							wheel_resume_cnt_ = 0;
							g_cliff_cnt = 0;
						}
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				}
				case 4:
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) > 0.25f || lidar.getObstacleDistance(1, ROBOT_RADIUS) < 0.06)
					{
						wheel.stop();
						// If cliff jam during bumper self resume.
//						if (cliff.getStatus() && ++g_cliff_cnt > 2)
						if (left_wheel_and_cliff || right_wheel_and_cliff)
						{
							wheel_cliff_state_ = 1;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
						} else
						{
							ROS_WARN("%s %d: Triggered cliff jam during resuming wheel cliff.", __FUNCTION__, __LINE__);
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
				case 6 :
				{
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
					ROS_WARN("%s %d: Wheel cliff jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					error.set(ERROR_CODE_CLIFF);
					break;
				}
			}
		}
	}
#if 0
	else if(ev.right_wheel_cliff || ev.left_wheel_cliff)
	{
		bool cliff_status = ev.right_wheel_cliff ? cliff.getRight() : cliff.getLeft();
		bool right_wheel_and_cliff{false};
		bool left_wheel_and_cliff{false};
		if (ev.right_wheel_cliff) {
			right_wheel_and_cliff = cliff.getRight();
		}
		if (ev.left_wheel_cliff) {
			left_wheel_and_cliff = cliff.getLeft();
		}
		float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());

		if(!wheel.getLeftWheelCliffStatus() && !wheel.getRightWheelCliffStatus())
		{
			sp_mt_->sp_mode_->is_wheel_cliff_triggered = false;
			ev.right_wheel_cliff = false;
			ev.left_wheel_cliff = false;
			ROS_WARN("%s %d: Wheel cliff resume succeeded.", __FUNCTION__, __LINE__);
		}
//		ROS_INFO("^:%d, cliff_status:%d, wheel_cliff_resume_cnt:%d,cliff_right:%d, cliff_left:%d, cliff_front:%d, right_wheel_cliff:%d, left_wheel_cliff:%d",
//							ev.right_wheel_cliff ^ ev.left_wheel_cliff,cliff_status, wheel_cliff_resume_cnt_,
//							cliff.getRight(), cliff.getLeft(), cliff.getFront(), ev.right_wheel_cliff, ev.left_wheel_cliff);
		if((ev.right_wheel_cliff ^ ev.left_wheel_cliff)
			 && (left_wheel_and_cliff || right_wheel_and_cliff)
			 && wheel_cliff_resume_cnt_ < 3)
		{
			if(distance > 0.02f || lidar.getObstacleDistance(1, ROBOT_RADIUS) < 0.06);
			{
				wheel_cliff_resume_cnt_++;
				s_pos_x = odom.getOriginX();
				s_pos_y = odom.getOriginY();
				if(wheel_cliff_resume_cnt_ <= 3)
					ROS_WARN("%s %d: Resume failed, try wheel cliff resume for the %d time is finished.", __FUNCTION__, __LINE__,wheel_cliff_resume_cnt_);
			}
		}
		else if(cliff.getStatus() && wheel_cliff_resume_cnt_ < 3)
		{
			ROS_ERROR("cliff detect!!!!!!!!!");
			switch(wheel_cliff_state_)
			{
				case 1:
					if(distance > 0.25f || lidar.getObstacleDistance(1, ROBOT_RADIUS) < 0.06)
					{
						wheel_cliff_state_ = static_cast<uint8_t>(ev.right_wheel_cliff ? 2 : 3); // 2 is turn left, 3 is turn right
						wheel_cliff_start_time_ = ros::Time::now().toSec();
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				default:
					if(ros::Time::now().toSec() - wheel_cliff_start_time_ > 1)
					{
						wheel_cliff_state_ = 1;
						wheel_cliff_resume_cnt_++;
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
						if(wheel_cliff_resume_cnt_ <= 3)
							ROS_WARN("%s %d: Resume failed, try wheel cliff resume for the %d time is finished.", __FUNCTION__, __LINE__,wheel_cliff_resume_cnt_);
					}
					break;
			}
		}
		else if(wheel_cliff_resume_cnt_ >= 3)
		{
			ROS_WARN("%s %d: Wheel cliff suspend,but resume failed.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
			error.set(ERROR_CODE_CLIFF);
		}
	}
#endif
	else if (ev.bumper_jam)
	{
		if (bumper.getStatus() != BLOCK_LEFT && bumper.getStatus() != BLOCK_RIGHT && bumper.getStatus() != BLOCK_ALL)
		{
			ROS_INFO("%s %d: Bumper resume succeeded.", __FUNCTION__, __LINE__);
			ev.bumper_jam = false;
			ev.bumper_triggered = 0;
			g_bumper_cnt = 0;
		}
		else if(bumper.getStatus() == BLOCK_LEFT || bumper.getStatus() == BLOCK_RIGHT || bumper.getStatus() == BLOCK_ALL)
		{
			switch (bumper_jam_state_)
			{
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				case 3: // Move back for the third time.
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) > 0.05f)
					{
						wheel.stop();
						// If cliff jam during bumper self resume.
						if (cliff.getStatus() && ++g_cliff_cnt > 2)
						{
							ROS_WARN("%s %d: Triggered cliff jam during resuming bumper.", __FUNCTION__, __LINE__);
							ev.cliff_jam = true;
							bumper_jam_state_ = 1;
							wheel_resume_cnt_ = 0;
							g_cliff_cnt = 0;
						} else
						{
							bumper_jam_state_++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
							if (bumper_jam_state_ == 4)
								bumper_resume_start_radian_ = odom.getRadian();
						}
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				}
				case 4:
				case 5:
				{
//					ROS_DEBUG("%s %d: robot::instance()->getWorldPoseRadian(): %d", __FUNCTION__, __LINE__,
//							  robot::instance()->getWorldPoseRadian());
					// If cliff jam during bumper self resume.
					if (cliff.getStatus() && ++g_cliff_cnt > 2)
					{
						ROS_WARN("%s %d: Triggered cliff jam during resuming bumper.", __FUNCTION__, __LINE__);
						ev.cliff_jam = true;
						bumper_jam_state_ = 1;
						wheel_resume_cnt_ = 0;
						g_cliff_cnt = 0;
					} else if (fabs(ranged_radian(odom.getRadian() - bumper_resume_start_radian_)) > degree_to_radian(90))
					{
						bumper_jam_state_++;
						bumper_resume_start_radian_ = odom.getRadian();
						ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
					}
					break;
				}
				default: //case 6:
				{
					ROS_WARN("%s %d: Bumper jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					error.set(ERROR_CODE_BUMPER);
					break;
				}
			}
		}
	}
	else if (ev.lidar_bumper_jam)
	{
		if (bumper.getStatus() != BLOCK_LIDAR_BUMPER)
		{
			ROS_INFO("%s %d: Bumper resume succeeded.", __FUNCTION__, __LINE__);
			ev.lidar_bumper_jam = false;
			ev.bumper_triggered = 0;
			g_bumper_cnt = 0;
		}
		else if (bumper.getStatus() == BLOCK_LIDAR_BUMPER)
		{
			switch (lidar_bumper_jam_state_)
			{
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				case 3: // Move back for the third time.
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
					if (std::abs(distance) > 0.05f)
					{
						wheel.stop();
						// If cliff jam during bumper self resume.
						if (cliff.getStatus() && ++g_cliff_cnt > 2)
						{
							ROS_WARN("%s %d: Triggered cliff jam during resuming bumper.", __FUNCTION__, __LINE__);
							ev.cliff_jam = true;
							lidar_bumper_jam_state_ = 1;
							wheel_resume_cnt_ = 0;
							g_cliff_cnt = 0;
						} else
						{
							lidar_bumper_jam_state_++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
							if (lidar_bumper_jam_state_ == 4)
								bumper_resume_start_radian_ = odom.getRadian();
						}
						s_pos_x = odom.getOriginX();
						s_pos_y = odom.getOriginY();
					}
					break;
				}
				case 4:
				case 5:
				{
//					ROS_DEBUG("%s %d: robot::instance()->getWorldPoseRadian(): %d", __FUNCTION__, __LINE__,
//							  robot::instance()->getWorldPoseRadian());
					// If cliff jam during bumper self resume.
					if (cliff.getStatus() && ++g_cliff_cnt > 2)
					{
						ROS_WARN("%s %d: Triggered cliff jam during resuming bumper.", __FUNCTION__, __LINE__);
						ev.cliff_jam = true;
						lidar_bumper_jam_state_ = 1;
						wheel_resume_cnt_ = 0;
						g_cliff_cnt = 0;
					} else if (fabs(ranged_radian(odom.getRadian() - bumper_resume_start_radian_)) > degree_to_radian(90))
					{
						lidar_bumper_jam_state_++;
						bumper_resume_start_radian_ = odom.getRadian();
						ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
					}
					break;
				}
				default: //case 6:
				{
					ROS_WARN("%s %d: Lidar Bumper jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					error.set(ERROR_CODE_LIDAR);
					break;
				}
			}
		}
	}
	else if (ev.oc_vacuum)
	{
		if (!vacuum.getOc())
		{
			ROS_INFO("%s %d: Vacuum over current resume succeeded!", __FUNCTION__, __LINE__);
			brush.normalOperate();
			if (!water_tank.checkEquipment(true))
				vacuum.setCleanState();
			vacuum.resetExceptionResume();
			ev.oc_vacuum = false;
		}
		else if (ros::Time::now().toSec() - resume_vacuum_start_time_ > 10)
		{
			ROS_WARN("%s %d: Vacuum resume failed..", __FUNCTION__, __LINE__);
			ev.oc_vacuum = false;
			ev.fatal_quit = true;
			vacuum.resetExceptionResume();
			error.set(ERROR_CODE_VACUUM);
		}
		else if (oc_vacuum_resume_cnt_ == 0)
		{
			brush.stop();
			vacuum.stop();
			vacuum.startExceptionResume();
			oc_vacuum_resume_cnt_++;
		}
	}
	else if(ev.robot_slip)
	{
		CellState isExitSlipBlock;
		if(sp_mt_->sp_mode_->mode_i_ != sp_mt_->sp_mode_->md_go_to_charger &&
				sp_mt_->sp_mode_->mode_i_ != sp_mt_->sp_mode_->md_remote){
			ACleanMode* p_mode = dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_);
			isExitSlipBlock = p_mode->clean_map_.getCell(CLEAN_MAP,getPosition().toCell().x,getPosition().toCell().y);
		}

		if(ros::Time::now().toSec() - resume_slip_start_time_ > 60){
			ev.robot_slip = false;
			ev.fatal_quit = true;
			error.set(ERROR_CODE_STUCK);

		}
		switch(robot_slip_flag_){
			case 0:{
				float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
				if ((isExitSlipBlock != BLOCKED_SLIP && std::abs(distance) > 0.15f) || lidar.getObstacleDistance(1, ROBOT_RADIUS) < 0.06)
				{
					if(!lidar.isRobotSlip())
					{
						ev.robot_slip = false;
						slip_start_turn_time_ = ros::Time::now().toSec();//in this place,slip_start_turn_time_ record the slip end time
					}
					else{
						robot_slip_flag_ = static_cast<uint8_t>(is_slip_last_turn_left_ ? 2 : 1);
						slip_start_turn_time_ = ros::Time::now().toSec();
					}
				}
				break;
			}
			case 1:{
				if(ros::Time::now().toSec() - slip_start_turn_time_ > 1) {
					s_pos_x = odom.getOriginX();
					s_pos_y = odom.getOriginY();
					is_slip_last_turn_left_ = true;
					robot_slip_flag_ = 0;
				}
				break;
			}
			case 2:{
				if(ros::Time::now().toSec() - slip_start_turn_time_ > 1)
				{
					s_pos_x = odom.getOriginX();
					s_pos_y = odom.getOriginY();
					is_slip_last_turn_left_ = false;
					robot_slip_flag_ = 0;
				}
				break;
			}
		}
	}
	else if (ev.lidar_stuck) {
		if (lidar_resume_cnt_ < 5)
		{
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getOriginX(), odom.getOriginY());
			if (std::abs(distance) > 0.02f)
			{
				wheel.stop();
				lidar_resume_cnt_++;
				if (lidar_resume_cnt_ <= 5)
					ROS_WARN("%s %d: Resume failed, try lidar resume for the %d time.",
									 __FUNCTION__, __LINE__, lidar_resume_cnt_);
				s_pos_x = odom.getOriginX();
				s_pos_y = odom.getOriginY();
			}
		} else if (lidar_resume_cnt_ == 5) {
			if (ros::Time::now().toSec() - resume_vacuum_start_time_ > 10) {//stop for 10 seconds for rescue the lidar
				lidar_resume_cnt_ = 6;
			}
		}
		else
		{
			ROS_WARN("%s %d: lidar jamed.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
			error.set(ERROR_CODE_LIDAR);
		}
	}

//	if (ev.fatal_quit)
//		ROS_INFO("%s %d: ev.fatal_quit is set to %d.", __FUNCTION__, __LINE__, ev.fatal_quit);
	return ev.fatal_quit;
}

