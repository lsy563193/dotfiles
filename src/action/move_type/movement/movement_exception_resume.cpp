//
// Created by lsy563193 on 11/29/17.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <error.h>
#include <event_manager.h>
#include "dev.h"

MovementExceptionResume::MovementExceptionResume()
{
	ROS_INFO("%s %d: Entering movement exception resume.", __FUNCTION__, __LINE__);

	// Save current position for moving back detection.
	s_pos_x = odom.getX();
	s_pos_y = odom.getY();

	resume_wheel_start_time_ = ros::Time::now().toSec();
	resume_main_bursh_start_time_ = ros::Time::now().toSec();
}

MovementExceptionResume::~MovementExceptionResume()
{

}

void MovementExceptionResume::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	if (ev.oc_suction)
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
	else if(ev.oc_brush_main)
	{
		if(oc_main_brush_cnt_ <= 0){
			wheel.setDirectionBackward();
			left_speed = right_speed = RUN_TOP_SPEED;
		}
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
	else if (ev.lidar_stuck)
	{
		wheel.setDirectionBackward();
		left_speed = right_speed = 2;
	}

}

bool MovementExceptionResume::isFinish()
{
	if (!(ev.bumper_jam || ev.cliff_jam || ev.cliff_all_triggered || ev.oc_wheel_left || ev.oc_wheel_right
		  || ev.oc_suction || ev.lidar_stuck || ev.robot_stuck || ev.oc_brush_main))
	{
		ROS_INFO("%s %d: All exception cleared.", __FUNCTION__, __LINE__);
		return true;
	}

	// Check for right wheel.
	if (ev.oc_wheel_left || ev.oc_wheel_right)
	{
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
					vacuum.setLastMode();
					brush.normalOperate();
				} else
				{
					ROS_WARN("%s %d: Right wheel resume succeeded.", __FUNCTION__, __LINE__);
					ev.oc_wheel_right = false;
					vacuum.setLastMode();
					brush.normalOperate();
				}
			}
		}
	}

	else if (ev.oc_brush_main){
		if(!brush.getMainOc())
		{
			ROS_INFO("%s %d: main brush over current resume succeeded!", __FUNCTION__, __LINE__);
			brush.normalOperate();
			ev.oc_brush_main = false;
		}

		float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
		ROS_DEBUG("%s %d: distance = %f, 3*cell_size = %f", __FUNCTION__, __LINE__, distance, CELL_SIZE*3);
		if (oc_main_brush_cnt_ < 1)
		{
			brush.stop();
			if(std::abs(distance) >= CELL_SIZE * 3)
			{
				wheel.stop();
				brush.mainBrushResume();
				oc_main_brush_cnt_++;
				resume_main_bursh_start_time_ = ros::Time::now().toSec();
			}
		}
		else if((ros::Time::now().toSec() - resume_main_bursh_start_time_) >=3 )
		{
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
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
			if (std::abs(distance) > 0.05f)
			{
				wheel.stop();
				robot_stuck_resume_cnt_++;
				ROS_WARN("%s %d: Try robot stuck resume for the %d time.", __FUNCTION__, __LINE__, robot_stuck_resume_cnt_);
				s_pos_x = odom.getX();
				s_pos_y = odom.getY();
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
		if (cliff.getStatus() != BLOCK_ALL)
		{
			ROS_INFO("%s %d: Cliff all resume succeeded.", __FUNCTION__, __LINE__);
			ev.cliff_all_triggered = false;
			ev.cliff_triggered = 0;
			g_cliff_cnt = 0;
		}
		else if (cliff_all_resume_cnt_ < 2)
		{
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
			if (std::abs(distance) > 0.02f)
			{
				wheel.stop();
				cliff_all_resume_cnt_++;
				if (cliff_all_resume_cnt_ < 2)
					ROS_WARN("%s %d: Resume failed, try cliff all resume for the %d time.",
							 __FUNCTION__, __LINE__, cliff_all_resume_cnt_ + 1);
				s_pos_x = odom.getX();
				s_pos_y = odom.getY();
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
			float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
			if (std::abs(distance) > 0.02f)
			{
				wheel.stop();
				cliff_resume_cnt_++;
				if (cliff_resume_cnt_ < 5)
					ROS_WARN("%s %d: Resume failed, try cliff resume for the %d time.",
							 __FUNCTION__, __LINE__, cliff_resume_cnt_ + 1);
				s_pos_x = odom.getX();
				s_pos_y = odom.getY();
			}
		}
		else
		{
			ROS_WARN("%s %d: Cliff jamed.", __FUNCTION__, __LINE__);
			ev.fatal_quit = true;
			error.set(ERROR_CODE_CLIFF);
		}
	}
	else if (ev.bumper_jam)
	{
		if (!bumper.getStatus())
		{
			ROS_INFO("%s %d: Bumper resume succeeded.", __FUNCTION__, __LINE__);
			ev.bumper_jam = false;
			ev.bumper_triggered = 0;
			g_bumper_cnt = 0;
		}
		else
		{
			switch (bumper_jam_state_)
			{
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				case 3: // Move back for the third time.
				{
					float distance = two_points_distance_double(s_pos_x, s_pos_y, odom.getX(), odom.getY());
					if (std::abs(distance) > 0.05f)
					{
						wheel.stop();
						// If cliff jam during bumper self resume.
						if (cliff.getStatus() && ++g_cliff_cnt > 2)
						{
							ev.cliff_jam = true;
							bumper_jam_state_ = 1;
							wheel_resume_cnt_ = 0;
						} else
						{
							bumper_jam_state_++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state_);
							if (bumper_jam_state_ == 4)
								resume_wheel_start_time_ = ros::Time::now().toSec();
						}
						s_pos_x = odom.getX();
						s_pos_y = odom.getY();
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
						ev.cliff_jam = true;
						bumper_jam_state_ = 1;
						wheel_resume_cnt_ = 0;
					} else if (ros::Time::now().toSec() - resume_wheel_start_time_ >= 2)
					{
						bumper_jam_state_++;
						resume_wheel_start_time_ = ros::Time::now().toSec();
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

//	if (ev.fatal_quit)
//		ROS_INFO("%s %d: ev.fatal_quit is set to %d.", __FUNCTION__, __LINE__, ev.fatal_quit);
	return ev.fatal_quit;
}
