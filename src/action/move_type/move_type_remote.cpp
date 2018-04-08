//
// Created by austin on 18-1-26.
//

#include <event_manager.h>
#include <robot.hpp>
#include <gyro.h>
#include "move_type.hpp"

MoveTypeRemote::MoveTypeRemote()
{
	ROS_INFO("%s,%d: Enter move type remote.", __FUNCTION__, __LINE__);
	movement_i_ = mm_stay;
	p_movement_.reset(new MovementStayRemote(15));
}

MoveTypeRemote::~MoveTypeRemote()
{
	ROS_INFO("%s,%d: Exit move type remote.", __FUNCTION__, __LINE__);
}

bool MoveTypeRemote::isFinish()
{
	if (movement_i_ == mm_turn &&
		(ev.remote_direction_forward || ev.remote_direction_left || ev.remote_direction_right))
	{
		// For stopping turning with remote direction keys.
		if (ev.remote_direction_forward)
		{
			ev.remote_direction_forward = false;
			movement_i_ = mm_straight;
			bool slow_down = true;
			p_movement_.reset(new MovementDirectGo(slow_down));
		} else if (ev.remote_direction_left)
		{
			ev.remote_direction_left = false;
			movement_i_ = mm_turn;
			p_movement_.reset(
					new MovementTurn(robot::instance()->getWorldPoseRadian() + degree_to_radian(30), ROTATE_TOP_SPEED));
		} else if (ev.remote_direction_right)
		{
			ev.remote_direction_right = false;
			movement_i_ = mm_turn;
			p_movement_.reset(
					new MovementTurn(robot::instance()->getWorldPoseRadian() - degree_to_radian(30), ROTATE_TOP_SPEED));
		}
	}

	if (p_movement_->isFinish())
	{
		if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered) {
			movement_i_ = mm_back;
			float back_distance;
			if (ev.tilt_triggered/* || gyro.getAngleR() > 5*/)
				back_distance = TILT_BACK_DISTANCE;
			else if (ev.cliff_triggered)
				back_distance = 0.05;
			else
				back_distance = 0.01;
			p_movement_.reset(new MovementBack(back_distance, BACK_MAX_SPEED));
			ev.bumper_triggered = 0;
			ev.cliff_triggered = 0;
			ev.tilt_triggered = false;
		}
		else if (movement_i_ == mm_straight && ev.rcon_status)
		{
			ev.rcon_status = 0;
			movement_i_ = mm_back;
			p_movement_.reset(new MovementBack(0.05, BACK_MAX_SPEED));
		}
		else if (movement_i_ == mm_stay && p_movement_->isTimeUp())
		{
			ROS_INFO("%s %d: Movement stay timeout.", __FUNCTION__, __LINE__);
			return true;
		}
		else if (movement_i_ != mm_back)
		{
			if (ev.remote_direction_forward)
			{
				ev.remote_direction_forward = false;
				bool slow_down = true;
				if (movement_i_ != mm_straight)
					p_movement_.reset(new MovementDirectGo(slow_down));
				else
				{
					p_movement_->start_timer_ = ros::Time::now().toSec();
					ROS_INFO("%s %d: Start timer is updated.", __FUNCTION__, __LINE__);
				}
				movement_i_ = mm_straight;

			}
			else if (ev.remote_direction_left)
			{
				ev.remote_direction_left = false;
				movement_i_ = mm_turn;
				p_movement_.reset(new MovementTurn(robot::instance()->getWorldPoseRadian() + degree_to_radian(30), ROTATE_TOP_SPEED));
			}
			else if (ev.remote_direction_right)
			{
				ev.remote_direction_right = false;
				movement_i_ = mm_turn;
				p_movement_.reset(new MovementTurn(robot::instance()->getWorldPoseRadian() - degree_to_radian(30), ROTATE_TOP_SPEED));
			}
			else
			{
				movement_i_ = mm_stay;
				p_movement_.reset(new MovementStayRemote(15));
			}
		}
		else //if (movement_i_ == mm_back)
		{
			movement_i_ = mm_stay;
			p_movement_.reset(new MovementStayRemote(15));
		}
	}

	return false;
}

void MoveTypeRemote::run()
{
	p_movement_->run();
}
