//
// Created by austin on 18-1-26.
//

#include <event_manager.h>
#include "move_type.hpp"
#include <robot.hpp>

int MoveTypeRemote::start_command_ = MoveTypeRemote::command_type::start_null;

MoveTypeRemote::MoveTypeRemote()
{
	IMovement::sp_mt_ = this;
	ROS_WARN("%s,%d: Enter.", __FUNCTION__, __LINE__);
	switch (start_command_)
	{
		case command_type::start_forward:
		{
			goForward();
			break;
		}
		case command_type::start_left:
		{
			turnLeft();
			break;
		}
		case command_type::start_right:
		{
			turnRight();
			break;
		}
		default: // command_type::start_null;
		{
			stay();
			break;
		}
	}
	ev.remote_direction_forward = false;
	ev.remote_direction_left = false;
	ev.remote_direction_right = false;
}

MoveTypeRemote::~MoveTypeRemote()
{
	start_command_ = command_type::start_null;
	ROS_WARN("%s,%d: Exit.", __FUNCTION__, __LINE__);
}

bool MoveTypeRemote::isFinish()
{
	if (movement_i_ == mm_turn &&
		(ev.remote_direction_forward || ev.remote_direction_left || ev.remote_direction_right))
	{
		// For stopping turning with remote direction keys.
		if (ev.remote_direction_forward)
			goForward();
		else if (ev.remote_direction_left)
			turnLeft();
		else if (ev.remote_direction_right)
			turnRight();
		ev.remote_direction_forward = false;
		ev.remote_direction_left = false;
		ev.remote_direction_right = false;
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
				if (movement_i_ != mm_straight)
					goForward();
				else
					stay();

			}
			else if (ev.remote_direction_left)
			{
				ev.remote_direction_left = false;
				turnLeft();
			}
			else if (ev.remote_direction_right)
			{
				ev.remote_direction_right = false;
				turnRight();
			}
			else
				stay();
		}
		else //if (movement_i_ == mm_back)
			stay();
	}

	return false;
}

void MoveTypeRemote::run()
{
	p_movement_->run();
}

void MoveTypeRemote::turnLeft()
{
	movement_i_ = mm_turn;
	p_movement_.reset(new MovementTurn(robot::instance()->getWorldPoseRadian() + degree_to_radian(30), ROTATE_TOP_SPEED));
}

void MoveTypeRemote::turnRight()
{
	movement_i_ = mm_turn;
	p_movement_.reset(new MovementTurn(robot::instance()->getWorldPoseRadian() - degree_to_radian(30), ROTATE_TOP_SPEED));
}

void MoveTypeRemote::goForward()
{
	movement_i_ = mm_straight;
	bool slow_down = true;
	p_movement_.reset(new MovementDirectGo(slow_down));
}

void MoveTypeRemote::stay()
{
	movement_i_ = mm_stay;
	p_movement_.reset(new MovementStayRemote(15));
}
