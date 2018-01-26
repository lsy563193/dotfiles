//
// Created by austin on 18-1-26.
//

#include <event_manager.h>
#include <robot.hpp>
#include "move_type.hpp"

MoveTypeRemote::MoveTypeRemote()
{
	ROS_INFO("%s,%d: Enter move type remote.", __FUNCTION__, __LINE__);
	movement_i_ = mm_stay;
	p_movement_.reset(new MovementStay());
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
		ev.remote_direction_forward = false;
		ev.remote_direction_left = false;
		ev.remote_direction_right = false;
		movement_i_ = mm_stay;
		p_movement_.reset(new MovementStay());
	}

	if (p_movement_->isFinish())
	{
		if (ev.bumper_triggered || ev.cliff_triggered)
		{
			ev.bumper_triggered = 0;
			ev.cliff_triggered = 0;
			movement_i_ = mm_back;
			p_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
		}
		else if (movement_i_ == mm_stay)
		{
			if (ev.remote_direction_forward)
			{
				ev.remote_direction_forward = false;
				movement_i_ = mm_straight;
				p_movement_.reset(new MovementRemoteDirectGo());
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
			else if (p_movement_->isTimeUp())
			{
				ROS_INFO("%s %d: Movement stay timeout.", __FUNCTION__, __LINE__);
				return true;
			}
			else
				ROS_ERROR("%s %d: Movement stay should not finish except for reasons above, please check!!!"
				, __FUNCTION__, __LINE__);
		}
		else if (movement_i_ == mm_straight)
		{
			if (ev.rcon_triggered)
			{
				ev.rcon_triggered = 0;
				movement_i_ = mm_back;
				p_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
			}
			else
			{
				ev.remote_direction_forward = false;
				ev.remote_direction_left = false;
				ev.remote_direction_right = false;
				movement_i_ = mm_stay;
				p_movement_.reset(new MovementStay());
			}
		}
		else if (movement_i_ == mm_turn || movement_i_ == mm_back)
		{
			movement_i_ = mm_stay;
			p_movement_.reset(new MovementStay());
		}
	}

	return false;
}

void MoveTypeRemote::run()
{
	p_movement_->run();
}
