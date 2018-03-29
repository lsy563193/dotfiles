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
		ev.remote_direction_forward = false;
		ev.remote_direction_left = false;
		ev.remote_direction_right = false;
		movement_i_ = mm_stay;
		p_movement_.reset(new MovementStayRemote(15));
	}

	if (p_movement_->isFinish())
	{
		if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered) {
			ev.bumper_triggered = 0;
			ev.cliff_triggered = 0;
			ev.tilt_triggered =0;
			movement_i_ = mm_back;
			auto back_distance = (ev.tilt_triggered/* || gyro.getAngleR() > 5*/) ? TILT_BACK_DISTANCE : 0.01;
//		if(gyro.getAngleR() > 5)
//			beeper.beepForCommand(VALID);
			p_movement_.reset(new MovementBack(back_distance, BACK_MAX_SPEED));
		}
		else if (movement_i_ == mm_stay)
		{
			if (ev.remote_direction_forward)
			{
				ev.remote_direction_forward = false;
				movement_i_ = mm_straight;
				bool slow_down = true;
				p_movement_.reset(new MovementDirectGo(slow_down));
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
			if (ev.rcon_status)
			{
				ev.rcon_status = 0;
				movement_i_ = mm_back;
				p_movement_.reset(new MovementBack(0.05, BACK_MAX_SPEED));
			}
			else
			{
				ev.remote_direction_forward = false;
				ev.remote_direction_left = false;
				ev.remote_direction_right = false;
				movement_i_ = mm_stay;
				p_movement_.reset(new MovementStayRemote(15));
			}
		}
		else if (movement_i_ == mm_turn || movement_i_ == mm_back)
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
