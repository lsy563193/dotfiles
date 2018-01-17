//
// Created by austin on 17-12-8.
//

#include <event_manager.h>
#include <robot.hpp>
#include "dev.h"
#include "mode.hpp"

ModeRemote::ModeRemote()
{//use dynamic then you can limit using derived class member
	// TODO: Remote mode after nav is acting weird.
	ROS_INFO("%s %d: Entering remote mode\n=========================" , __FUNCTION__, __LINE__);
	event_manager_register_handler(this);
	event_manager_set_enable(true);

	led.set_mode(LED_STEADY, LED_GREEN);
	if (gyro.isOn())
	{
		sp_action_.reset(new MovementStay());
		action_i_ = ac_movement_stay;
	}
	else
	{
		sp_action_.reset(new ActionOpenGyro());
		action_i_ = ac_open_gyro;
	}
	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	robot_timer.resetPlanStatus();
	event_manager_reset_status();

	remote_mode_time_stamp_ = ros::Time::now().toSec();
}

ModeRemote::~ModeRemote()
{
	event_manager_set_enable(false);
	sp_action_.reset();

	wheel.stop();
	brush.stop();
	vacuum.stop();

	ROS_INFO("%s %d: Exit remote mode.", __FUNCTION__, __LINE__);
}

bool ModeRemote::isExit()
{
	if ((action_i_ == ac_movement_stay && sp_action_->isTimeUp()) || ev.key_clean_pressed)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (ev.key_long_pressed)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_sleep);
		return true;
	}

	return false;
}

bool ModeRemote::isFinish()
{
	if (sp_action_->isFinish() ||
			(action_i_ == ac_turn &&
				(ev.remote_direction_forward || ev.remote_direction_left || ev.remote_direction_right)
			)
		)
	{
		PP_INFO();
		sp_action_.reset(getNextAction());
		if (sp_action_ == nullptr)
		{
			setNextMode(md_idle);
			return true;
		}
	}

	return false;
}

IAction* ModeRemote::getNextAction()
{
	if (action_i_ == ac_open_gyro)
	{
		action_i_ = ac_movement_stay;
		return new MovementStay();
	}

	if (action_i_ == ac_movement_stay)
	{
		if (bumper.getStatus() || cliff.getStatus())
		{
			action_i_ = ac_back;
			return new MovementBack(0.01, BACK_MAX_SPEED);
		}
		else if (ev.remote_direction_forward)
		{
			action_i_ = ac_movement_direct_go;
			ev.remote_direction_forward = false;
			return new MovementDirectGo();
		}
		else if (ev.remote_direction_left)
		{
			action_i_ = ac_turn;
			ev.remote_direction_left = false;
			return new MovementTurn(robot::instance()->getWorldPoseYaw() + 300*PI/180.0, ROTATE_TOP_SPEED);
		}
		else if (ev.remote_direction_right)
		{
			action_i_ = ac_turn;
			ev.remote_direction_right = false;
			return new MovementTurn(robot::instance()->getWorldPoseYaw() - 300*PI/180.0, ROTATE_TOP_SPEED);
		}
	}

	else if (action_i_ == ac_movement_direct_go)
	{
		if (ev.bumper_triggered || ev.cliff_triggered)
		{
			ROS_INFO("%s %d: ev.bumper_triggered(%d), ev.cliff_triggered(%d).",
					 __FUNCTION__, __LINE__, ev.bumper_triggered, ev.cliff_triggered);
			ev.bumper_triggered = 0;
			ev.cliff_triggered = 0;
			action_i_ = ac_back;
			return new MovementBack(0.01, BACK_MAX_SPEED);
		}
		else if (ev.rcon_triggered)
		{
			ROS_INFO("%s %d: ev.rcon_triggered(%d).",
					 __FUNCTION__, __LINE__, ev.rcon_triggered);
			ev.rcon_triggered = 0;
			action_i_ = ac_back;
			return new MovementBack(0.05, BACK_MAX_SPEED);
		}
		else
		{
			ev.remote_direction_forward = false;
			ev.remote_direction_left = false;
			ev.remote_direction_right = false;
			action_i_ = ac_movement_stay;
			return new MovementStay();
		}
	}
	else if (action_i_ == ac_turn)
	{
		if (ev.bumper_triggered || ev.cliff_triggered)
		{
			ROS_INFO("%s %d: ev.bumper_triggered(%d), ev.cliff_triggered(%d).",
					 __FUNCTION__, __LINE__, ev.bumper_triggered, ev.cliff_triggered);
			ev.bumper_triggered = 0;
			ev.cliff_triggered = 0;
			action_i_ = ac_back;
			return new MovementBack(0.01, BACK_MAX_SPEED);
		}
		else
		{
			ev.remote_direction_forward = false;
			ev.remote_direction_left = false;
			ev.remote_direction_right = false;
			action_i_ = ac_movement_stay;
			return new MovementStay();
		}
	}
	else if (action_i_ == ac_back)
	{
		if (bumper.getStatus() || cliff.getStatus())
		{
			action_i_ = ac_back;
			return new MovementBack(0.01, BACK_MAX_SPEED);
		}
		else
		{
			action_i_ = ac_movement_stay;
			return new MovementStay();
		}
	}

	return nullptr;
}

void ModeRemote::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);
	beeper.play_for_command(VALID);
	ev.key_clean_pressed = true;
	remote.reset();
}

void ModeRemote::remoteDirectionForward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote forward.", __FUNCTION__, __LINE__);
	beeper.play_for_command(VALID);
	ev.remote_direction_forward = true;
	remote.reset();
}

void ModeRemote::remoteDirectionLeft(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote left.", __FUNCTION__, __LINE__);
	beeper.play_for_command(VALID);
	ev.remote_direction_left = true;
	remote.reset();
}

void ModeRemote::remoteDirectionRight(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote right.", __FUNCTION__, __LINE__);
	beeper.play_for_command(VALID);
	ev.remote_direction_right = true;
	remote.reset();
}

void ModeRemote::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);

	// Wait for key released.
	bool long_press = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);
			beeper.play_for_command(VALID);
			long_press = true;
		}
		usleep(20000);
	}

	if (long_press)
		ev.key_long_pressed = true;
	else
		ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}
