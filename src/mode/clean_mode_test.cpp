//
// Created by austin on 17-12-21.
//

#include <event_manager.h>
#include "dev.h"
#include <error.h>
#include "mode.hpp"
CleanModeTest::CleanModeTest()
{
	ROS_WARN("%s %d: Entering Test mode\n=========================" , __FUNCTION__, __LINE__);
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	IMoveType::sp_mode_ = this;

	speaker.play(VOICE_TEST_MODE);
}

CleanModeTest::~CleanModeTest()
{
	IMoveType::sp_mode_ = nullptr;
	event_manager_set_enable(false);
}

bool CleanModeTest::isFinish()
{
	if (!sp_action_->isFinish() && !ev.remote_direction_back && !ev.remote_direction_forward)
		return false;

	return false;
}

/*bool CleanModeTest::setNextAction()
{
	if (ev.remote_direction_back)
	{
		ev.remote_direction_back = false;
		led.set_mode(LED_STEADY, LED_OFF);
		wheel.stop();
		brush.stop();
		vacuum.stop();
		action_i_ = ac_check_vacuum;
	}
	else if (ev.remote_direction_forward)
	{
		ev.remote_direction_forward = false;
		led.set_mode(LED_STEADY, LED_GREEN);
		wheel.stop();
		brush.stop();
		vacuum.stop();
		action_i_ = ac_movement_direct_go;
	}
	else if (action_i_ == ac_open_gyro || action_i_ == ac_bumper_hit_test || action_i_ == ac_check_vacuum || action_i_ == ac_movement_direct_go)
	{
		led.set_mode(LED_STEADY, LED_OFF);
		wheel.stop();
		brush.stop();
		vacuum.stop();
		action_i_ = ac_check_bumper;
	}
	else if (action_i_ == ac_check_bumper)
	{
		led.setMode(LED_STEADY, LED_GREEN);
		wheel.stop();
		brush.stop();
		vacuum.stop();
		action_i_ = ac_bumper_hit_test;
	}

	genNextAction();
	PP_INFO();
	return action_i_ != ac_null;
}*/

void CleanModeTest::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean pressed.", __FUNCTION__, __LINE__);
	ev.key_clean_pressed = true;
	key.resetTriggerStatus();
}

void CleanModeTest::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max pressed.", __FUNCTION__, __LINE__);
	ev.remote_direction_back = true;
	remote.reset();
}

void CleanModeTest::remoteDirectionForward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote forward pressed.", __FUNCTION__, __LINE__);
	ev.remote_direction_forward = true;
	remote.reset();
}
