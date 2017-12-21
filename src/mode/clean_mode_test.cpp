//
// Created by austin on 17-12-21.
//

#include <event_manager.h>
#include "dev.h"
#include <error.h>
#include <global.h>
#include "arch.hpp"

CleanModeTest::CleanModeTest()
{
	ROS_WARN("%s %d: Entering Test mode\n=========================" , __FUNCTION__, __LINE__);
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	IMoveType::sp_mode_.reset(this);

	speaker.play(VOICE_TEST_MODE);
}

bool CleanModeTest::mapMark()
{
	return false;
}

bool CleanModeTest::isFinish()
{
	if (!sp_action_->isFinish())
		return false;

	setNextAction();

	return false;
}

bool CleanModeTest::setNextAction()
{
	if(action_i_ == ac_open_gyro)
		action_i_ = ac_check_bumper;
	else if(action_i_ == ac_check_bumper)
		action_i_ = ac_bumper_hit_test;
	else if(action_i_ == ac_bumper_hit_test)
		action_i_ = ac_check_bumper;

	genNextAction();
	PP_INFO();
	return action_i_ != ac_null;
}

void CleanModeTest::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean pressed.", __FUNCTION__, __LINE__);
	ev.key_clean_pressed = true;
	key.resetTriggerStatus();
}

