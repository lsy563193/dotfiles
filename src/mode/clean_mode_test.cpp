//
// Created by austin on 17-12-21.
//

#include <event_manager.h>
#include <error.h>
#include "dev.h"
#include "mode.hpp"
CleanModeTest::CleanModeTest()
{
	ROS_WARN("%s %d: Entering Test mode\n=========================" , __FUNCTION__, __LINE__);
	speaker.play(VOICE_TEST_MODE, false);
}

CleanModeTest::~CleanModeTest()
{
	ROS_WARN("%s %d: Exit Test mode\n=========================" , __FUNCTION__, __LINE__);
	if (test_state_ != -1)
		speaker.play(VOICE_TEST_SUCCESS, false);
	else
		speaker.play(VOICE_TEST_FAIL, false);
}

bool CleanModeTest::isExit()
{
	if (sp_state == state_init)
	{
		if (action_i_ == ac_open_lidar && sp_action_->isTimeUp())
		{
			error.set(ERROR_CODE_LIDAR);
			setNextMode(md_idle);
			ev.fatal_quit = true;
			return true;
		}
	}
	if (ev.fatal_quit || sp_action_->isExit())
	{
		ROS_WARN("%s %d: Exit for ev.fatal_quit or sp_action_->isExit()", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if(ev.key_clean_pressed || ev.key_long_pressed){
		ev.key_clean_pressed = false;
		ROS_WARN("%s %d: Exit for remote key or clean key or long press clean key.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}
}

// For handlers.
void CleanModeTest::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean pressed.", __FUNCTION__, __LINE__);
	ev.key_clean_pressed = true;
	key.resetTriggerStatus();
}

void CleanModeTest::remoteDirectionForward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote forward pressed.", __FUNCTION__, __LINE__);
	ev.remote_direction_forward = true;
	remote.reset();
}

// State init.
void CleanModeTest::switchInStateInit()
{
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_desk_test;
	sp_state->init();
}

// State desk test.
bool CleanModeTest::updateActionInStateDeskTest()
{
	if (test_state_ == 0)
	{
		action_i_ = ac_desk_test;
		genNextAction();
		test_state_++;
		return true;
	}
	else
		return false;
}

void CleanModeTest::switchInStateDeskTest()
{
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = nullptr;
}
