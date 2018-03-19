//
// Created by austin on 17-12-21.
//

#include <event_manager.h>
#include <error.h>
#include "dev.h"
#include "mode.hpp"
CleanModeTest::CleanModeTest(uint8_t mode)
{
	ROS_WARN("%s %d: Entering Test mode\n=========================" , __FUNCTION__, __LINE__);
	speaker.play(VOICE_TEST_MODE, false);
	event_manager_set_enable(false);
	test_mode_ = mode;
	switch (test_mode_)
	{
		case DESK_TEST_CURRENT_MODE:
		{
			serial.setWorkMode(DESK_TEST_CURRENT_MODE);
			sp_state = state_test;
			sp_state->init();
			action_i_ = ac_desk_test;
			genNextAction();
			break;
		}
		case GYRO_TEST_MODE:
		{
//			serial.setWorkMode(GYRO_TEST_MODE);
			serial.setWorkMode(WORK_MODE);
			sp_state = state_test;
			sp_state->init();
			action_i_ = ac_gyro_test;
			genNextAction();
			break;
		}
		case WATER_TANK_TEST_MODE:
		{
//			serial.setWorkMode(WATER_TANK_TEST_MODE);
			serial.setWorkMode(WORK_MODE);
			sp_state = state_test;
			sp_state->init();
			action_i_ = ac_water_tank_test;
			genNextAction();
			break;
		}
		case LIFE_TEST_MODE:
		{
			serial.setWorkMode(LIFE_TEST_MODE);
			sp_state = state_test;
			sp_state->init();
			action_i_ = ac_life_test;
			genNextAction();
			break;
		}
		case R16_AND_LIDAR_TEST_MODE:
		{
			serial.setWorkMode(R16_AND_LIDAR_TEST_MODE);
			sp_state = state_test;
			sp_state->init();
			action_i_ = ac_r16_test;
			genNextAction();
			break;
		}
	}
}

CleanModeTest::~CleanModeTest()
{
	ROS_WARN("%s %d: Exit Desk Test mode\n=========================" , __FUNCTION__, __LINE__);
/*	if (test_state_ != -1)
		speaker.play(VOICE_TEST_SUCCESS, false);
	else
		speaker.play(VOICE_TEST_FAIL, false);*/
}

bool CleanModeTest::isFinish()
{
	return false;
}
bool CleanModeTest::isExit()
{
	return false;
/*	if (sp_state == state_init)
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
	}*/
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

// State desk test.
bool CleanModeTest::updateActionInStateDeskTest()
{
	action_i_ = ac_desk_test;
	genNextAction();
	return true;
}

void CleanModeTest::switchInStateDeskTest()
{
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = nullptr;
}

