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
	speaker.play(VOICE_SOFTWARE_VERSION_UNOFFICIAL, false);
	lidar.slipCheckingCtrl(OFF);
	event_manager_set_enable(false);
	test_mode_ = mode;
	plan_path_.clear();
	Point_t tmp_point{0, 0};
	plan_path_.push_front(tmp_point);
	iterate_point_ = plan_path_.begin();
	switch (test_mode_)
	{
		case DESK_TEST_CURRENT_MODE:
		case DESK_TEST_MOVEMENT_MODE:
		case DESK_TEST_WRITE_BASELINE_MODE:
		{
			serial.setWorkMode(DESK_TEST_CURRENT_MODE);
			sp_state = state_test.get();
			sp_state->init();
			action_i_ = ac_desk_test;
			genNextAction();

			// Init for map in case follow wall move type will not reach the map edge.
			clean_map_.setCell(CLEAN_MAP, MAP_SIZE / 3, MAP_SIZE / 3, CLEANED);
			clean_map_.setCell(CLEAN_MAP, -MAP_SIZE / 3, -MAP_SIZE / 3, CLEANED);

			break;
		}
		case GYRO_TEST_MODE:
		{
//			serial.setWorkMode(GYRO_TEST_MODE);
			serial.setWorkMode(WORK_MODE);
			sp_state = state_test.get();
			sp_state->init();
			action_i_ = ac_gyro_test;
			genNextAction();
			break;
		}
		case WATER_TANK_TEST_MODE:
		{
			serial.setWorkMode(WATER_TANK_TEST_MODE);
//			serial.setWorkMode(WORK_MODE);
			sp_state = state_test.get();
			sp_state->init();
			action_i_ = ac_water_tank_test;
			genNextAction();
			break;
		}
		case LIFE_TEST_MODE:
		{
			serial.setWorkMode(LIFE_TEST_MODE);
			sp_state = state_test.get();
			sp_state->init();
			action_i_ = ac_life_test;
			genNextAction();
			break;
		}
		case R16_AND_LIDAR_TEST_MODE:
		{
			serial.setWorkMode(R16_AND_LIDAR_TEST_MODE);
			sp_state = state_test.get();
			sp_state->init();
			action_i_ = ac_r16_test;
			genNextAction();
			break;
		}
		case BUMPER_TEST_MODE:
		{
			serial.setWorkMode(BUMPER_TEST_MODE);
			sp_state = state_test.get();
			sp_state->init();
			action_i_ = ac_bumper_hit_test;
			genNextAction();
			break;
		}
	}
}

CleanModeTest::~CleanModeTest()
{
	ROS_WARN("%s %d: Exit Test mode\n=========================" , __FUNCTION__, __LINE__);
/*	if (test_state_ != -1)
		speaker.play(VOICE_TEST_SUCCESS, false);
	else
		speaker.play(VOICE_TEST_FAIL, false);*/
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

// State test.
bool CleanModeTest::updateActionInStateTest()
{
	action_i_ = ac_desk_test;
	genNextAction();
	return true;
}

void CleanModeTest::switchInStateTest()
{
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = nullptr;
}

