//
// Created by lsy563193 on 1/2/18.
//

#include <mode.hpp>
#include <robot.hpp>
#include <water_tank.hpp>
#include <brush.h>

#include "key_led.h"

void StateInit::init() {
	if(Mode::next_mode_i_ == Mode::cm_exploration)
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else if (Mode::next_mode_i_ == Mode::cm_navigation && sp_cm_->isRemoteGoHomePoint())
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else
		key_led.setMode(LED_STEADY, LED_GREEN);
	ROS_INFO("%s %d: Enter state init.", __FUNCTION__, __LINE__);
}

void StateInit::init2() {
	if (Mode::next_mode_i_ == Mode::cm_navigation && sp_cm_->isRemoteGoHomePoint())
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else
		key_led.setMode(LED_STEADY, LED_GREEN);
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.setCleanState();
	brush.normalOperate();
	ROS_INFO("%s %d: Enter state init2.", __FUNCTION__, __LINE__);
}

