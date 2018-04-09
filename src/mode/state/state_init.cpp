//
// Created by lsy563193 on 1/2/18.
//

#include <mode.hpp>
#include <robot.hpp>
#include <water_tank.hpp>
#include <brush.h>

#include "key_led.h"

void StateInit::init() {
	if(Mode::next_mode_i_ == Mode::cm_exploration || Mode::next_mode_i_ == Mode::md_go_to_charger)
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else if (Mode::next_mode_i_ == Mode::cm_navigation && sp_cm_->isRemoteGoHomePoint())
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else
		key_led.setMode(LED_STEADY, LED_GREEN);
	ROS_INFO("%s %d: Enter state init.", __FUNCTION__, __LINE__);
}

void StateInit::initOpenLidar() {
	if (Mode::next_mode_i_ == Mode::cm_navigation && sp_cm_->isRemoteGoHomePoint())
	{
		key_led.setMode(LED_STEADY, LED_ORANGE);
		brush.slowOperate();
		water_tank.setTankMode(WaterTank::TANK_LOW);
	}
	else
	{
		key_led.setMode(LED_STEADY, LED_GREEN);
		brush.normalOperate();
		water_tank.setTankMode(WaterTank::TANK_HIGH);
	}
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.setCleanState();
	ROS_INFO("%s %d: Enter state initOpenLidar.", __FUNCTION__, __LINE__);
}

void StateInit::initBackFromCharge() {
	key_led.setMode(LED_STEADY, LED_GREEN, 600);
	brush.slowOperate();
	water_tank.setTankMode(WaterTank::TANK_LOW);
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.setCleanState();
	ROS_INFO("%s %d: Enter state initBackFromCharge.", __FUNCTION__, __LINE__);
}

void StateInit::initForExploration() {
	key_led.setMode(LED_STEADY, LED_ORANGE, 600);
	brush.slowOperate();
	water_tank.setTankMode(WaterTank::TANK_LOW);
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.bldcSpeed(Vac_Speed_Low);
	ROS_INFO("%s %d: Enter state initForExploration.", __FUNCTION__, __LINE__);
}


