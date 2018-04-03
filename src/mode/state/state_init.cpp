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
		key_led.setMode(LED_STEADY, LED_ORANGE, 600);
	else
		key_led.setMode(LED_STEADY, LED_GREEN, 600);
}

void StateInit::initBackFromCharge() {
	key_led.setMode(LED_STEADY, LED_GREEN, 600);
	brush.slowOperate();
	water_tank.setTankMode(WaterTank::TANK_LOW);
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.setCleanState();
}

void StateInit::initOpenLidar() {
	key_led.setMode(LED_STEADY, LED_GREEN, 600);
	brush.normalOperate();
	water_tank.setTankMode(WaterTank::TANK_HIGH);
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.setCleanState();
}

void StateInit::initForExploration() {
	key_led.setMode(LED_STEADY, LED_ORANGE, 600);
	brush.slowOperate();
	water_tank.setTankMode(WaterTank::TANK_LOW);
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.bldcSpeed(Vac_Speed_Low);
}



