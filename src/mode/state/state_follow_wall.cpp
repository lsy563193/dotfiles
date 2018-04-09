//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>

#include <robot_timer.h>
#include <water_tank.hpp>
#include <brush.h>
#include "key_led.h"

void StateFolllowWall::init() {
	if(getMode()->isExpMode())
	{
		key_led.setMode(LED_STEADY,LED_ORANGE);
		brush.slowOperate();
		water_tank.setTankMode(WaterTank::TANK_LOW);
		water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.bldcSpeed(Vac_Speed_Low);
	}
	else
	{
		key_led.setMode(LED_STEADY,LED_GREEN);
		brush.normalOperate();
		water_tank.setTankMode(WaterTank::TANK_HIGH);
		water_tank.checkEquipment(false) ? water_tank.open(WaterTank::tank_pump) : vacuum.setCleanState();
	}
	robot_timer.initTrapTimer();
	ROS_INFO("%s %d: Enter state follow wall init.", __FUNCTION__, __LINE__);
}

//bool StateFolllowWall::isFinish() {
//	return false;
//}

