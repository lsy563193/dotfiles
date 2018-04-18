//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include "wifi/wifi.h"
#include <robot_timer.h>
#include <water_tank.hpp>
#include <brush.h>
#include <robot.hpp>
#include "key_led.h"

void StateFolllowWall::init() {
	if(robot::instance()->getRobotWorkMode() == Mode::cm_exploration)
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
	s_wifi.setWorkMode(robot::instance()->getRobotWorkMode());
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	robot_timer.initTrapTimer();
	ROS_INFO("%s %d: Enter state follow wall init.", __FUNCTION__, __LINE__);
}

//bool StateFolllowWall::isFinish() {
//	return false;
//}

