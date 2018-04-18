//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <brush.h>
#include <water_tank.hpp>
#include "wifi/wifi.h"

#include "key_led.h"

void StateExploration::init() {
	key_led.setMode(LED_STEADY, LED_ORANGE);
	brush.slowOperate();
	water_tank.setTankMode(WaterTank::TANK_LOW);
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.bldcSpeed(Vac_Speed_Low);
	s_wifi.setWorkMode(Mode::cm_exploration);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	ROS_INFO("%s %d: Enter state exploration.", __FUNCTION__, __LINE__);
}


