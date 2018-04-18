//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <brush.h>
#include <water_tank.hpp>

#include "gyro.h"
#include "key_led.h"
#include "wifi/wifi.h"

void StateGoToCharger::init() {
	key_led.setMode(LED_STEADY, LED_ORANGE);
	brush.slowOperate();
	water_tank.setTankMode(WaterTank::TANK_LOW);
	if(water_tank.checkEquipment(false))
	{
		water_tank.open(WaterTank::water_tank);
		water_tank.stop(WaterTank::pump);
	} else {
		vacuum.bldcSpeed(Vac_Speed_Low);
	}
	gyro.setTiltCheckingEnable(false); //disable tilt detect
	s_wifi.setWorkMode(Mode::cm_exploration);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	ROS_INFO("%s %d: Enter state go to charger init.", __FUNCTION__, __LINE__);
}
