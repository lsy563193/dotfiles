//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <brush.h>
#include <water_tank.hpp>
#include "wifi/wifi.h"

#include "key_led.h"

void StateExploration::init()
{
	key_led.setMode(LED_STEADY, LED_ORANGE);
	brush.slowOperate();
	water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_LOW);
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
								: vacuum.setForCurrentMode(Vacuum::VacMode::vac_low_mode);
	s_wifi.setWorkMode(Mode::cm_exploration);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	ROS_INFO("%s %d: Enter state exploration.", __FUNCTION__, __LINE__);
}


