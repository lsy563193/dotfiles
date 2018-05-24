//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <water_tank.hpp>
#include <robot.hpp>
#include <gyro.h>
#include "key_led.h"
#include "brush.h"
#include "wifi/wifi.h"
#include "vacuum.h"

void StateClean::init()
{
	key_led.setMode(LED_STEADY, LED_GREEN);
	brush.normalOperate();
	water_tank.setCurrentSwingMotorMode(water_tank.getUserSetSwingMotorMode());
	water_tank.setCurrentPumpMode(water_tank.getUserSetPumpMode());
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor_and_pump)
								: vacuum.setSpeedByUserSetMode();
	s_wifi.setWorkMode(robot::instance()->getRobotWorkMode());
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	gyro.setTiltCheckingEnable(true);
	ROS_INFO("%s %d: Enter state clean.", __FUNCTION__, __LINE__);
}

//bool StateClean::isFinish() {
//	return path_next_nav(sp_action_->, path);
//}

//State *StateClean::setNextState() {
////	auto path = sp_mode_->generatePath(nav_map,nav_map.getCurrCell(),g_old_dir);
////	if(path.empty()) {
////		ROS_INFO("%s%d:", __FUNCTION__, __LINE__);
////		if (sp_mode_->checkTrapped(nav_map, nav_map.getCurrCell())) {
////			return new StateTrapped;
////		}
////		else {
////			return new StateGoHomePoint;
////		}
////	}
//	return this;
//
//}

