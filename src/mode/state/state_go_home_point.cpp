//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include "dev.h"
#include <event_manager.h>

void StateGoHomePoint::init(){
	wheel.stop();
	wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);

	if (robot::instance()->getRobotWorkMode() == Mode::cm_exploration || sp_cm_->isRemoteGoHomePoint()
		|| sp_cm_->isGoHomePointForLowBattery() || sp_cm_->isWifiGoHomePoint() || !sp_cm_->isFirstTimeGoHomePoint())
	{
		key_led.setMode(LED_STEADY, LED_ORANGE);
		brush.slowOperate();
		water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_LOW);
		water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
									: vacuum.setForCurrentMaxMode(false);
		s_wifi.setWorkMode(Mode::cm_exploration);
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	}
	else
	{
		key_led.setMode(LED_STEADY, LED_GREEN);
		brush.normalOperate();
		water_tank.setCurrentSwingMotorMode(water_tank.getUserSetSwingMotorMode());
		water_tank.setCurrentPumpMode(water_tank.getUserSetPumpMode());
		water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor_and_pump)
									: vacuum.setSpeedByUserSetMode();
	}

	ev.remote_home = false;
	ev.battery_home = false;
	ROS_INFO("%s %d: Enter state go home point init.", __FUNCTION__, __LINE__);
}

//
//State *StateGoHomePoint::setNextState() {
//	if(gh_state_ == gh_succuss) {
//		if (g_home_point != g_zero_home || cm_turn_and_check_charger_signal()) {
////			curr(CS_GO_CHANGER);
//			return new StateGoToCharger;
//		}
//	}
//	else if(gh_state_ == gh_faile)
//	{
//		return new StateExploration;
////		curr(CS_EXPLORATION);
//	}
//	return this;
//}

//bool StateGoHomePoint::isStop() {
//	return false;
//}
