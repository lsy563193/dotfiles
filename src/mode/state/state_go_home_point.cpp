//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include "dev.h"
#include <event_manager.h>

void StateGoHomePoint::init(){
	wheel.stop();
	wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);

	if (sp_cm_->isRemoteGoHomePoint() || sp_cm_->isExpMode() || sp_cm_->isGoHomePointForLowBattery()
		|| !sp_cm_->isFirstTimeGoHomePoint())
	{
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
	}
	else
	{
		key_led.setMode(LED_STEADY, LED_GREEN);
		brush.normalOperate();
		water_tank.setTankMode(WaterTank::TANK_HIGH);
		water_tank.checkEquipment(false) ? water_tank.open(WaterTank::tank_pump) : vacuum.setCleanState();
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
