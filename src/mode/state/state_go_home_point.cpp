//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include "dev.h"
#include <event_manager.h>

void StateGoHomePoint::init(){
	if (!water_tank.checkEquipment(true))
		vacuum.setTmpMode(Vac_Normal);
	wheel.stop();
	if(!sp_cm_->isExpMode())
		brush.normalOperate();

	if(sp_cm_->isGoHomePointForLowPower());
	{
		brush.slowOperate();
		water_tank.stop(WaterTank::tank_pump);
	}

	wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
	if (sp_cm_->isRemoteGoHomePoint() || sp_cm_->isExpMode())
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else if(key_led.getColor() != LED_ORANGE)
		key_led.setMode(LED_STEADY, LED_GREEN);

	ev.remote_home = false;
	ev.battery_home = false;
}

//
//State *StateGoHomePoint::setNextState() {
//	if(gh_state_ == gh_succuss) {
//		if (g_home_point != g_zero_home || cm_turn_and_check_charger_signal()) {
////			curr(CS_GO_CHANGER);
//			return new StateGoCharger;
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
