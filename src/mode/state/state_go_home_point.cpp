//
// Created by lsy563193 on 12/4/17.
//

#include "pp.h"
#include <arch.hpp>
#include <event_manager.h>

void StateGoHomePoint::init(){
	vacuum.setTmpMode(Vac_Normal);
	wheel.stop();

	wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
	if (sp_cm_->isRemoteGoHomePoint())
		led.set_mode(LED_STEADY, LED_ORANGE);
	else
		led.set_mode(LED_STEADY, LED_GREEN);

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
