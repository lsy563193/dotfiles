//
// Created by lsy563193 on 12/4/17.
//

#include "pp.h"
#include <arch.hpp>

StateGoHomePoint::StateGoHomePoint():gh_state_(gh_ing) {
	vacuum.setMode(Vac_Normal, false);
	brush.setSidePwm(30, 30);
	wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
	if (ev.remote_home || cm_is_go_charger())
		led.set_mode(LED_STEADY, LED_ORANGE);
	// Play wavs.
	if (ev.battrey_home)
		speaker.play(SPEAKER_BATTERY_LOW);
	speaker.play(SPEAKER_BACK_TO_CHARGER);

	if (ev.remote_home)
		g_go_home_by_remote = true;
	ev.remote_home = false;
	ev.battrey_home = false;
}

IMoveType *StateGoHomePoint::getNextMoveType() {
	return nullptr;
}
//
//State *StateGoHomePoint::getNextState() {
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
