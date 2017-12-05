//
// Created by lsy563193 on 12/4/17.
//

#include "pp.h"
#include <state.hpp>

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

bool StateGoHomePoint::isFinish() {
//	if (start == g_home_point) {
//		gh_state_ = gh_succuss;
//		return true;
//	}
//	else if (path_get_home_point_target(start, path))
//	{
//		gh_state_ = gh_faile;
////		setNext(CS_EXPLORATION);
//		return true;
//	}

	return false;
}

State *StateGoHomePoint::setNextState() {
	if(gh_state_ == gh_succuss) {
		if (g_home_point != g_zero_home || cm_turn_and_check_charger_signal()) {
//			setActionIndex(CS_GO_CHANGER);
			return new StateGoCharger;
		}
	}
	else if(gh_state_ == gh_faile)
	{
		return new StateExploration;
//		setActionIndex(CS_EXPLORATION);
	}
}

//bool StateGoHomePoint::isStop() {
//	return false;
//}
