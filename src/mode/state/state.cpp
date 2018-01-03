//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>

/*bool State::setNextState() {
	State::cm->setState(getNextState());
	State::cm->getState()->update();
	return false;
}*/

/*

State *State::getNextState() {
	if(cm->isStateInit())
		return cm->getNextStateOfInit();

	else if(cm->isStateClean())
		return cm->getNextStateOfClean();

	else if(cm->isStateGoHomePoint())
		return cm->getNextStateOfGoHomePoint();

	else if(cm->isStateGoCharger())
		return cm->getNextStateOfGoCharger();

	else if(cm->isStateTmpSpot())
		return cm->getNextStateOfTmpSpot();

	else if(cm->isStateTrapped())
		return cm->getNextStateOfTrapped();

	else if(cm->isStateSelfCheck())
		return cm->getNextStateOfSelfCheck();

	else if(cm->isStateExploration())
		return cm->getNextStateOfExploration();

	else if(cm->isStateResumeLowBatteryCharge())
		return cm->getNextStateOfResumeLowBatteryCharge();

	else if(cm->isStateSavedBeforePause())
		return cm->getNextStateOfSavedBeforePause();
}
*/

bool State::isFinish() {
		if(cm->isStateInit())
		return cm->isFinishInit();

	else if(cm->isStateClean())
		return cm->isFinishClean();

	else if(cm->isStateGoHomePoint())
		return cm->isFinishGoHomePoint();

	else if(cm->isStateGoCharger())
		return cm->isFinishGoCharger();

	else if(cm->isStateTmpSpot())
		return cm->isFinishTmpSpot();

	else if(cm->isStateTrapped())
		return cm->isFinishTrapped();

	else if(cm->isStateSelfCheck())
		return cm->isFinishSelfCheck();

	else if(cm->isStateExploration())
		return cm->isFinishExploration();

	else if(cm->isStateResumeLowBatteryCharge())
		return cm->isFinishResumeLowBatteryCharge();

	else if(cm->isStateSavedBeforePause())
		return cm->isFinishLowBatteryResume();
	return false;
}
