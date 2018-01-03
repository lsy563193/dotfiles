//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>

/*bool State::setNextState() {
	State::sp_cm_->setState(getNextState());
	State::sp_cm_->getState()->update();
	return false;
}*/

/*

State *State::getNextState() {
	if(sp_cm_->isStateInit())
		return sp_cm_->getNextStateOfInit();

	else if(sp_cm_->isStateClean())
		return sp_cm_->getNextStateOfClean();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->getNextStateOfGoHomePoint();

	else if(sp_cm_->isStateGoCharger())
		return sp_cm_->getNextStateOfGoCharger();

	else if(sp_cm_->isStateTmpSpot())
		return sp_cm_->getNextStateOfTmpSpot();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->getNextStateOfTrapped();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->getNextStateOfSelfCheck();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->getNextStateOfExploration();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->getNextStateOfResumeLowBatteryCharge();

	else if(sp_cm_->isStateSavedBeforePause())
		return sp_cm_->getNextStateOfSavedBeforePause();
}
*/

ACleanMode* State::sp_cm_{};
bool State::isFinish()
{
	if(sp_cm_->isStateInit())
	{
		ROS_INFO("!!isStateInit");
		return sp_cm_->isFinishInit();
	}

	else if(sp_cm_->isStateClean())
		return sp_cm_->isFinishClean();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->isFinishGoHomePoint();

	else if(sp_cm_->isStateGoCharger())
		return sp_cm_->isFinishGoCharger();

	else if(sp_cm_->isStateTmpSpot())
		return sp_cm_->isFinishTmpSpot();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->isFinishTrapped();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->isFinishExceptionResume();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->isFinishExploration();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->isFinishResumeLowBatteryCharge();

	else if(sp_cm_->isStateSavedBeforePause())
		return sp_cm_->isFinishLowBatteryResume();
	return false;
}
