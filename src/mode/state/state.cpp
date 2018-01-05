//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>

ACleanMode* State::sp_cm_{};

bool State::isSwitchByEvent() {

	if(sp_cm_->isStateInit())
		return sp_cm_->isSwitchByEventInStateInit();

	else if(sp_cm_->isStateClean())
		return sp_cm_->isSwitchByEventInStateClean();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->isSwitchByEventInStateGoHomePoint();

	else if(sp_cm_->isStateGoCharger())
		return sp_cm_->isSwitchByEventInStateGoToCharger();

	else if(sp_cm_->isStateTmpSpot())
		return sp_cm_->isSwitchByEventInStateTmpSpot();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->isSwitchByEventInStateTrapped();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->isSwitchByEventInStateExceptionResume();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->isSwitchByEventInStateExploration();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->isSwitchByEventInStateResumeLowBatteryCharge();

	else if(sp_cm_->isStatePause())
		return sp_cm_->isSwitchByEventInStatePause();

}

bool State::updateAction() {
	if(sp_cm_->isStateInit())
		return sp_cm_->updateActionInStateInit();

	else if(sp_cm_->isStateClean())
		return sp_cm_->updateActionInStateClean();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->updateActionInStateGoHomePoint();

	else if(sp_cm_->isStateGoCharger())
		return sp_cm_->updateActionInStateGoToCharger();

	else if(sp_cm_->isStateTmpSpot())
		return sp_cm_->updateActionInStateTmpSpot();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->updateActionInStateTrapped();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->updateActionInStateExceptionResume();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->updateActionInStateExploration();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->updateActionInStateResumeLowBatteryCharge();

	else if(sp_cm_->isStatePause())
		return sp_cm_->updateActionInStatePause();
}

void State::switchState() {
	if(sp_cm_->isStateInit())
		return sp_cm_->switchInStateInit();

	else if(sp_cm_->isStateClean())
		return sp_cm_->switchInStateClean();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->switchInStateGoHomePoint();

	else if(sp_cm_->isStateGoCharger())
		return sp_cm_->switchInStateGoToCharger();

	else if(sp_cm_->isStateTmpSpot())
		return sp_cm_->switchInStateTmpSpot();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->switchInStateTrapped();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->switchInStateExceptionResume();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->switchInStateExploration();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->switchInStateResumeLowBatteryCharge();

	else if(sp_cm_->isStatePause())
		return sp_cm_->switchInStatePause();
}
