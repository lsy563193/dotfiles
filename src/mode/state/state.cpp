//
// Created by lsy563193 on 12/4/17.
//
#include <state.hpp>
#include <action.hpp>
#include <mode.hpp>

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
		return sp_cm_->isSwitchByEventInStateSpot();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->isSwitchByEventInStateTrapped();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->isSwitchByEventInStateExceptionResume();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->isSwitchByEventInStateExploration();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->isSwitchByEventInStateResumeLowBatteryCharge();

	else if(sp_cm_->isStateCharge())
		return sp_cm_->isSwitchByEventInStateCharge();

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
		return sp_cm_->updateActionInStateSpot();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->updateActionInStateTrapped();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->updateActionInStateExceptionResume();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->updateActionInStateExploration();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->updateActionInStateResumeLowBatteryCharge();

	else if(sp_cm_->isStateCharge())
		return sp_cm_->updateActionStateCharge();

	else if(sp_cm_->isStatePause())
		return sp_cm_->updateActionInStatePause();
}

void State::switchState() {
	if(sp_cm_->isStateInit())
		sp_cm_->switchInStateInit();

	else if(sp_cm_->isStateClean())
		sp_cm_->switchInStateClean();

	else if(sp_cm_->isStateGoHomePoint())
		sp_cm_->switchInStateGoHomePoint();

	else if(sp_cm_->isStateGoCharger())
		sp_cm_->switchInStateGoToCharger();

	else if(sp_cm_->isStateTmpSpot())
		sp_cm_->switchInStateSpot();

	else if(sp_cm_->isStateTrapped())
		sp_cm_->switchInStateTrapped();

	else if(sp_cm_->isStateExceptionResume())
		sp_cm_->switchInStateExceptionResume();

	else if(sp_cm_->isStateExploration())
		sp_cm_->switchInStateExploration();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		sp_cm_->switchInStateResumeLowBatteryCharge();

	else if(sp_cm_->isStateCharge())
		sp_cm_->switchInStateCharge();
	else if(sp_cm_->isStatePause())
		sp_cm_->switchInStatePause();
}
