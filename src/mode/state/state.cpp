//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>

ACleanMode* State::sp_cm_{};

bool State::isUpdateFinish()
{
	if(sp_cm_->isStateInit())
		return sp_cm_->isStateInitUpdateFinish();

	else if(sp_cm_->isStateClean())
		return sp_cm_->isStateCleanUpdateFinish();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->isStateGoHomePointUpdateFinish();

	else if(sp_cm_->isStateGoCharger())
		return sp_cm_->isStateGoToChargerUpdateFinish();

	else if(sp_cm_->isStateTmpSpot())
		return sp_cm_->isStateTmpSpotUpdateFinish();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->isStateTrappedUpdateFinish();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->isStateExceptionResumeUpdateFinish();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->isStateExplorationUpdateFinish();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->isStateResumeLowBatteryChargeUpdateFinish();

	else if(sp_cm_->isStatePause())
		return sp_cm_->isStatePauseUpdateFinish();

	return false;
}
