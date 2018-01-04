//
// Created by lsy563193 on 12/4/17.
//
#include <arch.hpp>

ACleanMode* State::sp_cm_{};

bool State::isConfirmed()
{
	if(sp_cm_->isStateInit())
		return sp_cm_->isStateInitConfirmed();

	else if(sp_cm_->isStateClean())
		return sp_cm_->isStateCleanConfirmed();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->isStateGoHomePointConfirmed();

	else if(sp_cm_->isStateGoCharger())
		return sp_cm_->isStateGoToChargerConfirmed();

	else if(sp_cm_->isStateTmpSpot())
		return sp_cm_->isStateTmpSpotConformed();

	else if(sp_cm_->isStateTrapped())
		return sp_cm_->isStateTrappedConfirmed();

	else if(sp_cm_->isStateExceptionResume())
		return sp_cm_->isStateExceptionResumeConfirmed();

	else if(sp_cm_->isStateExploration())
		return sp_cm_->isStateExplorationConfirmed();

	else if(sp_cm_->isStateResumeLowBatteryCharge())
		return sp_cm_->isStateResumeLowBatteryChargeConfirmed();

	else if(sp_cm_->isStatePause())
		return sp_cm_->isStatePauseConfirmed();

	return false;
}
