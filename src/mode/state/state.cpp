//
// Created by lsy563193 on 12/4/17.
//
#include <mode.hpp>
#include <key_led.h>

ACleanMode* State::sp_cm_{};

bool State::isSwitchByEvent() {

	if(sp_cm_->isStateInit())
		return sp_cm_->isSwitchByEventInStateInit();

	else if(sp_cm_->isStateClean())
		return sp_cm_->isSwitchByEventInStateClean();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->isSwitchByEventInStateGoHomePoint();

	else if(sp_cm_->isStateGoToCharger())
		return sp_cm_->isSwitchByEventInStateGoToCharger();

	else if(sp_cm_->isStateSpot())
		return sp_cm_->isSwitchByEventInStateSpot();

	else if(sp_cm_->isStateFollowWall())
		return sp_cm_->isSwitchByEventInStateFollowWall();

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

	else if(sp_cm_->isStateTest())
		return sp_cm_->isSwitchByEventInStateTest();
}

bool State::updateAction() {
	if(sp_cm_->isStateInit())
		return sp_cm_->updateActionInStateInit();

	else if(sp_cm_->isStateClean())
		return sp_cm_->updateActionInStateClean();

	else if(sp_cm_->isStateGoHomePoint())
		return sp_cm_->updateActionInStateGoHomePoint();

	else if(sp_cm_->isStateGoToCharger())
		return sp_cm_->updateActionInStateGoToCharger();

	else if(sp_cm_->isStateSpot())
		return sp_cm_->updateActionInStateSpot();

	else if(sp_cm_->isStateFollowWall())
		return sp_cm_->updateActionInStateFollowWall();

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

	else if(sp_cm_->isStateTest())
		return sp_cm_->updateActionInStateTest();
}

void State::switchState() {
	if(sp_cm_->isStateInit())
		sp_cm_->switchInStateInit();

	else if(sp_cm_->isStateClean())
		sp_cm_->switchInStateClean();

	else if(sp_cm_->isStateGoHomePoint())
		sp_cm_->switchInStateGoHomePoint();

	else if(sp_cm_->isStateGoToCharger())
		sp_cm_->switchInStateGoToCharger();

	else if(sp_cm_->isStateSpot())
		sp_cm_->switchInStateSpot();

	else if(sp_cm_->isStateFollowWall())
		sp_cm_->switchInStateFollowWall();

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

	else if(sp_cm_->isStateTest())
		return sp_cm_->switchInStateTest();
}

void StateSleep::init() {
	system("unturbo_cpu.sh");
	key_led.setMode(LED_STEADY, LED_OFF);
}

std::vector<State*>::iterator stable_unique(std::vector<State*>::iterator b, std::vector<State*>::iterator e)
{
	auto rst_end = remove_if(b+1, e,[&](const State* i){return i == *b;});
	if(b+1 == rst_end)
		return rst_end;
	stable_unique(b + 1, rst_end);
}
