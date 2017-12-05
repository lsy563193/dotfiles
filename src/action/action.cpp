//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "action.hpp"

int IAction::s_index_ = ac_null;
Mode* IAction::sp_mode_ = nullptr;

IAction *IAction::getNextAction() {
	if(s_index_ == ac_open_gyro)
		sp_mode_->getNextActionOpenGyro();
	else if(s_index_ == ac_back_form_charger)
		sp_mode_->getNextActionBackFromCharger();
	else if(s_index_ == ac_open_lidar)
		sp_mode_->getNextActionOpenLidar();
	else if(s_index_ == ac_align)
		sp_mode_->getNextActionAlign();
	else if(s_index_ == ac_open_slam)
		sp_mode_->getNextActionOpenSlam();
	else {
		s_index_ = ac_null;
		return nullptr;
	}
}

