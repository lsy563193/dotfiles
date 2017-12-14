//
// Created by lsy563193 on 12/4/17.
//
#include "pp.h"
#include "arch.hpp"

//bool MoveTypeGoToCharger::isFinish() {
//	return false;
//}

//IAction *MoveTypeGoToCharger::setNextAction() {
//	return nullptr;
//}

MoveTypeGoToCharger::MoveTypeGoToCharger()
{
	ROS_INFO("%s,%d: mt_is_go_to_charger", __FUNCTION__, __LINE__);
	sp_gtc_movement_.reset(new MovementGoToCharger());
	sp_back_movement_.reset();
	sp_turn_movement_.reset();
}

bool MoveTypeGoToCharger::isFinish()
{
	if (sp_gtc_movement_->isFinish())
		return true;
	else if ((sp_back_movement_ != nullptr) && sp_back_movement_->isFinish())
		sp_back_movement_.reset();
	else if ((sp_turn_movement_ != nullptr) && sp_turn_movement_->isFinish())
		sp_turn_movement_.reset();
	else if (sp_gtc_movement_->isSwitch())
	{
		int16_t turn_angle;
		sp_gtc_movement_->getTurnBackInfo(turn_angle, back_distance_);
		if (back_distance_ != 0)
			sp_back_movement_.reset(new MovementBack(back_distance_));
		if (turn_angle != 0)
		{
			turn_target_angle_ = robot::instance()->getPoseAngle() + turn_angle;
			sp_turn_movement_.reset(new MovementTurn(turn_target_angle_));
		}
	}

	return false;
}

void MoveTypeGoToCharger::run()
{
//	PP_INFO();
	if (sp_back_movement_ != nullptr)
	{
//		PP_INFO();
		sp_back_movement_->run();
	}
	else if (sp_turn_movement_ != nullptr)
	{
//		PP_INFO();
		sp_turn_movement_->run();
	}
	else
	{
//		PP_INFO();
		sp_gtc_movement_->run();
	}
}
