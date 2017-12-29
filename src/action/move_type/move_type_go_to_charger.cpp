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
	ROS_INFO("%s,%d: Move type is go to charger.", __FUNCTION__, __LINE__);
	p_gtc_movement_.reset(new MovementGoToCharger());
	p_back_movement_.reset();
	p_turn_movement_.reset();
}

bool MoveTypeGoToCharger::isFinish()
{
	if (p_gtc_movement_->isFinish())
		return true;
	else if (p_back_movement_ != nullptr)
	{
		if (p_back_movement_->isFinish())
			p_back_movement_.reset();
	}
	else if (p_turn_movement_ != nullptr)
	{
		if (p_turn_movement_->isFinish())
			p_turn_movement_.reset();
	}
	else if (p_gtc_movement_->isSwitch())
	{
		int16_t turn_angle;
		p_gtc_movement_->getTurnBackInfo(turn_angle, back_distance_);
		if (back_distance_ != 0)
			p_back_movement_.reset(new MovementBack(back_distance_, BACK_MAX_SPEED));
		if (turn_angle != 0)
		{
			turn_target_angle_ = robot::instance()->getWorldPoseAngle() + turn_angle;
			p_turn_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
		}
	}

	return false;
}

void MoveTypeGoToCharger::run()
{
//	PP_INFO();
	if (p_back_movement_ != nullptr)
	{
//		PP_INFO();
		p_back_movement_->run();
	}
	else if (p_turn_movement_ != nullptr)
	{
//		PP_INFO();
		p_turn_movement_->run();
	}
	else
	{
//		PP_INFO();
		p_gtc_movement_->run();
	}
}
