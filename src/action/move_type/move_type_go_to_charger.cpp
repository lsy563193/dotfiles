//
// Created by lsy563193 on 12/4/17.
//
#include <move_type.hpp>
#include <robot.hpp>
#include <obs.h>
//#include <state.hpp>
//#include <mode.hpp>
//bool MoveTypeGoToCharger::isFinish() {
//	return false;
//}

//IAction *MoveTypeGoToCharger::setNextAction() {
//	return nullptr;
//}

MoveTypeGoToCharger::MoveTypeGoToCharger()
{
	ROS_WARN("%s,%d: Enter move type go to charger.", __FUNCTION__, __LINE__);
	obs.control(OFF);
	IMovement::sp_mt_ = this;
	p_gtc_movement_.reset(new MovementGoToCharger());
	p_back_movement_.reset();
	p_turn_movement_.reset();
}

MoveTypeGoToCharger::~MoveTypeGoToCharger()
{
	obs.control(ON);
	ROS_WARN("%s,%d: Exit move type go to charger.", __FUNCTION__, __LINE__);
}

bool MoveTypeGoToCharger::isFinish()
{
	if (p_gtc_movement_->isFinish())
		return true;
	else if (p_back_movement_ != nullptr)
	{
		if (p_back_movement_->isFinish())
		{
			p_back_movement_.reset();
			c_rcon.resetStatus();
		}
	}
	else if (p_turn_movement_ != nullptr)
	{
		if (p_turn_movement_->isFinish())
		{
			p_turn_movement_.reset();
			c_rcon.resetStatus();
		}
	}
	else if (p_gtc_movement_->isSwitch())
	{
		double turn_radian;
		p_gtc_movement_->getTurnBackInfo(turn_radian, back_distance_);
		if (back_distance_ != 0)
			p_back_movement_.reset(new MovementBack(back_distance_, BACK_MAX_SPEED));
		if (turn_radian != 0)
		{
			turn_target_radian_ = ranged_radian(robot::instance()->getWorldPoseRadian() + turn_radian);
			p_turn_movement_.reset(new MovementTurn(turn_target_radian_, ROTATE_TOP_SPEED));
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
