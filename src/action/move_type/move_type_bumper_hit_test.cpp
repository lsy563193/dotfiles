//
// Created by austin on 17-12-21.
//

#include <global.h>
#include <robot.hpp>
#include "arch.hpp"
#include "dev.h"

MoveTypeBumperHitTest::MoveTypeBumperHitTest()
{
	ROS_INFO("%s,%d: Move type is bumper hit test.", __FUNCTION__, __LINE__);
	p_direct_go_movement_.reset(new MovementDirectGo());
	p_back_movement_.reset();
	p_turn_movement_.reset();

}

bool MoveTypeBumperHitTest::isFinish()
{
	if (ev.key_clean_pressed)
	{
		ev.key_clean_pressed = false;
		return true;
	}

	if (p_back_movement_ != nullptr)
	{
		if (p_back_movement_->isFinish())
		{
			if (bumper.get_status())
				return true;
			p_back_movement_.reset();
			turn_time_stamp_ = ros::Time::now().toSec();
		}
	}
	else if (p_turn_movement_ != nullptr)
	{
		if (ros::Time::now().toSec() - turn_time_stamp_ > 0.6)
			p_turn_movement_.reset();
	}
	else if (bumper.get_status())
	{
		float back_distance = 0.08;
		p_back_movement_.reset(new MovementBack(back_distance, 40));
		if (turn_left_)
			turn_target_angle_ = ranged_angle(static_cast<int16_t>(robot::instance()->getWorldPoseAngle() + 900));
		else
			turn_target_angle_ = ranged_angle(static_cast<int16_t>(robot::instance()->getWorldPoseAngle() - 900));
		p_turn_movement_.reset(new MovementTurn(turn_target_angle_, 40));
		turn_left_ = !turn_left_;
	}

	return false;
}

void MoveTypeBumperHitTest::run()
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
		p_direct_go_movement_->run();
	}
}
