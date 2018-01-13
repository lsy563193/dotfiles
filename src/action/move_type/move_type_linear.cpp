//
// Created by lsy563193 on 12/4/17.
//
#include <event_manager.h>
#include "pp.h"
#include "arch.hpp"



MoveTypeLinear::MoveTypeLinear() {
	resetTriggeredValue();
	auto p_mode = ((ACleanMode*)sp_mode_);
	auto target_point_ = p_mode->plan_path_.front();
	turn_target_angle_ = p_mode->new_dir_;
	ROS_INFO("%s,%d: Enter move type linear, turn target angle(%d), first target(%d, %d).",
			 __FUNCTION__, __LINE__, turn_target_angle_, target_point_.toCell().x, target_point_.toCell().y);
	movement_i_ = mm_turn;
	sp_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
//	ROS_INFO("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
//	ROS_ERROR("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
	IMovement::sp_mt_ = this;
//	ROS_WARN("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
}

MoveTypeLinear::~MoveTypeLinear()
{
	ROS_INFO("%s %d: Exit move type linear.", __FUNCTION__, __LINE__);
}

bool MoveTypeLinear::isFinish()
{
	if (IMoveType::isFinish())
	{
		ROS_INFO("%s %d: Move type aborted.", __FUNCTION__, __LINE__);
		return true;
	}

	auto p_clean_mode = (ACleanMode*)sp_mode_;

	if (p_clean_mode->MoveTypeLinearIsFinish(this))
		return true;

	if (isLinearForward())
		switchLinearTarget(p_clean_mode);

	if (sp_movement_->isFinish()) {

		if(movement_i_ == mm_turn)
		{
			if (!handleMoveBackEvent(p_clean_mode))
			{
				movement_i_ = mm_forward;
				resetTriggeredValue();
				sp_movement_.reset(new MovementFollowPointLinear());
			}
		}
		else if (movement_i_ == mm_forward)
		{
			if (!handleMoveBackEvent(p_clean_mode))
				return true;
		}
		else //if (movement_i_ == mm_back)
			return true;
	}
	return false;
}

bool MoveTypeLinear::isCellReach()
{
	// Checking if robot has reached target cell.
	auto s_curr_p = getPosition();
	auto p_clean_mode = (ACleanMode*)sp_mode_;
	auto target_point_ = p_clean_mode->plan_path_.front();
	if (std::abs(s_curr_p.x - target_point_.x) < CELL_COUNT_MUL_1_2 &&
		std::abs(s_curr_p.y - target_point_.y) < CELL_COUNT_MUL_1_2)
	{
		ROS_INFO("%s, %d: MoveTypeLinear, reach the target cell (%d,%d), current angle(%d), target angle(%d).", __FUNCTION__, __LINE__,
						 target_point_.toCell().x, target_point_.toCell().y, s_curr_p.th, target_point_.th);
//		g_turn_angle = ranged_angle(new_dir - robot::instance()->getWorldPoseAngle());
		return true;
	}

	return false;
}

bool MoveTypeLinear::isPoseReach()
{
	// Checking if robot has reached target cell and target angle.
//	PP_INFO();
	auto target_point_ = ((ACleanMode*)sp_mode_)->plan_path_.front();
	if (isCellReach() && std::abs(getPosition().angleDiffPoint(target_point_)) < 200)
	{
		ROS_INFO("\033[1m""%s, %d: MoveTypeLinear, reach the target cell and pose(%d,%d,%d)""\033[0m", __FUNCTION__, __LINE__,
				 target_point_.toCell().x, target_point_.toCell().y, target_point_.th);
		return true;
	}
	return false;
}

bool MoveTypeLinear::isPassTargetStop(int &dir)
{
//	PP_INFO();
	// Checking if robot has reached target cell.
	auto s_curr_p = getPosition();
	auto curr = (isXAxis(dir)) ? s_curr_p.x : s_curr_p.y;
	auto target_point_ = ((ACleanMode*)sp_mode_)->plan_path_.front();
	auto target = (isXAxis(dir)) ? target_point_.x : target_point_.y;
	if ((isPos(dir) && (curr > target + CELL_COUNT_MUL / 4)) ||
		(!isPos(dir) && (curr < target - CELL_COUNT_MUL / 4)))
	{
		ROS_WARN("%s, %d: MoveTypeLinear, pass target: dir(\033[32m%d\033[0m),is_x_axis(\033[32m%d\033[0m),is_pos(\033[32m%d\033[0m),curr(\033[32m%d\033[0m),target(\033[32m%d\033[0m)",
				 __FUNCTION__, __LINE__, dir, isXAxis(dir), isPos(dir), curr, target);
		ROS_INFO("%s,%s,%d,\033[32m curr_cell(%d,%d),target_cell(%d,%d)\033[0m",__FILE__,__FUNCTION__,__LINE__,s_curr_p.toCell().x,s_curr_p.toCell().y,target_point_.toCell().x,target_point_.toCell().y);
		return true;
	}
	return false;
}

bool MoveTypeLinear::isLinearForward()
{
	return movement_i_ == mm_forward;
}

void MoveTypeLinear::switchLinearTarget(ACleanMode * p_clean_mode)
{
	if (p_clean_mode->plan_path_.size() > 1)
	{
		auto target_point_ = p_clean_mode->plan_path_.front();
		auto &target_xy = (isXAxis(p_clean_mode->new_dir_)) ? target_point_.x : target_point_.y;
		auto curr_xy = (isXAxis(p_clean_mode->new_dir_)) ? getPosition().x : getPosition().y;

		if (abs(target_xy - curr_xy) < LINEAR_NEAR_DISTANCE) {
			p_clean_mode->old_dir_ = p_clean_mode->new_dir_;
			p_clean_mode->new_dir_ = p_clean_mode->plan_path_.front().th;
			p_clean_mode->plan_path_.pop_front();
//			target_point_ = p_clean_mode->plan_path_.front();

//			ROS_INFO("%s,%d,next target_point(%d,%d), dir(%d)",
//					 __FUNCTION__,__LINE__,target_point_.toCell().x,target_point_.toCell().y, p_clean_mode->new_dir_);
		}
	}
}

bool MoveTypeLinear::handleMoveBackEvent(ACleanMode *p_clean_mode)
{
	if (ev.bumper_triggered || ev.cliff_triggered)
	{
		p_clean_mode->actionLinearSaveBlocks();
		movement_i_ = mm_back;
		sp_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
		return true;
	}
	else if(ev.tilt_triggered){
		p_clean_mode->actionLinearSaveBlocks();
		movement_i_ = mm_back;
		sp_movement_.reset(new MovementBack(0.3, BACK_MAX_SPEED));
		return true;
	}
	else if (ev.robot_slip)
	{
		p_clean_mode->actionLinearSaveBlocks();
		movement_i_ = mm_back;
		sp_movement_.reset(new MovementBack(0.3, BACK_MIN_SPEED));
		return true;
	}

	return false;
}

