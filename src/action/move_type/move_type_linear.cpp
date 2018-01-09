//
// Created by lsy563193 on 12/4/17.
//
#include <event_manager.h>
#include "pp.h"
#include "arch.hpp"



MoveTypeLinear::MoveTypeLinear() {
	resetTriggeredValue();

	auto p_clean_mode = (ACleanMode*)sp_mode_;
	target_point_ = p_clean_mode->plan_path_.front();
	dir_ = p_clean_mode->new_dir_;
	turn_target_angle_ = p_clean_mode->new_dir_;
	ROS_INFO("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
	movement_i_ = mm_turn;
	sp_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
//	ROS_INFO("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
//	ROS_ERROR("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
	IMovement::sp_mt_ = this;
//	ROS_WARN("%s,%d: mt_is_linear,turn(%d)", __FUNCTION__, __LINE__, turn_target_angle_);
}

//MoveTypeLinear::MoveTypeLinearar() {
//
//}
bool MoveTypeLinear::isFinish()
{
	if (IMoveType::isFinish())
	{
		ROS_INFO("%s %d: Move type aborted.", __FUNCTION__, __LINE__);
		return true;
	}

	auto p_clean_mode = (ACleanMode*)sp_mode_;

	if (p_clean_mode->actionLinearIsFinish(this))
		return true;

	if (isLinearForward())
		switchLinearTarget(p_clean_mode);

	if (sp_movement_->isFinish()) {
		PP_INFO();

		if (movement_i_ == mm_turn) {
			PP_INFO();
			// todo: Add checking for bumper/cliff/etc.
			movement_i_ = mm_forward;
			resetTriggeredValue();
			sp_movement_.reset(new MovementFollowPointLinear());
		}
		else if (movement_i_ == mm_forward) {
			PP_INFO();
			if (ev.bumper_triggered || ev.cliff_triggered) {

				movement_i_ = mm_back;
				sp_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
			}
			else if(ev.tilt_triggered){
				movement_i_ = mm_back;
				sp_movement_.reset(new MovementBack(0.3, BACK_MAX_SPEED));
			}
			else if (ev.robot_slip)
			{
				movement_i_ = mm_back;
				sp_movement_.reset(new MovementBack(0.3, BACK_MIN_SPEED));
			}
			else {
//				resetTriggeredValue();
				return true;
			}
		}
		else {//back
//			resetTriggeredValue();
			PP_INFO();
			return true;
		}
	}
	return false;
}

MoveTypeLinear::~MoveTypeLinear()
{
//	PP_WARN();
}


bool MoveTypeLinear::isCellReach()
{
	// Checking if robot has reached target cell.
	auto s_curr_p = getPosition();
	auto target_p = target_point_;
	if (std::abs(s_curr_p.x - target_p.x) < CELL_COUNT_MUL_1_2 &&
		std::abs(s_curr_p.y - target_p.y) < CELL_COUNT_MUL_1_2)
	{
		ROS_INFO("\033[1m""%s, %d: MoveTypeLinear, reach the target cell (%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
						 target_point_.toCell().x, target_point_.toCell().y);
//		g_turn_angle = ranged_angle(new_dir - robot::instance()->getWorldPoseAngle());
		return true;
	}

	return false;
}

bool MoveTypeLinear::isPoseReach()
{
	// Checking if robot has reached target cell and target angle.
//	PP_INFO();
	auto target_angle = target_point_.th;
	if (isCellReach() && std::abs(ranged_angle(robot::instance()->getWorldPoseAngle() - target_angle)) < 200)
	{
		ROS_INFO("\033[1m""%s, %d: MoveTypeLinear, reach the target cell and pose(%d,%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
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
	auto target_p = (target_point_);
	auto target = (isXAxis(dir)) ? target_p.x : target_p.y;
	if ((isPos(dir) && (curr > target + CELL_COUNT_MUL / 4)) ||
		(!isPos(dir) && (curr < target - CELL_COUNT_MUL / 4)))
	{
		ROS_WARN("%s, %d: MoveTypeLinear, pass target: dir(\033[32m%d\033[0m),is_x_axis(\033[32m%d\033[0m),is_pos(\033[32m%d\033[0m),curr(\033[32m%d\033[0m),target(\033[32m%d\033[0m)",
				 __FUNCTION__, __LINE__, dir, isXAxis(dir), isPos(dir), curr, target);
		ROS_INFO("%s,%s,%d,\033[32m curr_cell(%d,%d),target_cell(%d,%d)\033[0m",__FILE__,__FUNCTION__,__LINE__,s_curr_p.toCell().x,s_curr_p.toCell().y,target_p.toCell().x,target_p.toCell().y);
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
		auto &target_xy = (isXAxis(p_clean_mode->new_dir_)) ? target_point_.x : target_point_.y;
		auto curr_xy = (isXAxis(p_clean_mode->new_dir_)) ? getPosition().x : getPosition().y;

		if (abs(target_xy - curr_xy) < LINEAR_NEAR_DISTANCE) {
			p_clean_mode->old_dir_ = p_clean_mode->new_dir_;
			p_clean_mode->new_dir_ = p_clean_mode->plan_path_.front().th;
			dir_ = p_clean_mode->new_dir_;
			p_clean_mode->plan_path_.pop_front();
			target_point_ = p_clean_mode->plan_path_.front();

			ROS_INFO("%s,%d,next target_point(%d,%d), dir(%d)",
					 __FUNCTION__,__LINE__,target_point_.toCell().x,target_point_.toCell().y, p_clean_mode->new_dir_);
		}
	}
}

