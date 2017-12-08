//
// Created by lsy563193 on 17-12-3.
//

#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;
boost::shared_ptr<IMoveType> ACleanMode::sp_move_type_ = nullptr;

Path_t ACleanMode::passed_path_ = {};
Path_t ACleanMode::plan_path_ = {};
Cell_t ACleanMode::last_ = {};
//boost::shared_ptr<IMovement> ACleanMode::sp_movement_ = nullptr;

ACleanMode::ACleanMode() {
	g_homes.resize(1,g_zero_home);
}

//bool ACleanMode::isFinish() {
//	PP_INFO();
//	if(sp_state_ == nullptr)
//	{
//		getNextState();
//		getNextMoveType(nav_map.getCurrCell(),g_old_dir);
//	}
//
//	if(sp_state_->isFinish(this,sp_move_type_.get(),sp_action_.get() ,action_i_))
//	{
//		PP_INFO();
//		getNextState();
//		PP_INFO();
//		if(sp_state_ == nullptr)
//			return true;
//	}
//	PP_INFO();
//	return false;
//}
bool is_equal_with_angle_(const Cell_t &l, const Cell_t &r)
{
	return  l == r && std::abs(ranged_angle(l.TH - r.TH)) < 200;
}

Cell_t ACleanMode::updatePath()
{
//	PP_INFO()
	auto curr = nav_map.updatePosition();
	auto point = nav_map.getCurrPoint();
//	ROS_INFO("point(%d,%d,%d)",point.X, point.Y, curr.TH);
//	ROS_INFO("curr(%d,%d,%d)",curr.X, curr.Y, curr.TH);
//	ROS_INFO("last(%d,%d,%d)",last_.X, last_.Y, last_.TH);
	if (!is_equal_with_angle_(curr, last_)) {
//		PP_INFO()
		last_ = curr;
		auto loc = std::find_if(passed_path_.begin(), passed_path_.end(), [&](Cell_t it) {
				return is_equal_with_angle_(curr, it);
		});
		auto distance = std::distance(loc, passed_path_.end());
		if (distance == 0) {
			ROS_INFO("curr(%d,%d,%d)",curr.X, curr.Y, curr.TH);
			passed_path_.push_back(curr);
		}
		if (distance > 5) {
			passed_path_.clear();
			g_wf_reach_count++;
		}
		nav_map.saveBlocks();
//		displayPath(passed_path_);
	}
//	else
//		is_time_up = !cs.is_trapped();
	return curr;
}

void ACleanMode::genMoveAction() {
	if (action_i_ == ac_forward)
		sp_action_.reset(new MovementForward(GridMap::cellToPoint(plan_path_.back()), plan_path_));
	else if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
		sp_action_.reset(new MovementFollowWall(GridMap::getCurrPoint(), GridMap::cellToPoint(plan_path_.back()),action_i_ == ac_follow_wall_left));
	else if (action_i_ == ac_back)
		sp_action_.reset(new MovementBack());
	else if (action_i_ == ac_turn) {
		sp_action_.reset(new MovementTurn(plan_path_.back().TH));
	}
}

//bool ACleanMode::isFinish() {
//	return false;
//}

