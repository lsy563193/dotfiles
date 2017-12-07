//
// Created by lsy563193 on 17-12-3.
//

#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;
boost::shared_ptr<IMoveType> ACleanMode::sp_move_type_ = nullptr;

Path_t ACleanMode::passed_path_ = {};
Cell_t ACleanMode::last_ = {};
//boost::shared_ptr<IMovement> ACleanMode::sp_movement_ = nullptr;

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

Cell_t ACleanMode::updatePath(const Cell_t& curr)
{
	if (!is_equal_with_angle_(curr, last_)) {
		last_ = curr;
		auto loc = std::find_if(passed_path_.begin(), passed_path_.end(), [&](Cell_t it) {
				return is_equal_with_angle_(curr, it);
		});
		auto distance = std::distance(loc, passed_path_.end());
		if (distance == 0) {
			passed_path_.push_back(curr);
		}
		if (distance > 5) {
			passed_path_.clear();
			g_wf_reach_count++;
		}
		nav_map.saveBlocks();
	}
//	else
//		is_time_up = !cs.is_trapped();
	return curr;
}

