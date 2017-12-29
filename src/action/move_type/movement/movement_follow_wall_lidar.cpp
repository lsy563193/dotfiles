//
// Created by lsy563193 on 12/19/17.
//
#include "pp.h"
#include "arch.hpp"

#define WF_SCAN_TYPE						(2)


MovementFollowWallLidar::MovementFollowWallLidar(bool is_left)
				: IFollowWall(is_left)
{
	min_speed_ = FALL_WALL_MIN_SPEED;
	max_speed_ = FALL_WALL_MAX_SPEED;
	base_speed_ = min_speed_;
	tick_limit_ = 0;

//	path_thread_ = new boost::thread(boost::bind(&MovementFollowWallLidar::calcTmpTarget));
//	path_thread_->detach();
}

bool MovementFollowWallLidar::calcTmpTarget(Point32_t& tmp_target) {
//		tmp_target = p_mt->tmp_plan_path_.front();
//		if (std::abs(getPosition().X - tmp_target.X) < 30 && std::abs(getPosition().Y - tmp_target.Y) < 30) {
//			p_mt->tmp_plan_path_.pop_front();
//			if (p_mt->tmp_plan_path_.empty()) {
//				return false;
//			}
//			tmp_target = p_mt->tmp_plan_path_.front();
//		}
//		return true;
//	auto p_mt = (ActionFollowWall *) (sp_mt_);
	auto tmp_targets = robot::instance()->getTempTarget();

	ROS_WARN("curr_point(%d,%d)",getPosition().X, getPosition().Y);
//	if (robot::instance()->getTempTarget().empty())
//		return false;
//	tmp_target = p_mt->tmp_plan_path_.front();
	if(tmp_targets.empty())
	{
//		Points d_points = GridMap::r
//		tmp_target = getPosition() + {};
		ROS_WARN("tmp_targets is emply");
		auto dy =  is_left_ ? -1 : 1;
		GridMap::robotToPoint(getPosition(), CELL_SIZE * dy, CELL_SIZE * 1, &tmp_target.X, &tmp_target.Y);
	}
	else {
		tmp_target = tmp_targets.front();
	}

	ROS_WARN("tmp_target(%d,%d)",tmp_target.X, tmp_target.Y);
	return true;
}

bool MovementFollowWallLidar::isFinish() {
//	return false;
	auto p_mt = (ActionFollowWall*)(sp_mt_);
	return p_mt->shouldMoveBack() || p_mt->shouldTurn();
}

