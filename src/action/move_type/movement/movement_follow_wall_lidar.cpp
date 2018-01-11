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
	lidar_targets_.empty();
	virtual_targets_.empty();
	p_tmp_targets_ = &virtual_targets_;

//	path_thread_ = new boost::thread(boost::bind(&MovementFollowWallLidar::calcTmpTarget));
//	path_thread_->detach();
}

Points MovementFollowWallLidar::_calcTmpTarget() {
	Point32_t tmp_target{};
	Points tmp_targets{};

	tmp_target = getPosition().getRelative(CELL_SIZE * 1, CELL_SIZE * 0);
	tmp_targets.push_back(tmp_target);
	auto dy = is_left_ ? 1 : -1;
	tmp_target = getPosition().getRelative(CELL_SIZE * 1, CELL_SIZE * dy);
	tmp_targets.push_back(tmp_target);

//	ROS_INFO("_calc tmp_targets.size(%d)",tmp_targets.size());
	return tmp_targets;
}

Point32_t MovementFollowWallLidar::calcTmpTarget() {

//	ROS_WARN("curr_point(%d,%d)", getPosition().x, getPosition().y);
	auto path_head = robot::instance()->getTempTarget();

	if (path_head.seq != seq_) {
		seq_ = path_head.seq;
		lidar_targets_ = path_head.tmp_plan_path_;
		p_tmp_targets_ = lidar_targets_.empty() ? &virtual_targets_ : &lidar_targets_;
		ROS_WARN("get_lidar_target(%d)", lidar_targets_.size());
	}

	if(p_tmp_targets_->empty()) {
		virtual_targets_ = _calcTmpTarget();
		p_tmp_targets_ = &virtual_targets_;
		INFO_PURPLE("p_tmp_targets_->empty(), use virtual target");
	}

	if (p_tmp_targets_->front().isNearTo(getPosition(), CELL_COUNT_MUL * 0.75)){
		p_tmp_targets_->pop_front();
		ROS_WARN("near pop target(%d)",p_tmp_targets_->size());
	}
//	ROS_WARN("is_virtual_target(%d,%d)", lidar_targets_.empty(),lidar_targets_.size());
	robot::instance()->pubTmpTarget(virtual_targets_, p_tmp_targets_ == &virtual_targets_ );
	return p_tmp_targets_->front();
}

bool MovementFollowWallLidar::isFinish() {
	return sp_mt_->shouldMoveBack() || sp_mt_->shouldTurn();
}

bool MovementFollowWallLidar::is_near() {
//	if(tmp_targets.empty())
		return false;
}

