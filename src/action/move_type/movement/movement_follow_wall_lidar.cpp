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

bool MovementFollowWallLidar::calcTmpTarget() {

//	ROS_WARN("curr_point(%d,%d)", getPosition().x, getPosition().y);
	auto lidar_targets = robot::instance()->getTempTarget();
	if (lidar_targets.empty()) {
		p_tmp_targets_ = &virtual_targets_;
		if(virtual_targets_.empty()){
			ROS_WARN("lidar_targets is emply");
			virtual_targets_ = _calcTmpTarget();
		}
	}
	else{
		if(std::equal(lidar_targets_old_.begin(),lidar_targets_old_.end(), lidar_targets.begin()), [](Point32_t &l,Point32_t &r){
			return l == r;
		}) {
			p_tmp_targets_ = &lidar_targets_;
//			ROS_WARN("lidar_targets init ");
			lidar_targets_ = lidar_targets;
			lidar_targets_old_ = lidar_targets;
		}
	}
//	ROS_INFO("targets:");
//	for(auto &target:*p_tmp_targets_)
//	{
//		ROS_INFO("   (%d,%d)",target.x, target.y);
//	}
	tmp_target_ = p_tmp_targets_->front();
	if (std::abs(getPosition().x - tmp_target_.x) < CELL_COUNT_MUL*0.75 && std::abs(getPosition().y - tmp_target_.y) < CELL_COUNT_MUL*0.75) {
//		ROS_INFO_FL();
		p_tmp_targets_->pop_front();
		if (p_tmp_targets_->empty()) {
			_calcTmpTarget();
			p_tmp_targets_ = &virtual_targets_;
		}
		tmp_target_ = p_tmp_targets_->front();
	}
//	ROS_WARN("tmp_target(%d,%d)", tmp_target.x, getPosition().y);
	robot::instance()->pubTmpTarget(tmp_target_);
	return true;
}

bool MovementFollowWallLidar::isFinish() {
	return sp_mt_->shouldMoveBack() || sp_mt_->shouldTurn();
}

bool MovementFollowWallLidar::is_near() {
//	if(tmp_targets.empty())
		return false;
}

