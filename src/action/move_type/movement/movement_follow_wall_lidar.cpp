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

typedef struct{
	int32_t r;
	Points getPoints(int precision);
}Circle;

Points Circle::getPoints(int precision)
{
	Points points1;
	Points points2;
	for(auto i=0; i<=precision; i++)
	{
		auto y = (this->r*2)/precision*i;
		auto x = static_cast<int32_t>(sqrt(pow(this->r, 2) - pow(y - this->r, 2)));
		printf("x,y(%d,%d) ",x, y);
		points1.push_back({x,y,0});
		points2.push_front({-x,y,0});
	}

	std::move(points2.begin(), points2.end(), std::back_inserter(points1));
	return points1;
}

Points MovementFollowWallLidar::_calcTmpTarget() {
	Circle circle{CELL_SIZE_3/2};

	Points tmp_targets{};
	auto d_points = circle.getPoints(10);
	for(auto& point:d_points)
	{
		if(!is_left_)
			point.y = -point.y;
		tmp_targets.push_back(getPosition().getRelative(point.x + CELL_SIZE, point.y));
	}
	return tmp_targets;
}

Point32_t MovementFollowWallLidar::calcTmpTarget() {

//	ROS_WARN("curr_point(%d,%d)", getPosition().x, getPosition().y);
	auto path_head = robot::instance()->getTempTarget();

	if (path_head.seq != seq_) {
		seq_ = path_head.seq;
		lidar_targets_ = path_head.tmp_plan_path_;
		if(!lidar_targets_.empty())
			virtual_targets_.clear();
		p_tmp_targets_ = lidar_targets_.empty() ? &virtual_targets_ : &lidar_targets_;
//		ROS_WARN("get_lidar_target(%d)", lidar_targets_.size());
	}

	if(p_tmp_targets_->empty()) {
		virtual_targets_ = _calcTmpTarget();
		p_tmp_targets_ = &virtual_targets_;
//		INFO_PURPLE("p_tmp_targets_->empty(), use virtual target");
	}

	if (p_tmp_targets_->front().isNearTo(getPosition(), CELL_COUNT_MUL * 0.75)){
		p_tmp_targets_->pop_front();
//		ROS_WARN("near pop target(%d)",p_tmp_targets_->size());
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

