//
// Created by lsy563193 on 12/19/17.
//

#include <mode.hpp>
#include "robot.hpp"
#include "beeper.h"

#define WF_SCAN_TYPE						(2)


MovementFollowWallLidar::MovementFollowWallLidar(bool is_left)
				: IFollowWall(is_left)
{

	ROS_INFO("%s %d: Enter movement follow wall %s.", __FUNCTION__, __LINE__, is_left ? "left" : "right");
	angle_forward_to_turn_ = degree_to_radian(60);
	min_speed_ = FALL_WALL_MIN_SPEED;
	max_speed_ = FALL_WALL_MAX_SPEED;
	base_speed_ = min_speed_;
	tick_limit_ = 0;
	lidar_targets_.empty();
	virtual_targets_.empty();
	p_tmp_targets_ = &virtual_targets_;
	corner_time = ros::Time::now();
//	path_thread_ = new boost::thread(boost::bind(&MovementFollowWallLidar::calcTmpTarget));
//	path_thread_->detach();
}

typedef struct{
	float r;
	Points getPoints(int precision, bool is_inclue_zero);
}Circle;

Points Circle::getPoints(int precision, bool is_inclue_zero)
{
	Points points1;
	Points points2;
	auto init_i = is_inclue_zero ? 0 : 1;
	for(auto i=init_i; i<=precision; i++)
	{
		float y = (this->r*2)/precision*i;
		float x = sqrt(pow(this->r, 2) - pow(y - this->r, 2));
		printf("x,y(%f,%f) ",x, y);
		points1.push_back({x,y,0});
		points2.push_front({-x,y,0});
	}

	std::move(points2.begin(), points2.end(), std::back_inserter(points1));
	return points1;
}

Points MovementFollowWallLidar::_calcTmpTarget() {
	Circle circle{CELL_SIZE_3/2};
	Points tmp_targets{};
	bool is_corner_beginning;
	const auto time_lim{1};
	auto time_diff = (ros::Time().now() - corner_time).toSec();
	if (time_diff < time_lim){
		is_corner_beginning = false;
		ROS_WARN("time_diff(%lf) < %d", time_diff, time_lim);
#if DEBUG_ENABLE
//		beeper.beepForCommand(INVALID);
#endif
	} else {
		is_corner_beginning = true;
		ROS_WARN("time_diff(%lf) > %d", time_diff, time_lim);
#if DEBUG_ENABLE
//		beeper.beepForCommand(VALID);
#endif
	}
	corner_time = ros::Time::now();
	auto offset_x = is_corner_beginning ? CELL_SIZE * 0.7 : 0;
	auto d_points = circle.getPoints(10,is_corner_beginning);
	for(auto& point:d_points)
	{
		if(!is_left_)
			point.y = -point.y;
		tmp_targets.push_back(getPosition().getRelative(point.x + offset_x, point.y));
	}
	return tmp_targets;
}

Point_t MovementFollowWallLidar::calcTmpTarget() {

//	ROS_WARN("curr_point(%d,%d)", getPosition().x, getPosition().y);
	auto path_head = dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_)->getTempTarget();

	if (path_head.seq != seq_) {
		seq_ = path_head.seq;
		lidar_targets_ = path_head.tmp_plan_path_;
		if(!lidar_targets_.empty()) {
//			INFO_PURPLE("p_tmp_targets_ = &lidar_targets_");
			virtual_targets_.clear();
			p_tmp_targets_ = &lidar_targets_;
		} else {
//			INFO_PURPLE("p_tmp_targets_ = &virtual_targets_");
			p_tmp_targets_ = &virtual_targets_;
		}
//		ROS_WARN("lidar_targets_.size() = %d", lidar_targets_.size());
	}
//	ROS_WARN_COND(DEBUG_ENABLE, "p_tmp_targets_.size() = %d, first target:(%f, %f), current(%f, %f).",
//				  p_tmp_targets_->size(), p_tmp_targets_->front().x, p_tmp_targets_->front().y,
//				  getPosition().x, getPosition().y);

	if(!p_tmp_targets_->empty()) {
		if (p_tmp_targets_->front().isNearTo(getPosition(), CELL_SIZE * 0.75)) {
			p_tmp_targets_->pop_front();
//			ROS_WARN("near pop target(%d)", p_tmp_targets_->size());
			if (p_tmp_targets_->empty()){
				virtual_targets_ = _calcTmpTarget();
				p_tmp_targets_ = &virtual_targets_;
//				INFO_PURPLE("p_tmp_targets_->empty(), use virtual target");
			}
		}
	} else {
		virtual_targets_ = _calcTmpTarget();
		p_tmp_targets_ = &virtual_targets_;
//		INFO_PURPLE("p_tmp_targets_->empty(), use virtual target");
	}
//	ROS_WARN("is_virtual_target(%d,%d)", lidar_targets_.empty(),lidar_targets_.size());
	dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_)->pubTmpTarget(p_tmp_targets_->front(), p_tmp_targets_ == &virtual_targets_ );
//	ROS_ERROR_COND(DEBUG_ENABLE,"p_tmp_targets_(%f,%f,%d)", p_tmp_targets_->front().x, p_tmp_targets_->front().y, p_tmp_targets_->front().th);
	return p_tmp_targets_->front();
}

bool MovementFollowWallLidar::isFinish() {
	radian_diff = getPosition().courseToDest(calcTmpTarget());
//	Point_t odom_p = {odom.getOriginX(),odom.getOriginY(),odom.getRadian()};
//	radian_diff = odom_p.courseToDest(calcTmpTarget());
	if(AMovementFollowPoint::isFinish())
		return true;
	return sp_mt_->shouldMoveBack() || sp_mt_->shouldTurn() || sp_mt_->RconTrigger();
}

uint8_t MovementFollowWallLidar::isNear() {
//	if(tmp_targets.empty())
//	auto obs_dis_front = lidar.getObstacleDistance(0,ROBOT_RADIUS);
//	ROS_ERROR("obs_dis_front(%d)",obs_dis_front);
//	return obs_dis_front < 0.25;
	return 0;
}

