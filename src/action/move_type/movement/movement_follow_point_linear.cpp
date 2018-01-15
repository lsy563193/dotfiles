// // Created by lsy563193 on 12/5/17.  //

#include <event_manager.h>
#include "pp.h"
#include "arch.hpp"

MovementFollowPointLinear::MovementFollowPointLinear()
{
	PP_INFO();
	min_speed_ = LINEAR_MIN_SPEED;
	max_speed_ = LINEAR_MAX_SPEED;
//	sp_mt_->sp_cm_->tmp_plan_path_ = path;
//	s_target_p = GridMap::cellToPoint(sp_mt_->sp_cm_->tmp_plan_path_.back());
	base_speed_ = LINEAR_MIN_SPEED;
	tick_limit_ = 1;
	auto p_clean_mode = (ACleanMode*)(sp_mt_->sp_mode_);
	sp_mt_->target_point_ = p_clean_mode->plan_path_.front();
//	tmp_target_.th = sp_mt_->target_point_.th;
//	tmp_target_ = _calcTmpTarget();
//	sp_mt_->sp_cm_->plan_path_display_sp_mt_->sp_cm_->plan_path_points();
//	g_is_should_follow_wall = false;
//	s_target = target;
//	sp_mt_->sp_cm_->tmp_plan_path_ = path;
}

Point32_t MovementFollowPointLinear::_calcTmpTargetRealTime()
{
	auto tmp_target_ = sp_mt_->target_point_;
	auto target_xy = (isXAxis(tmp_target_.th)) ? sp_mt_->target_point_.x : sp_mt_->target_point_.y;
	auto curr_xy = (isXAxis(tmp_target_.th)) ? getPosition().x : getPosition().y;
	auto &tmp_xy = (isXAxis(tmp_target_.th)) ? tmp_target_.x : tmp_target_.y;
//	ROS_WARN("curr_xy(%d), target_xy(%d)", curr_xy, target_xy);
	auto dis = std::min(std::abs(curr_xy - target_xy), (int32_t) (LINEAR_NEAR_DISTANCE /*+ CELL_COUNT_MUL*/));
//	ROS_INFO("dis(%d)",dis);
	if (!isPos(tmp_target_.th))
		dis *= -1;
	tmp_xy = curr_xy + dis;
	return tmp_target_;
//	ROS_WARN("tmp(%d,%d)",tmp_target_.x, tmp_target_.y);
//	ROS_WARN("dis(%d),dir(%d), curr(%d, %d), tmp_target(%d, %d)", dis, tmp_target_.th, curr.x, curr.y, tmp_target.x, tmp_target.y);
}

bool MovementFollowPointLinear::calcTmpTarget()
{
	tmp_target_ = _calcTmpTargetRealTime();
	robot::instance()->pubTmpTarget(tmp_target_);
	return true;
}

bool MovementFollowPointLinear::isFinish()
{
	return sp_mt_->shouldMoveBack();
}

bool MovementFollowPointLinear::is_near()
{
	bool is_decrease_blocked = decrease_map.isFrontBlocked();
//	auto curr_p = getPosition();
//	auto distance = two_points_distance(curr_p.x, curr_p.y, s_target_p.x, s_target_p.y);
	auto obstacle_distance_front = lidar.getObstacleDistance(0,ROBOT_RADIUS);
	return obs.getStatus() > 0 || /*(distance < SLOW_DOWN_DISTANCE) ||*/  (obstacle_distance_front < 0.25) || is_decrease_blocked;
}

Point32_t MovementFollowPointLinear::_calcTmpTargetNoneRealTime() {
	auto curr = getPosition();
	ACleanMode* p_mode = dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_);
	auto curr_xy = (isXAxis(p_mode->new_dir_)) ? curr.x : curr.y;
	auto tmp_target_xy = (isXAxis(p_mode->new_dir_)) ? tmp_target_.x :tmp_target_.y;
	if(abs(curr_xy - tmp_target_xy) > LINEAR_NEAR_DISTANCE &&
		 abs(curr_xy - tmp_target_xy) < (LINEAR_NEAR_DISTANCE + CELL_COUNT_MUL) &&
		 tmp_target_xy != 0) {
		return tmp_target_;
	}
	if(isXAxis(p_mode->new_dir_)){
		tmp_target_.x = sp_mt_->target_point_.x;
		tmp_target_.y = getPosition().y;
	}else{
		tmp_target_.x = getPosition().x;
		tmp_target_.y = sp_mt_->target_point_.y;
	}
	auto &tmp_xy = (isXAxis(p_mode->new_dir_)) ? tmp_target_.x : tmp_target_.y;
	auto &target_xy = (isXAxis(p_mode->new_dir_)) ? sp_mt_->target_point_.x : sp_mt_->target_point_.y;
	auto dis = std::min(std::abs(curr_xy - target_xy), (int32_t) (LINEAR_NEAR_DISTANCE + CELL_COUNT_MUL));
	if (!isPos(p_mode->new_dir_))
		dis *= -1;
	tmp_xy = curr_xy + dis;
	return tmp_target_;
}
