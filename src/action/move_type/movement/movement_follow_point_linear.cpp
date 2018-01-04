// // Created by lsy563193 on 12/5/17.  //

#include <event_manager.h>
#include "pp.h"
#include "arch.hpp"

MovementFollowPointLinear::MovementFollowPointLinear()
{
	min_speed_ = LINEAR_MIN_SPEED;
	max_speed_ = LINEAR_MAX_SPEED;
//	sp_mt_->sp_cm_->tmp_plan_path_ = path;
//	s_target_p = GridMap::cellToPoint(sp_mt_->sp_cm_->tmp_plan_path_.back());
	base_speed_ = LINEAR_MIN_SPEED;
	tick_limit_ = 1;
	auto p_clean_mode = (ACleanMode*)(sp_mt_->sp_mode_);
	sp_mt_->target_point_ = p_clean_mode->plan_path_.front();
//	sp_mt_->sp_cm_->plan_path_display_sp_mt_->sp_cm_->plan_path_points();
//	g_is_should_follow_wall = false;
//	s_target = target;
//	sp_mt_->sp_cm_->tmp_plan_path_ = path;
}

bool MovementFollowPointLinear::_checkIsNear(const Point32_t& tmp_target, const Point32_t& target,MapDirection new_dir) {
	auto &target_xy = (isXAxis(new_dir)) ? target.x : target.y;
	auto &tmp_xy = (isXAxis(new_dir)) ? tmp_target.x : tmp_target.y;

	return (isPos(new_dir)) ? target_xy <= tmp_xy : target_xy >= tmp_xy;
}

Point32_t MovementFollowPointLinear::_calcTmpTarget(const Point32_t& curr, const Point32_t& target,MapDirection new_dir) {
	auto curr_xy = (isXAxis(new_dir)) ? curr.x : curr.y;
	auto &target_xy = (isXAxis(new_dir)) ? target.x : target.y;
	Point32_t tmp_target;
	tmp_target.x = isXAxis(new_dir) ? curr_xy : target.x;
	tmp_target.y = isXAxis(new_dir) ? target.y : curr_xy;
	auto &tmp_xy = (isXAxis(new_dir)) ? tmp_target.x : tmp_target.y;
//	ROS_WARN("curr_xy(%d), target_xy(%d)", curr_xy, target_xy);
	auto dis = std::min(std::abs(curr_xy - target_xy), (int32_t) (CELL_COUNT_MUL*0.75));
//	ROS_INFO("dis(%d)",dis);
	if (!isPos(new_dir))
		dis *= -1;
	tmp_xy = curr_xy + dis;
//	ROS_WARN("tmp(%d,%d)",tmp_target.x, tmp_target.y);
//	ROS_WARN("dis(%d),dir(%d), curr(%d, %d), tmp_target(%d, %d)", dis, new_dir, curr.x, curr.y, tmp_target.x, tmp_target.y);
	return tmp_target;
}

bool MovementFollowPointLinear::calcTmpTarget(Point32_t& tmp_target) {
	auto p_cm = (ACleanMode*)(sp_mt_->sp_mode_);
	auto curr = getPosition();

	tmp_target = _calcTmpTarget(curr, sp_mt_->target_point_,p_cm->new_dir_);

	auto is_near = _checkIsNear(tmp_target, sp_mt_->target_point_, p_cm->new_dir_);

	if (is_near && p_cm->plan_path_.size() > 1) {
		p_cm->old_dir_ = p_cm->new_dir_;
		p_cm->new_dir_ = (MapDirection) p_cm->plan_path_.front().th;
		p_cm->plan_path_.pop_front();
		sp_mt_->target_point_ = p_cm->plan_path_.front();
//		ROS_INFO("%s,%d,is_near(%d),dir(%d),target(%d,%d),tmp(%d,%d)", __FUNCTION__, __LINE__, is_near, p_cm->new_dir_, sp_mt_->target_point_.x, sp_mt_->target_point_.y, tmp_target.x, tmp_target.y);
		tmp_target = _calcTmpTarget(curr, sp_mt_->target_point_,p_cm->new_dir_);

		ROS_INFO("%s,%d,dir(%d),target(%d,%d),tmp(%d,%d)", __FUNCTION__, __LINE__, p_cm->new_dir_, sp_mt_->target_point_.x, sp_mt_->target_point_.y, tmp_target.x, tmp_target.y);
		ROS_INFO("%s,%d,next target_point(%d,%d)",__FUNCTION__,__LINE__,sp_mt_->target_point_.toCell().x,sp_mt_->target_point_.toCell().y);
	}

	return true;
}

bool MovementFollowPointLinear::isFinish()
{
	return sp_mt_->shouldMoveBack();
}

bool MovementFollowPointLinear::is_near()
{
	auto curr_p = getPosition();
	bool is_decrease_blocked = decrease_map.isFrontBlocked();
//	auto distance = two_points_distance(curr_p.x, curr_p.y, s_target_p.x, s_target_p.y);
	auto obstacle_distance_front = lidar.getObstacleDistance(0,ROBOT_RADIUS);
	return obs.getStatus() > 0 || /*(distance < SLOW_DOWN_DISTANCE) ||*/  (obstacle_distance_front < 0.25) || is_decrease_blocked;
}
