// // Created by lsy563193 on 12/5/17.  //

#include <movement.hpp>
#include <move_type.hpp>
#include <state.hpp>
#include <mode.hpp>
#include <event_manager.h>
#include <robot.hpp>
#include "dev.h"
//CELL_COUNT_MUL*1.5
MovementFollowPointLinear::MovementFollowPointLinear()
{
	angle_forward_to_turn_ = degree_to_radian(150);
	min_speed_ = LINEAR_MIN_SPEED;
	max_speed_ = LINEAR_MAX_SPEED;
	base_speed_ = LINEAR_MIN_SPEED;
	tick_limit_ = 1;

//	ROS_INFO_FL();
//	tick_limit_ = 1;
//	auto p_clean_mode = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
//	sp_mt_->target_point_ = p_clean_mode->plan_path_.front();
}

Point_t MovementFollowPointLinear::_calcTmpTarget()
{
	auto p_mode = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	auto tmp_target_ = p_mode->plan_path_.front();
	auto &tmp_target_xy = (isXAxis(p_mode->new_dir_)) ? tmp_target_.x : tmp_target_.y;
	auto curr_xy = (isXAxis(p_mode->new_dir_)) ? getPosition().x : getPosition().y;
//	ROS_INFO("curr_xy(%f), target_xy(%f)", curr_xy, tmp_target_xy);
	auto dis = std::min(std::abs(curr_xy - tmp_target_xy),  (CELL_SIZE * 1.5f /*+ CELL_COUNT_MUL*/));
	if (!isPos(p_mode->new_dir_))
		dis *= -1;
	tmp_target_xy = curr_xy + dis;
//	ROS_INFO("dis(%d)",dis);
//	ROS_WARN("curr(%f,%d, target(%f,%f), dir(%f) ", getPosition().x,getPosition().y, tmp_target_.x,tmp_target_.y,p_mode->new_dir_);
//	ROS_WARN("tmp(%f,%f)",tmp_target_.x, tmp_target_.y);
	return tmp_target_;
}

Point_t MovementFollowPointLinear::calcTmpTarget()
{
//	auto curr_xy = (isXAxis(tmp_target_.th)) ? getPosition().x : getPosition().y;
//	auto tmp_target_xy = (isXAxis(tmp_target_.th)) ? tmp_target_.x :tmp_target_.y;
//	if(std::abs(curr_xy - tmp_target_xy) > LINEAR_NEAR_DISTANCE) {
//		return false;
//	}
	auto tmp_target_ = _calcTmpTarget();

	dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_)->pubTmpTarget(tmp_target_);
	return tmp_target_;
}

bool MovementFollowPointLinear::isFinish()
{
	return AMovementFollowPoint::isFinish() || sp_mt_->shouldMoveBack() || sp_mt_->isLidarStop();
}

bool MovementFollowPointLinear::isNear()
{
	bool near_blocked_in_slam_map = slam_grid_map.isFrontBlocked();
//	if (near_blocked_in_slam_map)
//		ROS_ERROR("%s %d: Near blocks in slam_map, slow down.", __FUNCTION__, __LINE__);
//	auto curr_p = getPosition();
//	auto distance = two_points_distance(curr_p.x, curr_p.y, s_target_p.x, s_target_p.y);
	auto obstacle_distance_front = lidar.getObstacleDistance(0,ROBOT_RADIUS);
//	ROS_INFO("dis(%lf)", obstacle_distance_front);
	return obs.getStatus() > 0 || /*(distance < SLOW_DOWN_DISTANCE) ||*/  (obstacle_distance_front < 0.25) || near_blocked_in_slam_map;
}

//Point_t MovementFollowPointLinear::_calcTmpTargetRealTime()
//{
//	auto tmp_target_ = sp_mt_->target_point_;
//	auto target_xy = (isXAxis(tmp_target_.th)) ? sp_mt_->target_point_.x : sp_mt_->target_point_.y;
//	auto curr_xy = (isXAxis(tmp_target_.th)) ? getPosition().x : getPosition().y;
//	auto &tmp_xy = (isXAxis(tmp_target_.th)) ? tmp_target_.x : tmp_target_.y;
////	ROS_WARN("curr_xy(%d), target_xy(%d)", curr_xy, target_xy);
//	auto dis = std::min(std::abs(curr_xy - target_xy), (int32_t) (LINEAR_NEAR_DISTANCE /*+ CELL_COUNT_MUL*/));
////	ROS_INFO("dis(%d)",dis);
//	if (!isPos(tmp_target_.th))
//		dis *= -1;
//	tmp_xy = curr_xy + dis;
//	return tmp_target_;
////	ROS_WARN("tmp(%d,%d)",tmp_target_.x, tmp_target_.y);
////	ROS_WARN("dis(%d),dir(%d), curr(%d, %d), tmp_target(%d, %d)", dis, tmp_target_.th, curr.x, curr.y, tmp_target.x, tmp_target.y);
//}
