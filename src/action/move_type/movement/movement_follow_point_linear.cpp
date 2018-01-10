// // Created by lsy563193 on 12/5/17.  //

#include <event_manager.h>
#include "pp.h"
#include "arch.hpp"

MovementFollowPointLinear::MovementFollowPointLinear()
{
	min_speed_ = LINEAR_MIN_SPEED;
	max_speed_ = LINEAR_MAX_SPEED;
	base_speed_ = LINEAR_MIN_SPEED;
	tick_limit_ = 1;
}

Point32_t MovementFollowPointLinear::_calcTmpTarget()
{
	auto p_mode = ((ACleanMode*)(sp_mt_->sp_mode_));
	auto curr = getPosition();
	auto tmp_target_ = p_mode->plan_path_.front();
	auto &tmp_target_xy = (isXAxis(p_mode->new_dir_)) ? tmp_target_.x : tmp_target_.y;
	auto curr_xy = (isXAxis(p_mode->new_dir_)) ? getPosition().x : getPosition().y;
//	ROS_INFO("curr_xy(%d), target_xy(%d)", curr_xy, tmp_target_xy);
	auto dis = std::min(std::abs(curr_xy - tmp_target_xy), (int32_t) (LINEAR_NEAR_DISTANCE /*+ CELL_COUNT_MUL*/));
	if (!isPos(p_mode->new_dir_))
		dis *= -1;
	tmp_target_xy = curr_xy + dis;
//	ROS_INFO("dis(%d)",dis);
//	ROS_WARN("curr(%d,%d), target(%d,%d), dir(%d) ", curr.x,curr.y, tmp_target_.x,tmp_target_.y,p_mode->new_dir_);
//	ROS_WARN("tmp(%d,%d)",tmp_target_.x, tmp_target_.y);
	return tmp_target_;
//	ROS_WARN("dis(%d),dir(%d), curr(%d, %d), tmp_target(%d, %d)", dis, tmp_target_.th, curr.x, curr.y, tmp_target.x, tmp_target.y);
}

bool MovementFollowPointLinear::calcTmpTarget()
{
//	auto curr_xy = (isXAxis(tmp_target_.th)) ? getPosition().x : getPosition().y;
//	auto tmp_target_xy = (isXAxis(tmp_target_.th)) ? tmp_target_.x :tmp_target_.y;
//	if(std::abs(curr_xy - tmp_target_xy) > LINEAR_NEAR_DISTANCE) {
//		return false;
//	}
	tmp_target_ = _calcTmpTarget();

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
