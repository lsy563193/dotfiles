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
//	kp_ = 4;
	tmp_pos = getPosition();
	angle_forward_to_turn_ = degree_to_radian(150);
	min_speed_ = LINEAR_MIN_SPEED;
	max_speed_ = LINEAR_MAX_SPEED;
	base_speed_ = LINEAR_MIN_SPEED;
	tick_limit_ = 1;

//	ROS_INFO_FL();
//	tick_limit_ = 1;
//	auto p_clean_mode = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
//	sp_mt_->target_point_ = p_clean_mode->remain_path_.front();
}
void MovementFollowPointLinear::scaleCorrectionPos() {
//	CELL_SIZE
	auto cal_scale = [](double val) {
//			double scale = fabs(val) > 0.05 ? 0.5 * fabs(val) : 0.03;
		double scale = fabs(val) > 0.05 ? 0.5 * fabs(val) : 0.005;
		scale = std::min(1.0, scale);
//		printf("scale{%f} ", scale);
		return scale;
	};

	auto diff = getPosition() - tmp_pos;
//	if(std::abs(diff.y) >= CELL_SIZE/3)
//	{
	diff.y = cal_scale(diff.y) * diff.y;
	diff.x = cal_scale(diff.x) * diff.x;
//		ROS_INFO("y(%f,%f)",ty, diff.y);
//	}
//	else
//	{
//		diff.y = 0;
//		ROS_INFO("y(%f,%f)",ty, diff.y);
//	}
//	diff.y = 0;
//	ROS_INFO("y_pos(%f,%f)",getPosition().y, tmp_pos.y);
	tmp_pos += diff;
//	ROS_INFO("y_pos(%f,%f)",getPosition().y, tmp_pos.y);
//	printf("\n");
//	ROS_INFO("diff_y(%f)",diff.y);
}

Point_t MovementFollowPointLinear::_calcTmpTarget()
{
	auto p_mode = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	auto tmp_target_ = sp_mt_->remain_path_.front();

	if(isAny(p_mode->iterate_point_.dir))
		return tmp_target_;
	scaleCorrectionPos();
	auto &tmp_target_xy = (isXAxis(p_mode->iterate_point_.dir)) ? tmp_target_.x : tmp_target_.y;
	auto curr_xy = (isXAxis(p_mode->iterate_point_.dir)) ? tmp_pos.x : tmp_pos.y;
	auto &other_tmp_target_xy = (isXAxis(p_mode->iterate_point_.dir)) ? tmp_target_.y : tmp_target_.x ;
	auto &other_curr_xy = (isXAxis(p_mode->iterate_point_.dir)) ? tmp_pos.y :tmp_pos.x ;
//	ROS_INFO("curr_xy(%f), target_xy(%f)", curr_xy, tmp_target_xy);
	auto dis = std::min(std::abs(curr_xy - tmp_target_xy),  (CELL_SIZE * 1.5f /*+ CELL_COUNT_MUL*/));
	if (!isPos(p_mode->iterate_point_.dir))
		dis *= -1;
	tmp_target_xy = curr_xy + dis;
	other_tmp_target_xy = other_curr_xy;
//	ROS_INFO("dis(%d)",dis);
//	ROS_WARN("curr(%f,%d, target(%f,%f), dir(%f) ", getPosition().x,getPosition().y, tmp_target_.x,tmp_target_.y,p_mode->start_point_.dir);
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

	if(std::abs(radian_diff) > degree_to_radian(50))
		kp_ = 2;
	else if(std::abs(radian_diff) > degree_to_radian(40))
		kp_ = 3;
	else if(std::abs(radian_diff) > degree_to_radian(30))
		kp_ = 4;
	else if(std::abs(radian_diff) > degree_to_radian(10))
		kp_ = 5;
	else
		kp_ = 5;
	return tmp_target_;
}

bool MovementFollowPointLinear::isFinish()
{
	return AMovementFollowPoint::isFinish() || sp_mt_->shouldMoveBack() || sp_mt_->isLidarStop();
}

uint8_t MovementFollowPointLinear::isNear()
{
	bool near_blocked_in_slam_map = slam_grid_map.isFrontSlamBlocked();
	auto obstacle_distance_front = lidar.getObstacleDistance(0,ROBOT_RADIUS);
	auto b_obs = obs.getStatus() > 0;
	auto b_lidar = (obstacle_distance_front < 0.25);
	auto b_map = near_blocked_in_slam_map;
	if (b_obs || b_lidar) {
//		ROS_WARN("slowdown: obs(%d), lidar(%d), map(%d)", b_obs, b_lidar, b_map);
		return 1;
	} else if (b_map){
		return 2;
	} else {
		return 0;
	}
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
