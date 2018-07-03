// // Created by lsy563193 on 12/5/17.  //

#include <movement.hpp>
#include <move_type.hpp>
#include <mode.hpp>
#include <robot.hpp>
#include <wheel.hpp>
#include <lidar.hpp>
#include <obs.h>

//CELL_COUNT_MUL*1.5
MovementFollowPointLinear::MovementFollowPointLinear()
{
//	kp_ = 4;
//	tmp_pos = getPosition();
	ROS_WARN("%s %d: Enter movement follow point linear.", __FUNCTION__, __LINE__);
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

void MovementFollowPointLinear::scaleCorrectionPos(Point_t &tmp_pos) {
//	auto p_cm = boost::dynamic_pointer_cast<ACleanMode> (p_mode);
	auto p_cm = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	auto dir = p_cm->iterate_point_->dir;
	auto curr = getPosition();
	if(isAny(dir))
		return;

	auto target_xy = (isXAxis(dir)) ? p_cm->iterate_point_->y : p_cm->iterate_point_->x;
//	auto slam_xy = (isXAxis(dir)) ? slam_pos.getOriginY() : slam_pos.getOriginX();
	auto slam_xy = (isXAxis(dir)) ? curr.y : curr.x;
	auto diff_xy = (slam_xy - target_xy)/3;

	if(diff_xy > CELL_SIZE/2)
		diff_xy = CELL_SIZE/2;
	else if(diff_xy < -CELL_SIZE/2)
		diff_xy = -CELL_SIZE/2;
		(isXAxis(dir)) ? (tmp_pos.y = (target_xy + diff_xy)) : (tmp_pos.x = (target_xy + diff_xy));
}

Point_t MovementFollowPointLinear::_calcTmpTarget()
{
	auto p_mode = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	auto tmp_target_ = *std::next(p_mode->iterate_point_);

	if(isAny(p_mode->iterate_point_->dir))
		return tmp_target_;
	auto tmp_pos = getPosition();
	auto &tmp_target_xy = (isXAxis(p_mode->iterate_point_->dir)) ? tmp_target_.x : tmp_target_.y;
	auto curr_xy = (isXAxis(p_mode->iterate_point_->dir)) ? tmp_pos.x : tmp_pos.y;
	auto &other_tmp_target_xy = (isXAxis(p_mode->iterate_point_->dir)) ? tmp_target_.y : tmp_target_.x ;
	auto &other_curr_xy = (isXAxis(p_mode->iterate_point_->dir)) ? tmp_pos.y :tmp_pos.x ;
//	ROS_INFO("curr_xy(%f), target_xy(%f)", curr_xy, tmp_target_xy);
	auto dis = std::min(std::abs(curr_xy - tmp_target_xy),  (CELL_SIZE * 1.5f /*+ CELL_COUNT_MUL*/));
	if (!isPos(p_mode->iterate_point_->dir))
		dis *= -1;
	tmp_target_xy = curr_xy + dis;
//	other_tmp_target_xy = other_curr_xy;
//	ROS_INFO("dis(%d)",dis);
//	ROS_WARN("curr(%f,%d, target(%f,%f), dir(%f) ", getPosition().x,getPosition().y, tmp_target_.x,tmp_target_.y,p_mode->start_points_.dir);
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

//	if(std::abs(radian_diff) > degree_to_radian(50))
//		kp_ = 2;
//	else if(std::abs(radian_diff) > degree_to_radian(40))
//		kp_ = 3;
//	else if(std::abs(radian_diff) > degree_to_radian(30))
//		kp_ = 4;
//	else if(std::abs(radian_diff) > degree_to_radian(10))
//		kp_ = 5;
//	else
//		kp_ = 2;
//	ROS_ERROR("tmp_target_(%lf , %lf)", tmp_target_.x, tmp_target_.y);
	return tmp_target_;
}

bool MovementFollowPointLinear::isFinish() {
	if(sp_mt_->radian_diff_count <10)
	{
//		radian_diff = getPosition().courseToDest(calcTmpTarget());
//		sp_mt_->odom_turn_target_radians_.
	if(std::abs(radian_to_degree(radian_diff)) < 0.5)
		sp_mt_->radian_diff_count++;
	else
		sp_mt_->radian_diff_count =0;
	}
	else
	{
		if(sp_mt_->radian_diff_count == 10) {
			sp_mt_->radian_diff_count++;
			auto offset_rad = ranged_radian(getPosition().th - odom.getRadian());
/*			if(std::abs(offset_rad) > degree_to_radian(4))
			{
				odom.setRadianOffset(getPosition().th - odom.getOriginRadian());
				ROS_INFO("offset_rad");
			}*/
		}
		radian_diff = ranged_radian(sp_mt_->turn_target_radian_ - odom.getRadian());
	}

	auto tmp_pos = getPosition();
	scaleCorrectionPos(tmp_pos);
	radian_diff = tmp_pos.courseToDest(calcTmpTarget());
//	ROS_WARN("radian_diff(%lf)", radian_diff);
//	ROS_WARN("curr(%lf, %lf)", getPosition().x, getPosition().y);
	auto is_lidar_stop = sp_mt_->isLidarStop();
//	ROS_ERROR("is_lidar_stop(%d)", is_lidar_stop);
	auto ret = AMovementFollowPoint::isFinish() || sp_mt_->isFinishForward() || is_lidar_stop;
	if (ret) {
		wheel.stop();
		dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_)->continue_to_isolate_ = false;
		dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_)->restart_wf_ = true;
		ROS_ERROR("%s %d: set continue_to_isolate_ false, set restart_wf_ true.", __FUNCTION__, __LINE__);
	}
	return ret;
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
