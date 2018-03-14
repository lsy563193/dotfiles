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
//	tmp_pos = getPosition();
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
//void MovementFollowPointLinear::scaleCorrectionPos() {
////	CELL_SIZE
//	auto cal_scale = [](double val) {
////			double scale = fabs(val) > 0.05 ? 0.5 * fabs(val) : 0.03;
//		double scale = fabs(val) > 0.05 ? 0.5 * fabs(val) : 0.03;
//		scale = std::min(1.0, scale);
////		printf("scale{%f} ", scale);
//		return scale;
//	};
//
//	auto robot_rad = robot::instance()->getWorldPoseRadian();
////	tf::Vector3	robot_pos={robot::instance()->getWorldPoseX(), robot::instance()->getWorldPoseY(), 0.0};
//	Point_t curr = {robot::instance()->getWorldPoseX(), robot::instance()->getWorldPoseY(),robot_rad};
//	auto diff = /*getPosition()*/curr - tmp_pos;
////	ROS_INFO("diff(%f,%f)", diff.x, diff.y);
////	if(!(diff.y > CELL_SIZE/5 && left_speed_ == right_speed_))
//	auto old_y = tmp_pos.y;
//	{
//		diff.y = cal_scale(diff.y) * diff.y;
////		diff.x = cal_scale(diff.x) * diff.x;
//		tmp_pos += diff;
//	}
////	if(!(radian_to_degree(diff.th) > 2 && left_speed_ == right_speed_))
////	{
////		diff.y = cal_scale(diff.y) * diff.y;
////		diff.x = cal_scale(diff.x) * diff.x;
////		tmp_pos += diff;
////	}
////	else{
////		beeper.beepForCommand(VALID);
////		ROS_ERROR("location jump!!! diff_y(%f),speed(%d,%d)",diff.y, left_speed_, right_speed_);
////	}
//
//	tf::Vector3	robot_pos={tmp_pos.x, tmp_pos.y, 0.0};
////	double	robot_rad= tmp_pos.th;
//	ROS_INFO("diff %f",diff.y,);
//	ROS_INFO("curr %f",curr.y);
//	ROS_INFO("tmp_ (%f %f), rad(%d,%d)", tmp_pos.y, old_y,tmp_pos.th, curr.th);
//	robot::instance()->odomPublish(robot_pos, robot_rad);
//}

Point_t MovementFollowPointLinear::_calcTmpTarget()
{
	auto p_mode = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	auto tmp_target_ = sp_mt_->remain_path_.front();

	if(isAny(p_mode->iterate_point_.dir))
		return tmp_target_;
	auto tmp_pos = getPosition();
	auto &tmp_target_xy = (isXAxis(p_mode->iterate_point_.dir)) ? tmp_target_.x : tmp_target_.y;
	auto curr_xy = (isXAxis(p_mode->iterate_point_.dir)) ? tmp_pos.x : tmp_pos.y;
	auto &other_tmp_target_xy = (isXAxis(p_mode->iterate_point_.dir)) ? tmp_target_.y : tmp_target_.x ;
	auto &other_curr_xy = (isXAxis(p_mode->iterate_point_.dir)) ? tmp_pos.y :tmp_pos.x ;
//	ROS_INFO("curr_xy(%f), target_xy(%f)", curr_xy, tmp_target_xy);
	auto dis = std::min(std::abs(curr_xy - tmp_target_xy),  (CELL_SIZE * 1.5f /*+ CELL_COUNT_MUL*/));
	if (!isPos(p_mode->iterate_point_.dir))
		dis *= -1;
	tmp_target_xy = curr_xy + dis;
//	other_tmp_target_xy = other_curr_xy;
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

		radian_diff = getPosition().courseToDest(calcTmpTarget());
	}
	else
	{
		if(sp_mt_->radian_diff_count == 10) {
			sp_mt_->radian_diff_count++;

			auto offset_adjustment = getPosition().th - odom.getRadian();
			odom.setRadianOffset(odom.getRadianOffset() + offset_adjustment);

			sp_mt_->odom_turn_target_radian_ = ranged_radian(sp_mt_->turn_target_radian_ - offset_adjustment);
			beeper.beepForCommand(VALID);
		}
//		radian_diff = getPosition().courseToDest(calcTmpTarget());
		radian_diff = ranged_radian(sp_mt_->turn_target_radian_ - odom.getRadian());
	}


//		radian_diff = sp_mt_->odom_turn_target_radian_;
//	radian_diff = ranged_radian(sp_mt_->turn_target_radian_ - odom.getRadian());
	radian_diff = getPosition().courseToDest(calcTmpTarget());

//	ROS_ERROR("count(%d), radian_diff(%f, %f,%f),",sp_mt_->radian_diff_count , radian_to_degree(sp_mt_->turn_target_radian_),
//						radian_to_degree(radian_diff),radian_to_degree(sp_mt_->odom_turn_target_radian_));

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
