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
	ROS_WARN("%s %d: Enter.", __FUNCTION__, __LINE__);
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

MovementFollowPointLinear::~MovementFollowPointLinear()
{
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

void MovementFollowPointLinear::scaleCorrectionPos(Point_t &tmp_pos) {
//	auto p_cm = boost::dynamic_pointer_cast<ACleanMode> (p_mode);
	auto p_cm = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	auto curr = getPosition();
	//todo isAny
//	if(p_cm->isAny(dir))
//		return;

	ROS_INFO("~~~~~~~~~~~~tmp_pos(%f,%f,%f)",tmp_pos.x, tmp_pos.y, tmp_pos.th);
	Point_t proj_p = curr.project(*p_cm->iterate_point_,*(p_cm->iterate_point_+1));
	ROS_INFO("~~~~~~~~~~~~proj_p(%f,%f,%f)",proj_p.x, proj_p.y, proj_p.th);
	Point_t corr_p{(curr - proj_p) / 3, tmp_pos.th};
	ROS_INFO("~~~~~~~~~~~~corr_p(%f,%f,%f)",corr_p.x, corr_p.y, corr_p.th);
	auto dis = curr.Distance(proj_p);
	if(dis > CELL_SIZE/2)
	{
		corr_p = {proj_p + (curr - proj_p) * (CELL_SIZE/2/dis), tmp_pos.th};
		ROS_ERROR("~~~~~~~~~~~~corr_p(%f,%f,%f)",corr_p.x, corr_p.y, corr_p.th);
	}

	ROS_WARN("2~~~~~~~~~~~~corr_p(%f,%f,%f)",corr_p.x, corr_p.y, corr_p.th);
	tmp_pos = {proj_p + corr_p, tmp_pos.th};
	ROS_WARN("2~~~~~~~~~~~~tmp_pos(%f,%f,%f)",tmp_pos.x, tmp_pos.y, tmp_pos.th);
}

Point_t MovementFollowPointLinear::_calcTmpTarget()
{
	auto p_cm = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	auto tmp_target_ = *(p_cm->iterate_point_+1);

	auto curr = getPosition();

	Point_t proj_p = curr.project(*p_cm->iterate_point_,*(p_cm->iterate_point_+1));

	auto dis = std::min(proj_p.Distance(tmp_target_),  (CELL_SIZE * 1.5f /*+ CELL_COUNT_MUL*/));

	tmp_target_ = {proj_p + (tmp_target_-proj_p) * (dis/tmp_target_.Distance(proj_p)), p_cm->iterate_point_->th};

	return tmp_target_;
}

Point_t MovementFollowPointLinear::calcTmpTarget()
{
	auto tmp_target_ = _calcTmpTarget();

	dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_)->pubTmpTarget(tmp_target_);

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
		auto p_clean_mode = dynamic_cast<ACleanMode*>(sp_mt_->sp_mode_);
		wheel.stop();
		if (p_clean_mode->should_handle_isolate_) {
			p_clean_mode->continue_to_isolate_ = false;
			p_clean_mode->restart_wf_ = true;
			ROS_ERROR("%s %d: set continue_to_isolate_ false, set restart_wf_ true.", __FUNCTION__, __LINE__);
		}
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
