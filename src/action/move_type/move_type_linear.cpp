//
// Created by lsy563193 on 12/4/17.
//
#include <event_manager.h>
#include "dev.h"
#include "robot.hpp"

#include <move_type.hpp>
#include <state.hpp>
#include <mode.hpp>

MoveTypeLinear::MoveTypeLinear(Points remain_path){
	resetTriggeredValue();
	remain_path.pop_front();
	remain_path_ = remain_path;

	auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
	auto target_point_ = remain_path_.front();
	turn_target_radian_ = p_mode->iterate_point_.th;

	ROS_INFO("%s,%d: Enter move type linear, angle(%f,%f, %f),  target(%f, %f).",
			 __FUNCTION__, __LINE__, getPosition().th, radian_to_degree(target_point_.th), radian_to_degree(turn_target_radian_), target_point_.x, target_point_.y);

	movement_i_ = p_mode->isGyroDynamic() ? mm_dynamic : mm_turn;
	if(movement_i_ == mm_dynamic)
		sp_movement_.reset(new MovementGyroDynamic());
	else
		sp_movement_.reset(new MovementTurn(turn_target_radian_, ROTATE_TOP_SPEED));

	IMovement::sp_mt_ = this;
}

MoveTypeLinear::~MoveTypeLinear()
{
	if(sp_mode_ != nullptr){
		auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
		p_mode->saveBlocks();
		p_mode->mapMark();
		memset(IMoveType::rcon_cnt,0,sizeof(int8_t)*6);
	}
	ROS_INFO("%s %d: Exit move type linear.", __FUNCTION__, __LINE__);
}

bool MoveTypeLinear::isFinish()
{
	if (IMoveType::isFinish() && isNotHandleEvent())
	{
		ROS_INFO("%s %d: Move type aborted.", __FUNCTION__, __LINE__);
		return true;
	}

	auto p_clean_mode = dynamic_cast<ACleanMode*> (sp_mode_);

	if (isLinearForward())
		switchLinearTarget(p_clean_mode);

	if (sp_movement_->isFinish()) {
		if(movement_i_ == mm_dynamic){
			movement_i_ = mm_turn;
			sp_movement_.reset(new MovementTurn(turn_target_radian_, ROTATE_TOP_SPEED));
//			odom_turn_target_radians_ =  turn_target_radian_  - getPosition().th + odom.getRadian();
		}
		else if(movement_i_ == mm_turn)
		{
			if (!handleMoveBackEvent(p_clean_mode)) {
				movement_i_ = mm_forward;
				resetTriggeredValue();
				sp_movement_.reset(new MovementFollowPointLinear());
				ROS_WARN("turn_target_radian_", radian_to_degree(turn_target_radian_));
//				odom_turn_target_radians_ =  ranged_radian(turn_target_radian_  - getPosition().th + odom.getRadian());
//				odom_turn_target_radians_.push_back(odom.getRadian());
				radian_diff_count = 0;

			}
		}
		else if(movement_i_ == mm_stay)
		{
			beeper.beepForCommand(VALID);
			ROS_ERROR("ev.cliff_triggered(%d)!!!", cliff.getStatus());
			if(cliff.getStatus())
				return true;
			movement_i_ = mm_forward;
			sp_movement_.reset(new MovementFollowPointLinear());
			return false;
		}
		else if (movement_i_ == mm_forward)
		{
			if(!handleMoveBackEventLinear(p_clean_mode)){
				if(ev.cliff_triggered)
				{
					beeper.beepForCommand(VALID);
					ROS_ERROR("ev.cliff_triggered(%d)!!!", ev.cliff_triggered);
					movement_i_ = mm_stay;
					sp_movement_.reset(new MovementStay(0.2));
				}
				if(!ev.tilt_triggered)
					p_clean_mode->should_follow_wall = true;
//				ROS_WARN("111should_follow_wall(%d)!!!", p_clean_mode->should_follow_wall);
				ROS_INFO_FL();
				return true;
			}else {
//				ROS_WARN("111should_follow_wall(%d,%d)!!!", p_clean_mode->should_follow_wall, ev.tilt_triggered);
				if (ev.bumper_triggered || ev.lidar_triggered || ev.rcon_status /*|| ev.obs_triggered*/) {
					p_clean_mode->should_follow_wall = true;
				}
//				ROS_WARN("222should_follow_wall(%d)!!!", p_clean_mode->should_follow_wall);
				ROS_INFO_FL();
				return false;
			}
		}
		else //if (movement_i_ == mm_back)
			return true;
	}
	return false;
}

bool MoveTypeLinear::isCellReach()
{
	// Checking if robot has reached target cell.
	auto s_curr_p = getPosition();
	auto p_clean_mode = dynamic_cast<ACleanMode*> (sp_mode_);
	auto target_point_ = remain_path_.front();
	if (std::abs(s_curr_p.x - target_point_.x) < CELL_SIZE/2 &&
		std::abs(s_curr_p.y - target_point_.y) < CELL_SIZE/2)
	{
//		ROS_INFO("%s, %d: MoveTypeLinear,current cell = (%d,%d) reach the target cell (%d,%d), current angle(%lf), target angle(%lf).", __FUNCTION__, __LINE__,
//						 s_curr_p.toCell().x,s_curr_p.toCell().y,target_point_.toCell().x, target_point_.toCell().y, radian_to_degree(s_curr_p.th), radian_to_degree(target_point_.th));
//		g_turn_angle = ranged_radian(new_dir - robot::instance()->getWorldPoseRadian());
		return true;
	}

	return false;
}

bool MoveTypeLinear::isPoseReach()
{
	// Checking if robot has reached target cell and target angle.
//	PP_INFO();
	auto target_point_ = remain_path_.front();
	if (isCellReach() ) {
		if (std::abs(getPosition().isRadianNear(target_point_))) {
			ROS_INFO("\033[1m""%s, %d: MoveTypeLinear, reach the target cell and pose(%d,%d,%f,%d)""\033[0m", __FUNCTION__,
							 __LINE__,
							 target_point_.toCell().x, target_point_.toCell().y, target_point_.th,target_point_.dir);
			return true;
		}
	}
	return false;
}

bool MoveTypeLinear::isPassTargetStop(Dir_t &dir)
{
//	PP_INFO();
	// Checking if robot has reached target cell.
	if(isAny(dir))
		return false;

	auto s_curr_p = getPosition();
	auto curr = (isXAxis(dir)) ? s_curr_p.x : s_curr_p.y;
	auto target_point_ = remain_path_.front();
	auto target = (isXAxis(dir)) ? target_point_.x : target_point_.y;
	if ((isPos(dir) && (curr > target + CELL_SIZE / 4)) ||
		(!isPos(dir) && (curr < target - CELL_SIZE / 4)))
	{
		ROS_WARN("%s, %d: MoveTypeLinear, pass target: dir(\033[32m%d\033[0m),is_x_axis(\033[32m%d\033[0m),is_pos(\033[32m%d\033[0m),curr(\033[32m%d\033[0m),target(\033[32m%d\033[0m)",
				 __FUNCTION__, __LINE__, dir, isXAxis(dir), isPos(dir), curr, target);
		ROS_INFO("%s,%s,%d,\033[32m curr_cell(%d,%d),target_cell(%d,%d)\033[0m",__FILE__,__FUNCTION__,__LINE__,s_curr_p.toCell().x,s_curr_p.toCell().y,target_point_.toCell().x,target_point_.toCell().y);
		return true;
	}
	return false;
}

bool MoveTypeLinear::isLinearForward()
{
	return movement_i_ == mm_forward;
}

void MoveTypeLinear::switchLinearTarget(ACleanMode * p_clean_mode)
{
	if (remain_path_.size() > 1)
	{
		auto target_point_ = remain_path_.front();
		auto &target_xy = (isXAxis(p_clean_mode->iterate_point_.dir)) ? target_point_.x : target_point_.y;
		auto curr_xy = (isXAxis(p_clean_mode->iterate_point_.dir)) ? getPosition().x : getPosition().y;

		if (std::abs(target_xy - curr_xy) < LINEAR_NEAR_DISTANCE) {
			p_clean_mode->old_dir_ = p_clean_mode->iterate_point_.dir;
			p_clean_mode->iterate_point_ = remain_path_.front();
			remain_path_.pop_front();
//			ROS_("target_xy(%f), curr_xy(%f),dis(%f)",target_xy, curr_xy, LINEAR_NEAR_DISTANCE);
			ROS_ERROR("%s,%d,curr(%d,%d), next target_point(%d,%d,%lf), dir(%d)",
					 __FUNCTION__,__LINE__,getPosition().toCell().x, getPosition().toCell().y, target_point_.toCell().x,target_point_.toCell().y,radian_to_degree(target_point_.th),
								p_clean_mode->iterate_point_.dir);
//			odom_turn_target_radians_ = remain_path_.front().th;
//			odom_turn_target_radians_.push_back(odom.getRadian());
//			odom_turn_target_radian_ = .push_back(odom.getRadian());
			radian_diff_count = 0;
		}
	} else if(remain_path_.size() == 1){
		if(stop_generate_next_target)
			return;

		if(p_clean_mode->isStateFollowWall())
			return;

		auto target_point_ = remain_path_.front();
		auto &target_xy = (isXAxis(p_clean_mode->iterate_point_.dir)) ? target_point_.x : target_point_.y;
		auto curr_xy = (isXAxis(p_clean_mode->iterate_point_.dir)) ? getPosition().x : getPosition().y;
		ROS_ERROR("%f,%f", std::abs(target_xy - curr_xy),LINEAR_NEAR_DISTANCE);
		if (std::abs(target_xy - curr_xy) < LINEAR_NEAR_DISTANCE) {
			stop_generate_next_target = true;
			beeper.beepForCommand(VALID);

			if(sp_mode_->getNextMode() == sp_mode_->cm_navigation || sp_mode_->getNextMode() == sp_mode_->cm_exploration) {
				auto p_algo= boost::dynamic_pointer_cast<NavCleanPathAlgorithm>(p_clean_mode->clean_path_algorithm_);
				Points path;
				auto is_found = p_algo->generatePath(p_clean_mode->clean_map_, remain_path_.front(), p_clean_mode->iterate_point_.dir, path);
                ROS_INFO("remain:");
				p_algo->displayPointPath(remain_path_);
				ROS_INFO("new:");
				ROS_WARN("aaaa:is_found(%d),curr dir", is_found, p_clean_mode->iterate_point_.dir);
                ROS_WARN_COND(is_found, "front.dir(%d)",path.front().dir);
				p_algo->displayPointPath(path);
				if (is_found) {
					if (path.front().dir != (p_clean_mode->iterate_point_.dir + 2) % 4) {
						ROS_WARN("1path.front().dir(%d,%d)",path.front().dir, p_clean_mode->iterate_point_.dir);
                        auto front = path.front();
						path.pop_front();
						p_clean_mode->iterate_point_ = path.front();
                        p_clean_mode->iterate_point_.dir = front.dir;
						std::copy(path.begin(), path.end(),std::back_inserter(p_clean_mode->plan_path_));
						remain_path_.clear();
						std::copy(path.begin(), path.end(),std::back_inserter(remain_path_));
						ROS_WARN("switch ok !!!!!!!!!!!!%s,%d,curr(%d,%d), next target_point(%d,%d,%lf), dir(%d)",
								 __FUNCTION__,__LINE__,getPosition().toCell().x, getPosition().toCell().y, target_point_.toCell().x,target_point_.toCell().y,radian_to_degree(target_point_.th),
								 p_clean_mode->iterate_point_.dir);
						p_clean_mode->pubCleanMapMarkers(p_clean_mode->clean_map_, p_clean_mode->pointsGenerateCells(p_clean_mode->plan_path_));
					}
					else{
						ROS_ERROR("2path.front().dir(%d,%d)",path.front().dir, p_clean_mode->iterate_point_.dir);
					}
				}
			}
        }
	}
}


