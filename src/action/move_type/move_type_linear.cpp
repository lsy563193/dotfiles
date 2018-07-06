//
// Created by lsy563193 on 12/4/17.
//
#include <event_manager.h>
#include <move_type.hpp>
#include <mode.hpp>
#include <beeper.h>
#include <cliff.h>
#include "robot.hpp"

MoveTypeLinear::MoveTypeLinear(Points remain_path)
{
	IMovement::sp_mt_ = this;
	resetTriggeredValue();
	remain_path.pop_front();

	auto p_mode = dynamic_cast<ACleanMode *>(sp_mode_);
	auto target_point_ = std::next(p_mode->iterate_point_);
	turn_target_radian_ = p_mode->iterate_point_->th;

	ROS_WARN("%s,%d: Enter, angle(%.2f,%.2f, %.2f),  target(%.3f, %.3f).",
			 __FUNCTION__, __LINE__, getPosition().th, radian_to_degree(target_point_->th),
			 radian_to_degree(turn_target_radian_), target_point_->x, target_point_->y);
//	ROS_WARN("curr(%lf,%lf)", getPosition().x, getPosition().y);

	movement_i_ = p_mode->isGyroDynamic() ? mm_dynamic : mm_turn;
	if (movement_i_ == mm_dynamic)
		sp_movement_.reset(new MovementGyroDynamic());
	else
		sp_movement_.reset(new MovementTurn(turn_target_radian_, ROTATE_TOP_SPEED));

}

MoveTypeLinear::~MoveTypeLinear()
{
	if(sp_mode_ != nullptr){
		auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
		p_mode->saveBlocks();
		p_mode->mapMark();
	}
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool MoveTypeLinear::isFinish()
{
	if (IMoveType::isFinish() && isNotHandleEvent())
	{
		ROS_WARN("%s %d: Move type aborted.", __FUNCTION__, __LINE__);
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
//				ROS_WARN("%s %d: turn_target_radian_:%f", __FUNCTION__, __LINE__, radian_to_degree(turn_target_radian_));
//				odom_turn_target_radians_ =  ranged_radian(turn_target_radian_  - getPosition().th + odom.getRadian());
//				odom_turn_target_radians_.push_back(odom.getRadian());
				radian_diff_count = 0;

			}
		}
		else if(movement_i_ == mm_stay)
		{
			beeper.debugBeep(VALID);
			if(cliff.getStatus())
			{
				ROS_WARN("ev.cliff_triggered(%d)!!!", cliff.getStatus());
				return true;
			}
			movement_i_ = mm_forward;
			sp_movement_.reset(new MovementFollowPointLinear());
			return false;
		}
		else if (movement_i_ == mm_forward)
		{
			if(!handleMoveBackEventForward(p_clean_mode)){
				if(ev.cliff_triggered)
				{
					beeper.debugBeep(VALID);
					ROS_WARN("ev.cliff_triggered(%d)!!!", ev.cliff_triggered);
					movement_i_ = mm_stay;
					sp_movement_.reset(new MovementStay(CLIFF_STAY_TIME_));
					return false;
				}
				if(!ev.tilt_triggered)
					p_clean_mode->should_follow_wall = true;
//				ROS_WARN("111should_follow_wall(%d)!!!", p_clean_mode->shouldFollowWall);
				ROS_INFO("%s %d: ", __FUNCTION__, __LINE__);
				return true;
			}else {
//				ROS_WARN("111should_follow_wall(%d,%d)!!!", p_clean_mode->shouldFollowWall, ev.tilt_triggered);
				if (ev.bumper_triggered || ev.lidar_triggered || ev.rcon_status /*|| ev.obs_triggered*/) {
					p_clean_mode->should_follow_wall = true;
				}
//				ROS_WARN("222should_follow_wall(%d)!!!", p_clean_mode->shouldFollowWall);
//				ROS_INFO_FL();
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
	auto target_point_ = std::next(p_clean_mode->iterate_point_ );
	auto delta_x = std::abs(s_curr_p.x - target_point_->x);
	auto delta_y = std::abs(s_curr_p.y - target_point_->y);
//	ROS_INFO("delta(%lf, %lf)", delta_x, delta_y);
	if (delta_x < CELL_SIZE/2 &&
			delta_y< CELL_SIZE/2)
	{
//		ROS_INFO("%s, %d: MoveTypeLinear,current cell = (%d,%d) reach the target cell (%d,%d), current angle(%lf), target angle(%lf).", __FUNCTION__, __LINE__,
//						 s_curr_p.toCell().x,s_curr_p.toCell().y,target_point_.toCell().x, target_point_.toCell().y, radian_to_degree(s_curr_p.th), radian_to_degree(target_point_.th));
//		g_turn_angle = ranged_radian(new_dir - robot::instance()->getWorldPoseRadian());
		PP_INFO();
		return true;
	}

	return false;
}

bool MoveTypeLinear::isPoseReach()
{
	// Checking if robot has reached target cell and target angle.
//	PP_INFO();
	auto p_clean_mode = dynamic_cast<ACleanMode*> (sp_mode_);
	auto target_point_ = *std::next(p_clean_mode->iterate_point_ );
	if (isCellReach() ) {
		//do not use this isRadianNear cause it is useless now
//		if (std::abs(getPosition().isRadianNear(target_point_))) {
//			ROS_INFO("\033[1m""%s, %d: MoveTypeLinear, reach the target cell and pose(%d,%d,%f,%d)""\033[0m", __FUNCTION__,
//							 __LINE__,
//							 target_point_.toCell().x, target_point_.toCell().y, target_point_.th,target_point_.dir);
			return true;
//		}
	}
	return false;
}

bool MoveTypeLinear::isPassTargetStop()
{
//	PP_INFO();
	// Checking if robot has reached target cell.
	auto p_clean_mode = dynamic_cast<ACleanMode*> (sp_mode_);
	return getPosition().project_ratio(*p_clean_mode->iterate_point_,*(p_clean_mode->iterate_point_+1)) >= 1;
}

bool MoveTypeLinear::isLinearForward()
{
	return movement_i_ == mm_forward;
}

void MoveTypeLinear::switchLinearTarget(ACleanMode * p_clean_mode)
{
	auto target_size = std::distance(p_clean_mode->iterate_point_, p_clean_mode->plan_path_.end());
	auto leng = p_clean_mode->iterate_point_->Distance(*(p_clean_mode->iterate_point_+1));
	auto r = getPosition().project_ratio(*p_clean_mode->iterate_point_, *(p_clean_mode->iterate_point_+1));
	if (target_size>2) {
		if (r >= (leng - LINEAR_NEAR_DISTANCE)/leng) {
			if (p_clean_mode->action_i_ == p_clean_mode->ac_linear)
				p_clean_mode->mapMark();
			if (robot::instance()->getRobotWorkMode() == Mode::cm_exploration && p_clean_mode->isStateExploration()) {
				if (switchLinearTargetByRecalc(p_clean_mode)) {
					radian_diff_count = 0;
				}
			}
			p_clean_mode->iterate_point_++;
			ROS_INFO("%s,%d,it(%d,%d)", __FUNCTION__, __LINE__, p_clean_mode->iterate_point_->toCell().x,
					 p_clean_mode->iterate_point_->toCell().y);
		}
	}
	else if(target_size==2){
		if(stop_generate_next_target)
			return;

		if (robot::instance()->getRobotWorkMode() == Mode::cm_navigation)
		{
			if(!(p_clean_mode->isStateClean() && p_clean_mode->action_i_ == p_clean_mode->ac_linear))
				return;
		}
		else if (robot::instance()->getRobotWorkMode() == Mode::cm_exploration)
		{
			if(!(p_clean_mode->isStateExploration() && p_clean_mode->action_i_ == p_clean_mode->ac_linear))
				return;
		}
		else
			return;

		if (r >= (leng - LINEAR_NEAR_DISTANCE)/leng)
		{
			p_clean_mode->mapMark();
			stop_generate_next_target = true;
			if(switchLinearTargetByRecalc(p_clean_mode))
			{
				p_clean_mode->iterate_point_++;
				stop_generate_next_target = false;
			}
		}
	}
}

bool MoveTypeLinear::switchLinearTargetByRecalc(ACleanMode *p_clean_mode) {
	bool val{};
	Points path;
	//comment temporary
//	p_clean_mode->saveBlocks();
//	resetTriggeredValue();

	auto target_point = std::next(p_clean_mode->iterate_point_);
	auto is_found = boost::dynamic_pointer_cast<NavCleanPathAlgorithm>( p_clean_mode->clean_path_algorithm_)->generatePath(p_clean_mode->clean_map_, *target_point, path);
	ROS_INFO("%s %d: is_found:(%d), remain:", __FUNCTION__, __LINE__, is_found);
	displayPointPath(path);
	if (is_found) {
		ROS_INFO("5555555555555555555555555555555555555555");
		if (std::abs(path.front().th - p_clean_mode->iterate_point_->th) < PI*3/4) {
			ROS_INFO("6666666666666666666666666666666666666666");
			p_clean_mode->plan_path_.erase(p_clean_mode->iterate_point_+1,p_clean_mode->plan_path_.end());
			std::move(path.begin(),path.end(),std::back_inserter(p_clean_mode->plan_path_));
			displayPointPath(p_clean_mode->plan_path_);
			p_clean_mode->pubCleanMapMarkers(p_clean_mode->clean_map_,
											 *points_to_cells(p_clean_mode->plan_path_));

			ROS_INFO("7777777777777777777777777777777777777777");
			val = true;
		} else {
			ROS_INFO("%s %d: Opposite dir curr", __FUNCTION__, __LINE__ );
		}
	}
	return val;
}
