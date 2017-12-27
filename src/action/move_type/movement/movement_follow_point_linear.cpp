//
// Created by lsy563193 on 12/5/17.
//

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
//	sp_mt_->sp_cm_->plan_path_display_sp_mt_->sp_cm_->plan_path_points();
//	g_is_should_follow_wall = false;
//	s_target = target;
//	sp_mt_->sp_cm_->tmp_plan_path_ = path;
}

bool MovementFollowPointLinear::calcTmpTarget(Point32_t& tmp_target) {
	auto p_clean_mode = (ACleanMode*)sp_mt_->sp_mode_;
	auto new_dir = p_clean_mode->new_dir_;
	auto curr = nav_map.getCurrPoint();
	tmp_target = nav_map.cellToPoint(p_clean_mode->plan_path_.front());
	auto curr_xy = (GridMap::isXDirection(new_dir)) ? curr.X : curr.Y;
	auto &target_xy = (GridMap::isXDirection(new_dir)) ? tmp_target.X : tmp_target.Y;
	auto tmp_xy = (GridMap::isXDirection(new_dir)) ? tmp_target.X : tmp_target.Y;
	auto is_beyond = (GridMap::isPositiveDirection(new_dir)) ? target_xy <= tmp_xy : target_xy >= tmp_xy;

	if (is_beyond && p_clean_mode->plan_path_.size() > 1) {
		p_clean_mode->old_dir_ = p_clean_mode->new_dir_;
		tmp_target = nav_map.cellToPoint(p_clean_mode->plan_path_.front());
		p_clean_mode->new_dir_ = (MapDirection) p_clean_mode->plan_path_.front().TH;
		p_clean_mode->plan_path_.pop_front();

		auto dis = std::min(std::abs(curr_xy - target_xy), (int32_t) (1.5 * CELL_COUNT_MUL));
		if (!GridMap::isPositiveDirection(new_dir))
			dis *= -1;
		target_xy = curr_xy + dis;

		ROS_INFO("%s,%d,dir(%d,%d)target(%d,%d)", __FUNCTION__, __LINE__, p_clean_mode->old_dir_, p_clean_mode->new_dir_,
						 (MapDirection) p_clean_mode->plan_path_.front().X, (MapDirection) p_clean_mode->plan_path_.front().Y);
		ROS_INFO("%s,%d,target,tmp(%d,%d)", __FUNCTION__, __LINE__, target_xy, tmp_xy);
	}

	return true;
}

bool MovementFollowPointLinear::isFinish()
{
	auto p_mt = (ActionFollowWall*)(sp_mt_);
	return isPoseReach() || isBoundaryStop() || isPassTargetStop() || p_mt->shouldMoveBack() || p_mt->shouldTurn() || p_mt->shouldTurn();
}

bool MovementFollowPointLinear::isCellReach()
{
	// Checking if robot has reached target cell.
	auto s_curr_p = nav_map.getCurrPoint();
	auto target_p = sp_mt_->target_point_;
	if (std::abs(s_curr_p.X - target_p.X) < CELL_COUNT_MUL_1_2 &&
		std::abs(s_curr_p.Y - target_p.Y) < CELL_COUNT_MUL_1_2)
	{
		ROS_INFO("\033[1m""%s, %d: MovementFollowPointLinear, reach the target cell (%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
						 sp_mt_->target_point_.X, sp_mt_->target_point_.Y);
//		g_turn_angle = ranged_angle(new_dir - robot::instance()->getPoseAngle());
		return true;
	}

	return false;
}

bool MovementFollowPointLinear::isPoseReach()
{
	// Checking if robot has reached target cell and target angle.
//	PP_INFO();
	auto target_angle = sp_mt_->target_point_.TH;
	if (isCellReach() && std::abs(ranged_angle(robot::instance()->getPoseAngle() - target_angle)) < 200)
	{
		ROS_INFO("\033[1m""%s, %d: MovementFollowPointLinear, reach the target cell and pose(%d,%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
				 sp_mt_->target_point_.X, sp_mt_->target_point_.Y, sp_mt_->target_point_.TH);
		return true;
	}
	return false;
}

bool MovementFollowPointLinear::isNearTarget()
{
//	auto p_clean_mode = boost::dynamic_pointer_cast<ACleanMode>(sp_mt_->sp_mode_);
//	auto new_dir = p_clean_mode->new_dir_;
//	auto s_curr_p = nav_map.getCurrPoint();
//	auto curr = (GridMap::isXDirection(new_dir)) ? s_curr_p.X : s_curr_p.Y;
//	auto target_p = sp_mt_->target_point_;
//	auto &target = (GridMap::isXDirection(new_dir)) ? target_p.X : target_p.Y;
//	//ROS_INFO("%s %d: s_curr_p(%d, %d), target_p(%d, %d), dir(%d)",
//	//		 __FUNCTION__, __LINE__, s_curr_p.X, s_curr_p.Y, target_p.X, target_p.Y, new_dir);
//	if ((GridMap::isPositiveDirection(new_dir) && (curr > target - 1.5 * CELL_COUNT_MUL)) ||
//		(!GridMap::isPositiveDirection(new_dir) && (curr < target + 1.5 * CELL_COUNT_MUL))) {
//		if(p_clean_mode->plan_path_.size() > 1)
//		{
//			// Switch to next target for smoothly turning.
//			new_dir = static_cast<MapDirection>(p_clean_mode->plan_path_.front().TH);
//			p_clean_mode->plan_path_.pop_front();
//			ROS_INFO("%s %d: Curr(%d, %d), switch next cell(%d, %d), new dir(%d).", __FUNCTION__, __LINE__,
//					 nav_map.getXCell(),
//					 nav_map.getYCell(), p_clean_mode->plan_path_.front().X, p_clean_mode->plan_path_.front().Y, new_dir);
//		}
//		else if(p_clean_mode->plan_path_.front() != g_zero_home && g_allow_check_path_in_advance)
//		{
//			g_check_path_in_advance = true;
//			ROS_INFO("%s %d: Curr(%d, %d), target(%d, %d), dir(%d), g_check_path_in_advance(%d)",
//					 __FUNCTION__, __LINE__, nav_map.getXCell(), nav_map.getYCell(),
//					 p_clean_mode->plan_path_.front().X, p_clean_mode->plan_path_.front().Y, new_dir, g_check_path_in_advance);
//			return true;
//		}
//	}
	return false;
}

bool MovementFollowPointLinear::isBoundaryStop()
{
//	PP_INFO();
	if (nav_map.isFrontBlockBoundary(2))
	{
		ROS_INFO("%s, %d: MovementFollowPointLinear, Blocked boundary.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

bool MovementFollowPointLinear::isPassTargetStop()
{
//	PP_INFO();
	// Checking if robot has reached target cell.
	auto p_clean_mode = (ACleanMode*)sp_mt_->sp_mode_;
	auto new_dir = p_clean_mode->new_dir_;
	auto s_curr_p = nav_map.getCurrPoint();
	auto curr = (GridMap::isXDirection(new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = (sp_mt_->target_point_);
	auto target = (GridMap::isXDirection(new_dir)) ? target_p.X : target_p.Y;
	if ((GridMap::isPositiveDirection(new_dir) && (curr > target + CELL_COUNT_MUL / 4)) ||
		(!GridMap::isPositiveDirection(new_dir) && (curr < target - CELL_COUNT_MUL / 4)))
	{
		ROS_INFO("%s, %d: MovementFollowPointLinear, pass target: new_dir(\033[32m%d\033[0m),is_x_axis(\033[32m%d\033[0m),is_pos(\033[32m%d\033[0m),curr(\033[32m%d\033[0m),target(\033[32m%d\033[0m)",
				 __FUNCTION__, __LINE__, new_dir, GridMap::isXDirection(new_dir), GridMap::isPositiveDirection(new_dir), curr, target);
		return true;
	}
	return false;
}

//void MovementFollowPointLinear::setTarget()
//{
//	turn_angle = ranged_angle(
//						course_to_dest(s_curr_p.X, s_curr_p.Y, cm_target_p_.X, cm_target_p_.Y) - robot::instance()->getPoseAngle());
//	s_target_p = nav_map.cellToPoint(p_clean_mode->tmp_plan_path_.back());
//	p_clean_mode->tmp_plan_path_ = p_clean_mode->tmp_plan_path_;
//}

void MovementFollowPointLinear::setBaseSpeed()
{
	base_speed_ = LINEAR_MIN_SPEED;
}
