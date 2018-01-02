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

	auto tmp_target = curr;
	auto &tmp_xy = (isXAxis(new_dir)) ? tmp_target.x : tmp_target.y;
//	ROS_WARN("curr_xy(%d), target_xy(%d)", curr_xy, target_xy);
	auto dis = std::min(std::abs(curr_xy - target_xy), (int32_t) (CELL_COUNT_MUL*0.75));
//	ROS_INFO("dis(%d)",dis);
	if (!isPos(new_dir))
		dis *= -1;
	tmp_xy = curr_xy + dis;
//	ROS_WARN("tmp(%d,%d)",tmp_target.x, tmp_target.y);
//	ROS_WARN("dis(%d),dir(%d),curr_xy(%d),tmp_xy(%d)",dis, GridMap::isPos(new_dir),curr_xy, tmp_xy);
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
	}

	return true;
}

bool MovementFollowPointLinear::isFinish()
{
	auto p_mt = (MoveTypeFollowWall*)(sp_mt_);
	return isPoseReach() || isPassTargetStop() || isRconStop() || p_mt->shouldMoveBack();
}

bool MovementFollowPointLinear::isRconStop()
{
	ev.rcon_triggered = countRconTriggered(c_rcon.getForwardTop());

	bool ret = false;
	if(ev.rcon_triggered)
	{
		ROS_WARN("%s %d: Rcon triggered and stop.", __FUNCTION__, __LINE__);
		ret = true;
	}

	return ret;
}

int MovementFollowPointLinear::countRconTriggered(uint32_t rcon_value)
{
	if(rcon_value == 0)
		return 0;

	int MAX_CNT = 1;
	if ( rcon_value& RconL_HomeT)
		rcon_cnt[left]++;
	if ( rcon_value& RconFL_HomeT)
		rcon_cnt[fl1]++;
	if ( rcon_value& RconFL2_HomeT)
		rcon_cnt[fl2]++;
	if ( rcon_value& RconFR2_HomeT)
		rcon_cnt[fr2]++;
	if ( rcon_value& RconFR_HomeT)
		rcon_cnt[fr1]++;
	if ( rcon_value& RconR_HomeT)
		rcon_cnt[right]++;
	auto ret = 0;
	for (int i = 0; i < 6; i++)
		if (rcon_cnt[i] > MAX_CNT) {
			rcon_cnt[left] = rcon_cnt[fl1] = rcon_cnt[fl2] = rcon_cnt[fr2] = rcon_cnt[fr1] = rcon_cnt[right] = 0;
			ret = i + 1;
			break;
		}
	return ret;
}

bool MovementFollowPointLinear::isCellReach()
{
	// Checking if robot has reached target cell.
	auto s_curr_p = getPosition();
	auto target_p = sp_mt_->target_point_;
	if (std::abs(s_curr_p.x - target_p.x) < CELL_COUNT_MUL_1_2 &&
		std::abs(s_curr_p.y - target_p.y) < CELL_COUNT_MUL_1_2)
	{
		ROS_INFO("\033[1m""%s, %d: MovementFollowPointLinear, reach the target cell (%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
						 sp_mt_->target_point_.x, sp_mt_->target_point_.y);
//		g_turn_angle = ranged_angle(new_dir - robot::instance()->getWorldPoseAngle());
		return true;
	}

	return false;
}

bool MovementFollowPointLinear::isPoseReach()
{
	// Checking if robot has reached target cell and target angle.
//	PP_INFO();
	auto target_angle = sp_mt_->target_point_.th;
	if (isCellReach() && std::abs(ranged_angle(robot::instance()->getWorldPoseAngle() - target_angle)) < 200)
	{
		ROS_INFO("\033[1m""%s, %d: MovementFollowPointLinear, reach the target cell and pose(%d,%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
				 sp_mt_->target_point_.x, sp_mt_->target_point_.y, sp_mt_->target_point_.th);
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
	auto s_curr_p = getPosition();
	auto curr = (isXAxis(new_dir)) ? s_curr_p.x : s_curr_p.y;
	auto target_p = (sp_mt_->target_point_);
	auto target = (isXAxis(new_dir)) ? target_p.x : target_p.y;
	if ((isPos(new_dir) && (curr > target + CELL_COUNT_MUL / 4)) ||
		(!isPos(new_dir) && (curr < target - CELL_COUNT_MUL / 4)))
	{
		ROS_WARN("%s, %d: MovementFollowPointLinear, pass target: new_dir(\033[32m%d\033[0m),is_x_axis(\033[32m%d\033[0m),is_pos(\033[32m%d\033[0m),curr(\033[32m%d\033[0m),target(\033[32m%d\033[0m)",
				 __FUNCTION__, __LINE__, new_dir, isXAxis(new_dir), isPos(new_dir), curr, target);
		return true;
	}
	return false;
}

void MovementFollowPointLinear::setBaseSpeed()
{
	base_speed_ = LINEAR_MIN_SPEED;
}
bool MovementFollowPointLinear::is_near()
{
	auto curr_p = getPosition();
	bool is_decrease_blocked = decrease_map.isFrontBlocked();
//	auto distance = two_points_distance(curr_p.x, curr_p.y, s_target_p.x, s_target_p.y);
	auto obstacle_distance_front = lidar.getObstacleDistance(0,ROBOT_RADIUS);
	return obs.getStatus() > 0 || /*(distance < SLOW_DOWN_DISTANCE) ||*/  (obstacle_distance_front < 0.25) || is_decrease_blocked;
}
