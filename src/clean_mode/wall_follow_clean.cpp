//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"

//WallFollowClean
WallFollowClean::WallFollowClean(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path) {
	g_plan_path.clear();
	s_curr_p = {cost_map.getXCount(), cost_map.getYCount()};
	auto target = cost_map.cellToPoint(target_cell);

	back_reg_ = new BackMovement();
	fw_reg_ = new FollowWallMovement(s_curr_p, target);
	line_reg_ = new ForwardMovement(target, path);
	gtc_reg_ = new GoToChargerMovement();
	turn_reg_ = new TurnMovement(0);
	mt_reg_ = line_reg_;

	p_reg_ = mt_reg_;
	path_next_fw(curr);


	ROS_INFO("%s, %d: WallFollowClean finish", __FUNCTION__, __LINE__);

	g_passed_path.clear();
	g_passed_path.push_back(curr);

}

WallFollowClean::~WallFollowClean()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
}

bool WallFollowClean::isReach() {
	if (cs.is_going_home()) {
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching target.
			else if (isBack())
				return back_reg_->isReach();
		}
	}

	if (mt.is_linear()) {
		if (isMt())
			return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with lidar.
	}
	else if (mt.is_follow_wall()) {
		if (isMt())
			return fw_reg_->isClosure(1);
	}
	return false;
}

bool WallFollowClean::isExit()
{
	if(CleanMode::isExit())
		return true;
	if (cs.is_trapped()) // For trapped status.
	{
		if (mt.is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isTimeUp();
		}
	}

	return false;
}

bool WallFollowClean::isStop()
{

	if(CleanMode::isStop())
		return true;

	if (cs.is_going_home())
	{
		if (mt.is_linear())
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLidarStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLidarStop();
		}
	}

	if (cm_is_follow_wall()) {
		if (mt.is_linear()) // 1. Going straight to find the wall at the beginning of wall follow mode_.
			// 2. Passed path is a closure or passed path is isolate, need to go straight to another wall.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
								|| line_reg_->isLidarStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLidarStop();
		}
		else if (mt.is_follow_wall()) // Robot is following wall for cleaning.
		{
			if (isMt())
				return fw_reg_->isIsolate();
			else if (isBack())
				return back_reg_->isLidarStop();
		}
	}
	return false;
}

bool WallFollowClean::isSwitch()
{
	auto old_p_reg_ = p_reg_;

	if (cs.is_going_home())
	{
		if (mt.is_linear()) // Robot is going straight with path.
		{
			if (isTurn())
			{
				if (turn_reg_->isReach())
					p_reg_ = mt_reg_;
				else if (turn_reg_->shouldMoveBack())
					p_reg_ = back_reg_;
			}
			else if (isMt() && line_reg_->shouldMoveBack())
				p_reg_ = back_reg_;
		}
	}

	else if (cs.is_clean()) {
		if (mt.is_linear()) // 1. Going straight to find the wall at the beginning of wall follow mode_.
			// 2. Passed path is a closure or passed path is isolate, need to go straight to another wall.
		{
			if (isTurn()) {
				if (turn_reg_->isReach())
					p_reg_ = mt_reg_;
				else if (turn_reg_->shouldMoveBack())
					p_reg_ = back_reg_;
			}
			else if (isMt() && line_reg_->shouldMoveBack())
				p_reg_ = back_reg_;
		}
		else if (mt.is_follow_wall()) // Robot is following wall for cleaning.
		{
			if (isTurn()) {
				if (turn_reg_->isReach())
					p_reg_ = mt_reg_;
				else if (turn_reg_->shouldMoveBack())
					p_reg_ = back_reg_;
			}
			else if (isMt()) {
				if (fw_reg_->shouldMoveBack()) {
					g_time_straight = 0.2;
					g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
					p_reg_ = back_reg_;
				}
				else if (fw_reg_->shouldTurn()) {
					g_time_straight = 0;
					g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
					p_reg_ = turn_reg_;
				}
			}
			else if (isBack() && back_reg_->isReach()) {
				p_reg_ = turn_reg_;
				resetTriggeredValue();
			}
		}
	}

	if (old_p_reg_ != p_reg_)
	{
		ROS_INFO("%s %d: Switch from %s to %s.", __FUNCTION__, __LINE__, (old_p_reg_->getName()).c_str(), (p_reg_->getName()).c_str());
		setTarget();
		return true;
	}

	return false;
}

void WallFollowClean::mark() {

	fw_map.saveBlocks();
//	uint8_t block_count = 0;
	if(mt.is_follow_wall()) {
		fw_map.setObs();
		fw_map.setBumper();
		fw_map.setCliff();
		fw_map.setTilt();
		fw_map.setSlip();
		fw_map.setLidar();
		fw_map.setFollowWall();
		fw_map.setRcon();
	}

//	cost_map.set_cleaned(g_passed_path);
//	cost_map.mark_robot(CLEAN_MAP);
}

bool WallFollowClean::csm_next(Cell_t &curr)
{
	printf("\n\033[42m======================================WallFollowClean===========================================\033[0m\n");
	mark();
	g_plan_path.clear();
	if(!path_next_fw(curr))
		return false;
	display();
	setMt();
	g_passed_path.clear();
	g_check_path_in_advance = false;
	printf("\033[44m====================================WallFollowClean=========================================\033[0m\n\n");
	return true;
}

Cell_t WallFollowClean::updatePosition(const Point32_t &curr_point)
{
	auto curr = CleanMode::updatePosition(curr_point);
	return updatePath(curr);
}
