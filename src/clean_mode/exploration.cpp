//
// Created by lsy563193 on 11/29/17.
//
#include "pp.h"
//Exploration
Exploration::Exploration(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path) {
	g_plan_path.clear();
	s_curr_p = {nav_map.getXCount(), nav_map.getYCount()};
	auto target = nav_map.cellToPoint(target_cell);

	back_reg_ = new BackMovement();
	fw_reg_ = new FollowWallMovement(s_curr_p, target);
	line_reg_ = new ForwardMovement(target, path);
	gtc_reg_ = new GoToChargerMovement();
	turn_reg_ = new TurnMovement(0);
	mt_reg_ = line_reg_;

	p_reg_ = mt_reg_;


	ROS_INFO("%s, %d: Exploration finish", __FUNCTION__, __LINE__);

	g_check_path_in_advance = false;

	g_passed_path.clear();
	g_passed_path.push_back(curr);

}

Exploration::~Exploration()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
}

bool Exploration::isReach()
{
	if (cs.is_trapped()) // For trapped status.
	{
		if (mt.is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
		{
			if (isMt())
			{
				return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with lidar.
			}
			else if (isBack())
				return back_reg_->isReach();
		}
		else if (mt.is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isBlockCleared();
		}
	}

	else if (cs.is_going_home())
	{
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching target.
			else if (isBack())
				return back_reg_->isReach();
		}
	}

	else if (cm_is_exploration())
	{
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching target.
			else if (isBack())
				return back_reg_->isReach();
		}
	}

	return false;
}

bool Exploration::isExit()
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

bool Exploration::isStop()
{
	if(CleanMode::isStop())
		return true;

	if (cs.is_trapped()) // For trapped status.
	{
		if (mt.is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLidarStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLidarStop();
		}
		else if (mt.is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isOverOriginLine() || fw_reg_->isClosure(2);
			else if (isBack())
				return back_reg_->isLidarStop();
		}
	}

	else if (cs.is_going_home())
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

	else if (cs.is_clean())
	{
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
	}


	return false;
}

bool Exploration::isSwitch()
{
	auto old_p_reg_ = p_reg_;
	if (cs.is_trapped()) // For trapped status.
	{
		if (mt.is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
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
		else if (mt.is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isTurn())
			{
				if (turn_reg_->isReach())
					p_reg_ = mt_reg_;
				else if (turn_reg_->shouldMoveBack())
					p_reg_ = back_reg_;
			}
			else if (isMt())
			{
				if (fw_reg_->shouldMoveBack())
				{
					g_time_straight = 0.2;
					g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
					p_reg_ = back_reg_;
				}
				else if (fw_reg_->shouldTurn())
				{
					g_time_straight = 0;
					g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
					p_reg_ = turn_reg_;
				}
			}
			else if (isBack() && back_reg_->isReach())
			{
				p_reg_ = turn_reg_;
				resetTriggeredValue();
			}
		}
	}

	else if (cs.is_going_home())
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

	else if (cs.is_clean())
	{
		if (mt.is_linear()) // Robot is going straight to find charger.
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

	if (old_p_reg_ != p_reg_)
	{
		ROS_INFO("%s %d: Switch from %s to %s.", __FUNCTION__, __LINE__, (old_p_reg_->getName()).c_str(), (p_reg_->getName()).c_str());
		setTarget();
		return true;
	}

	return false;
}

bool Exploration::csm_next(Cell_t &curr)
{
	find_target(curr);
}

