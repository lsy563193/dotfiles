//
// Created by lsy563193 on 11/29/17.
//
#include "pp.h"
//NavigationClean
NavigationClean::NavigationClean(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path) {
	g_plan_path.clear();
	s_curr_p = {cost_map.getXCount(), cost_map.getYCount()};
	auto target = cost_map.cellToPoint(target_cell);

	back_reg_ = new BackMovement();
	fw_reg_ = new FollowWallMovement(s_curr_p, target);
	line_reg_ = new ForwardMovement(target, path);
	gtc_reg_ = new GoToChargerMovement();
	turn_reg_ = new TurnMovement(0);
	mt_reg_ = line_reg_;

	if(cm_is_go_charger())
		mt_reg_ = gtc_reg_;
	p_reg_ = turn_reg_;

	path_next_nav(curr, g_plan_path);

	ROS_INFO("%s, %d: NavigationClean finish", __FUNCTION__, __LINE__);
	g_check_path_in_advance = false;

	g_passed_path.clear();
	g_passed_path.push_back(curr);

}

NavigationClean::~NavigationClean()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
}

bool NavigationClean::isReach()
{
	if (cs.is_trapped()) // For trapped status.
	{
		if (mt.is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with lidar.
			else if (isBack())
				return back_reg_->isReach();
		}
		else if (mt.is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isBlockCleared();
		}
	}

	else if (cs.is_tmp_spot()) // For temp spot during navigation.
	{
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching target.
			else if (isBack())
				return back_reg_->isReach();
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

	{
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isPoseReach() || line_reg_->isNearTarget();
			else if (isBack())
				return back_reg_->isReach();
		}
		else if (mt.is_follow_wall()) // Robot is going to new line.
		{
			if (isMt())
				return fw_reg_->isNewLineReach();
		}
	}

	return false;
}

bool NavigationClean::isExit()
{
	//ROS_INFO("%s %d: Run isExit", __FUNCTION__, __LINE__);
	if (cs.is_open_gyro() || cs.is_back_from_charger() || cs.is_align()
		|| cs.is_clean() || cs.is_go_home_point() || cs.is_tmp_spot() || cs.is_exploration()
		|| cs.is_self_check())
	{
		return CleanMode::isExit();
	}
	else if (cs.is_open_lidar())
	{
		return CleanMode::isExit() || lidar.openTimeOut();
	}
	else if (cs.is_open_slam())
	{
		return CleanMode::isExit() || slam.openTimeOut();
	}
	else if (cs.is_go_charger())
	{
		return CleanMode::isExit() || ev.charge_detect;
	}
	if (cs.is_trapped()) // For trapped status.
	{
		return CleanMode::isExit() || fw_reg_->isTimeUp();
	}
/*
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

	return false;*/
}

bool NavigationClean::isStop()
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

	else if (cs.is_tmp_spot()) // For temp spot during navigation.
	{
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLidarStop() || line_reg_->isBoundaryStop());
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
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLidarStop() || line_reg_->isBoundaryStop()
						|| line_reg_->isPassTargetStop());
			else if (isBack())
				return back_reg_->isLidarStop();
		}
		else if (mt.is_follow_wall()) // Robot is going to new line.
		{
			if (isMt())
				return fw_reg_->isOverOriginLine() || fw_reg_->isClosure(1) || fw_reg_->isIsolate();
			else if (isBack())
				return back_reg_->isLidarStop();
		}
	}


	return false;
}

bool NavigationClean::isSwitch()
{
	auto old_p_reg_ = p_reg_;
	if (cs.is_trapped()) // For trapped status.
	{
		if (mt.is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
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

	else if (cs.is_tmp_spot()) // For temp spot during navigation.
	{
		if (mt.is_linear()) // Robot is cleaning current line.
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
		if (mt.is_linear()) // Robot is cleaning current line.
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
				if (line_reg_->shouldMoveBack())
					p_reg_ = back_reg_;
				else if (line_reg_->isCellReach())
					p_reg_ = turn_reg_;
			}
		}
		else if (mt.is_follow_wall()) // Robot is going to new line.
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
					p_reg_ = back_reg_;
				}
				else if (fw_reg_->shouldTurn())
				{
					g_time_straight = 0;
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

	if (old_p_reg_ != p_reg_)
	{
		if(p_reg_ == mt_reg_ && mt_reg_ == line_reg_)
			line_reg_->setBaseSpeed();
		ROS_INFO("%s %d: Switch from %s to %s.", __FUNCTION__, __LINE__, (old_p_reg_->getName()).c_str(), (p_reg_->getName()).c_str());
		setTarget();
		return true;
	}

	return false;
}

bool NavigationClean::csm_next(Cell_t &curr)
{
	ROS_INFO("%s %d: cs:%d, current(%d %d), g_check_path_in_advance:%d", __FUNCTION__, __LINE__, cs.get(),
			 cost_map.getXCell(), cost_map.getYCell(), g_check_path_in_advance);
//	if (cs.is_clean() && g_check_path_in_advance)
//	{
//		printf("\n\033[42m========================Generate path in advance==============================\033[0m\n");
//		mark();
//		int16_t current_dir = g_new_dir;
//		int16_t temp_new_dir;
//
//		auto start_cell = g_plan_path.back();
//		ROS_INFO("%s %d: start cell(%d %d)", __FUNCTION__, __LINE__, start_cell.X, start_cell.Y);
//		PPTargetType path_in_advance;
//		if (path_next_nav_in_advance(temp_new_dir, start_cell, path_in_advance))
//		{
//			if (mt.should_follow_wall(current_dir, start_cell, path_in_advance)) {
//				// This is for follow wall case.
//				g_allow_check_path_in_advance = false;
//				ROS_INFO("%s %d: Fail for wall follow case.", __FUNCTION__, __LINE__);
//			}
//			else if ((IS_X_AXIS(current_dir) && IS_X_AXIS(temp_new_dir)) || (IS_Y_AXIS(current_dir) && IS_Y_AXIS(temp_new_dir))) // Old path and new path are in the same axis.
//			{
//				if (IS_POS_AXIS(current_dir) ^ IS_POS_AXIS(temp_new_dir))// For cases that target is at opposite direction.
//				{
//					g_allow_check_path_in_advance = false;
//					ROS_INFO("%s %d: Fail for opposite direction case.", __FUNCTION__, __LINE__);
//				}
//			}
//
//			if (g_allow_check_path_in_advance) // Switch new path.
//			{
//				g_plan_path.clear();
//				for (auto cell : path_in_advance)
//					g_plan_path.push_back(cell);
//				g_new_dir = temp_new_dir;
//				ROS_INFO("%s %d: Switch to new path.", __FUNCTION__, __LINE__);
//			}
//		}
//		else
//		{
//			g_allow_check_path_in_advance = false;
//			ROS_INFO("%s %d: Fail for no target case.", __FUNCTION__, __LINE__);
//		}
//
//		g_check_path_in_advance = false;
//		printf("\033[44m======================Generate path in advance End============================\033[0m\n\n");
//		return true;
//	}
//	else
//		return find_target(curr);
	printf("\n\033[42m======================================Generate path and update move type===========================================\033[0m\n");
	mark();
	auto previous_cs = cs.get();
	g_old_dir = g_new_dir; // Save current direction.
	g_plan_path.clear();
//	cs_path_next(curr, g_plan_path);
	while (cs.cs_next(curr,g_plan_path))

	display();

	if (!(previous_cs == CS_TRAPPED && cs.is_trapped())) {
		setMt();
		g_passed_path.clear();
	}
	g_allow_check_path_in_advance = true;

	printf("\033[44m====================================Generate path and update move type End=========================================\033[0m\n\n");
	return !g_plan_path.empty();

}
void NavigationClean::mark()
{
	ROS_INFO("%s, %d: NavigationClean::mark", __FUNCTION__, __LINE__);
	cost_map.saveBlocks();

//	uint8_t block_count = 0;
	cost_map.setObs();
	cost_map.setBumper();
	cost_map.setCliff();
	cost_map.setTilt();
	cost_map.setSlip();
	cost_map.setLidar();

	if(mt.is_follow_wall())
		cost_map.setFollowWall();
	if(cs.is_trapped())
		fw_map.setFollowWall();

	cost_map.setCleaned(g_passed_path);
	cost_map.markRobot(CLEAN_MAP);
	cost_map.setRcon();
//	cost_map.print(CLEAN_MAP,0,0);
}

Cell_t NavigationClean::updatePosition(const Point32_t &curr_point)
{
	auto curr = CleanMode::updatePosition(curr_point);
	return updatePath(curr);
}
