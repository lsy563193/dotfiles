//
// Created by lsy563193 on 17-9-27.
//

#include <pp.h>
#include <path_planning.h>
#include <wall_follow.h>

static uint8_t clean_mode = 0;

bool cm_is_navigation()
{
	return clean_mode == Clean_Mode_Navigation;
}

bool cm_is_follow_wall()
{
	return clean_mode == Clean_Mode_WallFollow;
}

bool cm_is_exploration()
{
	return clean_mode == Clean_Mode_Exploration;
}

bool cm_is_spot()
{
	return clean_mode == Clean_Mode_Spot;
}

bool cm_is_go_charger()
{
	return clean_mode == Clean_Mode_Go_Charger;
}

void cm_set(uint8_t mode)
{
	clean_mode = mode;
}

uint8_t cm_get(void)
{
	return clean_mode;
}

void CleanMode::run()
{
	bool eh_status_now = false, eh_status_last = false;

	while (ros::ok())
	{
		if (isExit()) {
			break;
		}

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		auto curr = updatePosition({cost_map.get_x_count(), cost_map.get_y_count()});

		if (isReach() || isStop())
		{
	printf("\n\033[42m======================================Generate path and update move type===========================================\033[0m\n");
			if(!csm_next(curr)) //cs manage
				return;
		}
		if (mt.is_linear()) {
			wall.dynamic_base(30);
		}

		if (isSwitch()) {
			CostMap::save_blocks();
		}

		int32_t left_speed = 0, right_speed = 0;
		adjustSpeed(left_speed, right_speed);
	}
}

//todo
void CleanMode::mark() {

};

bool CleanMode::isStop(){
		return Movement::isStop();
};

bool CleanMode::isExit(){
	return Movement::isExit();
};

void CleanMode::setMt()
{
	s_origin_p = {cost_map.get_x_count(), cost_map.get_y_count()};
	if(mt.is_follow_wall())
	{
		s_target_p = cost_map.cell_to_point(g_plan_path.back());
		mt_reg_ = fw_reg_;
		if(cm_is_follow_wall()) {
			ROS_INFO("%s %d: obs(\033[32m%d\033[0m), rcon(\033[32m%d\033[0m), bum(\033[32m%d\033[0m), cliff(\033[32m%d\033[0m), tilt(\033[32m%d\033[0m),slip(\033[32m%d\033[0m)",
							 __FUNCTION__, __LINE__, ev.obs_triggered, ev.rcon_triggered, ev.bumper_triggered, ev.cliff_triggered,
							 ev.tilt_triggered, g_robot_slip);
			int16_t block_angle = 0;
			if (ev.obs_triggered)
				block_angle = obs_turn_angle();
			else if (ev.bumper_triggered)
				block_angle = bumper_turn_angle();
			else if (ev.cliff_triggered)
				block_angle = cliff_turn_angle();
			else if (ev.tilt_triggered)
				block_angle = tilt_turn_angle();
				//	else if (ev.rcon_triggered)
				//		block_angle = rcon_turn_angle();
			else
				block_angle = 0;
			if (LASER_FOLLOW_WALL)
				if(!laser_turn_angle(g_turn_angle))
					g_turn_angle = ranged_angle( course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - robot::instance()->getPoseAngle());
		}else{
//			ROS_INFO("%s,%d: mt.is_fw",__FUNCTION__, __LINE__);
			if (LASER_FOLLOW_WALL)
				if(!laser_turn_angle(g_turn_angle))
					g_turn_angle = ranged_angle( course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - robot::instance()->getPoseAngle());
		}
		ROS_INFO("%s,%d: mt.is_follow_wall, s_target_p(%d, %d).",__FUNCTION__, __LINE__, s_target_p.X, s_target_p.Y);

	}else if(mt.is_linear())
	{
		ROS_INFO("%s,%d: mt.is_linear",__FUNCTION__, __LINE__);
		s_target_p = cost_map.cell_to_point(g_plan_path.front());
		mt_reg_ = line_reg_;
		g_turn_angle = ranged_angle(
					course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - robot::instance()->getPoseAngle());
	}
	else if (mt.is_go_to_charger())
	{
		mt_reg_ = gtc_reg_;
		g_turn_angle = 0;
	}
	p_reg_ = turn_reg_;
//	s_target_angle = g_turn_angle;
	s_target_angle = ranged_angle(robot::instance()->getPoseAngle() + g_turn_angle);
	ROS_INFO("%s,%d,curr(%d), g_turn_angle(%d), set_target_angle(%d)",__FUNCTION__, __LINE__, robot::instance()->getPoseAngle(), g_turn_angle, s_target_angle);
	resetTriggeredValue();
	g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
	bumper_turn_factor = 0.85;
	g_bumper_cnt = g_cliff_cnt = 0;
	g_slip_cnt = 0;
	g_slip_backward = false;
	c_rcon.resetStatus();
	robot::instance()->obsAdjustCount(20);
}

bool CleanMode::csm_next(Cell_t &curr) {
	return false;
}

bool is_equal_with_angle(const Cell_t &l, const Cell_t &r)
{
	return  l == r && std::abs(ranged_angle(l.TH - r.TH)) < 200;
}

Cell_t CleanMode::updatePosition(const Point32_t &curr_point)
{
		cost_map.update_position();
		s_curr_p = curr_point;
		return cost_map.get_curr_cell();
}

Cell_t CleanMode::updatePath(const Cell_t& curr)
{
	if (!is_equal_with_angle(curr, last_)) {
		last_ = curr;
		auto loc = std::find_if(g_passed_path.begin(), g_passed_path.end(), [&](Cell_t it) {
				return is_equal_with_angle(curr, it);
		});
		auto distance = std::distance(loc, g_passed_path.end());
		if (distance == 0) {
			g_passed_path.push_back(curr);
		}
		if (distance > 5) {
			g_passed_path.clear();
			g_wf_reach_count++;
		}
		CostMap::save_blocks();
	}
//	else
//		is_time_up = !cs.is_trapped();
	return curr;
}

void CleanMode::display()
{
	path_display_path_points(g_plan_path);
	robot::instance()->pubCleanMapMarkers(cost_map, g_plan_path);
}

void CleanMode::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	if (p_reg_ != nullptr)
		p_reg_->adjustSpeed(left_speed, right_speed);
	left_speed_ = left_speed;
	right_speed_ = right_speed;

#if GLOBAL_PID
		/*---PID is useless in wall follow mode_---*/
		if(isMt() && mt.is_follow_wall())
			wheel.setPidTargetSpeed(left_speed, right_speed, REG_TYPE_WALLFOLLOW);
		else if(isMt() && mt.is_linear())
			wheel.setPidTargetSpeed(left_speed, right_speed, REG_TYPE_LINEAR);
		else if(isMt() && mt.is_go_to_charger())
			wheel.setPidTargetSpeed(left_speed, right_speed, REG_TYPE_NONE);
		else if(isBack())
			wheel.setPidTargetSpeed(left_speed, right_speed, REG_TYPE_BACK);
		else if(isTurn())
			wheel.setPidTargetSpeed(left_speed, right_speed, REG_TYPE_TURN);
#else
		/*---PID is useless in wall follow mode_---*/
		wheel.set_speed(speed_left, speed_right, REG_TYPE_NONE);
#endif
}

void CleanMode::resetTriggeredValue(void)
{
	ev.laser_triggered = 0;
	ev.rcon_triggered = 0;
	ev.bumper_triggered = 0;
	ev.obs_triggered = 0;
	ev.cliff_triggered = 0;
	ev.tilt_triggered = 0;
}

bool CleanMode::find_target(Cell_t& curr)
{
//	printf("\n\033[42m======================================Generate path and update move type===========================================\033[0m\n");
//	mark();
//	auto previous_cs = cs.get();
//	g_old_dir = g_new_dir; // Save current direction.
//	g_plan_path.clear();
//	cs_path_next(curr, g_plan_path);
//
//	display();
//
//	if (!(previous_cs == CS_TRAPPED && cs.is_trapped())) {
//		setMt();
//		g_passed_path.clear();
//	}
//	g_allow_check_path_in_advance = true;
//
//	printf("\033[44m====================================Generate path and update move type End=========================================\033[0m\n\n");
//	return !g_plan_path.empty();
}

//NavigationClean
NavigationClean::NavigationClean(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path) {
	g_plan_path.clear();
	s_curr_p = {cost_map.get_x_count(),cost_map.get_y_count()};
	auto target = cost_map.cell_to_point(target_cell);

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
				return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with laser.
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
	else if (cs.is_open_laser())
	{
		return CleanMode::isExit() || laser.openTimeOut();
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
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
		else if (mt.is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isOverOriginLine() || fw_reg_->isClosure(2);
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs.is_tmp_spot()) // For temp spot during navigation.
	{
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs.is_going_home())
	{
		if (mt.is_linear())
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs.is_clean())
	{
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop()
						|| line_reg_->isPassTargetStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
		else if (mt.is_follow_wall()) // Robot is going to new line.
		{
			if (isMt())
				return fw_reg_->isOverOriginLine() || fw_reg_->isClosure(1) || fw_reg_->isIsolate();
			else if (isBack())
				return back_reg_->isLaserStop();
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
		ROS_INFO("%s %d: Switch from %s to %s.", __FUNCTION__, __LINE__, (old_p_reg_->getName()).c_str(), (p_reg_->getName()).c_str());
		setTarget();
		return true;
	}

	return false;
}

bool NavigationClean::csm_next(Cell_t &curr)
{
	ROS_INFO("%s %d: cs:%d, current(%d %d), g_check_path_in_advance:%d", __FUNCTION__, __LINE__, cs.get(), cost_map.get_x_cell(), cost_map.get_y_cell(), g_check_path_in_advance);
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
	CostMap::save_blocks();

//	uint8_t block_count = 0;
	cost_map.set_obs();
	cost_map.set_bumper();
	cost_map.set_rcon();
	cost_map.set_cliff();
	cost_map.set_tilt();
	cost_map.set_slip();
	cost_map.set_laser();

	if(mt.is_follow_wall())
		cost_map.set_follow_wall();
	if(cs.is_trapped())
		fw_map.set_follow_wall();

	cost_map.set_cleaned(g_passed_path);
	cost_map.mark_robot(MAP);
//	cost_map.print(MAP,0,0);
}

Cell_t NavigationClean::updatePosition(const Point32_t &curr_point)
{
	auto curr = CleanMode::updatePosition(curr_point);
	return updatePath(curr);
}
//SpotClean
SpotClean::SpotClean(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path)
{
	g_plan_path.clear();
	s_curr_p = {cost_map.get_x_count(),cost_map.get_y_count()};
	auto target = cost_map.cell_to_point(target_cell);

	back_reg_ = new BackMovement();
	fw_reg_ = new FollowWallMovement(s_curr_p, target);
	line_reg_ = new ForwardMovement(target, path);
	gtc_reg_ = new GoToChargerMovement();
	turn_reg_ = new TurnMovement(0);
	mt_reg_ = line_reg_;

	p_reg_ = mt_reg_;


	ROS_INFO("%s, %d: SpotClean finish", __FUNCTION__, __LINE__);
	g_check_path_in_advance = false;

	g_passed_path.clear();
	g_passed_path.push_back(curr);

}

SpotClean::~SpotClean()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
}

void SpotClean::mark()
{
	CleanMode::mark();

	if (ev.rcon_triggered || ev.obs_triggered || ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered)
	{
		SpotType spt = SpotMovement::instance()->getSpotType();
		if (spt == CLEAN_SPOT || spt == NORMAL_SPOT)
			SpotMovement::instance()->setOBSTrigger();
	}
}

bool SpotClean::isReach()
{
	if (mt.is_linear()) // Robot is cleaning current line.
	{
		if (isMt())
		{
			return line_reg_->isCellReach(); // For reaching target.
		}
		else if (isBack())
			return back_reg_->isReach();
	}

	return false;
}

bool SpotClean::isExit()
{
	if(CleanMode::isExit())
		return true;

	return false;
}

bool SpotClean::isStop()
{
	if(CleanMode::isStop())
		return true;

	else if (cs.is_clean())
	{
		if (mt.is_linear()) {
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
								|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}
	return false;
}

bool SpotClean::isSwitch()
{
	auto old_p_reg_ = p_reg_;

	if (cs.is_clean()) {
		if (mt.is_linear()) // Robot is cleaning current line.
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
	}

	if (old_p_reg_ != p_reg_)
	{
		ROS_INFO("%s %d: Switch from %s to %s.", __FUNCTION__, __LINE__, (old_p_reg_->getName()).c_str(), (p_reg_->getName()).c_str());
		setTarget();
		return true;
	}

	return false;
}

//WallFollowClean
WallFollowClean::WallFollowClean(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path) {
	g_plan_path.clear();
	s_curr_p = {cost_map.get_x_count(),cost_map.get_y_count()};
	auto target = cost_map.cell_to_point(target_cell);

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
			return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with laser.
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
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	if (cm_is_follow_wall()) {
		if (mt.is_linear()) // 1. Going straight to find the wall at the beginning of wall follow mode_.
			// 2. Passed path is a closure or passed path is isolate, need to go straight to another wall.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
								|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
		else if (mt.is_follow_wall()) // Robot is following wall for cleaning.
		{
			if (isMt())
				return fw_reg_->isIsolate();
			else if (isBack())
				return back_reg_->isLaserStop();
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

	CostMap::save_blocks();
//	uint8_t block_count = 0;
	if(mt.is_follow_wall()) {
		fw_map.set_obs();
		fw_map.set_bumper();
		fw_map.set_rcon();
		fw_map.set_cliff();
		fw_map.set_tilt();
		fw_map.set_slip();
		fw_map.set_laser();
		fw_map.set_follow_wall();
	}

//	cost_map.set_cleaned(g_passed_path);
//	cost_map.mark_robot(MAP);
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
//Exploration
Exploration::Exploration(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path) {
	g_plan_path.clear();
	s_curr_p = {cost_map.get_x_count(),cost_map.get_y_count()};
	auto target = cost_map.cell_to_point(target_cell);

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
				return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with laser.
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
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
		else if (mt.is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isOverOriginLine() || fw_reg_->isClosure(2);
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs.is_going_home())
	{
		if (mt.is_linear())
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs.is_clean())
	{
		{
			if (mt.is_linear())
			{
				if (isMt())
					return (line_reg_->isRconStop() || line_reg_->isOBSStop()
							|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
				else if (isBack())
					return back_reg_->isLaserStop();
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

