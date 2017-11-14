//
// Created by lsy563193 on 17-9-27.
//

#include <pp.h>

//CMMoveType g_cm_move_type;
static uint8_t clean_mode = 0;
static uint8_t clean_state = 0;

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

void CleanMode::mark() {
	map_set_blocks();
	map_set_cleaned(g_passed_path);
	map_mark_robot(MAP);
};

bool CleanMode::isStop(){
		return RegulatorBase::isStop();
};

bool CleanMode::isExit(){
	return RegulatorBase::isExit();
};

void CleanMode::setMt()
{
	s_origin_p = {map_get_x_count(), map_get_y_count()};
	s_target_p = map_cell_to_point(g_plan_path.front());
	if(mt_is_follow_wall())
	{
		ROS_INFO("%s,%d: mt_is_fw",__FUNCTION__, __LINE__);
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
					g_turn_angle = ranged_angle( course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - gyro_get_angle());
		}else{
//			ROS_INFO("%s,%d: mt_is_fw",__FUNCTION__, __LINE__);
			if (LASER_FOLLOW_WALL)
				if(!laser_turn_angle(g_turn_angle))
					g_turn_angle = ranged_angle( course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - gyro_get_angle());
		}

	}else if(mt_is_linear())
	{
		ROS_INFO("%s,%d: mt_is_linear",__FUNCTION__, __LINE__);
		mt_reg_ = line_reg_;
		g_turn_angle = ranged_angle(
					course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - gyro_get_angle());
	}
	else if (mt_is_go_to_charger())
	{
		mt_reg_ = gtc_reg_;
		g_turn_angle = 0;
	}
	p_reg_ = turn_reg_;
//	s_target_angle = g_turn_angle;
	s_target_angle = ranged_angle(gyro_get_angle() + g_turn_angle);
	ROS_INFO("%s,%d,curr(%d),set_target_angle(%d)",__FUNCTION__, __LINE__, gyro_get_angle(), s_target_angle);
	g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
	bumper_turn_factor = 0.85;
	g_bumper_cnt = g_cliff_cnt = 0;
	g_slip_cnt = 0;
	g_slip_backward = false;
	g_rcon_during_go_home = false;
	reset_rcon_status();
	robot::instance()->obsAdjustCount(20);
}

bool CleanMode::findTarget(Cell_t& curr) {
	return false;
}

bool is_equal_with_angle(const Cell_t &l, const Cell_t &r)
{
	return  l == r && std::abs(ranged_angle(l.TH - r.TH)) < 200;
}

Cell_t CleanMode::updatePosition(const Point32_t &curr_point)
{
		map_update_position();
		s_curr_p = curr_point;
		return map_get_curr_cell();
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
		map_save_blocks();
	}
//	else
//		is_time_up = !cs_is_trapped();
	return curr;
}

void CleanMode::display()
{
	path_display_path_points(g_plan_path);
	MotionManage::pubCleanMapMarkers(MAP, g_plan_path);
}

void CleanMode::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	if (p_reg_ != nullptr)
		p_reg_->adjustSpeed(left_speed, right_speed);
	left_speed_ = left_speed;
	right_speed_ = right_speed;
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
	printf("\n\033[42m======================================Generate path and update move type===========================================\033[0m\n");
	mark();
	auto cs_tmp = cs_get();
	if (!g_plan_path.empty())
		curr.TH = g_plan_path.back().TH;
	auto start = curr;
	g_old_dir = start.TH;
	if (g_is_near) {
		start = g_plan_path.back();
	}
	g_plan_path.clear();
	cs_path_next(start, g_plan_path);

	display();

	if (!((cs_tmp == CS_TRAPPED && cs_get() == CS_TRAPPED) || g_is_near)) {
		setMt();
		g_passed_path.clear();
	}
	g_is_near = false;

	printf("\033[44m====================================Generate path and update move type End=========================================\033[0m\n\n");
	return !g_plan_path.empty();
}

//NavigationClean
NavigationClean::NavigationClean(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path) {
	s_curr_p = {map_get_x_count(),map_get_y_count()};
	auto target = map_cell_to_point(target_cell);

	back_reg_ = new BackRegulator();
	fw_reg_ = new FollowWallRegulator(s_curr_p, target);
	line_reg_ = new LinearRegulator(target, path);
	gtc_reg_ = new GoToChargerRegulator();
	turn_reg_ = new TurnRegulator(0);
	mt_reg_ = line_reg_;

	if(cm_is_go_charger())
		mt_reg_ = gtc_reg_;
	p_reg_ = mt_reg_;

	path_next_nav(curr, g_plan_path);
	cm_set_event_manager_handler_state(true);

	ROS_INFO("%s, %d: NavigationClean finish", __FUNCTION__, __LINE__);
}

NavigationClean::~NavigationClean()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
	cm_set_event_manager_handler_state(false);
}

bool NavigationClean::isReach()
{
	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
		{
			if (isMt())
			{
				if(g_is_near)
					return true;
				return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with laser.
			}
			else if (isBack())
				return back_reg_->isReach();
		}
		else if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isBlockCleared();
		}
	}

	else if (cs_is_tmp_spot()) // For temp spot during navigation.
	{
		if (mt_is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching target.
			else if (isBack())
				return back_reg_->isReach();
		}
	}

	else if (cs_is_going_home())
	{
		if (mt_is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching target.
			else if (isBack())
				return back_reg_->isReach();
		}
	}

	{
		if (mt_is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isPoseReach();
			else if (isBack())
				return back_reg_->isReach();
		}
		else if (mt_is_follow_wall()) // Robot is going to new line.
		{
			if (isMt())
				return fw_reg_->isNewLineReach();
		}
	}

	return false;
}

bool NavigationClean::isExit()
{
	if(CleanMode::isExit())
		return true;

	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isTimeUp();
		}
	}

	return false;
}

bool NavigationClean::isStop()
{
	if(CleanMode::isStop())
		return true;

	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
		else if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isOverOriginLine() || fw_reg_->isClosure(2);
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs_is_tmp_spot()) // For temp spot during navigation.
	{
		if (mt_is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs_is_going_home())
	{
		if (mt_is_linear())
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs_is_clean())
	{
		if (cm_is_navigation())
		{
			if (mt_is_linear()) // Robot is cleaning current line.
			{
				if (isMt())
					return (line_reg_->isRconStop() || line_reg_->isOBSStop()
							|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
				else if (isBack())
					return back_reg_->isLaserStop();
			}
			else if (mt_is_follow_wall()) // Robot is going to new line.
			{
				if (isMt())
					return fw_reg_->isOverOriginLine() || fw_reg_->isClosure(1) || fw_reg_->isIsolate();
				else if (isBack())
					return back_reg_->isLaserStop();
			}
		}

		else if (cm_is_spot())
		{
			if (mt_is_linear())
			{
				if (isMt())
					return (line_reg_->isRconStop() || line_reg_->isOBSStop()
							|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
				else if (isBack())
					return back_reg_->isLaserStop();
			}
		}

		else if (cm_is_exploration())
		{
			if (mt_is_linear())
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

bool NavigationClean::isSwitch()
{
	auto old_p_reg_ = p_reg_;
	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
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
		else if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
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

	else if (cs_is_tmp_spot()) // For temp spot during navigation.
	{
		if (mt_is_linear()) // Robot is cleaning current line.
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

	else if (cs_is_going_home())
	{
		if (mt_is_linear()) // Robot is going straight with path.
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

	else if (cs_is_clean())
	{
		if (cm_is_navigation())
		{
			if (mt_is_linear()) // Robot is cleaning current line.
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
			else if (mt_is_follow_wall()) // Robot is going to new line.
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

		else if (cm_is_spot())
		{
			if (mt_is_linear()) // Robot is cleaning current line.
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

		else if (cm_is_exploration())
		{
			if (mt_is_linear()) // Robot is going straight to find charger.
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
	}

	if (old_p_reg_ != p_reg_)
	{
		ROS_INFO("%s %d: Switch from %s to %s.", __FUNCTION__, __LINE__, (old_p_reg_->getName()).c_str(), (p_reg_->getName()).c_str());
		setTarget();
		return true;
	}

	return false;
}

bool NavigationClean::findTarget(Cell_t& curr)
{
	return find_target(curr);
}

Cell_t NavigationClean::updatePosition(const Point32_t &curr_point)
{
	auto curr = CleanMode::updatePosition(curr_point);
	return updatePath(curr);
}
//SpotClean
SpotClean::SpotClean(const Cell_t& start_cell, const Cell_t& target_cell, const PPTargetType& path)
{
	s_curr_p = {map_get_x_count(),map_get_y_count()};
	auto target = map_cell_to_point(target_cell);

	back_reg_ = new BackRegulator();
	fw_reg_ = new FollowWallRegulator(s_curr_p, target);
	line_reg_ = new LinearRegulator(target, path);
	gtc_reg_ = new GoToChargerRegulator();
	turn_reg_ = new TurnRegulator(0);
	mt_reg_ = line_reg_;

	p_reg_ = mt_reg_;

	cm_set_event_manager_handler_state(true);

	ROS_INFO("%s, %d: SpotClean finish", __FUNCTION__, __LINE__);
}

SpotClean::~SpotClean()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
	cm_set_event_manager_handler_state(false);
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
	if (mt_is_linear()) // Robot is cleaning current line.
	{
		if (isMt())
		{
			if(g_is_near)
				return true;
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

	else if (cs_is_clean())
	{
		if (mt_is_linear()) {
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

	if (cs_is_clean()) {
		if (mt_is_linear()) // Robot is cleaning current line.
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
	s_curr_p = {map_get_x_count(),map_get_y_count()};
	auto target = map_cell_to_point(target_cell);

	back_reg_ = new BackRegulator();
	fw_reg_ = new FollowWallRegulator(s_curr_p, target);
	line_reg_ = new LinearRegulator(target, path);
	gtc_reg_ = new GoToChargerRegulator();
	turn_reg_ = new TurnRegulator(0);
	mt_reg_ = line_reg_;

	p_reg_ = mt_reg_;
	path_next_fw(curr);

	cm_set_event_manager_handler_state(true);

	ROS_INFO("%s, %d: WallFollowClean finish", __FUNCTION__, __LINE__);
}

WallFollowClean::~WallFollowClean()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
	cm_set_event_manager_handler_state(false);
}

bool WallFollowClean::isReach() {
	if (cs_is_going_home()) {
		if (mt_is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching target.
			else if (isBack())
				return back_reg_->isReach();
		}
	}

	if (mt_is_linear()) {
		if (isMt())
			return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with laser.
	}
	else if (mt_is_follow_wall()) {
		if (isMt())
			return fw_reg_->isClosure(1);
	}
	return false;
}

bool WallFollowClean::isExit()
{
	if(CleanMode::isExit())
		return true;
	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
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

	if (cs_is_going_home())
	{
		if (mt_is_linear())
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	if (cm_is_follow_wall()) {
		if (mt_is_linear()) // 1. Going straight to find the wall at the beginning of wall follow mode.
			// 2. Passed path is a closure or passed path is isolate, need to go straight to another wall.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
								|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
		else if (mt_is_follow_wall()) // Robot is following wall for cleaning.
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

	if (cs_is_going_home())
	{
		if (mt_is_linear()) // Robot is going straight with path.
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

	else if (cs_is_clean()) {
		if (mt_is_linear()) // 1. Going straight to find the wall at the beginning of wall follow mode.
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
		else if (mt_is_follow_wall()) // Robot is following wall for cleaning.
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

bool WallFollowClean::findTarget(Cell_t& curr)
{
	printf("\n\033[42m======================================WallFollowClean===========================================\033[0m\n");
	mark();
	g_plan_path.clear();
	if(!path_next_fw(curr))
		return false;
	display();
	setMt();
	g_passed_path.clear();
	g_is_near = false;
	printf("\033[44m====================================WallFollowClean=========================================\033[0m\n\n");
	return true;
}
Cell_t WallFollowClean::updatePosition(const Point32_t &curr_point)
{
	auto curr = CleanMode::updatePosition(curr_point);
	return updatePath(curr);
}
//Exploration
Exploration::Exploration(const Cell_t& start_cell, const Cell_t& target_cell, const PPTargetType& path) {
	s_curr_p = {map_get_x_count(),map_get_y_count()};
	auto target = map_cell_to_point(target_cell);

	back_reg_ = new BackRegulator();
	fw_reg_ = new FollowWallRegulator(s_curr_p, target);
	line_reg_ = new LinearRegulator(target, path);
	gtc_reg_ = new GoToChargerRegulator();
	turn_reg_ = new TurnRegulator(0);
	mt_reg_ = line_reg_;

	p_reg_ = mt_reg_;

	cm_set_event_manager_handler_state(true);

	ROS_INFO("%s, %d: Exploration finish", __FUNCTION__, __LINE__);
}

Exploration::~Exploration()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
	cm_set_event_manager_handler_state(false);
}

bool Exploration::isReach()
{
	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
		{
			if (isMt())
			{
				if(g_is_near)
					return true;
				return line_reg_->isCellReach(); // For reaching 8 meters limit or follow wall with laser.
			}
			else if (isBack())
				return back_reg_->isReach();
		}
		else if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isBlockCleared();
		}
	}

	else if (cs_is_going_home())
	{
		if (mt_is_linear()) // Robot is cleaning current line.
		{
			if (isMt())
				return line_reg_->isCellReach(); // For reaching target.
			else if (isBack())
				return back_reg_->isReach();
		}
	}

	else if (cm_is_exploration())
	{
		if (mt_is_linear()) // Robot is cleaning current line.
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

	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
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

	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
		else if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
		{
			if (isMt())
				return fw_reg_->isOverOriginLine() || fw_reg_->isClosure(2);
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs_is_going_home())
	{
		if (mt_is_linear())
		{
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
						|| line_reg_->isLaserStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLaserStop();
		}
	}

	else if (cs_is_clean())
	{
		{
			if (mt_is_linear())
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
	if (cs_is_trapped()) // For trapped status.
	{
		if (mt_is_linear()) // Escape path is a closure or escape path is isolate, need to go straight to another wall.
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
		else if (mt_is_follow_wall()) // Robot is following wall to escape from trapped.
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

	else if (cs_is_going_home())
	{
		if (mt_is_linear()) // Robot is going straight with path.
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

	else if (cs_is_clean())
	{
		{
			if (mt_is_linear()) // Robot is going straight to find charger.
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
	}

	if (old_p_reg_ != p_reg_)
	{
		ROS_INFO("%s %d: Switch from %s to %s.", __FUNCTION__, __LINE__, (old_p_reg_->getName()).c_str(), (p_reg_->getName()).c_str());
		setTarget();
		return true;
	}

	return false;
}

bool Exploration::findTarget(Cell_t& curr)
{
	find_target(curr);
}

