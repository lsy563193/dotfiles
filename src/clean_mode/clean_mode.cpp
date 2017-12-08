//
// Created by lsy563193 on 17-9-27.
//

#include <pp.h>

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

		auto curr = updatePosition({nav_map.getXCount(), nav_map.getYCount()});

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
			nav_map.saveBlocks();
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
	s_origin_p = {nav_map.getXCount(), nav_map.getYCount()};
	if(mt.is_follow_wall())
	{
		s_target_p = nav_map.cellToPoint(g_plan_path.back());
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
			if (LIDAR_FOLLOW_WALL)
				if(!lidar_turn_angle(g_turn_angle))
					g_turn_angle = ranged_angle( course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - robot::instance()->getPoseAngle());
		}else{
//			ROS_INFO("%s,%d: mt.is_fw",__FUNCTION__, __LINE__);
			if (LIDAR_FOLLOW_WALL)
				if(!lidar_turn_angle(g_turn_angle))
					g_turn_angle = ranged_angle( course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - robot::instance()->getPoseAngle());
		}
		ROS_INFO("%s,%d: mt.is_follow_wall, s_target_p(%d, %d).",__FUNCTION__, __LINE__, s_target_p.X, s_target_p.Y);

	}else if(mt.is_linear())
	{
		ROS_INFO("%s,%d: mt.is_linear",__FUNCTION__, __LINE__);
		s_target_p = nav_map.cellToPoint(g_plan_path.front());
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
//	turn_target_angle_ = g_turn_angle;
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
	nav_map.updatePosition();
		s_curr_p = curr_point;
		return nav_map.getCurrCell();
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
		nav_map.saveBlocks();
	}
//	else
//		is_time_up = !cs.is_trapped();
	return curr;
}

void CleanMode::display()
{
	path_display_path_points(g_plan_path);
	robot::instance()->pubCleanMapMarkers(nav_map, g_plan_path);
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
	ev.lidar_triggered = 0;
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
//		passed_path_.clear();
//	}
//	g_allow_check_path_in_advance = true;
//
//	printf("\033[44m====================================Generate path and update move type End=========================================\033[0m\n\n");
//	return !g_plan_path.empty();
}

