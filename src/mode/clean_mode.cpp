//
// Created by lsy563193 on 17-12-3.
//

#include "pp.h"
#include "arch.hpp"

//boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;
//boost::shared_ptr<IMoveType> ACleanMode::sp_move_type_ = nullptr;

Path_t ACleanMode::passed_path_ = {};
Path_t ACleanMode::plan_path_ = {};
Cell_t ACleanMode::last_ = {};
//boost::shared_ptr<IMovement> ACleanMode::sp_movement_ = nullptr;

ACleanMode::ACleanMode() {
	g_homes.resize(1,g_zero_home);
}

//bool ACleanMode::isFinish() {
//	PP_INFO();
//	if(sp_state_ == nullptr)
//	{
//		getNextState();
//		getNextMoveType(nav_map.getCurrCell(),g_old_dir);
//	}
//
//	if(sp_state_->isFinish(this,sp_move_type_.get(),sp_action_.get() ,action_i_))
//	{
//		PP_INFO();
//		getNextState();
//		PP_INFO();
//		if(sp_state_ == nullptr)
//			return true;
//	}
//	PP_INFO();
//	return false;
//}
bool is_equal_with_angle_(const Cell_t &l, const Cell_t &r)
{
	return  l == r && std::abs(ranged_angle(l.TH - r.TH)) < 200;
}

Cell_t ACleanMode::updatePath()
{
//	PP_INFO()
	auto curr = nav_map.updatePosition();
	auto point = nav_map.getCurrPoint();
//	ROS_INFO("point(%d,%d,%d)",point.X, point.Y, curr.TH);
//	ROS_INFO("curr(%d,%d,%d)",curr.X, curr.Y, curr.TH);
//	ROS_INFO("last(%d,%d,%d)",last_.X, last_.Y, last_.TH);
	if (!is_equal_with_angle_(curr, last_)) {
//		PP_INFO()
		last_ = curr;
		auto loc = std::find_if(passed_path_.begin(), passed_path_.end(), [&](Cell_t it) {
				return is_equal_with_angle_(curr, it);
		});
		auto distance = std::distance(loc, passed_path_.end());
		if (distance == 0) {
			ROS_INFO("curr(%d,%d,%d)",curr.X, curr.Y, curr.TH);
			passed_path_.push_back(curr);
		}
		if (distance > 5) {
			passed_path_.clear();
			g_wf_reach_count++;
		}
		nav_map.saveBlocks();
//		displayPath(passed_path_);
	}
//	else
//		is_time_up = !cs.is_trapped();
	return curr;
}

void ACleanMode::genMoveAction() {
	if (action_i_ == ac_forward)
		sp_action_.reset(new MovementForward(GridMap::cellToPoint(plan_path_.back()), plan_path_));
	else if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
			sp_action_.reset(new MovementFollowWall(GridMap::getCurrPoint(), GridMap::cellToPoint(plan_path_.back()),action_i_ == ac_follow_wall_left));
	else if (action_i_ == ac_back)
		sp_action_.reset(new MovementBack());
	else if (action_i_ == ac_turn) {
		sp_action_.reset(new MovementTurn(plan_path_.back().TH));
	}
}

void ACleanMode::resetTriggeredValue(void)
{
	ev.lidar_triggered = 0;
	ev.rcon_triggered = 0;
	ev.bumper_triggered = 0;
	ev.obs_triggered = 0;
	ev.cliff_triggered = 0;
	ev.tilt_triggered = 0;
}

void ACleanMode::st_init(int next) {
	if (next == st_clean) {
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_GREEN);
	}
	if (next == st_go_home_point) {
		cs_work_motor();
		wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
		if (ev.remote_home || cm_is_go_charger())
			led.set_mode(LED_STEADY, LED_ORANGE);

		// Special handling for wall follow mode_.
		if (cm_is_follow_wall()) {
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle); //For wall follow mode_.
			nav_map.updatePosition();
			//wf_mark_home_point();
			nav_map.reset(CLEAN_MAP);
			nav_map.mergeFromSlamGridMap(slam_grid_map, false, false, true, false, false);
			nav_map.markRobot(CLEAN_MAP);//note: To clear the obstacles before go home, please don't remove it!
		}
		// Play wavs.
		if (ev.battrey_home)
			speaker.play(SPEAKER_BATTERY_LOW);
		if (!cm_is_go_charger())
			speaker.play(SPEAKER_BACK_TO_CHARGER);

		if (ev.remote_home)
			g_go_home_by_remote = true;
		ev.remote_home = false;
		ev.battrey_home = false;
	}
	if (next == st_tmp_spot) {
		if (SpotMovement::instance()->getSpotType() == NO_SPOT) {
			ROS_INFO("%s %d: Entering temp spot during navigation.", __FUNCTION__, __LINE__);
			Cell_t curr_cell = nav_map.getCurrCell();
			ROS_WARN("%s %d: current cell(%d, %d).", __FUNCTION__, __LINE__, curr_cell.X, curr_cell.Y);
			SpotMovement::instance()->setSpotType(CLEAN_SPOT);
			wheel.stop();
		}
		else if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT) {
			ROS_INFO("%s %d: Exiting temp spot.", __FUNCTION__, __LINE__);
			SpotMovement::instance()->spotDeinit();
			wheel.stop();
			speaker.play(SPEAKER_CLEANING_CONTINUE);
		}
		ev.remote_spot = false;
	}
	if (next == st_trapped) {
		g_wf_start_timer = time(NULL);
		g_wf_diff_timer = ESCAPE_TRAPPED_TIME;
		led.set_mode(LED_FLASH, LED_GREEN, 300);
	}
	if (next == st_exploration) {
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == st_go_charger) {
		gyro.TiltCheckingEnable(false); //disable tilt detect
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == st_self_check) {
		led.set_mode(LED_STEADY, LED_GREEN);
	}
}

void ACleanMode::mt_init(int) {
	auto s_curr_p = GridMap::getCurrPoint();
	auto s_target_p = nav_map.cellToPoint(plan_path_.back());
	if (move_type_i_ == mt_follow_wall_left || move_type_i_ == mt_follow_wall_right) {
//			ROS_INFO("%s,%d: mt.is_fw",__FUNCTION__, __LINE__);
			if (LIDAR_FOLLOW_WALL)
				if (!lidar_turn_angle(g_turn_angle))
					g_turn_angle = ranged_angle(course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) -
																			robot::instance()->getPoseAngle());
		ROS_INFO("%s,%d: mt.is_follow_wall, s_target_p(%d, %d).", __FUNCTION__, __LINE__, s_target_p.X, s_target_p.Y);

	}
	else if (move_type_i_ == mt_linear) {
		ROS_INFO("%s,%d: mt.is_linear", __FUNCTION__, __LINE__);
		s_target_p = nav_map.cellToPoint(g_plan_path.front());
		g_turn_angle = ranged_angle(
						course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - robot::instance()->getPoseAngle());
	}
	else if (move_type_i_ == mt_go_charger) {
		g_turn_angle = 0;
	}
//	s_target_angle = g_turn_angle;
	s_target_angle = ranged_angle(robot::instance()->getPoseAngle() + g_turn_angle);
	resetTriggeredValue();
	g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
	bumper_turn_factor = 0.85;
	g_bumper_cnt = g_cliff_cnt = 0;
	g_slip_cnt = 0;
	g_slip_backward = false;
	c_rcon.resetStatus();
	robot::instance()->obsAdjustCount(20);
}

//bool ACleanMode::isFinish() {
//	return false;
//}

