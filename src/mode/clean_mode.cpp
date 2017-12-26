//
// Created by lsy563193 on 17-12-3.
//

#include <mathematics.h>
#include <pp.h>
#include "arch.hpp"

#define NAV_INFO() ROS_INFO("st(%d),ac(%d)", state_i_, action_i_)

Path_t ACleanMode::passed_path_ = {};
Path_t ACleanMode::plan_path_ = {};
Cell_t ACleanMode::last_ = {};
//boost::shared_ptr<IMovement> ACleanMode::sp_movement_ = nullptr;

ACleanMode::ACleanMode()
{
	led.set_mode(LED_FLASH, LED_GREEN, 1000);
	ev.key_clean_pressed = false;
	sp_action_.reset(new ActionOpenGyro());
	action_i_ = ac_open_gyro;
	state_i_ = st_null;
	robot_timer.initWorkTimer();
	key.resetPressStatus();

	c_rcon.resetStatus();

	home_cells_.resize(1,g_zero_home);
	clean_path_algorithm_.reset();
	go_home_path_algorithm_.reset();
}

bool ACleanMode::isInitState()
{
	return action_i_ == ac_open_gyro || action_i_ == ac_back_form_charger || action_i_ == ac_open_lidar ||
				action_i_ == ac_align;
}

bool ACleanMode::setNextAction()
{
	if(action_i_ == ac_open_gyro)
	{
		if (charger.isOnStub())
			action_i_ = ac_back_form_charger;
		else
			action_i_ = ac_open_lidar;
	}
	else if(action_i_ == ac_back_form_charger)
		action_i_ = ac_open_lidar;
	else if(action_i_ == ac_open_lidar)
		action_i_ = ac_open_slam;
	else
	{
		if (isExceptionTriggered())
			action_i_ = ac_exception_resume;
		else if (ev.fatal_quit)
		{
			PP_INFO();
			ROS_ERROR("ev.fatal_quit");
			action_i_ = ac_null;
		}
	}
	genNextAction();
	PP_INFO(); NAV_INFO();
	return action_i_ != ac_null;
}

void ACleanMode::setNextMode(int next)
{
	if (ev.charge_detect && charger.isOnStub()) {
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		Mode::setNextMode(md_charge);
	}
	else {
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		Mode::setNextMode(next);
	}
}

bool ACleanMode::isFinish()
{
	if (isInitState()) {
		if (!sp_action_->isFinish())
			return false;
		setNextAction();
	}
	else
	{
		updatePath(*cleanMap_);

		if (!sp_action_->isFinish())
			return false;

		sp_action_.reset();//for call ~constitution;
		PP_INFO();
		mapMark();

		do
		{
			if (!setNextState())
			{
				setNextMode(0);
				return true;
			}
		} while (!setNextAction());
	}
	return false;
}

bool ACleanMode::setNextState()
{
	PP_INFO();
	bool state_confirm = false;
	while (ros::ok() && !state_confirm)
	{
		if (state_i_ == st_null)
		{
			auto curr = nav_map.updatePosition();
			passed_path_.push_back(curr);

			home_cells_.back().TH = robot::instance()->getPoseAngle();
			PP_INFO();

			state_i_ = st_clean;
			stateInit(state_i_);
			action_i_ = ac_null;
		}
		else if (isExceptionTriggered())
		{
			ROS_INFO("%s %d: Pass this state switching for exception cases.", __FUNCTION__, __LINE__);
			// Apply for all states.
			// If all these exception cases happens, directly set next action to exception resume action.
			// BUT DO NOT CHANGE THE STATE!!! Because after exception resume it should restore the state.
			action_i_ = ac_null;
			state_confirm = true;
		}
		else if(state_i_ == st_clean)
		{
			PP_INFO();
			old_dir_ = new_dir_;
			ROS_ERROR("old_dir_(%d)", old_dir_);
			if (clean_path_algorithm_->generatePath(nav_map, nav_map.getCurrCell(), old_dir_, plan_path_))
			{
				new_dir_ = (MapDirection)plan_path_.front().TH;
				ROS_ERROR("new_dir_(%d)", new_dir_);
				plan_path_.pop_front();
				clean_path_algorithm_->displayPath(plan_path_);
				state_confirm = true;
			}
			else
			{
				if (clean_path_algorithm_->checkTrapped(nav_map, nav_map.getCurrCell()))
					state_i_ = st_trapped;
				else
					state_i_ = st_go_home_point;
				stateInit(state_i_);
				action_i_ = ac_null;
			}
		}
		else if (state_i_ == st_trapped)
		{
			PP_INFO();
			if (robot_timer.trapTimeout(ESCAPE_TRAPPED_TIME))
			{
				ROS_WARN("%s %d: Escape trapped timeout!(%d)", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME);
				state_i_ = st_null;
				state_confirm = true;
			}
			else if (!clean_path_algorithm_->checkTrapped(nav_map, nav_map.getCurrCell()))
			{
				ROS_WARN("%s %d: Escape trapped !", __FUNCTION__, __LINE__);
				state_i_ = st_clean;
				stateInit(state_i_);
			}
			else
				// Still trapped.
				state_confirm = true;
		}
		else if (state_i_ == st_go_home_point)
		{
			PP_INFO();
			old_dir_ = new_dir_;
			plan_path_.clear();
			if (go_home_path_algorithm_->generatePath(nav_map, nav_map.getCurrCell(),old_dir_, plan_path_))
			{
				// Reach home cell or new path to home cell is generated.
				if (plan_path_.empty())
				{
					// Reach home cell.
					PP_INFO();
					if (nav_map.getCurrCell() == g_zero_home)
					{
						PP_INFO();
						state_i_ = st_null;
					}
					else
					{
						PP_INFO();
						state_i_ = st_go_to_charger;
						stateInit(state_i_);
					}
					action_i_ = ac_null;
				}
				else
				{
					new_dir_ = (MapDirection)plan_path_.front().TH;
					plan_path_.pop_front();
					go_home_path_algorithm_->displayPath(plan_path_);
				}
			}
			else
			{
				// No more paths to home cells.
				PP_INFO();
				state_i_ = st_null;
			}
			state_confirm = true;
		}
		else if (state_i_ == st_go_to_charger)
		{
			PP_INFO();
			if (ev.charge_detect && charger.isOnStub())
				state_i_ = st_null;
			else
				state_i_ = st_go_home_point;
		}
	}

	return state_i_ != st_null;
}

bool is_equal_with_angle_(const Cell_t &l, const Cell_t &r)
{
	return  l == r && std::abs(ranged_angle(l.TH - r.TH)) < 200;
}

Cell_t ACleanMode::updatePath(GridMap& map)
{
	auto curr = map.updatePosition();
//	auto point = map.getCurrPoint();
//	robot::instance()->pubCleanMapMarkers(nav_map, tmp_plan_path_);
//	PP_INFO();
//	ROS_INFO("point(%d,%d,%d)",point.X, point.Y,point.TH);
//	ROS_INFO("last(%d,%d,%d)",last_.X, last_.Y, last_.TH);
	if (passed_path_.empty())
	{
		passed_path_.push_back(curr);
//		ROS_INFO("curr(%d,%d,%d)",curr.X, curr.Y, curr.TH);
	}
	else if (!is_equal_with_angle_(curr, last_))
	{
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
		map.saveBlocks(action_i_ == ac_linear);
//		displayPath(passed_path_);
	}
	return curr;
}

void ACleanMode::genNextAction()
{

	PP_INFO();
	if(action_i_ == ac_open_gyro)
		sp_action_.reset(new ActionOpenGyro);
	else if(action_i_ == ac_back_form_charger)
		sp_action_.reset(new ActionBackFromCharger);
	else if(action_i_ == ac_open_lidar)
		sp_action_.reset(new ActionOpenLidar);
	else if(action_i_ == ac_align)
		sp_action_.reset(new ActionAlign);
	else if(action_i_ == ac_open_slam)
		sp_action_.reset(new ActionOpenSlam);
	else if (action_i_ == ac_pause)
		sp_action_.reset(new ActionPause);
	else if (action_i_ == ac_linear)
		sp_action_.reset(new ActionLinear);
	else if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
		sp_action_.reset(new ActionFollowWall(action_i_ == ac_follow_wall_left, state_i_ == st_trapped));
	else if (action_i_ == ac_go_to_charger)
		sp_action_.reset(new MoveTypeGoToCharger);
	else if (action_i_ == ac_exception_resume)
		sp_action_.reset(new MovementExceptionResume);
	else if (action_i_ == ac_check_bumper)
		sp_action_.reset(new ActionCheckBumper);
	else if (action_i_ == ac_bumper_hit_test)
		sp_action_.reset(new MoveTypeBumperHitTest);
	else if (action_i_ == ac_check_vacuum)
		sp_action_.reset(new ActionCheckVacuum);
	else if (action_i_ == ac_movement_direct_go)
		sp_action_.reset(new MovementDirectGo);
	else if(action_i_ == ac_null)
		sp_action_.reset();
	PP_INFO();
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

void ACleanMode::stateInit(int next)
{
	if (next == st_clean) {
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_GREEN);
		PP_INFO();
	}
	if (next == st_go_home_point)
	{
		vacuum.setMode(Vac_Normal, false);
		brush.setSidePwm(30, 30);
		brush.setMainPwm(30);
		wheel.stop();

		wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
		if (ev.remote_home)
			led.set_mode(LED_STEADY, LED_ORANGE);

		// Play wavs.
		if (ev.battrey_home)
			speaker.play(VOICE_BATTERY_LOW, true);

		speaker.play(VOICE_BACK_TO_CHARGER, true);

		ev.remote_home = false;
		ev.battrey_home = false;

		ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_cells_.size());

	}
	if (next == st_tmp_spot) {
//		if (SpotMovement::instance()->getSpotType() == NO_SPOT) {
//			ROS_INFO("%s %d: Entering temp spot during navigation.", __FUNCTION__, __LINE__);
//			Cell_t curr_cell = nav_map.getCurrCell();
//			ROS_WARN("%s %d: current cell(%d, %d).", __FUNCTION__, __LINE__, curr_cell.X, curr_cell.Y);
//			SpotMovement::instance()->setSpotType(CLEAN_SPOT);
//			wheel.stop();
//		}
//		else if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT) {
//			ROS_INFO("%s %d: Exiting temp spot.", __FUNCTION__, __LINE__);
//			SpotMovement::instance()->spotDeinit();
//			wheel.stop();
//			speaker.play(VOICE_CLEANING_CONTINUE);
//		}
//		ev.remote_spot = false;
	}
	if (next == st_trapped) {
		robot_timer.initTrapTimer();
		led.set_mode(LED_FLASH, LED_GREEN, 300);
	}
	if (next == st_exploration) {
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == st_go_to_charger) {
		gyro.TiltCheckingEnable(false); //disable tilt detect
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == st_self_check) {
		led.set_mode(LED_STEADY, LED_GREEN);
	}
}

uint8_t ACleanMode::saveFollowWall(bool is_left)
{
	auto dy = is_left ? 2 : -2;
	int16_t x, y;
	//int32_t	x2, y2;
	std::string msg = "cell:";
	GridMap::robotToCell(GridMap::getCurrPoint(), dy * CELL_SIZE, 0, x, y);
	//robot_to_point(robot::instance()->getPoseAngle(), dy * CELL_SIZE, 0, &x2, &y2);
	//ROS_WARN("%s %d: d_cell(0, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
	//			, __FUNCTION__, __LINE__, dy, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
//	bool should_save_for_MAP = !(cm_is_navigation() && mt.is_follow_wall() && Movement::getMoveDistance() < 0.1);
	temp_fw_cells.push_back({x, y});
	msg += "[0," + std::to_string(dy) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	//ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());

	return 1;
}

bool ACleanMode::MovementFollowWallisFinish()
{
	return false;
}




