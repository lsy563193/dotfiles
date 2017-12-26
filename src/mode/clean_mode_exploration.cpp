//
// Created by pierre on 17-12-20.
//
#include <event_manager.h>
#include <pp.h>
#include <error.h>
#include "arch.hpp"

CleanModeExploration::CleanModeExploration() {
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	event_manager_reset_status();
	PP_INFO();
	ROS_INFO("%s %d: Entering Exporation mode\n=========================" , __FUNCTION__, __LINE__);
	speaker.play(VOICE_EXPLORATION_START);
	action_i_ = ac_open_gyro;
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());
	IMoveType::sp_mode_.reset(this);
	cleanMap_ = &exploration_map;
}

CleanModeExploration::~CleanModeExploration() {
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanLinearReady(0);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);
	slam.stop();
	odom.setAngleOffset(0);

	if(ev.key_clean_pressed || ev.key_long_pressed) {
		speaker.play(VOICE_CLEANING_STOP);
		ROS_WARN("%s %d: Press remote clean.Stop Exploration", __FUNCTION__, __LINE__);
	}
	else if (ev.cliff_all_triggered)
	{
		speaker.play(VOICE_ERROR_LIFT_UP, false);
		speaker.play(VOICE_CLEANING_STOP);
		ROS_WARN("%s %d: Cliff all triggered. Stop Exploration.", __FUNCTION__, __LINE__);
	}
	else if (ev.fatal_quit)
	{
		speaker.play(VOICE_CLEANING_STOP, false);
		error.alarm();
		ROS_WARN("%s %d: fatal_quit. Stop Exploration.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}
}
bool CleanModeExploration::mapMark()
{
//	PP_WARN();
	exploration_map.mergeFromSlamGridMap(slam_grid_map,true,true);
	exploration_map.setExplorationCleaned();
	exploration_map.setBlocks();
	exploration_map.markRobot(CLEAN_MAP);
	robot::instance()->pubCleanMapMarkers(exploration_map, plan_path_);
	passed_path_.clear();
	return false;
}

bool CleanModeExploration::isFinish() {
	if (isInitState()) {
		if (!sp_action_->isFinish())
			return false;
		setNextAction();
	}
	else
	{
		updatePath(*cleanMap_);
		mapMark();

		if (!sp_action_->isFinish())
		{
			return false;
		}

		printMapAndPath();
		ROS_ERROR("%s,%d,next",__FUNCTION__,__LINE__);

		sp_action_.reset();//for call ~constitution;
		PP_INFO();
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

bool CleanModeExploration::isExit() {
	if(ev.cliff_all_triggered){
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}
	if(ev.key_clean_pressed || ev.key_long_pressed){
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}
	return false;
}

bool CleanModeExploration::setNextAction() {
	PP_INFO();
	//todo action convert
	if(state_i_ == st_clean)
		action_i_ = ac_linear;
	else if(state_i_ == st_go_to_charger)
		action_i_ = ac_go_to_charger;
	else if(state_i_ == st_go_home_point)
		action_i_ = ac_linear;
	else
		action_i_ = ac_null;
	genNextAction();
	return action_i_ != ac_null;
}

bool CleanModeExploration::setNextState() {
	PP_INFO();
	bool state_confirm = false;
	while (ros::ok() && !state_confirm)
	{
		if (state_i_ == st_null)
		{
			auto curr = exploration_map.updatePosition();
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
			ROS_WARN("old_dir_(%d)", old_dir_);
			plan_path_.clear();
			if (clean_path_algorithm_->generatePath(exploration_map,exploration_map.getCurrCell(), old_dir_, plan_path_))
			{
				new_dir_ = (MapDirection)plan_path_.front().TH;
				ROS_WARN("new_dir_(%d)", new_dir_);
				plan_path_.pop_front();
				clean_path_algorithm_->displayPath(plan_path_);
				state_confirm = true;
			}
			else
			{
				ROS_WARN("%s,%d:exploration finish,did not find charge",__func__,__LINE__);
				state_i_ = st_go_home_point;
				if (go_home_path_algorithm_ == nullptr)
					go_home_path_algorithm_.reset(new GoHomePathAlgorithm(exploration_map, home_cells_));
				stateInit(state_i_);
				action_i_ = ac_null;
			}
			if(c_rcon.isTrigT())
			{
				ROS_WARN("%s,%d:find charge success,convert to go to charge state",__func__,__LINE__);
				state_i_ = st_go_to_charger;
				stateInit(state_i_);
				state_confirm = true;
				action_i_ = ac_go_to_charger;
			}
		}
		else if(state_i_ == st_go_home_point)
		{
			PP_INFO();
			old_dir_ = new_dir_;
			plan_path_.clear();
			if (go_home_path_algorithm_->generatePath(exploration_map, exploration_map.getCurrCell(),old_dir_, plan_path_))
			{
				// Reach home cell or new path to home cell is generated.
				if (plan_path_.empty())
				{
					// Reach home cell.
					ROS_WARN("Reach home point(0,0)");
					state_i_ = st_null;
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
				ROS_WARN("can't go to home point(0,0)");
				state_i_ = st_null;
			}
			state_confirm = true;
		}
		else if (state_i_ == st_go_to_charger)
		{
			PP_INFO();
			if (ev.charge_detect && charger.isOnStub())
			{
				state_i_ = st_null;
				state_confirm = true;
			}
			else
				state_i_ = st_clean;
		}
	}
	return state_i_ != st_null;
}

void CleanModeExploration::keyClean(bool state_now, bool state_last) {
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);

	// Wait for key released.
	bool long_press = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed.", __FUNCTION__, __LINE__);
			beeper.play_for_command(VALID);
			long_press = true;
		}
		usleep(20000);
	}

	if (long_press)
		ev.key_long_pressed = true;
	else
		ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void CleanModeExploration::remoteClean(bool state_now, bool state_last) {
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	ev.key_clean_pressed = true;
	remote.reset();
}

void CleanModeExploration::cliffAll(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);
	ev.cliff_all_triggered = true;
}

void CleanModeExploration::overCurrentWheelLeft(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_left = true;
}

void CleanModeExploration::overCurrentWheelRight(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_right = true;
}

void CleanModeExploration::chargeDetect(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
	if (charger.getChargeStatus() >= 1)
	{
		ROS_WARN("%s %d: Set ev.chargeDetect.", __FUNCTION__, __LINE__);
		ev.charge_detect = charger.getChargeStatus();
	}
}

void CleanModeExploration::printMapAndPath()
{
	clean_path_algorithm_->displayPath(passed_path_);
	exploration_map.print(CLEAN_MAP,exploration_map.getXCell(),exploration_map.getYCell());
}

void CleanModeExploration::stateInit(int next) {
if (next == st_clean) {
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_GREEN);
		PP_INFO();
	}
	if (next == st_go_home_point)
	{
		vacuum.setMode(Vac_Normal, false);
		wheel.stop();

		wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
		if (ev.remote_home)
			led.set_mode(LED_STEADY, LED_ORANGE);

		// Play wavs.
		if (ev.battery_home)
			speaker.play(VOICE_BATTERY_LOW, true);

		speaker.play(VOICE_BACK_TO_CHARGER, true);

		ev.remote_home = false;
		ev.battery_home = false;

		if (go_home_path_algorithm_ == nullptr)
			go_home_path_algorithm_.reset(new GoHomePathAlgorithm(exploration_map, home_cells_));
		ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_cells_.size());
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
