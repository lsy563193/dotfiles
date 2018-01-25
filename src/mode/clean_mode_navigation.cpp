//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <dev.h>
#include <robot.hpp>
#include <error.h>
#include <map.h>
#include "mode.hpp"
//#define NAV_INFO() ROS_INFO("st(%d),ac(%d)", state_i_, action_i_)

CleanModeNav::CleanModeNav()
{
	setNavMode(true);
	ROS_INFO("%s %d: Entering Navigation mode\n=========================" , __FUNCTION__, __LINE__);

	if(plan_activation_)
	{
		plan_activation_ = false;
		speaker.play(VOICE_APPOINTMENT_START, false);
	}
	else
		speaker.play(VOICE_CLEANING_START, false);

	has_aligned_and_open_slam_ = false;
	paused_odom_angle_ = 0;
	moved_during_pause_ = false;

	IMoveType::sp_mode_ = this;
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());
	go_home_path_algorithm_.reset();
}

CleanModeNav::~CleanModeNav()
{
	setNavMode(false);
}

bool CleanModeNav::mapMark()
{
	ROS_INFO("%s %d: Start updating map.", __FUNCTION__, __LINE__);
	clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
	clean_map_.setCleaned(pointsGenerateCells(passed_path_));

	if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
//		ROS_ERROR("-------------------------------------------------------");
		auto start = *passed_path_.begin();
		passed_path_.erase(std::remove_if(passed_path_.begin(),passed_path_.end(),[&start](Point_t& it){
			return it.toCell() == start.toCell();
		}),passed_path_.end());
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
//		ROS_ERROR("-------------------------------------------------------");
		clean_map_.setFollowWall(action_i_ == ac_follow_wall_left, passed_path_);
	}
	if (sp_state == state_trapped)
		fw_map.setFollowWall(action_i_ == ac_follow_wall_left,plan_path_);
	else if (sp_state == state_clean)
	{
		// Set home cell.
		if (ev.rcon_triggered)
			setHomePoint();
	}

	clean_map_.setBlocks();
	clean_map_.markRobot(CLEAN_MAP);
	clean_map_.print(CLEAN_MAP, getPosition().toCell().x, getPosition().toCell().y);

	passed_path_.clear();
	return false;
}

bool CleanModeNav::isExit()
{
	if (sp_state == state_init)
	{
		if (action_i_ == ac_open_lidar && sp_action_->isTimeUp())
		{
			error.set(ERROR_CODE_LIDAR);
			setNextMode(md_idle);
			ev.fatal_quit = true;
			return true;
		}
	}

	if (sp_state == state_pause)
	{
		if (sp_action_->isTimeUp())
		{
			ROS_WARN("%s %d: Exit for pause timeout(%d)", __FUNCTION__, __LINE__, IDLE_TIMEOUT);
			setNextMode(md_sleep);
			return true;
		}
		else if (sp_action_->isExit())
		{
			ROS_WARN("%s %d: Action pause exit.", __FUNCTION__, __LINE__);
			moved_during_pause_ = true;
			setNextMode(md_idle);
			return true;
		}
		else if (charger.getChargeStatus())
		{
			ROS_WARN("%s %d: Exit for pause and detect charge.", __FUNCTION__, __LINE__);
			setNextMode(md_charge);
			return true;
		}
		else if (ev.remote_direction_left || ev.remote_direction_right || ev.remote_direction_forward)
		{
			ROS_WARN("%s %d: Exit for pause and remote left/right/forward.", __FUNCTION__, __LINE__);
			setNextMode(md_remote);
			return true;
		}
		else if (ev.remote_follow_wall)
		{
			ROS_WARN("%s %d: Exit for pause and remote follow wall.", __FUNCTION__, __LINE__);
			setNextMode(cm_wall_follow);
			return true;
		}
		else if (ev.remote_spot)
		{
			ROS_WARN("%s %d: Exit for pause and remote spot.", __FUNCTION__, __LINE__);
			setNextMode(cm_spot);
			return true;
		}
	}

	if (ev.fatal_quit || ev.key_long_pressed || sp_action_->isExit())
	{
		ROS_WARN("%s %d: Exit for ev.fatal_quit || ev.key_long_pressed || sp_action_->isExit().", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (charger.isDirected())
	{
		ROS_WARN("%s %d: Exit for directly charge.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}
	return false;
}

void CleanModeNav::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	wheel.stop();

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

void CleanModeNav::overCurrentWheelLeft(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_left = true;
}

void CleanModeNav::overCurrentWheelRight(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_right = true;
}

void CleanModeNav::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

void CleanModeNav::remoteDirectionLeft(bool state_now, bool state_last)
{
	if (sp_state == state_pause || sp_state == state_spot)
	{
		beeper.play_for_command(VALID);
		ROS_INFO("%s %d: Remote right.", __FUNCTION__, __LINE__);
		ev.remote_direction_left = true;
	}
	/*else if (sp_state == state_clean)
	{
		//todo: Just for testing.
		beeper.play_for_command(VALID);
		continue_point_ = getPosition();
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.x, continue_point_.y);
		ev.battery_home = true;
		go_home_for_low_battery_ = true;
	}
	*/
	else
		beeper.play_for_command(INVALID);

	remote.reset();
}

void CleanModeNav::remoteDirectionRight(bool state_now, bool state_last)
{
	if (sp_state == state_pause || sp_state == state_spot)
	{
		beeper.play_for_command(VALID);
		ROS_INFO("%s %d: Remote right.", __FUNCTION__, __LINE__);
		ev.remote_direction_right = true;
	}
	else
		beeper.play_for_command(INVALID);

	remote.reset();
}

void CleanModeNav::remoteDirectionForward(bool state_now, bool state_last)
{
	if (sp_state == state_pause || sp_state == state_spot)
	{
		beeper.play_for_command(VALID);
		ROS_INFO("%s %d: Remote forward.", __FUNCTION__, __LINE__);
		ev.remote_direction_forward = true;
	}
	else
		beeper.play_for_command(INVALID);

	remote.reset();
}

void CleanModeNav::remoteWallFollow(bool state_now, bool state_last)
{
	if (sp_state == state_pause)
	{
		beeper.play_for_command(VALID);
		ROS_INFO("%s %d: Remote follow wall.", __FUNCTION__, __LINE__);
		ev.remote_follow_wall = true;
	}
	else
		beeper.play_for_command(INVALID);

	remote.reset();
}

void CleanModeNav::remoteSpot(bool state_now, bool state_last)
{
	if (sp_state == state_clean || sp_state == state_spot || sp_state == state_pause)
	{
		ev.remote_spot = true;
		beeper.play_for_command(VALID);
	}
	else
		beeper.play_for_command(INVALID);

	remote.reset();
}

void CleanModeNav::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	if(isStateClean() || isStateResumeLowBatteryCharge())
	{
		beeper.play_for_command(VALID);
		vacuum.switchToNext();
	}
	else if (isStateGoHomePoint() || isStateGoToCharger())
	{
		beeper.play_for_command(VALID);
		vacuum.switchToNext();
		vacuum.setTmpMode(Vac_Normal);
	}
	else
		beeper.play_for_command(INVALID);
	remote.reset();
}

void CleanModeNav::batteryHome(bool state_now, bool state_last)
{
	if (sp_state == state_clean)
	{
		continue_point_ = getPosition();
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.x, continue_point_.y);
		ev.battery_home = true;
		go_home_for_low_battery_ = true;
	}
}

void CleanModeNav::chargeDetect(bool state_now, bool state_last)
{
	if (!ev.charge_detect)
	{
		if (isStateInit() && action_i_ == ac_back_form_charger && sp_action_->isTimeUp())
		{
			ROS_WARN("%s %d: Switch is not on!.", __FUNCTION__, __LINE__);
			ev.charge_detect = charger.getChargeStatus();
			ev.fatal_quit = true;
			switch_is_off_ = true;
		}
		else if (charger.isDirected())
		{
			ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
			ev.charge_detect = charger.getChargeStatus();
			ev.fatal_quit = true;
		}
	}
}

// End event handlers.

bool CleanModeNav::moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt)
{
	if (sp_state == state_trapped)
		return p_mt->isBlockCleared(clean_map_, passed_path_);
	else
		return p_mt->isNewLineReach(clean_map_) || p_mt->isOverOriginLine(clean_map_);
	return false;
}

bool CleanModeNav::moveTypeLinearIsFinish(MoveTypeLinear *p_mt)
{
	if (p_mt->isLinearForward())
		return p_mt->isRconStop() || ACleanMode::moveTypeLinearIsFinish(p_mt);
	else
		return ACleanMode::moveTypeLinearIsFinish(p_mt);
}
// ------------------State init--------------------
bool CleanModeNav::isSwitchByEventInStateInit() {
	return checkEnterPause() || checkEnterExceptionResumeState();
}

bool CleanModeNav::updateActionInStateInit() {
	if (action_i_ == ac_null)
		action_i_ = ac_open_gyro;
	else if (action_i_ == ac_open_gyro)
	{
		// If it is the starting of navigation mode, paused_odom_angle_ will be zero.
		odom.setRadianOffset(paused_odom_angle_);

		vacuum.setLastMode();
		brush.normalOperate();

		if (charger.isOnStub())
			action_i_ = ac_back_form_charger;
		else
			action_i_ = ac_open_lidar;
	} else if (action_i_ == ac_back_form_charger)
	{
		if (!has_aligned_and_open_slam_)
			// Init odom position here.
			robot::instance()->initOdomPosition();
		action_i_ = ac_open_lidar;
		setHomePoint();
	} else if (action_i_ == ac_open_lidar)
	{
		if (!has_aligned_and_open_slam_)
			action_i_ = ac_align;
		else
			return false;
	} else if (action_i_ == ac_align)
		action_i_ = ac_open_slam;
	else if (action_i_ == ac_open_slam)
		return false;
	genNextAction();
	return true;
}

void CleanModeNav::switchInStateInit() {
	if (action_i_ == ac_open_lidar) {
		if (low_battery_charge_) {
			low_battery_charge_ = false;
			sp_state = state_resume_low_battery_charge;
		}
		else{ // Resume from pause, because slam is not opened for the first time that open lidar action finished.
			sp_state = sp_saved_states.back();
			sp_saved_states.pop_back();
		}
	}
	else {//if (action_i_ == ac_open_slam)
		has_aligned_and_open_slam_ = true;

		auto curr = getPosition();
		passed_path_.push_back(curr);
		start_point_.th = curr.th;
		sp_state = state_clean;
	}
	sp_state->init();
	action_i_ = ac_null;
	sp_action_.reset();
}

// ------------------State clean--------------------
bool CleanModeNav::isSwitchByEventInStateClean() {
	return checkEnterPause() ||
					checkEnterGoHomePointState() ||
					checkEnterExceptionResumeState() ||
					checkEnterTempSpotState();
}

bool CleanModeNav::updateActionInStateClean(){
	sp_action_.reset();//to mark in destructor
	pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
	old_dir_ = new_dir_;
	if (clean_path_algorithm_->generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {

//		ROS_ERROR("old_dir_(%d)", old_dir_);
		new_dir_ = plan_path_.front().th;
//		ROS_ERROR("new_dir_(%d)", new_dir_);
		plan_path_.pop_front();
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));

		auto start = getPosition().toCell();
		auto delta_y = plan_path_.back().toCell().y - start.y;
		ROS_INFO("%s,%d: path size(%u), old_dir_(%f), new_dir_(%f), bumper(%d), cliff(%d), lidar(%d), delta_y(%f)",
						 __FUNCTION__, __LINE__, plan_path_.size(), radian_to_degree(old_dir_), radian_to_degree(new_dir_), ev.bumper_triggered,
						 ev.cliff_triggered, ev.lidar_triggered, delta_y);
		if (!isXAxis(old_dir_) // If last movement is not x axis linear movement, should not follow wall.
				|| plan_path_.size() > 2 ||
				(!ev.bumper_triggered && !ev.cliff_triggered && !ev.lidar_triggered)
				|| delta_y == 0 || std::abs(delta_y) > 2) {
			action_i_ = ac_linear;
		}
		else
		{
			delta_y = plan_path_.back().toCell().y - start.y;
			bool is_left = isPos(old_dir_) ^ delta_y > 0;
//			ROS_INFO("%s,%d: target:, 1_left_0_right(%d = %d ^ %d)",
//							 __FUNCTION__, __LINE__, is_left, isPos(old_dir_), delta_y);
			action_i_ = is_left ? ac_follow_wall_left : ac_follow_wall_right;
		}
		genNextAction();
		return true;
	}
	return false;
}

void CleanModeNav::switchInStateClean() {
	if (clean_path_algorithm_->checkTrapped(clean_map_, getPosition().toCell())) {
		ROS_WARN("%s,%d: enter state trapped",__FUNCTION__,__LINE__);
		sp_saved_states.push_back(sp_state);
		sp_state = state_trapped;
	}
	else {
		sp_state = state_go_home_point;
		ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_points_.size());
		go_home_path_algorithm_.reset();
		go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
		speaker.play(VOICE_BACK_TO_CHARGER, true);
	}
	sp_state->init();
	action_i_ = ac_null;
	genNextAction();
}

// ------------------State go home point--------------------
bool CleanModeNav::checkEnterGoHomePointState()
{
	if (ev.battery_home)
		speaker.play(VOICE_BATTERY_LOW, false);

	return ACleanMode::checkEnterGoHomePointState();
}

bool CleanModeNav::isSwitchByEventInStateGoHomePoint()
{
	return checkEnterPause() || ACleanMode::isSwitchByEventInStateGoHomePoint();
}

// ------------------State go to charger--------------------
bool CleanModeNav::isSwitchByEventInStateGoToCharger()
{
	return checkEnterPause();
}

void CleanModeNav::switchInStateGoToCharger()
{
	if (charger.isOnStub())
	{
		if (go_home_for_low_battery_)
		{
			// If it is during low battery go home, it should not leave the clean mode, it should just charge.
			ROS_INFO("%s %d: Enter low battery charge.", __FUNCTION__, __LINE__);
			sp_state = state_charge;
			sp_state->init();
			paused_odom_angle_ = odom.getRadian();
			go_home_for_low_battery_ = false;
			go_home_path_algorithm_.reset();
		} else
		{
			// Reach charger and exit clean mode.
			sp_state = nullptr;
		}
		sp_action_.reset();
	}
	else
		ACleanMode::switchInStateGoToCharger();
}

// ------------------State tmp spot--------------------

bool CleanModeNav::checkEnterTempSpotState()
{
	if (ev.remote_spot)
	{
		ev.remote_spot= false;
//		mapMark();
		sp_action_.reset();
		clean_path_algorithm_.reset(new SpotCleanPathAlgorithm);
		sp_state = state_spot;
		sp_state->init();
		return true;
	}
	return false;
}

bool CleanModeNav::isSwitchByEventInStateSpot()
{
	if (ev.key_clean_pressed || ev.remote_spot || ev.remote_direction_forward || ev.remote_direction_left || ev.remote_direction_right)
	{
		if(sp_state == state_spot && ev.key_clean_pressed )
			sp_state = nullptr;
		else{
			sp_state = state_clean;
			sp_state->init();
			action_i_ = ac_null;
			sp_action_.reset();
			clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
		}
		ev.remote_direction_forward = false;
		ev.remote_direction_left = false;
		ev.remote_direction_right = false;
		ev.remote_spot = false;
		ev.key_clean_pressed = false;

		return true;
	}
	else if(ACleanMode::checkEnterGoHomePointState())
	{
		return true;
	}

	return false;
}

void CleanModeNav::switchInStateSpot()
{
	action_i_ = ac_null;
	sp_action_.reset();
	sp_state = state_clean;
	sp_state->init();
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
}

// ------------------State pause--------------------

bool CleanModeNav::checkEnterPause()
{
	if (ev.key_clean_pressed)
	{
		ev.key_clean_pressed = false;
		speaker.play(VOICE_CLEANING_PAUSE);
		ROS_INFO("%s %d: Key clean pressed, pause cleaning.", __FUNCTION__, __LINE__);
		paused_odom_angle_ = odom.getRadian();
		sp_action_.reset();
		sp_saved_states.push_back(sp_state);
		sp_state = state_pause;
		sp_state->init();
//		mapMark();
		return true;
	}

	return false;
}

bool CleanModeNav::checkResumePause()
{
	if (ev.key_clean_pressed || ev.remote_home)
	{
		ev.key_clean_pressed = false;
		speaker.play(VOICE_CLEANING_CONTINUE);
		sp_action_.reset();
		action_i_ = ac_null;
		ROS_INFO("%s %d: Resume cleaning.", __FUNCTION__, __LINE__);
		// It will NOT change the state.
		if (ev.remote_home && sp_saved_states.back() != state_go_home_point)
		{
			sp_saved_states.pop_back();
			sp_saved_states.push_back(state_go_home_point);
			if (go_home_path_algorithm_ == nullptr)
				go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
		}
		sp_state = state_init;
		sp_state->init();
	}
}

bool CleanModeNav::isSwitchByEventInStatePause()
{
	return checkResumePause();
}

bool CleanModeNav::updateActionInStatePause()
{
	action_i_ = ac_pause;
	genNextAction();
	return true;
}

// ------------------State trapped------------------
bool CleanModeNav::isSwitchByEventInStateTrapped()
{
	return ACleanMode::isSwitchByEventInStateTrapped() || checkEnterPause();
}

// ------------------State charge--------------------
bool CleanModeNav::isSwitchByEventInStateCharge()
{
	return checkEnterResumeLowBatteryCharge();
}

bool CleanModeNav::updateActionStateCharge()
{
	if (charger.isOnStub())
	{
		action_i_ = ac_charge;
		genNextAction();
		return true;
	}
	else
		return false;
}

void CleanModeNav::switchInStateCharge()
{
	// Resume from low battery charge.
	speaker.play(VOICE_CLEANING_CONTINUE, false);
	ROS_INFO("%s %d: Resume low battery charge.", __FUNCTION__, __LINE__);
	sp_action_.reset();
	action_i_ = ac_null;
	sp_state = state_init;
	sp_state->init();
	low_battery_charge_ = true;
}

// ------------------State resume low battery charge--------------------
bool CleanModeNav::checkEnterResumeLowBatteryCharge()
{
	if (ev.key_clean_pressed || battery.isReadyToResumeCleaning())
	{
		// For key clean force continue cleaning.
		if (ev.key_clean_pressed)
			ev.key_clean_pressed = false;

		// Resume from low battery charge.
		speaker.play(VOICE_CLEANING_CONTINUE, false);
		ROS_INFO("%s %d: Resume low battery charge.", __FUNCTION__, __LINE__);
		sp_action_.reset();
		action_i_ = ac_null;
		sp_state = state_init;
		sp_state->init();
		low_battery_charge_ = true;
		return true;
	}

	return false;
}

bool CleanModeNav::isSwitchByEventInStateResumeLowBatteryCharge()
{
	return checkEnterPause() ||
			checkEnterGoHomePointState() ||
			checkEnterExceptionResumeState();
}

bool CleanModeNav::updateActionInStateResumeLowBatteryCharge()
{
	if (getPosition().toCell() == continue_point_.toCell())
		return false;
	else {
//		clean_map_.saveBlocks(action_i_ == ac_linear, sp_state == state_clean);
//		mapMark();
		sp_action_.reset();
		old_dir_ = new_dir_;
		ROS_ERROR("old_dir_(%d)", old_dir_);
		clean_path_algorithm_->generateShortestPath(clean_map_, getPosition(), continue_point_, old_dir_, plan_path_);
		if (!plan_path_.empty()) {
			new_dir_ = plan_path_.front().th;
			ROS_ERROR("new_dir_(%d)", new_dir_);
			plan_path_.pop_front();
			clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
			action_i_ = ac_linear;
			genNextAction();
			return true;
		}
		else
			return false;
	}
}

void CleanModeNav::switchInStateResumeLowBatteryCharge()
{
	if (getPosition().toCell() == continue_point_.toCell())
		ROS_INFO("%s %d: Reach continue point(%d, %d).", __FUNCTION__, __LINE__, continue_point_.toCell().x, continue_point_.toCell().y);
	else
		ROS_INFO("%s %d: Fail to go to continue point(%d, %d).", __FUNCTION__, __LINE__, continue_point_.toCell().x, continue_point_.toCell().y);

	sp_state = state_clean;
	sp_state->init();
	sp_action_.reset();
}
// ------------------State Exception Resume--------------------
bool CleanModeNav::isSwitchByEventInStateExceptionResume() {
	return checkEnterPause();
}

