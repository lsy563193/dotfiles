//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <dev.h>
#include <robot.hpp>
#include <error.h>
#include <map.h>
#include <path_algorithm.h>
#include "mode.hpp"
#include "mathematics.h"
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
	clean_path_algorithm_->displayPointPath((passed_path_));

	if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		setCleaned(pointsGenerateCells(passed_path_));
		auto start = *passed_path_.begin();
		passed_path_.erase(std::remove_if(passed_path_.begin(),passed_path_.end(),[&start](Point_t& it){
			return it.toCell() == start.toCell();
		}),passed_path_.end());
		clean_path_algorithm_->displayPointPath(passed_path_);
//		ROS_ERROR("-------------------------------------------------------");
		clean_map_.setFollowWall(action_i_ == ac_follow_wall_left, passed_path_);
		clean_map_.markRobot(CLEAN_MAP);
	}
	else if (sp_state == state_clean)
	{
		setLinearCleaned();
		// Set home cell.
		if (ev.rcon_triggered)
			setHomePoint();
	}

	clean_map_.setBlocks();
//	clean_map_.print(CLEAN_MAP, getPosition().toCell().x, getPosition().toCell().y);

	passed_path_.clear();
	return false;
}

bool CleanModeNav::markRealTime()
{
//	while (ros::ok()) {
//		sleep(0.2);
//		wheel.stop();
		std::vector<Vector2<int>> markers;
		if (lidar.isScanCompensateReady())
			lidar.lidarMarker(markers, 0.23);
//		ROS_INFO("markers.size() = %d", markers.size());
		for (const auto& marker : markers) {
//			ROS_INFO("marker(%d, %d)", marker.x, marker.y);
			auto cell = getPosition().getRelative(marker.x * CELL_SIZE, marker.y * CELL_SIZE).toCell();
			clean_map_.setCell(CLEAN_MAP, cell.x, cell.y, BLOCKED_LIDAR);
		}
//	}
	return true;

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
		else if (ev.cliff_all_triggered)
		{
			ev.fatal_quit = true;
			ROS_WARN("%s %d: Exit for pause and robot lifted up.", __FUNCTION__, __LINE__);
			setNextMode(md_idle);
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

/*bool CleanModeNav::moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt)
{
	if (sp_state == state_folllow_wall)
		return p_mt->isBlockCleared(clean_map_, passed_path_);
	else
		return p_mt->isNewLineReach(clean_map_) || p_mt->isOverOriginLine(clean_map_);
}

bool CleanModeNav::moveTypeLinearIsFinish(MoveTypeLinear *p_mt)
{
	if (p_mt->isLinearForward())
		return p_mt->isRconStop() || ACleanMode::moveTypeLinearIsFinish(p_mt);
	else
		return ACleanMode::moveTypeLinearIsFinish(p_mt);
}*/

// Event handlers.
void CleanModeNav::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);
	wheel.stop();

	// Wait for key released.
	bool long_press = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed.", __FUNCTION__, __LINE__);
			beeper.beepForCommand(VALID);
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

	beeper.beepForCommand(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

void CleanModeNav::remoteDirectionLeft(bool state_now, bool state_last)
{
	if (sp_state == state_pause || sp_state == state_spot)
	{
		beeper.beepForCommand(VALID);
		ROS_INFO("%s %d: Remote right.", __FUNCTION__, __LINE__);
		ev.remote_direction_left = true;
	}
	/*else if (sp_state == state_clean)
	{
		//todo: Just for testing.
		beeper.beepForCommand(VALID);
		continue_point_ = getPosition();
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.x, continue_point_.y);
		ev.battery_home = true;
		go_home_for_low_battery_ = true;
	}
	*/
	else
		beeper.beepForCommand(INVALID);

	remote.reset();
}

void CleanModeNav::remoteDirectionRight(bool state_now, bool state_last)
{
	if (sp_state == state_pause || sp_state == state_spot)
	{
		beeper.beepForCommand(VALID);
		ROS_INFO("%s %d: Remote right.", __FUNCTION__, __LINE__);
		ev.remote_direction_right = true;
	}
	else
		beeper.beepForCommand(INVALID);

	remote.reset();
}

void CleanModeNav::remoteDirectionForward(bool state_now, bool state_last)
{
	if (sp_state == state_pause || sp_state == state_spot)
	{
		beeper.beepForCommand(VALID);
		ROS_INFO("%s %d: Remote forward.", __FUNCTION__, __LINE__);
		ev.remote_direction_forward = true;
	}
	else
		beeper.beepForCommand(INVALID);

	remote.reset();
}

void CleanModeNav::remoteWallFollow(bool state_now, bool state_last)
{
	if (sp_state == state_pause)
	{
		beeper.beepForCommand(VALID);
		ROS_INFO("%s %d: Remote follow wall.", __FUNCTION__, __LINE__);
		ev.remote_follow_wall = true;
	}
	else
		beeper.beepForCommand(INVALID);

	remote.reset();
}

void CleanModeNav::remoteSpot(bool state_now, bool state_last)
{
	if (sp_state == state_clean || sp_state == state_spot || sp_state == state_pause)
	{
		ev.remote_spot = true;
		beeper.beepForCommand(VALID);
	}
	else
		beeper.beepForCommand(INVALID);

	remote.reset();
}

void CleanModeNav::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	if(isStateClean() || isStateResumeLowBatteryCharge())
	{
		beeper.beepForCommand(VALID);
		vacuum.switchToNext();
	}
	else if (isStateGoHomePoint() || isStateGoToCharger())
	{
		beeper.beepForCommand(VALID);
		vacuum.switchToNext();
		vacuum.setTmpMode(Vac_Normal);
	}
	else
		beeper.beepForCommand(INVALID);
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
		if (isStateInit() && action_i_ == ac_back_form_charger)
		{
			if (sp_action_->isTimeUp())
			{
				if (cliff.getStatus() == BLOCK_ALL)
				{
					// If switch is not on, the cliff value should be around 0.
					ROS_WARN("%s %d: Switch is not on!.", __FUNCTION__, __LINE__);
					ev.charge_detect = charger.getChargeStatus();
					ev.fatal_quit = true;
					switch_is_off_ = true;
				}
				else
				{
					ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
					ev.charge_detect = charger.getChargeStatus();
					ev.fatal_quit = true;
				}
			}
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

// ------------------State init--------------------
bool CleanModeNav::isSwitchByEventInStateInit() {
	return checkEnterPause() || ACleanMode::isSwitchByEventInStateInit();
}

bool CleanModeNav::updateActionInStateInit() {
	if (action_i_ == ac_null)
		action_i_ = ac_open_gyro;
	else if (action_i_ == ac_open_gyro)
	{
		// If it is the starting of navigation mode, paused_odom_angle_ will be zero.
		odom.setRadianOffset(paused_odom_angle_);

		if (charger.isOnStub()){
			action_i_ = ac_back_form_charger;
			found_charger_ = true;
		}
		else{
			action_i_ = ac_open_lidar;
			vacuum.setLastMode();
			brush.normalOperate();
		}
	} else if (action_i_ == ac_back_form_charger)
	{
		if (!has_aligned_and_open_slam_)
			// Init odom position here.
			robot::instance()->initOdomPosition();

		action_i_ = ac_open_lidar;
		vacuum.setLastMode();
		brush.normalOperate();
		setHomePoint();
	} else if (action_i_ == ac_open_lidar)
	{
		if (!has_aligned_and_open_slam_)
			action_i_ = ac_align;
		else
			return false;
	} else if (action_i_ == ac_align){
		action_i_ = ac_open_slam;

	}
	else if (action_i_ == ac_open_slam){
		//after back_from_charger and line alignment
		//set charge position
		ACleanMode::checkShouldMarkCharger((float)odom.getRadianOffset(),0.6);
		return false;
	}
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
	return checkEnterPause() || checkEnterTempSpotState() || ACleanMode::isSwitchByEventInStateClean();
}

bool CleanModeNav::updateActionInStateClean(){
	sp_action_.reset();//to mark in destructor
//	pubCleanMapMarkers(clean_map_, pointsGenerateCells(remain_path_));
	old_dir_ = iterate_point_.dir;
	if(action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
		old_dir_ = MAP_ANY;
	if (clean_path_algorithm_->generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
		pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
		iterate_point_ = plan_path_.front();
//		plan_path_.pop_front();
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
		auto npa = boost::dynamic_pointer_cast<NavCleanPathAlgorithm>(clean_path_algorithm_);

		if (old_dir_ != MAP_ANY && clean_map_.isFrontBlocked(old_dir_)
				&& (npa->curr_filter_ == &npa->filter_p0_1t_xp
						 || npa->curr_filter_ == &npa->filter_p0_1t_xn
						 || npa->curr_filter_ == &npa->filter_p2
						 || npa->curr_filter_ == &npa->filter_p1
						 || npa->curr_filter_ == &npa->filter_n2
						 || npa->curr_filter_ == &npa->filter_n1)
				)
		{
			auto toward_pos = isXAxis(old_dir_) ? npa->curr_filter_->towardPos(): (iterate_point_.toCell().x - plan_path_.back().toCell().x) > 0;
			bool is_left = isPos(old_dir_) ^ toward_pos;
			action_i_ = is_left ? ac_follow_wall_left : ac_follow_wall_right;
		}
		else
			action_i_ = ac_linear;

		genNextAction();
		return true;
	}
	return false;
}

void CleanModeNav::switchInStateClean() {
	if (clean_path_algorithm_->checkTrapped(clean_map_, getPosition().toCell())) {
		ROS_WARN("%s,%d: enter state trapped",__FUNCTION__,__LINE__);
		sp_saved_states.push_back(sp_state);
		sp_state = state_folllow_wall;
		is_trapped_ = true;
		is_isolate = true;
		is_closed = true;
		closed_count_ = 0;
		isolate_count_ = 0;
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
	return checkEnterPause() || ACleanMode::isSwitchByEventInStateGoToCharger();
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
	if (ev.remote_spot || ev.remote_direction_forward || ev.remote_direction_left || ev.remote_direction_right)
	{
		sp_state = state_clean;
		sp_state->init();
		action_i_ = ac_null;
		sp_action_.reset();
		clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
		ev.remote_direction_forward = false;
		ev.remote_direction_left = false;
		ev.remote_direction_right = false;
		ev.remote_spot = false;

		return true;
	}

	return ACleanMode::checkEnterGoHomePointState() || ACleanMode::isSwitchByEventInStateSpot();
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
bool CleanModeNav::isSwitchByEventInStateFollowWall()
{
	return ACleanMode::isSwitchByEventInStateFollowWall() || checkEnterPause();
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
			ACleanMode::isSwitchByEventInStateResumeLowBatteryCharge();
}

bool CleanModeNav::updateActionInStateResumeLowBatteryCharge()
{
	if (getPosition().toCell() == continue_point_.toCell())
		return false;
	else {
//		clean_map_.saveBlocks(action_i_ == ac_linear, sp_state == state_clean);
//		mapMark();
		sp_action_.reset();
		old_dir_ = iterate_point_.dir;
		ROS_ERROR("old_dir_(%d)", old_dir_);
		clean_path_algorithm_->generateShortestPath(clean_map_, getPosition(), continue_point_, old_dir_, plan_path_);
		if (!plan_path_.empty()) {
			iterate_point_ = plan_path_.front();
			ROS_ERROR("start_point_.dir(%d)", iterate_point_.dir);
//			plan_path_.pop_front();
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

