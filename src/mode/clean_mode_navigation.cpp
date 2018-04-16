#include <mode.hpp>//
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
#include "wifi/wifi.h"

//#define NAV_INFO() ROS_INFO("st(%d),ac(%d)", state_i_, action_i_)

int CleanModeNav::align_count_ = 0;
CleanModeNav::CleanModeNav()
{
	setNavMode(true);
	ROS_INFO("%s %d: Entering Navigation mode\n=========================" , __FUNCTION__, __LINE__);

	if(plan_activation_)
	{
		plan_activation_ = false;
//		speaker.play(VOICE_APPOINTMENT_START_UNOFFICIAL, false);
	}
//	else
	speaker.play(VOICE_CLEANING_START, false);

	has_aligned_and_open_slam_ = false;
	paused_odom_radian_ = 0;
	moved_during_pause_ = false;

	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());

	go_home_path_algorithm_.reset();
	mode_i_ = cm_navigation;

	sp_state = state_init.get();
	sp_state->init();
	//clear real time map whitch store in cloud....
	s_wifi.setWorkMode(Mode::cm_navigation);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_CLEAR_MAP);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
}

CleanModeNav::~CleanModeNav()
{
	setNavMode(false);
	s_wifi.clearMapCache();
}

bool CleanModeNav::mapMark()
{

	ROS_INFO("%s %d: Start updating map.", __FUNCTION__, __LINE__);
	if(passed_path_.empty())
	{
		ROS_WARN("%s %d: pass_path is emply, add curr_point(%d,%d,%d,%d).", __FUNCTION__, __LINE__,getPosition().x, getPosition().y, getPosition().th, getPosition().dir);
		passed_path_.push_back(getPosition());
	}

	clean_path_algorithm_->displayPointPath((passed_path_));

	GridMap map{};
	for (auto &&p_it :passed_path_)
		map.setCells(CLEAN_MAP, p_it.toCell().x, p_it.toCell().y, CLEANED);

	auto is_cleaned_bound = [&](const Cell_t &c_it){
		if(map.getCell(CLEAN_MAP, c_it.x, c_it.y) == CLEANED)
		{
			for (auto index = 0; index < 4; index++) {
				auto neighbor = c_it + cell_direction_[index];
				if (map.getCell(CLEAN_MAP, neighbor.x, neighbor.y) != CLEANED)
					return true;
			}
			return false;
		}
		return false;
	};
	auto is_cleaned_bound2 = [&](const Cell_t &c_it){
		if(map.getCell(CLEAN_MAP, c_it.x, c_it.y) == UNCLEAN)
		{
			for (auto index = 0; index < 4; index++) {
				auto neighbor = c_it + cell_direction_[index];
				if (map.getCell(CLEAN_MAP, neighbor.x, neighbor.y) == CLEANED)
					return true;
			}
			return false;
		}
		return false;
	};
	Cells c_bound1;
	Cells c_bound2;
	const auto start = passed_path_.front().toCell();
	const auto curr = passed_path_.back().toCell();
	map.find_if(start, c_bound1,is_cleaned_bound);
	map.find_if(start, c_bound2,is_cleaned_bound2);

	if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right) {
		if (!c_blocks.empty()) {
			auto dy = action_i_ == ac_follow_wall_left ? 2 : -2;
			std::for_each(passed_path_.begin(), passed_path_.end(),[&](const Point_t& point){
				BoundingBox<Point_t> bound;
				bound.SetMinimum({passed_path_.front().x - CELL_SIZE/4, passed_path_.front().y - CELL_SIZE/4});
				bound.SetMaximum({passed_path_.front().x + CELL_SIZE/4, passed_path_.front().y + CELL_SIZE/4});
				if(!bound.Contains(point))
				{
//					ROS_INFO("in cfw(%d,%d),(%d,%d)", point.toCell().x, point.toCell().y, getPosition().toCell().x, getPosition().toCell().y);
					ROS_WARN("Not Cont front(%d,%d),curr(%d,%d),point(%d,%d)", passed_path_.front().toCell().x, passed_path_.front().toCell().y,
									 getPosition().toCell().x, getPosition().toCell().y, point.toCell().x, point.toCell().y);
					c_blocks.insert({BLOCKED_FW, point.getCenterRelative(0, dy * CELL_SIZE).toCell()});
				}
				else {
					ROS_WARN("Contains front(%d,%d),curr(%d,%d),point(%d,%d)", passed_path_.front().toCell().x, passed_path_.front().toCell().y,
									 getPosition().toCell().x, getPosition().toCell().y, point.toCell().x, point.toCell().y);
				}
			});
		}
	}
	else if (sp_state == state_clean.get()) {
		setLinearCleaned();
		// Set home cell.
		if (ev.rcon_status)
			setHomePoint();
	}
	for (auto &&cost_block : c_blocks) {
		if(std::find_if(c_bound2.begin(), c_bound2.end(), [&](const Cell_t& c_it)
		{ return c_it == cost_block.second; }) != c_bound2.end())
//			if(!(cost_block.first == BLOCKED_LIDAR && (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)))
				clean_map_.setCell(CLEAN_MAP, cost_block.second.x, cost_block.second.y, cost_block.first);
	}
//	clean_map_.print(CLEAN_MAP, Cells{curr});
	for (auto &&p_it :passed_path_)
	{
		clean_map_.setCells(CLEAN_MAP, p_it.toCell().x, p_it.toCell().y, CLEANED);
	}


//	clean_map_.print(CLEAN_MAP, Cells{curr});
	//For slip mark
	for(auto &&cost_block : c_blocks){
		if(cost_block.first == BLOCKED_SLIP)
			clean_map_.setCell(CLEAN_MAP,cost_block.second.x,cost_block.second.y,BLOCKED_SLIP);
	}

	//tx pass path via serial wifi

	s_wifi.cacheMapData(passed_path_);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_MAP);
	c_blocks.clear();
	passed_path_.clear();
	return false;
}

bool CleanModeNav::markRealTime()
{
//	while (ros::ok()) {
//		sleep(0.2);
//		wheel.stop();
		auto p_mt = boost::dynamic_pointer_cast<IMoveType>(sp_action_);
		std::vector<Vector2<int>> markers;
		if (lidar.isScanCompensateReady())
			lidar.lidarMarker(markers, p_mt->movement_i_, action_i_);
//		ROS_INFO("markers.size() = %d", markers.size());
		for (const auto& marker : markers) {
//			ROS_INFO("marker(%d, %d)", marker.x, marker.y);
			auto cell = getPosition().getRelative(marker.x * CELL_SIZE, marker.y * CELL_SIZE).toCell();
//			clean_map_.setCell(CLEAN_MAP, cell.x, cell.y, BLOCKED_LIDAR);
			c_blocks.insert({BLOCKED_LIDAR, cell});
		}
//	}
	return true;

}
bool CleanModeNav::isExit()
{
	if (isStateInit())
	{
		if (action_i_ == ac_open_lidar && sp_action_->isTimeUp())
		{
			error.set(ERROR_CODE_LIDAR);
			setNextMode(md_idle);
			ev.fatal_quit = true;
			return true;
		}
	}

	if (isStateGoHomePoint() || isStateGoToCharger())
	{
		if (ev.key_clean_pressed)
		{
			ROS_WARN("%s %d: Exit for ev.key_clean_pressed during state %s.", __FUNCTION__, __LINE__,
					 isStateGoHomePoint() ? "go home point" : "go to charger");
			setNextMode(md_idle);
			return true;
		}
	}

	if (isStatePause())
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
		else if (ev.remote_direction_left || ev.remote_direction_right || ev.remote_direction_forward || s_wifi.receiveRemote())
		{
			ROS_WARN("%s %d: Exit for pause and remote left/right/forward or wifi remote.", __FUNCTION__, __LINE__);
			setNextMode(md_remote);
			return true;
		}
		else if (ev.remote_follow_wall || s_wifi.receiveFollowWall())
		{
			ROS_WARN("%s %d: Exit for pause and remote or wifi follow wall.", __FUNCTION__, __LINE__);
			setNextMode(cm_wall_follow);
			return true;
		}
		else if (ev.remote_spot || s_wifi.receiveSpot())
		{
			ROS_WARN("%s %d: Exit for pause and remote or wifi spot.", __FUNCTION__, __LINE__);
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

	if (ev.fatal_quit || sp_action_->isExit())
	{
		ROS_WARN("%s %d: Exit for ev.fatal_quit || sp_action_->isExit().", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (ev.key_long_pressed)
	{
		ROS_WARN("%s %d: Exit for ev.key_long_pressed.", __FUNCTION__, __LINE__);
		setNextMode(md_sleep);
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
	bool reset_wifi = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed to sleep.", __FUNCTION__, __LINE__);
			beeper.beepForCommand(VALID);
			long_press = true;
		}
		if (isStatePause() && !reset_wifi && key.getPressTime() > 5)
		{
			ROS_WARN("%s %d: key clean long pressed to reset wifi.", __FUNCTION__, __LINE__);
			beeper.beepForCommand(VALID);
			reset_wifi = true;
		}
		usleep(20000);
	}
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	if (reset_wifi)
	{
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_SMART_AP_LINK);
		sp_action_.reset();
		sp_action_.reset(new ActionPause);
	}
	else if (long_press)
		ev.key_long_pressed = true;
	ev.key_clean_pressed = true;

	key.resetTriggerStatus();
}

void CleanModeNav::remoteHome(bool state_now, bool state_last)
{
	if (isStateClean() || isStatePause() || isStateSpot() || isStateFollowWall())
	{
		ROS_WARN("%s %d: remote home.", __FUNCTION__, __LINE__);
		beeper.beepForCommand(VALID);
		ev.remote_home = true;
	}
	else
	{
		ROS_WARN("%s %d: remote home but not valid.", __FUNCTION__, __LINE__);
		beeper.beepForCommand(INVALID);
	}
	remote.reset();
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
	if (isStatePause())
	{
		beeper.beepForCommand(VALID);
		ROS_INFO("%s %d: Remote right.", __FUNCTION__, __LINE__);
		ev.remote_direction_left = true;
	}
	/*else if (sp_state == state_clean.get())
	{
		//todo: Just for testing.
		beeper.beepForCommand(VALID);
		continue_point_ = getPosition();
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.x, continue_point_.y);
		ev.battery_home = true;
		go_home_for_low_battery_ = true;
	}*/
	else
		beeper.beepForCommand(INVALID);

	remote.reset();
}

void CleanModeNav::remoteDirectionRight(bool state_now, bool state_last)
{
	if (isStatePause())
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
	if (isStatePause())
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
	if (isStatePause())
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
	if (isStateClean() || isStateSpot() || isStatePause())
	{
		ROS_INFO("%s %d: Remote spot.", __FUNCTION__, __LINE__);
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
	if(water_tank.checkEquipment(true)){
		beeper.beepForCommand(INVALID);
	}else if(isInitState() || isStateClean() || isStateResumeLowBatteryCharge() || isStateGoHomePoint() || isStateGoToCharger() || isStatePause())
	{
		beeper.beepForCommand(VALID);
		vacuum.isMaxInClean(!vacuum.isMaxInClean());
		speaker.play(vacuum.isMaxInClean() ? VOICE_VACCUM_MAX : VOICE_CLEANING_NAVIGATION);
		if(isStateClean() || isStateResumeLowBatteryCharge() || (isInitState()&& action_i_ > ac_open_gyro)) {
			vacuum.setCleanState();
		}
	}
	if (isStatePause())
	{
		// Reset the start timer for action.
		sp_action_.reset();
		sp_action_.reset(new ActionPause);
	}
	remote.reset();
}

void CleanModeNav::remotePlan(bool state_now, bool state_last)
{
	if (isStatePause() && appmt_obj.getPlanStatus() > 2)
	{
		appmt_obj.resetPlanStatus();
		appmt_obj.timesUp();
		// Sleep for 50ms cause the status 3 will be sent for 3 times.
		usleep(50000);
		// Reuse key clean triggered to activate the plan.
		ev.key_clean_pressed = true;
		INFO_YELLOW("Plan activated, set ev.key_clean_pressed.");
	}
	else
		EventHandle::remotePlan(state_now, state_last);
}

void CleanModeNav::batteryHome(bool state_now, bool state_last)
{
	if (isStateClean())
	{
		continue_point_ = getPosition();
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.x, continue_point_.y);
		ev.battery_home = true;
	}
}

void CleanModeNav::chargeDetect(bool state_now, bool state_last)
{
	if (!ev.charge_detect)
	{
		if (isStateInit() && action_i_ == ac_back_from_charger)
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
		else if (isStateGoToCharger())
		{
			ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
			ev.charge_detect = charger.getChargeStatus();
		}
	}
}
// End event handlers.

// ------------------State init--------------------
bool CleanModeNav::isSwitchByEventInStateInit() {
	if (checkEnterPause() || ACleanMode::isSwitchByEventInStateInit())
	{
		if (action_i_ == ac_back_from_charger)
			setHomePoint();
		return true;
	}
	return false;
}

bool CleanModeNav::updateActionInStateInit() {
	if (action_i_ == ac_null)
		action_i_ = ac_open_gyro;
	else if (action_i_ == ac_open_gyro)
	{
		// If it is the starting of navigation mode, paused_odom_radian_ will be zero.
		odom.setRadianOffset(paused_odom_radian_);
		ROS_INFO("%s,%d,angle offset:%f",__FUNCTION__,__LINE__,radian_to_degree(paused_odom_radian_));

		if (charger.isOnStub()){
			action_i_ = ac_back_from_charger;
			found_charger_ = true;
			boost::dynamic_pointer_cast<StateInit>(state_init)->initBackFromCharge();
		}
		else{
			action_i_ = ac_open_lidar;
			boost::dynamic_pointer_cast<StateInit>(state_init)->initOpenLidar();
		}
	} else if (action_i_ == ac_back_from_charger)
	{
		if (!has_aligned_and_open_slam_) // Init odom position here.
			robot::instance()->initOdomPosition();

		boost::dynamic_pointer_cast<StateInit>(state_init)->initOpenLidar();
		action_i_ = ac_open_lidar;
		setHomePoint();
	} else if (action_i_ == ac_open_lidar)
	{
		if (!has_aligned_and_open_slam_)
		{
			action_i_ = ac_align;
		}
		else
			return false;
	} else if (action_i_ == ac_align){
		{
			action_i_ = ac_open_slam;
			align_count_ ++;
			start_align_radian_ = odom.getRadianOffset();
			if(align_count_%2 == 0)
			{
				start_align_radian_= ranged_radian(start_align_radian_ -PI/2);
				odom.setRadianOffset(start_align_radian_);
//				ROS_INFO("rad %f",start_align_radian_);
			}
			ROS_INFO("odom rad, align_count : %f, %d", odom.getRadian(), align_count_);
//			beeper.beepForCommand(INVALID);
		}

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
	if (has_aligned_and_open_slam_) {
		if (low_battery_charge_) {
			low_battery_charge_ = false;
//			sp_state = state_resume_low_battery_charge.get();
			// No need to go to continue point according to functional requirement v1.2.
			sp_state = state_clean.get();
		}
		else{ // Resume from pause, because slam is not opened for the first time that open lidar action finished.
			sp_state = sp_saved_states.back();
			sp_saved_states.pop_back();
		}
	}
	else {//if (action_i_ == ac_open_slam)
		has_aligned_and_open_slam_ = true;

		if (remote_go_home_point)
		{
			sp_state = sp_saved_states.back();
			sp_saved_states.pop_back();
		}
		else
		{
			auto curr = getPosition();
//			curr.dir = iterate_point_.dir;
//			passed_path_.push_back(curr);
			start_point_.th = curr.th;
			sp_state = state_clean.get();
		}
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
	bool ret = false;
	sp_action_.reset();//to mark in destructor
//	pubCleanMapMarkers(clean_map_, pointsGenerateCells(remain_path_));
	old_dir_ = iterate_point_.dir;
//    std::equal(history_.begin(),history_.end,[](const Cell_t his){
//	});
    BoundingBox<Point_t> bound;
	bound.SetMinimum({iterate_point_.x - CELL_SIZE, iterate_point_.y - CELL_SIZE});
	bound.SetMaximum({iterate_point_.x + CELL_SIZE, iterate_point_.y + CELL_SIZE});
    //check is always in same range;
//	ROS_ERROR("%s,%d: iterate_point in all in history_(%d)",__FUNCTION__, __LINE__,history_.size());
//	std::copy(history_.begin(), history_.end(),std::ostream_iterator<Point_t>(std::cout,","));
//	ROS_ERROR("%s,%d: iterate_point in all in history_(%d)",__FUNCTION__, __LINE__,history_.size());
//    if(history_.size() == 3 && std::all_of(std::begin(history_), std::end(history_), [&](const Point_t & p_it){ return bound.Contains(p_it); })){
//		ROS_ERROR("%s,%d: iterate_point in all in history_",__FUNCTION__, __LINE__);
//		std::copy(std::begin(history_), std::end(history_),std::ostream_iterator<Point_t>(std::cout,","));
//        beeper.beepForCommand(VALID);
//        ev.robot_slip = true;
//		history_.clear();
//		is_stay_in_same_postion_long_time = true;
//		return false;
//	};
//    history_.push_back(iterate_point_);
	if(action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{

		extern int g_follow_last_follow_wall_dir;
		if(g_follow_last_follow_wall_dir != 0)
		{
			ROS_ERROR("g_follow_last_follow_wall_dir, old_dir_(%d)",old_dir_);
			old_dir_ = plan_path_.back().dir;
		}
		else
			old_dir_ = MAP_ANY;
	}

	if (clean_path_algorithm_->generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
		pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
		iterate_point_ = plan_path_.front();
//		plan_path_.pop_front();
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
		auto npa = boost::dynamic_pointer_cast<NavCleanPathAlgorithm>(clean_path_algorithm_);

//		ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!should_follow_wall(%d)",should_follow_wall);
		if ( old_dir_ != MAP_ANY && should_follow_wall &&
						(		npa->curr_filter_ == &npa->filter_p0_1t_xp
						||	npa->curr_filter_ == &npa->filter_p0_1t_xn
						||	npa->curr_filter_ == &npa->filter_n0_1t_xp
						||	npa->curr_filter_ == &npa->filter_n0_1t_xn
						||	npa->curr_filter_ == &npa->filter_p2
						||	npa->curr_filter_ == &npa->filter_p1
						||	npa->curr_filter_ == &npa->filter_n2
						||	npa->curr_filter_ == &npa->filter_n1)
				)
		{
				auto toward_pos = isXAxis(old_dir_) ? npa->curr_filter_->towardPos(): (iterate_point_.toCell().x - plan_path_.back().toCell().x) > 0;
				bool is_left = isPos(old_dir_) ^ toward_pos;
				action_i_ = is_left ? ac_follow_wall_left : ac_follow_wall_right;
		}
		else
			action_i_ = ac_linear;

		genNextAction();
		ret = true;
	}
	should_follow_wall = false;
	return ret;
}

void CleanModeNav::switchInStateClean() {
	if (clean_path_algorithm_->checkTrapped(clean_map_, getPosition().toCell())) {
		ROS_WARN("%s,%d: enter state trapped",__FUNCTION__,__LINE__);
		sp_saved_states.push_back(sp_state);
		sp_state = state_folllow_wall.get();
		is_trapped_ = true;
		is_isolate = true;
		is_closed = true;
		closed_count_ = 0;
		isolate_count_ = 0;
	}
	else {
		sp_state = state_go_home_point.get();
		ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_points_.size());
		go_home_path_algorithm_.reset();
		go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
		//s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_LAST_CLEANMAP);
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
	return ACleanMode::isSwitchByEventInStateGoHomePoint();
}

// ------------------State go to charger--------------------

bool CleanModeNav::isSwitchByEventInStateGoToCharger()
{
	return ACleanMode::isSwitchByEventInStateGoToCharger();
}

void CleanModeNav::switchInStateGoToCharger()
{
	if (charger.isOnStub())
	{
		if (go_home_for_low_battery_)
		{
			// If it is during low battery go home, it should not leave the clean mode, it should just charge.
			ROS_INFO("%s %d: Enter low battery charge.", __FUNCTION__, __LINE__);
			sp_state = state_charge.get();
			sp_state->init();
			paused_odom_radian_ = odom.getRadian();
			go_home_for_low_battery_ = false;
			go_home_path_algorithm_.reset();
			setFirstTimeGoHomePoint(true);
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
	if (ev.remote_spot || s_wifi.receiveSpot())
	{
		if (s_wifi.receiveSpot())
			s_wifi.resetReceivedWorkMode();
		ev.remote_spot= false;
//		mapMark();
		sp_action_.reset();
		clean_path_algorithm_.reset(new SpotCleanPathAlgorithm);
		sp_state = state_spot.get();
		sp_state->init();
		return true;
	}
	return false;
}

bool CleanModeNav::isSwitchByEventInStateSpot()
{
	if (ev.remote_spot || s_wifi.receivePlan1() || s_wifi.receiveSpot())
	{
		if (s_wifi.receivePlan1() || s_wifi.receiveSpot())
			s_wifi.resetReceivedWorkMode();
		sp_state = state_clean.get();
		sp_state->init();
		action_i_ = ac_null;
		sp_action_.reset();
		clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
		ev.remote_spot = false;

		return true;
	}

	return checkEnterPause() || ACleanMode::checkEnterGoHomePointState() || ACleanMode::isSwitchByEventInStateSpot();
}

void CleanModeNav::switchInStateSpot()
{
	action_i_ = ac_null;
	sp_action_.reset();
	sp_state = state_clean.get();
	sp_state->init();
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
}

// ------------------State pause--------------------

bool CleanModeNav::checkEnterPause()
{
	if (ev.key_clean_pressed || s_wifi.receiveIdle()/*|| is_stay_in_same_postion_long_time*/)
	{
		if (s_wifi.receiveIdle())
			s_wifi.resetReceivedWorkMode();

//		is_stay_in_same_postion_long_time = false;
		ev.key_clean_pressed = false;
		speaker.play(VOICE_CLEANING_PAUSE);
		paused_odom_radian_ = odom.getRadian();
		ROS_INFO("%s %d: Key clean pressed, pause cleaning.Robot pose(%f)", __FUNCTION__, __LINE__,radian_to_degree(paused_odom_radian_));
		sp_action_.reset();
		sp_saved_states.push_back(sp_state);
		sp_state = state_pause.get();
		sp_state->init();
//		mapMark();
		return true;
	}

	return false;
}

bool CleanModeNav::checkResumePause()
{
	if (ev.key_clean_pressed || ev.remote_home || s_wifi.receivePlan1() || s_wifi.receiveHome())
	{
		if (s_wifi.receivePlan1())
			s_wifi.resetReceivedWorkMode();
		ev.key_clean_pressed = false;
		speaker.play(VOICE_CLEANING_CONTINUE);
		sp_action_.reset();
		action_i_ = ac_null;
		ROS_INFO("%s %d: Resume cleaning.", __FUNCTION__, __LINE__);
		// It will NOT change the state.
		if ((ev.remote_home || s_wifi.receiveHome()) && sp_saved_states.back() != state_go_home_point.get())
		{
			ROS_INFO("%s %d: Resume to go home point state.", __FUNCTION__, __LINE__);
			sp_saved_states.pop_back();
			sp_saved_states.push_back(state_go_home_point.get());
			if (go_home_path_algorithm_ == nullptr)
				go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
			ev.remote_home = false;
			if (s_wifi.receiveHome())
				s_wifi.resetReceivedWorkMode();
			speaker.play(VOICE_GO_HOME_MODE);
			remote_go_home_point = true;
		}
		sp_state = state_init.get();
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
	sp_state = state_init.get();
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
		sp_state = state_init.get();
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

	sp_state = state_clean.get();
	sp_state->init();
	sp_action_.reset();
}

// ------------------State Exception Resume--------------------
bool CleanModeNav::isSwitchByEventInStateExceptionResume() {
	return checkEnterPause();
}

