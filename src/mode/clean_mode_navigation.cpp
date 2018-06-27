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
#include "log.h"

//#define NAV_INFO() ROS_INFO("st(%d),ac(%d)", state_i_, action_i_)

int CleanModeNav::align_count_ = 0;
CleanModeNav::CleanModeNav()
{
	ROS_WARN("%s %d: Entering Navigation mode\n=========================" , __FUNCTION__, __LINE__);

	sp_state = state_init.get();
	if(plan_activation)
	{
		plan_activation = false;
//		speaker.play(VOICE_APPOINTMENT_START_UNOFFICIAL, false);
	}
//	else
	if (!charger.enterNavFromChargeMode() && !charger.isOnStub())
	{
		has_played_start_voice_ = true;
		speaker.play(VOICE_CLEANING_START);
	}
	action_i_ = ac_null;
	updateActionInStateInit();
	sp_state->init();

	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());
	mode_i_ = cm_navigation;

	//clear real time map which store in cloud....
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_CLEAR_MAP);

	// Clear the map in app.
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_CLEAR_APP_MAP);
}

CleanModeNav::~CleanModeNav()
{
	s_wifi.clearMapCache();
	charger.enterNavFromChargeMode(false);
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool CleanModeNav::mapMark()
{

	ROS_INFO("%s %d: Start updating block_map.", __FUNCTION__, __LINE__);
	if(passed_cell_path_.empty())
	{
		ROS_WARN("%s %d: pass_path is empty, add curr_point(%.2f,%.2f,%.2f,%d).", __FUNCTION__, __LINE__, getPosition().x,
				 getPosition().y, getPosition().th, getPosition().dir);
		passed_cell_path_.push_back(getPosition());
	}

	displayPointPath((passed_cell_path_));

	// Update for passed_path.
	GridMap block_map{};
	for (auto &&p_it :passed_cell_path_)
		block_map.setCells(p_it.toCell().x, p_it.toCell().y, CLEANED, ROBOT_SIZE_1_2);

	const auto start = passed_cell_path_.front().toCell();
	const auto curr = passed_cell_path_.back().toCell();
/*
	Cells c_bound1;
	auto is_cleaned_bound = [&](const Cell_t &c_it){
		if(block_map.getCell(c_it.x, c_it.y) == CLEANED)
		{
			for (auto index = 0; index < 4; index++) {
				auto neighbor = c_it + cell_direction_[index];
				if (block_map.getCell(neighbor.x, neighbor.y) != CLEANED)
					return true;
			}
			return false;
		}
		return false;
	};

	block_map.find_if(start, c_bound1,is_cleaned_bound);
//	ROS_INFO("1111111111111111111");
//	block_map.print(curr, c_bound1);

	Cells c_bound2;
	auto is_cleaned_bound2 = [&](const Cell_t &c_it){
		if(block_map.getCell(c_it.x, c_it.y) == UNCLEAN)
		{
			for (auto index = 0; index < 4; index++) {
				auto neighbor = c_it + cell_direction_[index];
				if (block_map.getCell(neighbor.x, neighbor.y) == CLEANED)
					return true;
			}
			return false;
		}
		return false;
	};
	block_map.find_if(start, c_bound2,is_cleaned_bound2);
//	ROS_INFO("2222222222222222222");
//	block_map.print(curr, c_bound2);

	Cells c_bound3;
	auto is_cleaned_bound3_target_selection = [&](const Cell_t &c_it)
	{
		if(block_map.getCell(c_it.x, c_it.y) == CLEANED)
		{
			for (auto index = 0; index < 4; index++) {
				auto neighbor = c_it + cell_direction_[index];
				if (block_map.getCell(neighbor.x, neighbor.y) == UNCLEAN)
					return true;
			}
		}
		return false;
	};
	auto is_cleaned_bound3_expand_condition = [&](const Cell_t cell, const Cell_t neighbor_cell)
	{
		return !block_map.cellIsOutOfRange(neighbor_cell) && !block_map.isOutOfTargetRange(neighbor_cell)
			   && block_map.getCell(COST_MAP, neighbor_cell.x, neighbor_cell.y) == 0
			   && block_map.getCost(neighbor_cell.x, neighbor_cell.y) == CLEANED;
	};
	block_map.dijkstra(curr, c_bound3, true, is_cleaned_bound3_target_selection, is_cleaned_bound3_expand_condition);
//	ROS_INFO("3333333333333333333");
//	block_map.print(curr, c_bound3);*/

	Cells out_bound_of_passed_path;
	auto expand_condition = [&](const Cell_t& next, const Cell_t& neighbor)
	{
		return block_map.getCost(next.x, next.y) == CLEANED;
	};
	clean_path_algorithm_->dijkstra(block_map, curr, out_bound_of_passed_path, false, TargetVal(&block_map, UNCLEAN),
									isAccessible(&block_map, expand_condition));
//	ROS_ERROR("4444444444444444444");
//	block_map.print(curr, out_bound_of_passed_path);
//	block_map.print(curr, COST_MAP, Cells{});
//	displayCellPath(out_bound_of_passed_path);
//	ROS_ERROR("4444444444444444444");

	if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right) {
		if (!c_blocks.empty()) {
			auto dy = action_i_ == ac_follow_wall_left ? 2 : -2;
			std::for_each(passed_cell_path_.begin(), passed_cell_path_.end(),[&](const Point_t& point){
				BoundingBox<Point_t> bound;
				bound.SetMinimum({passed_cell_path_.front().x - CELL_SIZE/4, passed_cell_path_.front().y - CELL_SIZE/4});
				bound.SetMaximum({passed_cell_path_.front().x + CELL_SIZE/4, passed_cell_path_.front().y + CELL_SIZE/4});
				if(!bound.Contains(point))
				{
//					ROS_INFO("in cfw(%d,%d),(%d,%d)", point.toCell().x, point.toCell().y, getPosition().toCell().x, getPosition().toCell().y);
					ROS_INFO("Not Cont front(%d,%d),curr(%d,%d),point(%d,%d)", passed_path_for_follow_wall_mark.front().toCell().x, passed_path_for_follow_wall_mark.front().toCell().y,
									 getPosition().toCell().x, getPosition().toCell().y, point.toCell().x, point.toCell().y);
					c_blocks.insert({BLOCKED_FW, point.getCenterRelative(0, dy * CELL_SIZE).toCell()});
				}
				else {
					ROS_INFO("Contains front(%d,%d),curr(%d,%d),point(%d,%d)", passed_path_for_follow_wall_mark.front().toCell().x, passed_path_for_follow_wall_mark.front().toCell().y,
									 getPosition().toCell().x, getPosition().toCell().y, point.toCell().x, point.toCell().y);
				}
			});
		}
	}
	else if (sp_state == state_clean.get()) {
		setLinearCleaned();
		// Set home cell.
		if (ev.rcon_status)
		{
			setRconPoint(getPosition());
			if (!hasSeenChargerDuringCleaning())
				setSeenChargerDuringCleaning();
		}
	}
	for (auto &&cost_block : c_blocks) {
		if(std::find_if(out_bound_of_passed_path.begin(), out_bound_of_passed_path.end(), [&](const Cell_t& c_it)
		{ return c_it == cost_block.second; }) != out_bound_of_passed_path.end())
//		if(std::find_if(c_bound2.begin(), c_bound2.end(), [&](const Cell_t& c_it)
//		{ return c_it == cost_block.second; }) != c_bound2.end())
//			if(!(cost_block.first == BLOCKED_LIDAR && (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)))
			clean_map_.setCost(cost_block.second.x, cost_block.second.y, cost_block.first);
	}
//	clean_map_.print(Cells{curr});
	for (auto &&p_it :passed_cell_path_)
	{
		clean_map_.setSpecificCells(p_it.toCell().x, p_it.toCell().y, CLEANED, BLOCKED_RCON, ROBOT_SIZE_1_2);
	}


//	clean_map_.print(Cells{curr});
	for(auto &&cost_block : c_blocks){
		//For slip mark
		if(cost_block.first == BLOCKED_SLIP)
			clean_map_.setCost(cost_block.second.x, cost_block.second.y, BLOCKED_SLIP);
		if(cost_block.first == BLOCKED_TILT)
			clean_map_.setCost(cost_block.second.x, cost_block.second.y, BLOCKED_TILT);
		// Special marking for rcon blocks.
		if(cost_block.first == BLOCKED_TMP_RCON)
			clean_map_.setCost(cost_block.second.x, cost_block.second.y, BLOCKED_TMP_RCON);
	}

	//tx pass path via serial wifi
	s_wifi.cacheMapData(passed_cell_path_);
	//s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_PATH);
	c_blocks.clear();
	passed_cell_path_.clear();
	passed_path_for_follow_wall_mark.clear();
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
//			clean_map_.setCost(cell.x, cell.y, BLOCKED_LIDAR);
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
			robot_error.set(ERROR_CODE_LIDAR);
			setNextMode(md_idle);
			ev.fatal_quit = true;
			return true;
		}

		if (!checkingIfSwitchIsOn() && (ev.key_clean_pressed || s_wifi.receiveIdle()))
		{
			ROS_WARN("%s %d: Exit for ev.key_clean_pressed or wifi receive idle.", __FUNCTION__, __LINE__);
			setNextMode(md_idle);
			return true;
		}
	}

	if (isStateGoHomePoint() || isStateGoToCharger() || isHasEnterStateIsGoHomePoints())
	{
		if (ev.key_clean_pressed)
		{
			ROS_WARN("%s %d: Exit for ev.key_clean_pressed during state %s.", __FUNCTION__, __LINE__,
					 isStateGoHomePoint() ? "go home point" : "go to charger");
			setNextMode(md_idle);
			return true;
		}

		if (s_wifi.receivePlan1())
		{
			ROS_WARN("%s %d: Exit for wifi plan1.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
			return true;
		}

		if (s_wifi.receiveIdle())
		{
			ROS_WARN("%s %d: Exit for wifi idle.", __FUNCTION__, __LINE__);
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
		else if (ev.remote_direction_left || ev.remote_direction_right || ev.remote_direction_forward)
		{
			ROS_WARN("%s %d: Exit for pause and remote left/right/forward or wifi remote.", __FUNCTION__, __LINE__);
			setNextMode(md_remote);
			if (ev.remote_direction_forward)
				MoveTypeRemote::forwardStart();
			else if (ev.remote_direction_left)
				MoveTypeRemote::leftStart();
			else if (ev.remote_direction_right)
				MoveTypeRemote::rightStart();
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

	if (isStateCharge())
	{
		if (ev.key_clean_pressed)
		{
			ROS_WARN("%s %d: Exit for ev.key_clean_pressed during state charge.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
			return true;
		}

		if (s_wifi.receivePlan1())
		{
			ROS_WARN("%s %d: Exit for wifi plan1.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
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

	if (ev.battery_low)
	{
		ROS_WARN("%s %d: Exit to idle mode for low battery(%.2fV).", __FUNCTION__, __LINE__, battery.getVoltage() / 100.0);
		setNextMode(md_idle);
		return true;
	}

	return false;
}

bool CleanModeNav::moveTypeNewCellIsFinish(IMoveType *p_mt)
{
	auto distance = updatePath();
	if (is_trapped_)
	{
		if (robot_timer.trapTimeout(ESCAPE_TRAPPED_TIME))
		{
			trapped_time_out_ = true;
			return true;
		}
		else if (pathAlgorithmCheckOutOfTrapped(p_mt))
			return true;
	}

	return checkClosed(p_mt, distance);
}

bool CleanModeNav::moveTypeRealTimeIsFinish(IMoveType *p_mt)
{
	Points ins_path{};//instantaneous path
	auto curr = getPosition();
	ins_path.push_back(curr);
	if (isStateClean() && (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)) //rounding
	{
		/*BoundingBox<Point_t> bound;
		bound.SetMinimum({passed_cell_path_.front().x - CELL_SIZE / 4, passed_cell_path_.front().y - CELL_SIZE / 4});
		bound.SetMaximum({passed_cell_path_.front().x + CELL_SIZE / 4, passed_cell_path_.front().y + CELL_SIZE / 4});
		if (!bound.Contains(curr))
		{
//			ROS_INFO("Not Cont front(%f,%f),curr(%f,%f),bound(min(%f,%f),max(%f,%f))", passed_cell_path_.front().x, passed_cell_path_.front().y,
//					 curr.x, curr.y,bound.min.x,bound.min.y, bound.max.x,bound.max.y);
			auto npa = boost::dynamic_pointer_cast<NavCleanPathAlgorithm>(clean_path_algorithm_);
			int16_t y = static_cast<int16_t>(plan_path_.begin()->toCell().y + ((npa->is_pox_y()) ? -2 : 2));
			setFollowWall(clean_map_, action_i_ == ac_follow_wall_left, ins_path);
		}
//		else
//		{
//					ROS_WARN("Cont front(%f,%f),curr(%f,%f),bound(min(%f,%f),max(%f,%f))", passed_cell_path_.front().x, passed_cell_path_.front().y,
//							 curr.x, curr.y,bound.min.x,bound.min.y, bound.max.x,bound.max.y);
//		}*/

		auto p_mt_follow_wall = dynamic_cast<MoveTypeFollowWall *>(p_mt);
		if(p_mt_follow_wall->movement_i_ == p_mt_follow_wall->mm_forward || p_mt_follow_wall->movement_i_ == p_mt_follow_wall->mm_straight)
		{
			return p_mt_follow_wall->isNewLineReach(clean_map_) || p_mt_follow_wall->isOverOriginLine(clean_map_);
		}
	}
	else if (isStateSpot() && action_i_ != ac_linear)
	{
		auto p_mt_follow_wall = dynamic_cast<MoveTypeFollowWall *>(p_mt);
		if(p_mt_follow_wall->outOfRange(getPosition(), iterate_point_))
			return true;
	}
	return ACleanMode::moveTypeRealTimeIsFinish(p_mt);
}

/*bool CleanModeNav::moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt)
{
	if (sp_state == state_folllow_wall)
		return p_mt->isBlockCleared(clean_map_, passed_cell_path_);
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
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_REBIND);
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
	if (isStateClean() || isStatePause() || isStateSpot() || isStateFollowWall()
		|| (isStateGoHomePoint() && isFirstTimeGoHomePoint()))
	{
		ROS_WARN("%s %d: remote home.", __FUNCTION__, __LINE__);
//		beeper.beepForCommand(VALID);
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

	if (isStateCharge())
		// Beep before back from charger.
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
		ROS_INFO("%s %d: Remote left.", __FUNCTION__, __LINE__);
		ev.remote_direction_left = true;
	}
	/*else if (isStateClean())
	{
		//todo: Just for testing.
		beeper.beepForCommand(VALID);
		continue_point_ = getPosition();
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.x, continue_point_.y);
		ev.battery_home = true;
		go_home_for_low_battery_ = true;
	}*/
	/*else if (isStateInit())
	{
		//todo: Just for testing.
		ROS_WARN("%s %d: Main brush overcurrent.", __FUNCTION__, __LINE__);
		ev.oc_brush_main = true;
	}*/
	/*else
	{
		//todo: Just for testing.
		ROS_WARN("%s %d: Low battery(%.2fV).", __FUNCTION__, __LINE__, battery.getVoltage() / 100.0);
		ev.battery_low = true;
		ev.fatal_quit = true;
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
	/*else if (isStateCharge())
	{
		beeper.beepForCommand(VALID);
		ROS_INFO("%s %d: Remote right.", __FUNCTION__, __LINE__);
		ev.remote_direction_right = true;
	}*/
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
//		beeper.beepForCommand(VALID);
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
//		beeper.beepForCommand(VALID);
	}
	else
		beeper.beepForCommand(INVALID);

	remote.reset();
}

void CleanModeNav::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	if(water_tank.getStatus(WaterTank::operate_option::swing_motor)){
		beeper.beepForCommand(INVALID);
	}else if(isInitState() || isStateClean() || isStateGoHomePoint() || isStateGoToCharger() || isStatePause())
	{
//		beeper.beepForCommand(VALID);
		vacuum.setForUserSetMaxMode(!vacuum.isUserSetMaxMode());
		ACleanMode::setVacuum();
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
	if (isStatePause() && !ev.key_clean_pressed && appmt_obj.getPlanStatus() > 2)
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
	if (!ev.battery_home && isStateClean())
	{
		continue_point_ = getPosition();
		ROS_WARN("%s %d: low battery, battery = %dmv, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.toCell().x, continue_point_.toCell().y);
		ev.battery_home = true;
	}
}

void CleanModeNav::chargeDetect(bool state_now, bool state_last)
{
	if (!ev.charge_detect)
	{
		if (isStateInit() && checkingIfSwitchIsOn() && sp_action_->isTimeUp())
		{
			// If switch is not on, the cliff value should be around 0.
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

// ------------------State init--------------------
bool CleanModeNav::isSwitchByEventInStateInit() {
	/*if (*//*checkEnterPause() || *//*ACleanMode::isSwitchByEventInStateInit())
	{
		if (action_i_ == ac_back_from_charger)
		{
			setRconPoint(getPosition());
			if (!hasSeenChargerDuringCleaning())
				setSeenChargerDuringCleaning();
		}
		return true;
	}*/
	return false;
}

bool CleanModeNav::updateActionInStateInit() {
	if (action_i_ == ac_null)
	{
		if (charger.enterNavFromChargeMode() || charger.isOnStub()){
			charger.enterNavFromChargeMode(false);
			action_i_ = ac_back_from_charger;
			found_charger_ = true;
			has_mark_charger_ = false;
		}
		else
			action_i_ = ac_open_gyro_and_lidar;
	} else if (action_i_ == ac_back_from_charger)
	{
		if (!has_played_start_voice_)
		{
			has_played_start_voice_ = true;
			speaker.play(VOICE_CLEANING_START);
		}

		if (!has_aligned_and_open_slam_) // Init odom position here.
			robot::instance()->initOdomPosition();

		action_i_ = ac_open_gyro_and_lidar;
//		boost::dynamic_pointer_cast<StateInit>(state_init)->initForNavigation();
		setRconPoint(getPosition());
		if (!hasSeenChargerDuringCleaning())
			setSeenChargerDuringCleaning();
	}
	else if (action_i_ == ac_open_gyro_and_lidar)
	{
		// If it is the starting of navigation mode, paused_odom_radian_ will be zero.
		odom.setRadianOffset(paused_odom_radian_);
		ROS_INFO("%s,%d,angle offset:%f",__FUNCTION__,__LINE__,radian_to_degree(paused_odom_radian_));

		action_i_ = ac_open_lidar;
	}
	else if (action_i_ == ac_open_lidar)
	{
		if (!has_aligned_and_open_slam_)
			action_i_ = ac_align;
		else
		{
			//set charge position
			ACleanMode::checkShouldMarkCharger((float)odom.getRadian(),0.6);
			return false;
		}
	}
	else if (action_i_ == ac_align)
	{
		action_i_ = ac_open_slam;
		align_count_++;
		start_align_radian_ = static_cast<float>(odom.getRadianOffset());
		if (align_count_ % 2 == 0)
		{
			start_align_radian_ = static_cast<float>(ranged_radian(start_align_radian_ - PI / 2));
			odom.setRadianOffset(start_align_radian_);
//				ROS_INFO("rad %f",start_align_radian_);
		}
		ROS_INFO("odom rad: %f, align_count %d", odom.getRadian(), align_count_);
		//set charge position
		ACleanMode::checkShouldMarkCharger((float) odom.getRadian(), 0.6);
//		beeper.beepForCommand(INVALID);

	}
	else if (action_i_ == ac_open_slam)
		return false;

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
//			sp_saved_states.popCurrRconPoint(stable_unique(sp_saved_states.begin(),sp_saved_states.end()),sp_saved_states.end());
			if (sp_saved_states.empty())
			{
				ROS_ERROR("%s %d: Saved state is empty!!", __FUNCTION__, __LINE__);
				sp_state = state_clean.get();
			}
			else
			{
				sp_state = sp_saved_states.back();
				sp_saved_states.pop_back();
			}
		}
	}
	else {//if (action_i_ == ac_open_slam)
		has_aligned_and_open_slam_ = true;

		if (isRemoteGoHomePoint() || isWifiGoHomePoint())
		{
			if (sp_saved_states.empty())
			{
				ROS_ERROR("%s %d: Saved state is empty!!", __FUNCTION__, __LINE__);
				sp_state = state_go_home_point.get();
			}
			else
			{
				sp_state = sp_saved_states.back();
				sp_saved_states.pop_back();
			}
		}
		else
		{
			auto curr = getPosition();
//			curr.dir = iterate_point_.dir;
//			passed_cell_path_.push_back(curr);
			home_points_manager_.setStartPointRad(curr.th);
			sp_state = state_clean.get();
		}
	}
	sp_state->init();
	action_i_ = ac_null;
	sp_action_.reset();

	// Prevent robot detects main brush over current during init state.
	ev.oc_brush_main = false;
}

// ------------------State clean--------------------
bool CleanModeNav::isSwitchByEventInStateClean() {
	return checkEnterPause() || checkEnterTempSpotState() || ACleanMode::isSwitchByEventInStateClean();
}

bool CleanModeNav::updateActionInStateClean(){
	bool ret = false;
//	ROS_ERROR("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~hello");
	sp_action_.reset();//to mark in destructor
//	pubCleanMapMarkers(clean_map_, points_to_cells(remain_path_));
    if(!plan_path_.empty())
	    old_dir_ = iterate_point_->dir;

	if(action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{

		extern int g_follow_last_follow_wall_dir;
		if(g_follow_last_follow_wall_dir != 0)
		{
			ROS_INFO("%s %d: g_follow_last_follow_wall_dir, old_dir_(%d)", __FUNCTION__, __LINE__, old_dir_);
			old_dir_ = plan_path_.back().dir;
		}
		else
			old_dir_ = MAP_ANY;
	}

	if (clean_path_algorithm_->generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
		pubCleanMapMarkers(clean_map_, *points_to_cells(plan_path_));
		iterate_point_ = plan_path_.begin();
//		plan_path_.pop_front();
		displayCellPath(*points_to_cells(plan_path_));
		auto npa = boost::dynamic_pointer_cast<NavCleanPathAlgorithm>(clean_path_algorithm_);

		if ( old_dir_ != MAP_ANY && should_follow_wall && npa->should_follow_wall() )
		{
				auto toward_pos = isXAxis(old_dir_) ? npa->is_pox_y(): (iterate_point_->toCell().x - plan_path_.back().toCell().x) > 0;
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
//		sp_saved_states.push_back(sp_state);
		sp_state = state_folllow_wall.get();
		beeper.debugBeep(VALID);
		is_trapped_ = true;
		is_isolate = true;
		is_closed = true;
		closed_count_ = 0;
		isolate_count_ = 0;
	}
	else {
		sp_state = state_go_home_point.get();
		clean_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_,&home_points_manager_));
	}
	sp_state->init();
	action_i_ = ac_null;
	genNextAction();
}

// ------------------State go home point--------------------
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
			clean_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_,&home_points_manager_));
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
		action_i_ = ac_null;
		speaker.play(VOICE_CLEANING_SPOT);
		clean_path_algorithm_.reset(new SpotCleanPathAlgorithm);
		sp_state = state_spot.get();
		sp_state->init();
		return true;
	}
	return false;
}

bool CleanModeNav::isSwitchByEventInStateSpot()
{
	if (ev.remote_spot || s_wifi.receivePlan1() || s_wifi.receiveIdle())
	{
		if (s_wifi.receivePlan1() || s_wifi.receiveIdle())
			s_wifi.resetReceivedWorkMode();
		sp_state = state_clean.get();
		sp_state->init();
		action_i_ = ac_null;
		sp_action_.reset();
		clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
		ev.remote_spot = false;

		return true;
	}

	if (checkEnterPause())
	{
		// Exit temp spot mode.
		sp_saved_states.clear();
		auto state = state_clean.get();
		sp_saved_states.push_back(state);
		clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
		return true;
	}

	return ACleanMode::checkEnterGoHomePointState() || ACleanMode::isSwitchByEventInStateSpot();
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
	if(isHasEnterStateIsGoHomePoints())
		return false;

	if (ev.key_clean_pressed || s_wifi.receiveIdle()/*|| is_stay_in_same_postion_long_time*/)
	{
		if (s_wifi.receiveIdle())
			s_wifi.resetReceivedWorkMode();

//		is_stay_in_same_postion_long_time = false;
		ev.key_clean_pressed = false;
		speaker.play(VOICE_CLEANING_PAUSE);
		if (action_i_ != ac_open_gyro_and_lidar) {
			paused_odom_radian_ = odom.getRadian();
		}
		ROS_INFO("%s %d: Key clean pressed, pause cleaning.Robot pose(%f)", __FUNCTION__, __LINE__,radian_to_degree(paused_odom_radian_));
		sp_action_.reset();
		if (!isStateInit())
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
		if ((ev.remote_home || s_wifi.receiveHome()))
		{
			ROS_INFO("%s %d: Resume to go home point state.", __FUNCTION__, __LINE__);
			if (sp_saved_states.empty())
				ROS_ERROR("%s %d: Saved state is empty!! Did it enter pause from init state?", __FUNCTION__, __LINE__);
			else
				sp_saved_states.pop_back();

			sp_saved_states.push_back(state_go_home_point.get());
			if (ev.remote_home)
				remote_go_home_point = true;
			else
				wifi_go_home_point = true;
			ev.remote_home = false;
			if (s_wifi.receiveHome())
				s_wifi.resetReceivedWorkMode();
			clean_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_,&home_points_manager_));
			speaker.play(VOICE_GO_HOME_MODE);
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
		// For M0 checking charge mode.
		serial.setWorkMode(CHARGE_MODE);
		return true;
	}
	else
		return false;
}

void CleanModeNav::switchInStateCharge()
{
	// Failed for charging. Maybe manual moved away from charger.
	ROS_WARN("%s %d: Failed for charging. Maybe manual moved away from charger.", __FUNCTION__, __LINE__);
	sp_state = nullptr;
	sp_action_.reset();
	moved_away_from_charger_ = true;
/*
	// Resume from low battery charge.
	speaker.play(VOICE_CLEANING_CONTINUE, false);
	ROS_INFO("%s %d: Resume low battery charge.", __FUNCTION__, __LINE__);
	sp_action_.reset();
	action_i_ = ac_null;
	sp_state = state_init.get();
	sp_state->init();
	low_battery_charge_ = true;*/
}

// ------------------State resume low battery charge--------------------
bool CleanModeNav::checkEnterResumeLowBatteryCharge()
{
	if (battery.isFull()/* || ev.remote_direction_right*/)
	{
		/*if (ev.remote_direction_right)
			ev.remote_direction_right = false;*/
		// Resume from low battery charge.
		speaker.play(VOICE_BATTERY_CHARGE_DONE, false);
		speaker.play(VOICE_CLEANING_CONTINUE, false);
		// For M0 resume work mode.
		serial.setWorkMode(WORK_MODE);
		ROS_INFO("%s %d: Resume low battery charge.", __FUNCTION__, __LINE__);
		if (action_i_ == ac_charge)
		{
			auto p_action = boost::dynamic_pointer_cast<MovementCharge>(sp_action_);
			if (p_action->stillCharging())
				charger.enterNavFromChargeMode(true);
		}
		sp_action_.reset();
		sp_state = state_init.get();
		action_i_ = ac_null;
		updateActionInStateInit();
		sp_state->init();
		clean_path_algorithm_.reset(new NavCleanPathAlgorithm);
		low_battery_charge_ = true;
		// For entering checking switch.
		ev.charge_detect = 0;

		brush.unblockMainBrushSlowOperation();
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
//	if (getPosition().toCell() == continue_point_.toCell())
//		return false;
//	else {
////		clean_map_.saveBlocks(action_i_ == ac_linear, sp_state == state_clean);
////		mapMark();
//		sp_action_.reset();
//		old_dir_ = iterate_point_->dir;
//		ROS_ERROR("old_dir_(%d)", old_dir_);
////		plan_path_ = *clean_path_algorithm_->shortestPath(getPosition(), continue_point_, ,old_dir_);
//		if (!plan_path_.empty()) {
//			iterate_point_ = plan_path_.begin();
//			ROS_ERROR("start_points_.dir(%d)", iterate_point_->dir);
////			plan_path_.pop_front();
//			displayCellPath(points_to_cells(plan_path_));
//			action_i_ = ac_linear;
//			genNextAction();
//			return true;
//		}
//		else
			return false;
//	}
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

bool CleanModeNav::checkingIfSwitchIsOn()
{
	return charger.isOnStub() && action_i_ == ac_back_from_charger && cliff.getStatus() == BLOCK_ALL;
}
