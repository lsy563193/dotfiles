//
// Created by pierre on 17-12-20.
//
#include <event_manager.h>
#include <dev.h>
#include <error.h>
#include <map.h>

#include "mode.hpp"

CleanModeExploration::CleanModeExploration()
{
	ROS_WARN("%s %d: Entering Exploration mode\n=========================" , __FUNCTION__, __LINE__);
	speaker.play(VOICE_GO_HOME_MODE, false);
	mode_i_ = cm_exploration;
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());
	go_home_path_algorithm_.reset(new GoHomePathAlgorithm());
	error_marker_.clear();
	clean_map_.mapInit();
	obs.control(OFF);
}

CleanModeExploration::~CleanModeExploration()
{
#if 0
	//	system("rm /var/volatile/tmp/map");
	FILE *f = fopen("/var/volatile/tmp/map", "w");
	if (f == nullptr)
		ROS_ERROR("%s %d: Open /var/volatile/tmp/map error.", __FUNCTION__, __LINE__);
	else
	{
		ROS_INFO("%s %d: Start writing data.", __FUNCTION__, __LINE__);
		fprintf(f, "Width: %d\n", slam_map.getWidth());
		fprintf(f, "Height: %d\n", slam_map.getHeight());
		fprintf(f, "Resolution: %f\n", slam_map.getResolution());
		fprintf(f, "Origin: x(%f), y(%f)\n", slam_map.getOriginX(), slam_map.getOriginY());
		auto size = slam_map.getData().size();
//		printf(" %s %d: data_index_size:%d.", __FUNCTION__, __LINE__, size);
		for (int index = 0; index < size; index++)
		{
			//ROS_INFO("slam_map_data s_index_:%d, data:%d", slam_map_data_index[s_index_], slam_map_data[slam_map_data_index[s_index_]]);
			if (slam_map.getData()[index] == 100) fprintf(f, "@");
			else if (slam_map.getData()[index] == -1) fprintf(f, ".");
			else if (slam_map.getData()[index] == 0) fprintf(f, "1");

			if (index % slam_map.getWidth() == 0)
				fprintf(f, "\n");

			if ((100 * index / size) % 10 == 0)
			{
				printf("\r %d0%%.", static_cast<int>(100 * index / size / 10));
			}
		}
		printf("\n");
		fclose(f);
		ROS_INFO("%s %d: Write data succeeded.", __FUNCTION__, __LINE__);
	}
#endif
	obs.control(ON);
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool CleanModeExploration::isExit()
{
	if (s_wifi.receiveHome())
	{
		ROS_WARN("%s %d: Exit for wifi home.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (s_wifi.receiveIdle())
	{
		ROS_WARN("%s %d: Exit for wifi idle.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	return ACleanMode::isExit();
}


bool CleanModeExploration::mapMark()
{
	clean_map_.merge(slam_grid_map, true, true, false, false, false, false);
	clean_map_.setCircleMarkers(getPosition(),10,CLEANED,error_marker_);
	resetErrorMarker();

	setBlocks(iterate_point_->dir);
	if(mark_robot_)
		clean_map_.markRobot(getPosition().toCell(), CLEAN_MAP);
//	passed_path_.clear();
	return false;
}

// event
void CleanModeExploration::keyClean(bool state_now, bool state_last) {
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);

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
	{
		ev.key_clean_pressed = true;
	}

	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void CleanModeExploration::remoteClean(bool state_now, bool state_last) {
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

//	beeper.beepForCommand(VALID);
	ev.key_clean_pressed = true;
	remote.reset();
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

	ev.charge_detect = charger.getChargeStatus();
}

void CleanModeExploration::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	if(water_tank.getStatus(WaterTank::operate_option::swing_motor)){
		beeper.beepForCommand(INVALID);
	}
	else if(isInitState() || isStateFollowWall() || isStateExploration() || isStateGoHomePoint() || isStateGoToCharger())
	{
//		beeper.beepForCommand(VALID);
		vacuum.setForUserSetMaxMode(!vacuum.isUserSetMaxMode());
		ACleanMode::setVacuum();
	}
	remote.reset();
}
/*void CleanModeExploration::printMapAndPath()
{
	clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
	clean_map_.print(CLEAN_MAP,getPosition().toCell().x,getPosition().toCell().y);
}*/

//state GoToCharger
void CleanModeExploration::switchInStateGoToCharger() {
	PP_INFO();
	if (ev.charge_detect && charger.isOnStub()) {
		sp_state = nullptr;
		return;
	}
	else
		sp_state = state_exploration.get();
	sp_state->init();
}

//state Init
void CleanModeExploration::switchInStateInit() {
	PP_INFO();
	action_i_ = ac_null;
	sp_action_ = nullptr;
	// Turn around and try receiving the rcon signal before exploration, it can increase the chance for going to charger.
	sp_state = state_go_to_charger.get();
	sp_state->init();
}

bool CleanModeExploration::updateActionInStateInit() {
	if (action_i_ == ac_null)
		action_i_ = ac_open_gyro_and_lidar;
	else if (action_i_ == ac_open_gyro_and_lidar) {
		boost::dynamic_pointer_cast<StateInit>(state_init)->initForExploration();
		action_i_ = ac_open_lidar;
	}
	else if (action_i_ == ac_open_lidar)
		action_i_ = ac_align;
	else if(action_i_ == ac_align)
	{
		auto curr = getPosition();
		go_home_path_algorithm_->updateStartPointRadian(curr.th);
		action_i_ = ac_open_slam;
	}
	else // action_open_slam
		return false;

	ACleanMode::genNextAction();
	return true;
}
//state GoHomePoint
void CleanModeExploration::switchInStateGoHomePoint() {
	PP_INFO();
	sp_state = nullptr;
}
/*

bool CleanModeExploration::moveTypeFollowWallIsFinish(IMoveType *p_move_type, bool is_new_cell) {
	if(action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		auto p_mt = dynamic_cast<MoveTypeFollowWall *>(p_move_type);
		return p_mt->isBlockCleared(clean_map_, passed_path_);
	}
	return false;
}
*/

bool CleanModeExploration::markMapInNewCell() {
	if(sp_state == state_folllow_wall.get())
	{
		mark_robot_ = false;
		mapMark();
		mark_robot_ = true;
	}
	else
		mapMark();
	return true;
}

void CleanModeExploration::resetErrorMarker() {
	//set unclean to map
	ROS_INFO("%s,%d,size:%d",__FUNCTION__,__LINE__,error_marker_.size());
	auto time = ros::Time::now().toSec();
	for(auto ite = error_marker_.begin();ite != error_marker_.end();ite++){
		if(error_marker_.empty())
			break;
		if(time - ite->time > 20){
			if(clean_map_.getCell(CLEAN_MAP,ite->x,ite->y) == CLEANED &&
					slam_grid_map.getCell(CLEAN_MAP,ite->x,ite->y) != SLAM_MAP_REACHABLE)
				clean_map_.setCell(CLEAN_MAP,ite->x,ite->y,UNCLEAN);
//			ROS_INFO("%s,%d,i:%d,size:%d",__FUNCTION__,__LINE__,i,error_marker_.size());
			error_marker_.erase(ite);
		}
	}
}

void CleanModeExploration::wifiSetWaterTank()
{
	if (isStateFollowWall())
		return;
	ACleanMode::wifiSetWaterTank();
}

void CleanModeExploration::setVacuum()
{
	if (isStateFollowWall())
		return;
	ACleanMode::setVacuum();
}


