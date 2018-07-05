//
// Created by lsy563193 on 12/9/17.
//

#include <dev.h>
#include <event_manager.h>
#include "robot.hpp"
#include "dev.h"
#include "mode.hpp"
Cells path_points;

CleanModeFollowWall::CleanModeFollowWall()
{
	ROS_WARN("%s %d: Entering Follow wall mode\n=========================" , __FUNCTION__, __LINE__);
//	event_manager_register_handler(this);
//	event_manager_set_enable(true);
//	ROS_INFO("%s %d: Entering Follow wall mode\n=========================" , __FUNCTION__, __LINE__);
//	IMoveType::sp_mode_ = this;
//	diff_timer_ = WALL_FOLLOW_TIME;
	speaker.play(VOICE_CLEANING_WALL_FOLLOW, false);
	clean_path_algorithm_.reset(new WFCleanPathAlgorithm);
	closed_count_limit_ = 1;
	mode_i_ = cm_wall_follow;
	s_wifi.setWorkMode(cm_wall_follow);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
}

CleanModeFollowWall::~CleanModeFollowWall()
{
/*
	IMoveType::sp_mode_ = nullptr;
	event_manager_set_enable(false);

	if (ev.key_clean_pressed)
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Key clean pressed. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else if (ev.cliff_all_triggered)
	{
		speaker.play(VOICE_ERROR_LIFT_UP, false);
		speaker.play(VOICE_CLEANING_STOP_UNOFFICIAL);
		ROS_WARN("%s %d: Cliff all triggered. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}
*/
	ROS_WARN("%s %d: Exit.", __FUNCTION__, __LINE__);
}

bool CleanModeFollowWall::mapMark() {
	displayPointPath(passed_cell_path_);
//	PP_WARN();
//	ROS_INFO("%s %d: ", __FUNCTION__, __LINE__);
	if (isStateGoHomePoint())
	{
		setCleaned(*points_to_cells(passed_cell_path_));
		setBlocks(iterate_point_->dir);
	}
	else if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		setCleaned(*points_to_cells(passed_cell_path_));
		setBlocks(iterate_point_->dir);
		ROS_INFO("-------------------------------------------------------");
		auto start = *passed_cell_path_.begin();
//		passed_cell_path_.erase(std::remove_if(passed_cell_path_.begin(),passed_cell_path_.end(),[&start](Point_t& it){
//			return it.toCell() == start.toCell();
//		}),passed_cell_path_.end());
		displayPointPath(passed_cell_path_);
		ROS_INFO("-------------------------------------------------------");
		setFollowWall(clean_map_, action_i_ == ac_follow_wall_left, passed_cell_path_);
	}
	clean_map_.markRobot(getPosition().toCell());
	clean_map_.print(getPosition().toCell(), Cells{getPosition().toCell()});
//	passed_cell_path_.clear();
	return false;
}

bool CleanModeFollowWall::isExit()
{
	if (ev.remote_follow_wall)
	{
		ROS_WARN("%s %d: Exit for ev.remote_follow_wall.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (s_wifi.receiveFollowWall())
	{
		ROS_WARN("%s %d: Exit for wifi follow wall.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	return ACleanMode::isExit();
}

void CleanModeFollowWall::keyClean(bool state_now, bool state_last)
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

//
//void CleanModeFollowWall::overCurrentWheelLeft(bool state_now, bool state_last)
//{
//	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
//	ev.oc_wheel_left = true;
//}
//
//void CleanModeFollowWall::overCurrentWheelRight(bool state_now, bool state_last)
//{
//	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
//	ev.oc_wheel_right = true;
//}
//

void CleanModeFollowWall::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	if(water_tank.getStatus(WaterTank::operate_option::swing_motor)){
		beeper.beepForCommand(INVALID);
	}
	else if(!isStateExceptionResume())
	{
//		beeper.beepForCommand(VALID);
		vacuum.setForUserSetMaxMode(!vacuum.isUserSetMaxMode());
		ACleanMode::setVacuum();
	}
	remote.reset();
}

void CleanModeFollowWall::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

//	beeper.beepForCommand(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

void CleanModeFollowWall::remoteWallFollow(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote wall follow.", __FUNCTION__, __LINE__);

//	beeper.beepForCommand(VALID);
	wheel.stop();
	ev.remote_follow_wall = true;
	remote.reset();
}

void CleanModeFollowWall::batteryHome(bool state_now, bool state_last)
{
	if (!ev.battery_home && (isStateClean() || isStateFollowWall()))
	{
		ROS_WARN("%s %d: low battery, battery =\033[33m %dmv \033[0m", __FUNCTION__, __LINE__, battery.getVoltage());
		ev.battery_home = true;
	}
}

void CleanModeFollowWall::switchInStateInit() {
//	PP_INFO();
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_clean.get();
	is_first_wf_ = true;
	is_closed_ = true;
	closed_count_ = 0;
	isolate_count_ = 0;
	sp_state->init();
}

bool CleanModeFollowWall::updateActionInStateClean()
{
/*	passed_cell_path_.clear();
	ROS_ERROR("isolate_path_.size(%d)", isolate_path_.size());
	if (!isolate_path_.empty()) {
		for (auto &iter : isolate_path_) {
			auto offset_point = iter.getRelative(0, -0.167);
			ROS_INFO("iter(%lf, %lf), offset_point(%lf, %lf)", iter.x, iter.y, offset_point.x, offset_point.y);
			iter = offset_point;
		}
	}
	passed_cell_path_.assign(isolate_path_.begin(), isolate_path_.end());
	std::reverse(passed_cell_path_.begin(), passed_cell_path_.end());
	ROS_INFO("%s %d: reload passed path. size = %d", __FUNCTION__, __LINE__, passed_cell_path_.size());
//	fw_tmp_map.reset();*/

	if (is_small_area_closed_)
	{
		// Switch to follow wall state.
		beeper.debugBeep(VALID);
		is_trapped_ = true;
		closed_count_ = 0;
		action_i_ = ac_null;
		genNextAction();
		return action_i_ != ac_null;
	}
	else
		return ACleanMode::updateActionInStateFollowWall();
}

void CleanModeFollowWall::switchInStateClean()
{
	if (is_trapped_)
	{
		sp_state = state_folllow_wall.get();
		sp_state->init();
		action_i_ = ac_null;
		genNextAction();
	}
	else if (trapped_closed_or_isolate)
	{
		sp_state = state_go_home_point.get();
		closed_count_limit_ = 2;
		closed_count_ = 0;
		isolate_count_ = 0;
		in_small_area_count_ = 0;
		clean_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_,&home_points_manager_, true));
		sp_state->init();
		action_i_ = ac_null;
		genNextAction();
	}
}

bool CleanModeFollowWall::updateActionInStateFollowWall()
{
	return ACleanMode::updateActionInStateFollowWall();
}

void CleanModeFollowWall::switchInStateFollowWall() {
	ACleanMode::switchInStateFollowWall();
}

bool CleanModeFollowWall::markMapInNewCell()
{
	if (!c_blocks.empty())
	{
		for (auto &&cost_block : c_blocks)
			clean_map_.setCost(cost_block.second.x, cost_block.second.y, cost_block.first);
		c_blocks.clear();
		clean_map_.markRobot(getPosition().toCell());
		if (ros::Time::now().toSec() - print_map_time_ > 5)
		{
			clean_map_.print(getPosition().toCell(), *points_to_cells(passed_cell_path_));
			Cells tmp_cell;
			fw_tmp_map.print(getPosition().toCell(), tmp_cell);
			print_map_time_ = ros::Time::now().toSec();
		}
	}
	else
		clean_map_.markRobot(getPosition().toCell());
}

bool CleanModeFollowWall::moveTypeNewCellIsFinish(IMoveType *p_mt)
{
	auto distance = updatePath();

	if (is_trapped_)
	{
		if (robot_timer.trapTimeout(ESCAPE_TRAPPED_TIME))
		{
			trapped_time_out_ = true;
			return true;
		}
		else if (!isHasEnterStateIsGoHomePoints() /*&& Check follow wall escape trapped logic.*/)
		{
			/*
			 * This variable is in case of the pulsate of the location, the close checking will find the
 			 * same pose(angle) before a specified dis before(SEARCH_BEFORE_DIS) the current pose
 			 */
			auto temp_mt = dynamic_cast<MoveTypeFollowWall *>(p_mt);
			if (temp_mt->isTrappedInSmallArea())
			{
				auto curr_pose = getPosition(SLAM_POSITION_SLAM_ANGLE);
				ROS_INFO("curr_pose.Distance(small_area_trapped_pose_) = %f",
						 curr_pose.Distance(small_area_trapped_pose_));
				if (curr_pose.Distance(small_area_trapped_pose_) > 1)
				{//1 metre
					is_small_area_closed_ = false;
					in_small_area_count_ = 0;
					out_of_trapped_ = true;
					is_trapped_ = false;
					beeper.debugBeep(VALID);
					ROS_WARN("%s %d: Out of trapped in small area.", __FUNCTION__, __LINE__);
					return true;
				}
			}
		}

		if (pathAlgorithmCheckOutOfTrapped(p_mt))
			return true;
	}
	else
		markMapInNewCell();

	return checkClosed(p_mt, distance);
}

bool CleanModeFollowWall::moveTypeRealTimeIsFinish(IMoveType *p_mt)
{
	if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		Points ins_path{};//instantaneous path
		ins_path.push_back(getPosition());
		setFollowWall(clean_map_, action_i_ == ac_follow_wall_left, ins_path);
	}

	return ACleanMode::moveTypeRealTimeIsFinish(p_mt);
}

