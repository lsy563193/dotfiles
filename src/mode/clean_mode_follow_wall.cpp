//
// Created by lsy563193 on 12/9/17.
//

#include <dev.h>
#include "robot.hpp"
#include "dev.h"
#include "mode.hpp"
Cells path_points;

CleanModeFollowWall::CleanModeFollowWall()
{
//	event_manager_register_handler(this);
//	event_manager_set_enable(true);
//	ROS_INFO("%s %d: Entering Follow wall mode\n=========================" , __FUNCTION__, __LINE__);
//	IMoveType::sp_mode_ = this;
//	diff_timer_ = WALL_FOLLOW_TIME;
	speaker.play(VOICE_CLEANING_WALL_FOLLOW, false);
	clean_path_algorithm_.reset(new WFCleanPathAlgorithm);
	go_home_path_algorithm_.reset();
	closed_count_limit_ = 1;
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
		speaker.play(VOICE_CLEANING_STOP);
		ROS_WARN("%s %d: Cliff all triggered. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}
*/
}

bool CleanModeFollowWall::mapMark(bool isMarkRobot) {
	clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
	PP_WARN();
	if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		clean_map_.setCleaned(pointsGenerateCells(passed_path_));
		clean_map_.setBlocks();
		ROS_ERROR("-------------------------------------------------------");
		auto start = *passed_path_.begin();
		passed_path_.erase(std::remove_if(passed_path_.begin(),passed_path_.end(),[&start](Point_t& it){
			return it.toCell() == start.toCell();
		}),passed_path_.end());
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
		ROS_ERROR("-------------------------------------------------------");
		clean_map_.setFollowWall(action_i_ == ac_follow_wall_left, passed_path_);
	}
	clean_map_.markRobot(CLEAN_MAP);
	clean_map_.print(CLEAN_MAP, getPosition().toCell().x, getPosition().toCell().y);
	passed_path_.clear();
	return false;
}

void CleanModeFollowWall::keyClean(bool state_now, bool state_last)
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

	beeper.play_for_command(VALID);
	vacuum.switchToNext();
	remote.reset();
}
void CleanModeFollowWall::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

bool CleanModeFollowWall::isIsolate() {
	BoundingBox2 bound{};

	clean_map_.getMapRange(CLEAN_MAP, &bound.min.x, &bound.max.x, &bound.min.y, &bound.max.y);

	auto target = bound.max + Cell_t{1, 1};
	bound.SetMinimum(bound.min - Cell_t{8, 8});
	bound.SetMaximum(bound.max + Cell_t{8, 8});

	auto path = clean_path_algorithm_->findShortestPath(clean_map_, getPosition().toCell(), target, 0, true, true,
																											bound.min, bound.max);
	return path.empty();
}

void CleanModeFollowWall::switchInStateInit() {
	PP_INFO();
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_trapped;
	sp_state->init();
	led.set_mode(LED_STEADY, LED_GREEN);
}

bool CleanModeFollowWall::updateActionInStateTrapped()
{
	ROS_INFO_FL();
	sp_action_.reset();// to mark in destructor
	old_dir_ = new_dir_;
	if (closed_count_ == 0) {
		if (generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
			new_dir_ = plan_path_.front().th;
			plan_path_.pop_front();
			pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
		}
	}
	else if (closed_count_ <= 3) {
		if (!isIsolate()) {
			if (generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
				new_dir_ = plan_path_.front().th;
				plan_path_.pop_front();
				pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
			}
		}
		else {
			ROS_WARN("%s,%d:follow clean finish", __func__, __LINE__);
			return false;
		}
	}else {
		return false;
	}

	PP_INFO();
	if (plan_path_.empty()) {
		ROS_WARN("%s,%d: mt_follow_wall_left", __FUNCTION__, __LINE__);
		action_i_ = ac_follow_wall_left;
//		action_i_ = ac_follow_wall_right;
		genNextAction();
		ROS_WARN("%s,%d: mt_follow_wall_left", __FUNCTION__, __LINE__);
	}
	else {
		action_i_ = ac_linear;
		genNextAction();
		ROS_WARN("%s,%d: ac_linear", __FUNCTION__, __LINE__);
	}
	return true;
}

bool CleanModeFollowWall::moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) {
	if (ACleanMode::moveTypeFollowWallIsFinish(p_mt))
	{
		ROS_WARN("moveTypeFollowWallIsFinish close!!!");
		return true;
	}

	return false;
}

void CleanModeFollowWall::switchInStateTrapped() {
	sp_state = state_go_home_point;
	ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_points_.size());
	speaker.play(VOICE_BACK_TO_CHARGER, true);
	go_home_path_algorithm_.reset();
	go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
	sp_state->init();
	action_i_ = ac_go_to_charger;
	genNextAction();
}

bool CleanModeFollowWall::generatePath(GridMap &map, const Point_t &curr, const int &last_dir, Points &targets)
{
	if (targets.empty()) {//fw ->linear
		auto curr = getPosition();
		fw_map.reset(CLEAN_MAP);
		auto angle = (closed_count_ != 0 && closed_count_ <= 3) ? -900 : 0;
		auto point = getPosition().addRadian(angle);
		targets.push_back(point);
		ROS_WARN("curr.th = %d, angle = %d,point.th(%d)", curr.th, angle,point.th);
		point = point.getRelative(8 * 1000, 0);
		targets.push_back(point);
		ROS_WARN("%s,%d: empty! point(%d, %d, %d)", __FUNCTION__, __LINE__,targets.back().x, targets.back().y, targets.back().th);
	}
	else//linear->fw
	{
		targets.clear();
		targets.push_back(getPosition());
		ROS_WARN("%s,%d: not empty! point(%d, %d, %d)", __FUNCTION__, __LINE__,targets.back().x, targets.back().y, targets.back().th);
	}
	return true;
}
