//
// Created by root on 11/7/17.
//
#include <ros/ros.h>
#include <core_move.h>
#include <clean_mode.h>
#include <dev.h>
#include <pp.h>
#include "clean_state.h"
#include "clean_mode.h"


int CleanStateBase::cs_=0;//clean state
//CleanStateBase* p_cs;
CleanStateManage cs;
bool CleanStateBase::init()
{
	if(cm_is_go_charger())
		cs_ = CS_GO_CHANGER;
	if(cm_is_exploration())
		cs_ = CS_EXPLORATION;
	else
		cs_ = CS_CLEAN;
}

bool CleanStateBase::is_open_gyro()
{
	return cs_ == CS_OPEN_GYRO;
}

bool CleanStateBase::is_back_from_charger()
{
	return cs_ == CS_BACK_FROM_CHARGER;
}

bool CleanStateBase::is_open_lidar()
{
	return cs_ == CS_OPEN_LIDAR;
}

bool CleanStateBase::is_align()
{
	return cs_ == CS_ALIGN;
}

bool CleanStateBase::is_open_slam()
{
	return cs_ == CS_OPEN_SLAM;
}

bool CleanStateBase::is_go_home_point()
{
	return cs_ == CS_GO_HOME_POINT;
}

bool CleanStateBase::is_go_charger()
{
	return cs_ == CS_GO_CHANGER;
}

bool CleanStateBase::is_exploration()
{
	return cs_ == CS_EXPLORATION;
}

bool CleanStateBase::is_going_home()
{
	return cs_ == CS_GO_HOME_POINT || cs_ == CS_GO_CHANGER /*||CS_EXPLORATION*/;
}

bool CleanStateBase::is_clean()
{
	return cs_ == CS_CLEAN;
}

bool CleanStateBase::is_tmp_spot()
{
	return cs_ == CS_TMP_SPOT;
}

bool CleanStateBase::is_trapped()
{
	return cs_ == CS_TRAPPED;
}

bool CleanStateBase::is_self_check()
{
	return cs_ == CS_SELF_CHECK;
}

void CleanStateBase::setNext(int state)
{
	ROS_INFO("%s %d:Clean mode set to %d.", __FUNCTION__,__LINE__, state);
	cs_ = state;
	cm_apply_cs(state);
}

int CleanStateBase::get(void)
{
	return cs_;
}

bool CleanStateBase::isTrapped() {
	int escape_cleaned_count = 0;
	auto curr = cost_map.getCurrCell();
	PPTargetType path{{0,0,0}};
	bool is_found = path_dijkstra(curr, path.back(), escape_cleaned_count);
	if(is_found)
		return false;
	auto map_cleand_count = cost_map.getCleanedArea();
	double clean_proportion = 0.0;
	clean_proportion = (double) escape_cleaned_count / (double) map_cleand_count;
	ROS_WARN("%s %d: escape escape_cleaned_count(%d)!!", __FUNCTION__, __LINE__, escape_cleaned_count);
	ROS_WARN("%s %d: escape map_cleand_count(%d)!!", __FUNCTION__, __LINE__, map_cleand_count);
	ROS_WARN("%s %d: clean_proportion(%f) ,when prop < 0,8 is trapped ", __FUNCTION__, __LINE__, clean_proportion);
	return (clean_proportion < 0.8 /*|| path_escape_trapped(curr) <= 0*/);
}

//---------------------------
bool TrappedCS::cs_next(const Cell_t& start, PPTargetType& path)
{
	if (!isTrapped())
			setNext(CS_CLEAN);
	return true;
}

void TrappedCS::setting() {
		g_wf_start_timer = time(NULL);
		g_wf_diff_timer = ESCAPE_TRAPPED_TIME;
		led.set_mode(LED_FLASH, LED_GREEN, 300);
		mt.set(MT_FOLLOW_LEFT_WALL);
}

//---------------------------
bool CleanCS::cs_next(const Cell_t& start, PPTargetType& path)
{
	if (!path_next(start, path))
	{
		ROS_INFO("%s%d:", __FUNCTION__, __LINE__);
		if (isTrapped())
		{
			setNext(CS_TRAPPED);
			path.push_back(g_virtual_target);
			return true;
		}
		else
		{
			setNext(CS_GO_HOME_POINT);
			return true;
		}
	}else{
		mt.update(start,path);
	}
	return false;
}

void CleanCS::setting()
{
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_GREEN);
}

//---------------------------
bool TmpSpotCS::cs_next(const Cell_t& start, PPTargetType& path)
{
	return path_next_spot(start, path);
}
void TmpSpotCS::setting() {
	if (SpotMovement::instance()->getSpotType() == NO_SPOT) {
		ROS_INFO("%s %d: Entering temp spot during navigation.", __FUNCTION__, __LINE__);
		Cell_t curr_cell = cost_map.getCurrCell();
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

//---------------------------
bool GoHomePointCS::cs_next(const Cell_t& start, PPTargetType& path) {
	if (start == g_home_point) {
		if (g_home_point != g_zero_home || cm_turn_and_check_charger_signal())
		{
			setNext(CS_GO_CHANGER);
			return true;
		}
	}
	else if (path_get_home_point_target(start, path))
	{
		setNext(CS_EXPLORATION);
		return true;
	}

	return false;
}

void GoHomePointCS::setting(void)
{
		cs_work_motor();
	wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
		if (ev.remote_home || cm_is_go_charger())
			led.set_mode(LED_STEADY, LED_ORANGE);

		// Special handling for wall follow mode_.
		if (cm_is_follow_wall()) {
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle); //For wall follow mode_.
			cost_map.updatePosition();
			//wf_mark_home_point();
			cost_map.reset(MAP);
			cost_map.merge(slam_cost_map, false, false, true, false, false);
			cost_map.markRobot(MAP);//note: To clear the obstacles before go home, please don't remove it!
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
		mt.set(MT_LINEARMOVE);
}
//---------------------------
bool ExplorationCS::cs_next(const Cell_t& start, PPTargetType& path) {
	if (g_have_seen_charger) {
		setNext(CS_GO_CHANGER);
		return true;
	}
	else if (path_next_nav(start, path))
		return false;
	return false;
}

void ExplorationCS::setting(void)
{
	mt.set(MT_LINEARMOVE);
	g_wf_reach_count = 0;
	led.set_mode(LED_STEADY, LED_ORANGE);
}

//---------------------------
bool SelfCheckCS::cs_next(const Cell_t& start, PPTargetType& path)
{
	return false;
}

void SelfCheckCS::setting(void)
{
//	tilt.enable(false); //disable tilt detect
	led.set_mode(LED_STEADY, LED_ORANGE);
}

//---------------------------
bool GoChargeCS::cs_next(const Cell_t& start, PPTargetType& path)
{
	return false;
}

void GoChargeCS::setting(void)
{
//	tilt.enable(false); //disable tilt detect
	led.set_mode(LED_STEADY, LED_ORANGE);
}

CleanStateManage::CleanStateManage()
{
	vss_[CS_CLEAN] = new CleanCS();
	vss_[CS_GO_HOME_POINT] = new GoHomePointCS();
	vss_[CS_GO_CHANGER] = new GoChargeCS();
	vss_[CS_TMP_SPOT] = new TmpSpotCS();
	vss_[CS_TRAPPED] = new TrappedCS();
	vss_[CS_EXPLORATION] = new ExplorationCS();
};

bool CleanStateManage::cs_next(const Cell_t& start, PPTargetType& path)
{
	vss_[cs_]->cs_next(start,path);
}

void CleanStateManage::setting(void)
{
//	tilt.enable(false); //disable tilt detect
//	led.set_mode(LED_STEADY, LED_ORANGE);
}
