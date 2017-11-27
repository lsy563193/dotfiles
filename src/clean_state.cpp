//
// Created by root on 11/7/17.
//
#include <ros/ros.h>
#include <core_move.h>
#include <clean_mode.h>
#include "clean_state.h"
#include "clean_mode.h"


CleanState cs;
bool CleanState::init()
{
	if(cm_is_go_charger())
		cs_ = CS_GO_CHANGER;
	if(cm_is_exploration())
		cs_ = CS_EXPLORATION;
	else
		cs_ = CS_CLEAN;
}

bool CleanState::is_open_gyro()
{
	return cs_ == CS_OPEN_GYRO;
}

bool CleanState::is_back_from_charger()
{
	return cs_ == CS_OPEN_GYRO;
}

bool CleanState::is_open_laser()
{
	return cs_ == CS_OPEN_LASER;
}

bool CleanState::is_align()
{
	return cs_ == CS_ALIGN;
}

bool CleanState::is_open_slam()
{
	return cs_ == CS_OPEN_SLAM;
}

bool CleanState::is_go_home_point()
{
	return cs_ == CS_GO_HOME_POINT;
}

bool CleanState::is_go_charger()
{
	return cs_ == CS_GO_CHANGER;
}

bool CleanState::is_exploration()
{
	return cs_ == CS_EXPLORATION;
}

bool CleanState::is_going_home()
{
	return cs_ == CS_GO_HOME_POINT || cs_ == CS_GO_CHANGER /*||CS_EXPLORATION*/;
}

bool CleanState::is_clean()
{
	return cs_ == CS_CLEAN;
}

bool CleanState::is_tmp_spot()
{
	return cs_ == CS_TMP_SPOT;
}

bool CleanState::is_trapped()
{
	return cs_ == CS_TRAPPED;
}

bool CleanState::is_self_check()
{
	return cs_ == CS_SELF_CHECK;
}

void CleanState::set(int state)
{
//	ROS_INFO("%s,%d:" __FUNCTION__,__LINE__);
	cs_ = state;
	cm_apply_cs();
}

int CleanState::get(void)
{
	return cs_;
}
