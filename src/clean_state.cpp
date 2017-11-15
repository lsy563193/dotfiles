//
// Created by root on 11/7/17.
//
#include <ros/ros.h>
#include <core_move.h>
#include <clean_mode.h>
#include "clean_state.h"
#include "clean_mode.h"

static int cs = 0;//clean state

bool cs_init()
{
	if(cm_is_go_charger())
		cs = CS_GO_CHANGER;
	if(cm_is_exploration())
		cs = CS_EXPLORATION;
	else
		cs = CS_CLEAN;
}
bool cs_is_go_home_point()
{
	return cs == CS_GO_HOME_POINT;
}
bool cs_is_go_charger()
{
	return cs == CS_GO_CHANGER;
}

bool cs_is_exploration()
{
	return cs == CS_EXPLORATION;
}

bool cs_is_going_home()
{
	return cs == CS_GO_HOME_POINT || cs == CS_GO_CHANGER /*||CS_EXPLORATION*/;
}

bool cs_is_clean()
{
	return cs == CS_CLEAN;
}

bool cs_is_tmp_spot()
{
	return cs == CS_TMP_SPOT;
}

bool cs_is_trapped()
{
	return cs == CS_TRAPPED;
}

void cs_set(int state)
{
//	ROS_INFO("%s,%d:" __FUNCTION__,__LINE__);
	cs = state;
	cm_apply_cs(state);
}

int cs_get(void)
{
	return cs;
}
