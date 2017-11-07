//
// Created by lsy563193 on 17-9-27.
//

#include "ros/ros.h"
#include <mathematics.h>
#include <movement.h>
#include <clean_mode.h>
#include <event_manager.h>
#include <path_planning.h>
#include "move_type.h"
#include "spot.h"

//CMMoveType g_cm_move_type;
static uint8_t g_cleaning_mode = 0;

//bool mt_is_left()
//{
//	return g_cm_move_type == CM_FOLLOW_LEFT_WALL;
//}
//
//bool mt_is_right()
//{
//	return g_cm_move_type == CM_FOLLOW_RIGHT_WALL;
//}
//
//bool mt_is_follow_wall()
//{
//	return g_cm_move_type == CM_FOLLOW_LEFT_WALL || g_cm_move_type == CM_FOLLOW_RIGHT_WALL;
//}
//
//bool mt_is_linear()
//{
//	return g_cm_move_type == CM_LINEARMOVE;
//}
//
bool cm_is_navigation()
{
	return g_cleaning_mode == Clean_Mode_Navigation;
}

bool cm_is_follow_wall()
{
	return g_cleaning_mode == Clean_Mode_WallFollow;
}

bool cm_is_exploration()
{
	return g_cleaning_mode == Clean_Mode_Exploration;
}

bool cm_is_spot()
{
	return g_cleaning_mode == Clean_Mode_Spot;
}


bool cm_is_go_home()
{
	return g_cleaning_mode == Clean_Mode_GoHome;
}

void cm_set(uint8_t mode)
{
	g_cleaning_mode = mode;
}

uint8_t cm_get(void)
{
	return g_cleaning_mode;
}
