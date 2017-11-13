//
// Created by lsy563193 on 17-9-27.
//

#include "ros/ros.h"
#include <clean_mode.h>
#include "clean_mode.h"

//CMMoveType g_cm_move_type;
static uint8_t clean_mode = 0;
static uint8_t clean_state = 0;

bool cm_is_navigation()
{
	return clean_mode == Clean_Mode_Navigation;
}

bool cm_is_follow_wall()
{
	return clean_mode == Clean_Mode_WallFollow;
}

bool cm_is_exploration()
{
	return clean_mode == Clean_Mode_Exploration;
}

bool cm_is_spot()
{
	return clean_mode == Clean_Mode_Spot;
}

bool cm_is_go_charger()
{
	return clean_mode == Clean_Mode_Go_Charger;
}

void cm_set(uint8_t mode)
{
	clean_mode = mode;
}

uint8_t cm_get(void)
{
	return clean_mode;
}


