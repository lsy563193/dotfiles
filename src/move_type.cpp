//
// Created by lsy563193 on 7/7/17.
//

#include "ros/ros.h"
#include <mathematics.h>
#include <movement.h>
#include <core_move.h>
#include <event_manager.h>
#include <path_planning.h>
#include "move_type.h"
#include "spot.h"

CMMoveType g_cm_move_type;

bool mt_is_left()
{
	return g_cm_move_type == CM_FOLLOW_LEFT_WALL;
}

bool mt_is_right()
{
	return g_cm_move_type == CM_FOLLOW_RIGHT_WALL;
}

bool mt_is_follow_wall()
{
	return g_cm_move_type == CM_FOLLOW_LEFT_WALL || g_cm_move_type == CM_FOLLOW_RIGHT_WALL;
}

bool mt_is_linear()
{
	return g_cm_move_type == CM_LINEARMOVE;
}

bool mt_is_go_to_charger()
{
	return g_cm_move_type == CM_GO_TO_CHARGER;
}

CMMoveType mt_get()
{
	return g_cm_move_type ;
}

void mt_update(const Cell_t& curr, PPTargetType& path, uint16_t dir) {
	ROS_WARN("%s,%d: dir(%d),obs(%d),bumper(%d)",__FUNCTION__,__LINE__,dir,g_obs_triggered, g_bumper_triggered);
	if (!IS_X_AXIS(dir) || (g_obs_triggered == 0 && g_bumper_triggered == 0) || g_trapped_mode == 1 || path.cells.size()>3)
		return;

	if(g_tilt_triggered)
		return;
//	ROS_WARN("%s,%d,abs: delta_y(%d)",__FUNCTION__, __LINE__,std::abs(delta_y));
	ROS_ERROR("%s,%d: next delta_y == 0",__FUNCTION__,__LINE__);
	auto 	delta_y = g_target_cell.Y - curr.Y;
	if(delta_y == 0) return;
	auto 	delta_x = g_target_cell.X - curr.X;
	CMMoveType move_type_tmp = ((dir == POS_X ^ delta_y > 0 ) ? CM_FOLLOW_LEFT_WALL : CM_FOLLOW_RIGHT_WALL);
	if(std::abs(delta_x) > 1 && (IS_POS_AXIS(dir) ^ (delta_x < 0) ))
	{
		if( (move_type_tmp == CM_FOLLOW_LEFT_WALL &&  (g_obs_triggered == Status_Right_OBS || g_bumper_triggered == RightBumperTrig) ) ||
			  (move_type_tmp == CM_FOLLOW_RIGHT_WALL && (g_obs_triggered == Status_Left_OBS || g_bumper_triggered == LeftBumperTrig) ))
		{
			ROS_WARN("%s,%d: move_type_tmp same side with block(%d),g_obs_triggered(%d), g_bumper_triggered(%d)",__FUNCTION__,__LINE__, move_type_tmp,g_obs_triggered, g_bumper_triggered);
			return ;
		}
	}

//	ROS_ERROR("%s,%d: target delta_y(%d)",__FUNCTION__,__LINE__,delta_y);
	if (std::abs(delta_y) <= 2) {
		path.cells.push_front(g_target_cell);
		g_next_cell = g_target_cell;
		g_cm_move_type = move_type_tmp;
		ROS_INFO("\033[31m""%s,%d: target:, 2_left_3_right(%d)""\033[0m",__FUNCTION__, __LINE__, g_cm_move_type);
	}
}

void mt_set(CMMoveType mt)
{
	g_cm_move_type = mt;
}
