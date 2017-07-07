//
// Created by lsy563193 on 7/7/17.
//

#include "ros/ros.h"
#include <mathematics.h>
#include <movement.h>
#include <core_move.h>
#include "move_type.h"

bool move_type_update(Point32_t *next_point, Point32_t target_point, uint16_t dir) {
	if(get_clean_mode() == Clean_Mode_WallFollow){
		return g_cm_move_type == CM_FOLLOW_LEFT_WALL || g_cm_move_type == CM_FOLLOW_RIGHT_WALL ;
	}else if(get_clean_mode() == Clean_Mode_Spot){
		return false;
	}
	g_cm_move_type = CM_LINEARMOVE;
//	ROS_ERROR("curr(%d,%d),next(%d,%d),target(%d,%d)",map_get_x_cell(), map_get_y_cell(),
//						                                        count_to_cell(next_point->X), count_to_cell(next_point->Y),
//																										count_to_cell(target_point.X), count_to_cell(target_point.Y));
//	ROS_ERROR("curr_point_y(%d),next_point_y(%d),dir(%d),should_follow_wall(%d)",map_get_y_count(), next_point->Y, dir, g_should_follow_wall);
	if (!IS_X_AXIS(dir) || !g_should_follow_wall ||next_point->Y == map_get_y_count()) {

		return false;
	}

	auto delta_y = count_to_cell(next_point->Y) - map_get_y_cell();
//	ROS_ERROR("curr_y(%d),next_y(%d),delta_y(%d),dir(%d)",map_get_y_cell(), count_to_cell(next_point->Y), delta_y, dir);

	if ( delta_y != 0 && std::abs(delta_y) <= 2 ) {
		g_cm_move_type = (dir == POS_X) ^ (delta_y > 0) ? CM_FOLLOW_LEFT_WALL: CM_FOLLOW_RIGHT_WALL;
		ROS_INFO("follow wall to new line, 2_left_3_right(%d)",g_cm_move_type);
	} else if(delta_y == 0){
//		ROS_ERROR("don't need to go to new line. curr_x(%d)", count_to_cell(next_point->X));
		if (!(count_to_cell(next_point->X) == SHRT_MAX || count_to_cell(next_point->X) == SHRT_MIN)) {
			delta_y = count_to_cell(target_point.Y) - map_get_y_cell();
			if (delta_y != 0 && std::abs(delta_y) <= 2) {
				next_point->Y = target_point.Y;
				g_cm_move_type = ((dir == POS_X ^ delta_y > 0 ) ? CM_FOLLOW_LEFT_WALL : CM_FOLLOW_RIGHT_WALL);
//				ROS_ERROR("don't need to go to new line. curr_x(%d)", count_to_cell(next_point->X));
				ROS_INFO("follow wall to new line, 2_left_3_right(%d)",g_cm_move_type);
			}
		}
	}
	return (g_cm_move_type == CM_FOLLOW_LEFT_WALL) || (g_cm_move_type == CM_FOLLOW_RIGHT_WALL);
}
