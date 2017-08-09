#include <iostream>
#include <boost/assert.hpp>
#include <map>
#include "vector"
#include <mathematics.h>
#include <list>
#include <path_planning.h>
#include <movement.h>
#include <ros/ros.h>
#include <core_move.h>
#include <gyro.h>
#include <robot.hpp>
#include <move_type.h>

//#include "a_star.h"
#include "map.h"

extern Cell_t g_cell_history[5];
extern void mark_follow(Cell_t start);
int main(int argc, char **argv)
{
//	ros::init(argc, argv, "pp");
//	ros::NodeHandle	nh_private("~");

//	robot	robot_obj;
/*
	int32_t x=0,y=0;
	map_init();
	path_planning_initialize(&x, &y);//init pathplan

	Cell_t last{0,0};
	Cell_t curr{10,0};
	extern PositionType g_cell_history[5];
	extern uint16_t g_last_dir;
	g_last_dir=1800;
	g_cell_history[0].x = curr.X;
	g_cell_history[0].y = curr.Y;
	g_cell_history[1].x = last.X;
	g_cell_history[1].y = last.Y;

//	debug_map(MAP,curr.X,curr.Y);
	linear_mark_clean(last, curr);
	debug_map(MAP,curr.X,curr.Y);
	map_set_cell(MAP, cell_to_count(0), cell_to_count(-1), BLOCKED);
	map_set_cell(MAP, cell_to_count(0), cell_to_count(1), BLOCKED);

	map_set_cell(MAP, cell_to_count(5), cell_to_count(-1), BLOCKED);
	map_set_cell(MAP, cell_to_count(5), cell_to_count(1), BLOCKED);

	map_set_cell(MAP, cell_to_count(10), cell_to_count(-1), BLOCKED);
	map_set_cell(MAP, cell_to_count(10), cell_to_count(1), BLOCKED);

	map_set_cell(MAP, cell_to_count(12), cell_to_count(0), BLOCKED);
	map_set_cell(MAP, cell_to_count(12), cell_to_count(1), BLOCKED);
	map_set_cell(MAP, cell_to_count(12), cell_to_count(2), BLOCKED);
	map_set_cell(MAP, cell_to_count(-2), cell_to_count(0), BLOCKED);
	map_set_cell(MAP, cell_to_count(-2), cell_to_count(1), BLOCKED);
	map_set_cell(MAP, cell_to_count(-2), cell_to_count(2), BLOCKED);
	path_update_cells();
//	CM_update_position();

//	extern pp::x900sensor   sensor;

//	sensor.lbumper = true;
//	sensor.rbumper = true;
//	sensor.rbumper = true;
//	CM_update_map_bumper();

//	sensor.fcliff = true;
//	sensor.rcliff = true;
//	sensor.lcliff = true;
//	cm_update_map_cliff();
//	debug_map(MAP,countToCell(curr.X),countToCell(curr.Y));
//	Set_Clean_Mode(Clean_Mode_Spot);
//	while (ros::ok)
//	{
//		Cell_t begin_cell{Map_GetXCell(),Map_GetYCell()};
//		Point32_t next_point,targets;
//		auto state = path_next_point(&next_point, &targets);
//		if (state == 0)
//		{
//			ROS_INFO("go to continue cell");
//			break;
//		}
//		if (state == 1)
//		{
//			Map_SetCount(next_point.X, next_point.Y);
//
//			CM_update_map_cleaned(begin_cell, Map_PointToCell(next_point));
//			CM_update_map();
//			debug_map(MAP,countToCell(next_point.X),countToCell(next_point.Y));
//			sleep(3);
//		} else if (state == 2)
//		{
//			ROS_INFO("go to escape mode");
//			break;
//		}
//	}*/

	extern uint16_t g_old_dir;
	map_init();

	Cell_t start{0,0};
	Cell_t stop{9,0};
	Cell_t next{9,0};
	auto point = map_cell_to_point(stop);
	map_set_position(point.X,point.Y);
//	mt_set(CM_FOLLOW_LEFT_WALL);
	g_old_dir = POS_X;
//	g_cell_history[2] = {10,0};
//	g_cell_history[1] = {-10,0};
//	g_cell_history[0] = {-10,2};
	auto curr = map_get_curr_cell();
	next = g_cell_history[0];
	path_lane_is_cleaned(curr, next);

	debug_map(MAP,stop.X,stop.Y);
	return 0;
}
