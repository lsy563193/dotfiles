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
//#include "a_star.h"
#include "map.h"
void updateMap(const Cell_t &start_cell, const Cell_t &next_cell){
	if(start_cell.Y == next_cell.Y)
	{//followction Back x axis
		auto start = std::min(start_cell.X, next_cell.X);
		auto end = std::max(start_cell.X, next_cell.X);
		for(auto i = start; i<=end+1; i++){
			for(auto dy = -1; dy<=1; dy++)
				Map_SetCell(MAP, cellToCount(i), cellToCount(next_cell.Y + dy), CLEANED);
		}
	}else
	{
		auto start = std::min(start_cell.Y, next_cell.Y);
		auto end = std::max(start_cell.Y, next_cell.Y);
		for(auto i = start; i<=end+1; i++)
			Map_SetCell(MAP, Map_GetXCount(), i, CLEANED);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pp");
	ros::NodeHandle	nh_private("~");

	robot	robot_obj;

	int32_t x=0,y=0;
	Map_Initialize();
	path_planning_initialize(&x, &y);//init pathplan

	Cell_t start{0,0};
	Cell_t stop{10,0};
	extern PositionType g_pos_history[5];
	extern uint16_t g_last_dir;
	g_last_dir=1800;
	g_pos_history[1].x = 10;
	g_pos_history[1].y = 0;
	g_pos_history[0].x = 0;
	g_pos_history[0].y = 0;
	updateMap(start,stop);
	Map_SetCell(MAP, cellToCount(0), cellToCount(-1), BLOCKED);
	Map_SetCell(MAP, cellToCount(0), cellToCount(1), BLOCKED);

	Map_SetCell(MAP, cellToCount(5), cellToCount(-1), BLOCKED);
	Map_SetCell(MAP, cellToCount(5), cellToCount(1), BLOCKED);

	Map_SetCell(MAP, cellToCount(10), cellToCount(-1), BLOCKED);
	Map_SetCell(MAP, cellToCount(10), cellToCount(1), BLOCKED);

//	Map_SetCell(MAP, cellToCount(10), cellToCount(-2), UNCLEAN);
//	Map_SetCell(MAP, cellToCount(10), cellToCount(2), UNCLEAN);

	Map_SetCell(MAP, cellToCount(12), cellToCount(0), BLOCKED);
	Map_SetCell(MAP, cellToCount(12), cellToCount(1), BLOCKED);
	Map_SetCell(MAP, cellToCount(12), cellToCount(2), BLOCKED);
	Map_SetCell(MAP, cellToCount(-2), cellToCount(0), BLOCKED);
	Map_SetCell(MAP, cellToCount(-2), cellToCount(1), BLOCKED);
	Map_SetCell(MAP, cellToCount(-2), cellToCount(2), BLOCKED);
	path_update_cells();
	debug_map(MAP,countToCell(stop.X),countToCell(stop.Y));
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
//	}
	return 0;
}
