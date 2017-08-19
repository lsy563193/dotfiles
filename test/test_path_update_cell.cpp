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

static void linear_mark(const Cell_t &start, const Cell_t &stop,CellState state)
{
	for (auto x = start.X-1; x <= stop.X + 1; x++)
	{
		for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; dy++)
			map_set_cell(MAP, cell_to_count(x), cell_to_count(stop.Y + dy), state);
	}
}

static void linear_mark_block(const Cell_t &start, const Cell_t &stop)
{
	for (auto x = start.X-1; x <= stop.X + 1; x++)
	{
			map_set_cell(MAP, cell_to_count(x), cell_to_count(start.Y+2), BLOCKED_OBS);
	}

}

static void linear_mark_block_y(const int16_t &x, const int16_t &start, const int16_t &stop)
{
	for (auto y = start; y <= stop; y++)
	{
		map_set_cell(MAP, cell_to_count(x), cell_to_count(y), BLOCKED_OBS);
	}

}

static void linear_mark_block_x(const int16_t &y, const int16_t &start, const int16_t &stop,CellState state)
{
	for (auto x = start; x <= stop; x++)
	{
		map_set_cell(MAP, cell_to_count(x), cell_to_count(y), state);
	}

}
extern Cell_t g_cell_history[5];

void case_2() {
	map_set_block({-20,-20},{20,20},CLEANED);
	map_set_block_with_bound({-5,-5},{5,5},CLEANED);
	map_set_block_with_bound({-10,-5},{-7,5},UNCLEAN);
	map_set_block({-11,-6},{6,-6},CLEANED);

	map_set_block({-2,0},{3,0},UNCLEAN);
}
bool move_to(const Cell_t& curr, PPTargetType& path)
{
	ROS_INFO("%s,%d", __FUNCTION__,__LINE__);
	std::vector<Cell_t> cells;
	std::copy(path.cells.begin(),path.cells.end(),cells.begin());
  for (const auto& cell :  cells)
	{
		ROS_INFO("cell(%d,%d)",cell.X,cell.Y);
		for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; dy++)
		{
			auto y = cell.Y + dy;
			map_set_cell(MAP, cell_to_count(cell.X), cell_to_count(y), CLEANED);
		}
	}
}
int main(int argc, char **argv)
{

	map_init(MAP);

	Cell_t curr{0,0};
	auto point = map_cell_to_point(curr);
	map_set_position(point.X,point.Y);

//    case_1();
	case_2();
	map_mark_robot(MAP);
//	map_set_block({2,-1},{2,1},BLOCKED_BUMPER);
//	map_set_block({-2,-1},{-2,1},BLOCKED_BUMPER);
//	map_set_block({-2,-1},{-2,1},BLOCKED_BUMPER);
	PPTargetType cleaning_path;
//	path_target(curr, cleaning_path);
	debug_map(MAP, cleaning_path.target.X, cleaning_path.target.Y);
	auto is_found = false;
//	do {
		path_full(curr, cleaning_path, is_found);
		if(is_found)
			move_to(curr, cleaning_path);
//	}while(is_found);

//	debug_map(MAP, cleaning_path.target.X, cleaning_path.target.Y);

	return 0;
}
