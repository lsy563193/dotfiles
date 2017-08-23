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

const Cell_t MIN_CELL{-MAP_SIZE,-MAP_SIZE};
const Cell_t MAX_CELL{ MAP_SIZE, MAP_SIZE};
extern Cell_t g_cell_history[5];

void case_2() {
	map_set_block_with_bound({-15,-15},{15,15},CLEANED);
	map_set_block_with_bound({-5,-5},{5,5},CLEANED);
	map_set_block_with_bound({-10,-5},{-7,5},UNCLEAN);
	map_set_block({-11,-6},{6,-6},CLEANED);

//	map_set_block({-2,0},{3,0},);
//	map_set_block({-2,0},{3,0},UNCLEAN);
}
bool move_to(const Cell_t& curr, PPTargetType& path,int dir)
{
//	std::vector<Cell_t> cells;
//	std::copy(path.cells.begin(),path.cells.end(),cells.begin());
  for (const auto& cell :  path.cells)
	{
		ROS_INFO("%s,%d: cell(%d,%d)",__FUNCTION__, __LINE__, cell.X,cell.Y);
		for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; dy++)
		{
			auto y = cell.Y + dy;
			map_set_cell(MAP, cell_to_count(cell.X), cell_to_count(y), CLEANED);
		}
	}
	Cell_t index[4];
	index[0] = { 2, 0};
	index[1] = {-2, 0};
	index[2] = { 0, 2};
	index[3] = { 0,-2};
  Cell_t  next = path.target + index[dir];
	if ( next > MIN_CELL || next < MAX_CELL) {
      next -= index[dir];
      next -= index[dir];
  }
	ROS_INFO("%s,%d next(%d,%d)", __FUNCTION__,__LINE__,next.X, next.Y);
    if(dir <2)
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; dy++)
		map_set_cell(MAP, cell_to_count(next.X), cell_to_count(next.Y+dy), BLOCKED_BUMPER);

	next -= index[dir];
	map_set_position(cell_to_count(next.X), cell_to_count(next.Y));
  map_mark_robot(MAP);
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
	PPTargetType path;
//	path_targets(curr, path);
//	path_find_all_targets();
	debug_map(MAP, path.target.X, path.target.Y);
//  Cell_t target;
//	path_dijkstra(curr,target);
	auto is_found = false;
	auto i = 0;
	auto dir =0;
//	do {
		is_found = path_full(curr, path);
		if(is_found){
//			move_to(curr, path, dir);
			debug_map(MAP, path.target.X, path.target.Y);
		}
//      i++;
//		if(i>=3)
//			break;
//	}while(is_found);

//  curr = map_get_curr_cell();
//	is_found = path_full(curr, path,dir);
//	if(is_found){
//		move_to(curr, path, dir);
//		map_set_position(path.target.X, path.target.Y);
//		debug_map(MAP, path.target.X, path.target.Y);
//	}

//    is_found = path_full(curr, path,dir);

	return 0;
}
