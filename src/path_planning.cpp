/**
******************************************************************************
* @file        Shortest Path
* @author      ILife Team Patrick Chau
* @version     Ver 20160118
* @date        18-Jan-2016
* @brief       Function to find the shorest path to target
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
******************************************************************************
*/

/*
 * Path Planning for robot movement is a ZigZag way. When cleaning process starts,
 * robot will try to clean its left hand side first, a new lane must be cleaned
 * in both ends before getting the new cleaning target. Targets will be added to
 * the target list as soon as the robot is moving.
 *
 * After the cleaning process is done, robot will back to its starting point and
 * finishes the cleaning.
 */
#include <stdint.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <list>
#include <string>

#include <ros/ros.h>
#include <movement.h>
#include <mathematics.h>
#include <wall_follow_slam.h>
#include <move_type.h>
#include <path_planning.h>

#include "core_move.h"
#include "gyro.h"
#include "mathematics.h"
#include "map.h"
#include "path_planning.h"
#include "shortest_path.h"
#include "spot.h"
#include "robot.hpp"
#include "movement.h"

#include "wav.h"
#include "BoundingBox.h"
#include <numeric>

#define FINAL_COST (1000)
#define NO_TARGET_LEFT 0
#define TARGET_REACHED 0
#define TARGET_FOUND 1
#define TRAPPED 2

using namespace std;

list <PPTargetType> g_targets;

uint8_t	g_first_start = 0;

uint8_t g_direct_go = 0; /* Enable direct go when there is no obstcal in between current pos. & dest. */

// This list is for storing the position that robot sees the charger stub.

std::vector<Cell_t> g_homes;
std::vector<int> g_home_way_list;
std::vector<int>::iterator g_home_way_it;
Cell_t g_home;
bool g_is_switch_target = true;
//int8_t g_home_cnt = 0;// g_homes.size()*HOMEWAY_NUM-1 3/9, 2/4, 1/2
bool g_home_gen_rosmap = true;

Cell_t g_index[4]={{1,0},{-1,0},{0,1},{0,-1}};

const int16_t g_home_x = 0, g_home_y = 0;
Cell_t g_zero_home = {0,0};

// This is for the continue point for robot to go after charge.
Cell_t g_continue_cell;
const Cell_t MIN_CELL{-MAP_SIZE,-MAP_SIZE};
const Cell_t MAX_CELL{ MAP_SIZE, MAP_SIZE};

Cell_t g_cell_history[5];

uint16_t g_new_dir;
uint16_t g_old_dir;

int g_trapped_mode = 1;

//std::vector<Cell_t> g_homes = {{0,0},{0,0},{0,0}};

uint8_t g_trapped_cell_size = ESCAPE_TRAPPED_REF_CELL_SIZE;

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

extern int16_t g_wf_x_min, g_wf_x_max, g_wf_y_min, g_wf_y_max;

static std::vector<int>::iterator _gen_home_ways(int size, std::vector<int> &go_home_way_list) {
	ROS_INFO("%s,%d: go_home_way_list 1:                       2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 2: 5,      4,     3,     2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 3: 8,5,    7,4    6,3,   2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 4: 11,8,5, 10,7,4 9,6,3  2,1,0",__FUNCTION__, __LINE__);
	go_home_way_list.resize(size * HOMEWAY_NUM,0);
	std::iota(go_home_way_list.begin(), go_home_way_list.end(),0);
	std::sort(go_home_way_list.begin(), go_home_way_list.end(), [](int x,int y){
			return (x >= 3 && y >= 3) && (x % 3) < (y % 3);
	});
	std::reverse(go_home_way_list.begin(),go_home_way_list.end());
	std::copy(go_home_way_list.begin(), go_home_way_list.end(),std::ostream_iterator<int>(std::cout, " "));
	std::cout << std::endl;
	return go_home_way_list.begin();
}
//void path_planning_initialize(int32_t *x, int32_t *y)
void path_planning_initialize()
{
#if (ROBOT_SIZE == 5)

//	g_weight_cnt_threshold = 4;

#else

//	g_weight_cnt_threshold = 3;

#endif

	g_direct_go = 0;

	g_cell_history[0] = {0,0};
	g_old_dir = 0;
	g_new_dir = 0;

	/* Reset the poisition list. */
	for (auto i = 0; i < 3; i++) {
		g_cell_history[i] =  {int16_t(i+1), int16_t(i+1)};
	}

	/* Initialize the shortest path. */
	path_position_init(g_direct_go);

#ifndef ZONE_WALLFOLLOW

	/* Set the back as blocked, since robot will align the start angle with the wall. */
	map_set_cell(MAP, cell_to_count(-3), cell_to_count(0), BLOCKED_BUMPER);

#endif

	map_mark_robot(MAP);
}

/* Update the robot position history. */
void path_update_cell_history()
{
	g_cell_history[4] = g_cell_history[3];
	g_cell_history[3] = g_cell_history[2];
	g_cell_history[2] = g_cell_history[1];
	g_cell_history[1] = g_cell_history[0];

	g_cell_history[0]= map_get_curr_cell();
//	g_cell_history[0].dir = path_get_robot_direction();
}

uint16_t path_get_robot_direction()
{
	return g_new_dir;
}

void path_get_range(uint8_t id, int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max)
{
	if (id == MAP || id == SPMAP) {
		*x_range_min = g_x_min - (abs(g_x_min - g_x_max) <= 3 ? 3 : 1);
		*x_range_max = g_x_max + (abs(g_x_min - g_x_max) <= 3 ? 3 : 1);
		*y_range_min = g_y_min - (abs(g_y_min - g_y_max) <= 3? 3 : 1);
		*y_range_max = g_y_max + (abs(g_y_min - g_y_max) <= 3 ? 3 : 1);
	} else if(id == WFMAP) {
		*x_range_min = g_wf_x_min - (abs(g_wf_x_min - g_wf_x_max) <= 3 ? 3 : 1);
		*x_range_max = g_wf_x_max + (abs(g_wf_x_min - g_wf_x_max) <= 3 ? 3 : 1);
		*y_range_min = g_wf_y_min - (abs(g_wf_y_min - g_wf_y_max) <= 3? 3 : 1);
		*y_range_max = g_wf_y_max + (abs(g_wf_y_min - g_wf_y_max) <= 3 ? 3 : 1);
	}

//	ROS_INFO("Get Range:\tx: %d - %d\ty: %d - %d\tx range: %d - %d\ty range: %d - %d",
//		g_x_min, g_x_max, g_y_min, g_y_max, *x_range_min, *x_range_max, *y_range_min, *y_range_max);
}

uint8_t is_a_block(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	CellState cs;

	cs = map_get_cell(MAP, x, y);
	if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY)
	//if (cs >= BLOCKED && cs <= BLOCKED_CLIFF)
		retval = 1;

	return retval;
}

uint8_t is_blocked_by_bumper(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i, j;
	CellState cs;

	/* Check the point by using the robot size. */
	for (i = ROBOT_RIGHT_OFFSET; retval == 0 && i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; retval == 0 && j <= ROBOT_LEFT_OFFSET; j++) {
			cs = map_get_cell(MAP, x + i, y + j);
			//if ((cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) && cs != BLOCKED_OBS) {
			if ((cs >= BLOCKED && cs <= BLOCKED_CLIFF) && cs != BLOCKED_OBS) {
				retval = 1;
			}
		}
	}

	return retval;
}

uint8_t is_block_accessible(int16_t x, int16_t y)
{
	uint8_t retval = 1;
	int16_t i, j;

	for (i = ROBOT_RIGHT_OFFSET; retval == 1 && i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; retval == 1 && j <= ROBOT_LEFT_OFFSET; j++) {
			if (is_a_block(x + i, y + j) == 1) {
				retval = 0;
			}
		}
	}

	return retval;
}

static bool is_block_cleanable(int16_t x, int16_t y)
{
	auto retval = is_block_unclean(x, y) && !is_block_blocked(x, y);
//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

int8_t is_block_cleaned_unblock(int16_t x, int16_t y)
{
	uint8_t cleaned = 0;
	int16_t i, j;

	for (i = ROBOT_RIGHT_OFFSET; i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; j <= ROBOT_LEFT_OFFSET; j++) {
			auto state = map_get_cell(MAP, x+i, y+j);
			if (state == CLEANED) {
				cleaned ++;
			} else if(is_block_blocked(x,y))
				return false;
		}
	}

	if (cleaned >= 8)
		return true;
	return false;
}

uint8_t is_block_unclean(int16_t x, int16_t y)
{
	uint8_t unclean_cnt = 0;
	for (int8_t i = (y + ROBOT_RIGHT_OFFSET); i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (map_get_cell(MAP, x, i) == UNCLEAN) {
			unclean_cnt++;
		}
	}
//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return unclean_cnt;
}

uint8_t is_block_boundary(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (map_get_cell(MAP, x, i) == BLOCKED_BOUNDARY) {
			retval = 1;
		}
	}

	return retval;
}

uint8_t is_block_blocked(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (is_a_block(x, i) == 1) {
			retval = 1;
		}
	}

//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

bool path_lane_is_cleaned(const Cell_t& curr, PPTargetType& path)
{
	int16_t i, is_found=0, min=SHRT_MAX, max=SHRT_MAX, min_stop=0, max_stop=0;
	uint16_t unclean_cnt_for_min = 0, unclean_cnt_for_max = 0;
	Cell_t tmp = curr;
#if INTERLACED_MOVE
	if(curr.Y % 2 != 0) {
		if (mt_is_follow_wall()) {
			auto dir = path.target.Y - g_cell_history[1].Y;//+2,-2
			auto step = curr.Y - g_cell_history[1].Y;
			auto is_local = (dir > 0) ? step <= 0 : step >= 0;
			tmp.Y  = (is_local) ? g_cell_history[1].Y : path.target.Y;
			ROS_ERROR("%s %d:follow_wall dir(%d),step(%d),tmp(%d,%d),his1(%d,%d)", __FUNCTION__, __LINE__, dir, step, tmp.X,
								tmp.Y, g_cell_history[1].X, g_cell_history[1].Y);
		}
		if (mt_is_linear() && IS_X_AXIS(g_new_dir)) {
			auto dir = path.target.Y - g_cell_history[1].Y;//+2,-2
			tmp.Y = g_cell_history[1].Y;
			ROS_ERROR("%s %d:mt_is_linear tmp(%d,%d),his1(%d,%d),curr(%d,%d)", __FUNCTION__, __LINE__, tmp.X, tmp.Y,
								g_cell_history[1].X, g_cell_history[1].Y, curr.X, curr.Y);
		}
	}
#endif
	ROS_INFO("%s %d: curr(%d,%d)", __FUNCTION__, __LINE__, curr.X, curr.Y);
	for (i = 1; (min_stop == 0 || max_stop == 0); i++)
	{
		//ROS_INFO("%s %d: i = %d, curr x - (i+1) = %d, curr x +i+1 = %d", __FUNCTION__, __LINE__, i, curr.X - (i + 1), curr.X + i + 1);
		if (curr.X - (i + 1) < -MAP_SIZE && curr.X + i + 1 > MAP_SIZE)
			break;

		/* Find the unclean area in the NEG_X direction of the lane. */
		if (min_stop == 0 && curr.X - (i + 1) >= -MAP_SIZE)
		{
			if (is_block_blocked(curr.X - i, curr.Y))
			{
				// Stop if the cells is blocked.
				min_stop = 1;
				// If block is near the unclean area, consider the block as unclean area.
				min = (i == min + 1) ? i : min;
				//ROS_INFO("%s %d: min stop:%d, (%d, %d) = %d", __FUNCTION__, __LINE__, min, curr.X - i, curr.Y, map_get_cell(MAP, curr.X - i, curr.Y));
			} else if (is_block_boundary(curr.X - i, curr.Y))
			{
				// Stop if reach the boundary.
				min_stop = 1;
			} else {
				auto unclean_cnt = is_block_unclean(curr.X - i, curr.Y);
				if (unclean_cnt)
				{
					// Find the furthest unclean cell.
					min = is_block_boundary(curr.X - (i + 1), curr.Y) ? min : i;
					unclean_cnt_for_min += unclean_cnt;
					//ROS_INFO("%s %d: min: %d", __FUNCTION__, __LINE__, min);
				}
			}
		}

		/* Find the unclean area in the POS_X direction of the lane. */
		if (max_stop == 0 && curr.X + i + 1 <= MAP_SIZE)
		{
			if (is_block_blocked(curr.X + i, curr.Y))
			{
				// Stop if the cells is blocked.
				max_stop = 1;
				// If block is near the unclean area, consider the block as unclean area.
				max = (i == max + 1) ? i : max;
				//ROS_INFO("%s %d: max stop:%d, (%d, %d) = %d", __FUNCTION__, __LINE__, max, curr.X + i, curr.Y, map_get_cell(MAP, curr.X + i, curr.Y));
			} else if (is_block_boundary(curr.X + i, curr.Y))
			{
				// Stop if reach the boundary.
				max_stop = 1;
			} else {
				auto unclean_cnt = is_block_unclean(curr.X + i, curr.Y);
				if (unclean_cnt)
				{
					// Find the furthest unclean cell.
					max = is_block_boundary(curr.X + i + 1, curr.Y) ? max : i;
					unclean_cnt_for_max += unclean_cnt;
					//ROS_INFO("%s %d: max: %d", __FUNCTION__, __LINE__, max);
				}
			}
		}
	}

	ROS_WARN("%s %d: min: %d\tmax: %d", __FUNCTION__, __LINE__, curr.X - min, curr.X + max);

	// If unclean area is too small, do not clean that area.
	if (max != SHRT_MAX && unclean_cnt_for_max < 3)
	{
		ROS_WARN("%s %d: Cleaning area for max direction is too small, drop it.", __FUNCTION__, __LINE__);
		max = SHRT_MAX;
	}
	if (min != SHRT_MAX && unclean_cnt_for_min < 3)
	{
		ROS_WARN("%s %d: Cleaning area for min direction is too small, drop it.", __FUNCTION__, __LINE__);
		min = SHRT_MAX;
	}

	if (min != SHRT_MAX && max != SHRT_MAX)
	{
		/*
		 * Both ends are not cleaned.
		 * If the number of cells to clean are the same of both ends, choose either one base the
		 * previous robot g_cell_history. Otherwise, move to the end that have more unclean cells.
		 */
		if (min > max)
			tmp.X += max;
		else if (min < max)
			tmp.X -= min;
		else
		{
			if (g_cell_history[2].Y == g_cell_history[1].Y)
			{
				if (g_cell_history[2].X > g_cell_history[1].X)
					tmp.X -= min;
				else if (g_cell_history[2].X < g_cell_history[1].X)
					tmp.X += max;
				else
				{
					if (g_cell_history[0].X <= g_cell_history[1].X)
						tmp.X += max;
					else
						tmp.X -= min;
				}
			} else if (g_cell_history[0].Y == g_cell_history[1].Y)
			{
				if (g_cell_history[0].X >= g_cell_history[1].X)
					tmp.X += max;
				else
					tmp.X -= min;
			} else
				tmp.X += max;
		}
		is_found = 2;
	} else if (min != SHRT_MAX)
	{
		/* Only the NEG_X end is not cleaned. */
		tmp.X -= min;
		is_found = 1;
	} else if (max != SHRT_MAX)
	{
		/* Only the POS_X end is not cleaned. */
		tmp.X += max;
		is_found = 1;
	}

#if !INTERLACED_MOVE
	if (is_found == 1)
	{
		if (g_cell_history[0] == g_cell_history[1] && g_cell_history[0] == g_cell_history[2])
//			 Duplicated cleaning.
			is_found = 0;
	}
#endif

	ROS_INFO("%s %d: is_found = %d, target(%d, %d).", __FUNCTION__, __LINE__, is_found, tmp.X, tmp.Y);
	if (is_found)
	{
		int8_t dir = tmp.X > curr.X ? -1 : 1;
		path.target = tmp;
		path.cells.clear();
#if LINEAR_MOVE_WITH_PATH
		for (auto temp_cell = path.target; temp_cell.X != curr.X; temp_cell.X += dir)
			path.cells.push_front(temp_cell);
		// Displaying for debug.
		list<Cell_t> temp_path;
		temp_path.push_back(path.cells.front());
		temp_path.push_back(path.cells.back());
		path_display_path_points(temp_path);
#else
		path.cells.push_front(path.target);
		path.cells.push_front(curr);
		path_display_path_points(path.cells);
#endif
		return true;
	}
	else
		return false;
}

bool is_axis_access(const Cell_t &start, int i, Cell_t &target)
{
	auto is_found = false;
	for (auto tmp = start; std::abs(tmp.X) < MAP_SIZE && std::abs(tmp.Y) < MAP_SIZE && std::abs(tmp.Y- start.Y)  <=1; tmp += g_index[i]) {
//		ROS_INFO("%s, %d:tmp(%d,%d)", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		if(is_block_cleanable(tmp.X, tmp.Y)) {
			is_found = true;
			target = tmp-g_index[i];
		}else {
			break;
		}
	}
	return is_found;
}


void path_find_target(const Cell_t& curr, PPTargetType& path) {
	bool all_set;
	int16_t offset, passValue, nextPassValue, passSet, targetCost;
	CellState cs;

	map_reset(SPMAP);

	for (auto x = g_x_min; x <= g_x_max; ++x) {
		for (auto y = g_y_min; y <= g_y_max; ++y) {
			cs = map_get_cell(MAP, x, y);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				for (auto dx = ROBOT_RIGHT_OFFSET; dx <= ROBOT_LEFT_OFFSET; dx++) {
					for (auto dy = ROBOT_RIGHT_OFFSET; dy <= ROBOT_LEFT_OFFSET; dy++) {
						map_set_cell(SPMAP, x + dx, y + dy, COST_HIGH);
					}
				}
			}
		}
	}
/* Set the current robot position has the cost value of 1. */
	map_set_cell(SPMAP, (int32_t) curr.X, (int32_t) curr.Y, COST_1);

	offset = 0;
	passSet = 1;
	passValue = 1;
	nextPassValue = 2;
	while (passSet == 1) {
		offset++;
		passSet = 0;
		Cell_t cell;
		for (cell.X = curr.X - offset; cell.X <= curr.X + offset; cell.X++) {
			for (cell.Y = curr.Y - offset; cell.Y <= curr.Y + offset; cell.Y++) {
				if (map_get_cell(SPMAP, cell.X, cell.Y) == passValue) {
					for (auto i = 0; i < 4; i++) {
						auto neighbor = g_index[i] + cell;
						if (neighbor > MIN_CELL && neighbor < MAX_CELL)
							if (map_get_cell(SPMAP, neighbor.X, neighbor.Y) == COST_NO) {
								map_set_cell(SPMAP, neighbor.X, neighbor.Y, (CellState) nextPassValue);
								passSet = 1;
							}
					}
				}
			}
		}

		all_set = true;
		if (map_get_cell(SPMAP, path.target.X, path.target.Y) == COST_NO) {
			all_set = false;
		}
		if (all_set) {
			ROS_INFO("%s %d: all possible target are checked & reachable.", __FUNCTION__, __LINE__);
			passSet = 0;
		}

		passValue = nextPassValue;
		nextPassValue++;
		if (nextPassValue == COST_PATH)
			nextPassValue = 1;
	}

//	debug_map(SPMAP, 0, 0);

	if (map_get_cell(SPMAP, path.target.X, path.target.Y) == COST_NO ||
			map_get_cell(SPMAP, path.target.X, path.target.Y) == COST_HIGH) {
		return;
	}

	Cell_t tmp;
	Cell_t trace = path.target;
	while (trace != curr) {
		targetCost = map_get_cell(SPMAP, trace.X, trace.Y) - 1;

		if (targetCost == 0) {
			targetCost = COST_5;
		}

		tmp = trace;
    ROS_INFO("%s %d: tmp(%d,%d)", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		path.cells.push_back(tmp);

		for (auto i = 0; i < 4; i++) {
			auto neighbor = trace + g_index[i];
			if (neighbor > MIN_CELL && neighbor < MAX_CELL)
				if (map_get_cell(SPMAP, neighbor.X, neighbor.Y) == targetCost) {
					trace = neighbor;
					break;
				}
		}
		if (trace != tmp)
			continue;
	}
	if (trace.X == curr.X || trace.Y == curr.Y) {
		tmp = trace;
		path.cells.push_back(tmp);
	}

	path.cells.reverse();
}

bool path_full(const Cell_t& curr, PPTargetType& path)
{
	auto is_found = false;
	auto target = curr;
	auto tmp = curr;
  if(curr.Y % 2 != 0) {
		if (mt_is_follow_wall()) {
			auto dir = path.target.Y - g_cell_history[1].Y;//+2,-2
			auto step = curr.Y - g_cell_history[1].Y;
			auto is_local = (dir > 0) ? step <= 0 : step >= 0;
			tmp.Y  = (is_local) ? g_cell_history[1].Y : path.target.Y;
			ROS_ERROR("%s %d:follow_wall dir(%d),step(%d),tmp(%d,%d),his1(%d,%d)", __FUNCTION__, __LINE__, dir, step, tmp.X,
								tmp.Y, g_cell_history[1].X, g_cell_history[1].Y);
		}
		if (mt_is_linear() && IS_X_AXIS(g_new_dir)) {
			auto dir = path.target.Y - g_cell_history[1].Y;//+2,-2
			tmp.Y = g_cell_history[1].Y;
			ROS_ERROR("%s %d:mt_is_linear tmp(%d,%d),his1(%d,%d),curr(%d,%d)", __FUNCTION__, __LINE__, tmp.X, tmp.Y,
								g_cell_history[1].X, g_cell_history[1].Y, curr.X, curr.Y);
		}
	}
	ROS_INFO("%s %d: curr(%d,%d)", __FUNCTION__, __LINE__, curr.X, curr.Y);
	auto i=0;
	for(;i<4;i++)
	{
//		ROS_INFO("%s %d: i(%d)", __FUNCTION__, __LINE__, i);
		auto neighbor = tmp+g_index[i]+g_index[i];
		is_found = is_axis_access(neighbor, i, target);
		if(is_found) {
			path.target = target;
		path.cells.clear();
    for(auto cell = path.target; cell != tmp; cell -= g_index[i]) {
			path.cells.push_front(cell);
        if(tmp != curr)
					path.cells.push_front(curr);
//			ROS_INFO("%s %d: target(%d,%d), cell(%d,%d)", __FUNCTION__, __LINE__,target.X, target.Y, cell.X, cell.Y);
		}
			break;
		}
	}
	ROS_INFO("%s %d: is_found(%d)", __FUNCTION__, __LINE__, is_found);
	if (! is_found) {
//		debug_map(MAP, 0, 0);
		is_found = path_dijkstra(curr, target);
		if(is_found) {
			ROS_INFO("%s %d: is_found(%d), i(%d) target(%d,%d)", __FUNCTION__, __LINE__, is_found, i, target.X, target.Y);
//			pathFind(curr, target, path.cells);
//			if( target.Y%2 != 0) {
//				ROS_ERROR("%s %d: is_found(%d), i(%d) target(%d,%d)", __FUNCTION__, __LINE__, is_found, i, target.X, target.Y);
//				if(curr.Y < target.Y && is_block_accessible(target.X,target.Y+1))
//					target.Y += 1;
//				else if(curr.Y > target.Y && is_block_accessible(target.X,target.Y-1))
//					target.Y -= 1;
//				ROS_ERROR("%s %d: is_found(%d), i(%d) target(%d,%d)", __FUNCTION__, __LINE__, is_found, i, target.X, target.Y);
//			}
			path.target = target;
			path.cells.clear();
			path_find_target(curr, path);
//			for (const auto &cell : path.cells)
//				ROS_WARN("%s %d: cell(%d,%d)", __FUNCTION__, __LINE__, cell.X, cell.Y);
		}
	}
	return is_found;
}

typedef int32_t(*Func_t)(void);

bool for_each_neighbor(const Cell_t& cell,CellState cost, Func_t func) {
	for (auto i = 0; i < 4; i++) {
		Cell_t neighbor = g_index[i] + cell;
		if (neighbor.Y >= g_y_min && neighbor.Y <= g_y_max && neighbor.X >= g_x_min && neighbor.X <= g_x_max)
			if (map_get_cell(SPMAP, neighbor.X, neighbor.Y) == cost) {
				func();
      }
	}
}

void path_find_all_targets(const Cell_t& curr)
{
	bool		all_set;
	int16_t		i, j, x, y, offset, passValue, nextPassValue, passSet, tracex, tracey, targetCost, x_min, x_max, y_min, y_max;
	CellState	cs;

	map_reset(SPMAP);

	path_get_range(SPMAP, &x_min, &x_max, &y_min, &y_max);
	for (i = x_min; i <= x_max; ++i) {
		for (j = y_min; j <= y_max; ++j) {
			cs = map_get_cell(MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				for (x = ROBOT_RIGHT_OFFSET; x <= ROBOT_LEFT_OFFSET; x++) {
					for (y = ROBOT_RIGHT_OFFSET; y <= ROBOT_LEFT_OFFSET; y++) {
						map_set_cell(SPMAP, i + x, j + y, COST_HIGH);
					}
				}
			}
		}
	}

	x = curr.X;
	y = curr.Y;

	/* Set the current robot position has the cost value of 1. */
	map_set_cell(SPMAP, (int32_t) x, (int32_t) y, COST_1);

	offset = 0;
	passSet = 1;
	passValue = 1;
	nextPassValue = 2;
	while (passSet == 1) {
		offset++;
		passSet = 0;
		for (i = x - offset; i <= x + offset; i++) {
			if (i < x_min || i > x_max)
				continue;

			for (j = y - offset; j <= y + offset; j++) {
				if (j < y_min || j > y_max)
					continue;

				if(map_get_cell(SPMAP, i, j) == passValue) {
					if (i - 1 >= x_min && map_get_cell(SPMAP, i - 1, j) == COST_NO) {
						map_set_cell(SPMAP, (i - 1), (j), (CellState) nextPassValue);
						passSet = 1;
					}

					if ((i + 1) <= x_max && map_get_cell(SPMAP, i + 1, j) == COST_NO) {
						map_set_cell(SPMAP, (i + 1), (j), (CellState) nextPassValue);
						passSet = 1;
					}

					if (j - 1  >= y_min && map_get_cell(SPMAP, i, j - 1) == COST_NO) {
						map_set_cell(SPMAP, (i), (j - 1), (CellState) nextPassValue);
						passSet = 1;
					}

					if ((j + 1) <= y_max && map_get_cell(SPMAP, i, j + 1) == COST_NO) {
						map_set_cell(SPMAP, (i), (j + 1), (CellState) nextPassValue);
						passSet = 1;
					}
				}
			}
		}

		all_set = true;
		for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
			if (map_get_cell(SPMAP, it->target.X, it->target.Y) == COST_NO) {
				all_set = false;
			}
		}
		if (all_set == true) {
			ROS_INFO("%s %d: all possible target are checked & reachable.", __FUNCTION__, __LINE__);
			passSet = 0;
		}

		passValue = nextPassValue;
		nextPassValue++;
		if(nextPassValue == COST_PATH)
			nextPassValue = 1;
	}

	ROS_INFO("%s %d: offset: %d\tx: %d - %d\ty: %d - %d", __FUNCTION__, __LINE__, offset, x - offset, x + offset, y - offset, y + offset);
	debug_map(SPMAP, 0, 0);

	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
		if (map_get_cell(SPMAP, it->target.X, it->target.Y) == COST_NO ||
						map_get_cell(SPMAP, it->target.X, it->target.Y) == COST_HIGH) {
			continue;
		}

		Cell_t	t;
		tracex = it->target.X;
		tracey = it->target.Y;
		while (tracex != x || tracey != y) {
			targetCost = map_get_cell(SPMAP, tracex, tracey) - 1;

			if (targetCost == 0) {
				targetCost = COST_5;
			}

			t.X = tracex;
			t.Y = tracey;
			it->cells.push_back(t);

			if ((tracex - 1 >= x_min) && (map_get_cell(SPMAP, tracex - 1, tracey) == targetCost)) {
				tracex--;
				continue;
			}

			if ((tracex + 1 <= x_max) && (map_get_cell(SPMAP, tracex + 1, tracey) == targetCost)) {
				tracex++;
				continue;
			}

			if ((tracey - 1 >= y_min) && (map_get_cell(SPMAP, tracex, tracey - 1) == targetCost)) {
				tracey--;
				continue;
			}

			if ((tracey + 1 <= y_max) && (map_get_cell(SPMAP, tracex, tracey + 1) == targetCost)) {
				tracey++;
				continue;
			}
		}
		if (tracex == x || tracey == y) {
			t.X = tracex;
			t.Y = tracey;
			it->cells.push_back(t);
		}
	}
}


static bool is_path_valid(const PPTargetType &it,int16_t towards_y, int16_t re_towards_y, int16_t target_y,int16_t curr_y, int16_t turnable)
{
//	ROS_WARN("towards_y(%d),re_towards_y(%d),turnable(%d)", towards_y, re_towards_y,turnable);
	bool inited = false;
	int16_t last_y = it.cells.front().Y;
	auto turn = false;
	auto re_towards = false;
	auto min = std::min(towards_y, re_towards_y);
	auto max = std::max(towards_y, re_towards_y);
	for (const auto &cell : it.cells)
	{
		if (!inited || (turnable == 1 && turn))
		{
//			ROS_WARN("inited(%d) turnable(%d)turn(%d)",inited, turnable, turn);
			if(inited){
				re_towards = true;
				turnable = 0;
			}
			inited = true;
		}
		if(turnable > 1)
			continue;
		auto pos_dir = (turnable == 1 || !re_towards) ? towards_y > re_towards_y : towards_y < re_towards_y ;
		turn = (cell.Y != last_y  && (pos_dir ^ cell.Y < last_y));
//		ROS_INFO("cell(%d,%d),turnable(%d),turn(%d),pos_dir(%d)",cell.X,cell.Y,turnable,turn, pos_dir);
		if (cell.Y < min || cell.Y > max || (! turnable && turn))
			return false;

		last_y = cell.Y;
	}
//	ROS_WARN("target:(%d,%d)~~~~~~~~~~~~~~~~~~~~",it.target.X, it.target.Y);
	return true;
}

static int16_t path_area_target(const Cell_t &curr, int16_t y_min, int16_t y_max, Cell_t &target,
														 list<PPTargetType> &g_targets)
{
	int16_t cost = FINAL_COST;
	enum { START_Y, TOWARDS, TURN, Y_NUM };
	int16_t area[6][Y_NUM] = {
					{ (int16_t)(curr.Y + 2), 1, 0},
//					{ curr.Y, 1, 1 },
//					{ (int16_t)(curr.Y - 2), -1, 0},
//					{ curr.Y, -1, 1 },
//					{ curr.Y, 1, INT8_MAX },
//					{ curr.Y, -1, INT8_MAX },
	};
	for (auto i = 0; i < 1; i++)
	{
		auto &dy = area[i][TOWARDS];
		auto towards_y = (dy == 1) ? y_max : y_min;
		auto re_towards_y = (dy == 1) ? y_min : y_max;
		auto towards_y_end = (dy == 1) ? y_max + 1: y_min - 1 ;
		auto re_towards_y_end = (dy == 1)? y_min - 1: y_max + 1;
		auto area_y_begin = area[i][START_Y];
//		ROS_WARN("%s %d: case %d, TOWARDS(%d), allow turn count: %d ", __FUNCTION__, __LINE__, i, area[i][TOWARDS], area[i][TURN]);
		do {
//			ROS_INFO(" %s %d: y: towards_y_end(%d), re_towards_y_end(%d), area_y_begin(%d),", __FUNCTION__, __LINE__,towards_y_end, re_towards_y_end, area_y_begin);
			for (auto target_y = area_y_begin; target_y != towards_y_end; target_y += dy)
			{
//				ROS_INFO(" %s %d: target_y(%d)", __FUNCTION__, __LINE__, target_y);
				for (const auto &target_it : g_targets)
				{
					if (target_y == target_it.target.Y)
					{
						if (is_path_valid(target_it, towards_y, re_towards_y, target_y, curr.Y, area[i][TURN]) &&
								cost > target_it.cells.size())
						{
							target = target_it.target;
							cost = target_it.cells.size();
//              ROS_INFO(" %s %d: target_y(%d),cost %d", __FUNCTION__, __LINE__, target_y,cost);
						}

					}
				}
				if (cost != FINAL_COST)
				{
					ROS_INFO("cost %d", cost);
					return cost;
				}
			}
			area_y_begin-=dy;
		} while(area[i][TURN] && area_y_begin != re_towards_y_end);
	}
		for (const auto& target_it : g_targets) {
			if (cost > target_it.cells.size()) {
				target = target_it.target;
				cost = target_it.cells.size();
			}
	}
	if (cost != FINAL_COST)
	{
		ROS_INFO("cost %d", cost);
		return cost;
	}
	return 0;
}

int16_t path_target(const Cell_t& curr, PPTargetType& path)
{
	BoundingBox2 map;
	BoundingBox2 map_tmp{{int16_t(g_x_min-1),int16_t(g_y_min - 1)},{g_x_max,g_y_max}};
	Cell_t target_tmp;

	for (const auto& cell : map_tmp)
	{
		if (map_get_cell(MAP, cell.X, cell.Y) != UNCLEAN)
			map.Add(cell);
	}

	map_tmp = map;
	for (const auto& cell : map_tmp)
	{
		if (map_get_cell(MAP, cell.X, cell.Y) != CLEANED /*|| std::abs(cell.Y % 2) == 1*/)
			continue;

		Cell_t neighbor;
		for (auto i = 0; i < 4; i++)
		{
			neighbor = cell+g_index[i];
			if (map_get_cell(MAP, neighbor.X, neighbor.Y) == UNCLEAN)
			{
				if (is_block_accessible(neighbor.X, neighbor.Y) == 1)
				{
#if INTERLACED_MOVE
					if(neighbor.Y % 2 != 0)
						neighbor.Y = cell.Y;
					if(neighbor.Y % 2 != 0)
						continue;
#endif
					map_set_cell(MAP, cell_to_count(neighbor.X), cell_to_count(neighbor.Y), TARGET);
					map.Add(neighbor);
				}
			}
		}
	}
	for (auto& target : g_targets)
		target.cells.clear();
	g_targets.clear();

//	 Narrow down the coodinate that robot should go
	for (auto y = map.min.Y; y <= map.max.Y; y++)
	{
		for(auto i =0;i<2;i++)
		{
			auto dx = i==0 ? 1 : -1;
			auto bound = i==0 ? SHRT_MAX : SHRT_MIN;
			auto start =bound;
			auto end = bound;
			auto begin_x = i==0 ? map.min.X : map.max.X;
			auto end_x = curr.X+dx;
			for (auto x = begin_x; x != end_x; x += dx)
			{
				if (map_get_cell(MAP, x, y) == TARGET)
				{
					if (start == bound) start = x;
					if (start != bound) end = x;
				}
				if (map_get_cell(MAP, x, y) != TARGET || x == curr.X)
				{
					if (start != bound &&std::abs(end-start) > 1) {
						for (auto it_x = start + dx; it_x != end; it_x += dx)
							if (map_get_cell(MAP, it_x, y) == TARGET)
								map_set_cell(MAP, cell_to_count(it_x), cell_to_count(y), UNCLEAN);
						start = end = bound;
					}
				}
			}
		}
	}
//	debug_map(MAP, g_home_x, g_home_y);
	map_tmp = map;
	for (const auto& cell : map_tmp) {
		if (map_get_cell(MAP, cell.X, cell.Y) == TARGET) {
			PPTargetType t;
			t.target = cell;
			t.cells.clear();
			g_targets.push_back(t);
			map.Add(cell);
		}
	}
	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
		map_set_cell(MAP, cell_to_count(it->target.X), cell_to_count(it->target.Y), UNCLEAN);
	}

	path_find_all_targets(curr);

	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end();) {
		if (it->cells.empty()) {
			it = g_targets.erase(it);
		} else {
			it++;
		}
	}

	/* No more target to clean */
	if (g_targets.empty()) {
		if (path_escape_trapped(curr) <= 0) {
			ROS_WARN("%s %d: trapped", __FUNCTION__, __LINE__);
			return -2;
		}
		ROS_INFO("%s %d: targets list empty.", __FUNCTION__, __LINE__);
		return 0;
	}

	ROS_INFO("%s %d: targets count: %d", __FUNCTION__, __LINE__, (int)g_targets.size());

//	for(const auto& path_it: g_targets)
//		path_display_path_points(path_it.cells);

	bool is_stop = false, is_found = false, within_range=false;
  int16_t last_y;
	Cell_t temp_target;
	auto final_cost = 1000;
	ROS_INFO("%s %d: case 1, towards Y+ only", __FUNCTION__, __LINE__);
	for (auto d = map.max.Y; d >= curr.Y; --d) {
		if (is_stop && d <= curr.Y + 1) {
			break;
		}
		for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
			if (map_get_cell(MAP, it->target.X, it->target.Y - 1) != CLEANED) {
				continue;
			}
			if (it->target.Y == d) {
				if (it->cells.size() > final_cost) {
					continue;
				}

				last_y = it->cells.front().Y;
				within_range = true;
				for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
					if (i->Y < curr.Y || i->Y > d) {
						within_range = false;
					}
					if (i->Y > last_y) {
						within_range = false;
					} else {
						last_y = i->Y;
					}
				}
				if (within_range == true) {
					temp_target = it->target;
					final_cost = it->cells.size();
					is_stop = true;
				}
			}
		}
	}

//#if !INTERLACED_MOVE
#if 1
	if (!is_stop) {
		ROS_INFO("%s %d: case 2, towards Y+, allow Y- shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto a = curr.Y; a >= map.min.Y && !is_stop; --a) {
			for (auto d = a; d <= map.max.Y && !is_stop; ++d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->target.Y == d) {
						if (it->cells.size() > final_cost) {
							continue;
						}

						within_range = true;
						last_y = it->cells.front().Y;
						bool turn = false;
						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
							if (i->Y < a || i->Y > (d > curr.Y ? d : curr.Y)) {
								within_range = false;
							}
							if (turn == false) {
								if (i->Y > last_y) {
									within_range = false;
								} else {
									last_y = i->Y;
								}
							} else {
								if (i->Y < last_y) {
									within_range = false;
								} else {
									last_y = i->Y;
								}
							}
							if (i->Y == a) {
								turn = true;
							}
						}
						if (within_range == true) {
							temp_target = it->target;
							final_cost = it->cells.size();
							is_stop = true;
						}
					}
				}
			}
		}
	}

	if (!is_stop) {
		ROS_INFO("%s %d: case 3, towards Y- only, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto d = map.min.Y; d >= curr.Y; ++d) {
			if (is_stop && d >= curr.Y - 1) {
				break;
			}

			for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
				if (map_get_cell(MAP, it->target.X, it->target.Y + 1) != CLEANED) {
					continue;
				}

				if (it->target.Y == d) {
					if (it->cells.size() > final_cost) {
						continue;
					}

					last_y = it->cells.front().Y;
					within_range = true;
					for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
						if (i->Y > curr.Y || i->Y < d) {
							within_range = false;
						}
						if (i->Y < last_y) {
							within_range = false;
						} else {
							last_y = i->Y;
						}
					}
					if (within_range == true) {
						temp_target = it->target;
						final_cost = it->cells.size();
						is_stop = true;
					}
				}
			}
		}
	}

	if (!is_stop) {
		ROS_INFO("%s %d: case 4, towards Y-, allow Y+ shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto a = curr.Y; a <= map.max.Y && !is_stop; ++a) {
			for (auto d = a; d >= map.min.Y && !is_stop; --d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->target.Y == d) {
						if (it->cells.size() > final_cost) {
							continue;
						}

						within_range = true;
						last_y = it->cells.front().Y;
						bool turn = false;
						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
							if (i->Y > a || i->Y < (d > curr.Y ? curr.Y : d)) {
								within_range = false;
							}
							if (turn == false) {
								if (i->Y < last_y) {
									within_range = false;
								} else {
									last_y = i->Y;
								}
							} else {
								if (i->Y > last_y) {
									within_range = false;
								} else {
									last_y = i->Y;
								}
							}
							if (i->Y == a) {
								turn = true;
							}
						}
						if (within_range == true) {
							temp_target = it->target;
							final_cost = it->cells.size();
							is_stop = true;
						}
					}
				}
			}
		}
	}
	if (!is_stop) {
		ROS_INFO("%s %d: case 5: towards Y+, allow Y- shift, allow turns, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto a = curr.Y; a <= map.max.Y  && is_stop == 0; ++a) {
			for (auto d = curr.Y; d <= a && is_stop == 0; ++d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->target.Y == d) {
						within_range = true;
						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
							if (i->Y < curr.Y || i->Y > a) {
								within_range = false;
							}
						}
						if (within_range == true && it->cells.size() < final_cost) {
							temp_target = it->target;
							final_cost = it->cells.size();
							is_stop = 1;
						}
					}
				}
			}
		}
	}
#endif
	/* fallback to find unclean area */
	if (!is_stop) {
		ROS_INFO("%s %d: case 6, fallback to A-start the nearest target, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto c = map.min.X; c <= map.max.X; ++c) {
			for (auto d = map.min.Y; d <= map.max.Y; ++d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->cells.size() < final_cost) {
						temp_target = it->target;
						final_cost = it->cells.size();
					}
				}
			}
		}
	}

	is_found = (final_cost != 1000) ? final_cost : 0 ;
	ROS_INFO("%s %d: is_found: %d (%d, %d)\n", __FUNCTION__, __LINE__, is_found, temp_target.X, temp_target.Y);
	debug_map(MAP, temp_target.X, temp_target.Y);

	if(is_found == 0)
		return 0;

	return path_next_shortest(curr, temp_target, path);
}

void path_update_cells()
{
	if(get_clean_mode() != Clean_Mode_Navigation)
		return;
	/* Skip, if robot is not moving towards POS_X. */
	if ((g_new_dir % 1800) != 0)
		return;

	auto curr_x = g_cell_history[0].X;
	auto curr_y = g_cell_history[0].Y;
	auto last_x = g_cell_history[1].X;
	auto start = std::min(curr_x, last_x);
	auto stop  = std::max(curr_x, last_x);
	ROS_INFO("%s %d: start: %d\tstop: %d", __FUNCTION__, __LINE__, start, stop);
	for (auto x = start; x <= stop; x++)
	{
		for (auto dy = -1; dy <= 1; dy += 2)
		{
			if (map_get_cell(MAP, x, curr_y + dy) == BLOCKED_OBS || map_get_cell(MAP, x, curr_y + dy) == BLOCKED_BUMPER)
			{
				auto state = CLEANED;
				auto dx_start = -1;
				auto dx_stop = 1;
				if (x == start) dx_start = dx_stop;
				if (x == stop) dx_stop = dx_start;
				for (auto dx = dx_start; dx <= dx_stop; dx += 2)
				{
					if (map_get_cell(MAP, x + dx, curr_y + dy) == CLEANED)
						break;
					else
						state = UNCLEAN;
				}
				ROS_WARN("%s %d: reset (%d,%d) to %d.", __FUNCTION__, __LINE__, x, curr_y + dy, start);
				map_set_cell(MAP, cell_to_count(x), cell_to_count(curr_y + dy), state);
			}
		}
	}
}

int16_t path_escape_trapped(const Cell_t& curr)
{
	int16_t	val = 0;
	for (auto home_it = g_homes.rbegin(); home_it != g_homes.rend(); ++home_it)
	{
		ROS_WARN("%s %d: home_it(%d,%d)", __FUNCTION__, __LINE__, home_it->X, home_it->Y);
	}
	for (auto home_it = g_homes.rbegin(); home_it != g_homes.rend() && std::abs(std::distance(home_it, g_homes.rbegin()))<ESCAPE_TRAPPED_REF_CELL_SIZE; ++home_it) {
		ROS_WARN("%s %d: home_it distance(%d)", __FUNCTION__, __LINE__, std::distance(home_it, g_homes.rbegin()));
		if (is_block_accessible(home_it->X, home_it->Y) == 0)
			map_set_cells(ROBOT_SIZE, home_it->X, home_it->Y, CLEANED);

		val = path_find_shortest_path(curr.X, curr.Y, home_it->X, home_it->Y, 0);
		ROS_WARN("%s %d: val %d", __FUNCTION__, __LINE__, val);
		val = (val < 0 || val == SCHAR_MAX) ? 0 : 1;
		if(val == 1) break;
	}
	return val;
}

int8_t path_next(const Cell_t& curr, PPTargetType& path)
{
	//ros_map_convert(false);
	extern bool g_keep_on_wf;
	if(!g_go_home && get_clean_mode() == Clean_Mode_WallFollow){
		ROS_INFO("path_next Clean_Mode:(%d)", get_clean_mode());
		if(mt_is_linear()){
			if(curr != path.target){
				ROS_INFO("start follow wall");
				mt_set(CM_FOLLOW_LEFT_WALL);
			}else{
				ROS_INFO("reach 8m, go_home.");
				g_finish_cleaning_go_home = true;
				cm_check_should_go_home();
			}
		} else {
			if(wf_is_go_home()) {
				ROS_INFO("follow wall finish");
				g_finish_cleaning_go_home = true;
				cm_check_should_go_home();

			} else if (g_keep_on_wf) {
				ROS_INFO("keep on follow wall");
				mt_set(CM_FOLLOW_LEFT_WALL);
				g_keep_on_wf = false;
			} else {
				ROS_INFO("CM_LINEARMOVE");
				mt_set(CM_LINEARMOVE);
				wf_break_wall_follow();
				auto angle = wf_is_first() ? 0 : -900;
				const float	FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
				cm_world_to_cell(gyro_get_angle() + angle, 0, FIND_WALL_DISTANCE * 1000, path.target.X, path.target.Y);
				path.cells.clear();
				path.cells.push_front(path.target);
				path.cells.push_front(curr);
				ROS_INFO("target.X = %d target.Y = %d", path.target.X, path.target.Y);
			}

		}
	}
	else if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT || SpotMovement::instance()->getSpotType() == NORMAL_SPOT){
		if (!SpotMovement::instance()->spotNextTarget(curr,&path))
			return 0;
		debug_map(MAP, path.target.X, path.target.Y);
		//path.cells.clear();
		//path.cells.push_front(path.target);
		path.cells.push_front(curr);

	}
	else if(!g_go_home && get_clean_mode() == Clean_Mode_Navigation) {
		if (g_resume_cleaning && path_get_continue_target(curr, path) != TARGET_FOUND)
			g_resume_cleaning = false;

		if (!g_resume_cleaning)
		{
#if !PATH_ALGORITHM_V2
			if (!path_lane_is_cleaned(curr, path))
			{
				extern bool g_isolate_triggered;
				int16_t ret;
				if (g_isolate_triggered) {
					ret = isolate_target(curr, path);
					g_isolate_triggered = false;
				} else {
					ret = path_target(curr, path);//0 not target, 1,found, -2 trap
				}
				ROS_WARN("%s %d: path_target return: %d. Next(%d,%d), Target(%d,%d).", __FUNCTION__, __LINE__, ret, path.cells.front().X, path.cells.front().Y, path.cells.back().X, path.cells.back().Y);
				if (ret == 0)
				{
					g_finish_cleaning_go_home = true;
					cm_check_should_go_home();
				}
				if (ret == -2){
					if(g_trapped_mode == 0 ){
						g_trapped_mode = 1;
						// This led light is for debug.
						set_led_mode(LED_FLASH, LED_GREEN, 300);
						mt_set(CM_FOLLOW_LEFT_WALL);
						extern uint32_t g_escape_trapped_timer;
						g_escape_trapped_timer = time(NULL);
					}
					return 1;
				}
			}
			//ROS_WARN("%s,%d: curr(%d,%d), next(%d,%d), target(%d,%d)", __FUNCTION__, __LINE__, curr.X, curr.Y, path.cells.front().X, path.cells.front().Y, path.cells.back().X, path.cells.back().Y);
#else
			extern bool g_isolate_triggered;
			int16_t ret;
			if (g_isolate_triggered) {
				ret = isolate_target(curr, path);
				g_isolate_triggered = false;
			}
			else {
				ret = path_full(curr, path);//0 not target, 1,found, -2 trap
				if(ret==0)
					if (path_escape_trapped() <= 0)
						ret = -2;
			}
			ROS_WARN("%s %d: path_target return: %d. Next(%d,%d), Target(%d,%d).", __FUNCTION__, __LINE__, ret, path.cells.front().X, path.cells.front().Y, path.cells.back().X, path.cells.back().Y);
			if (ret == 0)
			{
				g_finish_cleaning_go_home = true;
				cm_check_should_go_home();
			}
			if (ret == -2){
				if(g_trapped_mode == 0 ){
					g_trapped_mode = 1;
					// This led light is for debug.
					set_led_mode(LED_FLASH, LED_GREEN, 300);
					mt_set(CM_FOLLOW_LEFT_WALL);
					extern uint32_t g_escape_trapped_timer;
					g_escape_trapped_timer = time(NULL);
				}
				return 1;
			}
#endif
		}
	}

	if (g_go_home && path_get_home_target(curr, path) == NO_TARGET_LEFT) {
			return 0;
	}

	// Delete the first cell of list, it means current cell. Do this for checking whether it should follow the wall.
	if (path.cells.size() > 1)
		path.cells.pop_front();
	g_next_cell = path.cells.front();
	g_target_cell = path.target;

	g_old_dir = g_new_dir;
	//if (g_go_home || SpotMovement::instance()->getSpotType() != NO_SPOT)
	if (g_go_home)
		mt_set(CM_LINEARMOVE);
	else if(get_clean_mode() == Clean_Mode_Navigation)
		mt_update(curr, path, g_old_dir);
	// else if wall follow mode, the move type has been set before here.
	if (curr.X == g_next_cell.X)
		g_new_dir = curr.Y > g_next_cell.Y ? NEG_Y : POS_Y;
	else
		g_new_dir = curr.X > g_next_cell.X ? NEG_X : POS_X;

#if LINEAR_MOVE_WITH_PATH
	if (mt_is_linear() && get_clean_mode() != Clean_Mode_WallFollow)
	{
		// Add current cell for filling the path, otherwise it will lack for the path from current to the first turning cell.
		path.cells.push_front(curr);
		path_fill_path(path.cells);
		// Delete the current cell.
		if (path.cells.size() > 1)
			path.cells.pop_front();
		//Update the g_next_cell because the path has been filled.
		g_next_cell = path.cells.front();
	}
#endif
	return 1;
}

void path_fill_path(std::list<Cell_t>& path)
{
	uint16_t dir;
	Cell_t cell;
	list<Cell_t> saved_path = path;
	path.clear();
	//path_display_path_points(saved_path);

	//for (list<Cell_t>::iterator it = saved_path.begin(); it->X != saved_path.back().X || it->Y != saved_path.back().Y; it++)
	for (list<Cell_t>::iterator it = saved_path.begin(); it != saved_path.end(); it++)
	{
		list<Cell_t>::iterator next_it = it;
		if(++next_it == saved_path.end()){
			ROS_INFO("%s,%d,fill path to last interator",__FUNCTION__,__LINE__);
			break;
		}
		//ROS_DEBUG("%s %d: it(%d, %d), next it(%d, %d).", __FUNCTION__, __LINE__, it->X, it->Y, next_it->X, next_it->Y);
		if (next_it->X == it->X)
			dir = next_it->Y > it->Y ? POS_Y : NEG_Y;
		else
			dir = next_it->X > it->X ? POS_X : NEG_X;

		cell.X = it->X;
		cell.Y = it->Y;
		switch(dir)
		{
			case POS_X:
			{
				while (cell.X != next_it->X)
				{
					// Fill the path with the middle cells.
					path.push_back(cell);
					//ROS_DEBUG("%s %d: Push back(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
					cell.X++;
				}
				break;
			}
			case NEG_X:
			{
				while (cell.X != next_it->X)
				{
					// Fill the path with the middle cells.
					path.push_back(cell);
					//ROS_DEBUG("%s %d: Push back(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
					cell.X--;
				}
				break;
			}
			case POS_Y:
			{
				while (cell.Y != next_it->Y)
				{
					// Fill the path with the middle cells.
					path.push_back(cell);
					//ROS_DEBUG("%s %d: Push back(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
					cell.Y++;
				}
				break;
			}
			case NEG_Y:
			{
				while (cell.Y != next_it->Y)
				{
					// Fill the path with the middle cells.
					path.push_back(cell);
					//ROS_DEBUG("%s %d: Push back(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
					cell.Y--;
				}
				break;
			}
		}
	}
	// Push the target point to path.
	cell = saved_path.back();
	path.push_back(cell);
	//ROS_DEBUG("%s %d: End cell(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
	std::string msg = "Filled path:";
	for (std::list<Cell_t>::iterator it = path.begin(); it != path.end(); ++it) {
		msg += "->(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ")";
	}
	//ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());
}

void path_escape_set_trapped_cell( Cell_t *cell, uint8_t size )
{
	g_trapped_cell_size = size;
	for (auto i = 0; i < g_trapped_cell_size; ++i ) {
		g_homes[i] = cell[i];
		ROS_INFO("%s %d Set %d trapped reference cell: x: %d\ty:%d", __FUNCTION__, __LINE__, i, g_homes[i].X, g_homes[i].Y);
	}
}

//Cell_t *path_escape_get_trapped_cell()
//{
//	return g_homes;
//}

void path_set_home(const Cell_t& curr)
{
	bool is_found = false;

	for (const auto& it : g_homes) {
		ROS_INFO("%s %d: curr(%d, %d) home_it(%d,%d).", __FUNCTION__, __LINE__, curr.X, curr.Y,it.X,it.Y);
		if (it == curr) {
			is_found = true;
			break;
		}
	}
	if (!is_found) {
		ROS_INFO("%s %d: Push new reachable home: (%d, %d) to home point list.", __FUNCTION__, __LINE__, curr.X, curr.Y);
		extern bool g_have_seen_charge_stub;
		if(get_clean_mode() != Clean_Mode_Spot)
			g_have_seen_charge_stub = true;
		// If curr near (0, 0)
		if (abs(curr.X) >= 5 || abs(curr.Y) >= 5)
		{
        if(g_homes.size() >= ESCAPE_TRAPPED_REF_CELL_SIZE+1)//escape_count + zero_home = 3+1 = 4
				{
					std::copy(g_homes.begin() + 2, g_homes.end(), g_homes.begin()+1);//shift 1 but save zero_home
					g_homes.pop_back();
				}
				g_homes.push_back(curr);
		}
	}
	else if(curr == g_zero_home && get_clean_mode() != Clean_Mode_Spot)
	{
		extern bool g_start_point_seen_charger, g_have_seen_charge_stub;
		g_start_point_seen_charger = true;
		g_have_seen_charge_stub = true;
	}
}

/* Get next point and home point.
 * return :NO_TARGET_LEFT (0)
 *        :TARGET_FOUND (1)
 */
int8_t path_get_home_target(const Cell_t& curr, PPTargetType& path) {
	if(g_home_way_list.empty()) {
		g_home_way_it = _gen_home_ways(g_homes.size(), g_home_way_list);
		BoundingBox2 map_tmp{{g_x_min, g_y_min}, {g_x_max, g_y_max}};
		for (const auto &cell : map_tmp) {
			if (map_get_cell(MAP, cell.X, cell.Y) == BLOCKED_RCON)
				map_set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y), UNCLEAN);
		}
	}
	for (; g_home_way_it != g_home_way_list.end(); ++g_home_way_it) {
		auto way = *g_home_way_it % HOMEWAY_NUM;
		auto cnt = *g_home_way_it / HOMEWAY_NUM;
		g_home = g_homes[cnt];
		ROS_INFO("\033[1;46;37m" "%s,%d:g_home(%d), way(%d), cnt(%d) " "\033[0m", __FUNCTION__, __LINE__,g_home,way, cnt);
		if (way == USE_ROS && g_home_gen_rosmap) {
			g_home_gen_rosmap = false;
			ROS_INFO("\033[1;46;37m" "%s,%d:ros_map_convert" "\033[0m", __FUNCTION__, __LINE__);
			ros_map_convert(MAP, false, true);
		}

		if (path_next_shortest(curr, g_home, path) == 1) {
			return TARGET_FOUND;
		}
	}
	return NO_TARGET_LEFT;
}

int16_t path_get_home_x()
{
	return g_home_x;
}

int16_t path_get_home_y()
{
	return g_home_y;
}

void wf_path_planning_initialize()
{
	int16_t i;


	/* Initialize the default settings. */
//	preset_action_count = 0;

//	weight_enabled = 1;

#if (ROBOT_SIZE == 5)

//	g_weight_cnt_threshold = 4;

#else

//	g_weight_cnt_threshold = 3;

#endif

	g_direct_go = 0;

	g_cell_history[0] = {0,0};
	g_new_dir = 0;

	/* Reset the poisition list. */
	for (i = 0; i < 3; i++) {
		g_cell_history[i]= {int16_t(i + 1), int16_t(i + 1)};
	}

	/* Initialize the shortest path. */
	path_position_init(g_direct_go);
}

void path_set_continue_cell(Cell_t cell)
{
	g_continue_cell = cell;
	ROS_INFO("%s %d: Set continue cell: (%d, %d).", __FUNCTION__, __LINE__, g_continue_cell.X, g_continue_cell.Y);
}

int8_t path_get_continue_target(const Cell_t& curr, PPTargetType& path)
{
	int8_t return_val;
	path.target = g_continue_cell;

	ROS_WARN("%s %d: Need to resume cleaning, continue cell(%d, %d).", __FUNCTION__, __LINE__, g_continue_cell.X, g_continue_cell.Y);
	if (curr == g_continue_cell)
	{
		return_val = TARGET_REACHED;
		return return_val;
	}

	if (is_block_accessible(g_continue_cell.X, g_continue_cell.Y) == 0) {
		ROS_WARN("%s %d: target(%d, %d) is blocked, unblock the target.\n", __FUNCTION__, __LINE__, g_continue_cell.X, g_continue_cell.Y);
		map_set_cells(ROBOT_SIZE, g_continue_cell.X, g_continue_cell.Y, CLEANED);
	}

	auto path_next_status = (int8_t) path_next_shortest(curr, g_continue_cell, path);
	ROS_INFO("%s %d: Path Find: %d\tNext point: (%d, %d)\tNow: (%d, %d)", __FUNCTION__, __LINE__, path_next_status, path.cells.front().X, path.cells.front().Y, curr.X, curr.Y);
	if (path_next_status == 1/* && !cm_check_loop_back(next)*/)
		return_val = TARGET_FOUND;
	else
		return_val = NO_TARGET_LEFT;

	return return_val;
}

int16_t isolate_target(const Cell_t& curr, PPTargetType& path) {
	int16_t ret;
	//extern int g_isolate_count;
	//extern bool g_fatal_quit_event;
	//if (g_isolate_count <= 3) { 
		auto angle = -900;
		const float	FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
		cm_world_to_cell(gyro_get_angle() + angle, 0, FIND_WALL_DISTANCE * 1000, path.target.X, path.target.Y);
		path.cells.clear();
		path.cells.push_front(path.target);
		path.cells.push_front(curr);
		ret = 1;
		ROS_INFO("target.X = %d target.Y = %d", path.target.X, path.target.Y);
	//} else {
	//	g_fatal_quit_event = true;
	//	ret = 0;
	//}
	return ret;
}

bool path_dijkstra(const Cell_t& curr, Cell_t& target)
{
	typedef std::multimap<double, Cell_t> Queue;
	typedef std::pair<double, Cell_t> Entry;

//	int plan[g_x_max - g_x_min][g_y_max - g_y_min];
	map_reset(SPMAP);

	map_set_cell(SPMAP,curr.X,curr.Y,COST_1);
	Queue queue;
	Entry startPoint(0.0, curr);
	queue.insert(startPoint);
	bool is_found = false;
	ROS_INFO("Do full search with weightless Dijkstra-Algorithm\n");
	while (!queue.empty())
	{
//		 Get the nearest next from the queue
		auto start = queue.begin();
		auto next = start->second;
		queue.erase(start);

//		ROS_WARN("adjacent cell(%d,%d)", next.X, next.Y);
		if (is_block_unclean(next.X, next.Y) && is_block_accessible(next.X, next.Y))
		{
//			ROS_WARN("We find the Unclean next(%d,%d)", next.X, next.Y);
			is_found = true;
			target = next;
			break;
		} else
		{
			for (auto it = 0; it < 4; it++)
			{
				auto neighbor = next + g_index[it];
//				ROS_INFO("g_index[%d],next(%d,%d)", it, neighbor.X,neighbor.Y);
//				ROS_INFO("plan(%d)", map_get_cell(SPMAP, neighbor.X, neighbor.Y));
				if (map_get_cell(SPMAP, neighbor.X, neighbor.Y) == COST_NO && is_block_accessible(neighbor.X, neighbor.Y) )
				{
//					ROS_WARN("add to Queue:(%d,%d)", neighbor.X, neighbor.Y);
					queue.insert(Entry(0, neighbor));
					map_set_cell(SPMAP,neighbor.X, neighbor.Y,COST_1);
				}
			}
		}
	}
	return is_found;
}

bool is_fobbit_free() {
	//NOTE: g_home_way_it should last of g_home,for g_homeway_list may empty.
	return (g_go_home && *g_home_way_it % HOMEWAY_NUM == USE_CLEANED);
}
