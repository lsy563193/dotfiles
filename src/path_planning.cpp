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

#define INTERLACED_MOVE	(1)
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
std::list <Cell_t> g_home_point_old_path;
std::list <Cell_t> g_home_point_new_path;

bool g_switch_home_cell = true;
Cell_t g_current_home_cell;

int16_t g_home_x = 0, g_home_y = 0;

// This is for the continue point for robot to go after charge.
Cell_t g_continue_cell;

Cell_t g_cell_history[5];

uint16_t g_new_dir;
uint16_t g_old_dir;

int g_trapped_mode = 1;

Cell_t g_trapped_cell[ESCAPE_TRAPPED_REF_CELL_SIZE];

uint8_t g_trapped_cell_size = ESCAPE_TRAPPED_REF_CELL_SIZE;

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

extern int16_t g_wf_x_min, g_wf_x_max, g_wf_y_min, g_wf_y_max;

//void path_planning_initialize(int32_t *x, int32_t *y)
void path_planning_initialize(Cell_t cell)
{
	int16_t i;

	/* Save the starting point as home. */
	for ( i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
		g_trapped_cell[i] = cell;
	}
	g_trapped_cell_size = ESCAPE_TRAPPED_REF_CELL_SIZE;

#if (ROBOT_SIZE == 5)

//	g_weight_cnt_threshold = 4;

#else

//	g_weight_cnt_threshold = 3;

#endif

	g_direct_go = 0;

	g_cell_history[0] = {0,0};
	g_new_dir = 0;
	g_new_dir = 0;

	/* Reset the poisition list. */
	for (i = 0; i < 3; i++) {
		g_cell_history[i] =  {int16_t(i+1), int16_t(i+1)};
	}

	/* Initialize the shortest path. */
	path_position_init(g_direct_go);

#ifndef ZONE_WALLFOLLOW

	/* Set the back as blocked, since robot will align the start angle with the wall. */
	Map_SetCell(MAP, cell_to_count(-3), cell_to_count(0), BLOCKED_BUMPER);

#endif

	map_set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y), CLEANED);

	/* Set the starting point as cleaned. */
	map_set_cell(MAP, cell_to_count(-1), cell_to_count(-1), CLEANED);
	map_set_cell(MAP, cell_to_count(-1), cell_to_count(0), CLEANED);
	map_set_cell(MAP, cell_to_count(-1), cell_to_count(1), CLEANED);
	map_set_cell(MAP, cell_to_count(0), cell_to_count(-1), CLEANED);
	map_set_cell(MAP, cell_to_count(0), cell_to_count(0), CLEANED);
	map_set_cell(MAP, cell_to_count(0), cell_to_count(1), CLEANED);
	map_set_cell(MAP, cell_to_count(1), cell_to_count(-1), CLEANED);
	map_set_cell(MAP, cell_to_count(1), cell_to_count(0), CLEANED);
	map_set_cell(MAP, cell_to_count(1), cell_to_count(1), CLEANED);
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

int8_t is_block_cleanable(int16_t x, int16_t y)
{
	int8_t	retval = 1;
	int16_t	i, j;

	for (i = ROBOT_BRUSH_LEFT_OFFSET; retval == 1 && i <= ROBOT_BRUSH_RIGHT_OFFSET; i++) {
		for (j = ROBOT_BRUSH_LEFT_OFFSET; retval == 1 && j <= ROBOT_BRUSH_RIGHT_OFFSET; j++) {
			if (is_a_block(x + i, y + j) == 1) {
				retval = 0;
			}
		}
	}
	return retval;
}

int8_t is_block_cleaned(int16_t x, int16_t y)
{
	int8_t	retval = 1;
	if (map_get_cell(MAP, x, y) != CLEANED)
		retval = 0;
	return retval;
}

bool is_brush_block_unclean(int16_t x, int16_t y)
{
	bool retval = false;
	uint8_t unclean_cnt = 0;
	for (int8_t i = (y + ROBOT_RIGHT_OFFSET); i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (map_get_cell(MAP, x, i) == UNCLEAN) {
			unclean_cnt++;
		}
	}
#if INTERLACED_MOVE
	if (unclean_cnt > 0)
#else
	if (unclean_cnt > 1)
#endif
		retval = true;
	return retval;
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

	return retval;
}

bool path_lane_is_cleaned(const Cell_t& curr, PPTargetType& path)
{
	int16_t i, is_found=0, min=SHRT_MAX, max=SHRT_MAX, min_stop=0, max_stop=0;
	Cell_t tmp = curr;
#if INTERLACED_MOVE
	if(std::abs(curr.Y % 2) == 1) {
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
			} else if (is_brush_block_unclean(curr.X - i, curr.Y))
			{
				// Find the furthest unclean cell.
				min = is_block_boundary(curr.X - (i + 1), curr.Y) ? min : i;
			}
			//ROS_INFO("%s %d: min: %d", __FUNCTION__, __LINE__, min);
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
			} else if (is_brush_block_unclean(curr.X + i, curr.Y))
			{
				// Find the furthest unclean cell.
				max = is_block_boundary(curr.X + i + 1, curr.Y) ? max : i;
			}
			//ROS_INFO("%s %d: max: %d", __FUNCTION__, __LINE__, max);
		}
	}

	ROS_WARN("%s %d: min: %d\tmax: %d", __FUNCTION__, __LINE__, curr.X - min, curr.X + max);
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

	if (is_found == 1)
	{
//		if (g_cell_history[0] == g_cell_history[1] && g_cell_history[0] == g_cell_history[2])
			// Duplicated cleaning.
//			is_found = 0;
//		else
		{
#if !INTERLACED_MOVE
			uint8_t un_cleaned_cnt = 0;
			for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
				for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
					if (map_get_cell(MAP, tmp.X + dx, tmp.Y + dy) == UNCLEAN)
						un_cleaned_cnt++;

			if (un_cleaned_cnt < 4)
				// Uncleaned area is too small.
				is_found = 0;
#endif
		}
	}

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
		if (is_found)
		temp_path.push_back(path.cells.front());
		if (is_found)
		temp_path.push_back(path.cells.back());
		if (is_found)
		path_display_path_points(temp_path);
		if (is_found)
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

void path_find_all_targets()
{
	bool		all_set;
	int16_t		i, j, x, y, offset, passValue, nextPassValue, passSet, tracex, tracey, targetCost;
	CellState	cs;

	map_reset(SPMAP);

	for (i = g_x_min; i <= g_x_max; ++i) {
		for (j = g_y_min; j <= g_y_max; ++j) {
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

	if (g_trapped_mode == 1)
	{
		x = map_get_x_cell();
		y = map_get_y_cell();
	}
	else
	{
		x = g_cell_history[0].X;
		y = g_cell_history[0].Y;
	}

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
			if (i < g_x_min || i > g_x_max)
				continue;

			for (j = y - offset; j <= y + offset; j++) {
				if (j < g_y_min || j > g_y_max)
					continue;

				if(map_get_cell(SPMAP, i, j) == passValue) {
					if (i - 1 >= g_x_min && map_get_cell(SPMAP, i - 1, j) == COST_NO) {
						map_set_cell(SPMAP, (i - 1), (j), (CellState) nextPassValue);
						passSet = 1;
					}

					if ((i + 1) <= g_x_max && map_get_cell(SPMAP, i + 1, j) == COST_NO) {
						map_set_cell(SPMAP, (i + 1), (j), (CellState) nextPassValue);
						passSet = 1;
					}

					if (j - 1  >= g_y_min && map_get_cell(SPMAP, i, j - 1) == COST_NO) {
						map_set_cell(SPMAP, (i), (j - 1), (CellState) nextPassValue);
						passSet = 1;
					}

					if ((j + 1) <= g_y_max && map_get_cell(SPMAP, i, j + 1) == COST_NO) {
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
	//debug_map(SPMAP, 0, 0);

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

			if ((tracex - 1 >= g_x_min) && (map_get_cell(SPMAP, tracex - 1, tracey) == targetCost)) {
				tracex--;
				continue;
			}

			if ((tracex + 1 <= g_x_max) && (map_get_cell(SPMAP, tracex + 1, tracey) == targetCost)) {
				tracex++;
				continue;
			}

			if ((tracey - 1 >= g_y_min) && (map_get_cell(SPMAP, tracex, tracey - 1) == targetCost)) {
				tracey--;
				continue;
			}

			if ((tracey + 1 <= g_y_max) && (map_get_cell(SPMAP, tracex, tracey + 1) == targetCost)) {
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
//#if !INTERLACED_MOVE
//int16_t path_target(const Cell_t& curr, PPTargetType& path)
//{
//	bool	within_range;
//	int16_t found, final_cost, a, b, c, d, start, end, last_y;
//	int16_t x_min, y_min, x_max, y_max, x_min_tmp, y_min_tmp, x_max_tmp, y_max_tmp, stop;
//	Cell_t temp_target;
//
//	final_cost = 1000;
//	found = 0;
//	x_max = y_max = x_max_tmp = y_max_tmp = SHRT_MIN;
//	x_min = y_min = x_min_tmp = y_min_tmp = SHRT_MAX;
//
//	for (c = g_x_min - 1; c < g_x_max + 1; ++c) {
//		for (d = g_y_min - 1; d < g_y_max + 1; ++d) {
//			if (map_get_cell(MAP, c, d) != UNCLEAN) {
//				x_min_tmp = x_min_tmp > c ? c : x_min_tmp;
//				x_max_tmp = x_max_tmp < c ? c : x_max_tmp;
//				y_min_tmp = y_min_tmp > d ? d : y_min_tmp;
//				y_max_tmp = y_max_tmp < d ? d : y_max_tmp;
//			}
//		}
//	}
//
//	for (c = x_min_tmp; c <= x_max_tmp; ++c) {
//		for (d = y_min_tmp; d <= y_max_tmp; ++d) {
//			if (map_get_cell(MAP, c, d) != CLEANED)
//				continue;
//
//			if (c > g_x_min - 1 && map_get_cell(MAP, c - 1, d) == UNCLEAN) {
//				if (is_block_accessible(c - 1, d) == 1) {
//					map_set_cell(MAP, cell_to_count(c - 1), cell_to_count(d), TARGET);
//					x_min = x_min > (c - 1) ? (c - 1) : x_min;
//					x_max = x_max < (c - 1) ? (c - 1) : x_max;
//					y_min = y_min > d ? d : y_min;
//					y_max = y_max < d ? d : y_max;
//				}
//			}
//
//			if (c < g_x_max + 1 && map_get_cell(MAP, c + 1, d) == UNCLEAN) {
//				if (is_block_accessible(c + 1, d) == 1) {
//					map_set_cell(MAP, cell_to_count(c + 1), cell_to_count(d), TARGET);
//					x_min = x_min > (c + 1) ? (c + 1) : x_min;
//					x_max = x_max < (c + 1) ? (c + 1) : x_max;
//					y_min = y_min > d ? d : y_min;
//					y_max = y_max < d ? d : y_max;
//				}
//			}
//
//			if (d > g_y_min - 1 && map_get_cell(MAP, c, d - 1) == UNCLEAN) {
//				if (is_block_accessible(c, d - 1) == 1) {
//					map_set_cell(MAP, cell_to_count(c), cell_to_count(d - 1), TARGET);
//					x_min = x_min > c ? c : x_min;
//					x_max = x_max < c ? c : x_max;
//					y_min = y_min > (d - 1) ? (d - 1) : y_min;
//					y_max = y_max < (d - 1) ? (d - 1) : y_max;
//				}
//			}
//
//			if (d < g_y_max + 1 && map_get_cell(MAP, c, d + 1) == UNCLEAN) {
//				if (is_block_accessible(c, d + 1) == 1) {
//					map_set_cell(MAP, cell_to_count(c), cell_to_count(d + 1), TARGET);
//					x_min = x_min > c ? c : x_min;
//					x_max = x_max < c ? c : x_max;
//					y_min = y_min > (d + 1) ? (d + 1) : y_min;
//					y_max = y_max < (d + 1) ? (d + 1) : y_max;
//				}
//			}
//		}
//	}
//
//	/* Narrow down the coodinate that robot should go */
//	for (d = y_min; d <= y_max; d++) {
//		start = end = SHRT_MAX;
//		for (c = x_min; c <= map_get_x_cell(); c++) {
//			if (map_get_cell(MAP, c, d) == TARGET) {
//				if (start == SHRT_MAX)
//					start = c;
//				if (start != SHRT_MAX)
//					end = c;
//			}
//			if (map_get_cell(MAP, c, d) != TARGET || c == map_get_x_cell()){
//				if ( start != SHRT_MAX) {
//					for (a = start + 1; a < end; a++) {
//						map_set_cell(MAP, cell_to_count(a), cell_to_count(d), UNCLEAN);
//					}
//				}
//				start = end = SHRT_MAX;
//			}
//		}
//
//		start = end = SHRT_MIN;
//		for (c = x_max; c >= map_get_x_cell(); c--) {
//			if (map_get_cell(MAP, c, d) == TARGET) {
//				if (start == SHRT_MIN)
//					start = c;
//				if (start != SHRT_MIN)
//					end = c;
//			}
//			if (map_get_cell(MAP, c, d) != TARGET || c == map_get_x_cell()){
//				if (end != SHRT_MIN) {
//					for (a = start - 1; a > end; a--) {
//						map_set_cell(MAP, cell_to_count(a), cell_to_count(d), UNCLEAN);
//					}
//				}
//				start = end = SHRT_MIN;
//			}
//		}
//	}
//
//	x_min_tmp = x_min;
//	x_max_tmp = x_max;
//	y_min_tmp = y_min;
//	y_max_tmp = y_max;
//
//	for (auto& target : g_targets)
//		target.cells.clear();
//	g_targets.clear();
//
//	for (c = x_min_tmp; c <= x_max_tmp; ++c) {
//		for (d = y_min_tmp; d <= y_max_tmp; ++d) {
//			if (map_get_cell(MAP, c, d) == TARGET) {
//				PPTargetType t;
//				t.target.X = c;
//				t.target.Y = d;
//				t.cells.clear();
//				g_targets.push_back(t);
//				if (t.target == map_get_curr_cell())
//				{
//					ROS_ERROR("%s %d: Target is current cell, map_set_realtime +-1 cells to CLEANED.", __FUNCTION__, __LINE__);
//					ROS_WARN("Before:");
//					for (auto dx = t.target.X - 1; dx <= t.target.X + 1; ++dx)
//					{
//						for (auto dy = t.target.Y - 1; dy <= t.target.Y + 1; ++dy)
//							printf("%d ", map_get_cell(MAP, dx, dy));
//						printf("\n");
//					}
//					ROS_WARN("After:");
//					for (auto dx = t.target.X - 1; dx <= t.target.X + 1; ++dx)
//					{
//						for (auto dy = t.target.Y - 1; dy <= t.target.Y + 1; ++dy)
//						{
//							if (map_get_cell(MAP, dx, dy) == UNCLEAN)
//								map_set_cell(MAP, dx, dy, CLEANED);
//							printf("%d ", map_get_cell(MAP, dx, dy));
//						}
//						printf("\n");
//					}
//				}
//
//				x_min = x_min > c ? c : x_min;
//				x_max = x_max < c ? c : x_max;
//				y_min = y_min > d ? d : y_min;
//				y_max = y_max < d ? d : y_max;
//			}
//		}
//	}
//
//	debug_map(MAP, g_home_x, g_home_y);
//	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//		map_set_cell(MAP, cell_to_count(it->target.X), cell_to_count(it->target.Y), UNCLEAN);
//	}
//
//	path_find_all_targets();
//
//	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end();) {
//		if (it->cells.empty() == true) {
//			it = g_targets.erase(it);
//		} else {
//			it++;
//		}
//	}
//
//	/* No more target to clean */
//	if (g_targets.empty() == true) {
//		if (path_escape_trapped() <= 0) {
//			ROS_WARN("%s %d: trapped", __FUNCTION__, __LINE__);
//			return -2;
//		}
//		ROS_INFO("%s %d: targets list empty.", __FUNCTION__, __LINE__);
//		return 0;
//	}
//
//	ROS_INFO("%s %d: targets count: %d", __FUNCTION__, __LINE__, (int)g_targets.size());
//	//for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//	//	std::string	msg = __FUNCTION__;
//	//	msg += " " + std::to_string(__LINE__) + ": target (" + std::to_string(it->target.X) + ", " + std::to_string(it->target.Y) + ") " + std::to_string(it->cells.size()) + ": ";
//
//	//	for (list<Cell_t>::iterator i = it->cells.begin(); i != it->cells.end(); ++i) {
//	//		msg += "(" + std::to_string(i->X) + ", " + std::to_string(i->Y) + ")->";
//	//	}
//	//	msg += "\n";
//	//	ROS_INFO("%s",msg.c_str());
//	//}
//
//#if 1
//	stop = 0;
//	ROS_INFO("%s %d: case 1, towards Y+ only", __FUNCTION__, __LINE__);
//	for (d = y_max; d >= map_get_y_cell(); --d) {
//		if (stop == 1 && d <= map_get_y_cell() + 1) {
//			break;
//		}
//
//		for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//			if (map_get_cell(MAP, it->target.X, it->target.Y - 1) != CLEANED) {
//				continue;
//			}
//
//			if (it->target.Y == d) {
//				if (it->cells.size() > final_cost) {
//					continue;
//				}
//
//				last_y = it->cells.front().Y;
//				within_range = true;
//				for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//					if (i->Y < map_get_y_cell() || i->Y > d) {
//						within_range = false;
//					}
//					if (i->Y > last_y) {
//						within_range = false;
//					} else {
//						last_y = i->Y;
//					}
//				}
//				if (within_range == true) {
//					temp_target.X = it->target.X;
//					temp_target.Y = it->target.Y;
//					final_cost = it->cells.size();
//					stop = 1;
//				}
//			}
//		}
//	}
//
//	ROS_INFO("%s %d: case 2, towards Y+, allow Y- shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//	if (stop == 0) {
//		for (a = map_get_y_cell(); a >= y_min && stop == 0; --a) {
//			for (d = a; d <= y_max && stop == 0; ++d) {
//				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//					if (it->target.Y == d) {
//						if (it->cells.size() > final_cost) {
//							continue;
//						}
//
//						within_range = true;
//						last_y = it->cells.front().Y;
//						bool turn = false;
//						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//							if (i->Y < a || i->Y > (d > map_get_y_cell() ? d : map_get_y_cell())) {
//								within_range = false;
//							}
//							if (turn == false) {
//								if (i->Y > last_y) {
//									within_range = false;
//								} else {
//									last_y = i->Y;
//								}
//							} else {
//								if (i->Y < last_y) {
//									within_range = false;
//								} else {
//									last_y = i->Y;
//								}
//							}
//							if (i->Y == a) {
//								turn = true;
//							}
//						}
//						if (within_range == true) {
//							temp_target.X = it->target.X;
//							temp_target.Y = it->target.Y;
//							final_cost = it->cells.size();
//							stop = 1;
//						}
//					}
//				}
//			}
//		}
//	}
//
//	ROS_INFO("%s %d: case 3, towards Y- only, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//	if (stop == 0) {
//		for (d = y_min; d >= map_get_y_cell(); ++d) {
//			if (stop == 1 && d >= map_get_y_cell() - 1) {
//				break;
//			}
//
//			for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//				if (map_get_cell(MAP, it->target.X, it->target.Y + 1) != CLEANED) {
//					continue;
//				}
//
//				if (it->target.Y == d) {
//					if (it->cells.size() > final_cost) {
//						continue;
//					}
//
//					last_y = it->cells.front().Y;
//					within_range = true;
//					for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//						if (i->Y > map_get_y_cell() || i->Y < d) {
//							within_range = false;
//						}
//						if (i->Y < last_y) {
//							within_range = false;
//						} else {
//							last_y = i->Y;
//						}
//					}
//					if (within_range == true) {
//						temp_target.X = it->target.X;
//						temp_target.Y = it->target.Y;
//						final_cost = it->cells.size();
//						stop = 1;
//					}
//				}
//			}
//		}
//	}
//
//	ROS_INFO("%s %d: case 4, towards Y-, allow Y+ shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//	if (stop == 0) {
//		for (a = map_get_y_cell(); a <= y_max && stop == 0; ++a) {
//			for (d = a; d >= y_min && stop == 0; --d) {
//				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//					if (it->target.Y == d) {
//						if (it->cells.size() > final_cost) {
//							continue;
//						}
//
//						within_range = true;
//						last_y = it->cells.front().Y;
//						bool turn = false;
//						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//							if (i->Y > a || i->Y < (d > map_get_y_cell() ? map_get_y_cell() : d)) {
//								within_range = false;
//							}
//							if (turn == false) {
//								if (i->Y < last_y) {
//									within_range = false;
//								} else {
//									last_y = i->Y;
//								}
//							} else {
//								if (i->Y > last_y) {
//									within_range = false;
//								} else {
//									last_y = i->Y;
//								}
//							}
//							if (i->Y == a) {
//								turn = true;
//							}
//						}
//						if (within_range == true) {
//							temp_target.X = it->target.X;
//							temp_target.Y = it->target.Y;
//							final_cost = it->cells.size();
//							stop = 1;
//						}
//					}
//				}
//			}
//		}
//	}
//	ROS_INFO("%s %d: case 5: towards Y+, allow Y- shift, allow turns, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//	if (stop == 0) {
//		for (a = map_get_y_cell(); a <= y_max  && stop == 0; ++a) {
//	            for (d = map_get_y_cell(); d <= a && stop == 0; ++d) {
//				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//					if (it->target.Y == d) {
//						within_range = true;
//						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//							if (i->Y < map_get_y_cell() || i->Y > a) {
//								within_range = false;
//							}
//						}
//						if (within_range == true && it->cells.size() < final_cost) {
//							temp_target.X = it->target.X;
//							temp_target.Y = it->target.Y;
//							final_cost = it->cells.size();
//							stop = 1;
//						}
//					}
//				}
//			}
//		}
//	}
//#endif
//	ROS_INFO("%s %d: case 6, fallback to A-start the nearest target, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//	/* fallback to find unclean area */
//	if (stop == 0) {
//		for (c = x_min; c <= x_max; ++c) {
//			for (d = y_min; d <= y_max; ++d) {
//				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//					if (it->cells.size() < final_cost) {
//						temp_target.X = it->target.X;
//						temp_target.Y = it->target.Y;
//						final_cost = it->cells.size();
//					}
//				}
//			}
//		}
//	}
//
//	found = (final_cost != 1000) ? final_cost : 0 ;
//	ROS_INFO("%s %d: found: %d (%d, %d)\n", __FUNCTION__, __LINE__, found, temp_target.X, temp_target.Y);
//	debug_map(MAP, temp_target.X, temp_target.Y);
//
//	if(found == 0)
//		return 0;
//
//	set_explore_new_path_flag(true);
//	return path_next_best(curr, temp_target, path);
//}
//#else
//int16_t path_target(const Cell_t& curr, PPTargetType& path)
//{
//	bool	within_range;
//	int16_t found, final_cost, a, b, c, d, start, end, last_y;
//	int16_t x_min, y_min, x_max, y_max, x_min_tmp, y_min_tmp, x_max_tmp, y_max_tmp, stop;
//	Cell_t temp_target;
//
//	final_cost = 1000;
//	found = 0;
//	x_max = y_max = x_max_tmp = y_max_tmp = SHRT_MIN;
//	x_min = y_min = x_min_tmp = y_min_tmp = SHRT_MAX;
//
//	for (c = g_x_min - 1; c < g_x_max + 1; ++c) {
//		for (d = g_y_min - 1; d < g_y_max + 1; ++d) {
//			if (map_get_cell(MAP, c, d) != UNCLEAN) {
//				x_min_tmp = x_min_tmp > c ? c : x_min_tmp;
//				x_max_tmp = x_max_tmp < c ? c : x_max_tmp;
//				y_min_tmp = y_min_tmp > d ? d : y_min_tmp;
//				y_max_tmp = y_max_tmp < d ? d : y_max_tmp;
//			}
//		}
//	}
//
//	for (auto cell_X = x_min_tmp; cell_X <= x_max_tmp; ++cell_X) {
//		for (auto cell_Y = y_min_tmp; cell_Y <= y_max_tmp; ++cell_Y) {
//			{
//				if (map_get_cell(MAP, cell_X, cell_Y) != CLEANED /*|| std::abs(cell.Y % 2) == 1*/)
//					continue;
//				Cell_t neighbor[4];
//				neighbor[0] = {int16_t(cell_X - 1), cell_Y};
//				neighbor[1] = {int16_t(cell_X + 1), cell_Y};
//				neighbor[2] = {cell_X, int16_t(cell_Y - 1)};
//				neighbor[3] = {cell_X, int16_t(cell_Y + 1)};
//				for (auto i = 0; i < 4; i++) {
//					if (std::abs(neighbor[i].Y % 2) == 0) {
////						if(i == 0 || i==1)
////							(curr.Y > cell_Y) ? neighbor[i].Y++ : neighbor[i].Y--;
////						else if (i == 2)
////							neighbor[i].Y--;
////						else if (i == 3)
////							neighbor[i].Y++;
//					if (map_get_cell(MAP, neighbor[i].X, neighbor[i].Y) == UNCLEAN) {
//						if (is_block_accessible(neighbor[i].X, neighbor[i].Y) == 1) {
//							map_set_cell(MAP, cell_to_count(neighbor[i].X), cell_to_count(neighbor[i].Y), TARGET);
//							x_min = x_min_tmp > cell_X ? cell_X : x_min_tmp;
//							x_max = x_max_tmp < cell_X ? cell_X : x_max_tmp;
//							y_min = y_min_tmp > cell_Y ? cell_Y : y_min_tmp;
//							y_max = y_max_tmp < cell_Y ? cell_Y : y_max_tmp;
//						}
//					}
//					}
//				}
//			}
//		}
//	}
//
//  debug_map(MAP, g_home_x, g_home_y);
//	/* Narrow down the coodinate that robot should go */
//	for (d = y_min; d <= y_max; d++) {
//		start = end = SHRT_MAX;
//		for (c = x_min; c <= map_get_x_cell(); c++) {
//			if (map_get_cell(MAP, c, d) == TARGET) {
//				if (start == SHRT_MAX)
//					start = c;
//				if (start != SHRT_MAX)
//					end = c;
//			}
//			if (map_get_cell(MAP, c, d) != TARGET || c == map_get_x_cell()){
//				if ( start != SHRT_MAX) {
//					for (a = start + 1; a < end; a++) {
//						map_set_cell(MAP, cell_to_count(a), cell_to_count(d), UNCLEAN);
//					}
//				}
//				start = end = SHRT_MAX;
//			}
//		}
//
//		start = end = SHRT_MIN;
//		for (c = x_max; c >= curr.X; c--) {
//			if (map_get_cell(MAP, c, d) == TARGET) {
//				if (start == SHRT_MIN)
//					start = c;
//				if (start != SHRT_MIN)
//					end = c;
//			}
//			if (map_get_cell(MAP, c, d) != TARGET || c == map_get_x_cell()){
//				if (end != SHRT_MIN) {
//					for (a = start - 1; a > end; a--) {
//						map_set_cell(MAP, cell_to_count(a), cell_to_count(d), UNCLEAN);
//					}
//				}
//				start = end = SHRT_MIN;
//			}
//		}
//	}
//
//	x_min_tmp = x_min;
//	x_max_tmp = x_max;
//	y_min_tmp = y_min;
//	y_max_tmp = y_max;
//
//	for (auto& target : g_targets)
//		target.cells.clear();
//	g_targets.clear();
//
//	for (c = x_min_tmp; c <= x_max_tmp; ++c) {
//		for (d = y_min_tmp; d <= y_max_tmp; ++d) {
//			if (map_get_cell(MAP, c, d) == TARGET) {
//				PPTargetType t;
//				t.target.X = c;
//				t.target.Y = d;
//				t.cells.clear();
//				g_targets.push_back(t);
//				if (t.target == map_get_curr_cell())
//				{
//					ROS_ERROR("%s %d: Target is current cell, map_set_realtime +-1 cells to CLEANED.", __FUNCTION__, __LINE__);
//					ROS_WARN("Before:");
//					for (auto dx = t.target.X - 1; dx <= t.target.X + 1; ++dx)
//					{
//						for (auto dy = t.target.Y - 1; dy <= t.target.Y + 1; ++dy)
//							printf("%d ", map_get_cell(MAP, dx, dy));
//						printf("\n");
//					}
//					ROS_WARN("After:");
//					for (auto dx = t.target.X - 1; dx <= t.target.X + 1; ++dx)
//					{
//						for (auto dy = t.target.Y - 1; dy <= t.target.Y + 1; ++dy)
//						{
//							if (map_get_cell(MAP, dx, dy) == UNCLEAN)
//								map_set_cell(MAP, dx, dy, CLEANED);
//							printf("%d ", map_get_cell(MAP, dx, dy));
//						}
//						printf("\n");
//					}
//				}
//
//				x_min = x_min > c ? c : x_min;
//				x_max = x_max < c ? c : x_max;
//				y_min = y_min > d ? d : y_min;
//				y_max = y_max < d ? d : y_max;
//			}
//		}
//	}
//
//	debug_map(MAP, g_home_x, g_home_y);
//	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//		map_set_cell(MAP, cell_to_count(it->target.X), cell_to_count(it->target.Y), UNCLEAN);
//	}
//
//	path_find_all_targets();
//
//	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end();) {
//		if (it->cells.empty() == true) {
//			it = g_targets.erase(it);
//		} else {
//			it++;
//		}
//	}
//
//	/* No more target to clean */
//	if (g_targets.empty() == true) {
//		if (path_escape_trapped() <= 0) {
//			ROS_WARN("%s %d: trapped", __FUNCTION__, __LINE__);
//			return -2;
//		}
//		ROS_INFO("%s %d: targets list empty.", __FUNCTION__, __LINE__);
//		return 0;
//	}
//
//	ROS_INFO("%s %d: targets count: %d", __FUNCTION__, __LINE__, (int)g_targets.size());
//	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//		std::string	msg = __FUNCTION__;
//		msg += " " + std::to_string(__LINE__) + ": target (" + std::to_string(it->target.X) + ", " + std::to_string(it->target.Y) + ") " + std::to_string(it->cells.size()) + ": ";
//
//		for (list<Cell_t>::iterator i = it->cells.begin(); i != it->cells.end(); ++i) {
//			msg += "(" + std::to_string(i->X) + ", " + std::to_string(i->Y) + ")->";
//		}
//		msg += "\n";
//		ROS_INFO("%s",msg.c_str());
//	}
//
//#if 1
//	stop = 0;
//	ROS_INFO("%s %d: case 1, towards Y+ only", __FUNCTION__, __LINE__);
//	for (d = y_max; d >= curr.Y; --d) {
//		if (stop == 1 && d <= curr.Y + 1) {
//			break;
//		}
//
//		for (auto it = g_targets.begin(); it != g_targets.end(); ++it) {
//			if (map_get_cell(MAP, it->target.X, it->target.Y - 1) != CLEANED) {
//				continue;
//			}
//
//			if (it->target.Y == d) {
//				if (it->cells.size() > final_cost) {
//					continue;
//				}
//
//				last_y = it->cells.front().Y;
//				within_range = true;
//				for (auto i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//					if (i->Y < map_get_y_cell() || i->Y > d) {
//						within_range = false;
//					}
//					if (i->Y > last_y) {
//						within_range = false;
//					} else {
//						last_y = i->Y;
//					}
//				}
//				if (within_range == true) {
//					temp_target.X = it->target.X;
//					temp_target.Y = it->target.Y;
//					final_cost = it->cells.size();
//					stop = 1;
//				}
//			}
//		}
//	}
//
//	if (stop == 0) {
//      ROS_INFO("%s %d: case 2, towards Y+, allow Y- shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//		for (a = map_get_y_cell(); a >= y_min && stop == 0; --a) {
//			for (d = a; d <= y_max && stop == 0; ++d) {
//				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//					if (it->target.Y == d) {
//						if (it->cells.size() > final_cost) {
//							continue;
//						}
//
//						within_range = true;
//						last_y = it->cells.front().Y;
//						bool turn = false;
//						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//							if (i->Y < a || i->Y > (d > map_get_y_cell() ? d : map_get_y_cell())) {
//								within_range = false;
//							}
//							if (turn == false) {
//								if (i->Y > last_y) {
//									within_range = false;
//								} else {
//									last_y = i->Y;
//								}
//							} else {
//								if (i->Y < last_y) {
//									within_range = false;
//								} else {
//									last_y = i->Y;
//								}
//							}
//							if (i->Y == a) {
//								turn = true;
//							}
//						}
//						if (within_range == true) {
//							temp_target.X = it->target.X;
//							temp_target.Y = it->target.Y;
//							final_cost = it->cells.size();
//							stop = 1;
//						}
//					}
//				}
//			}
//		}
//	}
//
//	if (stop == 0) {
//      ROS_INFO("%s %d: case 3, towards Y- only, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//		for (d = y_min; d >= map_get_y_cell(); ++d) {
//			if (stop == 1 && d >= map_get_y_cell() - 1) {
//				break;
//			}
//
//			for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//				if (map_get_cell(MAP, it->target.X, it->target.Y + 1) != CLEANED) {
//					continue;
//				}
//
//				if (it->target.Y == d) {
//					if (it->cells.size() > final_cost) {
//						continue;
//					}
//
//					last_y = it->cells.front().Y;
//					within_range = true;
//					for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//						if (i->Y > map_get_y_cell() || i->Y < d) {
//							within_range = false;
//						}
//						if (i->Y < last_y) {
//							within_range = false;
//						} else {
//							last_y = i->Y;
//						}
//					}
//					if (within_range == true) {
//						temp_target.X = it->target.X;
//						temp_target.Y = it->target.Y;
//						final_cost = it->cells.size();
//						stop = 1;
//					}
//				}
//			}
//		}
//	}
//
//	if (stop == 0) {
//      ROS_INFO("%s %d: case 4, towards Y-, allow Y+ shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//		for (a = map_get_y_cell(); a <= y_max && stop == 0; ++a) {
//			for (d = a; d >= y_min && stop == 0; --d) {
//				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//					if (it->target.Y == d) {
//						if (it->cells.size() > final_cost) {
//							continue;
//						}
//
//						within_range = true;
//						last_y = it->cells.front().Y;
//						bool turn = false;
//						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//							if (i->Y > a || i->Y < (d > map_get_y_cell() ? map_get_y_cell() : d)) {
//								within_range = false;
//							}
//							if (turn == false) {
//								if (i->Y < last_y) {
//									within_range = false;
//								} else {
//									last_y = i->Y;
//								}
//							} else {
//								if (i->Y > last_y) {
//									within_range = false;
//								} else {
//									last_y = i->Y;
//								}
//							}
//							if (i->Y == a) {
//								turn = true;
//							}
//						}
//						if (within_range == true) {
//							temp_target.X = it->target.X;
//							temp_target.Y = it->target.Y;
//							final_cost = it->cells.size();
//							stop = 1;
//						}
//					}
//				}
//			}
//		}
//	}
//	if (stop == 0) {
//      ROS_INFO("%s %d: case 5: towards Y+, allow Y- shift, allow turns, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//		for (a = map_get_y_cell(); a <= y_max  && stop == 0; ++a) {
//	            for (d = map_get_y_cell(); d <= a && stop == 0; ++d) {
//				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//					if (it->target.Y == d) {
//						within_range = true;
//						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
//							if (i->Y < map_get_y_cell() || i->Y > a) {
//								within_range = false;
//							}
//						}
//						if (within_range == true && it->cells.size() < final_cost) {
//							temp_target.X = it->target.X;
//							temp_target.Y = it->target.Y;
//							final_cost = it->cells.size();
//							stop = 1;
//						}
//					}
//				}
//			}
//		}
//	}
//#endif
//	/* fallback to find unclean area */
//	if (stop == 0) {
//      ROS_INFO("%s %d: case 6, fallback to A-start the nearest target, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
//		for (c = x_min; c <= x_max; ++c) {
//			for (d = y_min; d <= y_max; ++d) {
//				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
//					if (it->cells.size() < final_cost) {
//						temp_target.X = it->target.X;
//						temp_target.Y = it->target.Y;
//						final_cost = it->cells.size();
//					}
//				}
//			}
//		}
//	}
//
//	found = (final_cost != 1000) ? final_cost : 0 ;
//	ROS_INFO("%s %d: found: %d (%d, %d)\n", __FUNCTION__, __LINE__, found, temp_target.X, temp_target.Y);
//	debug_map(MAP, temp_target.X, temp_target.Y);
//
//	if(found == 0)
//		return 0;
//
//	set_explore_new_path_flag(true);
//	return path_next_best(curr, temp_target, path);
//}
//#endif

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
	ROS_WARN("target:(%d,%d)~~~~~~~~~~~~~~~~~~~~",it.target.X, it.target.Y);
	return true;
}

static int16_t path_target_next_line(const Cell_t &curr, int16_t y_min, int16_t y_max, Cell_t &target)
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
		ROS_WARN("%s %d: case %d, TOWARDS(%d), allow turn count: %d ", __FUNCTION__, __LINE__, i, area[i][TOWARDS], area[i][TURN]);
		do {
			ROS_INFO(" %s %d: y: towards_y_end(%d), re_towards_y_end(%d), area_y_begin(%d),", __FUNCTION__, __LINE__,towards_y_end, re_towards_y_end, area_y_begin);
			for (auto target_y = area_y_begin; target_y != towards_y_end; target_y += dy)
			{
				ROS_INFO(" %s %d: target_y(%d)", __FUNCTION__, __LINE__, target_y);
				for (const auto &target_it : g_targets)
				{
					if (target_y == target_it.target.Y)
					{
						if (is_path_valid(target_it, towards_y, re_towards_y, target_y, curr.Y, area[i][TURN]) &&
								cost > target_it.cells.size())
						{
							target = target_it.target;
							cost = target_it.cells.size();
              ROS_INFO(" %s %d: target_y(%d),cost %d", __FUNCTION__, __LINE__, target_y,cost);
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
//
int16_t path_target(const Cell_t& curr, PPTargetType& path)
{
//	return 0;
	BoundingBox2 map;
	BoundingBox2 map_tmp{{int16_t(g_x_min-1),int16_t(g_y_min - 1)},{g_x_max,g_y_max}};
	Cell_t target_tmp;

	ROS_INFO("tmp: min(%d,%d),max(%d,%d)", map_tmp.min.X, map_tmp.min.Y, map_tmp.max.X, map_tmp.max.Y);
	for (const auto& cell : map_tmp)
	{
		if (map_get_cell(MAP, cell.X, cell.Y) != UNCLEAN)
			map.Add(cell);
	}
	ROS_INFO("map: min(%d,%d),max(%d,%d)", map.min.X, map.min.Y, map.max.X, map.max.Y);
	ROS_INFO("tmp: min(%d,%d),max(%d,%d)", map_tmp.min.X, map_tmp.min.Y, map_tmp.max.X, map_tmp.max.Y);

	map_tmp = map;
	for (const auto& cell : map_tmp)
	{
		if (map_get_cell(MAP, cell.X, cell.Y) != CLEANED /*|| std::abs(cell.Y % 2) == 1*/)
			continue;
		Cell_t neighbor[4];
		neighbor[0] = {int16_t(cell.X - 1), cell.Y};
		neighbor[1] = {int16_t(cell.X + 1), cell.Y};
		neighbor[2] = {cell.X, int16_t(cell.Y - 1)};
		neighbor[3] = {cell.X, int16_t(cell.Y + 1)};
		for (auto i = 0; i < 4; i++)
		{
			if (map_get_cell(MAP, neighbor[i].X, neighbor[i].Y) == UNCLEAN)
			{
				if (is_block_accessible(neighbor[i].X, neighbor[i].Y) == 1)
				{
					if(std::abs(neighbor[i].Y % 2) == 1) neighbor[i].Y = cell.Y;
					if(std::abs(neighbor[i].Y % 2) == 0){
						map_set_cell(MAP, cell_to_count(neighbor[i].X), cell_to_count(neighbor[i].Y), TARGET);
						map.Add({neighbor[i].X, neighbor[i].Y});
					}
				}
			}
		}
	}

	ROS_INFO("map: min(%d,%d),max(%d,%d)", map.min.X, map.min.Y, map.max.X, map.max.Y);
	debug_map(MAP, g_home_x, g_home_y);
//	return 0;
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

	debug_map(MAP, g_home_x, g_home_y);

	for (auto &target : g_targets)
		target.cells.clear();
	g_targets.clear();
	for(const auto& cell : map)
	{
		if (map_get_cell(MAP, cell.X, cell.Y) == TARGET)
		{
			PPTargetType t;
			t.target = cell;
			t.cells.clear();
			g_targets.push_back(t);
		}
	}

	for (const auto &it : g_targets)
		map_set_cell(MAP, cell_to_count(it.target.X), cell_to_count(it.target.Y), UNCLEAN);

	path_find_all_targets();

	for (auto it = g_targets.begin(); it != g_targets.end();)
	{
		if (it->cells.empty() || it->cells.size() > FINAL_COST)
			it = g_targets.erase(it);
		else
			it++;
	}

//	 No more target_tmp to clean
	if (g_targets.empty())
	{
		if (path_escape_trapped() <= 0)
		{
			ROS_WARN("%s %d: trapped", __FUNCTION__, __LINE__);
			return -2;
		}
		ROS_INFO("%s %d: targets list empty.", __FUNCTION__, __LINE__);
		return 0;
	}

	ROS_INFO("%s %d: targets count: %d", __FUNCTION__, __LINE__, (int) g_targets.size());
//	for (auto it = g_targets.begin(); it != g_targets.end(); ++it) {
//		std::string	msg = __FUNCTION__;
//		msg += " " + std::to_string(__LINE__) + ": target_tmp (" + std::to_string(it->target.X) + ", " + std::to_string(it->target.Y) + ") " + std::to_string(it->cells.size()) + ": ";
//
//		for (list<Cell_t>::iterator i = it->cells.begin(); i != it->cells.end(); ++i) {
//			msg += "(" + std::to_string(i->X) + ", " + std::to_string(i->Y) + ")->";
//		}
//		msg += "\n";
//		ROS_INFO("%s",msg.c_str());
//	}

	ROS_INFO("map: min(%d,%d),max(%d,%d)", map.min.X, map.min.Y, map.max.X, map.max.Y);
	if(!path_target_next_line(curr, map.min.Y, map.max.Y, target_tmp)){

		return 0;
	}

	set_explore_new_path_flag(true);
	return path_next_best(curr, target_tmp, path);
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

int16_t path_escape_trapped()
{

	int16_t	val = 0;
	uint16_t i = 0;
	Cell_t temp_cell;

	if (g_trapped_mode == 1)
		temp_cell = map_get_curr_cell();
	else
		temp_cell = g_cell_history[0];

	set_explore_new_path_flag(true);
	if ( g_trapped_cell[0].X != g_home_x || g_trapped_cell[0].Y != g_home_y ){
		for ( i = 0; i < g_trapped_cell_size; ++i ) {
			ROS_WARN("%s %d Check %d trapped reference cell: x: %d, y: %d", __FUNCTION__, __LINE__,
			         i, g_trapped_cell[i].X, g_trapped_cell[i].Y);
			if (is_block_accessible(g_trapped_cell[i].X, g_trapped_cell[i].Y) == 0) {
				map_set_cells(ROBOT_SIZE, g_trapped_cell[i].X, g_trapped_cell[i].Y, CLEANED);
			}

			val = path_find_shortest_path(temp_cell.X, temp_cell.Y, g_trapped_cell[i].X, g_trapped_cell[i].Y, 0);
			ROS_WARN("%s %d: val %d", __FUNCTION__, __LINE__, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* No path to home, which is set when path planning is initialized. */
				val = 0;
			} else {
				val = 1;
				break;
			}
		}
	} else {
		if (is_block_accessible(0, 0) == 1) {
			val = path_find_shortest_path(temp_cell.X, temp_cell.Y, 0, 0, 0);
#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
//			ROS_WARN("%s %d: pos (%d, %d)\tval: %d", __FUNCTION__, __LINE__, g_cell_history[0].x, g_cell_history[0].y, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = path_find_shortest_path(temp_cell.X, temp_cell.Y, g_home_x, g_home_y, 0);
				ROS_WARN("%s %d: val %d", __FUNCTION__, __LINE__, val);

#if DEBUG_MAP
				debug_map(MAP, g_home_x, g_home_y);
#endif

				if (val < 0 || val == SCHAR_MAX) {
					val = 0;
				} else {
					val = 1;
				};
			} else {
				val = 1;
			}
		} else {
			val = path_find_shortest_path(temp_cell.X, temp_cell.Y, g_home_x, g_home_y, 0);
			ROS_WARN("%s %d: val %d", __FUNCTION__, __LINE__, val);

#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
#if DEBUG_MAP
			debug_map(MAP, g_home_x, g_home_y);
#endif

			if (val < 0 || val == SCHAR_MAX) {
				/* No path to home, which is set when path planning is initialized. */
				val = 0;
			} else {
				val = 1;
			}
		}
	}

	return val;
}

int8_t path_next(const Cell_t& curr, PPTargetType& path)
{
	//ros_map_convert(false);
	extern bool g_go_home;
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
		if (!SpotMovement::instance()->spotNextTarget(path.target))
			return 0;
		path.cells.clear();
		path.cells.push_front(path.target);
		path.cells.push_front(curr);
	}
	else if(!g_go_home && get_clean_mode() == Clean_Mode_Navigation) {
		if (g_resume_cleaning && path_get_continue_target(curr, path) != TARGET_FOUND)
			g_resume_cleaning = false;

		if (!g_resume_cleaning)
		{
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
		}
	}

	if (g_go_home && path_get_home_target(curr, path) == NO_TARGET_LEFT)
		return 0;

	// Delete the first cell of list, it means current cell. Do this for checking whether it should follow the wall.
	if (path.cells.size() > 1)
		path.cells.pop_front();
	g_next_cell = path.cells.front();
	g_target_cell = path.target;

	g_old_dir = g_new_dir;
	if (g_go_home || SpotMovement::instance()->getSpotType() != NO_SPOT)
		mt_set(CM_LINEARMOVE);
	else if(get_clean_mode() == Clean_Mode_Navigation)
		mt_update(curr, path, g_old_dir);
	// else if wall follow mode, the move type has been set before here.
	if (curr.X == g_next_cell.X)
		g_new_dir = curr.Y > g_next_cell.Y ? NEG_Y : POS_Y;
	else
		g_new_dir = curr.X > g_next_cell.X ? NEG_X : POS_X;

#if LINEAR_MOVE_WITH_PATH
	if (mt_is_linear())
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

	for (list<Cell_t>::iterator it = saved_path.begin(); it->X != saved_path.back().X || it->Y != saved_path.back().Y; it++)
	{
		list<Cell_t>::iterator next_it = it;
		next_it++; // Get the next turing point.
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
	for (list<Cell_t>::iterator it = path.begin(); it != path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ")->";
	}
	ROS_DEBUG("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());
}

void path_escape_set_trapped_cell( Cell_t *cell, uint8_t size )
{
	g_trapped_cell_size = size;
	for (auto i = 0; i < g_trapped_cell_size; ++i ) {
		g_trapped_cell[i] = cell[i];
		ROS_INFO("%s %d Set %d trapped reference cell: x: %d\ty:%d", __FUNCTION__, __LINE__, i, g_trapped_cell[i].X, g_trapped_cell[i].Y);
	}
}

Cell_t *path_escape_get_trapped_cell()
{
	return g_trapped_cell;
}

void path_set_home(Cell_t cell)
{
	bool found = false;

	ROS_INFO("%s %d: Push new reachable home: (%d, %d) to home point list.", __FUNCTION__, __LINE__, cell.X, cell.Y);

	for (list<Cell_t>::iterator it = g_home_point_old_path.begin(); found == false && it != g_home_point_old_path.end(); ++it) {
		if (it->X == cell.X && it->Y == cell.Y) {
			found = true;
		}
	}
	if (found == false) {
		extern bool g_have_seen_charge_stub;
		if(get_clean_mode() != Clean_Mode_Spot)
			g_have_seen_charge_stub = true;
		g_home_point_old_path.push_front(cell);
		// If cell near (0, 0)
		if (abs(cell.X) <= 5 && abs(cell.Y) <= 5)
		{

			// This g_temp_trapped_cell is for trapped reference point.
			auto g_temp_trapped_cell = path_escape_get_trapped_cell();

			// Update the trapped reference cells
			for (int8_t i = ESCAPE_TRAPPED_REF_CELL_SIZE - 1; i > 0; i--)
			{
				g_temp_trapped_cell[i] = g_temp_trapped_cell[i-1];
				ROS_DEBUG("i = %d, g_temp_trapped_cell[i].X = %d, g_temp_trapped_cell[i].Y = %d", i, g_temp_trapped_cell[i].X, g_temp_trapped_cell[i].Y);
			}
			g_temp_trapped_cell[0] = cell;
			ROS_DEBUG("g_temp_trapped_cell[0].X = %d, g_temp_trapped_cell[0].Y = %d", g_temp_trapped_cell[0].X, g_temp_trapped_cell[0].Y);
			path_escape_set_trapped_cell(g_temp_trapped_cell, ESCAPE_TRAPPED_REF_CELL_SIZE);
		}
	}
	else if(cell.X == 0 && cell.Y == 0 && get_clean_mode() != Clean_Mode_Spot)
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
int8_t path_get_home_target(const Cell_t& curr, PPTargetType& path)
{
	int8_t return_val;
	Cell_t temp_target;
	static bool going_old_paths = true;
	while (ros::ok())
	{
		if (g_switch_home_cell)
		{
			// Get the home point.
			if (!g_home_point_old_path.empty())
			{
				// Get next home cell.
				temp_target = g_home_point_old_path.front();
				g_home_point_old_path.pop_front();
				ROS_WARN("%s, %d: Go home Target: (%d, %d), %u old path targets left, %u new targets left.", __FUNCTION__, __LINE__, temp_target.X, temp_target.Y, (uint)g_home_point_old_path.size(), (uint)g_home_point_new_path.size());
				// Try all the old path home point first.
				set_explore_new_path_flag(false);
				g_switch_home_cell = false;
				going_old_paths = true;

				if (is_block_accessible(temp_target.X, temp_target.Y) == 0) {
					ROS_WARN("%s %d: target is blocked, unblock the target.\n", __FUNCTION__, __LINE__);
					map_set_cells(ROBOT_SIZE, temp_target.X, temp_target.Y, CLEANED);
				}
			}
			else if (!g_home_point_new_path.empty())
			{
				// Try all the new path home point.
				set_explore_new_path_flag(true);
				// Get next home cell.
				temp_target = g_home_point_new_path.front();
				g_home_point_new_path.pop_front();
				ROS_WARN("%s, %d: Go home Target: (%d, %d), %u new targets left.", __FUNCTION__, __LINE__, temp_target.X, temp_target.Y, (uint)g_home_point_new_path.size());
				g_switch_home_cell = false;
				going_old_paths = false;

				if (is_block_accessible(temp_target.X, temp_target.Y) == 0) {
					ROS_WARN("%s %d: target is blocked, unblock the target.\n", __FUNCTION__, __LINE__);
					map_set_cells(ROBOT_SIZE, temp_target.X, temp_target.Y, CLEANED);
				}
			}
			else // Target list is empty.
			{
				ROS_WARN("%s, %d: No targets left.", __FUNCTION__, __LINE__);
				return_val = NO_TARGET_LEFT;
				break;
			}
		}
		else
			temp_target = g_current_home_cell;

		auto path_next_status = (int8_t) path_next_best(curr, temp_target, path);
		ROS_INFO("%s %d: Path Find: %d\tNext cell: (%d, %d)\tNow: (%d, %d)", __FUNCTION__, __LINE__, path_next_status, path.cells.front().X, path.cells.front().Y, curr.X, curr.Y);
		if (path_next_status == 1)
		{
//			if (cm_check_loop_back(next))
//				g_switch_home_cell = true;
//			else
//			{
				g_current_home_cell = temp_target;
				return_val = TARGET_FOUND;
				break;
//			}
		}
		else
		{
			Cell_t cell_zero{0, 0};
			if (!g_home_point_old_path.empty() || (going_old_paths && temp_target == cell_zero))
			{
				// If can not reach this point, save this point to new path home point list.
				g_home_point_new_path.push_back(temp_target);
				ROS_WARN("%s %d: Can't reach this home point(%d, %d), push to home point of new path list.", __FUNCTION__, __LINE__, temp_target.X, temp_target.Y);
			}
			g_switch_home_cell = true;
		}
	}

	return return_val;
}

int16_t path_get_home_x()
{
	return g_home_x;
}

int16_t path_get_home_y()
{
	return g_home_y;
}

void wf_path_planning_initialize(Cell_t cell)
{
	int16_t i;


	/* Initialize the default settings. */
//	preset_action_count = 0;

//	weight_enabled = 1;

	/* Save the starting point as home. */
	for ( i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
		g_trapped_cell[i] = cell;
	}
	g_trapped_cell_size = ESCAPE_TRAPPED_REF_CELL_SIZE;


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

	set_explore_new_path_flag(false);
	auto path_next_status = (int8_t) path_next_best(curr, g_continue_cell, path);
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
