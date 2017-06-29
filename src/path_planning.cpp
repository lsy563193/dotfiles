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

#include "core_move.h"
#include "gyro.h"
#include "mathematics.h"
#include "map.h"
#include "path_planning.h"
#include "shortest_path.h"
#include "spot.h"
#include "movement.h"

using namespace std;

typedef struct {
	Cell_t	target;
	list <Cell_t> points;
} PPTargetType;

list <PPTargetType> g_targets;

uint8_t	g_first_start = 0;

uint8_t g_direct_go = 0; /* Enable direct go when there is no obstcal in between current pos. & dest. */

int16_t g_home_x, g_home_y;

Cell_t g_cell_history[5];
const Cell_t& g_curr = g_cell_history[0];

uint16_t g_last_dir;

Cell_t g_trapped_cell[ESCAPE_TRAPPED_REF_CELL_SIZE];

uint8_t g_trapped_cell_size = ESCAPE_TRAPPED_REF_CELL_SIZE;

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

void path_planning_initialize(int32_t *x, int32_t *y)
{
	int16_t i;

	/* Save the starting point as home. */
	g_home_x = count_to_cell(*x);
	g_home_y = count_to_cell(*y);

	for ( i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
		g_trapped_cell[i].X = g_home_x;
		g_trapped_cell[i].Y = g_home_y;
	}
	g_trapped_cell_size = ESCAPE_TRAPPED_REF_CELL_SIZE;

#if (ROBOT_SIZE == 5)

//	g_weight_cnt_threshold = 4;

#else

//	g_weight_cnt_threshold = 3;

#endif

	g_direct_go = 0;

	g_cell_history[0] = {0,0};
	g_last_dir = 0;

	/* Reset the poisition list. */
	for (i = 0; i < 3; i++) {
		g_cell_history[i] =  {int16_t(i+1), int16_t(i+1)};
	}

	/* Initialize the shortest path. */
	path_position_init(g_direct_go);

#ifndef ZONE_WALLFOLLOW

	/* Set the back as blocked, since robot will align the start angle with the wall. */
	Map_SetCell(MAP, cellToCount(-3), cellToCount(0), BLOCKED_BUMPER);

#endif

	map_set_cell(MAP, *x, *y, CLEANED);

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

	g_cell_history[0]= {map_get_x_cell(), map_get_y_cell()};
//	g_cell_history[0].dir = path_get_robot_direction();
}

uint16_t path_get_robot_direction()
{
	return g_last_dir;
}

void path_get_range(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max)
{
	*x_range_min = g_x_min - (abs(g_x_min - g_x_max) <= 3 ? 3 : 1);
	*x_range_max = g_x_max + (abs(g_x_min - g_x_max) <= 3 ? 3 : 1);
	*y_range_min = g_y_min - (abs(g_y_min - g_y_max) <= 3? 3 : 1);
	*y_range_max = g_y_max + (abs(g_y_min - g_y_max) <= 3 ? 3 : 1);

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
	return (map_get_cell(MAP, x, y) == UNCLEAN);
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

bool path_lane_is_cleaned(Cell_t& next)
{
	int16_t i, is_found, min, max, min_stop, max_stop, x_tmp, y_tmp;

	min_stop = max_stop = is_found = 0;
	min = max = SHRT_MAX;
	x_tmp = next.X;
	y_tmp = next.Y;

	for (i = 1; (min_stop == 0 || max_stop == 0); i++)
	{
		/* Find the unclean area in the NEG_X direction of the lane. */
		if (min_stop == 0)
		{
			/* Stop if the cells is blocked, or reach the boundary. */
			if (is_block_blocked(x_tmp - i, y_tmp) == 1 || is_block_boundary(x_tmp - i, y_tmp) == 1)
			{
				min_stop = 1;
			} else
			{
				if (is_brush_block_unclean(x_tmp - i, y_tmp))
				{
					if (is_block_blocked(x_tmp - (i + 1), y_tmp) == 0)
						min = i;
					min_stop = 1;
				}
			}
		}

		/* Find the unclean area in the POS_X direction of the lane. */
		if (max_stop == 0)
		{
			/* Stop if the cells is blocked, or reach the boundary. */
			if (is_block_blocked(x_tmp + i, y_tmp) == 1 || is_block_boundary(x_tmp + i, y_tmp) == 1)
			{
				max_stop = 1;
			} else
			{
				if (is_brush_block_unclean(x_tmp + i, y_tmp))
				{
					if (is_block_blocked(x_tmp + i + 1, y_tmp) == 0)
						max = i;
					max_stop = 1;
				}
			}
		}
	}

	ROS_INFO("%s %d: min: %d\tmax: %d", __FUNCTION__, __LINE__, min, max);
	/* Both ends are not cleaned. */
	if (min != SHRT_MAX && max != SHRT_MAX)
	{
		/*
		 * If the number of cells to clean are the same of both ends, choose either one base the
		 * previous robot g_cell_history. Otherwise, move to the end that have more unclean cells.
		 */
		if (min == max)
		{
			if (g_cell_history[2].Y == g_cell_history[1].Y)
			{
				if (g_cell_history[2].X == g_cell_history[1].X)
				{
					if (g_cell_history[0].X == g_cell_history[1].X)
					{
						next.X += max;
					} else if (g_cell_history[0].X > g_cell_history[1].X)
					{
						next.X -= min;
					} else
					{
						next.X += max;
					}
				} else if (g_cell_history[2].X > g_cell_history[1].X)
				{
					next.X -= min;
				} else
				{
					next.X += max;
				}
			} else if (g_cell_history[0].Y == g_cell_history[1].Y)
			{
				if (g_cell_history[0].X == g_cell_history[1].X)
				{
					next.X += max;
				} else if (g_cell_history[0].X > g_cell_history[1].X)
				{
					next.X += max;
				} else
				{
					next.X -= min;
				}
			} else
			{
				next.X += max;
			}
		} else if (min > max)
		{
			next.X += max;
		} else
		{
			next.X -= min;
		}

		is_found = 2;
	} else if (min != SHRT_MAX)
	{
		/* Only the NEG_X end is not cleaned. */
		next.X -= min;
		is_found = 1;
	} else if (max != SHRT_MAX)
	{
		/* Only the POS_X end is not cleaned. */
		next.X += max;
		is_found = 1;
	}
	if (is_found == 1 && g_cell_history[0] == g_cell_history[1])
		is_found = 0;

	if (is_found == 1)
	{
		uint8_t un_cleaned_cnt = 0;
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
			for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
				if (map_get_cell(MAP, next.X + dx, next.Y + dy) == UNCLEAN)
					un_cleaned_cnt++;

		if (un_cleaned_cnt < 2)
			is_found = 0;
	}

	ROS_WARN("is_found =%d", is_found);

	const Cell_t curr{g_cell_history[0].X, g_cell_history[0].Y};
	if (is_found > 0)
	{
		auto dx1 = (next.X > curr.X) ? 1 : -1;
		auto boundary = (next.X > curr.X) ? g_x_max : g_x_min;
		while(next.X != boundary)
		{
			if (! is_brush_block_unclean(next.X, next.Y))
				break;
			next.X += dx1;
		}
	}
	return is_found>0;
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

	x = map_get_x_cell();
	y = map_get_y_cell();
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
			it->points.push_back(t);

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
			it->points.push_back(t);
		}
	}
}

int16_t path_target(Cell_t& next, Cell_t& target)
{
	bool	within_range;
	int16_t found, final_cost, a, b, c, d, start, end, last_y;
	int16_t x_min, y_min, x_max, y_max, x_min_tmp, y_min_tmp, x_max_tmp, y_max_tmp, stop;

	final_cost = 1000;
	found = 0;
	x_max = y_max = x_max_tmp = y_max_tmp = SHRT_MIN;
	x_min = y_min = x_min_tmp = y_min_tmp = SHRT_MAX;

	for (c = g_x_min - 1; c < g_x_max + 1; ++c) {
		for (d = g_y_min - 1; d < g_y_max + 1; ++d) {
			if (map_get_cell(MAP, c, d) != UNCLEAN) {
				x_min_tmp = x_min_tmp > c ? c : x_min_tmp;
				x_max_tmp = x_max_tmp < c ? c : x_max_tmp;
				y_min_tmp = y_min_tmp > d ? d : y_min_tmp;
				y_max_tmp = y_max_tmp < d ? d : y_max_tmp;
			}
		}
	}

	for (c = x_min_tmp; c <= x_max_tmp; ++c) {
		for (d = y_min_tmp; d <= y_max_tmp; ++d) {
			if (map_get_cell(MAP, c, d) != CLEANED)
				continue;

			if (c > g_x_min - 1 && map_get_cell(MAP, c - 1, d) == UNCLEAN) {
				if (is_block_accessible(c - 1, d) == 1) {
					map_set_cell(MAP, cell_to_count(c - 1), cell_to_count(d), TARGET);
					x_min = x_min > (c - 1) ? (c - 1) : x_min;
					x_max = x_max < (c - 1) ? (c - 1) : x_max;
					y_min = y_min > d ? d : y_min;
					y_max = y_max < d ? d : y_max;
				}
			}

			if (c < g_x_max + 1 && map_get_cell(MAP, c + 1, d) == UNCLEAN) {
				if (is_block_accessible(c + 1, d) == 1) {
					map_set_cell(MAP, cell_to_count(c + 1), cell_to_count(d), TARGET);
					x_min = x_min > (c + 1) ? (c + 1) : x_min;
					x_max = x_max < (c + 1) ? (c + 1) : x_max;
					y_min = y_min > d ? d : y_min;
					y_max = y_max < d ? d : y_max;
				}
			}

			if (d > g_y_min - 1 && map_get_cell(MAP, c, d - 1) == UNCLEAN) {
				if (is_block_accessible(c, d - 1) == 1) {
					map_set_cell(MAP, cell_to_count(c), cell_to_count(d - 1), TARGET);
					x_min = x_min > c ? c : x_min;
					x_max = x_max < c ? c : x_max;
					y_min = y_min > (d - 1) ? (d - 1) : y_min;
					y_max = y_max < (d - 1) ? (d - 1) : y_max;
				}
			}

			if (d < g_y_max + 1 && map_get_cell(MAP, c, d + 1) == UNCLEAN) {
				if (is_block_accessible(c, d + 1) == 1) {
					map_set_cell(MAP, cell_to_count(c), cell_to_count(d + 1), TARGET);
					x_min = x_min > c ? c : x_min;
					x_max = x_max < c ? c : x_max;
					y_min = y_min > (d + 1) ? (d + 1) : y_min;
					y_max = y_max < (d + 1) ? (d + 1) : y_max;
				}
			}
		}
	}

	/* Narrow down the coodinate that robot should go */
	for (d = y_min; d <= y_max; d++) {
		start = end = SHRT_MAX;
		for (c = x_min; c <= map_get_x_cell(); c++) {
			if (map_get_cell(MAP, c, d) == TARGET) {
				if (start == SHRT_MAX)
					start = c;
				if (start != SHRT_MAX)
					end = c;
			}
			if (map_get_cell(MAP, c, d) != TARGET || c == map_get_x_cell()){
				if ( start != SHRT_MAX) {
					for (a = start + 1; a < end; a++) {
						map_set_cell(MAP, cell_to_count(a), cell_to_count(d), UNCLEAN);
					}
				}
				start = end = SHRT_MAX;
			}
		}

		start = end = SHRT_MIN;
		for (c = x_max; c >= map_get_x_cell(); c--) {
			if (map_get_cell(MAP, c, d) == TARGET) {
				if (start == SHRT_MIN)
					start = c;
				if (start != SHRT_MIN)
					end = c;
			}
			if (map_get_cell(MAP, c, d) != TARGET || c == map_get_x_cell()){
				if (end != SHRT_MIN) {
					for (a = start - 1; a > end; a--) {
						map_set_cell(MAP, cell_to_count(a), cell_to_count(d), UNCLEAN);
					}
				}
				start = end = SHRT_MIN;
			}
		}
	}

	x_min_tmp = x_min;
	x_max_tmp = x_max;
	y_min_tmp = y_min;
	y_max_tmp = y_max;

	for (auto& target : g_targets)
		target.points.clear();

	g_targets.clear();
	for (c = x_min_tmp; c <= x_max_tmp; ++c) {
		for (d = y_min_tmp; d <= y_max_tmp; ++d) {
			if (map_get_cell(MAP, c, d) == TARGET) {
				PPTargetType t;
				t.target.X = c;
				t.target.Y = d;
				t.points.clear();
				g_targets.push_back(t);

				x_min = x_min > c ? c : x_min;
				x_max = x_max < c ? c : x_max;
				y_min = y_min > d ? d : y_min;
				y_max = y_max < d ? d : y_max;
			}
		}
	}

	debug_map(MAP, g_home_x, g_home_y);
	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
		map_set_cell(MAP, cell_to_count(it->target.X), cell_to_count(it->target.Y), UNCLEAN);
	}

	path_find_all_targets();

	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end();) {
		if (it->points.empty() == true) {
			it = g_targets.erase(it);
		} else {
			it++;
		}
	}

	/* No more target to clean */
	if (g_targets.empty() == true) {
		if (path_escape_trapped() <= 0) {
			ROS_WARN("%s %d: trapped", __FUNCTION__, __LINE__);
			return -2;
		}
		return 0;
	}

	ROS_INFO("%s %d: targets count: %d", __FUNCTION__, __LINE__, (int)g_targets.size());
	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
		std::string	msg = __FUNCTION__;
		msg += " " + std::to_string(__LINE__) + ": target (" + std::to_string(it->target.X) + ", " + std::to_string(it->target.Y) + ") " + std::to_string(it->points.size()) + ": ";

		for (list<Cell_t>::iterator i = it->points.begin(); i != it->points.end(); ++i) {
			msg += "(" + std::to_string(i->X) + ", " + std::to_string(i->Y) + ")->";
		}
		msg += "\n";
		ROS_INFO(msg.c_str());
	}

#if 1
	stop = 0;
	ROS_INFO("%s %d: case 1, towards Y+ only", __FUNCTION__, __LINE__);
	for (d = y_max; d >= map_get_y_cell(); --d) {
		if (stop == 1 && d <= map_get_y_cell() + 1) {
			break;
		}

		for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
			if (map_get_cell(MAP, it->target.X, it->target.Y - 1) != CLEANED) {
				continue;
			}

			if (it->target.Y == d) {
				if (it->points.size() > final_cost) {
					continue;
				}

				last_y = it->points.front().Y;
				within_range = true;
				for (list<Cell_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
					if (i->Y < map_get_y_cell() || i->Y > d) {
						within_range = false;
					}
					if (i->Y > last_y) {
						within_range = false;
					} else {
						last_y = i->Y;
					}
				}
				if (within_range == true) {
					target.X = it->target.X;
					target.Y = it->target.Y;
					final_cost = it->points.size();
					stop = 1;
				}
			}
		}
	}

	ROS_INFO("%s %d: case 2, towards Y+, allow Y- shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
	if (stop == 0) {
		for (a = map_get_y_cell(); a >= y_min && stop == 0; --a) {
			for (d = a; d <= y_max && stop == 0; ++d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->target.Y == d) {
						if (it->points.size() > final_cost) {
							continue;
						}

						within_range = true;
						last_y = it->points.front().Y;
						bool turn = false;
						for (list<Cell_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
							if (i->Y < a || i->Y > (d > map_get_y_cell() ? d : map_get_y_cell())) {
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
							target.X = it->target.X;
							target.Y = it->target.Y;
							final_cost = it->points.size();
							stop = 1;
						}
					}
				}
			}
		}
	}

	ROS_INFO("%s %d: case 3, towards Y- only, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
	if (stop == 0) {
		for (d = y_min; d >= map_get_y_cell(); ++d) {
			if (stop == 1 && d >= map_get_y_cell() - 1) {
				break;
			}

			for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
				if (map_get_cell(MAP, it->target.X, it->target.Y + 1) != CLEANED) {
					continue;
				}

				if (it->target.Y == d) {
					if (it->points.size() > final_cost) {
						continue;
					}

					last_y = it->points.front().Y;
					within_range = true;
					for (list<Cell_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
						if (i->Y > map_get_y_cell() || i->Y < d) {
							within_range = false;
						}
						if (i->Y < last_y) {
							within_range = false;
						} else {
							last_y = i->Y;
						}
					}
					if (within_range == true) {
						target.X = it->target.X;
						target.Y = it->target.Y;
						final_cost = it->points.size();
						stop = 1;
					}
				}
			}
		}
	}

	ROS_INFO("%s %d: case 4, towards Y-, allow Y+ shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
	if (stop == 0) {
		for (a = map_get_y_cell(); a <= y_max && stop == 0; ++a) {
			for (d = a; d >= y_min && stop == 0; --d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->target.Y == d) {
						if (it->points.size() > final_cost) {
							continue;
						}

						within_range = true;
						last_y = it->points.front().Y;
						bool turn = false;
						for (list<Cell_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
							if (i->Y > a || i->Y < (d > map_get_y_cell() ? map_get_y_cell() : d)) {
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
							target.X = it->target.X;
							target.Y = it->target.Y;
							final_cost = it->points.size();
							stop = 1;
						}
					}
				}
			}
		}
	}
	ROS_INFO("%s %d: case 5: towards Y+, allow Y- shift, allow turns, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
	if (stop == 0) {
		for (a = map_get_y_cell(); a <= y_max  && stop == 0; ++a) {
	            for (d = map_get_y_cell(); d <= a && stop == 0; ++d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->target.Y == d) {
						within_range = true;
						for (list<Cell_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
							if (i->Y < map_get_y_cell() || i->Y > a) {
								within_range = false;
							}
						}
						if (within_range == true && it->points.size() < final_cost) {
							target.X = it->target.X;
							target.Y = it->target.Y;
							final_cost = it->points.size();
							stop = 1;
						}
					}
				}
			}
		}
	}
#endif
	ROS_INFO("%s %d: case 6, fallback to A-start the nearest target, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
	/* fallback to find unclean area */
	if (stop == 0) {
		for (c = x_min; c <= x_max; ++c) {
			for (d = y_min; d <= y_max; ++d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->points.size() < final_cost) {
						target.X = it->target.X;
						target.Y = it->target.Y;
						final_cost = it->points.size();
					}
				}
			}
		}
	}

	found = (final_cost != 1000) ? final_cost : 0 ;
	ROS_INFO("%s %d: found: %d (%d, %d)\n", __FUNCTION__, __LINE__, found, target.X, target.Y);

	if(found == 0)
	return 0;

	return path_next_best(target, g_curr.X, g_curr.Y, next.X, next.Y);
}

void path_update_cells()
{
	if(get_clean_mode() != Clean_Mode_Navigation)
		return;
	/* Skip, if robot is not moving towards POS_X. */
	if ((g_last_dir % 1800) != 0)
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

	if ( g_trapped_cell[0].X != g_home_x || g_trapped_cell[0].Y != g_home_y ){
		for ( i = 0; i < g_trapped_cell_size; ++i ) {
			ROS_WARN("%s %d Check %d trapped reference cell: x: %d, y: %d", __FUNCTION__, __LINE__,
			         i, g_trapped_cell[i].X, g_trapped_cell[i].Y);
			if (is_block_accessible(g_trapped_cell[i].X, g_trapped_cell[i].Y) == 0) {
				map_set_cells(ROBOT_SIZE, g_trapped_cell[i].X, g_trapped_cell[i].Y, CLEANED);
			}

			val = path_find_shortest_path( g_cell_history[0].X, g_cell_history[0].Y, g_trapped_cell[i].X, g_trapped_cell[i].Y, 0);
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
			val = path_find_shortest_path(g_cell_history[0].X, g_cell_history[0].Y, 0, 0, 0);
#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
//			ROS_WARN("%s %d: pos (%d, %d)\tval: %d", __FUNCTION__, __LINE__, g_cell_history[0].x, g_cell_history[0].y, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = path_find_shortest_path(g_cell_history[0].X, g_cell_history[0].Y, g_home_x, g_home_y, 0);
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
			val = path_find_shortest_path(g_cell_history[0].X, g_cell_history[0].Y, g_home_x, g_home_y, 0);
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

int8_t path_next(Point32_t *next_point, Point32_t *target_point)
{
	Cell_t next = g_curr;
	Cell_t target = next;
	extern CMMoveType g_cm_move_type;
	if(get_clean_mode() == Clean_Mode_WallFollow){
		ROS_ERROR("path_next Clean_Mode:(%d)", get_clean_mode());
		if(g_cm_move_type == CM_LINEARMOVE){
			if(g_curr != map_point_to_cell(*next_point)){
				ROS_INFO("start follow wall");
				g_cm_move_type = CM_FOLLOW_LEFT_WALL;
				next = map_point_to_cell(*next_point);
			}else{
				ROS_INFO("reach 8m, go_home.");
				wf_clear();
				return 0;
			}
		} else {
			if(wf_is_go_home()) {
				ROS_INFO("follow wall finish");
				wf_clear();
				return 0;

			} else {
				ROS_INFO("CM_LINEARMOVE");
				g_cm_move_type = CM_LINEARMOVE;
				wf_break_wall_follow();
				auto angle = wf_is_first() ? 0 : 2700;
				int32_t x_point,y_point;
				const float	FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
				cm_world_to_point(Gyro_GetAngle() + angle, 0, FIND_WALL_DISTANCE * 1000, &x_point, &y_point);
				next = {count_to_cell(x_point),count_to_cell(y_point)};
				ROS_WARN("next.X = %d next.Y = %d", next.X, next.Y);
			}

		}
	}
	//if(get_clean_mode() == Clean_Mode_Spot || SpotMovement::instance()->getSpotType() == NORMAL_SPOT){	
	else if( SpotMovement::instance()->getSpotType() == CLEAN_SPOT || SpotMovement::instance()->getSpotType() == NORMAL_SPOT){
        int8_t ret = SpotMovement::instance()->getNextTarget(*next_point);
		target_point = next_point;
        //ROS_WARN("%s,%d target_point (%d,%d),next_point (%d,%d) return %d",__FUNCTION__,__LINE__,target_point->X,target_point->Y,next_point->X,next_point->Y,ret);
        return ret;
	}
	else if(get_clean_mode() == Clean_Mode_Navigation) {
		auto is_found = path_lane_is_cleaned(next);
		target = next;
		if (!is_found)
		{
			auto ret = path_target(next, target);//0 not target, 1,found, -2 trap
			if (ret == 0)
				return 0;

			if (ret == -2)
			{
				/* Robot is trapped and no path to starting point or home. */
				if (path_escape_trapped() == 0)
					return 2;
				else
					return -1;
			}
		}
	}
	//found ==1
	if (g_curr.X == next.X)
		g_last_dir = g_curr.Y > next.Y ? NEG_Y : POS_Y;
	else
		g_last_dir = g_curr.X > next.X ? NEG_X : POS_X;

	*next_point = map_cell_to_point(next);
	*target_point = map_cell_to_point(target);

	return 1;
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

int16_t path_get_home_x()
{
	return g_home_x;
}

int16_t path_get_home_y()
{
	return g_home_y;
}

void WF_PathPlanning_Initialize(int32_t *x, int32_t *y)
{
	int16_t i;

	/* Save the starting point as home. */
	g_home_x = count_to_cell(*x);
	g_home_y = count_to_cell(*y);

	/* Initialize the default settings. */
//	preset_action_count = 0;

//	weight_enabled = 1;

	for ( i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
		g_trapped_cell[i].X = g_home_x;
		g_trapped_cell[i].Y = g_home_y;
	}
	g_trapped_cell_size = ESCAPE_TRAPPED_REF_CELL_SIZE;


#if (ROBOT_SIZE == 5)

//	g_weight_cnt_threshold = 4;

#else

//	g_weight_cnt_threshold = 3;

#endif

	g_direct_go = 0;

	g_cell_history[0] = {0,0};
	g_last_dir = 0;

	/* Reset the poisition list. */
	for (i = 0; i < 3; i++) {
		g_cell_history[i]= {int16_t(i + 1), int16_t(i + 1)};
	}

	/* Initialize the shortest path. */
	path_position_init(g_direct_go);
}

