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

#include "core_move.h"
#include "gyro.h"
#include "mathematics.h"
#include "map.h"
#include "path_planning.h"
#include "shortest_path.h"

using namespace std;

typedef struct {
	Cell_t	target;
	list <Cell_t> points;
} PPTargetType;

list <PPTargetType> g_targets;

uint8_t	g_first_start = 0;

uint8_t g_direct_go = 0; /* Enable direct go when there is no obstcal in between current pos. & dest. */

int16_t g_home_x, g_home_y;

uint8_t	g_clear_block = 0;

PositionType g_pos_history[5];

uint16_t g_last_dir;

Cell_t g_trappedCell[ESCAPE_TRAPPED_REF_CELL_SIZE];

uint8_t g_trappedCellSize = ESCAPE_TRAPPED_REF_CELL_SIZE;

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

void path_planning_initialize(int32_t *x, int32_t *y)
{
	int16_t i;

	/* Save the starting point as home. */
	g_home_x = countToCell(*x);
	g_home_y = countToCell(*y);

	for ( i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
		g_trappedCell[i].X = g_home_x;
		g_trappedCell[i].Y = g_home_y;
	}
	g_trappedCellSize = ESCAPE_TRAPPED_REF_CELL_SIZE;

#if (ROBOT_SIZE == 5)

//	g_weight_cnt_threshold = 4;

#else

//	g_weight_cnt_threshold = 3;

#endif

	g_direct_go = 0;

	g_pos_history[0].x = g_pos_history[0].y = 0;
	g_last_dir = 0;

//	try_entrance = 0;
	g_first_start = 0;
//	g_last_x_pos = g_last_y_pos = 0;

	/* Reset the poisition list. */
	for (i = 0; i < 3; i++) {
		g_pos_history[i].x = g_pos_history[i].y = i + 1;
	}

	/* Initialize the shortest path. */
	path_position_init(g_direct_go);

#ifndef ZONE_WALLFOLLOW

	/* Set the back as blocked, since robot will align the start angle with the wall. */
	Map_SetCell(MAP, cellToCount(-3), cellToCount(0), BLOCKED_BUMPER);

#endif

	Map_SetCell(MAP, *x, *y, CLEANED);

	/* Set the starting point as cleaned. */
	Map_SetCell(MAP, cellToCount(-1), cellToCount(-1), CLEANED);
	Map_SetCell(MAP, cellToCount(-1), cellToCount(0), CLEANED);
	Map_SetCell(MAP, cellToCount(-1), cellToCount(1), CLEANED);
	Map_SetCell(MAP, cellToCount(0), cellToCount(-1), CLEANED);
	Map_SetCell(MAP, cellToCount(0), cellToCount(0), CLEANED);
	Map_SetCell(MAP, cellToCount(0), cellToCount(1), CLEANED);
	Map_SetCell(MAP, cellToCount(1), cellToCount(-1), CLEANED);
	Map_SetCell(MAP, cellToCount(1), cellToCount(0), CLEANED);
	Map_SetCell(MAP, cellToCount(1), cellToCount(1), CLEANED);
}

void path_set_current_pos()
{
	g_pos_history[4] = g_pos_history[3];
	g_pos_history[3] = g_pos_history[2];
	g_pos_history[2] = g_pos_history[1];
	g_pos_history[1] = g_pos_history[0];

	g_pos_history[0].x = Map_GetXCell();
	g_pos_history[0].y = Map_GetYCell();
	g_pos_history[0].dir = path_get_robot_direction();
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

	ROS_INFO("Get Range:\tx: %d - %d\ty: %d - %d\tx range: %d - %d\ty range: %d - %d",
		g_x_min, g_x_max, g_y_min, g_y_max, *x_range_min, *x_range_max, *y_range_min, *y_range_max);
}

uint8_t is_a_block(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	CellState cs;

	cs = Map_GetCell(MAP, x, y);
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
			cs = Map_GetCell(MAP, x + i, y + j);
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
	int16_t	i, j;

	for (i = ROBOT_BRUSH_LEFT_OFFSET; retval == 1 && i <= ROBOT_BRUSH_RIGHT_OFFSET; i++) {
		for (j = ROBOT_BRUSH_LEFT_OFFSET; retval == 1 && j <= ROBOT_BRUSH_RIGHT_OFFSET; j++) {
			if (Map_GetCell(MAP, x + i, y + j) != CLEANED) {
				retval = 0;
			}
		}
	}
	return retval;
}

uint8_t is_brush_block_unclean(int16_t x, int16_t y)
{
	uint8_t retval = 0, count = 0;

#if (ROBOT_SIZE == 5)

	int16_t i;

	//for (i = ROBOT_BRUSH_LEFT_OFFSET; count < 1 && i <= ROBOT_BRUSH_RIGHT_OFFSET; i++) {
	for (i = ROBOT_BRUSH_LEFT_OFFSET + 1; count < 1 && i <= ROBOT_BRUSH_RIGHT_OFFSET - 1; i++) {
		if (Map_GetCell(MAP, x, y + i) == UNCLEAN) {
			//retval = 1;
			count++;
		}
	}

#else

	if (Map_GetCell(MAP, x, y) == UNCLEAN)
		count++;

#endif

	if (count == 1)
		retval = 1;

	return retval;
}

uint8_t is_block_boundary(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (Map_GetCell(MAP, x, i) == BLOCKED_BOUNDARY) {
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

uint8_t path_lane_is_cleaned(int16_t *x, int16_t *y)
{
	int16_t	i, found, min, max, min_stop, max_stop, x_tmp, y_tmp;

	min_stop = max_stop = found = 0;
	min = max = SHRT_MAX;
	x_tmp = *x;
	y_tmp = *y;

	for (i = 1; (min_stop == 0 || max_stop == 0); i++) {
		/* Find the unclean area in the SOUTH direction of the lane. */
		if (min_stop == 0) {
			/* Stop if the cells is blocked, or reach the boundary. */
			if (is_block_blocked(x_tmp - i, y_tmp) == 1 || is_block_boundary(x_tmp - i, y_tmp) == 1) {
				min_stop = 1;
			} else {
				if (is_brush_block_unclean(x_tmp - i, y_tmp) == 1) {
					if (is_block_blocked(x_tmp - (i + 1), y_tmp) == 0)
						min = i;
					min_stop = 1;
				}
			}
		}

		/* Find the unclean area in the NORTH direction of the lane. */
		if (max_stop == 0) {
			/* Stop if the cells is blocked, or reach the boundary. */
			if (is_block_blocked(x_tmp + i, y_tmp) == 1 || is_block_boundary(x_tmp + i, y_tmp) == 1) {
				max_stop = 1;
			} else {
				if (is_brush_block_unclean(x_tmp + i, y_tmp) == 1) {
					if (is_block_blocked(x_tmp + i + 1, y_tmp) == 0)
						max = i;
					max_stop = 1;
				}
			}
		}
	}

	ROS_INFO("%s %d: min: %d\tmax: %d", __FUNCTION__, __LINE__, min, max);
	/* Both ends are not cleaned. */
	if (min != SHRT_MAX && max != SHRT_MAX) {
		/*
		 * If the number of cells to clean are the same of both ends, choose either one base the
		 * previous robot g_pos_history. Otherwise, move to the end that have more unclean cells.
		 */
		if (min == max) {
			if (g_pos_history[2].y == g_pos_history[1].y) {
				if (g_pos_history[2].x == g_pos_history[1].x) {
					if (g_pos_history[0].x == g_pos_history[1].x) {
						*x += max;
					} else if (g_pos_history[0].x > g_pos_history[1].x) {
						*x -= min;
					} else {
						*x += max;
					}
				} else if ( g_pos_history[2].x > g_pos_history[1].x) {
					*x -= min;
				} else {
					*x += max;
				}
			} else if (g_pos_history[0].y == g_pos_history[1].y) {
				if (g_pos_history[0].x == g_pos_history[1].x) {
					*x += max;
				} else if (g_pos_history[0].x > g_pos_history[1].x) {
					*x += max;
				} else {
					*x -= min;
				}
			} else {
				*x += max;
			}
		} else if (min > max) {
			*x += max;
		} else {
			*x -= min;
		}

		found = 2;
	} else if (min != SHRT_MAX) {
		/* Only the SOUTH end is not cleaned. */
		*x -= min;
		found = 1;
	} else if (max != SHRT_MAX ) {
		/* Only the NORTH end is not cleaned. */
		*x += max;
		found = 1;
	}

	return found;
}

void path_find_all_targets()
{
	bool		all_set;
	int16_t		i, j, x, y, offset, passValue, nextPassValue, passSet, tracex, tracey, targetCost;
	CellState	cs;

	Map_Reset(SPMAP);

	for (i = g_x_min; i <= g_x_max; ++i) {
		for (j = g_y_min; j <= g_y_max; ++j) {
			cs = Map_GetCell(MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				for (x = ROBOT_RIGHT_OFFSET; x <= ROBOT_LEFT_OFFSET; x++) {
					for (y = ROBOT_RIGHT_OFFSET; y <= ROBOT_LEFT_OFFSET; y++) {
						Map_SetCell(SPMAP, i + x, j + y, COST_HIGH);
					}
				}
			}
		}
	}

	x = Map_GetXCell();
	y = Map_GetYCell();
	/* Set the current robot position has the cost value of 1. */
	Map_SetCell(SPMAP, (int32_t)x, (int32_t)y, COST_1);

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

				if(Map_GetCell(SPMAP, i, j) == passValue) {
					if (i - 1 >= g_x_min && Map_GetCell(SPMAP, i - 1, j) == COST_NO) {
						Map_SetCell(SPMAP, (i - 1), (j), (CellState)nextPassValue);
						passSet = 1;
					}

					if ((i + 1) <= g_x_max && Map_GetCell(SPMAP, i + 1, j) == COST_NO) {
						Map_SetCell(SPMAP, (i + 1), (j), (CellState)nextPassValue);
						passSet = 1;
					}

					if (j - 1  >= g_y_min && Map_GetCell(SPMAP, i, j - 1) == COST_NO) {
						Map_SetCell(SPMAP, (i), (j - 1), (CellState)nextPassValue);
						passSet = 1;
					}

					if ((j + 1) <= g_y_max && Map_GetCell(SPMAP, i, j + 1) == COST_NO) {
						Map_SetCell(SPMAP, (i), (j + 1), (CellState)nextPassValue);
						passSet = 1;
					}
				}
			}
		}

		all_set = true;
		for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
			if (Map_GetCell(SPMAP, it->target.X, it->target.Y) == COST_NO) {
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
		if (Map_GetCell(SPMAP, it->target.X, it->target.Y) == COST_NO || Map_GetCell(SPMAP, it->target.X, it->target.Y) == COST_HIGH) {
			continue;
		}

		Cell_t	t;
		tracex = it->target.X;
		tracey = it->target.Y;
		while (tracex != x || tracey != y) {
			targetCost = Map_GetCell(SPMAP, tracex, tracey) - 1;

			if (targetCost == 0) {
				targetCost = COST_5;
			}

			t.X = tracex;
			t.Y = tracey;
			it->points.push_back(t);

			if ((tracex - 1 >= g_x_min) && (Map_GetCell(SPMAP, tracex - 1, tracey) == targetCost)) {
				tracex--;
				continue;
			}

			if ((tracex + 1 <= g_x_max) && (Map_GetCell(SPMAP, tracex + 1, tracey) == targetCost)) {
				tracex++;
				continue;
			}

			if ((tracey - 1 >= g_y_min) && (Map_GetCell(SPMAP, tracex, tracey - 1) == targetCost)) {
				tracey--;
				continue;
			}

			if ((tracey + 1 <= g_y_max) && (Map_GetCell(SPMAP, tracex, tracey + 1) == targetCost)) {
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

int16_t find_next_unclean_with_approaching(int16_t *x, int16_t *y)
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
			if (Map_GetCell(MAP, c, d) != UNCLEAN) {
				x_min_tmp = x_min_tmp > c ? c : x_min_tmp;
				x_max_tmp = x_max_tmp < c ? c : x_max_tmp;
				y_min_tmp = y_min_tmp > d ? d : y_min_tmp;
				y_max_tmp = y_max_tmp < d ? d : y_max_tmp;
			}
		}
	}

	for (c = x_min_tmp; c <= x_max_tmp; ++c) {
		for (d = y_min_tmp; d <= y_max_tmp; ++d) {
			if (Map_GetCell(MAP, c, d) != CLEANED)
				continue;

			if (c > g_x_min - 1 && Map_GetCell(MAP, c - 1, d) == UNCLEAN) {
				if (is_block_accessible(c - 1, d) == 1) {
					Map_SetCell(MAP, cellToCount(c - 1), cellToCount(d), TARGET);
					x_min = x_min > (c - 1) ? (c - 1) : x_min;
					x_max = x_max < (c - 1) ? (c - 1) : x_max;
					y_min = y_min > d ? d : y_min;
					y_max = y_max < d ? d : y_max;
				}
			}

			if (c < g_x_max + 1 && Map_GetCell(MAP, c + 1, d) == UNCLEAN) {
				if (is_block_accessible(c + 1, d) == 1) {
					Map_SetCell(MAP, cellToCount(c + 1), cellToCount(d), TARGET);
					x_min = x_min > (c + 1) ? (c + 1) : x_min;
					x_max = x_max < (c + 1) ? (c + 1) : x_max;
					y_min = y_min > d ? d : y_min;
					y_max = y_max < d ? d : y_max;
				}
			}

			if (d > g_y_min - 1 && Map_GetCell(MAP, c, d - 1) == UNCLEAN) {
				if (is_block_accessible(c, d - 1) == 1) {
					Map_SetCell(MAP, cellToCount(c), cellToCount(d - 1), TARGET);
					x_min = x_min > c ? c : x_min;
					x_max = x_max < c ? c : x_max;
					y_min = y_min > (d - 1) ? (d - 1) : y_min;
					y_max = y_max < (d - 1) ? (d - 1) : y_max;
				}
			}

			if (d < g_y_max + 1 && Map_GetCell(MAP, c, d + 1) == UNCLEAN) {
				if (is_block_accessible(c, d + 1) == 1) {
					Map_SetCell(MAP, cellToCount(c), cellToCount(d + 1), TARGET);
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
		for (c = x_min; c <= Map_GetXCell(); c++) {
			if (Map_GetCell(MAP, c, d) == TARGET) {
				if (start == SHRT_MAX)
					start = c;
				if (start != SHRT_MAX)
					end = c;
			}
			if (Map_GetCell(MAP, c, d) != TARGET || c == Map_GetXCell()){
				if ( start != SHRT_MAX) {
					for (a = start + 1; a < end; a++) {
						Map_SetCell(MAP, cellToCount(a), cellToCount(d), UNCLEAN);
					}
				}
				start = end = SHRT_MAX;
			}
		}

		start = end = SHRT_MIN;
		for (c = x_max; c >= Map_GetXCell(); c--) {
			if (Map_GetCell(MAP, c, d) == TARGET) {
				if (start == SHRT_MIN)
					start = c;
				if (start != SHRT_MIN)
					end = c;
			}
			if (Map_GetCell(MAP, c, d) != TARGET || c == Map_GetXCell()){
				if (end != SHRT_MIN) {
					for (a = start - 1; a > end; a--) {
						Map_SetCell(MAP, cellToCount(a), cellToCount(d), UNCLEAN);
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

	for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
		it->points.clear();
	}

	g_targets.clear();
	for (c = x_min_tmp; c <= x_max_tmp; ++c) {
		for (d = y_min_tmp; d <= y_max_tmp; ++d) {
			if (Map_GetCell(MAP, c, d) == TARGET) {
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
		Map_SetCell(MAP, cellToCount(it->target.X), cellToCount(it->target.Y), UNCLEAN);
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
	for (d = y_max; d >= Map_GetYCell(); --d) {
		if (stop == 1 && d <= Map_GetYCell() + 1) {
			break;
		}

		for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
			if (Map_GetCell(MAP, it->target.X, it->target.Y - 1) != CLEANED) {
				continue;
			}

			if (it->target.Y == d) {
				if (it->points.size() > final_cost) {
					continue;
				}

				last_y = it->points.front().Y;
				within_range = true;
				for (list<Cell_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
					if (i->Y < Map_GetYCell() || i->Y > d) {
						within_range = false;
					}
					if (i->Y > last_y) {
						within_range = false;
					} else {
						last_y = i->Y;
					}
				}
				if (within_range == true) {
					*x = it->target.X;
					*y = it->target.Y;
					final_cost = it->points.size();
					stop = 1;
				}
			}
		}
	}

	ROS_INFO("%s %d: case 2, towards Y+, allow Y- shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
	if (stop == 0) {
		for (a = Map_GetYCell(); a >= y_min && stop == 0; --a) {
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
							if (i->Y < a || i->Y > (d > Map_GetYCell() ? d : Map_GetYCell())) {
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
							*x = it->target.X;
							*y = it->target.Y;
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
		for (d = y_min; d >= Map_GetYCell(); ++d) {
			if (stop == 1 && d >= Map_GetYCell() - 1) {
				break;
			}

			for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
				if (Map_GetCell(MAP, it->target.X, it->target.Y + 1) != CLEANED) {
					continue;
				}

				if (it->target.Y == d) {
					if (it->points.size() > final_cost) {
						continue;
					}

					last_y = it->points.front().Y;
					within_range = true;
					for (list<Cell_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
						if (i->Y > Map_GetYCell() || i->Y < d) {
							within_range = false;
						}
						if (i->Y < last_y) {
							within_range = false;
						} else {
							last_y = i->Y;
						}
					}
					if (within_range == true) {
						*x = it->target.X;
						*y = it->target.Y;
						final_cost = it->points.size();
						stop = 1;
					}
				}
			}
		}
	}

	ROS_INFO("%s %d: case 4, towards Y-, allow Y+ shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, stop);
	if (stop == 0) {
		for (a = Map_GetYCell(); a <= y_max && stop == 0; ++a) {
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
							if (i->Y > a || i->Y < (d > Map_GetYCell() ? Map_GetYCell() : d)) {
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
							*x = it->target.X;
							*y = it->target.Y;
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
		for (a = Map_GetYCell(); a <= y_max  && stop == 0; ++a) {
	            for (d = Map_GetYCell(); d <= a && stop == 0; ++d) {
				for (list<PPTargetType>::iterator it = g_targets.begin(); it != g_targets.end(); ++it) {
					if (it->target.Y == d) {
						within_range = true;
						for (list<Cell_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
							if (i->Y < Map_GetYCell() || i->Y > a) {
								within_range = false;
							}
						}
						if (within_range == true && it->points.size() < final_cost) {
							*x = it->target.X;
							*y = it->target.Y;
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
						*x = it->target.X;
						*y = it->target.Y;
						final_cost = it->points.size();
					}
				}
			}
		}
	}

	found = (final_cost != 1000) ? final_cost : 0 ;
	ROS_INFO("%s %d: found: %d (%d, %d)\n", __FUNCTION__, __LINE__, found, *x, *y);

	return found;
}

int16_t path_ahead_to_clean(int16_t x, int16_t y, int16_t x_next)
{
	int16_t offset;

	/*
	 * Stop when the target's X coordinate is not to the boundary of the map.
	 * Since this kind of target is must be the target point for the cleaning
	 * destination.
	 */
	if (!(x == SHRT_MIN || x == SHRT_MAX))
		return 0;

	/*
	 * Find the offset about how many cells to clean ahead with
	 * reference to the given point x_next.
	 */
	offset = (x == SHRT_MIN ? -2 : 2);
	while ((x == SHRT_MIN && offset > x) || (x == SHRT_MAX && offset < x)) {
		/* Reach the boundary, stop. */
		if ((x == SHRT_MIN && x_next + offset <= g_x_min) || (x == SHRT_MAX && x_next + offset >= g_x_max) ) {
			offset = 0;
			break;
		} else if (is_brush_block_unclean(x_next + offset, y) == 1) {
			offset += (x == SHRT_MIN ? -1 : 1);
		} else {
			break;
		}
	}

	//ROS_INFO("%s %d: x: %d\ty: %d\toffset: %d", __FUNCTION__, __LINE__, x, y, offset);
	return offset;
}

void path_update_cells()
{
	int16_t 	i, start, end;
	int32_t		y;
	CellState	cs;

	/* Skip, if robot is not moving towards NORTH or SOUTH. */
	if ((g_last_dir % 1800) != 0)
		return;

	start = g_pos_history[1].x > g_pos_history[0].x ? g_pos_history[0].x : g_pos_history[1].x;
	end = g_pos_history[1].x > g_pos_history[0].x ? g_pos_history[1].x : g_pos_history[0].x;
	ROS_INFO("%s %d: start: %d\tend: %d", __FUNCTION__, __LINE__, start, end);
	for (i = start; i <= end; i++) {
		/* Check the Map cells which Y coordinate equal (y - 2). */

#if (ROBOT_SIZE == 5)

		y = g_pos_history[0].y - 2;

#else

		y = g_pos_history[0].y - 1;

#endif

		if (Map_GetCell(MAP, i, y) == BLOCKED_OBS || Map_GetCell(MAP, i, g_pos_history[0].y - 2) == BLOCKED_BUMPER) {
			if ( i == start) {
				if (Map_GetCell(MAP, i + 1, y) == CLEANED) {
					ROS_WARN("%s %d: reset (%d, %d) to cleaned.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			} else if (i == end) {
				if (Map_GetCell(MAP, i - 1, y) == CLEANED) {
					ROS_WARN("%s %d: reset (%d, %d) to cleaned.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			} else {
				if (Map_GetCell(MAP, i - 1, y) == CLEANED || Map_GetCell(MAP, i + 1, y) == CLEANED) {
					ROS_WARN("%s %d: reset (%d, %d) to cleaned.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			}
		}

		/* Check the Map cells which Y coordinate equal (y + 2). */
#if (ROBOT_SIZE == 5)

		y = g_pos_history[0].y + 2;

#else

		y = g_pos_history[0].y + 1;

#endif
		if (Map_GetCell(MAP, i, y) == BLOCKED_OBS || Map_GetCell(MAP, i, y) == BLOCKED_BUMPER) {
			if ( i == start) {
				if (Map_GetCell(MAP, i + 1, y) == CLEANED) {
					ROS_WARN("%s %d: reset (%d, %d) to cleaned.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			} else if (i == end) {
				if (Map_GetCell(MAP, i - 1, y) == CLEANED) {
					ROS_WARN("%s %d: reset (%d, %d) to cleaned.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			} else {
				if (Map_GetCell(MAP, i - 1, y) == CLEANED || Map_GetCell(MAP, i + 1, y) == CLEANED) {
					ROS_WARN("%s %d: reset (%d, %d) to cleaned.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), CLEANED);
				} else {
					ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, i, y);
					Map_SetCell(MAP, cellToCount(i), cellToCount(y), UNCLEAN);
				}
			}
		}
	}

	/*
	 * The following is try to handle the cases:
	 *
	 * 1. Robot moves towards NORTH
	 *
	 * 		1110	0111
	 * 		1x1e	e1x1
	 *		1110	0111
	 *		0uuu	uuu0
	 *
	 * 2. Robot moves towards SOUTH
	 *
	 *		0uuu	uuu0
	 * 		1110	0111
	 * 		1x1e	e1x1
	 *		1110	0111
	 *
	 * Where u is a value either 2 or 3, x is the current robot position,
	 * e is the target position. With the above changes, the movement of the
	 * robot will be looks nicer.
	 */
	if (g_last_dir == NORTH || g_last_dir == SOUTH) {
		if (g_last_dir == NORTH && g_pos_history[0].x > g_pos_history[1].x) {
			if (g_pos_history[0].y >= 0 && Map_GetCell(MAP, g_pos_history[0].x, g_pos_history[0].y + 2) == UNCLEAN) {
				for (i = 0; i < 3; i++) {
					cs = Map_GetCell(MAP, g_pos_history[0].x + 2, g_pos_history[0].y + i);
					if (cs != CLEANED && cs != UNCLEAN) {
						ROS_WARN("%s %d: reset (%d, %d) to %d.", __FUNCTION__, __LINE__, g_pos_history[0].x + 3, g_pos_history[0].y + i, cs);
						Map_SetCell(MAP, cellToCount(g_pos_history[0].x + 3), cellToCount(g_pos_history[0].y + i), cs);

						ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, g_pos_history[0].x + 2, g_pos_history[0].y + i);
						Map_SetCell(MAP, cellToCount(g_pos_history[0].x + 2), cellToCount(g_pos_history[0].y + i), UNCLEAN);
					}
				}
			} else if (g_pos_history[0].y < 0 && Map_GetCell(MAP, g_pos_history[0].x, g_pos_history[0].y - 2) == UNCLEAN) {
				for (i = 0; i < 3; i++) {
					cs = Map_GetCell(MAP, g_pos_history[0].x + 2, g_pos_history[0].y - i);
					if (cs != CLEANED && cs!= UNCLEAN) {
						ROS_WARN("%s %d: reset (%d, %d) to %d.", __FUNCTION__, __LINE__, g_pos_history[0].x + 3, g_pos_history[0].y - i, cs);
						Map_SetCell(MAP, cellToCount(g_pos_history[0].x + 3), cellToCount(g_pos_history[0].y - i), cs);

						ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, g_pos_history[0].x + 2, g_pos_history[0].y - i);
						Map_SetCell(MAP, cellToCount(g_pos_history[0].x + 2), cellToCount(g_pos_history[0].y - i), UNCLEAN);
					}
				}
			}
		} else if (g_last_dir == SOUTH && g_pos_history[0].x < g_pos_history[1].x) {
			if (g_pos_history[0].y >= 0 && Map_GetCell(MAP, g_pos_history[0].x, g_pos_history[0].y + 2) == UNCLEAN) {
				for (i = 0; i < 3; i++) {
					cs = Map_GetCell(MAP, g_pos_history[0].x - 2, g_pos_history[0].y + i);
					if (cs != CLEANED && cs!= UNCLEAN) {
						ROS_WARN("%s %d: reset (%d, %d) to %d.", __FUNCTION__, __LINE__, g_pos_history[0].x - 3, g_pos_history[0].y + i, cs);
						Map_SetCell(MAP, cellToCount(g_pos_history[0].x - 3), cellToCount(g_pos_history[0].y + i), cs);

						ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, g_pos_history[0].x - 2, g_pos_history[0].y + i);
						Map_SetCell(MAP, cellToCount(g_pos_history[0].x - 2), cellToCount(g_pos_history[0].y + i), UNCLEAN);
					}
				}
			} else if (g_pos_history[0].y < 0 && Map_GetCell(MAP, g_pos_history[0].x, g_pos_history[0].y - 2) == UNCLEAN) {
				for (i = 0; i < 3; i++) {
					cs = Map_GetCell(MAP, g_pos_history[0].x - 2, g_pos_history[0].y - i);
					if (cs != CLEANED && cs!= UNCLEAN) {
						ROS_WARN("%s %d: reset (%d, %d) to %d.", __FUNCTION__, __LINE__, g_pos_history[0].x - 3, g_pos_history[0].y - i, cs);
						Map_SetCell(MAP, cellToCount(g_pos_history[0].x - 3), cellToCount(g_pos_history[0].y - i), cs);

						ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, g_pos_history[0].x - 2, g_pos_history[0].y - i);
						Map_SetCell(MAP, cellToCount(g_pos_history[0].x - 2), cellToCount(g_pos_history[0].y - i), UNCLEAN);
					}
				}
			}
		}
	}
}

int16_t WF_path_escape_trapped()
{

	int16_t	val = 0;
	uint16_t i = 0;
	int16_t remote_x = 0, remote_y = 0;
	int16_t x_min, x_max, y_min, y_max;
	Point32_t	Remote_Point;
	Cell_t   tmpPnt, pnt16ArTmp[3];
	Remote_Point.X = 0;
	Remote_Point.Y = 0;
	//tmpPnt.X = countToCell(Remote_Point.X);
	//tmpPnt.Y = countToCell(Remote_Point.Y);
	path_get_range(&x_min, &x_max, &y_min, &y_max);
	tmpPnt.X = x_max + 1;
	tmpPnt.Y = y_max + 1;
	ROS_INFO("tmpPnt.X = %d, tmpPnt.Y = %d\n",tmpPnt.X, tmpPnt.Y);
	pnt16ArTmp[0] = tmpPnt;

	path_escape_set_trapped_cell(pnt16ArTmp, 1);

	if ( g_trappedCell[0].X != remote_x || g_trappedCell[0].Y != remote_y ){
		for ( i = 0; i < g_trappedCellSize; ++i ) {
			ROS_INFO("%s %d Check %d trapped reference cell: x: %d, y: %d\n", __FUNCTION__, __LINE__,
			         i, g_trappedCell[i].X, g_trappedCell[i].Y);
			/*if (is_block_accessible(g_trappedCell[i].X, g_trappedCell[i].Y) == 0) {
				Map_Set_Cells(ROBOT_SIZE, g_trappedCell[i].X, g_trappedCell[i].Y, CLEANED);
			}*/

			//val = WF_path_find_shortest_path( g_pos_history[0].x, g_pos_history[0].y, g_trappedCell[i].X, g_trappedCell[i].Y, 0);
			val = WF_path_find_shortest_path( g_pos_history[0].x, g_pos_history[0].y, g_trappedCell[i].X, g_trappedCell[i].Y, 0);
			ROS_INFO("%s %d: val %d\n", __FUNCTION__, __LINE__, val);
			ROS_INFO("SCHAR_MAX = %d\n", SCHAR_MAX);
			if (val < 0 || val == SCHAR_MAX) {
				/* No path to home, which is set when path planning is initialized. */
				val = 0;//not isolated
			} else {
				val = 1;//isolated
				break;
			}
		}
	} else {
		if (is_block_accessible(0, 0) == 1) {
			val = WF_path_find_shortest_path(g_pos_history[0].x, g_pos_history[0].y, 0, 0, 0);
#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
			ROS_INFO("%s %d: pos (%d, %d)\tval: %d\n", __FUNCTION__, __LINE__, g_pos_history[0].x, g_pos_history[0].y, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = WF_path_find_shortest_path(g_pos_history[0].x, g_pos_history[0].y, remote_x, remote_y, 0);
				ROS_INFO("%s %d: val %d\n", __FUNCTION__, __LINE__, val);

#if DEBUG_MAP
				debug_map(MAP, remote_x, remote_y);
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
			val = WF_path_find_shortest_path(g_pos_history[0].x, g_pos_history[0].y, remote_x, remote_y, 0);
			ROS_INFO("%s %d: val %d\n", __FUNCTION__, __LINE__, val);

#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
#if DEBUG_MAP
			debug_map(MAP, remote_x, remote_y);
#endif

			if (val < 0 || val == SCHAR_MAX) {
				/* No path to home, which is set when path planning is initialized. */
				val = 0;
			} else {
				val = 1;
			}
		}
	}

	ROS_INFO("%s %d: val %d\n", __FUNCTION__, __LINE__, val);
	return val;
}

int16_t path_escape_trapped()
{

	int16_t	val = 0;
	uint16_t i = 0;

	if ( g_trappedCell[0].X != g_home_x || g_trappedCell[0].Y != g_home_y ){
		for ( i = 0; i < g_trappedCellSize; ++i ) {
			ROS_WARN("%s %d Check %d trapped reference cell: x: %d, y: %d", __FUNCTION__, __LINE__,
			         i, g_trappedCell[i].X, g_trappedCell[i].Y);
			if (is_block_accessible(g_trappedCell[i].X, g_trappedCell[i].Y) == 0) {
				Map_Set_Cells(ROBOT_SIZE, g_trappedCell[i].X, g_trappedCell[i].Y, CLEANED);
			}

			val = path_find_shortest_path( g_pos_history[0].x, g_pos_history[0].y, g_trappedCell[i].X, g_trappedCell[i].Y, 0);
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
			val = path_find_shortest_path(g_pos_history[0].x, g_pos_history[0].y, 0, 0, 0);
#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
			ROS_WARN("%s %d: pos (%d, %d)\tval: %d", __FUNCTION__, __LINE__, g_pos_history[0].x, g_pos_history[0].y, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = path_find_shortest_path(g_pos_history[0].x, g_pos_history[0].y, g_home_x, g_home_y, 0);
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
			val = path_find_shortest_path(g_pos_history[0].x, g_pos_history[0].y, g_home_x, g_home_y, 0);
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

int8_t path_next(int32_t *target_x, int32_t *target_y, Point32_t *final_target_cell)
{

	/* Update the robot position history. */
	path_set_current_pos();

	/* Update the Map cells to avoid cells passed by the robot are marked as obstcals. */
	path_update_cells();

	path_reset_path_points();

	/*
	 * Check the current lane is clean or not and make sure the robot
	 * not non-stopply hit the wall, especially the wall with the black bricks.
	 */
	auto x_next_area = g_pos_history[0].x;
	auto y_next_area = g_pos_history[0].y;
	auto status = path_lane_is_cleaned(&x_next_area, &y_next_area);
	if (status == 1 && g_pos_history[0] == g_pos_history[1])
		status = 0;

	if ( status == 1 ) {
		uint8_t un_cleaned_cnt = 0;
		for (auto i = -1; i <= 1; ++i ) {
			for (auto j = -1; j <= 1; ++j ) {
				if ( Map_GetCell(MAP, x_next_area + i, y_next_area + j) == UNCLEAN ) {
					un_cleaned_cnt++;
				}
			}
		}
		if ( un_cleaned_cnt < 2) {
			status = 0;
		}
	}

	/*
	 * Clear the blocks as needed. A block as below
	 *
	 * 	111
	 * 	1u1
	 * 	111
	 *
	 * where u will be either 2 or 3, reset to
	 *
	 * 	111
	 * 	111
	 * 	111
	 *
	 */

	if (g_clear_block == 1) {
		ROS_WARN("Clear block\n");
		g_clear_block = 0;
		//return -1;
	}

	int16_t	val = 1;
	int16_t	x_tmp, y_tmp ;
	if (status > 0) {
		y_tmp = g_pos_history[0].y;
		ROS_INFO("%s %d: x1: %d\tx2: %d", __FUNCTION__, __LINE__, g_pos_history[1].x, g_pos_history[2].x);
		int16_t	 offset;
		if (x_next_area > g_pos_history[0].x) {
			x_tmp = (is_block_cleaned(x_next_area + 1, y_next_area) == 0) ? SHRT_MAX : x_next_area;

			if ((offset = path_ahead_to_clean(x_tmp, y_next_area, x_next_area)) != 0) {
//				ROS_INFO("%s %d: x: %d\tx_next_area: %d\toffset: %d", __FUNCTION__, __LINE__, x_tmp, x_next_area, offset);
				x_tmp = x_next_area + offset;// + (x == SHRT_MAX ? 2 : -2);
			}
		} else {
			x_tmp = (is_block_cleaned(x_next_area - 1, y_next_area) == 0) ? SHRT_MIN : x_next_area;

			if ((offset = path_ahead_to_clean(x_tmp, y_next_area, x_next_area)) != 0) {
//				ROS_INFO("%s %d: x: %d\tx_next_area: %d\toffset: %d", __FUNCTION__, __LINE__, x_tmp, x_next_area, offset);
				x_tmp = x_next_area + offset;// + (x == SHRT_MIN ? -2 : 2);
			}
		}
		if (g_first_start == 0)
			g_first_start++;

		g_pos_history[0].x_target = x_next_area;
		g_pos_history[0].y_target = y_next_area;
		g_last_dir = Map_GetXCell() > x_next_area ? SOUTH : NORTH;
	} else
	{
		/* Get the next target to clean. */
		debug_map(MAP, g_home_x, g_home_y);
		val = find_next_unclean_with_approaching(&x_next_area, &y_next_area);
//		ROS_INFO("%s %d: val: %d\t target: (%d, %d)\n", __FUNCTION__, __LINE__, val, x_next_area, y_next_area);
		if (val > 0) {
			if (g_first_start == 1)
				g_first_start++;

			g_pos_history[0].x_target = x_next_area;
			g_pos_history[0].y_target = y_next_area;
			(*final_target_cell).X = cellToCount(x_next_area);
			(*final_target_cell).Y = cellToCount(y_next_area);

			/* Find the path to the next target to clean. */
			//pos.X = Map_GetXCell();
			//pos.Y = Map_GetYCell();
			Cell_t	pos{x_next_area, y_next_area};
			val = path_move_to_unclean_area(pos, Map_GetXCell(), Map_GetYCell(), &x_tmp, &y_tmp);
			if (Map_GetXCell() == x_tmp) {
				g_last_dir = Map_GetYCell() > y_tmp ? WEST : EAST;
			} else {
				g_last_dir = Map_GetXCell() > x_tmp ? SOUTH : NORTH;
			}
			ROS_INFO("%s %d: x_next_area: %d\ty_next_area: %d\tx: %d\ty: %d\tlast_dir: %d", __FUNCTION__, __LINE__, x_next_area, y_next_area, x_tmp, y_tmp, g_last_dir);
		} else {
//			ROS_INFO("%s %d: val: %d\tx_next_area: %d\ty_next_area: %d\tx: %d\ty: %d", __FUNCTION__, __LINE__, val, x_next_area, y_next_area, x_tmp, y_tmp);
		}
		if (val == -2) {
			/* Robot is trapped and no path to starting point or home. */
			if (path_escape_trapped() == 0) {
				return 2;
			} else {
				return -1;
			}
		}
	}

	if (val > 0) {
		/*
		 * If the flag PP_MOVE_TO_CELL_CENTER is set, force the robot to move to
		 * the center of a cell for each action, otherwise, just use the encoder
		 * counter plus the offsets.
		 */
#ifdef PP_MOVE_TO_CELL_CENTER
		*target_x = (x_tmp == g_pos_history[0].x) ? Map_GetXCount() : cellToCount(x_tmp);
		*target_y = (y_tmp == g_pos_history[0].y) ? Map_GetYCount() : cellToCount(y_tmp);
#else
		*target_x = Map_GetXCount() + (x - g_pos_history[0].x) * CELL_COUNT_MUL;
		*target_y = Map_GetYCount() + (y - g_pos_history[0].y) * CELL_COUNT_MUL;
#endif
	}
	return val;
}

void path_escape_set_trapped_cell( Cell_t *cell, uint8_t size )
{
	uint8_t i = 0;
	g_trappedCellSize = size;
	for ( i = 0; i < g_trappedCellSize; ++i ) {
		g_trappedCell[i] = cell[i];
		ROS_INFO("%s %d Set %d trapped reference cell: x: %d\ty:%d", __FUNCTION__, __LINE__, i, g_trappedCell[i].X, g_trappedCell[i].Y);
	}
}

Cell_t *path_escape_get_trapped_cell()
{
	return g_trappedCell;
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
	g_home_x = countToCell(*x);
	g_home_y = countToCell(*y);

	/* Initialize the default settings. */
//	preset_action_count = 0;

//	weight_enabled = 1;

	for ( i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
		g_trappedCell[i].X = g_home_x;
		g_trappedCell[i].Y = g_home_y;
	}
	g_trappedCellSize = ESCAPE_TRAPPED_REF_CELL_SIZE;


#if (ROBOT_SIZE == 5)

//	g_weight_cnt_threshold = 4;

#else

//	g_weight_cnt_threshold = 3;

#endif

	g_direct_go = 0;

	g_pos_history[0].x = g_pos_history[0].y = 0;
	g_last_dir = 0;

//	try_entrance = 0;
	g_first_start = 0;
//	g_last_x_pos = g_last_y_pos = 0;

	/* Reset the poisition list. */
	for (i = 0; i < 3; i++) {
		g_pos_history[i].x = g_pos_history[i].y = i + 1;
	}

	/* Initialize the shortest path. */
	path_position_init(g_direct_go);
}

