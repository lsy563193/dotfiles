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
	Point16_t	target;
	list <Point16_t> points;
} PPTargetType;

list <PPTargetType> targets;

uint8_t	first_start = 0;

uint8_t try_entrance;

uint8_t	weight_enabled = 0;

/* Threshold about how many unclean block it will be discarded. */
uint8_t	weight_cnt_threshold = 5;

/* Enable direct go when there is no obstcal in between current pos. & dest. */
uint8_t direct_go = 0;

uint8_t max_try_cnt = 6;
uint8_t home_try_cnt = 0;
int16_t home_x, home_y;

uint8_t target_swapped = 0;

uint8_t	clear_block = 0;

PositionType positions[5];

int16_t	last_x_pos;
int16_t	last_y_pos;
uint16_t last_dir;

Point16_t trappedCell[ESCAPE_TRAPPED_REF_CELL_SIZE];
uint8_t trappedCellSize = 1;

extern int16_t xMin, xMax, yMin, yMax;

void path_targets_add_one(int16_t x, int16_t y, uint8_t accessible);

void path_set_current_pos(void);
void path_get_range(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);
uint16_t path_get_robot_direction(void);

uint8_t preset_action_count = 0;

#define Robot_GetDirection()	0

/*
 * Initialization function for path planning, it sets the starting
 * point as the home of the robot.
 *
 * @param *x	Pointer to robot home X coordinate
 * @param *y	Pointer to robot home Y coordinate
 *
 * @return
 */
void PathPlanning_Initialize(int32_t *x, int32_t *y) {
	int16_t i;

	/* Save the starting point as home. */
	home_x = countToCell(*x);
	home_y = countToCell(*y);

	/* Initialize the default settings. */
	preset_action_count = 0;

	weight_enabled = 1;

	for ( i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
		trappedCell[i].X = home_x;
		trappedCell[i].Y = home_y;
	}
	trappedCellSize = 1;

#if (ROBOT_SIZE == 5)

	weight_cnt_threshold = 4;

#else

	weight_cnt_threshold = 3;

#endif

	direct_go = 0;

	max_try_cnt = 1;
	home_try_cnt = 0;

	positions[0].x = positions[0].y = 0;
	last_dir = 0;

	try_entrance = 0;
	first_start = last_x_pos = last_y_pos = 0;

	/* Reset the poisition list. */
	for (i = 0; i < 3; i++) {
		positions[i].x = positions[i].y = i + 1;
	}

	/* Initialize the shortest path. */
	path_position_init(direct_go);

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

/*
 * Function to enable try the entrance first or not, an entrance only fits the robot
 * to get into, if it is set, the robot will try it first as soon as an entrance is
 * found. This is to increase the possibility to move to the entrance, otherwise, due
 * to the encoder and Gyro drift, it is hard to go into the entrance again after
 * cleaning some area.
 *
 * @param val	Enable or disable the try_entrance flag
 *
 * @return
 */
void path_set_try_entrance(uint8_t val)
{
	try_entrance = val;
}

/* Function to enable or disable the weight check for a target, if it is enable,
 * a target will check against its weight with the threshold value, it it is less
 * than the threshold value, discard the target.
 *
 * @param enable	Enable or disable the weight check for a target
 * @param count		Lower limit threshold that a target must be cleaned
 *
 * @return
 */
void path_set_weight(uint8_t enable, uint8_t count)
{
	weight_enabled = enable;
	weight_cnt_threshold = count;
}

/* Function to enable or disable direction go when moving to the target.
 * When enabled, robot will move to target when only if there is no obstcal
 * in between the robot and target.
 *
 * @param val	Enable or disable direction_go flag.
 *
 * @return
 */
void path_set_direct_go(uint8_t val)
{
	direct_go = val;
}

/*
 * Update current robot positions.
 *
 * @param
 *
 * @return
 */
void path_set_current_pos()
{
	positions[4] = positions[3];
	positions[3] = positions[2];
	positions[2] = positions[1];
	positions[1] = positions[0];

	positions[0].x = Map_GetXPos();
	positions[0].y = Map_GetYPos();
	positions[0].dir = path_get_robot_direction();
}

/*
 * Set the maximum try count for a target.
 *
 * @param val	Maximum try count for a target
 *
 * @return
 */
void path_set_max_try_cnt(uint8_t val)
{
	if (val > 0)
		max_try_cnt = val;
}

/*
 * Function to get the last robot movement's direction.
 *
 * @param
 *
 * @return	Last robot direction
 */
uint16_t path_get_robot_direction()
{
	return last_dir;
}

/*
 * Function to find the X/Y range of the Map, if the range is to small,
 * use the offset of those value to 3.
 *
 * @param *x_range_min	Pointer for minimum X value of the Map
 * @param *x_range_max	Pointer for maximum X value of the Map
 * @param *y_range_min	Pointer for minimum Y value of the Map
 * @param *y_range_max	Pointer for maximum Y value of the Map
 *
 * @return
 */
void path_get_range(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max)
{
	*x_range_min = xMin - (abs(xMin - xMax) <= 3 ? 3 : 1);
	*x_range_max = xMax + (abs(xMin - xMax) <= 3 ? 3 : 1);
	*y_range_min = yMin - (abs(yMin - yMax) <= 3? 3 : 1);
	*y_range_max = yMax + (abs(yMin - yMax) <= 3 ? 3 : 1);

	ROS_INFO("Get Range:\tx: %d - %d\ty: %d - %d\tx range: %d - %d\ty range: %d - %d",
		xMin, xMax, yMin, yMax, *x_range_min, *x_range_max, *y_range_min, *y_range_max);
}

/*
 * Reset the last X/Y & robot direction.
 *
 * @param
 * @param
 *
 * @return
 */
void path_reset_last_position(void)
{
	last_x_pos = Map_GetXPos();
	last_y_pos = Map_GetYPos();
}

/*
 * Check whether a given point is an blocked or not.
 *
 * @param x	X coordinate of the give point.
 * @param y	Y coordinate of the give point.
 *
 * @return	0 if the given point is not blocked
 * 		1 if the given point is blocked
 */
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

/*
 * Check a given point is blocked by bumper and/or cliff or not.
 *
 * @param x	X coordinate of the given point
 * @param y	Y coordinate of the given point
 *
 * @return	0 if it is not blocked by bumper and/or cliff
 *		1 if it is blocked by bumper and/or cliff
 */
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

/*
 * Check a block is accessible or not, a block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not accessible
 *		1 if the block is accessible
 */
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

/*
 * Check a block is cleanable or not, a block is defined as have the same size of brush.
 *
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not cleanable
 *		1 if the block is cleanable
 */
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

/*
 * Check a block is cleaned or not, a block is defined as have the same size of brush.
 *
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not cleaned
 *		1 if the block is cleaned
 */
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

/*
 * Check a block is uncleaned or not, a block is defined as have the same size of brush.
 * Since the brush occupies 3 cells, if there is any one of those 3 cells unclean, then the
 * block is treated as unclean.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is cleaned
 *		1 if the block is uncleaned
 */
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

/*
 * Check a block is on the boundary or not, a block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not on the boundary
 *		1 if the block is on the boundary
 */
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

/*
 * Check a block is accessible by the robot or not.
 * A block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not blocked by bumper, obs or cliff
 *		1 if the block is blocked
 */
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

/*
 * Check both ends of a lane are cleaned or not.
 *
 * @param *x	Pointer to the X coordinate that the robot should move to clean
 * @param *y	Pointer to the Y coordinate that the robot should move to clean
 *
 * @return	0 if both ends are cleaned
 * 		1 if either one end is not cleaned
 * 		2 if both ends are not cleaned
 */
uint8_t path_lane_is_cleaned(int16_t *x, int16_t *y) {
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
		 * previous robot positions. Otherwise, move to the end that have more unclean cells.
		 */
		if (min == max) {
			if (positions[2].y == positions[1].y) {
				if (positions[2].x == positions[1].x) {
					if (positions[0].x == positions[1].x) {
						*x += max;
					} else if (positions[0].x > positions[1].x) {
						*x -= min;
					} else {
						*x += max;
					}
				} else if ( positions[2].x > positions[1].x) {
					*x -= min;
				} else {
					*x += max;
				}
			} else if (positions[0].y == positions[1].y) {
				if (positions[0].x == positions[1].x) {
					*x += max;
				} else if (positions[0].x > positions[1].x) {
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

	for (i = xMin; i <= xMax; ++i) {
		for (j = yMin; j <= yMax; ++j) {
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

	x = Map_GetXPos();
	y = Map_GetYPos();
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
			if (i < xMin || i > xMax)
				continue;

				for (j = y - offset; j <= y + offset; j++) {
					if (j < yMin || j > yMax)
						continue;

				if(Map_GetCell(SPMAP, i, j) == passValue) {
					if (i - 1 >= xMin && Map_GetCell(SPMAP, i - 1, j) == COST_NO) {
						Map_SetCell(SPMAP, (i - 1), (j), (CellState)nextPassValue);
						passSet = 1;
					}

					if ((i + 1) <= xMax && Map_GetCell(SPMAP, i + 1, j) == COST_NO) {
						Map_SetCell(SPMAP, (i + 1), (j), (CellState)nextPassValue);
						passSet = 1;
					}

					if (j - 1  >= yMin && Map_GetCell(SPMAP, i, j - 1) == COST_NO) {
						Map_SetCell(SPMAP, (i), (j - 1), (CellState)nextPassValue);
						passSet = 1;
					}

					if ((j + 1) <= yMax && Map_GetCell(SPMAP, i, j + 1) == COST_NO) {
						Map_SetCell(SPMAP, (i), (j + 1), (CellState)nextPassValue);
						passSet = 1;
					}
				}
			}
		}

		all_set = true;
		for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
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

	for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
		if (Map_GetCell(SPMAP, it->target.X, it->target.Y) == COST_NO || Map_GetCell(SPMAP, it->target.X, it->target.Y) == COST_HIGH) {
			continue;
		}

		Point16_t	t;
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

			if ((tracex - 1 >= xMin) && (Map_GetCell(SPMAP, tracex - 1, tracey) == targetCost)) {
				tracex--;
				continue;
			}

			if ((tracex + 1 <= xMax) && (Map_GetCell(SPMAP, tracex + 1, tracey) == targetCost)) {
				tracex++;
				continue;
			}

			if ((tracey - 1 >= yMin) && (Map_GetCell(SPMAP, tracex, tracey - 1) == targetCost)) {
				tracey--;
				continue;
			}

			if ((tracey + 1 <= yMax) && (Map_GetCell(SPMAP, tracex, tracey + 1) == targetCost)) {
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

	for (c = xMin - 1; c < xMax + 1; ++c) {
		for (d = yMin - 1; d < yMax + 1; ++d) {
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

			if (c > xMin - 1 && Map_GetCell(MAP, c - 1, d) == UNCLEAN) {
				if (is_block_accessible(c - 1, d) == 1) {
					Map_SetCell(MAP, cellToCount(c - 1), cellToCount(d), TARGET);
					x_min = x_min > (c - 1) ? (c - 1) : x_min;
					x_max = x_max < (c - 1) ? (c - 1) : x_max;
					y_min = y_min > d ? d : y_min;
					y_max = y_max < d ? d : y_max;
				}
			}

			if (c < xMax + 1 && Map_GetCell(MAP, c + 1, d) == UNCLEAN) {
				if (is_block_accessible(c + 1, d) == 1) {
					Map_SetCell(MAP, cellToCount(c + 1), cellToCount(d), TARGET);
					x_min = x_min > (c + 1) ? (c + 1) : x_min;
					x_max = x_max < (c + 1) ? (c + 1) : x_max;
					y_min = y_min > d ? d : y_min;
					y_max = y_max < d ? d : y_max;
				}
			}

			if (d > yMin - 1 && Map_GetCell(MAP, c, d - 1) == UNCLEAN) {
				if (is_block_accessible(c, d - 1) == 1) {
					Map_SetCell(MAP, cellToCount(c), cellToCount(d - 1), TARGET);
					x_min = x_min > c ? c : x_min;
					x_max = x_max < c ? c : x_max;
					y_min = y_min > (d - 1) ? (d - 1) : y_min;
					y_max = y_max < (d - 1) ? (d - 1) : y_max;
				}
			}

			if (d < yMax + 1 && Map_GetCell(MAP, c, d + 1) == UNCLEAN) {
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
		for (c = x_min; c <= Map_GetXPos(); c++) {
			if (Map_GetCell(MAP, c, d) == TARGET) {
				if (start == SHRT_MAX)
					start = c;
				if (start != SHRT_MAX)
					end = c;
			}
			if (Map_GetCell(MAP, c, d) != TARGET || c == Map_GetXPos()){
				if ( start != SHRT_MAX) {
					for (a = start + 1; a < end; a++) {
						Map_SetCell(MAP, cellToCount(a), cellToCount(d), UNCLEAN);
					}
				}
				start = end = SHRT_MAX;
			}
		}

		start = end = SHRT_MIN;
		for (c = x_max; c >= Map_GetXPos(); c--) {
			if (Map_GetCell(MAP, c, d) == TARGET) {
				if (start == SHRT_MIN)
					start = c;
				if (start != SHRT_MIN)
					end = c;
			}
			if (Map_GetCell(MAP, c, d) != TARGET || c == Map_GetXPos()){
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

	for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
		it->points.clear();
	}

	targets.clear();
	for (c = x_min_tmp; c <= x_max_tmp; ++c) {
		for (d = y_min_tmp; d <= y_max_tmp; ++d) {
			if (Map_GetCell(MAP, c, d) == TARGET) {
				PPTargetType t;
				t.target.X = c;
				t.target.Y = d;
				t.points.clear();
				targets.push_back(t);

				x_min = x_min > c ? c : x_min;
				x_max = x_max < c ? c : x_max;
				y_min = y_min > d ? d : y_min;
				y_max = y_max < d ? d : y_max;
			}
		}
	}

	debug_map(MAP, home_x, home_y);
	for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
		Map_SetCell(MAP, cellToCount(it->target.X), cellToCount(it->target.Y), UNCLEAN);
	}

	path_find_all_targets();

	for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end();) {
		if (it->points.empty() == true) {
			it = targets.erase(it);
		} else {
			it++;
		}
	}

	/* No more target to clean */
	if (targets.empty() == true) {
		if (path_escape_trapped() <= 0) {
			ROS_WARN("%s %d: trapped", __FUNCTION__, __LINE__);
			return -2;
		}
		return 0;
	}

	ROS_INFO("%s %d: targets count: %d", __FUNCTION__, __LINE__, targets.size());
	for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
		std::string	msg = __FUNCTION__;
		msg += " " + std::to_string(__LINE__) + ": target (" + std::to_string(it->target.X) + ", " + std::to_string(it->target.Y) + ") " + std::to_string(it->points.size()) + ": ";

		for (list<Point16_t>::iterator i = it->points.begin(); i != it->points.end(); ++i) {
			msg += "(" + std::to_string(i->X) + ", " + std::to_string(i->Y) + ")->";
		}
		msg += "\n";
		ROS_INFO(msg.c_str());
	}

#if 1
	stop = 0;
	ROS_INFO("%s %d: case 1, towards Y+ only", __FUNCTION__, __LINE__);
	for (d = y_max; d >= Map_GetYPos(); --d) {
		if (stop == 1 && d <= Map_GetYPos() + 1) {
			break;
		}

		for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
			if (Map_GetCell(MAP, it->target.X, it->target.Y - 1) != CLEANED) {
				continue;
			}

			if (it->target.Y == d) {
				if (it->points.size() > final_cost) {
					continue;
				}

				last_y = it->points.front().Y;
				within_range = true;
				for (list<Point16_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
					if (i->Y < Map_GetYPos() || i->Y > d) {
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
		for (a = Map_GetYPos(); a >= y_min && stop == 0; --a) {
			for (d = a; d <= y_max && stop == 0; ++d) {
				for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
					if (it->target.Y == d) {
						if (it->points.size() > final_cost) {
							continue;
						}

						within_range = true;
						last_y = it->points.front().Y;
						bool turn = false;
						for (list<Point16_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
							if (i->Y < a || i->Y > (d > Map_GetYPos() ? d : Map_GetYPos())) {
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
		for (d = y_min; d >= Map_GetYPos(); ++d) {
			if (stop == 1 && d >= Map_GetYPos() - 1) {
				break;
			}

			for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
				if (Map_GetCell(MAP, it->target.X, it->target.Y + 1) != CLEANED) {
					continue;
				}

				if (it->target.Y == d) {
					if (it->points.size() > final_cost) {
						continue;
					}

					last_y = it->points.front().Y;
					within_range = true;
					for (list<Point16_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
						if (i->Y > Map_GetYPos() || i->Y < d) {
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
		for (a = Map_GetYPos(); a <= y_max && stop == 0; ++a) {
			for (d = a; d >= y_min && stop == 0; --d) {
				for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
					if (it->target.Y == d) {
						if (it->points.size() > final_cost) {
							continue;
						}

						within_range = true;
						last_y = it->points.front().Y;
						bool turn = false;
						for (list<Point16_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
							if (i->Y > a || i->Y < (d > Map_GetYPos() ? Map_GetYPos() : d)) {
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
		for (a = Map_GetYPos(); a <= y_max  && stop == 0; ++a) {
	            for (d = Map_GetYPos(); d <= a && stop == 0; ++d) {
				for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
					if (it->target.Y == d) {
						within_range = true;
						for (list<Point16_t>::iterator i = it->points.begin(); within_range == true && i != it->points.end(); ++i) {
							if (i->Y < Map_GetYPos() || i->Y > a) {
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
				for (list<PPTargetType>::iterator it = targets.begin(); it != targets.end(); ++it) {
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


/*
 * Find how many cells ahead to clean with a given target.
 *
 * @param x	X coordinate of robot before moving to the target
 * @param y	Y coordinate of robot before moving to the target
 * @param x_next	X coordinate of target
 *
 * @return
 */
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
		if ((x == SHRT_MIN && x_next + offset <= xMin) || (x == SHRT_MAX && x_next + offset >= xMax) ) {
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

/*
 * Update the Map cells for when robot move towards NORTH or SOUTH.
 * This is for avoiding the cells around the robot position is/are marked
 * as not accessible, since it shouldn't happen, the robot occupies 5x5
 * cells, if the robot is pass through, it means that the obstcal marked
 * before should be cleared.
 *
 * @param
 *
 * @return
 */
void path_update_cells()
{
	int16_t 	i, start, end;
	int32_t		y;
	CellState	cs;

	/* Skip, if robot is not moving towards NORTH or SOUTH. */
	if ((last_dir % 1800) != 0)
		return;

	start = positions[1].x > positions[0].x ? positions[0].x : positions[1].x;
	end = positions[1].x > positions[0].x ? positions[1].x : positions[0].x;
	ROS_INFO("%s %d: start: %d\tend: %d", __FUNCTION__, __LINE__, start, end);
	for (i = start; i <= end; i++) {
		/* Check the Map cells which Y coordinate equal (y - 2). */

#if (ROBOT_SIZE == 5)

		y = positions[0].y - 2;

#else

		y = positions[0].y - 1;

#endif

		if (Map_GetCell(MAP, i, y) == BLOCKED_OBS || Map_GetCell(MAP, i, positions[0].y - 2) == BLOCKED_BUMPER) {
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

		y = positions[0].y + 2;

#else

		y = positions[0].y + 1;

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
	if (last_dir == NORTH || last_dir == SOUTH) {
		if (last_dir == NORTH && positions[0].x > positions[1].x) {
			if (positions[0].y >= 0 && Map_GetCell(MAP, positions[0].x, positions[0].y + 2) == UNCLEAN) {
				for (i = 0; i < 3; i++) {
					cs = Map_GetCell(MAP, positions[0].x + 2, positions[0].y + i);
					if (cs != CLEANED && cs != UNCLEAN) {
						ROS_WARN("%s %d: reset (%d, %d) to %d.", __FUNCTION__, __LINE__, positions[0].x + 3, positions[0].y + i, cs);
						Map_SetCell(MAP, cellToCount(positions[0].x + 3), cellToCount(positions[0].y + i), cs);

						ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, positions[0].x + 2, positions[0].y + i);
						Map_SetCell(MAP, cellToCount(positions[0].x + 2), cellToCount(positions[0].y + i), UNCLEAN);
					}
				}
			} else if (positions[0].y < 0 && Map_GetCell(MAP, positions[0].x, positions[0].y - 2) == UNCLEAN) {
				for (i = 0; i < 3; i++) {
					cs = Map_GetCell(MAP, positions[0].x + 2, positions[0].y - i);
					if (cs != CLEANED && cs!= UNCLEAN) {
						ROS_WARN("%s %d: reset (%d, %d) to %d.", __FUNCTION__, __LINE__, positions[0].x + 3, positions[0].y - i, cs);
						Map_SetCell(MAP, cellToCount(positions[0].x + 3), cellToCount(positions[0].y - i), cs);

						ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, positions[0].x + 2, positions[0].y - i);
						Map_SetCell(MAP, cellToCount(positions[0].x + 2), cellToCount(positions[0].y - i), UNCLEAN);
					}
				}
			}
		} else if (last_dir == SOUTH && positions[0].x < positions[1].x) {
			if (positions[0].y >= 0 && Map_GetCell(MAP, positions[0].x, positions[0].y + 2) == UNCLEAN) {
				for (i = 0; i < 3; i++) {
					cs = Map_GetCell(MAP, positions[0].x - 2, positions[0].y + i);
					if (cs != CLEANED && cs!= UNCLEAN) {
						ROS_WARN("%s %d: reset (%d, %d) to %d.", __FUNCTION__, __LINE__, positions[0].x - 3, positions[0].y + i, cs);
						Map_SetCell(MAP, cellToCount(positions[0].x - 3), cellToCount(positions[0].y + i), cs);

						ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, positions[0].x - 2, positions[0].y + i);
						Map_SetCell(MAP, cellToCount(positions[0].x - 2), cellToCount(positions[0].y + i), UNCLEAN);
					}
				}
			} else if (positions[0].y < 0 && Map_GetCell(MAP, positions[0].x, positions[0].y - 2) == UNCLEAN) {
				for (i = 0; i < 3; i++) {
					cs = Map_GetCell(MAP, positions[0].x - 2, positions[0].y - i);
					if (cs != CLEANED && cs!= UNCLEAN) {
						ROS_WARN("%s %d: reset (%d, %d) to %d.", __FUNCTION__, __LINE__, positions[0].x - 3, positions[0].y - i, cs);
						Map_SetCell(MAP, cellToCount(positions[0].x - 3), cellToCount(positions[0].y - i), cs);

						ROS_WARN("%s %d: reset (%d, %d) to unclean.", __FUNCTION__, __LINE__, positions[0].x - 2, positions[0].y - i);
						Map_SetCell(MAP, cellToCount(positions[0].x - 2), cellToCount(positions[0].y - i), UNCLEAN);
					}
				}
			}
		}
	}
}


/*
 * Check whether the robot is trapped or not. The robot is trapped if there
 * is no path to (0, 0) or home.
 *
 * @param
 *
 * @return	0 if the robot is trapped
 * 		1 if the robot is not trapped.
 */
int16_t WF_path_escape_trapped() {

	int16_t	val = 0;
	uint16_t i = 0;
	int16_t remote_x = 0, remote_y = 0;
	int16_t x_min, x_max, y_min, y_max;
	Point32_t	Remote_Point;
	Point16_t   tmpPnt, pnt16ArTmp[3];
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

	if ( trappedCell[0].X != remote_x || trappedCell[0].Y != remote_y ){
		for ( i = 0; i < trappedCellSize; ++i ) {
			ROS_INFO("%s %d Check %d trapped reference cell: x: %d, y: %d\n", __FUNCTION__, __LINE__,
			         i, trappedCell[i].X, trappedCell[i].Y);
			/*if (is_block_accessible(trappedCell[i].X, trappedCell[i].Y) == 0) {
				Map_Set_Cells(ROBOT_SIZE, trappedCell[i].X, trappedCell[i].Y, CLEANED);
			}*/

			//val = WF_path_find_shortest_path( positions[0].x, positions[0].y, trappedCell[i].X, trappedCell[i].Y, 0);
			val = WF_path_find_shortest_path( positions[0].x, positions[0].y, trappedCell[i].X, trappedCell[i].Y, 0);
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
			val = WF_path_find_shortest_path(positions[0].x, positions[0].y, 0, 0, 0);
#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
			ROS_INFO("%s %d: pos (%d, %d)\tval: %d\n", __FUNCTION__, __LINE__, positions[0].x, positions[0].y, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = WF_path_find_shortest_path(positions[0].x, positions[0].y, remote_x, remote_y, 0);
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
			val = WF_path_find_shortest_path(positions[0].x, positions[0].y, remote_x, remote_y, 0);
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
/*
 * Check whether the robot is trapped or not. The robot is trapped if there
 * is no path to (0, 0) or home.
 *
 * @param
 *
 * @return	0 if the robot is trapped
 * 		1 if the robot is not trapped.
 */
int16_t path_escape_trapped() {

	int16_t	val = 0;
	uint16_t i = 0;

	if ( trappedCell[0].X != home_x || trappedCell[0].Y != home_y ){
		for ( i = 0; i < trappedCellSize; ++i ) {
			ROS_WARN("%s %d Check %d trapped reference cell: x: %d, y: %d", __FUNCTION__, __LINE__,
			         i, trappedCell[i].X, trappedCell[i].Y);
			if (is_block_accessible(trappedCell[i].X, trappedCell[i].Y) == 0) {
				Map_Set_Cells(ROBOT_SIZE, trappedCell[i].X, trappedCell[i].Y, CLEANED);
			}

			val = path_find_shortest_path( positions[0].x, positions[0].y, trappedCell[i].X, trappedCell[i].Y, 0);
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
			val = path_find_shortest_path(positions[0].x, positions[0].y, 0, 0, 0);
#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
			ROS_WARN("%s %d: pos (%d, %d)\tval: %d", __FUNCTION__, __LINE__, positions[0].x, positions[0].y, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = path_find_shortest_path(positions[0].x, positions[0].y, home_x, home_y, 0);
				ROS_WARN("%s %d: val %d", __FUNCTION__, __LINE__, val);

#if DEBUG_MAP
				debug_map(MAP, home_x, home_y);
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
			val = path_find_shortest_path(positions[0].x, positions[0].y, home_x, home_y, 0);
			ROS_WARN("%s %d: val %d", __FUNCTION__, __LINE__, val);

#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
#if DEBUG_MAP
			debug_map(MAP, home_x, home_y);
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

/*
 * Function to find the next target to clean. When the robot goes to a new
 * lane, it will try to make sure the new lane is cleaned on both ends, then
 * try to find a target to clean from the target list. The logic below will let
 * the robot move in ZigZag. and robot always try to clean the most left hand
 * side(which target has a greater Y coordiante) with reference from the grid Map.
 *
 * @param *target_x	Pointer to target position's X coordinate
 * @param *target_y	Pointer to target position's Y coordinate
 *
 * @return	0 if no more target is found
 * 		1 if a target is found
 * 		2 if robot is trapped
 * 		-1 if target is blocked
 */
int8_t path_next(int32_t *target_x, int32_t *target_y, Point32_t *final_target_cell) {
	int16_t	val;
	uint8_t status;
	int16_t	x, y, cnt, x_next_area, y_next_area, offset;
	int8_t i, j;
	Point16_t	pos;

	/* Update the robot position history. */
	path_set_current_pos();

	val = 1;
	/* Update the Map cells to avoid cells passed by the robot are marked as obstcals. */
	path_update_cells();

	path_reset_path_points();

	ROS_INFO("\n");
	ROS_INFO("%s %d: x: %d\ty: %d\tlx: %d\tly: %d\tlast_dir: %d", __FUNCTION__, __LINE__, positions[0].x, positions[0].y, last_x_pos, last_y_pos, last_dir);

	/*
	 * Check the current lane is clean or not and make sure the robot
	 * not non-stopply hit the wall, especially the wall with the black bricks.
	 */
	x_next_area = positions[0].x;
	y_next_area = positions[0].y;
	status = path_lane_is_cleaned(&x_next_area, &y_next_area);
	if (status == 1 && positions[0].x == positions[1].x && positions[0].y == positions[1].y && positions[0].dir == positions[1].dir) {
		status = 0;
	}

	if ( status == 1 ) {
		uint8_t unCleanedCnt = 0;
		for ( i = -1; i <= 1; ++i ) {
			for ( j = -1; j <= 1; ++j ) {
				if ( Map_GetCell(MAP, x_next_area + i, y_next_area + j) == UNCLEAN ) {
					unCleanedCnt++;
				}
			}
		}

		if ( unCleanedCnt < 2) {
			status = 0;
		}
	}

	ROS_INFO("status: %d\tx next: %d\ty next: %d\tcell: %d",
			status, x_next_area, y_next_area, Map_GetCell(MAP, x_next_area, y_next_area));

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

	if (clear_block == 1) {
		ROS_WARN("Clear block\n");
		clear_block = 0;
		//return -1;
	}

	if (status > 0) {
		y = positions[0].y;
		ROS_INFO("%s %d: x1: %d\tx2: %d", __FUNCTION__, __LINE__, positions[1].x, positions[2].x);
		if (x_next_area > positions[0].x) {
			x = (is_block_cleaned(x_next_area + 1, y_next_area) == 0) ? SHRT_MAX : x_next_area;

			if ((offset = path_ahead_to_clean(x, y_next_area, x_next_area)) != 0) {
				ROS_INFO("%s %d: x: %d\tx_next_area: %d\toffset: %d", __FUNCTION__, __LINE__, x, x_next_area, offset);
				x = x_next_area + offset;// + (x == SHRT_MAX ? 2 : -2);
			}
		} else {
			x = (is_block_cleaned(x_next_area - 1, y_next_area) == 0) ? SHRT_MIN : x_next_area;

			if ((offset = path_ahead_to_clean(x, y_next_area, x_next_area)) != 0) {
				ROS_INFO("%s %d: x: %d\tx_next_area: %d\toffset: %d", __FUNCTION__, __LINE__, x, x_next_area, offset);
				x = x_next_area + offset;// + (x == SHRT_MIN ? -2 : 2);
			}
		}
		if (first_start == 0)
			first_start++;

		positions[0].x_target = x_next_area;
		positions[0].y_target = y_next_area;
		last_dir = Map_GetXPos() > x_next_area ? SOUTH : NORTH;
	} else {
		/* Get the next target to clean. */
		debug_map(MAP, home_x, home_y);
		val = find_next_unclean_with_approaching(&x_next_area, &y_next_area);
		ROS_INFO("%s %d: val: %d\t target: (%d, %d)\n", __FUNCTION__, __LINE__, val, x_next_area, y_next_area);
		if (val > 0) {
			if (first_start == 1)
				first_start++;

			positions[0].x_target = x_next_area;
			positions[0].y_target = y_next_area;
			(*final_target_cell).X = cellToCount(x_next_area);
			(*final_target_cell).Y = cellToCount(y_next_area);

			/* Find the path to the next target to clean. */
			//pos.X = Map_GetXPos();
			//pos.Y = Map_GetYPos();
			pos.X = x_next_area;
			pos.Y = y_next_area;
			val = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &x, &y);
			if (Map_GetXPos() == x) {
				last_dir = Map_GetYPos() > y ? WEST : EAST;
			} else {
				last_dir = Map_GetXPos() > x ? SOUTH : NORTH;
			}
			ROS_INFO("%s %d: x_next_area: %d\ty_next_area: %d\tx: %d\ty: %d\tlast_dir: %d", __FUNCTION__, __LINE__, x_next_area, y_next_area, x, y, last_dir);
		} else {
			ROS_INFO("%s %d: val: %d\tx_next_area: %d\ty_next_area: %d\tx: %d\ty: %d", __FUNCTION__, __LINE__, val, x_next_area, y_next_area, x, y);
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
		*target_x = (x == positions[0].x) ? Map_GetXCount() : cellToCount(x);
		*target_y = (y == positions[0].y) ? Map_GetYCount() : cellToCount(y);
#else
		*target_x = Map_GetXCount() + (x - positions[0].x) * CELL_COUNT_MUL;
		*target_y = Map_GetYCount() + (y - positions[0].y) * CELL_COUNT_MUL;
#endif
	}

//done:
#ifdef	DEBUG_SM_MAP
	/* If the flag DEBUG_SM_MAP is set, print the shorest path map for debugging. */
	debug_map(SPMAP, countToCell(*target_x), countToCell(*target_y));
#endif

#if DEBUG_MAP
	/* If the flag DEBUG_MAP is set, print the map for debugging. */
	if (x == SHRT_MIN)
		debug_map(MAP, xMin, y);
	else if (x == SHRT_MAX)
		debug_map(MAP, xMax, y);
	else
		debug_map(MAP, x, y);
#endif

	ROS_INFO("%s %d: x: %d(%d)\ty: %d(%d)\t next dest: %d\tx: %d(%d)\ty: %d(%d)\n", __FUNCTION__, __LINE__,
		positions[0].x, Map_GetXCount(), positions[0].y, Map_GetYCount(), val, countToCell(*target_x), *target_x, countToCell(*target_y), *target_y);

	return val;
}

#if 0
/*
 * Function for the robot to go back to its starting point. If its
 * starting point is blocked or somehow not accessible, it will try
 * to find a nearest point to the starting point and move to it,
 * but such point is limited to (home_x -/+ 20, home_y -/+ 20)
 *
 * @param *target_x	Pointer to target position's X coordinate
 * @param *target_y	Pointer to target position's Y coordinate
 *
 * @return	0 if robot reaches its home point or excess max home try count,
 * 		other wise the cost to next stop point
 */
uint8_t path_home(int32_t *target_x, int32_t *target_y) {
	uint8_t		stop;
	int16_t		x, x_next, y, y_next, i, j, cost, offset, retval;
	Point16_t	pos;

	/* Clear the map blocks which path_home is called. */
	if (home_try_cnt == 0) {
		Map_ClearBlocks();
		Map_Set_Cells(ROBOT_SIZE, countToCell(*target_x), countToCell(*target_y), CLEANED);
	}

	path_set_current_pos();

	ROS_INFO("path_home: current: (%d, %d) (%d, %d) \thome: (%d, %d)\tdir: %d",
			positions[0].x, positions[0].y, Map_GetXCount(), Map_GetYCount(), countToCell(*target_x), countToCell(*target_y));

#if DEBUG_MAP
	/* If the flag DEBUG_MAP is set, print the map for debugging. */
	debug_map(MAP, countToCell(*target_x), countToCell(*target_y));
#endif
	cost = -1;
	offset = retval = 0;
	x_next = countToCell(*target_x);
	y_next = countToCell(*target_y);

	/* Stop only when the robot reaches the starting point. */
	if (!(x_next == positions[0].x && y_next == positions[0].y)) {
		stop = 0;
		while (!stop) {
			 /* Continuously find the suitable point that closest to the robot's starting point, with the maximum offset 20. */
			for (i = countToCell(*target_x) - offset; (i <= countToCell(*target_x) + offset) && (i <= MAP_SIZE); i += 2) {
				for (j = countToCell(*target_y) - offset; (j <= countToCell(*target_y) + offset) && (j <= MAP_SIZE); j += 2) {
					if (!(i == (countToCell(*target_x) - offset) || i == (countToCell(*target_x) + offset) || j == (countToCell(*target_y) - offset) || j == (countToCell(*target_y) + offset)))
						continue;

					if (Map_GetCell(MAP, i, j) == UNCLEAN || (is_a_block(i, j) == 1))
						continue;

					/* If unreachable from current position, clear the blocks. */
					if (is_block_accessible(i, j) == 0) {
						//Map_ClearBlocks();
						continue;
					}

					/* If no path found, clear the blocks. */
					if (path_find_shortest_path(positions[0].x, positions[0].y, i, j, 0) < 0) {
						//Map_ClearBlocks();
						continue;
					}
					ROS_INFO("%s %d: cost: %d\toffset: %d", __FUNCTION__, __LINE__, cost, abs(i - countToCell(*target_x)) + abs(j - countToCell(*target_y)));
					if (cost == -1 || cost > (abs(i - countToCell(*target_x)) + abs(j - countToCell(*target_y)))) {
						cost = abs(i- countToCell(*target_x)) + abs(j - countToCell(*target_y));
						x_next = i;
						y_next = j;
					}
				}
			}

			if (cost == -1 || (offset != 0 && offset <= cost)) {
				if (offset < MAP_SIZE && (abs(positions[0].x) + abs(positions[0].y) > offset)) {
					offset += 2;
					/* If over the offset limit, stop. */
					if (offset > 10) {
						ROS_WARN("%s %d: stop robot, offset is too large, home (%d, %d)", __FUNCTION__, __LINE__, x_next, y_next);
						retval = 0;
						*target_x = Map_GetXCount();
						*target_y = Map_GetYCount();
					}
				} else {
					ROS_WARN("%s %d: stop robot, no path to home (%d, %d)", __FUNCTION__, __LINE__, x_next, y_next);
					retval = 0;
					*target_x = Map_GetXCount();
					*target_y = Map_GetYCount();
					stop = 1;
				}
			} else {
				/* If no path find anymore, stop. */
				//pos.X = Map_GetXPos();
				//pos.Y = Map_GetYPos();
				pos.X = x_next;
				pos.Y = y_next;
				if ((retval = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &x, &y)) <= 0) {
					ROS_WARN("%s %d: stop robot, no path to home (%d, %d)", __FUNCTION__, __LINE__, x_next, y_next);
					retval = 0;
					*target_x = Map_GetXCount();
					*target_y = Map_GetYCount();
				} else {
					CM_SetHome(cellToCount(x_next), cellToCount(y_next));
#ifdef PP_MOVE_TO_CELL_CENTER
					/*
					 * If the flag PP_MOVE_TO_CELL_CENTER is set, force the robot to move to
					 * the center of a cell for each action, otherwise, just use the encoder
					 * counter plus the offsets.
					 */
					*target_x = (x == positions[0].x) ? Map_GetXCount() : cellToCount(x);
					*target_y = (y == positions[0].y) ? Map_GetYCount() : cellToCount(y);
#else
					*target_x = Map_GetXCount() + (x - positions[0].x) * CELL_COUNT_MUL;
					*target_y = Map_GetYCount() + (y - positions[0].y) * CELL_COUNT_MUL;
#endif
				}
				stop = 1;
			}
		}
	}

	/* If the robot movement is only 1 cells, increase the try count. */
	if (((abs(positions[0].x - positions[1].x) <= 1) && (abs(positions[0].y - positions[1].y) <= 1))) {
		home_try_cnt++;
	}

	/* If the try count over 5, stop. */
	if (home_try_cnt > 5) {
		retval = 0;
		*target_x = Map_GetXCount();
		*target_y = Map_GetYCount();
	}

	ROS_INFO("home next dest:\t%d\tx: %d(%d)\ty: %d(%d)\tcnt: %d", retval, countToCell(*target_x), *target_x, countToCell(*target_y), *target_y, home_try_cnt);

	return retval;
}
#endif

void path_escape_set_trapped_cell( Point16_t *cell, uint8_t size ) {
	uint8_t i = 0;
	trappedCellSize = size;
	for ( i = 0; i < trappedCellSize; ++i ) {
		trappedCell[i] = cell[i];
		ROS_INFO("%s %d Set %d trapped reference cell: x: %d\ty:%d", __FUNCTION__, __LINE__, i, trappedCell[i].X, trappedCell[i].Y);
	}
}

Point16_t *path_escape_get_trapped_cell() {
	return trappedCell;
}

int16_t path_get_home_x() {
	return home_x;
}

int16_t path_get_home_y() {
	return home_y;
}
