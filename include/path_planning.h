#ifndef __PATHPLANNING_H__
#define __PATHPLANNING_H__

#include "config.h"

#include "map.h"
#include "mathematics.h"
#include "debug.h"

typedef struct PositionType_{
	Cell_t	cell;
//	uint16_t dir;
	friend bool operator==(const PositionType_ left, const PositionType_ right)
	{
		return left.cell.X == right.cell.X && left.cell.Y == right.cell.Y/* && left.dir == right.dir*/;
	}
} PositionType;

typedef struct {
	int16_t	x;
	int16_t	y;
	int8_t	x_pos;
	int8_t	y_pos;
	uint8_t	state;
	uint8_t	try_cnt;
} TargetType;

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
void path_get_range(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);

/*
 * Update current robot g_pos_history.
 *
 * @param
 *
 * @return
 */
void path_update_cell_history(void);

/*
 * Initialization function for path planning, it sets the starting
 * point as the home of the robot.
 *
 * @param *x	Pointer to robot home X coordinate
 * @param *y	Pointer to robot home Y coordinate
 *
 * @return
 */
void path_planning_initialize(int32_t *x, int32_t *y);

/*
 * Initialization function for path planning in wall follow mode, it sets the starting
 * point as the home of the robot.
 *
 * @param *x	Pointer to robot home X coordinate
 * @param *y	Pointer to robot home Y coordinate
 *
 * @return
 */
void WF_PathPlanning_Initialize(int32_t *x, int32_t *y);

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
int8_t path_next(Point32_t *next_point, Point32_t *target_point);

uint8_t path_home(int32_t *x, int32_t *y);

void path_update_cell_history(void);

void path_set_max_try_cnt(uint8_t val);

uint16_t path_get_robot_direction(void);

void path_get_range(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);

void path_reset_last_position(void);

/*
 * Check a block is accessible or not, a block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not accessible
 *		1 if the block is accessible
 */
uint8_t is_block_accessible(int16_t x, int16_t y);

uint16_t path_targets_get_count(void);

void path_targets_clear_list(void);

uint8_t path_targets_get_last(int16_t *x, int16_t *y);

uint8_t path_targets_try_add_one(int16_t x, int16_t y, uint8_t accessible);

void path_targets_add_one(int16_t x, int16_t y, uint8_t accessible);

void path_targets_update(void);

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
void path_update_cells(void);

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
bool path_lane_is_cleaned(Cell_t& next);

/*
 * Find how many cells ahead to clean with a given target.
 *
 * @param x	X coordinate of robot before moving to the target
 * @param y	Y coordinate of robot before moving to the target
 * @param x_next	X coordinate of target
 *
 * @return
 */
int16_t path_ahead_to_clean(int16_t x, Cell_t next);

/*
 * Check whether the robot is trapped or not. The robot is trapped if there
 * is no path to (0, 0) or home.
 *
 * @param
 *
 * @return	0 if the robot is trapped
 * 		1 if the robot is not trapped.
 */
int16_t path_escape_trapped(void);

 /* Set the around 9 cell as state */
void path_set_9cell(int16_t cell_x, int16_t cell_y, CellState state);

 /* Set the around 9 cell as state */
void path_set_25cell(int16_t cell_x, int16_t cell_y, CellState state);

void path_escape_set_trapped_cell( Cell_t *cell, uint8_t size );

Cell_t *path_escape_get_trapped_cell(void);

int16_t path_get_home_x(void);

int16_t path_get_home_y(void);

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
uint8_t is_block_blocked(int16_t x, int16_t y);

/*
 * Check a block is on the boundary or not, a block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is not on the boundary
 *		1 if the block is on the boundary
 */
uint8_t is_block_boundary(int16_t x, int16_t y);

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
bool is_brush_block_unclean(int16_t x, int16_t y);

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
int8_t is_block_cleaned(int16_t x, int16_t y);

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
int8_t is_block_cleanable(int16_t x, int16_t y);

/*
 * Check a given point is blocked by bumper and/or cliff or not.
 *
 * @param x	X coordinate of the given point
 * @param y	Y coordinate of the given point
 *
 * @return	0 if it is not blocked by bumper and/or cliff
 *		1 if it is blocked by bumper and/or cliff
 */
uint8_t is_blocked_by_bumper(int16_t x, int16_t y);

/*
 * Check whether a given point is an blocked or not.
 *
 * @param x	X coordinate of the give point.
 * @param y	Y coordinate of the give point.
 *
 * @return	0 if the given point is not blocked
 * 		1 if the given point is blocked
 */
uint8_t is_a_block(int16_t x, int16_t y);

/*
 * Function to get the last robot movement's direction.
 *
 * @param
 *
 * @return	Last robot direction
 */
uint16_t path_get_robot_direction(void);

#endif
