#ifndef __PATHPLANNING_H__
#define __PATHPLANNING_H__

#include "config.h"
#include "map.h"
#include "mathematics.h"
#include "debug.h"
#include "BoundingBox.h"

#include <list>
#include <deque>

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

//typedef struct {
//	Cell_t	target;
//	std::list <Cell_t> cells;
//} PPTargetType;
typedef std::list <Cell_t> PPTargetType;
typedef enum {
	USE_ROS,
	USE_UNCLEAN,
	USE_CLEANED,
	HOMEWAY_NUM
}HomeWay_t;
//#define NO_TARGET_LEFT 0
//#define TARGET_REACHED 0
//#define TARGET_FOUND 1
extern std::vector<Cell_t> g_homes;
extern std::vector<int> g_home_way_list;
extern std::vector<int>::iterator g_home_way_it;
extern bool g_go_home;
extern bool g_keep_on_wf;
extern bool g_no_uncleaned_target;
extern Cell_t g_home;
extern Cell_t g_zero_home;
extern bool g_home_gen_rosmap;
extern int g_wf_reach_count;
/*
 * Function to find the X/Y range of the Map or wfMap, if the range is to small,
 * use the offset of those value to 3.
 *
 * @param *x_range_min	Pointer for minimum X value of the Map
 * @param *x_range_max	Pointer for maximum X value of the Map
 * @param *y_range_min	Pointer for minimum Y value of the Map
 * @param *y_range_max	Pointer for maximum Y value of the Map
 *
 * @return
 */
void path_get_range(uint8_t id, int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);

/*
 * Update current robot g_pos_history.
 *
 * @param
 *
 * @return
 */
//void path_update_cell_history(void);

/*
 * Initialization function for path planning, it sets the starting
 * point as the home of the robot.
 *
 * @param cell	robot home cell coordinate
 *
 * @return
 */
void path_planning_initialize();

/*
 * Initialization function for path planning in wall follow mode, it sets the starting
 * point as the home of the robot.
 *
 * @param cell	robot home cell coordinate
 *
 * @return
 */
void wf_path_planning_initialize();

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
bool path_next(const Cell_t& curr, PPTargetType& path, const int is_reach);

//void path_update_cell_history(void);

uint16_t path_get_robot_direction(void);

int16_t path_target(const Cell_t& curr, PPTargetType& path);
//int16_t path_target2(const Cell_t& curr, PPTargetType& path);
void path_find_all_targets(const Cell_t& curr, BoundingBox2& map);

void generate_SPMAP(const Cell_t& curr);

bool get_reachable_targets(const Cell_t& curr, BoundingBox2& map);

void generate_path_to_targets(const Cell_t& curr);

bool path_select_target(const Cell_t& curr, Cell_t& temp_target, const BoundingBox2& map);

int16_t isolate_target(const Cell_t& curr, PPTargetType& path);

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
bool path_full(const Cell_t& curr, PPTargetType& path);

/*
 * Check whether the robot is trapped or not. The robot is trapped if there
 * is no path to (0, 0) or home.
 *
 * @param
 *
 * @return	0 if the robot is trapped
 * 		1 if the robot is not trapped.
 */
int16_t path_escape_trapped(const Cell_t& curr);

void path_escape_set_trapped_cell( Cell_t *cell, uint8_t size );

Cell_t *path_escape_get_trapped_cell(void);

void path_set_home(const Cell_t& cell);

bool path_get_home_target(const Cell_t& curr, PPTargetType& path, const int is_reach);

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

uint8_t is_block_blocked_x_axis(int16_t x, int16_t y);
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
uint8_t is_block_unclean(int16_t x, int16_t y);

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
int8_t is_block_cleaned_unblock(int16_t x, int16_t y);

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
//int8_t is_block_cleanable(int16_t x, int16_t y);

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

// This function is for setting the continue cell for robot to go after charge.
void path_set_continue_cell(Cell_t cell);

bool path_get_continue_target(const Cell_t& curr, PPTargetType& path);

/*
 * Function to fill the path list with every cell that it will pass.
 *
 * @param path: Cell list that only contains the starting cells and the turning cells and destination.
 *
 * @return none. But it will change the path to cell list that contains all the cells in the path from starting cell to destination.
 */
void path_fill_path(std::list<Cell_t>& path);

bool path_dijkstra(const Cell_t& curr, Cell_t& p_goal, int& cleaned_count);

bool is_fobbit_free();

bool fw_is_time_up();
#endif
