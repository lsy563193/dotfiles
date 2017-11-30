#ifndef __PATHPLANNING_H__
#define __PATHPLANNING_H__

#include "config.h"
#include "map.h"
#include "mathematics.h"
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
extern bool g_keep_on_wf;
extern MapDirection g_new_dir;
extern MapDirection g_old_dir;
extern Cell_t g_home;
extern Cell_t g_zero_home;
extern bool g_home_gen_rosmap;
extern Cell_t g_home_point;
extern int g_wf_reach_count;
extern bool g_check_path_in_advance;
extern bool g_allow_check_path_in_advance;
extern Cell_t g_virtual_target;//for followall

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
bool path_next(const Cell_t& curr, PPTargetType& path);

bool cs_path_next(const Cell_t& start, PPTargetType& path);
/*
 * Generating the pose direction for every target cell in the path, and return the direction towards the first target.
 * @param	start: Start cell of this path.
 * @param	path: The target list.
 *
 * @return	POS_X/POS_Y/NEG_X/NEG_Y for indicating the direction from start cell to first target.
 */
bool cm_turn_and_check_charger_signal(void);
MapDirection path_full_angle(const Cell_t& start, PPTargetType& path);

//void path_update_cell_history(void);


bool path_target(const Cell_t& curr, PPTargetType& path);
//int16_t path_target2(const Cell_t& curr, PPTargetType& path);
void path_find_all_targets(const Cell_t& curr, BoundingBox2& map);

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
bool cm_is_reach();

bool path_next_spot(const Cell_t &start, PPTargetType &path);
bool path_next_fw(const Cell_t &start);
bool path_next_nav(const Cell_t &start, PPTargetType &path);
/*
 * Calculate the path to next target in advance.
 * @param	dir: variable that saves the new direction from start cell towards first target if it returns true.
 * @param	start: Start cell.
 * @param	path: The target list.
 *
 * @return	true if there is any valid uncleaned targets.
 * 			false if there is no more uncleaned targets.
 */
bool path_next_nav_in_advance(int16_t &dir, const Cell_t &start, PPTargetType &path);
void path_escape_set_trapped_cell( Cell_t *cell, uint8_t size );

Cell_t *path_escape_get_trapped_cell(void);

void path_set_home(const Cell_t& cell);

bool path_get_home_point_target(const Cell_t& curr, PPTargetType& path);

int16_t path_get_home_x(void);

int16_t path_get_home_y(void);

/*
 * Function to get the last robot movement's direction.
 *
 * @param
 *
 * @return	Last robot direction
 */
MapDirection path_get_robot_direction(void);

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
