#ifndef __MAP_H__
#define __MAP_H__

#include "config.h"
#include <deque>
#include <vector>
#include "mathematics.h"
#include "BoundingBox.h"

#define MAP 0
#define SPMAP 1

typedef std::deque<Cell_t> PPTargetType;

typedef enum {
  UNCLEAN  = 0,
  CLEANED = 1,
  BLOCKED = 2,
  BLOCKED_OBS = 2,
  BLOCKED_BUMPER = 3,
  BLOCKED_CLIFF = 4,
  BLOCKED_RCON = 5,
  BLOCKED_LASER = 6,
  BLOCKED_TILT = 7,
  BLOCKED_SLIP = 8,
  BLOCKED_ROS_MAP = 9,
  BLOCKED_BOUNDARY = 10,
  TARGET_CLEAN = 13,
  TARGET = 14,
  COST_NO = 0,
  COST_1 = 1,
  COST_2 = 2,
  COST_3 = 3,
  COST_4 = 4,
  COST_5 = 5,
  COST_PATH = 6,
  COST_HIGH = 7,
} CellState;

typedef enum {
  POS_X = 0,
  PX_PY = 450,
  POS_Y = 900,
  NS_PY = 1350,
  NEG_X = 1800,
  NX_NY =-1350,
  NEG_Y =-900,
  PX_NY =-450,
  NONE = 0,
} Direction_Cardinal;

#define IS_POS_AXIS(x) (x == POS_X || x == POS_Y || x == NONE)
#define	IS_X_AXIS(x) (x == NEG_X || x == POS_X || x == NONE)
#define	IS_Y_AXIS(x) (x == POS_Y || x == NEG_Y)

class CostMap {
public:

	CostMap();

	static int32_t get_x_count(void);

	static int32_t get_y_count(void);

	static Point32_t get_curr_point(void);

	static int16_t get_x_cell(void);

	static int16_t get_y_cell(void);

	static Cell_t get_curr_cell();

	void set_cell(uint8_t id, int32_t x, int32_t y, CellState value);

	static void set_position(double x, double y);

	static Point32_t get_relative(Point32_t point, int16_t dy, int16_t dx, bool using_point_pos);

	static void robot_to_point(Point32_t point, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);

	CellState get_cell(int id, int16_t x, int16_t y);

	void clear_blocks(void);

	static int32_t cell_to_count(int16_t distance);

	static int16_t count_to_cell(int32_t count);

	static Point32_t cell_to_point(const Cell_t &cell);

	Cell_t point_to_cell(Point32_t pnt);

	void set_cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state);

	void reset(uint8_t id);

	void copy(uint8_t id, uint8_t **new_map);

/*
 * @author Alvin Xie
 * @brief Convert the ros map to grid map for the pathplanning algorithm.
 * @param is_mark_cleaned to decide if mark the free space to CLENAED
 * @return None
 */
	void ros_convert(int16_t id, bool is_mark_cleaned, bool is_clear_block, bool is_freshen_map,int limit=0);

	void to_world(double origin_x_, double origin_y_, float resolution_, unsigned int mx, unsigned int my, double &wx,
								double &wy);

	bool worldToMap(double origin_x_, double origin_y_, float resolution_, int size_x_, int size_y_, double wx,
									double wy, unsigned int &mx, unsigned int &my);

	unsigned int getIndex(int size_x_, unsigned int mx, unsigned int my);

	void indexToCells(int size_x_, unsigned int index, unsigned int &mx, unsigned int &my);

	bool worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy);

	bool mark_robot(uint8_t id);

	Cell_t update_position();

	uint8_t set_laser();

	uint8_t set_obs();

	uint8_t set_bumper();

	uint8_t set_rcon();

	uint8_t set_cliff();

	uint8_t set_tilt();

	uint8_t set_slip();

	uint8_t set_charge_position(const Cell_t homepoint);

	uint8_t set_follow_wall();


	static uint8_t save_laser();

	static uint8_t save_obs();

	static uint8_t save_bumper();

	static uint8_t save_rcon();

	static uint8_t save_cliff();

	static uint8_t save_tilt();

	static uint8_t save_slip();

	static uint8_t save_follow_wall();

	static uint8_t save_blocks();

	uint8_t set_blocks();

	static void robot_to_cell(Point32_t point, int16_t offset_lat, int16_t offset_long, int16_t &x, int16_t &y);


	void set_cleaned(std::deque<Cell_t> &cells);

	uint32_t get_cleaned_area();

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
	uint8_t is_block_accessible(int16_t x, int16_t y);
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
	bool is_block_cleanable(int16_t x, int16_t y);

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

	bool is_front_block_boundary(int dx);

	void generate_SPMAP(const Cell_t& curr, std::deque<PPTargetType>& g_paths);
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

	bool is_block(void);
	BoundingBox2 generateBound()
	{
		BoundingBox2 bound{{int16_t(g_x_min), int16_t(g_y_min)}, {g_x_max, g_y_max}};
		return bound;
	}

	BoundingBox2 generateBound2()
	{
		BoundingBox2 bound{{int16_t(g_x_min - 1), int16_t(g_y_min - 1)}, {g_x_max, g_y_max}};
		return bound;
	}
	void path_get_range(uint8_t id, int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);

	void color_print(char *outString, int16_t y_min, int16_t y_max);
	void print(uint8_t id, int16_t endx, int16_t endy);
private:
	uint8_t costmap[MAP_SIZE][(MAP_SIZE + 1) / 2];
	uint8_t spmap[MAP_SIZE][(MAP_SIZE + 1) / 2];

	int16_t g_x_min, g_x_max, g_y_min, g_y_max;
	int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;
	static double xCount, yCount;

};

uint8_t map_set_blocks();
extern CostMap cost_map;
/*wf_map is to record the wall follow path to caculate the isolate islands*/
extern CostMap fw_map;
extern CostMap ros_map;
extern CostMap ros2_map;

double world_distance(void);
#endif /* __MAP_H */
