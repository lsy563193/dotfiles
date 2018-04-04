#ifndef __MAP_H__
#define __MAP_H__

#include "config.h"
#include <deque>
#include <vector>
#include "mathematics.h"
#include "BoundingBox.h"
#include <boost/thread/mutex.hpp>
#include "slam_map.hpp"

#define CLEAN_MAP 0
#define COST_MAP 1

#define BLOCK_LEFT				((uint8_t) 0x01)
#define BLOCK_RIGHT			((uint8_t) 0x02)
#define BLOCK_FRONT			((uint8_t) 0x04)
#define BLOCK_ALL			((uint8_t) 0x07)
#define BLOCK_LIDAR_BUMPER			((uint8_t) 0x08)

typedef std::deque<Point_t> Points;

class GridMap {
public:

	GridMap();
	~GridMap();
	void mapInit();

	void setCell(uint8_t id, int16_t x, int16_t y, CellState value);

	CellState getCell(int id, int16_t x, int16_t y);

	void clearBlocks(void);

	void setCells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state);

	void reset(uint8_t id);

	void copy(GridMap &source_map);

/*
 * @author Alvin Xie/ Li Shao Yan
 * @brief Convert the ros map to grid map for the path algorithm.
 * @return None
 */
	void convertFromSlamMap(float threshold);

	void merge(GridMap source_map, bool add_slam_map_blocks_to_uncleaned = false,
			   bool add_slam_map_blocks_to_cleaned = false,
			   bool add_slam_map_cleanable_area = false, bool clear_map_blocks = false,
			   bool clear_slam_map_blocks = false,
			   bool clear_bumper_and_lidar_blocks = false);

	void slamMapToWorld(double origin_x_, double origin_y_, float resolution_, int16_t slam_map_x,
						int16_t slam_map_y, double &world_x, double &world_y);

	bool worldToSlamMap(double origin_x_, double origin_y_, float resolution_, uint32_t slam_map_width,
								 uint32_t slam_map_height, double world_x, double world_y, uint32_t &data_map_x, uint32_t &data_map_y);

	uint32_t getIndexOfSlamMapData(uint32_t slam_map_width, uint32_t data_map_x, uint32_t data_map_y);

	void indexToCells(int size_x_, unsigned int index, unsigned int &mx, unsigned int &my);

//	bool worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy);

	void cellToWorld(double &worldX, double &worldY, int16_t &cellX, int16_t &cellY);

	bool markRobot(uint8_t id);

	bool trapMarkRobot(uint8_t id);


	uint8_t saveSlip();

	void	setBlockWithBound(Cell_t min, Cell_t max, CellState state, bool with_block);

	/*
	 * Mark a circle of radius from point with cell_state.
	 *
	 * @param point, center of the circle, it is POINT but not CELL.
	 * @param cover_block, whether it should cover c_blocks to @param cell_state.
	 * @param radius, the radius of the circle.
	 * @param cell_state, target cell state of marking.
	 */
	void setCircleMarkers(Point_t point, int radius, CellState cell_state,Marks& error_marker);

	/*
	 * Mark a square of x_len * y_len from center with cell_state.
	 *
	 * @param center, center cell of the area.
	 * @param cell_state, target cell state of marking.
	 * @param x_len, the x length of the area should be (2 * x_len + 1).
	 * @param y_len, the y length of the area should be (2 * y_len + 1).
	 */
	void setArea(Cell_t center, CellState cell_state, uint16_t x_len = 0, uint16_t y_len = 0);

	uint32_t getCleanedArea();

/*
 * Check a block is accessible by the robot or not.
 * A block is defined as have the same size of robot.
 *
 * @param x	x coordinate of the block
 * @param y	y coordinate of the block
 *
 * @return	0 if the block is not blocked by bumper, obs or cliff
 *		1 if the block is blocked
 */
	bool isBlockAccessible(int16_t x, int16_t y);

	uint8_t isBlocksAtY(int16_t x, int16_t y);

	uint8_t isBlockBlockedXAxis(int16_t x, int16_t y,bool is_left);

/*
 * Check a block is on the boundary or not, a block is defined as have the same size of robot.
 *
 * @param x	x coordinate of the block
 * @param y	y coordinate of the block
 *
 * @return	0 if the block is not on the boundary
 *		1 if the block is on the boundary
 */
	uint8_t isBlockBoundary(int16_t x, int16_t y);

/*
 * Check a block is uncleaned or not, a block is defined as have the same size of brush.
 * Since the brush occupies 3 cells, if there is any one of those 3 cells unclean, then the
 * block is treated as unclean.
 *
 * @param x	x coordinate of the block
 * @param y	y coordinate of the block
 *
 * @return	0 if the block is cleaned
 *		1 if the block is uncleaned
 */
	uint8_t isUncleanAtY(int16_t x, int16_t y);

	uint8_t isBlockAtY(int, int16_t x, int16_t y);
/*
 * Check a block is cleaned or not, a block is defined as have the same size of brush.
 *
 *
 * @param x	x coordinate of the block
 * @param y	y coordinate of the block
 *
 * @return	0 if the block is not cleaned
 *		1 if the block is cleaned
 */
	int8_t isNotBlockAndCleaned(int16_t x, int16_t y);

/*
 * Check a block is cleanable or not, a block is defined as have the same size of brush.
 *
 *
 * @param x	x coordinate of the block
 * @param y	y coordinate of the block
 *
 * @return	0 if the block is not cleanable
 *		1 if the block is cleanable
 */
	bool isBlockCleanable(int16_t x, int16_t y);

/*
 * Check a given point is blocked by bumper and/or cliff or not.
 *
 * @param x	x coordinate of the given point
 * @param y	y coordinate of the given point
 *
 * @return	0 if it is not blocked by bumper and/or cliff
 *		1 if it is blocked by bumper and/or cliff
 */
	uint8_t isBlockedByBumper(int16_t x, int16_t y);

/*
 * Check whether a given point is an blocked or not.
 *
 * @param x	x coordinate of the give point.
 * @param y	y coordinate of the give point.
 *
 * @return	0 if the given point is not blocked
 * 		1 if the given point is blocked
 */
	uint8_t isABlock(int16_t x, int16_t y);

	bool isFrontBlockBoundary(int dx);

	int8_t isNeedClean(int16_t x, int16_t y);
	bool find_if(const Cell_t &curr_cell, Cells &targets, std::function<bool(const Cell_t &next)> compare, bool is_count=false, bool is_stop=false);
	int count_if(const Cell_t &curr_cell, std::function<bool(const Cell_t &next)> compare);
//	void generateSPMAP(const Cell_t &curr, Cells &target_list);
//	void generateSPMAP(const Cell_t &curr);
/*
 * Function to find the x/y range of the Map or wfMap, if the range is to small,
 * use the offset of those value to 3.
 *
 * @param *x_range_min	Pointer for minimum x value of the Map
 * @param *x_range_max	Pointer for maximum x value of the Map
 * @param *y_range_min	Pointer for minimum y value of the Map
 * @param *y_range_max	Pointer for maximum y value of the Map
 *
 * @return
 */

	bool isFrontBlocked(Dir_t dir);
	bool isFrontSlamBlocked(void);

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
	void getMapRange(uint8_t id, int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);

	bool isOutOfMap(const Cell_t &cell);
	bool isOutOfTargetRange(const Cell_t &cell);
	bool cellIsOutOfRange(Cell_t cell);

	void colorPrint(const char *outString, int16_t y_min, int16_t y_max);
	void print(uint8_t id, const Cells& targets);

private:
	uint8_t clean_map[MAP_SIZE][(MAP_SIZE + 1) / 2];
	uint8_t cost_map[MAP_SIZE][(MAP_SIZE + 1) / 2];

	int16_t g_x_min, g_x_max, g_y_min, g_y_max;
	int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;

	// Cells that temporary save the c_blocks.

};

/*wf_map is to record the wall follow path to caculate the isolate islands*/
extern GridMap slam_grid_map;

#endif /* __MAP_H */
