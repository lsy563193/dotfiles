#ifndef __MAP_H__
#define __MAP_H__

#include "config.h"
#include <deque>
#include <vector>
#include "mathematics.h"
#include "BoundingBox.h"
#include <boost/thread/mutex.hpp>
#include "slam_map.hpp"

#define BOTH_MAP 2

#define BLOCK_LEFT				((uint8_t) 0x01)
#define BLOCK_RIGHT			((uint8_t) 0x02)
#define BLOCK_FRONT			((uint8_t) 0x04)
#define BLOCK_ALL			((uint8_t) 0x07)
#define BLOCK_LIDAR_BUMPER			((uint8_t) 0x08)

class GridMap {
public:

	GridMap();
	~GridMap() = default;

	/*
	 * This function is for resetting specific map value.
	 *
	 * @param id: Map id
	 */
	void reset();

	/*
	 * This function is for getting specific cell's state.
	 *
	 * @param id: Map id
	 * @param x: x coordinate for this cell.
	 * @param y: y coordinate for this cell.
	 * @param value: CellState for this cell.
	 */
	void setCost(int16_t x, int16_t y, CellState value);

	/*
	 * GridMap::getCost description
	 *
	 * This function is for setting specific cell's state.
	 *
	 * @param id: Map id
	 * @param x: x coordinate for this cell.
	 * @param y: y coordinate for this cell.
	 * @return: CellState for this cell.
	 */
	CellState getCost(int16_t x, int16_t y);

	/*
	 * This function is for setting cells around this specific cell with the parameter CellState.
	 *
	 * @param id: Map id
	 * @param x: x coordinate for this cell.
	 * @param y: y coordinate for this cell.
	 * @param state: CellState for this cell.
	 */
	void setCells(int16_t cell_x, int16_t cell_y, CellState state, int8_t offset);

	/*
	 * This function is for setting cells around this specific cell with the parameter CellState, but if the
	 * origin cell state equals exception_state, it will not be overrode.
	 *
	 * @param id: Map id
	 * @param x: x coordinate for this cell.
	 * @param y: y coordinate for this cell.
	 * @param state: CellState for this cell.
	 * @param exception_state: Exception cell state for all these cells.
	 */
	void setSpecificCells(int16_t cell_x, int16_t cell_y, CellState state, CellState exception_state,
						  int8_t offset);

	/*
	 * This function is for copying the clean_map_ data of the source map to this GridMap instance.
	 *
	 * @param source_map: The source map instance.
	 */
	void copy(GridMap &source_map);

	/*
	 * This function is for converting the ros map into grid map.
	 *
	 * @author Alvin Xie/ Li Shao Yan
	 * @param target_resolution: The resolution of the output data.
	 * @param threshold: The percentage for judging the cell state.
	 * @param bound: The data boundary for this conversion.
	 */
	void convertFromSlamMap(float target_resolution, float threshold, const BoundingBox2& bound);

	/*
	 * This function is for merging the source map into this GridMap instance.
	 *
	 * @author Austin Liu
	 * @param source_map: Source GridMap instance.
	 * @param add_slam_map_blocks_to_uncleaned: If this parameter is true, it will merge the cells with
	 * SLAM_MAP_BLOCKED state in source map to cells with UNCLEANED state in this instance.
	 * @param add_slam_map_blocks_to_cleaned: If this parameter is true, it will merge the cells with
	 * SLAM_MAP_BLOCKED state in source map to cells with CLEANED state in this instance.
	 * @param add_slam_map_blocks_to_cleaned: If this parameter is true, it will merge the cells with
	 * SLAM_MAP_REACHABLE state in source map to this instance.
	 * @param clear_map_blocks: If this parameter is true, it will merge the cells with SLAM_MAP_REACHABLE state
	 * in source map to cells with all kinds of blocks(except cliff blocks) state in this instance.
	 * @param clear_slam_map_blocks: If this parameter is true, it will merge the cells with SLAM_MAP_REACHABLE state
	 * in source map to cells with SLAM_MAP_BLOCKED state in this instance.
	 * @param clear_slam_map_blocks: If this parameter is true, it will merge the cells with SLAM_MAP_REACHABLE state
	 * in source map to cells with BLOCKED_BUMPER or BLOCKED_LIDAR or BLOCKED_SLIP state in this instance.
	 */
	void merge(GridMap source_map, bool add_slam_map_blocks_to_uncleaned = false,
			   bool add_slam_map_blocks_to_cleaned = false,
			   bool add_slam_map_cleanable_area = false, bool clear_map_blocks = false,
			   bool clear_slam_map_blocks = false,
			   bool clear_bumper_and_lidar_and_slip_blocks = false);

	/*
	 * This function is for calculating the real world coordinate from slam map coordinate.
	 *
	 * @author Alvin Xie
	 * @param origin_x: Origin x coordinate of slam map.
	 * @param origin_y: Origin y coordinate of slam map.
	 * @param resolution: Resolution of slam map.
	 * @param slam_map_x: x coordinate of slam map.
	 * @param slam_map_y: y coordinate of slam map.
	 * @param world_x: x coordinate of real world.
	 * @param world_y: y coordinate of real world.
	 */
	void slamMapToWorld(double origin_x, double origin_y, float resolution, int16_t slam_map_x,
						int16_t slam_map_y, double &world_x, double &world_y);

	bool worldToSlamMap(double origin_x_, double origin_y_, float resolution_, uint32_t slam_map_width,
								 uint32_t slam_map_height, double world_x, double world_y, uint32_t &data_map_x, uint32_t &data_map_y);

	uint32_t getIndexOfSlamMapData(uint32_t slam_map_width, uint32_t data_map_x, uint32_t data_map_y);

	void indexToCells(int size_x_, unsigned int index, unsigned int &mx, unsigned int &my);

	void cellToWorld(double &worldX, double &worldY, int16_t &cellX, int16_t &cellY);

	bool markRobot(const Cell_t& curr, bool is_clean_rcon=true);

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
	void setCircleMarkers(const Point_t& point, int radius, CellState cell_state,Marks& error_marker);

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
	bool isNotBlockAndCleaned(int16_t x, int16_t y);

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
//	bool dijstra(const Cell_t &curr_cell, Cells &targets, std::function<bool(const Cell_t &next)> compare_for_targets,
//				 bool stop_if_found_one, bool use_uncleaned_area = false);
	// Put the cells that fit the compare function into parameter targets.
//	bool find_if(const Cell_t &curr_cell, Cells &targets, std::function<bool(const Cell_t &next)> compare);
	// Return the count of cells that all 9 cells around it match the compare function in this map.
//	bool count_if(const Cell_t &curr_cell, std::function<bool(const Cell_t &next)> compare, int& count);

//	bool dijkstra(const Cell_t &curr_cell, Cells &targets, bool greedy_match,
//				  std::function<bool(const Cell_t &tmp_target)> targets_selection,
//				  std::function<bool(const Cell_t &tmp_target, const Cell_t &neighbour)> expand_condition);

	// Basic expend condiction for most dijkstra usage.
//	bool isAccessibleNeighbor(Cell_t neighbor_cell);

	// Basic expend condiction for finding path.
//	bool isAccessibleCleanedNeighbor(Cell_t neighbor_cell);

	// Using dijkstra to count reachable cleaned area.
//	uint16_t dijkstraCountCleanedArea(Point_t curr, Cells &targets);
//	void generateSPMAP(const Cell_t &curr, Cells &target_list);
//	void generateSPMAP(const Cell_t &curr);


	bool isFrontBlocked(Dir_t dir);
	bool isFrontSlamBlocked(void);

	BoundingBox2 generateBound();
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
	void getMapRange(int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max);

	bool cellIsOutOfTargetRange(Cell_t cell);
	BoundingBox2 genTargetRange();
	bool pointIsPointingOutOfTargetRange(Point_t point);
	void cellPreventOutOfRange(Cell_t &cell);

	void colorPrint(const char *outString, int16_t y_min, int16_t y_max);
	void print(const Cell_t& curr, const Cells& targets);
    void printInRange(const Cell_t& curr_cell, const Cells& targets,bool is_bound,BoundingBox2 bound);

	/*
	 * Generate the direct path between two cells.
	 *
	 * @author: Austin Liu
	 * @param cell_1: The first cell.
	 * @param cell_2: The second cell.
	 * @return Cells: Cells between these two cells.
	 */
	Cells generateCellsBetweenTwoCells(const Cell_t &start, const Cell_t &end, bool &is_x_direction);

	// Loading the log map for debug.
	void loadMap(int16_t x_min, int16_t x_max, int16_t y_min, int16_t y_max);
	void loadMap(bool use_map,Cell_t& curr, Dir_t& dir,bool& trend_pos, const std::string& str);
private:
	uint8_t clean_map[MAP_SIZE][MAP_SIZE];

	int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;
};

extern GridMap slam_grid_map;

#endif /* __MAP_H */
