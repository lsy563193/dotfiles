#ifndef __MAP_H__
#define __MAP_H__

#include "config.h"
#include <deque>
#include <vector>
#include "mathematics.h"
#include "BoundingBox.h"
#include <boost/thread/mutex.hpp>
#include "slam_map.hpp"

#define MAP 0
#define SPMAP 1

#define BLOCK_LEFT				((uint8_t) 0x01)
#define BLOCK_RIGHT			((uint8_t) 0x02)
#define BLOCK_FRONT			((uint8_t) 0x04)
#define BLOCK_ALL			((uint8_t) 0x07)

typedef std::deque<Cell_t> PPTargetType;

typedef enum {
	// The sequence of MAP value must be UNCLEAN < CLEANED < MAP_BLOCKED < SLAM_MAP_BLOCKED
  UNCLEAN  = 0,
  SLAM_MAP_UNKNOWN = 0,
  CLEANED = 1,
  SLAM_MAP_CLEANABLE = 1,
  BLOCKED = 2,
  BLOCKED_OBS = 2,
  BLOCKED_BUMPER = 3,
  BLOCKED_CLIFF = 4,
  BLOCKED_RCON = 5,
  BLOCKED_LIDAR = 6,
  BLOCKED_TILT = 7,
  BLOCKED_SLIP = 8,
  SLAM_MAP_BLOCKED = 9,
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
  MAP_POS_X = 0,
  MAP_PX_PY = 450,
  MAP_POS_Y = 900,
  MAP_NS_PY = 1350,
  MAP_NEG_X = 1800,
  MAP_NX_NY =-1350,
  MAP_NEG_Y =-900,
  MAP_PX_NY =-450,
  MAP_NONE = 0,
} MapDirection;

class GridMap {
public:

	GridMap();

	static bool isPositiveDirection(MapDirection dir);

	static bool isXDirection(MapDirection dir);

	static bool isYDirection(MapDirection dir);

	int32_t getXCount(void);

	int32_t getYCount(void);

	Point32_t getCurrPoint(void);

	int16_t getXCell(void);

	int16_t getYCell(void);

	Cell_t getCurrCell();

	void setCell(uint8_t id, int32_t x, int32_t y, CellState value);

	CellState getCell(int id, int16_t x, int16_t y);

	void setPosition(double x, double y);

	Point32_t getRelative(Point32_t point, int16_t dy, int16_t dx, bool using_point_pos);

	void robotToPoint(Point32_t point, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);

	void robotToCell(Point32_t point, int16_t offset_lat, int16_t offset_long, int16_t &x, int16_t &y);

	void clearBlocks(void);

	int32_t cellToCount(int16_t distance);

	int16_t countToCell(int32_t count);

	Point32_t cellToPoint(const Cell_t &cell);

	Cell_t pointToCell(Point32_t pnt);

	void setCells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state);

	void reset(uint8_t id);

	void copy(uint8_t id, uint8_t **new_map);

/*
 * @author Alvin Xie/ Li Shao Yan
 * @brief Convert the ros map to grid map for the path algorithm.
 * @return None
 */
	void convertFromSlamMap(float threshold);

	void merge(GridMap source_map, bool add_slam_map_blocks_to_uncleaned = false, bool add_slam_map_blocks_to_cleaned = false,
						bool add_slam_map_cleanable_area = false, bool clear_map_blocks = false, bool clear_slam_map_blocks = false);

	void slamMapToWorld(double origin_x_, double origin_y_, float resolution_, int16_t slam_map_x,
						int16_t slam_map_y, double &world_x, double &world_y);

	bool worldToSlamMap(double origin_x_, double origin_y_, float resolution_, uint32_t slam_map_width,
								 uint32_t slam_map_height, double world_x, double world_y, uint32_t &data_map_x, uint32_t &data_map_y);

	uint32_t getIndexOfSlamMapData(uint32_t slam_map_width, uint32_t data_map_x, uint32_t data_map_y);

	void indexToCells(int size_x_, unsigned int index, unsigned int &mx, unsigned int &my);

	bool worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy);

	void cellToWorld(double &worldX, double &worldY, int16_t &cellX, int16_t &cellY);

	bool markRobot(uint8_t id);

	Cell_t updatePosition();

	uint8_t setLidar();

	uint8_t setObs();

	uint8_t setBumper();

	uint8_t setRcon();

	uint8_t setCliff();

	uint8_t setTilt();

	uint8_t setSlip();

	uint8_t saveChargerArea(const Cell_t homepoint);

	uint8_t setFollowWall();


	uint8_t saveLidar();

	uint8_t saveObs();

	uint8_t saveBumper();

	uint8_t saveRcon();

	uint8_t saveCliff();

	uint8_t saveTilt();

	uint8_t saveSlip();

	uint8_t saveFollowWall();

	uint8_t saveBlocks();

	uint8_t setBlocks();


	void setCleaned(std::deque<Cell_t> &cells);

	uint32_t getCleanedArea();

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
	bool isBlockAccessible(int16_t x, int16_t y);

	uint8_t isBlocksAtY(int16_t x, int16_t y);

	uint8_t isBlockBlockedXAxis(int16_t x, int16_t y);

/*
 * Check a block is on the boundary or not, a block is defined as have the same size of robot.
 *
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
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
 * @param x	X coordinate of the block
 * @param y	Y coordinate of the block
 *
 * @return	0 if the block is cleaned
 *		1 if the block is uncleaned
 */
	uint8_t isUncleanAtY(int16_t x, int16_t y);

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
	int8_t isBlockCleaned(int16_t x, int16_t y);

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
	bool isBlockCleanable(int16_t x, int16_t y);

/*
 * Check a given point is blocked by bumper and/or cliff or not.
 *
 * @param x	X coordinate of the given point
 * @param y	Y coordinate of the given point
 *
 * @return	0 if it is not blocked by bumper and/or cliff
 *		1 if it is blocked by bumper and/or cliff
 */
	uint8_t isBlockedByBumper(int16_t x, int16_t y);

/*
 * Check whether a given point is an blocked or not.
 *
 * @param x	X coordinate of the give point.
 * @param y	Y coordinate of the give point.
 *
 * @return	0 if the given point is not blocked
 * 		1 if the given point is blocked
 */
	uint8_t isABlock(int16_t x, int16_t y);

	bool isFrontBlockBoundary(int dx);

	void generateSPMAP(const Cell_t &curr, PPTargetType &target_list);
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

	bool isFrontBlocked(void);

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

	bool cellIsOutOfRange(Cell_t cell);

	void colorPrint(char *outString, int16_t y_min, int16_t y_max);
	void print(uint8_t id, int16_t endx, int16_t endy);

private:
	uint8_t costmap[MAP_SIZE][(MAP_SIZE + 1) / 2];
	uint8_t spmap[MAP_SIZE][(MAP_SIZE + 1) / 2];

	int16_t g_x_min, g_x_max, g_y_min, g_y_max;
	int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;

	static double xCount, yCount;

};

extern GridMap cost_map;
/*wf_map is to record the wall follow path to caculate the isolate islands*/
extern GridMap fw_map;
extern GridMap exploration_map;
extern GridMap slam_cost_map;
extern GridMap decrease_map;

#endif /* __MAP_H */
