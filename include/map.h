#ifndef __MAP_H__
#define __MAP_H__

#include "config.h"
#include <deque>
#include <vector>
#include "mathematics.h"
#include "BoundingBox.h"
#include <boost/thread.hpp>

#define MAP 0
#define SPMAP 1

#define BLOCK_LEFT				((uint8_t) 0x01)
#define BLOCK_RIGHT			((uint8_t) 0x02)
#define BLOCK_FRONT			((uint8_t) 0x04)
#define BLOCK_ALL			((uint8_t) 0x07)

typedef std::deque<Cell_t> PPTargetType;

typedef enum {
  UNCLEAN  = 0,
  CLEANED = 1,
  BLOCKED = 2,
  BLOCKED_OBS = 2,
  BLOCKED_BUMPER = 3,
  BLOCKED_CLIFF = 4,
  BLOCKED_RCON = 5,
  BLOCKED_LIDAR = 6,
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

class SlamMap
{
public:
	SlamMap();
	~SlamMap();

	void setWidth(uint32_t width);
	uint32_t getWidth();
	void setHeight(uint32_t height);
	uint32_t getHeight();
	void setResolution(float resolution);
	float getResolution();
	void setOriginX(double origin_x);
	double getOriginX();
	void setOriginY(double origin_y);
	double getOriginY();
	void setData(std::vector<int8_t> data);
	std::vector<int8_t> getData();

private:
	uint32_t width_;
	uint32_t height_;
	float resolution_;
	double origin_x_;
	double origin_y_;
	std::vector<int8_t> map_data_;
};

extern SlamMap slam_map;
extern boost::mutex slam_map_mutex;


class CostMap {
public:

	CostMap();

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
 * @author Alvin Xie
 * @brief Convert the ros map to grid map for the pathplanning algorithm.
 * @param is_mark_cleaned to decide if mark the free space to CLENAED
 * @return None
 */
	void ros_convert(int16_t id, bool is_mark_cleaned, bool is_clear_block, bool is_freshen_map,int limit=20, SlamMap* slam_map_=&slam_map);

	void mapToWorld(double origin_x_, double origin_y_, float resolution_, unsigned int mx, unsigned int my, double &wx,
					double &wy);

	bool worldToMap(double origin_x_, double origin_y_, float resolution_, int size_x_, int size_y_, double wx,
									double wy, unsigned int &mx, unsigned int &my);

	unsigned int getIndex(int size_x_, unsigned int mx, unsigned int my);

	void indexToCells(int size_x_, unsigned int index, unsigned int &mx, unsigned int &my);

	bool worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy);

	bool countToWorld(double &wx, double &wy, int32_t &cx, int32_t &cy);

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

extern CostMap cost_map;
/*wf_map is to record the wall follow path to caculate the isolate islands*/
extern CostMap fw_map;
extern CostMap ros_map;
extern CostMap decrease_map;

extern boost::mutex slam_map_mutex;

double world_distance(void);
#endif /* __MAP_H */
