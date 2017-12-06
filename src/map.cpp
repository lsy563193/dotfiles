#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <motion_manage.h>
#include <robot.hpp>
#include <core_move.h>
#include <gyro.h>
#include <movement.h>
#include <event_manager.h>
#include <move_type_new.h>
#include <regulator.h>
#include <mathematics.h>
#include <space_exploration.h>
#include <clean_state.h>
#include "rcon.h"
#include "map.h"
#include "mathematics.h"
#include "robot.hpp"
#include "clean_mode.h"

#define DEBUG_MSG_SIZE	1 // 20
GridMap nav_map;
GridMap fw_map;
GridMap exploration_map;
GridMap slam_cost_map;
GridMap decrease_map;

boost::mutex slam_map_mutex;

#ifndef SHORTEST_PATH_V2
#endif

//int16_t homeX, homeY;

// Cells that temporary save the blocks.
std::vector<Cell_t> temp_bumper_cells;
std::vector<Cell_t> temp_obs_cells;
std::vector<Cell_t> temp_rcon_cells;
std::vector<Cell_t> temp_tilt_cells;
std::vector<Cell_t> temp_slip_cells;
std::vector<Cell_t> temp_cliff_cells;
std::vector<Cell_t> temp_fw_cells;
std::vector<Cell_t> temp_WFMAP_follow_wall_cells;

uint16_t relative_theta = 3600;

Cell_t g_stub_cell(0,0);

double GridMap::xCount = 0;
double GridMap::yCount = 0;

GridMap::GridMap() {
		for(auto c = 0; c < MAP_SIZE; ++c) {
			for(auto d = 0; d < (MAP_SIZE + 1) / 2; ++d) {
				clean_map[c][d] = 0;
			}
		}
		g_x_min = g_x_max = g_y_min = g_y_max = 0;
		xRangeMin = g_x_min - (MAP_SIZE - (g_x_max - g_x_min + 1));
		xRangeMax = g_x_max + (MAP_SIZE - (g_x_max - g_x_min + 1));
		yRangeMin = g_y_min - (MAP_SIZE - (g_y_max - g_y_min + 1));
		yRangeMax = g_y_max + (MAP_SIZE - (g_y_max - g_y_min + 1));

//		xCount = 0;
//		yCount = 0;
}

bool GridMap::isPositiveDirection(MapDirection dir)
{
	if (dir == MAP_POS_X || dir == MAP_POS_Y || dir == MAP_NONE)
		return true;

	return false;
}

bool GridMap::isXDirection(MapDirection dir)
{
	if (dir == MAP_POS_X || dir == MAP_NEG_X || dir == MAP_NONE)
		return true;

	return false;
}

bool GridMap::isYDirection(MapDirection dir)
{
	if (dir == MAP_POS_Y || dir == MAP_NEG_Y || dir == MAP_NONE)
		return true;

	return false;
}

int32_t GridMap::getXCount(void) {
	return (int32_t)round(xCount);
}

int32_t GridMap::getYCount(void) {
	return (int32_t)round(yCount);
}

Point32_t GridMap::getCurrPoint(void)
{
	return {getXCount(), getYCount(),robot::instance()->getPoseAngle()};
}

int16_t GridMap::getXCell(void) {
	return countToCell(xCount);
}

int16_t GridMap::getYCell(void) {
	return countToCell(yCount);
}

Cell_t GridMap::getCurrCell()
{
	return Cell_t{getXCell(), GridMap::getYCell(),robot::instance()->getPoseAngle()};
}

void GridMap::setPosition(double x, double y) {
	xCount = x;
	yCount = y;
}

/*
 * GridMap::get_cell description
 * @param id	Map id
 * @param x	Cell x
 * @param y	Cell y
 * @return	CellState
 */
CellState GridMap::getCell(int id, int16_t x, int16_t y) {
	CellState val;
	int16_t x_min, x_max, y_min, y_max;
	if (id == CLEAN_MAP || id == COST_MAP) {
		x_min = xRangeMin;
		x_max = xRangeMax;
	   	y_min = yRangeMin;
	   	y_max = yRangeMax;
	} 
	if(x >= x_min && x <= x_max && y >= y_min && y <= y_max) {
		x += MAP_SIZE + MAP_SIZE / 2;
		x %= MAP_SIZE;
		y += MAP_SIZE + MAP_SIZE / 2;
		y %= MAP_SIZE;

#ifndef SHORTEST_PATH_V2
		//val = (CellState)((id == CLEAN_MAP) ? (clean_map[x][y / 2]) : (cost_map[x][y / 2]));
		if(id == CLEAN_MAP)
			val = (CellState)(clean_map[x][y / 2]);
		else if(id == COST_MAP)
			val = (CellState)(cost_map[x][y / 2]);
#else
		//val = (CellState)(clean_map[x][y / 2]);
		if (id == CLEAN_MAP) {
			val = (CellState)(clean_map[x][y / 2]);
		}
#endif

		/* Upper 4 bits & lower 4 bits. */
		val = (CellState) ((y % 2) == 0 ? (val >> 4) : (val & 0x0F));

	} else {
		if(id == CLEAN_MAP) {
			val = BLOCKED_BOUNDARY;
		} else {
			val = COST_HIGH;
		}
	}

	return val;
}

/*
 * GridMap::set_cell description
 * @param id		Map id
 * @param x		 For CLEAN_MAP it is count x, for COST_MAP it is cell x.
 * @param y		 For CLEAN_MAP it is count y, for COST_MAP it is cell y.
 * @param value CellState
 */
void GridMap::setCell(uint8_t id, int32_t x, int32_t y, CellState value) {
	CellState val;
	int16_t ROW, COLUMN;

	if(id == CLEAN_MAP) {
		if(value == CLEANED) {
			ROW = cellToCount(countToCell(x)) - x;
			COLUMN = cellToCount(countToCell(y)) - y;

			if(abs(ROW) > (CELL_SIZE - 2 * 20) * CELL_COUNT_MUL / (2 * CELL_SIZE) || abs(COLUMN) > (CELL_SIZE - 2 * 20) * CELL_COUNT_MUL / (2 * CELL_SIZE)) {
				//return;
			}
		}

		x = countToCell(x);
		y = countToCell(y);
	}

	if(id == CLEAN_MAP) {
		if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
			if(x < g_x_min) {
				g_x_min = x;
				xRangeMin = g_x_min - (MAP_SIZE - (g_x_max - g_x_min + 1));
				xRangeMax = g_x_max + (MAP_SIZE - (g_x_max - g_x_min + 1));
			} else if(x > g_x_max) {
				g_x_max = x;
				xRangeMin = g_x_min - (MAP_SIZE - (g_x_max - g_x_min + 1));
				xRangeMax = g_x_max + (MAP_SIZE - (g_x_max - g_x_min + 1));
			}
			if(y < g_y_min) {
				g_y_min = y;
				yRangeMin = g_y_min - (MAP_SIZE - (g_y_max - g_y_min + 1));
				yRangeMax = g_y_max + (MAP_SIZE - (g_y_max - g_y_min + 1));
			} else if(y > g_y_max) {
				g_y_max = y;
				yRangeMin = g_y_min - (MAP_SIZE - (g_y_max - g_y_min + 1));
				yRangeMax = g_y_max + (MAP_SIZE - (g_y_max - g_y_min + 1));
			}

			ROW = x + MAP_SIZE + MAP_SIZE / 2;
			ROW %= MAP_SIZE;
			COLUMN = y + MAP_SIZE + MAP_SIZE / 2;
			COLUMN %= MAP_SIZE;

			val = (CellState) clean_map[ROW][COLUMN / 2];
			if (((COLUMN % 2) == 0 ? (val >> 4) : (val & 0x0F)) != value) {
				clean_map[ROW][COLUMN / 2] = ((COLUMN % 2) == 0 ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
			}
		}
	}  else if (id == COST_MAP){
		if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
			x += MAP_SIZE + MAP_SIZE / 2;
			x %= MAP_SIZE;
			y += MAP_SIZE + MAP_SIZE / 2;
			y %= MAP_SIZE;

			val = (CellState) cost_map[x][y / 2];

			/* Upper 4 bits and last 4 bits. */
			cost_map[x][y / 2] = (((y % 2) == 0) ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
		}
	}
}

void GridMap::clearBlocks(void) {
	int16_t c, d;

	for(c = g_x_min; c < g_x_max; ++c) {
		for(d = g_y_min; d < g_y_max; ++d) {
			CellState state = getCell(CLEAN_MAP, c, d);
			if(state == BLOCKED_LIDAR || state == BLOCKED_BUMPER ||	state == BLOCKED_CLIFF || state  == BLOCKED_OBS) {
				if(getCell(CLEAN_MAP, c - 1, d) != UNCLEAN && getCell(CLEAN_MAP, c, d + 1) != UNCLEAN &&
						getCell(CLEAN_MAP, c + 1, d) != UNCLEAN &&
						getCell(CLEAN_MAP, c, d - 1) != UNCLEAN) {
					setCell(CLEAN_MAP, cellToCount(c), cellToCount(d), CLEANED);
				}
			}
		}
	}

	setCell(CLEAN_MAP, cellToCount(getXCell() - 1), cellToCount(getYCell()), CLEANED);
	setCell(CLEAN_MAP, cellToCount(getXCell()), cellToCount(getYCell() + 1), CLEANED);
	setCell(CLEAN_MAP, cellToCount(getXCell() + 1), cellToCount(getYCell()), CLEANED);
	setCell(CLEAN_MAP, cellToCount(getXCell()), cellToCount(getYCell() - 1), CLEANED);
}

Point32_t GridMap::getRelative(Point32_t point, int16_t dy, int16_t dx, bool using_point_pos) {
	double relative_sin, relative_cos;
	if(point.TH != relative_theta) {
		if(point.TH == 0) {
			relative_sin = 0;
			relative_cos = 1;
		} else if(point.TH == 900) {
			relative_sin = 1;
			relative_cos = 0;
		} else if(point.TH == 1800) {
			relative_sin = 0;
			relative_cos = -1;
		} else if(point.TH == -900) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg_to_rad(point.TH, 10));
			relative_cos = cos(deg_to_rad(point.TH, 10));
		}
	}

	if (using_point_pos)
	{
		point.X += (int32_t)( ( ((double)dx * relative_cos * CELL_COUNT_MUL) - ((double)dy	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
		point.Y += (int32_t)( ( ((double)dx * relative_sin * CELL_COUNT_MUL) + ((double)dy	* relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	}
	else
	{
		point.X = cellToCount(countToCell(point.X)) + (int32_t)( ( ((double)dx * relative_cos * CELL_COUNT_MUL) - ((double)dy	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
		point.Y = cellToCount(countToCell(point.Y)) + (int32_t)( ( ((double)dx * relative_sin * CELL_COUNT_MUL) + ((double)dy	* relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	}
	return point;
}

void GridMap::robotToPoint(Point32_t point, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	auto relative_point = getRelative(point, offset_lat, offset_long, true);
	*x = cellToCount(countToCell(relative_point.X));
	*y = cellToCount(countToCell(relative_point.Y));
}

void GridMap::robotToCell(Point32_t point, int16_t offset_lat, int16_t offset_long, int16_t &x, int16_t &y)
{
	auto relative_point = getRelative(point, offset_lat, offset_long, false);
	x = countToCell(relative_point.X);
	y = countToCell(relative_point.Y);
}

int32_t GridMap::cellToCount(int16_t i) {
	return i * CELL_COUNT_MUL;
}

int16_t GridMap::countToCell(int32_t count) {
	if(count < -CELL_COUNT_MUL_1_2) {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL - 1;
	} else {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL;
	}
}

Point32_t GridMap::cellToPoint(const Cell_t &cell) {
	Point32_t pnt;
	pnt.X = cellToCount(cell.X);
	pnt.Y = cellToCount(cell.Y);
	pnt.TH = cell.TH;
	return pnt;
}

Cell_t GridMap::pointToCell(Point32_t pnt) {
	Cell_t cell;
	cell.X = countToCell(pnt.X);
	cell.Y = countToCell(pnt.Y);
	cell.TH = robot::instance()->getPoseAngle();
	return cell;
}

void GridMap::setCells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state)
{
	int8_t i, j;

	for ( i = -(count / 2); i <= count / 2; i++ ) {
		for ( j = -(count / 2); j <= count / 2; j++ ) {
			setCell(CLEAN_MAP, cellToCount(cell_x + i), cellToCount(cell_y + j), state);
		}
	}
}

void GridMap::reset(uint8_t id)
{
#ifndef SHORTEST_PATH_V2
	uint16_t idx;
	if (id == COST_MAP) {
		for (idx = 0; idx < MAP_SIZE; idx++) {
			memset((cost_map[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
		}
	} else if (id == CLEAN_MAP) {
		for (idx = 0; idx < MAP_SIZE; idx++) {
			memset((clean_map[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
		}
	}
#endif
}

void GridMap::convertFromSlamMap(float threshold)
{
	// Clear the cost map itself.
	reset(CLEAN_MAP);

	// Get the data from slam_map.
	std::vector<int8_t> slam_map_data;
	auto width = slam_map.getWidth();
	auto height = slam_map.getHeight();
	auto resolution = slam_map.getResolution();
	auto origin_x = slam_map.getOriginX();
	auto origin_y = slam_map.getOriginY();
	slam_map_data = slam_map.getData();

	// Set resolution multi between cost map and slam map.
	auto multi = CELL_SIZE / (resolution * 1000.0);
	//ROS_INFO("%s %d: resolution: %f, multi: %f.", __FUNCTION__, __LINE__, resolution, multi);
	// Limit count for checking block.*/
	auto limit_count = static_cast<uint16_t>((multi * multi) * threshold);
	//ROS_INFO("%s %d: limit_count: %d.", __FUNCTION__, __LINE__, limit_count);
	// Set boundary for this cost map.
	auto map_x_min = static_cast<int16_t>(origin_x * 1000.0 / CELL_SIZE);
	if (map_x_min < -MAP_SIZE)
		map_x_min = -MAP_SIZE;
	auto map_x_max = map_x_min + static_cast<int16_t>(width / multi);
	if (map_x_max > MAP_SIZE)
		map_x_max = MAP_SIZE;
	auto map_y_min = static_cast<int16_t>(origin_y * 1000.0 / CELL_SIZE);
	if (map_y_min < -MAP_SIZE)
		map_y_min = -MAP_SIZE;
	auto map_y_max = map_y_min + static_cast<int16_t>(height / multi);
	if (map_y_max > MAP_SIZE)
		map_y_max = MAP_SIZE;

	//ROS_INFO("%s,%d: map_x_min: %d, map_x_max: %d, map_y_min: %d, map_y_max: %d",
	// 		   __FUNCTION__, __LINE__, map_x_min, map_x_max, map_y_min, map_y_max);
	for (auto cell_x = map_x_min; cell_x <= map_x_max; ++cell_x)
	{
		for (auto cell_y = map_y_min; cell_y <= map_y_max; ++cell_y)
		{
			// Get the range of this cell in the grid map of slam map data.
			double world_x, world_y;
			cellToWorld(world_x, world_y, cell_x, cell_y);
			uint32_t data_map_x, data_map_y;
			if (worldToSlamMap(origin_x, origin_y, resolution, width, height, world_x, world_y, data_map_x, data_map_y))
			{
				auto data_map_x_min = (data_map_x > multi/2) ? (data_map_x - multi/2) : 0;
				auto data_map_x_max = ((data_map_x + multi/2) < width) ? (data_map_x + multi/2) : width;
				auto data_map_y_min = (data_map_y > multi/2) ? (data_map_y - multi/2) : 0;
				auto data_map_y_max = ((data_map_y + multi/2) < height) ? (data_map_y + multi/2) : height;
				//ROS_INFO("%s %d: data map: x_min(%d), x_max(%d), y_min(%d), y_max(%d)",
				//		 __FUNCTION__, __LINE__, data_map_x_min, data_map_x_max, data_map_y_min, data_map_y_max);

				// Get the slam map data s_index_ of this range.
				std::vector<int32_t> slam_map_data_index;
				for (uint32_t i=data_map_x_min; i<=data_map_x_max; i++)
					for(uint32_t j=data_map_y_min; j<=data_map_y_max; j++)
						slam_map_data_index.push_back(getIndexOfSlamMapData(width, i, j));

				// Values for counting sum of difference slam map data.
				uint32_t block_counter = 0, cleanable_counter = 0, unknown_counter = 0;
				bool block_set = false;
				auto size = slam_map_data_index.size();
				//ROS_INFO("%s %d: data_index_size:%d.", __FUNCTION__, __LINE__, size);
				for(int index = 0; index < size; index++)
				{
					//ROS_INFO("slam_map_data s_index_:%d, data:%d", slam_map_data_index[s_index_], slam_map_data[slam_map_data_index[s_index_]]);
					if(slam_map_data[slam_map_data_index[index]] == 100)		block_counter++;
					else if(slam_map_data[slam_map_data_index[index]] == -1)	unknown_counter++;
					else if(slam_map_data[slam_map_data_index[index]] == 0)		cleanable_counter++;

					if(block_counter > limit_count)/*---occupied cell---*/
					{
						setCell(CLEAN_MAP, cellToCount(cell_x), cellToCount(cell_y), SLAM_MAP_BLOCKED);
						block_set = true;
						break;
					}
				}

				if(!block_set && unknown_counter == size)/*---unknown cell---*/
				{
					setCell(CLEAN_MAP, cellToCount(cell_x), cellToCount(cell_y), SLAM_MAP_UNKNOWN);
					block_set = true;
				}
				if(!block_set)/*---unknown cell---*/
					setCell(CLEAN_MAP, cellToCount(cell_x), cellToCount(cell_y), SLAM_MAP_CLEANABLE);

				//ROS_INFO("unknown counter: %d, block_counter: %d, cleanable_counter: %d", unknown_counter, block_counter, cleanable_counter);
			}
		}
	}
}

void GridMap::merge(GridMap source_map, bool add_slam_map_blocks_to_uncleaned, bool add_slam_map_blocks_to_cleaned,
					bool add_slam_map_cleanable_area, bool clear_map_blocks, bool clear_slam_map_blocks)
{
	int16_t map_x_min, map_y_min, map_x_max, map_y_max;
	source_map.getMapRange(CLEAN_MAP, &map_x_min, &map_x_max, &map_y_min, &map_y_max);
	for (int16_t x = map_x_min; x <= map_x_max; x++)
	{
		for (int16_t y = map_x_min; y <= map_x_max; y++)
		{
			CellState map_cell_state, source_map_cell_state;
			map_cell_state = getCell(CLEAN_MAP, x, y);
			source_map_cell_state = source_map.getCell(CLEAN_MAP, x, y);

			if (clear_map_blocks && map_cell_state >= BLOCKED && map_cell_state < SLAM_MAP_BLOCKED && source_map_cell_state == SLAM_MAP_CLEANABLE)
				setCell(CLEAN_MAP, cellToCount(x), cellToCount(y), CLEANED);
			if (clear_slam_map_blocks && map_cell_state == SLAM_MAP_BLOCKED && source_map_cell_state == SLAM_MAP_CLEANABLE)
				setCell(CLEAN_MAP, cellToCount(x), cellToCount(y), CLEANED);
			if (add_slam_map_blocks_to_uncleaned && map_cell_state == UNCLEAN && source_map_cell_state == SLAM_MAP_BLOCKED)
				setCell(CLEAN_MAP, cellToCount(x), cellToCount(y), SLAM_MAP_BLOCKED);
			if (add_slam_map_blocks_to_cleaned && map_cell_state == CLEANED && source_map_cell_state == SLAM_MAP_BLOCKED)
				setCell(CLEAN_MAP, cellToCount(x), cellToCount(y), SLAM_MAP_BLOCKED);
			if (add_slam_map_cleanable_area && map_cell_state == UNCLEAN && source_map_cell_state == SLAM_MAP_CLEANABLE)
				setCell(CLEAN_MAP, cellToCount(x), cellToCount(y), CLEANED);
		}
	}

}

void GridMap::slamMapToWorld(double origin_x_, double origin_y_, float resolution_, int16_t slam_map_x,
							 int16_t slam_map_y, double &world_x, double &world_y)
{
	world_x = origin_x_ + (slam_map_x + 0.5) * resolution_;
	world_y = origin_y_ + (slam_map_y + 0.5) * resolution_;
//wx = origin_x_ + (mx) * resolution_;
//wy = origin_y_ + (my) * resolution_;
}

bool GridMap::worldToSlamMap(double origin_x_, double origin_y_, float resolution_, uint32_t slam_map_width,
							 uint32_t slam_map_height, double world_x, double world_y, uint32_t &data_map_x, uint32_t &data_map_y)
{
	if (world_x < origin_x_ || world_y < origin_y_
		|| world_x > origin_x_ + slam_map_width * resolution_ || world_y > origin_y_ + slam_map_height * resolution_)
		return false;

	data_map_x = static_cast<uint32_t>((world_x - origin_x_) / resolution_);
	data_map_y = static_cast<uint32_t>((world_y - origin_y_) / resolution_);

	return true;
}

uint32_t GridMap::getIndexOfSlamMapData(uint32_t slam_map_width, uint32_t data_map_x, uint32_t data_map_y)
{
	return data_map_y * slam_map_width + data_map_x;
}

void GridMap::indexToCells(int size_x_, unsigned int index, unsigned int &mx, unsigned int &my)
{
	my = index / size_x_;
	mx = index - (my * size_x_);
}

bool GridMap::worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy)
{
	auto count_x = wx * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto count_y = wy * 1000 * CELL_COUNT_MUL / CELL_SIZE;
//cx = count_to_cell(count_x);
	cx = count_x;
//cy = count_to_cell(count_y);
	cy = count_y;
	return true;
}

void GridMap::cellToWorld(double &worldX, double &worldY, int16_t &cellX, int16_t &cellY)
{
	worldX = (double)cellX * CELL_SIZE / 1000.0;
	worldY = (double)cellY * CELL_SIZE / 1000.0;
}

uint8_t GridMap::setLidar()
{
#if LIDAR_MARKER
	//MotionManage::s_lidar->lidarMarker(true);
#endif
}

uint8_t GridMap::setObs()
{
	if (temp_obs_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_obs_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		setCell(CLEAN_MAP, cellToCount(cell.X), cellToCount(cell.Y), BLOCKED_OBS);
		block_count++;
	}
	temp_obs_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), mark \033[32m%s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return block_count;
}

uint8_t GridMap::setBumper()
{
	if (temp_bumper_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_bumper_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		setCell(CLEAN_MAP, cellToCount(cell.X), cellToCount(cell.Y), BLOCKED_BUMPER);
		block_count++;
	}
	temp_bumper_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), mark \033[32m%s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return block_count;
}

uint8_t GridMap::setTilt()
{
	if (temp_tilt_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_tilt_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		setCell(CLEAN_MAP, cellToCount(cell.X), cellToCount(cell.Y), BLOCKED_TILT);
		block_count++;
	}
	temp_tilt_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark %s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return block_count;
}

uint8_t GridMap::setSlip()
{
	if (temp_slip_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_slip_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		setCell(CLEAN_MAP, cellToCount(cell.X), cellToCount(cell.Y), BLOCKED_SLIP);
		block_count++;
	}
	temp_slip_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark %s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return block_count;
}

uint8_t GridMap::setCliff()
{
	if (temp_cliff_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_cliff_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		setCell(CLEAN_MAP, cellToCount(cell.X), cellToCount(cell.Y), BLOCKED_CLIFF);
		block_count++;
	}
	temp_cliff_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark %s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return block_count;
}

uint8_t GridMap::setRcon()
{
	if (temp_rcon_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_rcon_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		setCell(CLEAN_MAP, cellToCount(cell.X), cellToCount(cell.Y), BLOCKED_RCON);
		block_count++;
	}
	temp_rcon_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark %s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return block_count;
}

uint8_t GridMap::saveChargerArea(const Cell_t home_point)
{
	//before set BLOCKED_RCON, clean BLOCKED_RCON in first.
	temp_rcon_cells.clear();
	int16_t x_min,x_max,y_min,y_max;
	getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);
	for(int16_t i = x_min;i<=x_max;i++){
		for(int16_t j = y_min;j<=y_max;j++){
			if(getCell(CLEAN_MAP, i, j) == BLOCKED_RCON)
				setCell(CLEAN_MAP, cellToCount(i), cellToCount(j), UNCLEAN);
		}
	}
	uint8_t block_count = 0;
	const int WIDTH = 5;//cells
	const int HIGHT = 5;//cells
	const int ltx=-HIGHT/2;//left top x offset
	const int lty=-WIDTH/2;//right top y offset
	int16_t x,y;
	std::string print_msg("");
	for(int i=0; i<HIGHT; i++){ 
		for(int j = 0; j<WIDTH; j++){
			y = lty+j+home_point.Y;
			x = ltx+i+home_point.X;
			temp_rcon_cells.push_back({x,y});
			print_msg+="("+std::to_string(x)+","+std::to_string(y)+"),";
			block_count++;
		}
	}
	ROS_INFO("%s,%d: set charge area:\033[32m%s\033[0m",__FUNCTION__,__LINE__,print_msg.c_str());
	return block_count;
}

uint8_t GridMap::setFollowWall()
{

	uint8_t block_count = 0;
	if (!g_passed_path.empty())
	{
		std::string msg = "cell:";
		Cell_t block_cell;
		auto dy = mt.is_left() ? 2 : -2;
		for(auto& cell : g_passed_path){
			if(getCell(CLEAN_MAP,cell.X,cell.Y) != BLOCKED_RCON){
				robotToCell(cellToPoint(cell), dy * CELL_SIZE, 0, block_cell.X, block_cell.Y);
				msg += "(" + std::to_string(block_cell.X) + "," + std::to_string(block_cell.Y) + ")";
				setCell(CLEAN_MAP, cellToCount(block_cell.X), cellToCount(block_cell.Y), BLOCKED_CLIFF);
				block_count++;
			}
		}
		ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark CLEAN_MAP %s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	}
}

uint8_t GridMap::saveSlip()
{
	if (!g_robot_slip)
		return 0;

	std::vector<Cell_t> d_cells;
	d_cells = {{1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, 0}, {0, -1}, {-1, 1}, {-1, 0}, {-1, -1}};

	int16_t	x, y;
	//int32_t	x2, y2;
	std::string msg = "cells:";
	for(auto& d_cell : d_cells)
	{
		robotToCell(getCurrPoint(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//cm_world_to_point(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_slip_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t GridMap::saveTilt()
{
	auto tilt_trig = ev.tilt_triggered;
//	ROS_INFO("%s,%d: Current(%d, %d), trig(%d)",__FUNCTION__, __LINE__,get_curr_cell().X,get_curr_cell().Y, tilt_trig);
	if (!tilt_trig)
		return 0;

	std::vector<Cell_t> d_cells;

	if(tilt_trig & TILT_LEFT)
		d_cells = {{2, 2}, {2, 1}, {2, 0}, {1, 2}, {1, 1}, {1, 0}, {0, 2}, {0, 1}, {0, 0}};
	else if(tilt_trig & TILT_RIGHT)
		d_cells = {{2, -2}, {2, -1}, {2, 0}, {1, -2}, {1, -1}, {1, 0}, {0, -2}, {0, -1}, {0, 0}};
	else if(tilt_trig & TILT_FRONT)
		d_cells = {{2, 1}, {2, 0}, {2, -1}, {1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, 0}, {0, -1}};

	int16_t	x, y;
	//int32_t	x2, y2;
	std::string msg = "cells:";
	for(auto& d_cell : d_cells)
	{
		robotToCell(getCurrPoint(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//cm_world_to_point(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_tilt_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t GridMap::saveObs()
{
	// Now obs is just for slowing down.
	return 0;
/*
	auto obs_trig = ev.obs_triggered;
//	ROS_INFO("%s, %d: ev.obs_triggered(%d)",__FUNCTION__,__LINE__,ev.obs_triggered);
	if(! obs_trig)
		return 0;

	std::vector<Cell_t> d_cells; // Direction indicator cells.
	if (obs_trig & BLOCK_FRONT)
		d_cells.push_back({2, 0});
	if (obs_trig & BLOCK_LEFT)
		d_cells.push_back({2, 1});
	if (obs_trig & BLOCK_RIGHT)
		d_cells.push_back({2, -1});

	int16_t	x, y;
	//int32_t	x2, y2;
	std::string msg = "cells:";
	for(auto& d_cell : d_cells)
	{
		if (d_cell.Y == 0 || (d_cell.Y == 1 & get_wall_adc(0) > 200) || (d_cell.Y == -1 & get_wall_adc(1) > 200))
		{
			cm_world_to_cell(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
			//cm_world_to_point(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
			robot_to_cell(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
			//robot_to_point(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
			//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
			//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
			temp_obs_cells.push_back({x, y});
			msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
		}
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());*/
}

uint8_t GridMap::saveCliff()
{
	auto cliff_trig = ev.cliff_triggered/*cliff.get_status()*/;
	if (! cliff_trig)
		// During self check.
		return 0;

	std::vector<Cell_t> d_cells;
	if (cliff_trig & BLOCK_FRONT){
		d_cells.push_back({2,-1});
		d_cells.push_back({2, 0});
		d_cells.push_back({2, 1});
	}
	if (cliff_trig & BLOCK_LEFT){
		d_cells.push_back({2, 1});
		d_cells.push_back({2, 2});
	}
	if (cliff_trig & BLOCK_RIGHT){
		d_cells.push_back({2,-1});
		d_cells.push_back({2,-2});
	}

	int16_t	x, y;
	//int32_t	x2, y2;
	std::string msg = "cells:";
	for(auto& d_cell : d_cells)
	{
		robotToCell(getCurrPoint(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//robot_to_point(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_cliff_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t GridMap::saveBumper()
{
	auto bumper_trig = ev.bumper_triggered/*bumper.get_status()*/;
//	ROS_INFO("%s,%d: Current(%d, %d), jam(%d), cnt(%d), trig(%d)",__FUNCTION__, __LINE__,get_curr_cell().X,get_curr_cell().Y, ev.bumper_jam, g_bumper_cnt, bumper_trig);
	if (!bumper_trig)
		// During self check.
		return 0;

	std::vector<Cell_t> d_cells; // Direction indicator cells.

	if ((bumper_trig & BLOCK_RIGHT) && (bumper_trig & BLOCK_LEFT))
		d_cells = {/*{2,-1},*/ {2,0}/*, {2,1}*/};
	else if (bumper_trig & BLOCK_LEFT)
	{
		if(mt.is_linear())
			d_cells = {{2, 1}/*, {2,2},{1,2}*/};
		else
			d_cells = {/*{2, 1},*//* {2,2}, */{1,2}};
	}
	else if (bumper_trig & BLOCK_RIGHT)
	{
		if(mt.is_linear())
			d_cells = {{2,-1}/*,{2,-2},{1,-2}*/};
		else
			d_cells = {/*{2,-1},*//*{2,-1},*/{1, -2}};
	}

	int16_t	x, y;
	//int32_t	x2, y2;
	std::string msg = "cells:";
	for(auto& d_cell : d_cells)
	{
		robotToCell(getCurrPoint(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//robot_to_point(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_bumper_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, getXCell(), getYCell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t GridMap::saveRcon()
{
	uint8_t block_count;
	if(c_rcon.should_mark_charger_ ){
		c_rcon.should_mark_charger_ = false;
		block_count += nav_map.saveChargerArea(c_rcon.getRconPos());
	}
	else if(c_rcon.should_mark_temp_charger_ ){
		c_rcon.should_mark_temp_charger_ = false;
		block_count += nav_map.saveChargerArea(c_rcon.getRconPos());
	}
	return block_count;

#if 0
	auto rcon_trig = ev.rcon_triggered/*rcon_get_trig()*/;
	if(! rcon_trig)
		return 0;
	if( g_from_charger && g_in_charge_signal_range && cs.is_going_home())//while in cs.is_going_home() mode_ or from_station dont mark rcon signal
	{
		ev.rcon_triggered = 0;
		return 0;
	}

	path_set_home(get_curr_cell());
	enum {
			left, fl2, fl1, fr1, fr2, right,
	};
	std::vector<Cell_t> d_cells;
	switch (ev.rcon_triggered - 1)
	{
		case left:
			d_cells.push_back({1,2});
			d_cells.push_back({1,3});
			d_cells.push_back({1,4});
			d_cells.push_back({2,2});
			d_cells.push_back({2,3});
			d_cells.push_back({2,4});
			d_cells.push_back({3,2});
			d_cells.push_back({3,3});
			d_cells.push_back({3,4});
			break;
		case fl2:
			d_cells.push_back({2,1});
			d_cells.push_back({2,2});
			d_cells.push_back({2,3});
			d_cells.push_back({3,1});
			d_cells.push_back({3,2});
			d_cells.push_back({3,3});
			d_cells.push_back({4,1});
			d_cells.push_back({4,2});
			d_cells.push_back({4,3});
//			dx = 1, dy = 2;
//			dx2 = 2, dy2 = 1;
			break;
		case fl1:
		case fr1:
			d_cells.push_back({2,0});
			d_cells.push_back({3,0});
			d_cells.push_back({4,0});
			d_cells.push_back({2,-1});
			d_cells.push_back({3,-1});
			d_cells.push_back({4,-1});
			d_cells.push_back({2,1});
			d_cells.push_back({3,1});
			d_cells.push_back({4,1});
			break;
		case fr2:
//			dx = 1, dy = -2;
//			dx2 = 2, dy2 = -1;
			d_cells.push_back({2,-1});
			d_cells.push_back({2,-2});
			d_cells.push_back({2,-3});
			d_cells.push_back({3,-1});
			d_cells.push_back({3,-2});
			d_cells.push_back({3,-3});
			d_cells.push_back({4,-1});
			d_cells.push_back({4,-2});
			d_cells.push_back({4,-3});
			break;
		case right:
			d_cells.push_back({1,-2});
			d_cells.push_back({1,-3});
			d_cells.push_back({1,-4});
			d_cells.push_back({2,-2});
			d_cells.push_back({2,-3});
			d_cells.push_back({2,-4});
			d_cells.push_back({3,-2});
			d_cells.push_back({3,-3});
			d_cells.push_back({3,-4});
			break;
	}

	int16_t	x, y;
	//int32_t	x2, y2;
	std::string msg = "cells:";
	for(auto& d_cell : d_cells)
	{
		auto point = get_curr_point();
		point.TH = g_new_dir;
		robot_to_cell(point, d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//robot_to_point(robot::instance()->getPoseAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_rcon_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
#endif
}

uint8_t GridMap::saveFollowWall()
{
	auto dy = mt.is_left() ? 2 : -2;
	int16_t x, y;
	//int32_t	x2, y2;
	std::string msg = "cell:";
	robotToCell(getCurrPoint(), dy * CELL_SIZE, 0, x, y);
	//robot_to_point(robot::instance()->getPoseAngle(), dy * CELL_SIZE, 0, &x2, &y2);
	//ROS_WARN("%s %d: d_cell(0, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
	//			, __FUNCTION__, __LINE__, dy, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
	bool should_save_for_MAP = !(cm_is_navigation() && mt.is_follow_wall() && Movement::getMoveDistance() < 0.1);
	if (should_save_for_MAP)
		temp_fw_cells.push_back({x, y});
	temp_WFMAP_follow_wall_cells.push_back({x, y});
	msg += "[0," + std::to_string(dy) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	//ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());

	return 1;
}

uint8_t GridMap::saveBlocks()
{
	uint8_t block_count = 0;
	block_count += saveBumper();
	block_count += saveCliff();
	block_count += saveObs();
	block_count += saveRcon();
	block_count += saveSlip();
//	block_count += save_follow_wall();

	return block_count;
}

//uint8_t GridMap::set_blocks()
//{
//	if(cm_is_follow_wall() || cm_is_go_charger())
//		return 0;
//
//	uint8_t block_count = 0;
//	block_count += set_obs();
//	block_count += set_bumper();
//	block_count += set_rcon();
//	block_count += set_cliff();
//	block_count += set_tilt();
//	block_count += set_slip();
//	block_count += set_lidar();
//
//	return block_count;
//}


void GridMap::setCleaned(std::deque<Cell_t> &cells)
{
	if(cells.empty())
		return;
	int8_t x_offset;

	if (!mt.is_linear())
	{
		x_offset = (cells.front().X < cells.back().X) ? 1 : -1;//X_POS
		Cell_t cell_front = {int16_t(cells.front().X - x_offset),cells.front().Y};
		Cell_t cell_back = {int16_t(cells.back().X + x_offset),cells.back().Y};
		cells.push_front(cell_front);
		cells.push_back(cell_back);
//		auto is_follow_y_min = x_offset == 1 ^ mt.is_left();
	}
	else
	{
		/*auto dir = g_plan_path.front().TH;
		if (dir == POS_X)
			x_offset = 1;
		else if (dir == NEG_X)
			x_offset = -1;
		else // POS_Y/NEG_Y
			x_offset = 0;

		if (x_offset)
		{
			Cell_t cell = {int16_t(cells.back().X + x_offset),cells.back().Y};
			cells.push_back(cell);
		}*/
	}

	std::string msg = "Cell:\n";
	for (const auto& cell :  cells)
	{
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y)  + "," + std::to_string(cell.TH)+ "),";
		for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; dy++)
		{
			auto y = cell.Y + dy;
			auto status = getCell(CLEAN_MAP, cell.X, y);
			if (status != BLOCKED_TILT && status != BLOCKED_SLIP && status != BLOCKED_RCON)
			{
				setCell(CLEAN_MAP, cellToCount(cell.X), cellToCount(y), CLEANED);
				//msg += "(" + std::to_string(cell.X) + "," + std::to_string(y) + "),";
			}
		}
//		msg += '\n';
	}

	//int32_t x, y;
	msg += "Robot it self:";
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
			//robot_to_point(robot::instance()->getPoseAngle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			auto status = getCell(CLEAN_MAP, getXCell() + dx, getYCell() + dy);
			if (status == UNCLEAN){
				setCell(CLEAN_MAP, cellToCount(getXCell() + dx), cellToCount(getYCell() + dy), CLEANED);
				msg += "(" + std::to_string(getXCell() + dx) + "," + std::to_string(getYCell() + dy) + "),";
			}
		}
	}
	ROS_INFO("%s,%d:""\033[32m %s\033[0m",__FUNCTION__, __LINE__, msg.c_str());
}

bool GridMap::markRobot(uint8_t id)
{
	if(!cs.is_trapped())
		return false;
	int32_t x, y;
	bool ret = false;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
			robotToPoint(getCurrPoint(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			auto status = getCell(id, countToCell(x), countToCell(y));
			if (status > CLEANED && status < BLOCKED_BOUNDARY && (status != BLOCKED_RCON)){
				ROS_INFO("\033[1;33m" "%s,%d: (%d,%d)" "\033[0m", __FUNCTION__, __LINE__, countToCell(x),
						 countToCell(y));
				setCell(id, x, y, CLEANED);
				ret = true;
			}
		}
	}
	return ret;
}

Cell_t GridMap::updatePosition()
{
	auto pos_x = robot::instance()->getPoseX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto pos_y = robot::instance()->getPoseY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	setPosition(pos_x, pos_y);
//	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	return getCurrCell();
}

uint32_t GridMap::getCleanedArea(void)
{
	uint32_t cleaned_count = 0;
	int16_t map_x_min, map_y_min, map_x_max, map_y_max;
	getMapRange(CLEAN_MAP, &map_x_min, &map_x_max, &map_y_min, &map_y_max);
	for (int16_t i = map_x_min; i <= map_x_max; ++i) {
		for (int16_t j = map_y_min; j <= map_y_max; ++j) {
			if (getCell(CLEAN_MAP, i, j) == CLEANED) {
				cleaned_count++;
			}
		}
	}
//	ROS_INFO("cleaned_count = %d, area = %.2fm2", cleaned_count, area);
	return cleaned_count;
}

uint8_t GridMap::isABlock(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	CellState cs;

	cs = getCell(CLEAN_MAP, x, y);
	if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY)
	//if (cs >= BLOCKED && cs <= BLOCKED_CLIFF)
		retval = 1;

	return retval;
}

uint8_t GridMap::isBlockedByBumper(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i, j;
	CellState cs;

	/* Check the point by using the robot size. */
	for (i = ROBOT_RIGHT_OFFSET; retval == 0 && i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; retval == 0 && j <= ROBOT_LEFT_OFFSET; j++) {
			cs = getCell(CLEAN_MAP, x + i, y + j);
			//if ((cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) && cs != BLOCKED_OBS) {
			if ((cs >= BLOCKED && cs <= BLOCKED_CLIFF) && cs != BLOCKED_OBS) {
				retval = 1;
			}
		}
	}

	return retval;
}

bool GridMap::isBlockAccessible(int16_t x, int16_t y)
{
	bool retval = true;
	int16_t i, j;

	for (i = ROBOT_RIGHT_OFFSET; retval == 1 && i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; retval == 1 && j <= ROBOT_LEFT_OFFSET; j++) {
			if (isABlock(x + i, y + j) == 1) {
				retval = false;
			}
		}
	}

	return retval;
}

bool GridMap::isBlockCleanable(int16_t x, int16_t y)
{
	auto retval = isUncleanAtY(x, y) && !isBlocksAtY(x, y);
//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

int8_t GridMap::isBlockCleaned(int16_t x, int16_t y)
{
	uint8_t cleaned = 0;
	int16_t i, j;

	for (i = ROBOT_RIGHT_OFFSET; i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; j <= ROBOT_LEFT_OFFSET; j++) {
			auto state = getCell(CLEAN_MAP, x + i, y + j);
			if (state == CLEANED) {
				cleaned ++;
			} else if(isABlock(x, y))
				return false;
		}
	}

	return cleaned >= 7;
}

uint8_t GridMap::isUncleanAtY(int16_t x, int16_t y)
{
	uint8_t unclean_cnt = 0;
	for (int8_t i = (y + ROBOT_RIGHT_OFFSET); i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (getCell(CLEAN_MAP, x, i) == UNCLEAN) {
			unclean_cnt++;
		}
	}
//	ROS_INFO("%s, %d:unclean_cnt(%d)", __FUNCTION__, __LINE__, unclean_cnt);
	return unclean_cnt;
}

uint8_t GridMap::isBlockBoundary(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (getCell(CLEAN_MAP, x, i) == BLOCKED_BOUNDARY) {
			retval = 1;
		}
	}

	return retval;
}

uint8_t GridMap::isBlocksAtY(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (isABlock(x, i) == 1) {
			retval = 1;
		}
	}

//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

uint8_t GridMap::isBlockBlockedXAxis(int16_t curr_x, int16_t curr_y)
{
	uint8_t retval = 0;
	int16_t x,y;
	auto dy = mt.is_left()  ?  2 : -2;
	for(auto dx =-1; dx<=1,retval == 0; dx++) {
		robotToCell(getCurrPoint(), CELL_SIZE * dy, CELL_SIZE * dx, x, y);
		if (isABlock(x, y) == 1) {
			retval = 1;
		}
	}

//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

void GridMap::generateSPMAP(const Cell_t &curr, PPTargetType &target_list)
{
	bool		all_set;
	int16_t		x, y, offset, passValue, nextPassValue, passSet, x_min, x_max, y_min, y_max;
	CellState	cs;
	reset(COST_MAP);

	getMapRange(COST_MAP, &x_min, &x_max, &y_min, &y_max);
	for (auto i = x_min; i <= x_max; ++i) {
		for (auto j = y_min; j <= y_max; ++j) {
			cs = getCell(CLEAN_MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				for (x = ROBOT_RIGHT_OFFSET; x <= ROBOT_LEFT_OFFSET; x++) {
					for (y = ROBOT_RIGHT_OFFSET; y <= ROBOT_LEFT_OFFSET; y++) {
						setCell(COST_MAP, i + x, j + y, COST_HIGH);
					}
				}
			}
		}
	}

	x = curr.X;
	y = curr.Y;

	/* Set the current robot position has the cost value of 1. */
	setCell(COST_MAP, (int32_t) x, (int32_t) y, COST_1);

	offset = 0;
	passSet = 1;
	passValue = 1;
	nextPassValue = 2;
	while (passSet == 1) {
		offset++;
		passSet = 0;
		for (auto i = x - offset; i <= x + offset; i++) {
			if (i < x_min || i > x_max)
				continue;

			for (auto j = y - offset; j <= y + offset; j++) {
				if (j < y_min || j > y_max)
					continue;

				if(getCell(COST_MAP, i, j) == passValue) {
					if (i - 1 >= x_min && getCell(COST_MAP, i - 1, j) == COST_NO) {
						setCell(COST_MAP, (i - 1), (j), (CellState) nextPassValue);
						passSet = 1;
					}

					if ((i + 1) <= x_max && getCell(COST_MAP, i + 1, j) == COST_NO) {
						setCell(COST_MAP, (i + 1), (j), (CellState) nextPassValue);
						passSet = 1;
					}

					if (j - 1  >= y_min && getCell(COST_MAP, i, j - 1) == COST_NO) {
						setCell(COST_MAP, (i), (j - 1), (CellState) nextPassValue);
						passSet = 1;
					}

					if ((j + 1) <= y_max && getCell(COST_MAP, i, j + 1) == COST_NO) {
						setCell(COST_MAP, (i), (j + 1), (CellState) nextPassValue);
						passSet = 1;
					}
				}
			}
		}

		all_set = true;
		for (auto it = target_list.begin(); it != target_list.end(); ++it) {
			if (getCell(COST_MAP, it->X, it->Y) == COST_NO) {
				all_set = false;
			}
		}
		if (all_set) {
			//ROS_INFO("%s %d: all possible target are checked & reachable.", __FUNCTION__, __LINE__);
			passSet = 0;
		}

		passValue = nextPassValue;
		nextPassValue++;
		if(nextPassValue == COST_PATH)
			nextPassValue = 1;
	}
//	print(COST_MAP, 0,0);
}

bool GridMap::isFrontBlockBoundary(int dx)
{
	int32_t x, y;
	for (auto dy = -1; dy <= 1; dy++)
	{
		robotToPoint(getCurrPoint(), dy * CELL_SIZE, CELL_SIZE * dx, &x, &y);
		if (getCell(CLEAN_MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY)
			return true;
	}
	return false;
}
void GridMap::getMapRange(uint8_t id, int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min,
						  int16_t *y_range_max)
{
	if (id == CLEAN_MAP || id == COST_MAP) {
		*x_range_min = g_x_min - (abs(g_x_min - g_x_max) <= 3 ? 3 : 1);
		*x_range_max = g_x_max + (abs(g_x_min - g_x_max) <= 3 ? 3 : 1);
		*y_range_min = g_y_min - (abs(g_y_min - g_y_max) <= 3? 3 : 1);
		*y_range_max = g_y_max + (abs(g_y_min - g_y_max) <= 3 ? 3 : 1);
	}
//	ROS_INFO("Get Range:\tx: %d - %d\ty: %d - %d\tx range: %d - %d\ty range: %d - %d",
//		g_x_min, g_x_max, g_y_min, g_y_max, *x_range_min, *x_range_max, *y_range_min, *y_range_max);
}

bool GridMap::cellIsOutOfRange(Cell_t cell)
{
	return std::abs(cell.X) > MAP_SIZE || std::abs(cell.Y) > MAP_SIZE;
}

void GridMap::print(uint8_t id, int16_t endx, int16_t endy)
{
	char outString[256];
	#if ENABLE_DEBUG
	int16_t		i, j, x_min, x_max, y_min, y_max, index;
	CellState	cs;
	Cell_t temp_cell;

	temp_cell = getCurrCell();

	getMapRange(id, &x_min, &x_max, &y_min, &y_max);

	if (id == CLEAN_MAP) {
		ROS_INFO("Map: %s", "CLEAN_MAP");
	} else if (id == COST_MAP) {
		ROS_INFO("Map: %s", "COST_MAP");
	}
	index = 0;
	outString[index++] = '\t';
	for (j = y_min; j <= y_max; j++) {
		if (abs(j) % 10 == 0) {
			outString[index++] = (j < 0 ? '-' : ' ');
			outString[index++] = (abs(j) >= 100 ? abs(j) / 100 + 48 : ' ');
			outString[index++] = 48 + (abs(j) >= 10 ? ((abs(j) % 100) / 10) : 0);
			j += 3;
		} else {
			outString[index++] = ' ';
		}
	}
	outString[index++] = 0;

	printf("%s\n",outString);
	index = 0;
	outString[index++] = '\t';
	for (j = y_min; j <= y_max; j++) {
		outString[index++] = abs(j) % 10 + 48;
	}
	outString[index++] = 0;
	printf("%s\n",outString);
	memset(outString,0,256);
	for (i = x_min; i <= x_max; i++) {
		index = 0;

		outString[index++] = (i < 0 ? '-' : ' ');
		outString[index++] = 48 + (abs(i) >= 100 ? abs(i) / 100 : 0);
		outString[index++] = 48 + (abs(i) >= 10 ? ((abs(i) % 100) / 10) : 0);
		outString[index++] = abs(i) % 10 + 48;
		outString[index++] = '\t';

		for (j = y_min; j <= y_max; j++) {
			cs = getCell(id, i, j);
			if (i == temp_cell.X && j == temp_cell.Y) {
				outString[index++] = 'x';
			} else if (i == endx && j == endy) {
				outString[index++] = 'e';
			} else {
				outString[index++] = cs + 48;
			}
		}
		#if COLOR_DEBUG_MAP
		colorPrint(outString, 0, index);
		#else
		printf("%s\n", outString);
		#endif
	}
	printf("\n");
	#endif
}

void GridMap::colorPrint(char *outString, int16_t y_min, int16_t y_max)
{
	int16_t j = 0;
	char cs;
	bool ready_print_map = false;
	std::string y_col("");
	for(j =y_min; j<=y_max; j++){
		cs = *(outString+j);
		if(cs =='\t' && !ready_print_map){
			ready_print_map = true;
			y_col+="\t";
			continue;
		}
		else if(!ready_print_map){
			y_col+=cs;
			continue;
		}
		if(ready_print_map){
			if(cs == '0'){//unclean
				y_col+=cs;
			}
			else if(cs == '1'){//clean
				if(std::abs(j%2) == 0)
					y_col+="\033[1;46;37m1\033[0m";// cyan
				else
					y_col+="\033[1;42;37m1\033[0m";// green
			}
			else if(cs == '2'){//obs
				y_col+="\033[1;44;37m2\033[0m";// blue
			}
			else if(cs == '3'){//bumper
				y_col+="\033[1;41;37m3\033[0m";// red
			}
			else if(cs == '4'){//cliff
				y_col+="\033[1;45;37m4\033[0m";// magenta
			}
			else if(cs == '5'){//rcon
				y_col+="\033[1;47;37m5\033[0m";// white
			}
			else if(cs == '6'){//lidar maker
				y_col+="\033[1;44;37m6\033[0m";// blue
			}
			else if(cs == '7'){//tilt
				y_col+="\033[1;47;30m7\033[0m";// white
			}
			else if(cs == '8'){//slip
				y_col+="\033[1;43;37m8\033[0m";// yellow
			}
			else if(cs == '9'){//slam_map_block
				y_col+="\033[0;41;37m9\033[0m";// red
			}
			else if(cs == 'a'){//bundary
				y_col+="\033[1;43;37ma\033[0m";
			}
			else if(cs == 'e'){//end point
				y_col+="\033[1;43;37me\033[0m";
			}
			else if(cs == 'x'){//cur point
				y_col+="\033[1;43;37mx\033[0m";
			}
			else if(cs == '>'){//target point
				y_col+="\033[1;40;37m>\033[0m";
			}
			else{
				y_col+=cs;
			}
		}
	}
	printf("%s\033[0m\n",y_col.c_str());
}

bool GridMap::isFrontBlocked(void)
{
	bool retval = false;
	int16_t x,y;
	std::vector<Cell_t> d_cells;
	d_cells = {{2,1},{2,0},{2,-1},{1,2},{1,1},{1,0},{1,-1},{1,-2},{0,0}};

	for(auto& d_cell : d_cells)
	{
		robotToCell(getCurrPoint(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		if(getCell(CLEAN_MAP, x, y) == SLAM_MAP_BLOCKED)
		{
			retval = true;
			break;
		}
	}
	return retval;
}

