#include <gyro.h>
#include <event_manager.h>
#include <mode.hpp>
#include "map.h"
#include "robot.hpp"
#include "event_manager.h"
#include "rcon.h"
#include "lidar.hpp"

GridMap slam_grid_map;
GridMap decrease_map;
extern const Cell_t cell_direction_[9];

Cell_t g_stub_cell(0,0);

GridMap::GridMap(){
	mapInit();
}

//GridMap::GridMap(uint16_t map_size){
//	mapInit();
//}

void GridMap::mapInit()
{
	for(auto c = 0; c < MAP_SIZE; ++c) {
		for(auto d = 0; d < (MAP_SIZE + 1) / 2; ++d) {
			clean_map[c][d] = 0;
			cost_map[c][d] = 0;
		}
	}
	g_x_min = g_x_max = g_y_min = g_y_max = 0;
	xRangeMin = static_cast<int16_t>(g_x_max - (MAP_SIZE - 1));
	xRangeMax = static_cast<int16_t>(g_x_min + (MAP_SIZE - 1));
	yRangeMin = static_cast<int16_t>(g_y_max - (MAP_SIZE - 1));
	yRangeMax = static_cast<int16_t>(g_y_min + (MAP_SIZE - 1));

//		xCount = 0;
//		yCount = 0;

}

GridMap::~GridMap()
{
}

/*
 * GridMap::get_cell description
 * @param id	Map id
 * @param x	Cell x
 * @param y	Cell y
 * @return	CellState
 */
CellState GridMap::getCell(int id, int16_t x, int16_t y) {
	CellState val=0;
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
void GridMap::setCell(uint8_t id, int16_t x, int16_t y, CellState value) {
	CellState val;
	int16_t ROW, COLUMN;
	if(id == CLEAN_MAP) {
		if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
			if(x < g_x_min) {
				g_x_min = x;
				xRangeMin = static_cast<int16_t>(g_x_max - (MAP_SIZE - 1));//40 --> 1
				xRangeMax = static_cast<int16_t>(g_x_min + (MAP_SIZE - 1));
			} else if(x > g_x_max) {
				g_x_max = x;
				xRangeMin = static_cast<int16_t>(g_x_max - (MAP_SIZE - 1));
				xRangeMax = static_cast<int16_t>(g_x_min + (MAP_SIZE - 1));
			}
			if(y < g_y_min) {
				g_y_min = y;
				yRangeMin = static_cast<int16_t>(g_y_max - (MAP_SIZE - 1));
				yRangeMax = static_cast<int16_t>(g_y_min + (MAP_SIZE - 1));
			} else if(y > g_y_max) {
				g_y_max = y;
				yRangeMin = static_cast<int16_t>(g_y_max - (MAP_SIZE - 1));
				yRangeMax = static_cast<int16_t>(g_y_min + (MAP_SIZE - 1));
			}

			ROW = static_cast<int16_t>(x + MAP_SIZE + MAP_SIZE / 2);
			ROW %= MAP_SIZE;
			COLUMN = static_cast<int16_t>(y + MAP_SIZE + MAP_SIZE / 2);
			COLUMN %= MAP_SIZE;

			val = (CellState) clean_map[ROW][COLUMN / 2];
			if (((COLUMN % 2) == 0 ? (val >> 4) : (val & 0x0F)) != value) {
				clean_map[ROW][COLUMN / 2] = ((COLUMN % 2) == 0 ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
//				clean_map[ROW][COLUMN] = value;
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
//			cost_map[x][y] = value;
		}
	}
}

void GridMap::clearBlocks(void) {
	int16_t c, d;

	for(c = g_x_min; c < g_x_max; ++c) {
		for(d = g_y_min; d < g_y_max; ++d) {
			CellState state = getCell(CLEAN_MAP, c, d);
			if(state == BLOCKED_LIDAR || state == BLOCKED_BUMPER ||	state == BLOCKED_CLIFF || state  == BLOCKED_FW) {
				if(getCell(CLEAN_MAP, c - 1, d) != UNCLEAN && getCell(CLEAN_MAP, c, d + 1) != UNCLEAN &&
						getCell(CLEAN_MAP, c + 1, d) != UNCLEAN &&
						getCell(CLEAN_MAP, c, d - 1) != UNCLEAN) {
					setCell(CLEAN_MAP,c,d, CLEANED);
				}
			}
		}
	}

	setCell(CLEAN_MAP,getPosition().toCell().x - 1, getPosition().toCell().y , CLEANED);
	setCell(CLEAN_MAP,getPosition().toCell().x,getPosition().toCell().y + 1, CLEANED);
	setCell(CLEAN_MAP,getPosition().toCell().x + 1,getPosition().toCell().y, CLEANED);
	setCell(CLEAN_MAP,getPosition().toCell().x,getPosition().toCell().y - 1, CLEANED);
}

void GridMap::setCells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state)
{
	int8_t i, j;

	for ( i = -1; i <= 1; i++ ) {
		for ( j = -1; j <= 1; j++ ) {
			setCell(CLEAN_MAP,cell_x + i,cell_y + j, state);
		}
	}
}

void GridMap::setCellsBut(int8_t count, int16_t cell_x, int16_t cell_y, CellState state,CellState but_state)
{
	int8_t i, j;

	for ( i = -1; i <= 1; i++ ) {
		for ( j = -1; j <= 1; j++ ) {
			auto s = getCell(CLEAN_MAP,cell_x + i,cell_y + j);
			if(s != but_state)
				setCell(CLEAN_MAP,cell_x + i,cell_y + j, state);
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
		g_x_min = g_x_max = g_y_min = g_y_max = 0;
		xRangeMin = static_cast<int16_t>(g_x_min - (MAP_SIZE - (g_x_max - g_x_min + 1)));
		xRangeMax = static_cast<int16_t>(g_x_max + (MAP_SIZE - (g_x_max - g_x_min + 1)));
		yRangeMin = static_cast<int16_t>(g_y_min - (MAP_SIZE - (g_y_max - g_y_min + 1)));
		yRangeMax = static_cast<int16_t>(g_y_max + (MAP_SIZE - (g_y_max - g_y_min + 1)));
	}
#endif
}

void GridMap::copy(GridMap &source_map)
{
	int16_t map_x_min, map_y_min, map_x_max, map_y_max;
	reset(CLEAN_MAP);
	source_map.getMapRange(CLEAN_MAP, &map_x_min, &map_x_max, &map_y_min, &map_y_max);

	for (int16_t x = map_x_min; x <= map_x_max; x++)
	{
		for (int16_t y = map_y_min; y <= map_y_max; y++)
			setCell(CLEAN_MAP, x, y, source_map.getCell(CLEAN_MAP, x, y));
	}
}

void GridMap::convertFromSlamMap(float resolution_target,float threshold,const BoundingBox2& bound)
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

	slam_map.getData(slam_map_data);

	// Set resolution multi between cost map and slam map.
	auto multi = resolution_target / resolution;
	// Limit count for checking block.*/
	auto limit_count = static_cast<uint16_t>((multi * multi) * threshold);
	// Set boundary for this cost map.

	int16_t map_x_min = std::max(static_cast<int16_t>(origin_x  / resolution_target), bound.min.x);
	int16_t map_x_max = std::min(static_cast<int16_t>(map_x_min + width / multi), bound.max.x);
	int16_t map_y_min = std::max(static_cast<int16_t>(origin_y / resolution_target), bound.min.y);
	int16_t map_y_max = std::min(static_cast<int16_t>(map_y_min + height / multi), bound.max.y);

	ROS_INFO("%s,%d: resolution: %f, multi: %f, limit cnt:%d, map_x_min: %d, map_x_max: %d, map_y_min: %d, map_y_max: %d",
	 		   __FUNCTION__, __LINE__, resolution, multi, limit_count, map_x_min, map_x_max, map_y_min, map_y_max);
	for (auto cell_x = map_x_min; cell_x <= map_x_max; ++cell_x)
	{
//		ROS_ERROR("cell:");
		for (auto cell_y = map_y_min; cell_y <= map_y_max; ++cell_y)
		{
			// Get the range of this cell in the grid map of slam map data.
			double world_x, world_y;
			world_x = (double)cell_x * resolution_target;
			world_y = (double)cell_y * resolution_target;
			uint32_t data_map_x, data_map_y;
			if (worldToSlamMap(origin_x, origin_y, resolution, width, height, world_x, world_y, data_map_x, data_map_y))
			{

				auto data_map_x_min = std::max(static_cast<uint32_t>(data_map_x - multi/2) , 0u);
				auto data_map_x_max = std::min(static_cast<uint32_t>(data_map_x + multi/2) , static_cast<uint32_t>(width));
				auto data_map_y_min = std::max(static_cast<uint32_t>(data_map_y - multi/2) , 0u) ;
				auto data_map_y_max = std::min(static_cast<uint32_t>(data_map_y + multi/2) , static_cast<uint32_t>(height));

				// Get the slam map data s_index_ of this range.
				std::vector<int32_t> slam_map_data_index{};
				for (uint32_t i = data_map_x_min; i <= data_map_x_max; i++)
					for(uint32_t j = data_map_y_min; j <= data_map_y_max; j++)
					{
                        int32_t index = getIndexOfSlamMapData(width, i, j);
						if(index < slam_map_data.size())
							slam_map_data_index.push_back(index);
					}

				// Values for counting sum of difference slam map data.
				uint32_t block_counter = 0, cleanable_counter = 0, unknown_counter = 0;
				bool block_set = false;
				auto size = slam_map_data_index.size();
				for(int index = 0; index < size; index++)
				{
					if(slam_map_data[slam_map_data_index[index]] == 100)		block_counter++;
					else if(slam_map_data[slam_map_data_index[index]] == -1)	unknown_counter++;
					else if(slam_map_data[slam_map_data_index[index]] == 0)		cleanable_counter++;
					if(block_counter > limit_count)/*---occupied cell---*/
					{
						setCell(CLEAN_MAP,cell_x,cell_y, SLAM_MAP_BLOCKED);
						block_set = true;
						break;
					}
				}

				if(!block_set && unknown_counter == size)/*---unknown cell---*/
				{
					setCell(CLEAN_MAP,cell_x,cell_y, SLAM_MAP_UNKNOWN);
					block_set = true;
				}

				if(!block_set)/*---unknown cell---*/
				{
//					ROS_ERROR_COND(cell_x > 0 && cell_y > 0,"~~~~~~~~~~~range(%d,%d,%d,%d),size(%d)", data_map_x_min, data_map_y_min, data_map_x_max, data_map_y_max,size);
					setCell(CLEAN_MAP,cell_x,cell_y, SLAM_MAP_REACHABLE);
//					ROS_ERROR_COND(cell_x > 0 && cell_y > 0,"cell(%d,%d)~~~~range(%d,%d,%d,%d)",cell_x,cell_y, g_x_min, g_y_min, g_x_max, g_y_max);
				}
			}
		}
//		printf("\n");
	}

//    ROS_ERROR("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
	int16_t		x, y, x_min, x_max, y_min, y_max;
	getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);
}

void GridMap::merge(GridMap source_map, bool add_slam_map_blocks_to_uncleaned,
					bool add_slam_map_blocks_to_cleaned,
					bool add_slam_map_cleanable_area, bool clear_map_blocks, bool clear_slam_map_blocks,
					bool clear_bumper_and_lidar_blocks)
{
	int16_t map_x_min, map_y_min, map_x_max, map_y_max;
	source_map.getMapRange(CLEAN_MAP, &map_x_min, &map_x_max, &map_y_min, &map_y_max);
	for (int16_t x = map_x_min; x <= map_x_max; x++)
	{
		for (int16_t y = map_y_min; y <= map_y_max; y++)
		{
			CellState map_cell_state, source_map_cell_state;
			map_cell_state = getCell(CLEAN_MAP, x, y);
			source_map_cell_state = source_map.getCell(CLEAN_MAP, x, y);

			if (clear_bumper_and_lidar_blocks &&
				(map_cell_state == BLOCKED_BUMPER || map_cell_state == BLOCKED_LIDAR) &&
				source_map_cell_state == SLAM_MAP_REACHABLE)
				setCell(CLEAN_MAP,x,y, CLEANED);

			if (clear_map_blocks && map_cell_state >= BLOCKED && map_cell_state < SLAM_MAP_BLOCKED && source_map_cell_state == SLAM_MAP_REACHABLE)
				setCell(CLEAN_MAP,x,y, CLEANED);

			if (clear_slam_map_blocks && map_cell_state == SLAM_MAP_BLOCKED && source_map_cell_state == SLAM_MAP_REACHABLE)
				setCell(CLEAN_MAP,x,y, CLEANED);

			if (add_slam_map_blocks_to_uncleaned && map_cell_state == UNCLEAN && source_map_cell_state == SLAM_MAP_BLOCKED)
				setCell(CLEAN_MAP,x,y, SLAM_MAP_BLOCKED);
			if (add_slam_map_blocks_to_cleaned && map_cell_state == CLEANED && source_map_cell_state == SLAM_MAP_BLOCKED)
				setCell(CLEAN_MAP,x,y, SLAM_MAP_BLOCKED);

			if (add_slam_map_cleanable_area && map_cell_state == UNCLEAN && source_map_cell_state == SLAM_MAP_REACHABLE)
				setCell(CLEAN_MAP,x,y, CLEANED);
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

//bool GridMap::worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy)
//{
//	auto count_x = wx * 1000 * CELL_COUNT_MUL / CELL_SIZE;
//	auto count_y = wy * 1000 * CELL_COUNT_MUL / CELL_SIZE;
////cx = count_to_cell(count_x);
//	cx = count_x;
////cy = count_to_cell(count_y);
//	cy = count_y;
//	return true;
//}

void GridMap::cellToWorld(double &worldX, double &worldY, int16_t &cellX, int16_t &cellY)
{
	worldX = (double)cellX * CELL_SIZE;
	worldY = (double)cellY * CELL_SIZE;
}

bool GridMap::markRobot(uint8_t id)
{
	auto curr =  getPosition().toCell();
	bool ret = false;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
//			robotToCell(getPosition(), CELL_SIZE * dy, CELL_SIZE * dx, x, y);
			auto x = curr.x + (int16_t)dx;
			auto y = curr.y + (int16_t)dy;
			auto status = getCell(id, x, y);
			if (/*status > CLEANED && */status < BLOCKED_BOUNDARY && (status != BLOCKED_RCON)){
				ROS_INFO("\033[1;33m" "%s,%d: (%d,%d)" "\033[0m", __FUNCTION__, __LINE__,x, y);
				setCell(id, x, y, CLEANED);
				ret = true;
			}
		}
	}
	return ret;
}

bool GridMap::trapMarkRobot(uint8_t id)
{
	int16_t x, y;
	bool ret = false;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
//			robotToCell(getPosition(), CELL_SIZE * dy, CELL_SIZE * dx, x, y);
			x = getPosition().toCell().x + dx;
			y = getPosition().toCell().y + dy;
			auto status = getCell(id, x, y);
			if (/*status > CLEANED && */status < BLOCKED_BOUNDARY/* && (status != BLOCKED_RCON)*/){
//				ROS_INFO("\033[1;33m" "%s,%d: (%d,%d)" "\033[0m", __FUNCTION__, __LINE__,x, y);
				setCell(id, x, y, CLEANED);
				ret = true;
			}
		}
	}
	return ret;
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
			//if ((cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) && cs != BLOCKED_FW) {
			if ((cs >= BLOCKED && cs <= BLOCKED_CLIFF) && cs != BLOCKED_FW) {
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

int8_t GridMap::isNotBlockAndCleaned(int16_t x, int16_t y)
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

uint8_t GridMap::isBlockAtY(int block, int16_t x, int16_t y)
{
	uint8_t unclean_cnt = 0;
	for (int8_t i = (y + ROBOT_RIGHT_OFFSET); i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (getCell(CLEAN_MAP, x, i) == block) {
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

uint8_t GridMap::isBlockBlockedXAxis(int16_t curr_x, int16_t curr_y, bool is_left)
{
	uint8_t retval = 0;
	auto dy = is_left  ?  2 : -2;
	for(auto dx =-1; dx<=1,retval == 0; dx++) {
		auto cell = getPosition().getRelative(CELL_SIZE * dx, CELL_SIZE * dy).toCell();
		if (isABlock(cell.x, cell.y) == 1) {
			retval = 1;
		}
	}

//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

int8_t GridMap::isNeedClean(int16_t x, int16_t y)
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

	return cleaned <= 6;
}

bool GridMap::count_if(const Cell_t &curr_cell, std::function<bool(const Cell_t &next)> compare, int& count) {
	Cells targets{};
    std::set<Cell_t> c_cleans;
	find_if(getPosition().toCell(), targets, [&](Cell_t c_it) {
        for (auto c_neight = std::begin(cell_direction_); c_neight != std::end(cell_direction_); ++c_neight) {
			auto tmp = c_it + *c_neight;
			if (compare(tmp))
				c_cleans.insert(tmp);
		}
		return c_it.y%2 == 0 && getCell(CLEAN_MAP, c_it.x, c_it.y) == UNCLEAN  && isBlockAccessible(c_it.x, c_it.y);
	},true,true,true);
	count = c_cleans.size();
    ROS_ERROR("%s,is_trapped(%d),trapped_clean_count(%d)",__FUNCTION__,targets.empty(), count);
	return targets.empty();
}

bool GridMap::find_if(const Cell_t &curr_cell, Cells &targets, std::function<bool(const Cell_t &next)> compare  ,bool is_count,bool is_stop, bool is_target) {
	typedef std::multimap<int16_t, Cell_t> Queue;
	typedef std::pair<int16_t, Cell_t> Entry;

	reset(COST_MAP);
	Queue queue;
	setCell(COST_MAP, curr_cell.x, curr_cell.y, 1);
	queue.emplace(1, curr_cell);

	while (!queue.empty()) {
//		 Get the nearest next from the queue
		if (queue.begin()->first == 5) {
			Queue tmp_queue;
			std::for_each(queue.begin(), queue.end(), [&](const Entry &iterators) {
				tmp_queue.emplace(0, iterators.second);
			});
			queue.swap(tmp_queue);
		}
		auto start = queue.begin();
		auto next = start->second;
		auto cost = start->first;
		queue.erase(start);

		for (auto index = 0; index < 4; index++) {
			auto neighbor = next + cell_direction_[index];

			if(is_target ? isOutOfTargetRange(next) : isOutOfMap(next))
				continue;

			if (getCell(COST_MAP, neighbor.x, neighbor.y) == 0) {
                if(is_count)
					compare(neighbor);
				if (isBlockAccessible(neighbor.x, neighbor.y))
				{

					if (compare(next))
					{
						targets.push_back(next);
                        if(is_stop)
							return true;
					}
					queue.emplace(cost + 1, neighbor);
					setCell(COST_MAP, neighbor.x, neighbor.y, cost + 1);
				}
			}
		}
	}
	return !targets.empty();
}

//void GridMap::generateSPMAP(const Cell_t& curr_cell,Cells& targets)
//{
//				if (getCell(COST_MAP, neighbor.x, neighbor.y) == 0) {
//					if (isBlockAccessible(neighbor.x, neighbor.y)) {
//						if(getCell(CLEAN_MAP, neighbor.x, neighbor.y) == UNCLEAN && neighbor.y % 2 == 0)
//							targets.push_back(neighbor);
//						queue.emplace(cost+1, neighbor);
//						setCell(COST_MAP, neighbor.x, neighbor.y, cost+1);
//					}
//				}
//}

bool GridMap::isFrontBlockBoundary(int dx)
{
	Point_t point;
	for (auto dy = -1; dy <= 1; dy++)
	{
		auto cell = getPosition().getRelative(dx * CELL_SIZE, CELL_SIZE * dy).toCell();
		if (getCell(CLEAN_MAP, cell.x, cell.y) == BLOCKED_BOUNDARY)
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
//	ROS_INFO("Get Range:min(%d,%d),max(%d,%d)", g_x_min,g_y_min, g_x_max,  g_y_max);
}

bool GridMap::isOutOfMap(const Cell_t &cell)
{
	return cell.x < g_x_min-1 || cell.y < g_y_min-1 || cell.x > g_x_max+1 || cell.y > g_y_max+1;
}
bool GridMap::isOutOfTargetRange(const Cell_t &cell)
{
	return cell.x < g_x_min-2 || cell.y < g_y_min-2 || cell.x > g_x_max+2 || cell.y > g_y_max+2;
}
bool GridMap::cellIsOutOfRange(Cell_t cell)
{
	return std::abs(cell.x) > MAP_SIZE || std::abs(cell.y) > MAP_SIZE;
}

void GridMap::print(const Cell_t& curr_cell, uint8_t id, const Cells& targets)
{
//	Cell_t curr_cell = getPosition().toCell();
	std::ostringstream outString;
	outString.str("");
	#if 1
	int16_t		x, y, x_min, x_max, y_min, y_max;
	CellState	cs;

	getMapRange(id, &x_min, &x_max, &y_min, &y_max);
//	x_min -= 1;x_max += 1; y_min -= 1; y_max +=1;
	outString << '\t';
	for (y = y_min; y <= y_max; y++) {
		if (abs(y) % 10 == 0) {
			outString << std::abs(y/10);
		} else {
			outString << ' ';
		}
	}

	printf("%s\n",outString.str().c_str());
	outString.str("");
	outString << '\t';
	for (y = y_min; y <= y_max; y++) {
		outString << abs(y) % 10;
	}
//	std::cout << std::to_string(static_cast<int>(&g_ab)) << std::endl;
	printf("%s\n",outString.str().c_str());
	outString.str("");
	for (x = x_min; x <= x_max; x++) {
		outString.width(4);
		outString << x;
		outString << '\t';
		for (y = y_min; y <= y_max; y++) {
			cs = getCell(id, x, y);
			if (x == curr_cell.x && y == curr_cell.y)
				outString << 'x';
			else if (std::find_if(targets.begin(), targets.end(), [&](const Cell_t& c_it ){return c_it == Cell_t{x,y};}) != targets.end())
				outString << 'e';
			else if (cs == SLAM_MAP_BLOCKED)
				outString << 'a';
			else
			{
                if(id == CLEAN_MAP &&  (x == x_min || x==x_max) || (y == y_min || y == y_max) )
					outString << '*';
				else
					outString << cs;
			}

		}
//		printf("%s\n",outString.str().c_str());
//		#if COLOR_DEBUG_MAP
		colorPrint(outString.str().c_str(), 0, static_cast<int16_t>(outString.str().size()));
		outString.str("");
//		#else
//		printf("%s\n", outString);
//		#endif
	}
//	printf("\n");
	#endif
}

void GridMap::printInRange(const Cell_t& curr_cell, uint8_t id, const Cells& targets,bool is_bound,BoundingBox2 bound) {

    ROS_INFO("range(%d,%d,%d,%d)curr(%d,%d)", bound.min.x,bound.max.x, bound.min.y, bound.max.y,curr_cell.x, curr_cell.y);
	std::ostringstream outString;
    outString << '\t';
	for (auto y = bound.min.y; y <= bound.max.y; y++) {
		if (abs(y) % 10 == 0) {
			outString << std::abs(y/10);
		} else {
			outString << ' ';
		}
	}
	outString << '\n';

	printf("%s\n",outString.str().c_str());
	outString.str("");
	outString << '\t';
	for (auto y = bound.min.y; y <= bound.max.y; y++) {
		outString << abs(y) % 10;
	}
	outString << '\n';
	for (auto x = bound.min.x; x <= bound.max.x; ++x) {
   		outString.width(4);
		outString << x;
		outString << '\t';
		for (auto y = bound.min.y; y <= bound.max.y; ++y) {
            auto val = getCell(CLEAN_MAP, x, y);
            if(curr_cell.x == x && curr_cell.y == y)
				outString << 'x';
			else if (val == 0) {
				outString << '0';
			} else if (val == 1) {
				outString << '1';
			} else {
				outString << 'a';
			}
		}
		outString << '\n';
	}
//	printf("%s\n",outString.str().c_str());
	colorPrint(outString.str().c_str(), 0, static_cast<int16_t>(outString.str().size()));
}

void GridMap::colorPrint(const char *outString, int16_t y_min, int16_t y_max)
{
	char cs;
	bool ready_print_map = false;
	std::string y_col;
	for(auto j = y_min; j<=y_max; j++){
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
			else if(cs == '4'){//lidar maker
				y_col+="\033[1;44;37m4\033[0m";// blue
			}
			else if(cs == '5'){//cliff
				y_col+="\033[1;45;37m5\033[0m";// magenta
			}
			else if(cs == '6'){//rcon
				y_col+="\033[1;47;37m6\033[0m";// white
			}
			else if(cs == '7'){//tmp rcon
				y_col+="\033[1;47;37m7\033[0m";// white
			}
			else if(cs == '8'){//tilt
				y_col+="\033[1;47;30m8\033[0m";// white
			}
			else if(cs == '9'){//slip
				y_col+="\033[1;43;37m9\033[0m";// yellow
			}
			else if(cs == 'a'){//slam_map_block
				y_col+="\033[0;41;37ma\033[0m";// red
			}
			else if(cs == 'b'){//bundary
				y_col+="\033[1;43;37mb\033[0m";
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

bool GridMap::isFrontBlocked(Dir_t dir)
{
	bool retval = false;
	std::vector<Cell_t> d_cells;
	if(isAny(dir) || (isXAxis(dir) && isPos(dir)))
		d_cells = {{2,1},{2,0},{2,-1}};
	else if(isXAxis(dir) && !isPos(dir))
		d_cells = {{-2,1},{-2,0},{-2,-1}};
	else if(!isXAxis(dir) && isPos(dir))
		d_cells = {{1,2},{0,2},{-1,2}};
	else if(!isXAxis(dir) && !isPos(dir))
		d_cells = {{1,-2},{0,-2},{-1,-2}};
	for(auto& d_cell : d_cells)
	{
		Cell_t cell;
		if(isAny(dir))
		{
			cell = getPosition().getRelative(d_cell.x * CELL_SIZE, d_cell.y * CELL_SIZE).toCell();
		}
		else{
				cell = getPosition().toCell() + d_cell;
		}

		if(isABlock(cell.x, cell.y))
		{
			retval = true;
			break;
		}
	}
	return retval;
}

bool GridMap::isFrontSlamBlocked(void)
{
	bool retval = false;
	std::vector<Cell_t> d_cells;
	d_cells = {{2,1},{2,0},{2,-1}};

	for(auto& d_cell : d_cells)
	{
		auto cell = getPosition().getRelative(d_cell.x * CELL_SIZE, d_cell.y * CELL_SIZE).toCell();
		if(getCell(CLEAN_MAP, cell.x, cell.y) == SLAM_MAP_BLOCKED)
		{
			retval = true;
			break;
		}
	}
	return retval;
}

void GridMap::setCircleMarkers(Point_t point, int radius, CellState cell_state,Marks& error_maker)
{
	const int RADIUS_CELL = radius;
	Point_t tmp_point = point;
	auto deg_point_th = static_cast<int16_t>(radian_to_degree(point.th));
	double time = ros::Time::now().toSec();
	ROS_INFO("\033[1;40;32m deg_point_th = %d, point(%d,%d)\033[0m",deg_point_th,point.toCell().x,point.toCell().y);

	for (int16_t angle_i = 0; angle_i <360; angle_i += 2) {
		for (int dy = 0; dy < RADIUS_CELL; ++dy) {
			tmp_point.th = ranged_radian(degree_to_radian(deg_point_th + angle_i));
			Cell_t cell = tmp_point.getRelative(0, dy * CELL_SIZE).toCell();
			auto status = getCell(CLEAN_MAP,cell.x,cell.y);
			if (status > CLEANED && status < BLOCKED_BOUNDARY)
				break;
			setCell(CLEAN_MAP, cell.x, cell.y, cell_state);

			auto source_map_cell_state = slam_grid_map.getCell(CLEAN_MAP, cell.x, cell.y);
			if(source_map_cell_state != SLAM_MAP_REACHABLE)
			{
				Mark_t tmp = {cell.x,cell.y,time};
				error_maker.push_back(tmp);
			}
		}
	}
}

void GridMap::setBlockWithBound(Cell_t min, Cell_t max, CellState state,bool with_block) {
	for (auto i = min.x; i <= max.x; ++i) {
		for (auto j = min.y; j <= max.y; ++j) {
			setCell(CLEAN_MAP, i,j,state);
		}
	}
	if(with_block) {
		for (auto i = min.x - 1; i <= max.x + 1; ++i)
			setCell(CLEAN_MAP, i, max.y + 1, BLOCKED);
		for (auto i = min.x - 1; i <= max.x + 1; ++i)
			setCell(CLEAN_MAP, i, min.y - 1, BLOCKED);
		for (auto i = min.y - 1; i <= max.y + 1; ++i)
			setCell(CLEAN_MAP, max.x + 1, i, BLOCKED);
		for (auto i = min.y - 1; i <= max.y + 1; ++i)
			setCell(CLEAN_MAP, min.x - 1, i, BLOCKED);
	}

}

void GridMap::setArea(Cell_t center, CellState cell_state, uint16_t x_len, uint16_t y_len)
{
	for (auto dx = -x_len; dx <= x_len; dx++)
		for (auto dy = -y_len; dy <= y_len; dy++)
			setCell(CLEAN_MAP, static_cast<int16_t>(center.x + dx), static_cast<int16_t>(center.y + dy), cell_state);
}

