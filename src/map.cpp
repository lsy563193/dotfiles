#include <gyro.h>
#include <event_manager.h>
#include <mode.hpp>
#include <fstream>
#include "map.h"
#include "robot.hpp"
#include "event_manager.h"
#include "rcon.h"
#include "lidar.hpp"
#include <fstream>

GridMap slam_grid_map;
extern const Cell_t cell_direction_[9];

GridMap::GridMap(){
	reset(BOTH_MAP);
}

void GridMap::reset(uint8_t id)
{
	uint16_t idx;
	if (id == COST_MAP || id == BOTH_MAP) {
		for (idx = 0; idx < MAP_SIZE; idx++)
			memset((cost_map[idx]), 0, (MAP_SIZE * sizeof(uint8_t)));
	}
	if (id == CLEAN_MAP || id == BOTH_MAP) {
		for (idx = 0; idx < MAP_SIZE; idx++)
			memset((clean_map[idx]), 0, (MAP_SIZE * sizeof(uint8_t)));
		xRangeMin = xRangeMax = yRangeMin = yRangeMax = 0;
	}
}

CellState GridMap::getCell(int id, int16_t x, int16_t y)
{
	CellState val = 0;

	if (x >= xRangeMax - (MAP_SIZE - 1) && x <= xRangeMin + (MAP_SIZE - 1) &&
		y >= yRangeMax - (MAP_SIZE - 1) && y <= yRangeMin + (MAP_SIZE - 1))
	{
		x += MAP_SIZE + MAP_SIZE / 2;
		x %= MAP_SIZE;
		y += MAP_SIZE + MAP_SIZE / 2;
		y %= MAP_SIZE;

		if (id == CLEAN_MAP)
			val = (CellState) (clean_map[x][y]);
		else if (id == COST_MAP)
			val = (CellState) (cost_map[x][y]);

	} else
	{
		if (id == CLEAN_MAP)
			val = BLOCKED_BOUNDARY;
		else
			val = COST_HIGH;
	}

	return val;
}

void GridMap::setCell(uint8_t id, int16_t x, int16_t y, CellState value) {
	int16_t ROW, COLUMN;
	if(id == CLEAN_MAP) {
		if (x >= xRangeMax - (MAP_SIZE - 1) && x <= xRangeMin + (MAP_SIZE - 1) &&
			y >= yRangeMax - (MAP_SIZE - 1) && y <= yRangeMin + (MAP_SIZE - 1))
		{
			if(x < xRangeMin)
				xRangeMin = x;
			else if(x > xRangeMax)
				xRangeMax = x;

			if(y < yRangeMin)
				yRangeMin = y;
			else if(y > yRangeMax)
				yRangeMax = y;

			ROW = static_cast<int16_t>(x + MAP_SIZE + MAP_SIZE / 2);
			ROW %= MAP_SIZE;
			COLUMN = static_cast<int16_t>(y + MAP_SIZE + MAP_SIZE / 2);
			COLUMN %= MAP_SIZE;

			clean_map[ROW][COLUMN] = static_cast<uint8_t>(value);
		}
	} else if (id == COST_MAP){
		if (x >= xRangeMax - (MAP_SIZE - 1) && x <= xRangeMin + (MAP_SIZE - 1) &&
			y >= yRangeMax - (MAP_SIZE - 1) && y <= yRangeMin + (MAP_SIZE - 1))
		{
			ROW = static_cast<int16_t>(x + MAP_SIZE + MAP_SIZE / 2);
			ROW %= MAP_SIZE;
			COLUMN = static_cast<int16_t>(y + MAP_SIZE + MAP_SIZE / 2);
			COLUMN %= MAP_SIZE;

			cost_map[ROW][COLUMN] = static_cast<uint8_t>(value);
		}
	}
}

void GridMap::setCells(uint8_t id, int16_t cell_x, int16_t cell_y, CellState state, int8_t offset)
{
	int8_t i, j;

	for ( i = -offset; i <= offset; i++ )
	{
		for ( j = -offset; j <= offset; j++ )
			setCell(id,cell_x + i,cell_y + j, state);
	}
}

void GridMap::setSpecificCells(uint8_t id, int16_t cell_x, int16_t cell_y, CellState state, CellState exception_state,
							   int8_t offset)
{
	int8_t i, j;

	for ( i = -offset; i <= offset; i++ ) {
		for ( j = -offset; j <= offset; j++ ) {
			if(getCell(id, cell_x + i, cell_y + j) != exception_state)
				setCell(id, cell_x + i, cell_y + j, state);
		}
	}
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

void GridMap::convertFromSlamMap(float target_resolution,float threshold,const BoundingBox2& bound)
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
	auto multi = target_resolution / resolution;
	// Limit count for checking block.*/
	auto limit_count = static_cast<uint16_t>((multi * multi) * threshold);
	// Set boundary for this cost map.

	int16_t map_x_min = std::max(static_cast<int16_t>(origin_x  / target_resolution), bound.min.x);
	int16_t map_x_max = std::min(static_cast<int16_t>(map_x_min + width / multi), bound.max.x);
	int16_t map_y_min = std::max(static_cast<int16_t>(origin_y / target_resolution), bound.min.y);
	int16_t map_y_max = std::min(static_cast<int16_t>(map_y_min + height / multi), bound.max.y);

	ROS_INFO("%s,%d: resolution: %f, multi: %f, limit cnt:%d, map_x_min: %d, map_x_max: %d, map_y_min: %d, map_y_max: %d",
	 		   __FUNCTION__, __LINE__, resolution, multi, limit_count, map_x_min, map_x_max, map_y_min, map_y_max);
	for (auto cell_x = map_x_max; cell_x >= map_x_min; --cell_x)
	{
//		ROS_ERROR("cell:");
		for (auto cell_y = map_y_max; cell_y >= map_y_min; --cell_y)
		{
			// Get the range of this cell in the grid map of slam map data.
			double world_x, world_y;
			world_x = (double)cell_x * target_resolution;
			world_y = (double)cell_y * target_resolution;
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

//	ROS_ERROR("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
//	int16_t x, y, x_min, x_max, y_min, y_max;
//	getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);
}

void GridMap::merge(GridMap source_map, bool add_slam_map_blocks_to_uncleaned,
					bool add_slam_map_blocks_to_cleaned,
					bool add_slam_map_cleanable_area, bool clear_map_blocks, bool clear_slam_map_blocks,
					bool clear_bumper_and_lidar_and_slip_blocks)
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

			if (clear_bumper_and_lidar_and_slip_blocks &&
				(map_cell_state == BLOCKED_BUMPER || map_cell_state == BLOCKED_LIDAR ||
						map_cell_state == BLOCKED_SLIP) &&
				source_map_cell_state == SLAM_MAP_REACHABLE)
				setCell(CLEAN_MAP,x,y, CLEANED);

			if (clear_map_blocks && map_cell_state >= BLOCKED && map_cell_state < SLAM_MAP_BLOCKED
				&& map_cell_state != BLOCKED_CLIFF && source_map_cell_state == SLAM_MAP_REACHABLE)
				setCell(CLEAN_MAP,x,y, CLEANED);

			if (clear_slam_map_blocks && map_cell_state == SLAM_MAP_BLOCKED && source_map_cell_state == SLAM_MAP_REACHABLE)
				setCell(CLEAN_MAP,x,y, SLAM_MAP_REACHABLE);

			if (add_slam_map_blocks_to_uncleaned && map_cell_state == UNCLEAN && source_map_cell_state == SLAM_MAP_BLOCKED)
				setCell(CLEAN_MAP,x,y, SLAM_MAP_BLOCKED);
			if (add_slam_map_blocks_to_cleaned && map_cell_state == CLEANED && source_map_cell_state == SLAM_MAP_BLOCKED)
				setCell(CLEAN_MAP,x,y, SLAM_MAP_BLOCKED);

			if (add_slam_map_cleanable_area && map_cell_state == UNCLEAN && source_map_cell_state == SLAM_MAP_REACHABLE)
				setCell(CLEAN_MAP,x,y, CLEANED);
		}
	}
}

void GridMap::slamMapToWorld(double origin_x, double origin_y, float resolution, int16_t slam_map_x,
							 int16_t slam_map_y, double &world_x, double &world_y)
{
	world_x = origin_x + (slam_map_x + 0.5) * resolution;
	world_y = origin_y + (slam_map_y + 0.5) * resolution;
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

void GridMap::cellToWorld(double &worldX, double &worldY, int16_t &cellX, int16_t &cellY)
{
	worldX = (double)cellX * CELL_SIZE;
	worldY = (double)cellY * CELL_SIZE;
}

bool GridMap::markRobot(const Cell_t& curr, uint8_t id,bool is_clean_rcon)
{
	bool ret = false;
	std::string debug_str;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
//			robotToCell(getPosition(), CELL_SIZE * dy, CELL_SIZE * dx, x, y);
			auto x = curr.x + (int16_t)dx;
			auto y = curr.y + (int16_t)dy;
			auto status = getCell(id, x, y);
			if (status < BLOCKED_BOUNDARY /*&& (status != BLOCKED_RCON)*/){
				if(status != BLOCKED_RCON || is_clean_rcon)
				{
					debug_str += "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
					setCell(id, x, y, CLEANED);
					ret = true;
				}
			}
		}
	}
	ROS_INFO("\033[1;33m" "%s,%d: %s" "\033[0m", __FUNCTION__, __LINE__, debug_str.c_str());
	return ret;
}

uint32_t GridMap::getCleanedArea(void)
{
	uint32_t cleaned_count = 0;
	int16_t map_x_min, map_y_min, map_x_max, map_y_max;
	getMapRange(CLEAN_MAP, &map_x_min, &map_x_max, &map_y_min, &map_y_max);
	for (int16_t i = map_x_min; i <= map_x_max; ++i) {
		for (int16_t j = map_y_min; j <= map_y_max; ++j)
			cleaned_count += (getCell(CLEAN_MAP, i, j) == CLEANED) ? 1 : 0;
	}
//	ROS_INFO("cleaned_count = %d, area = %.2fm2", cleaned_count, area);
	return cleaned_count;
}

uint8_t GridMap::isABlock(int16_t x, int16_t y)
{
	uint8_t ret = 0;
	CellState cs = getCell(CLEAN_MAP, x, y);
	if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY)
		ret = 1;

	return ret;
}

uint8_t GridMap::isBlockedByBumper(int16_t x, int16_t y)
{
	uint8_t ret = 0;
	int16_t i, j;

	/* Check the point by using the robot size. */
	for (i = -ROBOT_SIZE_1_2; ret == 0 && i <= ROBOT_SIZE_1_2; i++) {
		for (j = -ROBOT_SIZE_1_2; ret == 0 && j <= ROBOT_SIZE_1_2; j++)
			ret += (getCell(CLEAN_MAP, x + i, y + j) == BLOCKED_BUMPER) ? 1 : 0;
	}

	return ret;
}

bool GridMap::isBlockAccessible(int16_t x, int16_t y)
{
	bool ret = true;
	int16_t i, j;

	for (i = -ROBOT_SIZE_1_2; ret && i <= ROBOT_SIZE_1_2; i++) {
		for (j = -ROBOT_SIZE_1_2; ret && j <= ROBOT_SIZE_1_2; j++)
			ret = isABlock(x + i, y + j) == 0;
	}

	return ret;
}

bool GridMap::isBlockCleanable(int16_t x, int16_t y)
{
	auto retval = isUncleanAtY(x, y) && !isBlocksAtY(x, y);
//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

bool GridMap::isNotBlockAndCleaned(int16_t x, int16_t y)
{
	uint8_t cleaned = 0;
	int16_t i, j;

	for (i = -ROBOT_SIZE_1_2; i <= ROBOT_SIZE_1_2; i++) {
		for (j = -ROBOT_SIZE_1_2; j <= ROBOT_SIZE_1_2; j++) {
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
	for (int16_t i = -ROBOT_SIZE_1_2; i <= ROBOT_SIZE_1_2; i++)
		unclean_cnt += (getCell(CLEAN_MAP, x, y + i) == UNCLEAN) ? 1 : 0;

//	ROS_INFO("%s, %d:unclean_cnt(%d)", __FUNCTION__, __LINE__, unclean_cnt);
	return unclean_cnt;
}

uint8_t GridMap::isBlockAtY(int block, int16_t x, int16_t y)
{
	uint8_t block_cnt = 0;

	for (int16_t i = -ROBOT_SIZE_1_2; i <= ROBOT_SIZE_1_2; i++)
		block_cnt += (getCell(CLEAN_MAP, x, y + i) == block) ? 1 : 0;
//	ROS_INFO("%s, %d:block_cnt(%d)", __FUNCTION__, __LINE__, unclean_cnt);
	return block_cnt;
}

uint8_t GridMap::isBlockBoundary(int16_t x, int16_t y)
{
	uint8_t ret = 0;
	for (int16_t i = -ROBOT_SIZE_1_2; ret == 0 && i <= ROBOT_SIZE_1_2; i++)
		ret += (getCell(CLEAN_MAP, x, y + i) == BLOCKED_BOUNDARY) ? 1 : 0;

	return ret;
}

uint8_t GridMap::isBlocksAtY(int16_t x, int16_t y)
{
	uint8_t ret = 0;

	for (int16_t i = -ROBOT_SIZE_1_2; ret == 0 && i <= ROBOT_SIZE_1_2; i++)
		ret += (isABlock(x, y + i) == 1) ? 1 : 0;

//	ROS_INFO("%s, %d:ret(%d)", __FUNCTION__, __LINE__, retval);
	return ret;
}

uint8_t GridMap::isBlockBlockedXAxis(int16_t curr_x, int16_t curr_y, bool is_left)
{
	uint8_t ret = 0;
	auto dy = is_left ? 2 : -2;
	for(auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2 && ret == 0; dx++) {
		auto cell = getPosition().getRelative(CELL_SIZE * dx, CELL_SIZE * dy).toCell();
		ret += (isABlock(cell.x, cell.y) == 1) ? 1 : 0;
	}

//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return ret;
}

int8_t GridMap::isNeedClean(int16_t x, int16_t y)
{
	uint8_t cleaned = 0;
	int16_t i, j;

	for (i = -ROBOT_SIZE_1_2; i <= ROBOT_SIZE_1_2; i++) {
		for (j = -ROBOT_SIZE_1_2; j <= ROBOT_SIZE_1_2; j++) {
			auto state = getCell(CLEAN_MAP, x + i, y + j);
			if (state == CLEANED) {
				cleaned ++;
			} else if(isABlock(x, y))
				return false;
		}
	}

	return cleaned <= 6;
}

//bool GridMap::count_if(const Cell_t &curr_cell, std::function<bool(const Cell_t &next)> compare, int& count) {
//	Cells targets{};
//	std::set<Cell_t> c_cleans;
//	find_if(getPosition().toCell(), targets, [&](Cell_t c_it) {
//		for (auto c_neight = std::begin(cell_direction_); c_neight != std::end(cell_direction_); ++c_neight) {
//			auto tmp = c_it + *c_neight;
//			if (compare(tmp))
//				c_cleans.insert(tmp);
//		}
//		return c_it.y%2 == 0 && getCell(CLEAN_MAP, c_it.x, c_it.y) == UNCLEAN  && isBlockAccessible(c_it.x, c_it.y);
//	});
//	count = c_cleans.size();
////	ROS_WARN("%s,is_trapped(%d),trapped_clean_count(%d)",__FUNCTION__,targets.empty(), count);
//	return targets.empty();
//}

//bool GridMap::dijstra(const Cell_t &curr_cell, Cells &targets,
//					  std::function<bool(const Cell_t &next)> compare_for_targets,
//					  bool stop_if_found_one, bool use_uncleaned_area)
//{
//	typedef std::multimap<int16_t, Cell_t> Queue;
//	typedef std::pair<int16_t, Cell_t> Entry;
//
//	reset(COST_MAP);
//	Queue queue;
//	setCell(COST_MAP, curr_cell.x, curr_cell.y, 1);
//	queue.emplace(1, curr_cell);
//
//	while (!queue.empty()) {
////		 Get the nearest next from the queue
//		if (queue.begin()->first == 5) {
//			Queue tmp_queue;
//			std::for_each(queue.begin(), queue.end(), [&](const Entry &iterators) {
//				tmp_queue.emplace(0, iterators.second);
//			});
//			queue.swap(tmp_queue);
//		}
//		auto start = queue.begin();
//		auto next = start->second;
//		auto cost = start->first;
//		queue.erase(start);
//		if (compare_for_targets(next))
//		{
//			targets.push_back(next);
//			if(stop_if_found_one)
//			{
//				ROS_INFO("find target(%d,%d)",next.x, next.y);
//				return true;
//			}
//		}
//
//		for (auto index = 0; index < 4; index++) {
//
//			if (cellIsOutOfRange(next) || isOutOfTargetRange(next))
////			{
////				printf("(%d, %d), %d, %d\n", next.x, next.y, cellIsOutOfRange(next),
////					   (is_target ? isOutOfTargetRange(next) : isOutOfMap(next)));
//				break;
////			}
//
//			auto neighbor = next + cell_direction_[index];
//
//			if (!use_uncleaned_area && getCell(CLEAN_MAP, neighbor.x, neighbor.y) == UNCLEAN)
//				continue;
//
//			if (getCell(COST_MAP, neighbor.x, neighbor.y) == 0 && isBlockAccessible(neighbor.x, neighbor.y))
//			{
//				queue.emplace(cost + 1, neighbor);
//				setCell(COST_MAP, neighbor.x, neighbor.y, cost + 1);
//			}
//		}
//	}
//	return !targets.empty();
//}

//bool GridMap::find_if(const Cell_t &curr_cell, Cells &targets, std::function<bool(const Cell_t &next)> compare) {
//	typedef std::multimap<int16_t, Cell_t> Queue;
//	typedef std::pair<int16_t, Cell_t> Entry;
//
//	reset(COST_MAP);
//	Queue queue;
//	setCell(COST_MAP, curr_cell.x, curr_cell.y, 1);
//	queue.emplace(1, curr_cell);
//
////	ROS_INFO("%s %d: curr(%d, %d), range(%d, %d, %d, %d), g_(%d, %d, %d, %d).", __FUNCTION__, __LINE__, curr_cell.x,
////			 curr_cell.y, xRangeMin, xRangeMax, yRangeMin, yRangeMax, g_x_min, g_x_max, g_y_min, g_y_max);
//	while (!queue.empty()) {
////		 Get the nearest next from the queue
//		if (queue.begin()->first == 5) {
//			Queue tmp_queue;
//			std::for_each(queue.begin(), queue.end(), [&](const Entry &iterators) {
//				tmp_queue.emplace(0, iterators.second);
//			});
//			queue.swap(tmp_queue);
//		}
//		auto start = queue.begin();
//		auto next = start->second;
//		auto cost = start->first;
//		queue.erase(start);
//
//		for (auto index = 0; index < 4; index++) {
//
//			if (cellIsOutOfRange(next) || isOutOfMap(next))
//				break;
//
//			auto neighbor = next + cell_direction_[index];
//
//			if (getCell(COST_MAP, neighbor.x, neighbor.y) == 0) {
//				compare(neighbor);
//				if (isBlockAccessible(neighbor.x, neighbor.y))
//				{
//
//					if (compare(next))
//					{
//						targets.push_back(next);
//					}
//					queue.emplace(cost + 1, neighbor);
//					setCell(COST_MAP, neighbor.x, neighbor.y, cost + 1);
//				}
//			}
//		}
//	}
//	return !targets.empty();
//}

//bool GridMap::dijkstra(const Cell_t &curr_cell, Cells &targets, bool greedy_match,
//					   std::function<bool(const Cell_t &tmp_target)> targets_selection,
//					   std::function<bool(const Cell_t &tmp_target, const Cell_t &neighbour)> expand_condition)
//{
//	typedef std::multimap<int16_t, Cell_t> Queue;
//	typedef std::pair<int16_t, Cell_t> Entry;
//
//	reset(COST_MAP);
//	Queue queue;
//	setCell(COST_MAP, curr_cell.x, curr_cell.y, 1);
//	queue.emplace(1, curr_cell);
//
////	ROS_INFO("%s %d: curr(%d, %d), range(%d, %d, %d, %d), g_(%d, %d, %d, %d).", __FUNCTION__, __LINE__, curr_cell.x,
////			 curr_cell.y, xRangeMin, xRangeMax, yRangeMin, yRangeMax, g_x_min, g_x_max, g_y_min, g_y_max);
//	while (!queue.empty()) {
////		 Get the nearest next from the queue
//		if (queue.begin()->first == 5) {
//			Queue tmp_queue;
//			std::for_each(queue.begin(), queue.end(), [&](const Entry &iterators) {
//				tmp_queue.emplace(0, iterators.second);
//			});
//			queue.swap(tmp_queue);
//		}
//		auto start = queue.begin();
//		auto next = start->second;
//		auto cost = start->first;
//		queue.erase(start);
//		if (targets_selection(next))
//		{
//			targets.push_back(next);
//			if(!greedy_match)
//			{
//				ROS_INFO("find target(%d,%d)",next.x, next.y);
//				return true;
//			}
//		}
//
//		for (auto index = 0; index < 4; index++)
//		{
//			auto neighbor = next + cell_direction_[index];
//			if (expand_condition(next, neighbor))
//			{
//				queue.emplace(cost + 1, neighbor);
//				setCell(COST_MAP, neighbor.x, neighbor.y, cost + 1);
//			}
//		}
//	}
//	return !targets.empty();
//}

//bool GridMap::isAccessibleNeighbor(Cell_t neighbor_cell)
//{
//	// Do not care weather it is cleaned or not.
//	return !cellIsOutOfRange(neighbor_cell) && !isOutOfMap(neighbor_cell)
//		   && isBlockAccessible(neighbor_cell.x, neighbor_cell.y);
//}
//
//bool GridMap::isAccessibleCleanedNeighbor(Cell_t neighbor_cell)
//{
//	return isAccessibleNeighbor(neighbor_cell) && getCell(CLEAN_MAP, neighbor_cell.x, neighbor_cell.y) == CLEANED;
//}

bool GridMap::isFrontBlockBoundary(int dx)
{
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
//		auto x_range = xRangeMax - xRangeMin;
//		auto y_range = yRangeMax - yRangeMin;
//		*x_range_min = static_cast<int16_t>(xRangeMin - ((x_range <= 3) ? 3 : 1));
//		*x_range_max = static_cast<int16_t>(xRangeMax + ((x_range <= 3) ? 3 : 1));
//		*y_range_min = static_cast<int16_t>(yRangeMin - ((y_range <= 3) ? 3 : 1));
//		*y_range_max = static_cast<int16_t>(yRangeMax + ((y_range <= 3) ? 3 : 1));
		*x_range_min = xRangeMin;
		*x_range_max = xRangeMax;
		*y_range_min = yRangeMin;
		*y_range_max = yRangeMax;
	}
//	ROS_INFO("Get Range:min(%d,%d),max(%d,%d)", g_x_min,g_y_min, g_x_max,  g_y_max);
}

bool GridMap::cellIsOutOfTargetRange(Cell_t cell)
{
	BoundingBox2 bound = genTargetRange();
	return cell.x < bound.min.x || cell.y < bound.min.y || cell.x > bound.max.x || cell.y > bound.max.y;
}
BoundingBox2 GridMap::genTargetRange()
{
	int16_t map_x_min, map_y_min, map_x_max, map_y_max;
	getMapRange(CLEAN_MAP, &map_x_min, &map_x_max, &map_y_min, &map_y_max);
	map_x_min -= 1;
	map_x_max += 1;
	map_y_min -= 1;
	map_y_max += 1;
	auto x_min_limit = static_cast<int16_t>(xRangeMax - (MAP_SIZE - 2));
	if (map_x_min < x_min_limit)
		map_x_min = x_min_limit;
	auto x_max_limit = static_cast<int16_t>(xRangeMin + (MAP_SIZE - 2));
	if (map_x_max > x_max_limit)
		map_x_max = x_max_limit;
	auto y_min_limit = static_cast<int16_t>(yRangeMax - (MAP_SIZE - 2));
	if (map_y_min < y_min_limit)
		map_y_min = y_min_limit;
	auto y_may_limit = static_cast<int16_t>(yRangeMin + (MAP_SIZE - 2));
	if (map_y_max > y_may_limit)
		map_y_max = y_may_limit;

//	ROS_INFO("%s %d: min(%d, %d), max(%d, %d).", __FUNCTION__, __LINE__, map_x_min, map_y_min, map_x_max, map_y_max);
	return {Cell_t{map_x_min, map_y_min}, Cell_t{map_x_max, map_y_max}};
}

bool GridMap::pointIsPointingOutOfTargetRange(Point_t point)
{
	auto angle = radian_to_degree(point.th);
//	printf("%s %d: angle:%f\n.", __FUNCTION__, __LINE__, angle);
	BoundingBox2 bound = genTargetRange();
	if (point.toCell().x > bound.max.x && angle > -90 && angle < 90)
		return true;
	else if (point.toCell().x < bound.min.x && (angle > 90 || angle < -90))
		return true;
	else if (point.toCell().y < bound.min.y && angle < 0)
		return true;
	else if (point.toCell().y > bound.max.y && angle > 0)
		return true;

	return false;
}

void GridMap::cellPreventOutOfRange(Cell_t &cell)
{
	BoundingBox2 bound = genTargetRange();
	if (cell.x < bound.min.x)
		cell.x = bound.min.x;
	if (cell.x > bound.max.x)
		cell.x = bound.max.x;
	if (cell.y < bound.min.y)
		cell.y = bound.min.y;
	if (cell.y > bound.max.y)
		cell.y = bound.max.y;
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
    if(id == COST_MAP)
	{
		x_min -= 1;x_max += 1; y_min -= 1; y_max +=1;
	}
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
			else if (cs == BLOCKED_BOUNDARY)
				outString << 'b';
			else
				outString << cs;

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

void GridMap::printInRange(const Cell_t& curr_cell, uint8_t id, const Cells& targets,bool is_bound,BoundingBox2 bound)
{
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
			else if (std::find_if(targets.begin(), targets.end(), [&](const Cell_t& c_it ){return c_it == Cell_t{x,y};}) != targets.end())
				outString << 'e';
			else if (val == 0) {
				outString << '0';
			} else if (val == 1) {
				outString << '1';
			} else  {
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
				y_col+="\033[1;44;37mb\033[0m";// blue
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
			cell = getPosition().getRelative(d_cell.x * CELL_SIZE, d_cell.y * CELL_SIZE).toCell();
		else
			cell = getPosition().toCell() + d_cell;

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
	ROS_INFO("\033[1;40;32mdeg_point_th = %d, point(%d,%d)\033[0m",deg_point_th,point.toCell().x,point.toCell().y);

	for (int16_t angle_i = 0; angle_i <360; angle_i += 3) {
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
	for (int16_t i = min.x; i <= max.x; ++i) {
		for (int16_t j = min.y; j <= max.y; ++j) {
			setCell(CLEAN_MAP, i,j,state);
		}
	}
	if(with_block) {
		for (int16_t i = min.x - 1; i <= max.x + 1; ++i)
			setCell(CLEAN_MAP, i, max.y + 1, BLOCKED);
		for (int16_t i = min.x - 1; i <= max.x + 1; ++i)
			setCell(CLEAN_MAP, i, min.y - 1, BLOCKED);
		for (int16_t i = min.y - 1; i <= max.y + 1; ++i)
			setCell(CLEAN_MAP, max.x + 1, i, BLOCKED);
		for (int16_t i = min.y - 1; i <= max.y + 1; ++i)
			setCell(CLEAN_MAP, min.x - 1, i, BLOCKED);
	}

}

void GridMap::setArea(Cell_t center, CellState cell_state, uint16_t x_len, uint16_t y_len)
{
	for (auto dx = -x_len; dx <= x_len; dx++)
		for (auto dy = -y_len; dy <= y_len; dy++)
			setCell(CLEAN_MAP, static_cast<int16_t>(center.x + dx), static_cast<int16_t>(center.y + dy), cell_state);
}

void GridMap::loadMap(int16_t x_min, int16_t x_max, int16_t y_min, int16_t y_max)
{
	std::string map_file = "/opt/ros/indigo/share/pp/map";
	if (access(map_file.c_str(), F_OK) == -1)
		// If file does not exist, return.
		return;

	FILE *f_read = fopen(map_file.c_str(), "r");
	if (f_read == nullptr)
		ROS_ERROR("%s %d: Open %s error.", __FUNCTION__, __LINE__, map_file.c_str());
	else
	{
		for (int x = x_min; x <= x_max; x++)
		{
			for (int y = y_min; y <= y_max; y++)
			{
				CellState cell_state;
				if (fscanf(f_read, "%1d", &cell_state) == 1)
				{
					setCell(CLEAN_MAP, x, y, cell_state);
//					printf("(%d, %d) %d\n", x, y, cell_state);
				}
			}
		}
		Cell_t curr{0, 0};
		print(curr, CLEAN_MAP, Cells{});
		fclose(f_read);
		ROS_INFO("%s %d: Read data succeeded.", __FUNCTION__, __LINE__);
	}
}
void GridMap::loadMap(bool use_map,Cell_t& curr,Dir_t& dir, bool& trend_pos)
{
	using namespace std;
	std::string map_file = "/opt/ros/indigo/share/pp/map";
	std::ifstream fin(map_file);
	if(!fin.is_open())
	{
		ROS_ERROR("Open false");
		return;
	}

	std::string s;;
	getline(fin, s);
	std::cout << s << endl;
	istringstream iss(s);
	string word;
	Cell_t min_p;
	iss >> word;
	min_p.x = static_cast<int16_t>(std::stoi(word));
	iss >> word;
	min_p.y = static_cast<int16_t>(std::stoi(word));
	iss >> word;
	dir = static_cast<Dir_t >(std::stoi(word));
	iss >> word;
	trend_pos = static_cast<bool>(std::stoi(word));
	ROS_INFO("map_origin:curr(%d,%d),min_p(%d,%d),dir(%d)",curr.x, curr.y,min_p.x, min_p.y,dir);

	auto m_begin = fin.tellg();
	getline(fin,s);
	int16_t width = s.size();
	ROS_INFO("width:(%d)",width);

	fin.seekg(m_begin);
	char x;
	if(use_map) {
		while (!fin.eof()) {
			fin.get(x);
			if (x == 'x') {
				auto sp = fin.tellg()-m_begin;
				cout << "sp: " << sp <<endl;
				curr =  Cell_t{static_cast<int16_t>(sp / (width +1)),
											static_cast<int16_t>(sp % (width +1)-1)};
				ROS_INFO("map_origin:curr(%d,%d),min_p(%d,%d)",curr.x, curr.y,min_p.x, min_p.y);
				curr +=	min_p;
				ROS_INFO("map_offset:curr(%d,%d)",curr.x, curr.y);
				break;
			}
		}
	}

	fin.seekg(m_begin);
	while(!fin.eof()){
		auto val = fin.get();
		if(val =='x')
		{
			setCell(CLEAN_MAP, curr.x, curr.y, 1);
		}else
		if(val !='\n'&& val !=-1)
		{
			auto sp = fin.tellg()-m_begin;
//			cout << "sp: " << sp <<endl;
			Cell_t c_it = min_p + Cell_t{static_cast<int16_t>(sp / (width + 1)),
																	 static_cast<int16_t>(sp % (width + 1)-1)};
			setCell(CLEAN_MAP, c_it.x, c_it.y, val-'0');
		}
	}
	ROS_ERROR("33332");
	fin.close();
}

BoundingBox2 GridMap::generateBound()
{
	int16_t map_x_min, map_y_min, map_x_max, map_y_max;
	getMapRange(CLEAN_MAP, &map_x_min, &map_x_max, &map_y_min, &map_y_max);
	BoundingBox2 bound{{map_x_min, map_y_min},
					   {map_x_max, map_y_max}};
	return bound;
}

Cells GridMap::generateCellsBetweenTwoCells(const Cell_t &start, const Cell_t &end, bool &is_x_direction)
{
	Cells cells_path;
	int8_t x_direction;
	int8_t y_direction;
	uint16_t x_diff = static_cast<uint16_t>(abs(end.x - start.x));
	uint16_t y_diff = static_cast<uint16_t>(abs(end.y - start.y));
	is_x_direction = x_diff >= y_diff;
	x_direction = static_cast<int8_t>(end.x >= start.x ? 1 : -1);
	y_direction = static_cast<int8_t>(end.y >= start.y ? 1 : -1);

	Cell_t cell_step{start.x, start.y};

	if (is_x_direction)
	{
		for (;abs(cell_step.x - end.x) > 0;)
		{
			cells_path.push_back(cell_step);
			cell_step.x += x_direction;
			auto y_scale = floor(y_diff * fabs(cell_step.x - start.x) / x_diff);
			cell_step.y = static_cast<int16_t>(start.y + y_direction * y_scale);
//			setCell(CLEAN_MAP, cell_step.x, cell_step.y, 1);
//			ROS_INFO("%s %d: cell_step(%d, %d).", __FUNCTION__, __LINE__, cell_step.x, cell_step.y);
		}
	} else{
		for (;abs(cell_step.y - end.y) > 0;)
		{
			cells_path.push_back(cell_step);
			cell_step.y += y_direction;
			auto x_scale = floor(x_diff * fabs(cell_step.y - start.y) / y_diff);
			cell_step.x = static_cast<int16_t>(start.x + x_direction * x_scale);
//			setCell(CLEAN_MAP, cell_step.x, cell_step.y, 1);
//			ROS_INFO("%s %d: cell_step(%d, %d).", __FUNCTION__, __LINE__, cell_step.x, cell_step.y);
		}
	}

	cells_path.push_back(end);

	return cells_path;
}

