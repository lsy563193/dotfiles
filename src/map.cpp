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
#include <move_type.h>
#include <regulator.h>
#include <mathematics.h>
#include <space_exploration.h>
#include <clean_state.h>

#include "map.h"
#include "mathematics.h"
#include "robot.hpp"
#include "clean_mode.h"

#define DEBUG_MSG_SIZE	1 // 20
CostMap cost_map;
CostMap fw_map;
CostMap ros_map;
CostMap decrease_map;

SlamMap slam_map;

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

double CostMap::xCount = 0;
double CostMap::yCount = 0;

CostMap::CostMap() {
		for(auto c = 0; c < MAP_SIZE; ++c) {
			for(auto d = 0; d < (MAP_SIZE + 1) / 2; ++d) {
				costmap[c][d] = 0;
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
/*
int16_t CostMap::get_estimated_room_size(void) {
	int16_t i, j;

	i = g_x_max - g_x_min;
	j = g_y_max - g_y_min;

	if(i < j) {
		i = g_y_max - g_y_min;
		j = g_x_max - g_x_min;
	}

	if(i * 2 > j * 3) {
		i = (i * i) / 27;				//Convert no of cell to tenth m^2
	} else {
		i = (i * j) / 18;				//Convert no of cell to tenth m^2
	}

	return i;
}
*/

int32_t CostMap::get_x_count(void) {
	return (int32_t)round(xCount);
}

int32_t CostMap::get_y_count(void) {
	return (int32_t)round(yCount);
}

Point32_t CostMap::get_curr_point(void)
{
	return {get_x_count(), get_y_count(),gyro.get_angle()};
}

int16_t CostMap::get_x_cell(void) {
	return count_to_cell(xCount);
}

int16_t CostMap::get_y_cell(void) {
	return count_to_cell(yCount);
}

Cell_t CostMap::get_curr_cell()
{
	return Cell_t{get_x_cell(), CostMap::get_y_cell(),gyro.get_angle()};
}

void CostMap::set_position(double x, double y) {
	xCount = x;
	yCount = y;
}

/*
 * CostMap::get_cell description
 * @param id	Map id
 * @param x	Cell x
 * @param y	Cell y
 * @return	CellState
 */
CellState CostMap::get_cell(int id,int16_t x, int16_t y) {
	CellState val;
	int16_t x_min, x_max, y_min, y_max;
	if (id == MAP || id == SPMAP) {
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
		//val = (CellState)((id == MAP) ? (costmap[x][y / 2]) : (spmap[x][y / 2]));
		if(id == MAP)
			val = (CellState)(costmap[x][y / 2]);
		else if(id == SPMAP)
			val = (CellState)(spmap[x][y / 2]);
#else
		//val = (CellState)(costmap[x][y / 2]);
		if (id == MAP) {
			val = (CellState)(costmap[x][y / 2]);
		}
#endif

		/* Upper 4 bits & lower 4 bits. */
		val = (CellState) ((y % 2) == 0 ? (val >> 4) : (val & 0x0F));

	} else {
		if(id == MAP) {
			val = BLOCKED_BOUNDARY;
		} else {
			val = COST_HIGH;
		}
	}

	return val;
}

/*
 * CostMap::set_cell description
 * @param id		Map id
 * @param x		 Count x
 * @param y		 Count y
 * @param value CellState
 */
void CostMap::set_cell(uint8_t id, int32_t x, int32_t y, CellState value) {
	CellState val;
	int16_t ROW, COLUMN;

	if(id == MAP) {
		if(value == CLEANED) {
			ROW = cell_to_count(count_to_cell(x)) - x;
			COLUMN = cell_to_count(count_to_cell(y)) - y;

			if(abs(ROW) > (CELL_SIZE - 2 * 20) * CELL_COUNT_MUL / (2 * CELL_SIZE) || abs(COLUMN) > (CELL_SIZE - 2 * 20) * CELL_COUNT_MUL / (2 * CELL_SIZE)) {
				//return;
			}
		}

		x = count_to_cell(x);
		y = count_to_cell(y);
	}

	if(id == MAP) {
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

			val = (CellState) costmap[ROW][COLUMN / 2];
			if (((COLUMN % 2) == 0 ? (val >> 4) : (val & 0x0F)) != value) {
				costmap[ROW][COLUMN / 2] = ((COLUMN % 2) == 0 ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
			}
		}
	}  else if (id == SPMAP){
		if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
			x += MAP_SIZE + MAP_SIZE / 2;
			x %= MAP_SIZE;
			y += MAP_SIZE + MAP_SIZE / 2;
			y %= MAP_SIZE;

			val = (CellState) spmap[x][y / 2];

			/* Upper 4 bits and last 4 bits. */
			spmap[x][y / 2] = (((y % 2) == 0) ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
		}
	}
}

void CostMap::clear_blocks(void) {
	int16_t c, d;

	for(c = g_x_min; c < g_x_max; ++c) {
		for(d = g_y_min; d < g_y_max; ++d) {
			CellState state = get_cell(MAP, c, d);
			if(state == BLOCKED_LASER || state == BLOCKED_BUMPER ||	state == BLOCKED_CLIFF || state  == BLOCKED_OBS) {
				if(get_cell(MAP, c - 1, d) != UNCLEAN && get_cell(MAP, c, d + 1) != UNCLEAN &&
								get_cell(MAP, c + 1, d) != UNCLEAN &&
								get_cell(MAP, c, d - 1) != UNCLEAN) {
					set_cell(MAP, cell_to_count(c), cell_to_count(d), CLEANED);
				}
			}
		}
	}

	set_cell(MAP, cell_to_count(get_x_cell() - 1), cell_to_count(get_y_cell()), CLEANED);
	set_cell(MAP, cell_to_count(get_x_cell()), cell_to_count(get_y_cell() + 1), CLEANED);
	set_cell(MAP, cell_to_count(get_x_cell() + 1), cell_to_count(get_y_cell()), CLEANED);
	set_cell(MAP, cell_to_count(get_x_cell()), cell_to_count(get_y_cell() - 1), CLEANED);
}

Point32_t CostMap::get_relative(Point32_t point, int16_t dy, int16_t dx, bool using_point_pos) {
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
		point.X = cell_to_count(count_to_cell(point.X)) + (int32_t)( ( ((double)dx * relative_cos * CELL_COUNT_MUL) - ((double)dy	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
		point.Y = cell_to_count(count_to_cell(point.Y)) + (int32_t)( ( ((double)dx * relative_sin * CELL_COUNT_MUL) + ((double)dy	* relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	}
	return point;
}

void CostMap::robot_to_point(Point32_t point, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	auto relative_point = get_relative(point, offset_lat, offset_long, true);
	*x = cell_to_count(count_to_cell(relative_point.X));
	*y = cell_to_count(count_to_cell(relative_point.Y));
}

void CostMap::robot_to_cell(Point32_t point, int16_t offset_lat, int16_t offset_long, int16_t &x, int16_t &y)
{
	auto relative_point = get_relative(point, offset_lat, offset_long, false);
	x = count_to_cell(relative_point.X);
	y = count_to_cell(relative_point.Y);
}

int32_t CostMap::cell_to_count(int16_t i) {
	return i * CELL_COUNT_MUL;
}

int16_t CostMap::count_to_cell(int32_t count) {
	if(count < -CELL_COUNT_MUL_1_2) {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL - 1;
	} else {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL;
	}
}

Point32_t CostMap::cell_to_point(const Cell_t& cell) {
	Point32_t pnt;
	pnt.X = cell_to_count(cell.X);
	pnt.Y = cell_to_count(cell.Y);
	return pnt;
}

Cell_t CostMap::point_to_cell(Point32_t pnt) {
	Cell_t cell;
	cell.X = count_to_cell(pnt.X);
	cell.Y = count_to_cell(pnt.Y);
	cell.TH = gyro.get_angle();
	return cell;
}

void CostMap::set_cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state)
{
	int8_t i, j;

	for ( i = -(count / 2); i <= count / 2; i++ ) {
		for ( j = -(count / 2); j <= count / 2; j++ ) {
			set_cell(MAP, cell_to_count(cell_x + i), cell_to_count(cell_y + j), state);
		}
	}
}

void CostMap::reset(uint8_t id)
{
#ifndef SHORTEST_PATH_V2
	uint16_t idx;
	if (id == SPMAP) {
		for (idx = 0; idx < MAP_SIZE; idx++) {
			memset((spmap[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
		}
	} else if (id == MAP) {
		for (idx = 0; idx < MAP_SIZE; idx++) {
			memset((costmap[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
		}
	}
#endif
}

void CostMap::ros_convert(int16_t id, bool is_mark_cleaned,bool is_clear_false_block, bool is_freshen_map,int limit, SlamMap* slam_map_)
{
	std::vector<int> data_index;
	std::vector<int8_t> p_map_data;
	unsigned int map_x,map_y,map_x_min,map_x_max,map_y_min,map_y_max;
	double world_x, world_y;
	int32_t count_x, count_y;
	int16_t cell_x, cell_y;
	int size;
	/*---values for counting sum of difference rosmap cells---*/
	uint32_t occupied_counter = 0, free_counter = 0, unknown_counter = 0;

	slam_map_mutex.lock();
	auto width = slam_map_->getWidth();
	auto height = slam_map_->getHeight();
	auto resolution = slam_map_->getResolution();
	auto origin_x = slam_map_->getOriginX();
	auto origin_y = slam_map_->getOriginY();
	p_map_data = slam_map_->getData();

	/*---side length multi between cell and rosmap cell---*/
	auto multi = CELL_SIZE / (uint32_t)(resolution * 1000.0);
	/*----limit count for checking blocked---*/
	auto limit_count = (multi * multi) * limit / 100;
	/*---work out the boundary---*/
	auto cell_x_min = (int16_t)(origin_x * 1000.0 / CELL_SIZE);
	auto cell_x_max = cell_x_min + (int16_t)(width / multi);
	auto cell_y_min = (int16_t)(origin_y * 1000.0 / CELL_SIZE);
	auto cell_y_max = cell_y_min + (int16_t)(height / multi);

	ROS_INFO("cell_x_min: %d, cell_x_max: %d, cell_y_min: %d, cell_y_max: %d", cell_x_min, cell_x_max, cell_y_min, cell_y_max);
	for (auto cell_x = cell_x_min; cell_x <= cell_x_max; ++cell_x)
	{
		for (auto cell_y = cell_y_min; cell_y <= cell_y_max; ++cell_y)
		{
			auto status = get_cell(id, cell_x, cell_y);
			count_x = cell_to_count(cell_x);
			count_y = cell_to_count(cell_y);
			countToWorld(world_x, world_y, count_x, count_y);
			worldToMap(origin_x, origin_y, resolution, width, height, world_x, world_y, map_x, map_y);

			map_x_min = (map_x > multi/2) ? (map_x - multi/2) : 0;
			map_x_max = ((map_x + multi/2) < width) ? (map_x + multi/2) : width;
			map_y_min = (map_y > multi/2) ? (map_y - multi/2) : 0;
			map_y_max = ((map_y + multi/2) < height) ? (map_y + multi/2) : height;

			for (int i=map_x_min; i<=map_x_max; i++)
				for(int j=map_y_min; j<=map_y_max; j++)
					data_index.push_back(getIndex(width, i, j));

			size = data_index.size();
			for(int index=0; index<size; index++)
			{
				if(p_map_data[data_index[index]] == 100)		occupied_counter++;
				else if(p_map_data[data_index[index]] == -1)	unknown_counter++;
				else if(p_map_data[data_index[index]] == 0)	free_counter++;
			}

			if(unknown_counter == size)/*---unknown cell---*/
			{
			//	ROS_INFO("unknown cell");
			}
			else if(occupied_counter > limit_count)/*---occupied cell---*/
			{
			//	ROS_INFO("---occupied cell---");
				if (is_freshen_map)
					if((status < BLOCKED || status > BLOCKED_BOUNDARY) && (status != BLOCKED_ROS_MAP))
						set_cell(id, count_x, count_y, BLOCKED_ROS_MAP);
				else
					set_cell(id, count_x, count_y, BLOCKED_ROS_MAP);
			}
			else/*---free cell---*/
			{
			//	ROS_INFO("---free cell---");
				if(is_mark_cleaned && status <= 1)
					set_cell(id, count_x, count_y, CLEANED);
				if(is_clear_false_block && (status < BLOCKED || status > BLOCKED_BOUNDARY))
					set_cell(id, count_x, count_y, CLEANED);
				/*freshen the map*/
				if (is_freshen_map && (status == BLOCKED_ROS_MAP))
					set_cell(id, count_x, count_y, CLEANED);
			}
			/*---clear data_index and counter for next loop---*/
			data_index.clear();
			occupied_counter = 0;
			free_counter = 0;
			unknown_counter = 0;
		}
	}
	slam_map_mutex.unlock();
}

void CostMap::to_world(double origin_x_, double origin_y_, float resolution_, unsigned int mx, unsigned int my, double &wx,
									double &wy)
{
	wx = origin_x_ + (mx + 0.5) * resolution_;
	wy = origin_y_ + (my + 0.5) * resolution_;
//wx = origin_x_ + (mx) * resolution_;
//wy = origin_y_ + (my) * resolution_;
}

bool CostMap::worldToMap(double origin_x_, double origin_y_, float resolution_, int size_x_, int size_y_, double wx,
										double wy, unsigned int &mx, unsigned int &my)
{
	if (wx < origin_x_ || wy < origin_y_)
		return false;

	mx = (int)((wx - origin_x_) / resolution_);
	my = (int)((wy - origin_y_) / resolution_);

	if (mx < size_x_ && my < size_y_)
		return true;

	return false;
}

unsigned int CostMap::getIndex(int size_x_, unsigned int mx, unsigned int my)
{
	return my * size_x_ + mx;
}

void CostMap::indexToCells(int size_x_, unsigned int index, unsigned int &mx, unsigned int &my)
{
	my = index / size_x_;
	mx = index - (my * size_x_);
}

bool CostMap::worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy)
{
	auto count_x = wx * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto count_y = wy * 1000 * CELL_COUNT_MUL / CELL_SIZE;
//cx = count_to_cell(count_x);
	cx = count_x;
//cy = count_to_cell(count_y);
	cy = count_y;
	return true;
}

bool CostMap::countToWorld(double &wx, double &wy, int32_t &cx, int32_t &cy)
{
	wx = (double)cx * CELL_SIZE / CELL_COUNT_MUL / 1000.0;
	wy = (double)cy * CELL_SIZE / CELL_COUNT_MUL / 1000.0;
	return true;
}

uint8_t CostMap::set_laser()
{
#if LASER_MARKER
	//MotionManage::s_laser->laserMarker(true);
#endif
}

uint8_t CostMap::set_obs()
{
	if (temp_obs_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_obs_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y), BLOCKED_OBS);
		block_count++;
	}
	temp_obs_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), mark \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return block_count;
}

uint8_t CostMap::set_bumper()
{
	if (temp_bumper_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_bumper_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y), BLOCKED_BUMPER);
		block_count++;
	}
	temp_bumper_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), mark \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return block_count;
}

uint8_t CostMap::set_tilt()
{
	if (temp_tilt_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_tilt_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y), BLOCKED_TILT);
		block_count++;
	}
	temp_tilt_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark %s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return block_count;
}

uint8_t CostMap::set_slip()
{
	if (temp_slip_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_slip_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y), BLOCKED_SLIP);
		block_count++;
	}
	temp_slip_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark %s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return block_count;
}

uint8_t CostMap::set_cliff()
{
	if (temp_cliff_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_cliff_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y), BLOCKED_CLIFF);
		block_count++;
	}
	temp_cliff_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark %s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return block_count;
}

uint8_t CostMap::set_rcon()
{
	if (temp_rcon_cells.empty())
		return 0;

	uint8_t block_count = 0;
	std::string msg = "cell:";
	for(auto& cell : temp_rcon_cells){
		msg += "(" + std::to_string(cell.X) + "," + std::to_string(cell.Y) + ")";
		set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y), BLOCKED_CLIFF);
		block_count++;
	}
	temp_rcon_cells.clear();
	ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark %s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return block_count;
}

uint8_t CostMap::set_charge_position(const Cell_t home_point)
{
	int ltx=-2;
	int lty=-2;
	g_stub_cell.X =  home_point.X;
	g_stub_cell.Y =  home_point.Y;
	ROS_INFO("%s,%d: set g_stub_cell(%d,%d)",__FUNCTION__,__LINE__,g_stub_cell.X,g_stub_cell.Y);
	int32_t x,y;
	std::string print_msg("");
	for(int i=0;i<4;i++){//hight
		for(int j = 0;j<5;j++){//width
			robot_to_point(get_curr_point(),CELL_SIZE*(lty+j+home_point.Y),CELL_SIZE*(ltx+i+home_point.X),&x,&y);
			set_cell(MAP,x,y,BLOCKED_RCON);
			print_msg+="("+std::to_string(count_to_cell(x))+","+std::to_string(count_to_cell(y))+"),";
		}
	}
	ROS_INFO("%s,%d: set charge stub area:%s",__FUNCTION__,__LINE__,print_msg.c_str());
	return 0;
}

uint8_t CostMap::set_follow_wall()
{

	uint8_t block_count = 0;
	if (!g_passed_path.empty())
	{
		std::string msg = "cell:";
		Cell_t block_cell;
		auto dy = mt.is_left() ? 2 : -2;
		for(auto& cell : g_passed_path){
			robot_to_cell(cell_to_point(cell), dy * CELL_SIZE, 0, block_cell.X, block_cell.Y);
			msg += "(" + std::to_string(block_cell.X) + "," + std::to_string(block_cell.Y) + ")";
			set_cell(MAP, cell_to_count(block_cell.X), cell_to_count(block_cell.Y), BLOCKED_CLIFF);
			block_count++;
		}
		ROS_INFO("%s,%d: Current(%d, %d), \033[32m mark MAP %s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	}
}

uint8_t CostMap::save_slip()
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
		robot_to_cell(get_curr_point(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//cm_world_to_point(gyro_get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, gyro.get_angle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_slip_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t CostMap::save_tilt()
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
		robot_to_cell(get_curr_point(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//cm_world_to_point(gyro_get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, gyro.get_angle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_tilt_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t CostMap::save_obs()
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
			cm_world_to_cell(gyro.get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
			//cm_world_to_point(gyro.get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
			robot_to_cell(gyro_get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
			//robot_to_point(gyro_get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
			//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
			//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, gyro.get_angle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
			temp_obs_cells.push_back({x, y});
			msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
		}
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());*/
}

uint8_t CostMap::save_cliff()
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
		robot_to_cell(get_curr_point(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//robot_to_point(gyro.get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, gyro_get_angle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_cliff_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t CostMap::save_bumper()
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
		robot_to_cell(get_curr_point(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		//robot_to_point(gyro.get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, gyro.get_angle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_bumper_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t CostMap::save_rcon()
{
	auto rcon_trig = ev.rcon_triggered/*rcon_get_trig()*/;
	if(! rcon_trig)
		return 0;
	if( g_from_station && g_in_charge_signal_range && cs.is_going_home())//while in cs.is_going_home() mode_ or from_station dont mark rcon signal
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
		//robot_to_point(gyro.get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x2, &y2);
		//ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
		//			, __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, gyro.get_angle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
		temp_rcon_cells.push_back({x, y});
		msg += "[" + std::to_string(d_cell.X) + "," + std::to_string(d_cell.Y) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	}
	ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
	return static_cast<uint8_t >(d_cells.size());
}

uint8_t CostMap::save_follow_wall()
{
	bool should_save_for_MAP = !(cm_is_navigation() && mt.is_follow_wall() && world_distance() < 0.1);

	auto dy = mt.is_left() ? 2 : -2;
	int16_t x, y;
	//int32_t	x2, y2;
	std::string msg = "cell:";
	robot_to_cell(get_curr_point(), dy * CELL_SIZE, 0, x, y);
	//robot_to_point(gyro.get_angle(), dy * CELL_SIZE, 0, &x2, &y2);
	//ROS_WARN("%s %d: d_cell(0, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
	//			, __FUNCTION__, __LINE__, dy, gyro_get_angle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
	if (should_save_for_MAP)
		temp_fw_cells.push_back({x, y});
	temp_WFMAP_follow_wall_cells.push_back({x, y});
	msg += "[0," + std::to_string(dy) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	//ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());

	return 1;
}

uint8_t CostMap::save_blocks()
{
	uint8_t block_count = 0;
	block_count += save_bumper();
	block_count += save_cliff();
	block_count += save_obs();
	block_count += save_rcon();
	block_count += save_slip();
//	block_count += save_follow_wall();

	return block_count;
}

//uint8_t CostMap::set_blocks()
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
//	block_count += set_laser();
//
//	return block_count;
//}

double world_distance(void) {
	auto dis = sqrtf(powf(RegulatorBase::s_curr_p.X - RegulatorBase::s_origin_p.X, 2) + powf(RegulatorBase::s_curr_p.Y - RegulatorBase::s_origin_p.Y, 2));

//	auto dis =  two_points_distance(RegulatorBase::s_origin_p.X, RegulatorBase::s_origin_p.Y , \
//														RegulatorBase::s_curr_p.X, RegulatorBase::s_curr_p.Y);
	return dis*CELL_SIZE/CELL_COUNT_MUL/1000;
}

void CostMap::set_cleaned(std::deque<Cell_t>& cells)
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
			auto status = get_cell(MAP, cell.X, y);
			if (status != BLOCKED_TILT && status != BLOCKED_SLIP)
			{
				set_cell(MAP, cell_to_count(cell.X), cell_to_count(y), CLEANED);
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
			//robot_to_point(gyro.get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			auto status = get_cell(MAP, get_x_cell() + dx, get_y_cell() + dy);
			if (status == UNCLEAN){
				set_cell(MAP, cell_to_count(get_x_cell() + dx), cell_to_count(get_y_cell() + dy), CLEANED);
				msg += "(" + std::to_string(get_x_cell() + dx) + "," + std::to_string(get_y_cell() + dy) + "),";
			}
		}
	}
	ROS_INFO("%s,%d:""\033[32m %s\033[0m",__FUNCTION__, __LINE__, msg.c_str());
}

bool CostMap::mark_robot(uint8_t id)
{
	if(!cs.is_trapped())
		return false;
	int32_t x, y;
	bool ret = false;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
			robot_to_point(get_curr_point(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			auto status = get_cell(id, count_to_cell(x), count_to_cell(y));
			if (status > CLEANED && status < BLOCKED_BOUNDARY && (status != BLOCKED_RCON)){
				ROS_INFO("\033[1;33m" "%s,%d: (%d,%d)" "\033[0m", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
				set_cell(id, x, y, CLEANED);
				ret = true;
			}
		}
	}
	return ret;
}

Cell_t CostMap::update_position()
{
	auto pos_x = robot::instance()->getPoseX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto pos_y = robot::instance()->getPoseY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	set_position(pos_x, pos_y);
//	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	return get_curr_cell();
}

uint32_t CostMap::get_cleaned_area(void)
{
	uint32_t cleaned_count = 0;
//	ROS_INFO("g_x_min= %d, g_x_max = %d",g_x_min,g_x_max);
//	ROS_INFO("g_y_min= %d, g_y_max = %d",g_y_min,g_y_max);
	for (int i = g_x_min; i <= g_x_max; ++i) {
		for (int j = g_y_min; j <= g_y_max; ++j) {
			if (get_cell(MAP, i, j) == CLEANED) {
				cleaned_count++;
			}
		}
	}
//	ROS_INFO("cleaned_count = %d, area = %.2fm2", cleaned_count, area);
	return cleaned_count;
}

uint8_t CostMap::is_a_block(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	CellState cs;

	cs = get_cell(MAP, x, y);
	if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY)
	//if (cs >= BLOCKED && cs <= BLOCKED_CLIFF)
		retval = 1;

	return retval;
}

uint8_t CostMap::is_blocked_by_bumper(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i, j;
	CellState cs;

	/* Check the point by using the robot size. */
	for (i = ROBOT_RIGHT_OFFSET; retval == 0 && i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; retval == 0 && j <= ROBOT_LEFT_OFFSET; j++) {
			cs = get_cell(MAP, x + i, y + j);
			//if ((cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) && cs != BLOCKED_OBS) {
			if ((cs >= BLOCKED && cs <= BLOCKED_CLIFF) && cs != BLOCKED_OBS) {
				retval = 1;
			}
		}
	}

	return retval;
}

uint8_t CostMap::is_block_accessible(int16_t x, int16_t y)
{
	uint8_t retval = 1;
	int16_t i, j;

	for (i = ROBOT_RIGHT_OFFSET; retval == 1 && i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; retval == 1 && j <= ROBOT_LEFT_OFFSET; j++) {
			if (is_a_block(x + i, y + j) == 1) {
				retval = 0;
			}
		}
	}

	return retval;
}

bool CostMap::is_block_cleanable(int16_t x, int16_t y)
{
	auto retval = is_block_unclean(x, y) && !is_block_blocked(x, y);
//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

int8_t CostMap::is_block_cleaned_unblock(int16_t x, int16_t y)
{
	uint8_t cleaned = 0;
	int16_t i, j;

	for (i = ROBOT_RIGHT_OFFSET; i <= ROBOT_LEFT_OFFSET; i++) {
		for (j = ROBOT_RIGHT_OFFSET; j <= ROBOT_LEFT_OFFSET; j++) {
			auto state = get_cell(MAP, x+i, y+j);
			if (state == CLEANED) {
				cleaned ++;
			} else if(is_block_blocked(x, y))
				return false;
		}
	}

	if (cleaned >= 7)
		return true;
	return false;
}

uint8_t CostMap::is_block_unclean(int16_t x, int16_t y)
{
	uint8_t unclean_cnt = 0;
	for (int8_t i = (y + ROBOT_RIGHT_OFFSET); i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (get_cell(MAP, x, i) == UNCLEAN) {
			unclean_cnt++;
		}
	}
//	ROS_INFO("%s, %d:unclean_cnt(%d)", __FUNCTION__, __LINE__, unclean_cnt);
	return unclean_cnt;
}

uint8_t CostMap::is_block_boundary(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (get_cell(MAP, x, i) == BLOCKED_BOUNDARY) {
			retval = 1;
		}
	}

	return retval;
}

uint8_t CostMap::is_block_blocked(int16_t x, int16_t y)
{
	uint8_t retval = 0;
	int16_t i;

	for (i = (y + ROBOT_RIGHT_OFFSET); retval == 0 && i <= (y + ROBOT_LEFT_OFFSET); i++) {
		if (is_a_block(x, i) == 1) {
			retval = 1;
		}
	}

//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

uint8_t CostMap::is_block_blocked_x_axis(int16_t curr_x, int16_t curr_y)
{
	uint8_t retval = 0;
	int16_t x,y;
	auto dy = mt.is_left()  ?  2 : -2;
	for(auto dx =-1; dx<=1,retval == 0; dx++) {
		robot_to_cell(get_curr_point(), CELL_SIZE * dy, CELL_SIZE * dx, x, y);
		if (is_a_block(x, y) == 1) {
			retval = 1;
		}
	}

//	ROS_INFO("%s, %d:retval(%d)", __FUNCTION__, __LINE__, retval);
	return retval;
}

void CostMap::generate_SPMAP(const Cell_t& curr, std::deque <PPTargetType>& g_paths)
{
	bool		all_set;
	int16_t		x, y, offset, passValue, nextPassValue, passSet, x_min, x_max, y_min, y_max;
	CellState	cs;
	reset(SPMAP);

	path_get_range(SPMAP, &x_min, &x_max, &y_min, &y_max);
	for (auto i = x_min; i <= x_max; ++i) {
		for (auto j = y_min; j <= y_max; ++j) {
			cs = get_cell(MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				for (x = ROBOT_RIGHT_OFFSET; x <= ROBOT_LEFT_OFFSET; x++) {
					for (y = ROBOT_RIGHT_OFFSET; y <= ROBOT_LEFT_OFFSET; y++) {
						set_cell(SPMAP, i + x, j + y, COST_HIGH);
					}
				}
			}
		}
	}

	x = curr.X;
	y = curr.Y;

	/* Set the current robot position has the cost value of 1. */
	set_cell(SPMAP, (int32_t) x, (int32_t) y, COST_1);

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

				if(get_cell(SPMAP, i, j) == passValue) {
					if (i - 1 >= x_min && get_cell(SPMAP, i - 1, j) == COST_NO) {
						set_cell(SPMAP, (i - 1), (j), (CellState) nextPassValue);
						passSet = 1;
					}

					if ((i + 1) <= x_max && get_cell(SPMAP, i + 1, j) == COST_NO) {
						set_cell(SPMAP, (i + 1), (j), (CellState) nextPassValue);
						passSet = 1;
					}

					if (j - 1  >= y_min && get_cell(SPMAP, i, j - 1) == COST_NO) {
						set_cell(SPMAP, (i), (j - 1), (CellState) nextPassValue);
						passSet = 1;
					}

					if ((j + 1) <= y_max && get_cell(SPMAP, i, j + 1) == COST_NO) {
						set_cell(SPMAP, (i), (j + 1), (CellState) nextPassValue);
						passSet = 1;
					}
				}
			}
		}

		all_set = true;
		for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
			if (get_cell(SPMAP, it->front().X, it->front().Y) == COST_NO) {
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
//	print(SPMAP, 0,0);
}

bool CostMap::is_front_block_boundary(int dx)
{
	int32_t x, y;
	for (auto dy = -1; dy <= 1; dy++)
	{
		robot_to_point(get_curr_point(), dy * CELL_SIZE, CELL_SIZE * dx, &x, &y);
		if (get_cell(MAP, count_to_cell(x), count_to_cell(y)) == BLOCKED_BOUNDARY)
			return true;
	}
	return false;
}
void CostMap::path_get_range(uint8_t id, int16_t *x_range_min, int16_t *x_range_max, int16_t *y_range_min, int16_t *y_range_max)
{
	if (id == MAP || id == SPMAP) {
		*x_range_min = g_x_min - (abs(g_x_min - g_x_max) <= 3 ? 3 : 1);
		*x_range_max = g_x_max + (abs(g_x_min - g_x_max) <= 3 ? 3 : 1);
		*y_range_min = g_y_min - (abs(g_y_min - g_y_max) <= 3? 3 : 1);
		*y_range_max = g_y_max + (abs(g_y_min - g_y_max) <= 3 ? 3 : 1);
	}
//	ROS_INFO("Get Range:\tx: %d - %d\ty: %d - %d\tx range: %d - %d\ty range: %d - %d",
//		g_x_min, g_x_max, g_y_min, g_y_max, *x_range_min, *x_range_max, *y_range_min, *y_range_max);
}
void CostMap::print(uint8_t id, int16_t endx, int16_t endy)
{
	char outString[256];
	#if ENABLE_DEBUG
	int16_t		i, j, x_min, x_max, y_min, y_max, index;
	CellState	cs;
	Cell_t temp_cell;

		temp_cell = cost_map.get_curr_cell();

	path_get_range(id, &x_min, &x_max, &y_min, &y_max);

	if (id == MAP) {
		ROS_INFO("Map: %s", "MAP");
	} else if (id == SPMAP) {
		ROS_INFO("Map: %s", "SPMAP");
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
			cs = cost_map.get_cell(id, i, j);
			if (i == temp_cell.X && j == temp_cell.Y) {
				outString[index++] = 'x';
			} else if (i == endx && j == endy) {
				outString[index++] = 'e';
			} else {
				outString[index++] = cs + 48;
			}
		}
		#if COLOR_DEBUG_MAP
		color_print(outString, 0, index);
		#else
		printf("%s\n", outString);
		#endif
	}
	printf("\n");
	#endif
}
void CostMap::color_print(char *outString, int16_t y_min, int16_t y_max)
{
	int16_t j = 0;
	char cs;
	bool ready_print_map = 0;
	std::string y_col("");
	for(j =y_min; j<=y_max; j++){
		cs = *(outString+j);
		if(cs =='\t' && !ready_print_map){
			ready_print_map = 1;
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
					y_col+="\033[1;46;37m1\033[0m";
				else
					y_col+="\033[1;42;37m1\033[0m";
			}
			else if(cs == '2'){//obs
				y_col+="\033[1;44;37m2\033[0m";
			}
			else if(cs == '3'){//bumper
				y_col+="\033[1;41;37m3\033[0m";
			}
			else if(cs == '4'){//cliff
				y_col+="\033[1;45;37m4\033[0m";
			}
			else if(cs == '5'){//rcon
				y_col+="\033[1;46;37m5\033[0m";
			}
			else if(cs == '6'){//laser maker
				y_col+="\033[1;44;37m6\033[0m";
			}
			else if(cs == '7'){//tilt
				y_col+="\033[1;47;30m6\033[0m";
			}
			else if(cs == '8'){//slip
				y_col+="\033[1;43;37m7\033[0m";
			}
			else if(cs == '9'){
				if(std::abs(j%2) == 0)
					y_col+="\033[1;46;37m1\033[0m";
				else
					y_col+="\033[1;42;37m1\033[0m";
			}
			else if(cs == 'a'){//bundary
				y_col+="\033[1;43;37m8\033[0m";
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
bool CostMap::is_block(void)
{
	bool retval = false;
	int16_t x,y;
	std::vector<Cell_t> d_cells;
	d_cells = {{2,1},{2,0},{2,-1},{1,2},{1,1},{1,0},{1,-1},{1,-2},{0,0}};

	for(auto& d_cell : d_cells)
	{
		robot_to_cell(get_curr_point(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x, y);
		if(get_cell(MAP, x, y) == BLOCKED_ROS_MAP)
		{
			retval = true;
			break;
		}
	}
	return retval;
}

SlamMap::SlamMap()
{
}

SlamMap::~SlamMap()
{
}

void SlamMap::setWidth(uint32_t width)
{
	width_ = width;
}

uint32_t SlamMap::getWidth()
{
	return width_;
}

void SlamMap::setHeight(uint32_t height)
{
	height_ = height;
}

uint32_t SlamMap::getHeight()
{
	return height_;
}

void SlamMap::setResolution(float resolution)
{
	resolution_ = resolution;
}

float SlamMap::getResolution()
{
	return resolution_;
}

void SlamMap::setOriginX(double origin_x)
{
	origin_x_ = origin_x;
}

double SlamMap::getOriginX()
{
	return origin_x_;
}

void SlamMap::setOriginY(double origin_y)
{
	origin_y_ = origin_y;
}

double SlamMap::getOriginY()
{
	return origin_y_;
}

void SlamMap::setData(std::vector<int8_t> data)
{
	map_data_ = data;
}

std::vector<int8_t> SlamMap::getData()
{
	return map_data_;
}
