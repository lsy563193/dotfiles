#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "map.h"
#include "mathematics.h"
#include "robot.hpp"

#define DEBUG_MSG_SIZE	1 // 20

uint8_t map[MAP_SIZE][(MAP_SIZE + 1) / 2];

#ifndef SHORTEST_PATH_V2
uint8_t spmap[MAP_SIZE][(MAP_SIZE + 1) / 2];
#endif

//int16_t homeX, homeY;

double xCount, yCount, relative_sin, relative_cos;
uint16_t relative_theta = 3600;
int16_t g_x_min, g_x_max, g_y_min, g_y_max;
int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;

void map_init(void) {
	uint8_t c, d;

	for(c = 0; c < MAP_SIZE; ++c) {
		for(d = 0; d < (MAP_SIZE + 1) / 2; ++d) {
			map[c][d] = 0;
		}
	}

	g_x_min = g_x_max = g_y_min = g_y_max = 0;
	xRangeMin = g_x_min - (MAP_SIZE - (g_x_max - g_x_min + 1));
	xRangeMax = g_x_max + (MAP_SIZE - (g_x_max - g_x_min + 1));
	yRangeMin = g_y_min - (MAP_SIZE - (g_y_max - g_y_min + 1));
	yRangeMax = g_y_max + (MAP_SIZE - (g_y_max - g_y_min + 1));

	xCount = 0;
	yCount = 0;
}

int16_t map_get_estimated_room_size(void) {
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

int32_t map_get_x_count(void) {
	return (int32_t)round(xCount);
}

int32_t map_get_y_count(void) {
	return (int32_t)round(yCount);
}

int16_t map_get_x_cell(void) {
	return count_to_cell(xCount);
}

int16_t map_get_y_cell(void) {
	return count_to_cell(yCount);
}

Cell_t map_get_curr_cell()
{
	return Cell_t{map_get_x_cell(), map_get_y_cell()};
}

void map_move_to(double d_x, double d_y) {
	xCount += d_x;
	yCount += d_y;
}

void map_set_position(double x, double y) {
	xCount = x;
	yCount = y;
}
/*
void addXCount(double d) {
	xCount += d;
}

void addYCount(double d) {
	yCount += d;
}
*/

/*
 * map_get_cell description
 * @param id	Map id
 * @param x	Cell x
 * @param y	Cell y
 * @return	CellState
 */
CellState map_get_cell(uint8_t id, int16_t x, int16_t y) {
	CellState val;

	if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
		x += MAP_SIZE + MAP_SIZE / 2;
		x %= MAP_SIZE;
		y += MAP_SIZE + MAP_SIZE / 2;
		y %= MAP_SIZE;

#ifndef SHORTEST_PATH_V2
	val = (CellState)((id == MAP) ? (map[x][y / 2]) : (spmap[x][y / 2]));
#else
	val = (CellState)(map[x][y / 2]);
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
 * map_set_cell description
 * @param id		Map id
 * @param x		 Count x
 * @param y		 Count y
 * @param value CellState
 */
void map_set_cell(uint8_t id, int32_t x, int32_t y, CellState value) {
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

			val = (CellState) map[ROW][COLUMN / 2];
			if (((COLUMN % 2) == 0 ? (val >> 4) : (val & 0x0F)) != value) {
				map[ROW][COLUMN / 2] = ((COLUMN % 2) == 0 ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
			}
		}
#ifndef SHORTEST_PATH_V2
	} else {
		if(x >= xRangeMin && x <= xRangeMax && y >= yRangeMin && y <= yRangeMax) {
			x += MAP_SIZE + MAP_SIZE / 2;
			x %= MAP_SIZE;
			y += MAP_SIZE + MAP_SIZE / 2;
			y %= MAP_SIZE;

			val = (CellState) spmap[x][y / 2];

			/* Upper 4 bits and last 4 bits. */
			spmap[x][y / 2] = (((y % 2) == 0) ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
		}
#endif
	}
}

void map_clear_blocks(void) {
	int16_t c, d;

	for(c = g_x_min; c < g_x_max; ++c) {
		for(d = g_y_min; d < g_y_max; ++d) {
			if(map_get_cell(MAP, c, d) == BLOCKED_OBS || map_get_cell(MAP, c, d) == BLOCKED_BUMPER ||
							map_get_cell(MAP, c, d) == BLOCKED_CLIFF) {
				if(map_get_cell(MAP, c - 1, d) != UNCLEAN && map_get_cell(MAP, c, d + 1) != UNCLEAN &&
								map_get_cell(MAP, c + 1, d) != UNCLEAN &&
								map_get_cell(MAP, c, d - 1) != UNCLEAN) {
					map_set_cell(MAP, cell_to_count(c), cell_to_count(d), CLEANED);
				}
			}
		}
	}

	map_set_cell(MAP, cell_to_count(map_get_x_cell() - 1), cell_to_count(map_get_y_cell()), CLEANED);
	map_set_cell(MAP, cell_to_count(map_get_x_cell()), cell_to_count(map_get_y_cell() + 1), CLEANED);
	map_set_cell(MAP, cell_to_count(map_get_x_cell() + 1), cell_to_count(map_get_y_cell()), CLEANED);
	map_set_cell(MAP, cell_to_count(map_get_x_cell()), cell_to_count(map_get_y_cell() - 1), CLEANED);
}

int32_t map_get_relative_x(int16_t heading, int16_t dx, int16_t dy) {
	if(heading != relative_theta) {
		if(heading == 0) {
			relative_sin = 0;
			relative_cos = 1;
		} else if(heading == 900) {
			relative_sin = 1;
			relative_cos = 0;
		} else if(heading == 1800) {
			relative_sin = 0;
			relative_cos = -1;
		} else if(heading == -900) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg_to_rad(heading, 10));
			relative_cos = cos(deg_to_rad(heading, 10));
		}
	}

	return map_get_x_count() + (int32_t)( ( ((double)dy * relative_cos * CELL_COUNT_MUL) -
	                                      ((double)dx	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
}

int32_t map_get_relative_y(int16_t heading, int16_t offset_lat, int16_t offset_long) {
	if(heading != relative_theta) {
		if(heading == 0) {
			relative_sin = 0;
			relative_cos = 1;
		} else if(heading == 900) {
			relative_sin = 1;
			relative_cos = 0;
		} else if(heading == 1800) {
			relative_sin = 0;
			relative_cos = -1;
		} else if(heading == -900) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg_to_rad(heading, 10));
			relative_cos = cos(deg_to_rad(heading, 10));
		}
	}

	return map_get_y_count() + (int32_t)( ( ((double)offset_long * relative_sin * CELL_COUNT_MUL) +
	                                      ((double)offset_lat *	relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
}
/*
int16_t Map_GetLateralOffset(uint16_t heading) {
	cellToCount(countToCell(map_get_relative_x(heading, 75, 0)));
	cell_to_count(count_to_cell(map_get_relative_y(heading, 75, 0)));

}

int16_t Map_GetLongitudinalOffset(uint16_t heading) {
}
*/
int16_t next_x_id(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	return map_get_x_cell() + offset_long * round(cos(deg_to_rad(heading, 10))) - offset_lat * round(sin(
					deg_to_rad(heading, 10)));
}

int16_t next_y_id(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	return map_get_y_cell() + offset_long * round(sin(deg_to_rad(heading, 10))) + offset_lat * round(cos(
					deg_to_rad(heading, 10)));
}

int32_t cell_to_count(int16_t i) {
	return i * CELL_COUNT_MUL;
}

int16_t count_to_cell(double count) {
	if(count < -CELL_COUNT_MUL_1_2) {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL - 1;
	} else {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL;
	}
}

Point32_t map_cell_to_point(const Cell_t& cell) {
	Point32_t pnt;
	pnt.X = cell_to_count(cell.X);
	pnt.Y = cell_to_count(cell.Y);
	return pnt;
}

Cell_t map_point_to_cell(Point32_t pnt) {
	Cell_t cell;
	cell.X = count_to_cell(pnt.X);
	cell.Y = count_to_cell(pnt.Y);
	return cell;
}

void map_set_cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state)
{
	int8_t i, j;

	for ( i = -(count / 2); i <= count / 2; i++ ) {
		for ( j = -(count / 2); j <= count / 2; j++ ) {
			map_set_cell(MAP, cell_to_count(cell_x + i), cell_to_count(cell_y + j), state);
		}
	}
}

void map_reset(uint8_t id)
{
#ifndef SHORTEST_PATH_V2
	uint16_t idx;

	for (idx = 0; idx < MAP_SIZE; idx++) {
		memset((id == SPMAP ? spmap[idx] : map[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
	}
#endif
}

void ros_map_convert(void)
{
	std::vector<int8_t> *p_map_data;
	uint32_t width, height;
	float resolution;
	double origin_x, origin_y;
	unsigned int mx,my;
	double wx, wy;
	int32_t cx, cy;
	int8_t cost;
	width = robot::instance()->mapGetWidth();
	height = robot::instance()->mapGetHeight();
	resolution = robot::instance()->mapGetResolution();
	origin_x = robot::instance()->mapGetOriginX();
	origin_y = robot::instance()->mapGetOriginY();
	p_map_data = robot::instance()->mapGetMapData();
	//worldToMap(origin_x, origin_y,resolution, width, height, wx, wy, mx, my);
	//cost = getCost(p_map_data, width, mx, my);
	int size = (*p_map_data).size();
	for (int i = 0; i< size; i++) {
		indexToCells(width, i, mx, my);
		cost = (*p_map_data)[getIndex(width, mx, my)];
		mapToWorld(origin_x, origin_y, resolution, mx, my, wx, wy);
		worldToCount(wx, wy, cx, cy);
		if (cost == -1) {
			//ROS_INFO("cost == -1");
			//map_set_cell(MAP, cx, cy, CLEANED);
		} else if (cost == 0) {
			//map_set_cell(MAP, cell_to_count(cx), cell_to_count(cy), CLEANED);
			//map_set_cell(MAP, cx, cy, BLOCKED_RCON);
			//map_set_cell(MAP, cx, cy, CLEANED);
			//ROS_INFO("cost == 0");
		} else if (cost == 100) {
			//map_set_cell(MAP, cell_to_count(cx), cell_to_count(cy), BLOCKED);
			map_set_cell(MAP, cx, cy, BLOCKED);
			//ROS_INFO("cost == 100");
		}
	}
	ROS_WARN("end ros map convert");
	ROS_ERROR("size = %d, width = %d, height = %d, origin_x = %lf, origin_y = %lf, wx = %lf, wy = %lf, mx = %d, my = %d, cx = %d, cy = %d, cost = %d.", size, width, height, origin_x, origin_y, wx, wy, mx, my, cx, cy,cost);
}

/* int8_t getCost(std::vector<int8_t> *p_map_data, uint32_t width, unsigned int mx, unsigned int my)
{
	return (*p_map_data)[getIndex(width, mx, my)];
} */

void mapToWorld(double origin_x_, double origin_y_, float resolution_, unsigned int mx, unsigned int my, double& wx, double& wy)
{
	wx = origin_x_ + (mx + 0.5) * resolution_;
	wy = origin_y_ + (my + 0.5) * resolution_;
	//wx = origin_x_ + (mx) * resolution_;
	//wy = origin_y_ + (my) * resolution_;
}

bool worldToMap(double origin_x_, double origin_y_, float resolution_, int size_x_, int size_y_, double wx, double wy, unsigned int& mx, unsigned int& my)
{
	if (wx < origin_x_ || wy < origin_y_)
		return false;

	mx = (int)((wx - origin_x_) / resolution_);
	my = (int)((wy - origin_y_) / resolution_);

	if (mx < size_x_ && my < size_y_)
		return true;

	return false;
}

unsigned int getIndex(int size_x_, unsigned int mx, unsigned int my)
{
	return my * size_x_ + mx;
}

void indexToCells(int size_x_, unsigned int index, unsigned int& mx, unsigned int& my)
{
	my = index / size_x_;
	mx = index - (my * size_x_);
}

void worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy)
{
	auto count_x = wx * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto count_y = wy * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	//cx = count_to_cell(count_x);
	cx = count_x;
	//cy = count_to_cell(count_y);
	cy = count_y;
}
