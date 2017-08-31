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

#include "map.h"
#include "mathematics.h"
#include "robot.hpp"

#define DEBUG_MSG_SIZE	1 // 20

uint8_t map[MAP_SIZE][(MAP_SIZE + 1) / 2];

#ifndef SHORTEST_PATH_V2
uint8_t spmap[MAP_SIZE][(MAP_SIZE + 1) / 2];
#endif

/*wfmap is to record the wall follow path to caculate the isolate islands*/
uint8_t wfmap[MAP_SIZE][(MAP_SIZE + 1) / 2];

uint8_t rosmap[MAP_SIZE][(MAP_SIZE + 1) / 2];
//int16_t homeX, homeY;

double xCount, yCount, relative_sin, relative_cos;
double xWfCount, yWfCount;
double xRosCount, yRosCount;
uint16_t relative_theta = 3600;
int16_t g_x_min, g_x_max, g_y_min, g_y_max;
int16_t g_wf_x_min, g_wf_x_max, g_wf_y_min, g_wf_y_max;
int16_t g_ros_x_min, g_ros_x_max, g_ros_y_min, g_ros_y_max;
int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;
int16_t xWfRangeMin, xWfRangeMax, yWfRangeMin, yWfRangeMax;
int16_t xRosRangeMin, xRosRangeMax, yRosRangeMin, yRosRangeMax;
extern Cell_t g_cell_history[];
extern uint16_t g_old_dir;
void map_init(uint8_t id) {
	uint8_t c, d;
	if (id == MAP) {
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
	} else if(id == WFMAP) {
		for(c = 0; c < MAP_SIZE; ++c) {
			for(d = 0; d < (MAP_SIZE + 1) / 2; ++d) {
				wfmap[c][d] = 0;
			}
		}

		g_wf_x_min = g_wf_x_max = g_wf_y_min = g_wf_y_max = 0;
		xWfRangeMin = g_wf_x_min - (MAP_SIZE - (g_wf_x_max - g_wf_x_min + 1));
		xWfRangeMax = g_wf_x_max + (MAP_SIZE - (g_wf_x_max - g_wf_x_min + 1));
		yWfRangeMin = g_wf_y_min - (MAP_SIZE - (g_wf_y_max - g_wf_y_min + 1));
		yWfRangeMax = g_wf_y_max + (MAP_SIZE - (g_wf_y_max - g_wf_y_min + 1));

		xWfCount = 0;
		yWfCount = 0;
	}
	else if(id == ROSMAP) {
		for(c = 0; c < MAP_SIZE; ++c) {
			for(d = 0; d < (MAP_SIZE + 1) / 2; ++d) {
				rosmap[c][d] = 0;
			}
		}

		g_ros_x_min = g_ros_x_max = g_ros_y_min = g_ros_y_max = 0;
		xRosRangeMin = g_ros_x_min - (MAP_SIZE - (g_ros_x_max - g_ros_x_min + 1));
		xRosRangeMax = g_ros_x_max + (MAP_SIZE - (g_ros_x_max - g_ros_x_min + 1));
		yRosRangeMin = g_ros_y_min - (MAP_SIZE - (g_ros_y_max - g_ros_y_min + 1));
		yRosRangeMax = g_ros_y_max + (MAP_SIZE - (g_ros_y_max - g_ros_y_min + 1));

		xRosCount = 0;
		yRosCount = 0;
	}
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
CellState map_get_cell(uint8_t id, int16_t x, int16_t y, bool is_wf_map) {
	CellState val;
	int16_t x_min, x_max, y_min, y_max;
	if (id == MAP || id == SPMAP) {
		x_min = xRangeMin;
		x_max = xRangeMax;
	   	y_min = yRangeMin;
	   	y_max = yRangeMax;
	} 
	if (id == WFMAP || is_wf_map == true) {
		x_min = xWfRangeMin;
		x_max = xWfRangeMax;
	   	y_min = yWfRangeMin;
	   	y_max = yWfRangeMax;
	}
	if(x >= x_min && x <= x_max && y >= y_min && y <= y_max) {
		x += MAP_SIZE + MAP_SIZE / 2;
		x %= MAP_SIZE;
		y += MAP_SIZE + MAP_SIZE / 2;
		y %= MAP_SIZE;

#ifndef SHORTEST_PATH_V2
		//val = (CellState)((id == MAP) ? (map[x][y / 2]) : (spmap[x][y / 2]));
		if (id == MAP) {
			val = (CellState)(map[x][y / 2]);
		} else if (id == WFMAP) {
			val = (CellState)(wfmap[x][y / 2]);
		} else if (id == SPMAP) {
			val = (CellState)(spmap[x][y / 2]);
		}
#else
		//val = (CellState)(map[x][y / 2]);
		if (id == MAP) {
			val = (CellState)(map[x][y / 2]);
		} else if (id == WFMAP) {
			val = (CellState)(wfmap[x][y / 2]);
		}
#endif

		/* Upper 4 bits & lower 4 bits. */
		val = (CellState) ((y % 2) == 0 ? (val >> 4) : (val & 0x0F));

	} else {
		if(id == MAP || id == WFMAP) {
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

	if(id == MAP || id == WFMAP || id == ROSMAP) {
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
	} else if (id == WFMAP) {
		if(x >= xWfRangeMin && x <= xWfRangeMax && y >= yWfRangeMin && y <= yWfRangeMax) {
			if(x < g_wf_x_min) {
				g_wf_x_min = x;
				xWfRangeMin = g_wf_x_min - (MAP_SIZE - (g_wf_x_max - g_wf_x_min + 1));
				xWfRangeMax = g_wf_x_max + (MAP_SIZE - (g_wf_x_max - g_wf_x_min + 1));
			} else if(x > g_wf_x_max) {
				g_wf_x_max = x;
				xWfRangeMin = g_wf_x_min - (MAP_SIZE - (g_wf_x_max - g_wf_x_min + 1));
				xWfRangeMax = g_wf_x_max + (MAP_SIZE - (g_wf_x_max - g_wf_x_min + 1));
			}
			if(y < g_wf_y_min) {
				g_wf_y_min = y;
				yWfRangeMin = g_wf_y_min - (MAP_SIZE - (g_wf_y_max - g_wf_y_min + 1));
				yWfRangeMax = g_wf_y_max + (MAP_SIZE - (g_wf_y_max - g_wf_y_min + 1));
			} else if(y > g_wf_y_max) {
				g_wf_y_max = y;
				yWfRangeMin = g_wf_y_min - (MAP_SIZE - (g_wf_y_max - g_wf_y_min + 1));
				yWfRangeMax = g_wf_y_max + (MAP_SIZE - (g_wf_y_max - g_wf_y_min + 1));
			}

			ROW = x + MAP_SIZE + MAP_SIZE / 2;
			ROW %= MAP_SIZE;
			COLUMN = y + MAP_SIZE + MAP_SIZE / 2;
			COLUMN %= MAP_SIZE;

			val = (CellState) wfmap[ROW][COLUMN / 2];
			if (((COLUMN % 2) == 0 ? (val >> 4) : (val & 0x0F)) != value) {
				wfmap[ROW][COLUMN / 2] = ((COLUMN % 2) == 0 ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
			}
		}
#ifndef SHORTEST_PATH_V2
	} else if (id == SPMAP){
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
	}else if (id == ROSMAP) {
		if(x >= xRosRangeMin && x <= xRosRangeMax && y >= yRosRangeMin && y <= yRosRangeMax) {
			if(x < g_ros_x_min) {
				g_ros_x_min = x;
				xRosRangeMin = g_ros_x_min - (MAP_SIZE - (g_ros_x_max - g_ros_x_min + 1));
				xRosRangeMax = g_ros_x_max + (MAP_SIZE - (g_ros_x_max - g_ros_x_min + 1));
			} else if(x > g_ros_x_max) {
				g_ros_x_max = x;
				xRosRangeMin = g_ros_x_min - (MAP_SIZE - (g_ros_x_max - g_ros_x_min + 1));
				xRosRangeMax = g_ros_x_max + (MAP_SIZE - (g_ros_x_max - g_ros_x_min + 1));
			}
			if(y < g_ros_y_min) {
				g_ros_y_min = y;
				yRosRangeMin = g_ros_y_min - (MAP_SIZE - (g_ros_y_max - g_ros_y_min + 1));
				yRosRangeMax = g_ros_y_max + (MAP_SIZE - (g_ros_y_max - g_ros_y_min + 1));
			} else if(y > g_ros_y_max) {
				g_ros_y_max = y;
				yRosRangeMin = g_ros_y_min - (MAP_SIZE - (g_ros_y_max - g_ros_y_min + 1));
				yRosRangeMax = g_ros_y_max + (MAP_SIZE - (g_ros_y_max - g_ros_y_min + 1));
			}

			ROW = x + MAP_SIZE + MAP_SIZE / 2;
			ROW %= MAP_SIZE;
			COLUMN = y + MAP_SIZE + MAP_SIZE / 2;
			COLUMN %= MAP_SIZE;

			val = (CellState) rosmap[ROW][COLUMN / 2];
			if (((COLUMN % 2) == 0 ? (val >> 4) : (val & 0x0F)) != value) {
				rosmap[ROW][COLUMN / 2] = ((COLUMN % 2) == 0 ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));
			}
		}
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

int32_t map_get_relative_x(int16_t heading, int16_t dy, int16_t dx, bool using_point_pos) {
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

	if (using_point_pos)
		// Using robot current count as position.
		return map_get_x_count() + (int32_t)( ( ((double)dx * relative_cos * CELL_COUNT_MUL) -
											((double)dy	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
	else
		// Using middle of robot current cell as position.
		return cell_to_count(map_get_x_cell()) + (int32_t)( ( ((double)dx * relative_cos * CELL_COUNT_MUL) -
											((double)dy	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
}

int32_t map_get_relative_y(int16_t heading, int16_t dy, int16_t dx, bool using_point_pos) {
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

	if (using_point_pos)
		// Using robot current count as position.
		return map_get_y_count() + (int32_t)( ( ((double)dx * relative_sin * CELL_COUNT_MUL) +
											((double)dy	* relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	else
		// Using middle of robot current cell as position.
		return cell_to_count(map_get_y_cell()) + (int32_t)( ( ((double)dx * relative_sin * CELL_COUNT_MUL) +
											((double)dy	* relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
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
	if (id == SPMAP) {
		for (idx = 0; idx < MAP_SIZE; idx++) {
			memset((spmap[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
		}
	} else if (id == MAP) {
		for (idx = 0; idx < MAP_SIZE; idx++) {
			memset((map[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
		}
	} else if (id == WFMAP) {
		for (idx = 0; idx < MAP_SIZE; idx++) {
			memset((wfmap[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
		}
	}else if (id == ROSMAP) {
		for (idx = 0; idx < MAP_SIZE; idx++) {
			memset((rosmap[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
		}
	}
#endif
}

void ros_map_convert(int16_t id, bool is_mark_cleaned,bool is_clear_block)
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
		} else if (cost == 0 ) {
			//map_set_cell(MAP, cell_to_count(cx), cell_to_count(cy), CLEANED);
			//map_set_cell(MAP, cx, cy, BLOCKED_RCON);
			auto status = map_get_cell(id, count_to_cell(cx), count_to_cell(cy));
      if(is_mark_cleaned && status <= 1)
					map_set_cell(id, cx, cy, CLEANED);
			if(is_clear_block && (status == BLOCKED_BUMPER || status == BLOCKED_OBS))
					map_set_cell(id, cx, cy, CLEANED);
			//ROS_INFO("cost == 0");
		} else if (cost == 100) {
			//map_set_cell(MAP, cell_to_count(cx), cell_to_count(cy), BLOCKED);
			map_set_cell(id, cx, cy, BLOCKED_CLIFF);
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

//map--------------------------------------------------------
void map_set_obs()
{
#if LASER_MARKER
	MotionManage::s_laser->laserMarker(true);
#else
	auto obs_trig = /*g_obs_triggered*/get_obs_status();
	ROS_INFO("%s,%d: g_obs_triggered(%d)",__FUNCTION__,__LINE__,g_obs_triggered);
	if(! obs_trig)
		return;
	uint8_t obs_lr[] = {Status_Left_OBS, Status_Right_OBS};
	for (auto dir = 0; dir < 2; ++dir)
	{
		if (obs_trig & obs_lr[dir])
		{
			auto dx = 1;
			auto dy = (dir==0) ?2:-2;
			int32_t x,y;
			cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			if (get_wall_adc(dir) > 200)
			{
				if (map_get_cell(MAP, count_to_cell(x), count_to_cell(y)) != BLOCKED_BUMPER)
				{
					ROS_INFO("%s,%d: (%d,%d)",__FUNCTION__,__LINE__,count_to_cell(x),count_to_cell(y));
					map_set_cell(MAP, x, y, BLOCKED_OBS); //BLOCKED_OBS);
				}
			}
		}
	}

	uint8_t obs_all[] = {Status_Right_OBS, Status_Front_OBS, Status_Left_OBS};
	for (auto dy = 0; dy <= 2; ++dy) {
		auto is_trig = obs_trig & obs_all[dy];
		int32_t x, y;
		cm_world_to_point(gyro_get_angle(), (dy-1) * CELL_SIZE, CELL_SIZE_2, &x, &y);
		auto status = map_get_cell(MAP, count_to_cell(x), count_to_cell(y));
		if (is_trig && status != BLOCKED_BUMPER) {
//			ROS_WARN("%s,%d: (%d,%d)",__FUNCTION__,__LINE__,count_to_cell(x),count_to_cell(y));
//			if(dy == 2 && g_turn_angle<0)
//				ROS_ERROR("%s,%d: not map_set_realtime left in turn right, (%d,%d)",__FUNCTION__,__LINE__,count_to_cell(x),count_to_cell(y));
//			else if (dy == 0 && g_turn_angle>0)
//				ROS_ERROR("%s,%d: not map_set_realtime right turn left, (%d,%d)",__FUNCTION__,__LINE__,count_to_cell(x),count_to_cell(y));
//			else
//				map_set_cell(MAP, x, y, BLOCKED_OBS);
			map_set_cell(MAP, x, y, BLOCKED_OBS);
		} else if(! is_trig && status == BLOCKED_OBS){
			ROS_INFO("%s,%d:unclean (%d,%d)",__FUNCTION__,__LINE__,count_to_cell(x),count_to_cell(y));
			map_set_cell(MAP, x, y, UNCLEAN);
		}
	}
#endif
}

void map_set_bumper()
{
	auto bumper_trig = /*g_bumper_triggered*/get_bumper_status();
	if (g_bumper_jam || g_bumper_cnt>=2 || ! bumper_trig)
		// During self check.
		return;

	std::vector<Cell_t> d_cells;

	if ((bumper_trig & RightBumperTrig) && (bumper_trig & LeftBumperTrig))
		d_cells = {{2,-1}, {2,0}, {2,1}};
	else if (bumper_trig & LeftBumperTrig) {
		//d_cells = {{2, 1}, {2,2}/*,{1,2}*/};
		if(mt_is_linear())
			d_cells = {{2, 1}/*, {2,2},{1,2}*/};
		else
			d_cells = {{2, 1}, {2,2}, {1,2}};
		if (g_cell_history[0] == g_cell_history[1] && g_cell_history[0] == g_cell_history[2])
			d_cells.push_back({2,0});
	} else if (bumper_trig & RightBumperTrig) {
		if(mt_is_linear())
			d_cells = {{2,-1}/*,{2,-2},{1,-2}*/};
		else
			d_cells = {{2,-2},{2,-1},{1,-2}};
		if (g_cell_history[0] == g_cell_history[1]  && g_cell_history[0] == g_cell_history[2])
			d_cells.push_back({2,0});
	}

	int32_t	x, y;
	int16_t	x2, y2;
	std::string msg = "Cell:";
	for(auto& d_cell : d_cells){
		cm_world_to_point(gyro_get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x, &y);
		cm_world_to_cell(gyro_get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, x2, y2);
		ROS_WARN("%s %d: d_cell(%d, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d).", __FUNCTION__, __LINE__, d_cell.X, d_cell.Y, gyro_get_angle(), x, y, count_to_cell(x), count_to_cell(y), x2, y2);
		msg += "(" + std::to_string(x2) + "," + std::to_string(y2) + ")";
		map_set_cell(MAP, cell_to_count(x2), cell_to_count(y2), BLOCKED_BUMPER);
	}
	ROS_INFO("\033[31m""%s,%d: Current(%d, %d), mark %s""\033[0m",__FUNCTION__, __LINE__, map_get_x_cell(), map_get_y_cell(), msg.c_str());
}

void map_set_tilt()
{
	auto tilt_trig = get_tilt_status();
	if (!tilt_trig)
		return;

	std::vector<Cell_t> d_cells;

	if(tilt_trig & TILT_LEFT){
		//d_cells = {{2, 1}, {2, 0}};
		d_cells = {{2, 2}, {2, 1}, {2, 0}, {1, 2}, {1, 1}, {1, 0}, {0, 2}, {0, 1}, {0, 0}};
		//if (g_cell_history[0] == g_cell_history[1] && g_cell_history[0] == g_cell_history[2])
		//	d_cells.push_back({2,0});
	}

	if(tilt_trig & TILT_RIGHT){
		//d_cells = {{2, -1}, {2, 0}};
		d_cells = {{2, -2}, {2, -1}, {2, 0}, {1, -2}, {1, -1}, {1, 0}, {0, -2}, {0, -1}, {0, 0}};
		//if (g_cell_history[0] == g_cell_history[1] && g_cell_history[0] == g_cell_history[2])
		//	d_cells.push_back({2,0});
	}

	if(tilt_trig & TILT_FRONT)
		//d_cells = {{2, 1}, {2, 0}, {2, -1}};
		d_cells = {{2, 1}, {2, 0}, {2, -1}, {1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, 0}, {0, -1}};

	int32_t	x, y;
	std::string msg = "Cell:";
	for(auto& d_cell : d_cells){
		cm_world_to_point(gyro_get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x, &y);
		ROS_INFO("%s,%d: (%d,%d)",__FUNCTION__,__LINE__,count_to_cell(x),count_to_cell(y));
		msg += "(" + std::to_string(count_to_cell(x)) + "," + std::to_string(count_to_cell(y)) + ")";
		map_set_cell(MAP, x, y, BLOCKED_TILT);
	}
	ROS_INFO("\033[31m""%s,%d: Current(%d, %d), mark %s""\033[0m",__FUNCTION__, __LINE__, map_get_x_cell(), map_get_y_cell(), msg.c_str());
}

void map_set_cliff()
{
	auto cliff_trig = /*g_cliff_triggered*/get_cliff_status();
	if (g_cliff_jam || g_cliff_cnt>=2 || ! cliff_trig)
		// During self check.
		return;

	std::vector<Cell_t> d_cells;
	if (cliff_trig & Status_Cliff_Front){
		d_cells.push_back({2,-1});
		d_cells.push_back({2, 0});
		d_cells.push_back({2, 1});
	}
	if (cliff_trig & Status_Cliff_Left){
		d_cells.push_back({2, 1});
		d_cells.push_back({2, 2});
	}
	if (cliff_trig & Status_Cliff_Right){
		d_cells.push_back({2,-1});
		d_cells.push_back({2,-2});
	}

	int32_t	x, y;
	for (auto& d_cell : d_cells) {
		cm_world_to_point(gyro_get_angle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x, &y);
		ROS_INFO("%s,%d: (%d,%d)",__FUNCTION__,__LINE__,count_to_cell(x),count_to_cell(y));
		map_set_cell(MAP, x, y, BLOCKED_CLIFF);
	}
}

void map_set_rcon()
{
	auto rcon_trig = g_rcon_triggered/*get_rcon_trig()*/;
	if(mt_is_linear())
		g_rcon_triggered = 0;
	if(! rcon_trig)
		return;

	enum {
			left, fl2, fl1, fr1, fr2, right,
	};
	int dx = 0, dy = 0;
	int dx2 = 0, dy2 = 0;
	switch (rcon_trig - 1)
	{
		case left:
			dx = 1, dy = 2;
					break;
		case fl2:
			dx = 1, dy = 2;
					dx2 = 2, dy2 = 1;
					break;
		case fl1:
		case fr1:
			dx = 2, dy = 0;
					dx2 = 3, dy2 = 0;
					break;
		case fr2:
			dx = 1, dy = -2;
					dx2 = 2, dy2 = -1;
					break;
		case right:
			dx = 1, dy = -2;
					break;
	}
	int32_t x,y;
	cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
//	ROS_ERROR("%s,%d:curr(%d,%d), map_set_realtime(%d,%d),rcon_trig(%d)",__FUNCTION__,__LINE__,map_get_curr_cell().X,map_get_curr_cell().Y, count_to_cell(x),count_to_cell(y),rcon_trig);
	map_set_cell(MAP, x, y, BLOCKED_RCON);
	if (dx2 != 0){
		cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy2, CELL_SIZE * dx2, &x, &y);
//		ROS_ERROR("%s,%d: map_set_realtime(%d,%d)",__FUNCTION__,__LINE__,count_to_cell(x),count_to_cell(y));
		map_set_cell(MAP, x, y, BLOCKED_RCON);
	}
//	MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell);
//	stop_brifly();
//	sleep(5);
}

void map_set_blocked()
{
	if(robot::instance()->getBaselinkFrameType() != Map_Position_Map_Angle)
		return;

//	ROS_ERROR("----------------map_set_blocked");
	map_set_obs();
	map_set_bumper();
	map_set_rcon();
	map_set_cliff();
	map_set_tilt();
	std::list<Cell_t> empty_path;
	empty_path.clear();
	MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell, empty_path);
}

void map_set_cleaned(const Cell_t& curr)
{
	int32_t x, y;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = 0/*-ROBOT_SIZE_1_2*/; dx <= ROBOT_SIZE_1_2; ++dx)
		{
			cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			auto status = map_get_cell(MAP, x, y);
//			if (status > CLEANED && status < BLOCKED_BOUNDARY)
//				ROS_ERROR("%s,%d: (%d,%d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));

//			ROS_ERROR("%s,%d: (%d,%d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
			map_set_cell(MAP, x, y, CLEANED);
		}
	}
}

void map_set_follow_wall(const Cell_t& curr)
{
	auto dx = curr.X - count_to_cell(RegulatorBase::s_origin.X);
		if(dx == 0)
			return;
		auto dy = mt_is_left()  ?  2 : -2;
		ROS_INFO("%s,%d: mt(%d),dx(%d),dy(%d)",__FUNCTION__,__LINE__,mt_is_left(),dx, dy);
		if((g_old_dir == POS_X && dx <= -2) || (g_old_dir == NEG_X && dx >= 2))
		{
			for (dx = -1; dx <= 0; dx++)
			{
				int x, y;
				cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
				ROS_INFO("%s,%d: diff_y(%d)",__FUNCTION__, __LINE__, count_to_cell(y) - curr.Y);
				if ( std::abs(count_to_cell(y) - curr.Y) >= 2 )
					map_set_cell(MAP, x, y, BLOCKED_CLIFF);
			}
		}
		if((g_old_dir == POS_X && dx >= 2) || (g_old_dir == NEG_X && dx <= -2))
		{
			for (dx = -1; dx <= 0; dx++)
			{
				int x, y;
				cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
				ROS_INFO("%s,%d: diff_y(%d)",__FUNCTION__, __LINE__, count_to_cell(y) - curr.Y);
				if (count_to_cell(y) -curr.Y <= 2);
					map_set_cell(MAP, x, y, BLOCKED_CLIFF);
			}
		}
}

void map_set_cleaned(std::vector<Cell_t>& cells)
{
	if(cells.empty())
		return;
	auto min_y = cells.front().Y - ROBOT_SIZE_1_2;
	auto max_y = cells.front().Y + ROBOT_SIZE_1_2;
	int8_t dx;

	if (!mt_is_linear())
	{
		dx = (cells.front().X < cells.back().X) ? 1 : -1;//X_POS
		Cell_t cell_front = {int16_t(cells.front().X - dx),cells.front().Y};
		Cell_t cell_back = {int16_t(cells.back().X + dx),cells.back().Y};
		cells.push_back(cell_front);
		cells.push_back(cell_back);
//		auto is_follow_y_min = dx == 1 ^ mt_is_left();
	}
	else
	{
		extern uint16_t g_new_dir;
		if (g_new_dir == POS_X)
			dx = 1;
		else if (g_new_dir == NEG_X)
			dx = -1;
		else // POS_Y/NEG_Y
			dx = 0;

		if (dx)
		{
			Cell_t cell = {int16_t(cells.back().X + dx),cells.back().Y};
			cells.push_back(cell);
		}
	}

	std::string msg = "Cell:";
	for (const auto& cell :  cells)
	{
		for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; dy++)
		{
			auto y = cell.Y + dy;
//			if((! is_follow_y_min && y < min_y) || (is_follow_y_min && y > max_y))
//				continue;
			auto status = map_get_cell(MAP, cell.X, y);
			if (status != BLOCKED_TILT)
				map_set_cell(MAP, cell_to_count(cell.X), cell_to_count(y), CLEANED);
			msg += "(" + std::to_string(cell.X) + "," + std::to_string(y) + ")";
		}
	}

	int32_t x, y;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
			cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			if (map_get_cell(MAP, count_to_cell(x), count_to_cell(y)) == UNCLEAN){
				map_set_cell(MAP, x, y, CLEANED);
			}
		}
	}
	ROS_INFO("\033[32m""%s,%d: %s""\033[0m",__FUNCTION__, __LINE__, msg.c_str());
}

void map_set_follow_wall(std::vector<Cell_t>& cells)
{
	auto diff = cells.back().X - cells.front().X;

	if (cells.size() < 2 || std::abs(diff) <= 4)
		return;

	for (const auto &cell : cells)
	{
		ROS_INFO("\033[47;31m""%s,%d: cell(%d,%d)""\033[0m", __FUNCTION__, __LINE__, cell.X, cell.Y);
	}
	auto dy = diff>0 ^ mt_is_left() ? -2 : 2;
	auto min = std::min(cells.front().X, cells.back().X) + 3;
	auto max = std::max(cells.front().X, cells.back().X) - 3;

	if(min >= max)
		return;

	for (const auto &cell : cells)
	{
		if (cell.X >= min && cell.X <= max)
		{
			map_set_cell(MAP, cell_to_count(cell.X), cell_to_count(cell.Y + dy), BLOCKED_CLIFF);
			ROS_INFO("\033[47;31m" "%s,%d: cell(%d,%d)" "\033[0m", __FUNCTION__, __LINE__, cell.X, cell.Y + dy);
		}
	}
}

bool map_mark_robot(uint8_t id)
{
	int32_t x, y;
	bool ret = false;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
			cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			auto status = map_get_cell(id, count_to_cell(x), count_to_cell(y));
			if (status > CLEANED && status < BLOCKED_BOUNDARY){
				ROS_INFO("\033[1;33m" "%s,%d: (%d,%d)" "\033[0m", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
				ret = true;
			}
			map_set_cell(id, x, y, CLEANED);
		}
	}
	return ret;
}

Cell_t cm_update_position(bool is_turn)
{
	auto pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	map_set_position(pos_x, pos_y);
//	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	return map_get_curr_cell();
}

void map_set_linear(const Cell_t &start, const Cell_t &stop, CellState state)
{
	ROS_ERROR("%s,%d: start(%d,%d),stop(%d,%d)",__FUNCTION__, __LINE__, start.X,start.Y,stop.X,stop.Y);

	float slop = (((float) start.Y) - ((float) stop.Y)) / (((float) start.X) - ((float) stop.X));
	float intercept = ((float) (stop.Y)) - slop * ((float) (stop.X));

	auto start_x = std::min(start.X, stop.X);
	auto stop_x = std::max(start.X, stop.X);
	for (auto x = start_x; x <= stop_x; x++)
	{
		auto y = (int16_t) (slop * (stop.X) + intercept);
		ROS_ERROR("%s,%d: cell(%d,%d)",__FUNCTION__, __LINE__, x, y);
		map_set_cell(MAP, cell_to_count(x), cell_to_count(y), state);
	}
}

void map_set_follow(Cell_t start)
{
	auto stop = map_get_curr_cell();
	ROS_ERROR("%s,%d: start(%d,%d),stop(%d,%d)",__FUNCTION__, __LINE__, start.X,start.Y,stop.X,stop.Y);
	auto dx = stop.X - start.X;
	if (dx != 0)
	{
		auto dy_ = (g_old_dir == POS_X ^ dx > 0) ? 3 : 2;
		dy_ = (dx > 0 ^ mt_is_left()) ? -dy_ : dy_;
		start.Y += dy_;
		stop.Y += dy_;

		auto dx_ = (g_old_dir == POS_X) ? 2 : -2;
		start.X += dx_;
		stop.X -= dx_;
		auto new_dx = stop.X - start.X;
		if(new_dx != 0 && dx>0 ^ new_dx<0)
			map_set_linear(start, stop, BLOCKED_CLIFF);
	}
}

float map_get_area(void)
{
	uint32_t cleaned_count = 0;
	float area = 0;
//ROS_INFO("g_x_min= %d, g_x_max = %d",g_x_min,g_x_max);
//ROS_INFO("g_y_min= %d, g_y_max = %d",g_y_min,g_y_max);
	for (int i = g_x_min; i <= g_x_max; ++i) {
		for (int j = g_y_min; j <= g_y_max; ++j) {
			//ROS_INFO("i = %d, j = %d", i, j);
			if (map_get_cell(MAP, i, j) == CLEANED) {
				cleaned_count++;
			}
		}
	}
	area = cleaned_count * (CELL_SIZE * 0.001) * (CELL_SIZE * 0.001);
//ROS_WARN("cleaned_count = %d, area = %.2fm2", cleaned_count, area);
	return area;
}

void map_set_block(const Cell_t &start, const Cell_t &stop,CellState state)
{
	for (auto x = start.X; x <= stop.X; x++) {
		for (auto y = start.Y; y <= stop.Y; y++)
			map_set_cell(MAP, cell_to_count(x), cell_to_count(y), state);
	}
}

void map_set_block_with_bound(const Cell_t &start, const Cell_t &stop,CellState state)
{
	Cell_t bound = {1,1};

	Cell_t b_start = start - bound;
	Cell_t b_stop = stop + bound;

	map_set_block(b_start, b_stop, BLOCKED);
	map_set_block(start, stop, state);
}
