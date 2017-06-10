#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "map.h"
#include "mathematics.h"

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

void Map_Initialize(void) {
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

int16_t Map_GetEstimatedRoomSize(void) {
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

int32_t Map_GetXCount(void) {
	return (int32_t)round(xCount);
}

int32_t Map_GetYCount(void) {
	return (int32_t)round(yCount);
}

int16_t Map_GetXCell(void) {
	return countToCell(xCount);
}

int16_t Map_GetYCell(void) {
	return countToCell(yCount);
}

void Map_MoveTo(double d_x, double d_y) {
	xCount += d_x;
	yCount += d_y;
}

void Map_SetPosition(double x, double y) {
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
 * Map_GetCell description
 * @param id	Map id
 * @param x	Cell x
 * @param y	Cell y
 * @return	CellState
 */
CellState Map_GetCell(uint8_t id, int16_t x, int16_t y) {
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
 * Map_SetCell description
 * @param id		Map id
 * @param x		 Count x
 * @param y		 Count y
 * @param value CellState
 */
void Map_SetCell(uint8_t id, int32_t x, int32_t y, CellState value) {
	CellState val;
	int16_t ROW, COLUMN;

	if(id == MAP) {
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

void Map_ClearBlocks(void) {
	int16_t c, d;

	for(c = g_x_min; c < g_x_max; ++c) {
		for(d = g_y_min; d < g_y_max; ++d) {
			if(Map_GetCell(MAP, c, d) == BLOCKED_OBS || Map_GetCell(MAP, c, d) == BLOCKED_BUMPER || Map_GetCell(MAP, c, d) == BLOCKED_CLIFF) {
				if(Map_GetCell(MAP, c - 1, d) != UNCLEAN && Map_GetCell(MAP, c, d + 1) != UNCLEAN && Map_GetCell(MAP, c + 1, d) != UNCLEAN && Map_GetCell(MAP, c, d - 1) != UNCLEAN) {
					Map_SetCell(MAP, cellToCount(c), cellToCount(d), CLEANED);
				}
			}
		}
	}

	Map_SetCell(MAP, cellToCount(Map_GetXCell() - 1), cellToCount(Map_GetYCell()), CLEANED);
	Map_SetCell(MAP, cellToCount(Map_GetXCell()), cellToCount(Map_GetYCell() + 1), CLEANED);
	Map_SetCell(MAP, cellToCount(Map_GetXCell() + 1), cellToCount(Map_GetYCell()), CLEANED);
	Map_SetCell(MAP, cellToCount(Map_GetXCell()), cellToCount(Map_GetYCell() - 1), CLEANED);
}

int32_t Map_GetRelativeX(uint16_t heading, int16_t dx, int16_t offset_long) {
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
		} else if(heading == 2700) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg2rad(heading, 10));
			relative_cos = cos(deg2rad(heading, 10));
		}
	}

	return Map_GetXCount() + (int32_t)( ( ((double)offset_long * relative_cos * CELL_COUNT_MUL) -
	                                      ((double)dx	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
}

int32_t Map_GetRelativeY(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
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
		} else if(heading == 2700) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg2rad(heading, 10));
			relative_cos = cos(deg2rad(heading, 10));
		}
	}

	return Map_GetYCount() + (int32_t)( ( ((double)offset_long * relative_sin * CELL_COUNT_MUL) +
	                                      ((double)offset_lat *	relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
}
/*
int16_t Map_GetLateralOffset(uint16_t heading) {
	cellToCount(countToCell(Map_GetRelativeX(heading, 75, 0)));
	cellToCount(countToCell(Map_GetRelativeY(heading, 75, 0)));

}

int16_t Map_GetLongitudinalOffset(uint16_t heading) {
}
*/
int16_t nextXID(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	return Map_GetXCell() + offset_long * round(cos(deg2rad(heading, 10))) - offset_lat * round(sin(deg2rad(heading, 10)));
}

int16_t nextYID(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	return Map_GetYCell() + offset_long * round(sin(deg2rad(heading, 10))) + offset_lat * round(cos(deg2rad(heading, 10)));
}

int32_t cellToCount(int16_t i) {
	return i * CELL_COUNT_MUL;
}

int16_t countToCell(double count) {
	if(count < -CELL_COUNT_MUL_1_2) {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL - 1;
	} else {
		return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL;
	}
}

Point32_t Map_CellToPoint( Cell_t cell ) {
	Point32_t pnt;
	pnt.X = cellToCount(cell.X);
	pnt.Y = cellToCount(cell.Y);
	return pnt;
}

Cell_t Map_PointToCell( Point32_t pnt ) {
	Cell_t cell;
	cell.X = countToCell(pnt.X);
	cell.Y = countToCell(pnt.Y);
	return cell;
}

void Map_Set_Cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state)
{
	int8_t i, j;

	for ( i = -(count / 2); i <= count / 2; i++ ) {
		for ( j = -(count / 2); j <= count / 2; j++ ) {
			Map_SetCell(MAP, cellToCount(cell_x + i), cellToCount(cell_y + j), state);
		}
	}
}

void Map_Reset(uint8_t id)
{
#ifndef SHORTEST_PATH_V2
	uint16_t idx;

	for (idx = 0; idx < MAP_SIZE; idx++) {
		memset((id == SPMAP ? spmap[idx] : map[idx]), 0, ((MAP_SIZE + 1) / 2) * sizeof(uint8_t));
	}
#endif
}
