#include <math.h>
#include <stdint.h>

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
int16_t xMin, xMax, yMin, yMax;
int16_t xRangeMin, xRangeMax, yRangeMin, yRangeMax;

int16_t debugMsg[DEBUG_MSG_SIZE][3];
uint8_t debugMsg_idx;

void Map_Initialize(void) {
	uint8_t c, d;

	for(c = 0; c < MAP_SIZE; ++c) {
		for(d = 0; d < (MAP_SIZE + 1) / 2; ++d) {
			map[c][d] = 0;
		}
	}

	xMin = xMax = yMin = yMax = 0;
	xRangeMin = xMin - (MAP_SIZE - (xMax - xMin + 1));
	xRangeMax = xMax + (MAP_SIZE - (xMax - xMin + 1));
	yRangeMin = yMin - (MAP_SIZE - (yMax - yMin + 1));
	yRangeMax = yMax + (MAP_SIZE - (yMax - yMin + 1));

	xCount = 0;
	yCount = 0;

	debugMsg_idx = 0;
}

int16_t Map_GetEstimatedRoomSize(void) {
	int16_t i, j;

	i = xMax - xMin;
	j = yMax - yMin;

	if(i < j) {
		i = yMax - yMin;
		j = xMax - xMin;
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

int16_t Map_GetXPos(void) {
	return countToCell(xCount);
}

int16_t Map_GetYPos(void) {
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
			if(debugMsg_idx < DEBUG_MSG_SIZE && (x - xRangeMax == 1 || xRangeMin - x == 1 || y - yRangeMax == 1 || yRangeMax - y == 1)) {
				debugMsg[debugMsg_idx][0] = x;
				debugMsg[debugMsg_idx][1] = y;
				debugMsg[debugMsg_idx][2] = BLOCKED_BOUNDARY;
				debugMsg_idx++;
			}
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
			if(x < xMin) {
				xMin = x;
				xRangeMin = xMin - (MAP_SIZE - (xMax - xMin + 1));
				xRangeMax = xMax + (MAP_SIZE - (xMax - xMin + 1));
			} else if(x > xMax) {
				xMax = x;
				xRangeMin = xMin - (MAP_SIZE - (xMax - xMin + 1));
				xRangeMax = xMax + (MAP_SIZE - (xMax - xMin + 1));
			}
			if(y < yMin) {
				yMin = y;
				yRangeMin = yMin - (MAP_SIZE - (yMax - yMin + 1));
				yRangeMax = yMax + (MAP_SIZE - (yMax - yMin + 1));
			} else if(y > yMax) {
				yMax = y;
				yRangeMin = yMin - (MAP_SIZE - (yMax - yMin + 1));
				yRangeMax = yMax + (MAP_SIZE - (yMax - yMin + 1));
			}

			ROW = x + MAP_SIZE + MAP_SIZE / 2;
			ROW %= MAP_SIZE;
			COLUMN = y + MAP_SIZE + MAP_SIZE / 2;
			COLUMN %= MAP_SIZE;

	val = (CellState) map[ROW][COLUMN / 2];
	if (((COLUMN % 2) == 0 ? (val >> 4) : (val & 0x0F)) != value) {
		map[ROW][COLUMN / 2] = ((COLUMN % 2) == 0 ? (((value << 4) & 0xF0) | (val & 0x0F)) : ((val & 0xF0) | (value & 0x0F)));

		if(debugMsg_idx < DEBUG_MSG_SIZE) {
			debugMsg[debugMsg_idx][0] = x;
			debugMsg[debugMsg_idx][1] = y;
			debugMsg[debugMsg_idx][2] = value;
			debugMsg_idx++;
		}
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

	for(c = xMin; c < xMax; ++c) {
		for(d = yMin; d < yMax; ++d) {
			if(Map_GetCell(MAP, c, d) == BLOCKED_OBS || Map_GetCell(MAP, c, d) == BLOCKED_BUMPER || Map_GetCell(MAP, c, d) == BLOCKED_CLIFF) {
				if(Map_GetCell(MAP, c - 1, d) != UNCLEAN && Map_GetCell(MAP, c, d + 1) != UNCLEAN && Map_GetCell(MAP, c + 1, d) != UNCLEAN && Map_GetCell(MAP, c, d - 1) != UNCLEAN) {
					Map_SetCell(MAP, cellToCount(c), cellToCount(d), CLEANED);
				}
			}
		}
	}

	Map_SetCell(MAP, cellToCount(Map_GetXPos() - 1), cellToCount(Map_GetYPos()), CLEANED);
	Map_SetCell(MAP, cellToCount(Map_GetXPos()), cellToCount(Map_GetYPos() + 1), CLEANED);
	Map_SetCell(MAP, cellToCount(Map_GetXPos() + 1), cellToCount(Map_GetYPos()), CLEANED);
	Map_SetCell(MAP, cellToCount(Map_GetXPos()), cellToCount(Map_GetYPos() - 1), CLEANED);
}

int32_t Map_GetRelativeX(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
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
	                                      ((double)offset_lat	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
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
	return Map_GetXPos() + offset_long * round(cos(deg2rad(heading, 10))) - offset_lat * round(sin(deg2rad(heading, 10)));
}

int16_t nextYID(uint16_t heading, int16_t offset_lat, int16_t offset_long) {
	return Map_GetYPos() + offset_long * round(sin(deg2rad(heading, 10))) + offset_lat * round(cos(deg2rad(heading, 10)));
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

Point32_t Map_CellToPoint( Point16_t cell ) {
	Point32_t pnt;
	pnt.X = cellToCount(cell.X);
	pnt.Y = cellToCount(cell.Y);
	return pnt;
}

Point16_t Map_PointToCell( Point32_t pnt ) {
	Point16_t cell;
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

void Map_PrintDebug(uint8_t id) {
	uint8_t c;

	id = id;
	if(debugMsg_idx) {
		//Debug_String("$UPD");
		for(c = 0; c < debugMsg_idx; ++c) {
			//Debug_Number(debugMsg[c][0]);
			//Debug_String(",");
			//Debug_Number(debugMsg[c][1]);
			//Debug_String(",");
			//Debug_Number(debugMsg[c][2]);
			//Debug_String(";");
		}
		//Debug_String("#");
		debugMsg_idx = 0;
	}
}

void Map_PrintPosition(void) {
	static int16_t x = 32767, y = 32767;

	if(x != Map_GetXPos() || y != Map_GetYPos()) {
		x = Map_GetXPos();
		y = Map_GetYPos();
		////Debug_String("$RBT");
		//Debug_Number(x);
		//Debug_String(",");
		//Debug_Number(y);
		//Debug_String(";");
		//Debug_String("#\r\n");
	}
	/*
	//Debug_Number(xCount / CELL_COUNT_MUL);
	//Debug_String(".");
	//Debug_Number(((int32_t)xCount % CELL_COUNT_MUL) * 100 / CELL_COUNT_MUL);
	//Debug_String(" (");
	//Debug_Number(xCount);
	//Debug_String(")\t");
	//Debug_Number(yCount / CELL_COUNT_MUL);
	//Debug_String(".");
	//Debug_Number(((int32_t)yCount % CELL_COUNT_MUL) * 100 / CELL_COUNT_MUL);
	//Debug_String(" (");
	//Debug_Number(yCount);
	//Debug_String(")\r\n");
	*/
}
