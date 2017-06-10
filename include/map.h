#ifndef __MAP_H__
#define __MAP_H__

#include "config.h"
#include "mathematics.h"

#define MAP 0
#define SPMAP 1

typedef enum {
  UNCLEAN  = 0,
  CLEANED = 1,
  BLOCKED = 2,
  BLOCKED_OBS = 2,
  BLOCKED_BUMPER = 3,
  BLOCKED_CLIFF = 4,
  BLOCKED_BOUNDARY = 5,
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
  POS_X = 0,
  PX_PY = 450,
  POS_Y = 900,
  NS_PY = 1350,
  NEG_X = 1800,
  NX_NY = 2250,
  NEG_Y = 2700,
  PX_NY = 3150,
  NONE = 3600,
} Direction_Cardinal;

void Map_Initialize(void);

int32_t Map_GetXCount(void);
int32_t Map_GetYCount(void);
int16_t Map_GetXCell(void);
int16_t Map_GetYCell(void);

void Map_MoveTo(double x, double y);
void Map_SetPosition(double x, double y);

int32_t Map_GetRelativeX(uint16_t heading, int16_t offset_lat, int16_t offset_long);
int32_t Map_GetRelativeY(uint16_t heading, int16_t offset_lat, int16_t offset_long);

int16_t Map_GetLateralOffset(uint16_t heading);
int16_t Map_GetLongitudinalOffset(uint16_t heading);

int16_t nextXID(uint16_t heading, int16_t offset_lat, int16_t offset_long);
int16_t nextYID(uint16_t heading, int16_t offset_lat, int16_t offset_long);

void Map_GetRelativePosition(uint16_t heading, int16_t *x, int16_t *y);

CellState Map_GetCell(uint8_t id, int16_t x, int16_t y);
void Map_SetCell(uint8_t id, int32_t x, int32_t y, CellState value);

void Map_ClearBlocks(void);
int32_t Map_DistanceLimit(uint16_t heading);

int32_t cellToCount(int16_t distance);
int16_t countToCell(double count);
Point32_t Map_CellToPoint( Cell_t cell );
Cell_t Map_PointToCell( Point32_t pnt );

void Map_Set_Cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state);

void Map_Reset(uint8_t id);

#endif /* __MAP_H */
