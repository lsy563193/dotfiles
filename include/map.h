#ifndef __MAP_H__
#define __MAP_H__

#include "config.h"
#include "mathematics.h"
#include <vector>

#define MAP 0
#define SPMAP 1
#define WFMAP 2
#define ROSMAP 3

typedef enum {
  UNCLEAN  = 0,
  CLEANED = 1,
  BLOCKED = 2,
  BLOCKED_OBS = 2,
  BLOCKED_BUMPER = 3,
  BLOCKED_CLIFF = 4,
  BLOCKED_RCON = 5,
  BLOCKED_TILT = 6,
  BLOCKED_SLIP = 7,
  BLOCKED_ROS_MAP = 8,
  BLOCKED_BOUNDARY = 9,
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

#define IS_POS_AXIS(x) (x == POS_X || x == POS_Y || x == NONE)
#define	IS_X_AXIS(x) (x == NEG_X || x == POS_X || x == NONE)
#define	IS_Y_AXIS(x) (x == POS_Y || x == NEG_Y)

void map_init(uint8_t id);

int32_t map_get_x_count(void);
int32_t map_get_y_count(void);
int16_t map_get_x_cell(void);
int16_t map_get_y_cell(void);

void map_move_to(double x, double y);
void map_set_position(double x, double y);

int32_t map_get_relative_x(int16_t heading, int16_t dy, int16_t dx, bool using_point_pos);
int32_t map_get_relative_y(int16_t heading, int16_t dy, int16_t dx, bool using_point_pos);

int16_t Map_GetLateralOffset(uint16_t heading);
int16_t Map_GetLongitudinalOffset(uint16_t heading);

int16_t next_x_id(uint16_t heading, int16_t offset_lat, int16_t offset_long);
int16_t next_y_id(uint16_t heading, int16_t offset_lat, int16_t offset_long);

void Map_GetRelativePosition(uint16_t heading, int16_t *x, int16_t *y);

CellState map_get_cell(uint8_t id, int16_t x, int16_t y, bool is_wf_map = false);
Cell_t map_get_curr_cell();
void map_set_cell(uint8_t id, int32_t x, int32_t y, CellState value);

void map_clear_blocks(void);
int32_t Map_DistanceLimit(uint16_t heading);

int32_t cell_to_count(int16_t distance);
int16_t count_to_cell(double count);
Point32_t map_cell_to_point(const Cell_t& cell);
Cell_t map_point_to_cell(Point32_t pnt);

void map_set_cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state);

void map_reset(uint8_t id);

void map_copy(uint8_t id,uint8_t **new_map);
/*
 * @author Alvin Xie
 * @brief Convert the ros map to grid map for the pathplanning algorithm.
 * @param is_mark_cleaned to decide if mark the free space to CLENAED
 * @return None
 */
void ros_map_convert(int16_t id, bool is_mark_cleaned, bool is_clear_block, bool is_freshen_map);

unsigned char getCost(std::vector<int8_t> &p_map_data, unsigned int mx, unsigned int my);
void mapToWorld(double origin_x_, double origin_y_, float resolution_, unsigned int mx, unsigned int my, double& wx, double& wy);
bool worldToMap(double origin_x_, double origin_y_, float resolution_, int size_x_, int size_y_, double wx, double wy, unsigned int& mx, unsigned int& my);
unsigned int getIndex(int size_x_, unsigned int mx, unsigned int my);
void indexToCells(int size_x_, unsigned int index, unsigned int& mx, unsigned int& my);
bool worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy);
bool map_mark_robot(uint8_t id);
uint8_t map_set_laser();
uint8_t map_set_obs();
uint8_t map_set_bumper();
//			if (mt_is_follow_wall() || path_get_path_points_count() < 3 || !cm_curve_move_to_point())
uint8_t map_set_rcon();
uint8_t map_set_cliff();
uint8_t map_set_tilt();
uint8_t map_set_slip();
uint8_t map_set_blocked();
void map_set_cleaned(const Cell_t& curr);
void map_set_follow_wall(const Cell_t& curr);
void map_set_cleaned(std::vector<Cell_t>& cells);
void map_set_follow_wall(std::vector<Cell_t>& cells);
uint32_t map_get_cleaned_area();
void map_set_block(const Cell_t &start, const Cell_t &stop,CellState state);
void map_set_block_with_bound(const Cell_t &start, const Cell_t &stop,CellState state);
#endif /* __MAP_H */
