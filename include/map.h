#ifndef __MAP_H__
#define __MAP_H__

#include "config.h"
#include "mathematics.h"
#include <deque>
#include <vector>

#define MAP 0
#define SPMAP 1
#define WFMAP 2
#define ROSMAP 3
#define ROSMAP2 4

typedef enum {
  UNCLEAN  = 0,
  CLEANED = 1,
  BLOCKED = 2,
  BLOCKED_OBS = 2,
  BLOCKED_BUMPER = 3,
  BLOCKED_CLIFF = 4,
  BLOCKED_RCON = 5,
  BLOCKED_LASER = 6,
  BLOCKED_TILT = 7,
  BLOCKED_SLIP = 8,
  BLOCKED_ROS_MAP = 9,
  BLOCKED_BOUNDARY = 10,
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
  NX_NY =-1350,
  NEG_Y =-900,
  PX_NY =-450,
  NONE = 0,
} Direction_Cardinal;

#define IS_POS_AXIS(x) (x == POS_X || x == POS_Y || x == NONE)
#define	IS_X_AXIS(x) (x == NEG_X || x == POS_X || x == NONE)
#define	IS_Y_AXIS(x) (x == POS_Y || x == NEG_Y)

void map_init(uint8_t id);

int32_t map_get_x_count(void);
int32_t map_get_y_count(void);
int16_t map_get_x_cell(void);
int16_t map_get_y_cell(void);

void map_set_position(double x, double y);

int32_t map_get_relative_x(int16_t heading, int16_t dy, int16_t dx, bool using_point_pos);
int32_t map_get_relative_y(int16_t heading, int16_t dy, int16_t dx, bool using_point_pos);

CellState map_get_cell(uint8_t id, int16_t x, int16_t y, bool is_wf_map = false);
Cell_t map_get_curr_cell();
void map_set_cell(uint8_t id, int32_t x, int32_t y, CellState value);

void map_clear_blocks(void);

int32_t cell_to_count(int16_t distance);
int16_t count_to_cell(int32_t count);
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
void ros_map_convert(int16_t id, bool is_mark_cleaned, bool is_clear_block, bool is_freshen_map, uint8_t limit = 100);

void mapToWorld(double origin_x_, double origin_y_, float resolution_, unsigned int mx, unsigned int my, double& wx, double& wy);
bool worldToMap(double origin_x_, double origin_y_, float resolution_, int size_x_, int size_y_, double wx, double wy, unsigned int& mx, unsigned int& my);
unsigned int getIndex(int size_x_, unsigned int mx, unsigned int my);
void indexToCells(int size_x_, unsigned int index, unsigned int& mx, unsigned int& my);
bool worldToCount(double &wx, double &wy, int32_t &cx, int32_t &cy);
bool count_to_world(double &wx, double &wy, int32_t &cx, int32_t &cy);
bool map_mark_robot(uint8_t id);
Cell_t map_update_position();
uint8_t map_set_laser();
uint8_t map_set_obs();
uint8_t map_set_bumper();
//			if (mt_is_follow_wall() || path_get_path_points_count() < 3 || !cm_curve_move_to_point())
uint8_t map_set_rcon();
uint8_t map_set_cliff();
uint8_t map_set_tilt();
uint8_t map_set_slip();
uint8_t map_set_charge_position(const Cell_t homepoint);
uint8_t map_set_blocked();

double world_distance(void);
void map_set_follow_wall(uint8_t id, const Cell_t& curr);
void map_set_cleaned(std::deque<Cell_t>& cells);
uint32_t map_get_cleaned_area();
void fw_marker(const Cell_t&  curr);
#endif /* __MAP_H */
