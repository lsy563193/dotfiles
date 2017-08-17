#ifndef __SHORTESTPATH_H__
#define __SHORTESTPATH_H__

#include <list>

#include "config.h"

#include "stdlib.h"
#include "map.h"
#include "path_planning.h"

using namespace std;

typedef struct{
	int16_t	x;
	int16_t	y;
	uint8_t	x_off;
	uint8_t level;
} LineType;

#ifdef SHORTEST_PATH_V2

void path_position_init(uint8_t dg);
void path_position_update(void);

uint16_t path_line_get_count(void);

#else

#define path_position_init(dg) { do { dg = dg; } while (0); }
#define path_position_update() { do { ; } while (0); }
#define path_line_get_count()	{ do { ; } while (0); }

#endif

int16_t path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound);
int16_t path_next_best(const Cell_t& curr, const Cell_t& target, PPTargetType& path);

int path_get_path_points_count();

list<Cell_t> *path_get_path_points();
void path_reset_path_points();

void path_display_path_points(list<Cell_t> path);

#endif
void set_explore_new_path_flag(bool flag);
int16_t wf_path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound);
int16_t wf_path_find_shortest_path_ranged(int16_t curr_x, int16_t curr_y, int16_t end_x, int16_t end_y, uint8_t bound, int16_t x_min, int16_t x_max, int16_t y_min, int16_t y_max);
