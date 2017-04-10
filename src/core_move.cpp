#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>

#include "main.h"
#include "laser.hpp"
#include "robot.hpp"
#include "core_move.h"

#include "gyro.h"
#include "map.h"
#include "mathematics.h"
#include "path_planning.h"
#include "rounding.h"
#include "shortest_path.h"

#ifdef PP_CURVE_MOVE
#include "curve_move.h"
#endif

#include "movement.h"
#include "wall_follow_multi.h"
#include <ros/ros.h>
#include <vector>
#include <chrono>
#include <functional>
#include <future>
#include <list>
#include <charger.hpp>

//Note that these two value should meet that length can be divided by increment, for example:
//MOVE_TO_CELL_SEARCH_INCREMENT 1, MOVE_TO_CELL_SEARCH_INCREMENT 1
//1 1 1
//1 1 1
//1 1 1
//
//MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH 2, MOVE_TO_CELL_SEARCH_INCREMENT 1
//1 1 1 1 1
//1 1 1 1 1
//1 1 1 1 1
//1 1 1 1 1
//1 1 1 1 1
//
//MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH 2, MOVE_TO_CELL_SEARCH_INCREMENT 2
//1 0 1 0 1
//0 0 0 0 0
//1 0 1 0 1
//0 0 0 0 0
//1 0 1 0 1
//
//MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH 4, MOVE_TO_CELL_SEARCH_INCREMENT 2
//1 0 1 0 1 0 1 0 1
//0 0 0 0 0 0 0 0 0
//1 0 1 0 1 0 1 0 1
//0 0 0 0 0 0 0 0 0
//1 0 1 0 1 0 1 0 1
//0 0 0 0 0 0 0 0 0
//1 0 1 0 1 0 1 0 1
//0 0 0 0 0 0 0 0 0
//1 0 1 0 1 0 1 0 1
//
#define MOVE_TO_CELL_SEARCH_INCREMENT 2
#define MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH 10
#define MOVE_TO_CELL_SEARCH_ARRAY_LENGTH (2 * MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH / MOVE_TO_CELL_SEARCH_INCREMENT + 1)
#define MOVE_TO_CELL_SEARCH_ARRAY_LENGTH_MID_IDX ((MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH - 1) / 2)

extern bool is_line_angle_offset;
int8_t enable_slam_offset = 0;


// This list is for storing the position that robot sees the charger stub.
std::list <Point32_t> Home_Point;
// This is for adding new point to Home Point list.
Point32_t New_Home_Point;

uint8_t map_touring_cancel = 0;

uint8_t	go_home = 0;
uint8_t	remote_go_home = 0;
uint8_t	from_station = 0;
int16_t station_zone = -1;
uint8_t lowBattery = 0;
int16_t map_gyro_offset = 0;
uint8_t tiledUpCount = 0;

uint8_t	should_follow_wall = 0;

Point16_t relativePos[MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH] = {{0, 0}};

extern PositionType positions[];

extern int16_t xMin, xMax, yMin, yMax;

extern volatile uint8_t cleaning_mode;

// This status is for rounding function to decide the angle it should turn.
uint8_t Bumper_Status_For_Rounding;

void CM_count_normalize(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = cellToCount(countToCell(Map_GetRelativeX(heading, offset_lat, offset_long)));
	*y = cellToCount(countToCell(Map_GetRelativeY(heading, offset_lat, offset_long)));
}

//not yet minus the x_off
bool CM_Check_is_exploring()
{
	int	index, plus_sign, a_max;
	float	position_x, position_y, resolution, search_length = 0.10, search_width = 0.303; //unit for meter
	double	yaw, origin_x, origin_y;

	uint32_t	width,  height;

	std::vector<int8_t>	*p_map_data;

	position_x = robot::instance()->robot_get_position_x();
	position_y = robot::instance()->robot_get_position_y();
	yaw = robot::instance()->robot_get_map_yaw();
	yaw = robot::instance()->robot_get_obs_left();
	
	width = robot::instance()->robot_get_width();
	height = robot::instance()->robot_get_height();
	resolution = robot::instance()->robot_get_resolution();
	origin_x = robot::instance()->robot_get_origin_x();
	origin_y = robot::instance()->robot_get_origin_y();
	

	p_map_data = robot::instance()->robot_get_map_data();
	if (abs(yaw) <= (M_PI / 2) ){
		plus_sign = 1;
		a_max = (plus_sign * abs(int(round(search_length * sin(abs(yaw)) / 0.05))));
		for (int a = 0; a <= a_max; a = a + 1) {		//n = (search_length * cos(yaw)) / resolution
			int c = 0;
			float x = position_x + (a / tan(abs(yaw))) * 0.05;
			float y = position_y + a * 0.05;
			for (int b = -int((round(search_width / 0.05)) / 2) + c; b <= int((round(search_width / 0.05)) / 2); b = b + 1){
				float x_1 = x + b * 0.05;
				/*over map scope*/
				if ((x_1 < origin_x ) or (x_1 > (width * 0.05) ) or (y < origin_y) or (y > (height * 0.05))){
					printf("over scope\n");
				} else {
					if ((*p_map_data)[CM_Get_grid_index(x_1, y, width, height, resolution, origin_x, origin_y)] == 100) {
						c++;	//add one grid wall
						if (c >= (search_width / 0.05)){
							printf("exist wall\n");
							return 2;
						}
					} else {
						if ((*p_map_data)[CM_Get_grid_index(x_1, y, width, height, resolution, origin_x, origin_y)] == -1) {
							return 1;
						}
					}
				}
			}
		}

	} else{
		plus_sign = -1;
		a_max = (plus_sign * abs(int(round(search_length * sin(abs(yaw)) / 0.05))));
		for (int a = 0; a >= a_max; a = a - 1) {	//n = (search_length * cos(yaw)) / resolution
			int c = 0;
			float x = position_x + (a / tan(abs(yaw))) * 0.05;
			float y = position_y + a * 0.05;
			for (int b = -int((round(search_width / 0.05)) / 2) + c; b <= int((round(search_width / 0.05)) / 2); b = b + 1) {
				float x_1 = x + b * 0.05;
				/*over map scope*/
				if ((x_1 < origin_x ) or (x_1 > (width * 0.05) ) or (y < origin_y) or (y > (height * 0.05))) {
					printf("over scope\n");
				} else {
					if ((*p_map_data)[CM_Get_grid_index(x_1, y, width, height, resolution, origin_x, origin_y)] == 100) {
						c++;
						if (c >= (search_width / 0.05)) {
							printf("exist wall\n");
							return 2;
						}
					} else {
						if ((*p_map_data)[CM_Get_grid_index(x_1, y, width, height, resolution, origin_x, origin_y)] == -1) {
							printf("exist unkown\n");
							return 1;
						}
					}
				}
			}
		}
	}

	return 0;
}

int CM_Get_grid_index(float position_x, float position_y, uint32_t width, uint32_t height, float resolution, double origin_x, double origin_y )
{
	int index, grid_x, grid_y;

	/*get index*/
	grid_x = int(round((position_x - origin_x) / resolution));
	grid_y = int(round((position_y - origin_y) / resolution));
	index = grid_x + grid_y * width;
	
	return index;
}

int32_t CM_ABS(int32_t A, int32_t B)
{
	return ((A > B) ? (A - B) : (B - A));
}

void CM_update_position(uint16_t heading_0, int16_t heading_1) {
	int8_t	e;
	int16_t c, d, x, y, path_heading;
	int32_t i, j, k;

	float	pos_x, pos_y;

	if (heading_0 > heading_1 && heading_0 - heading_1 > 1800) {
		path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
	} else if (heading_1 > heading_0 && heading_1 - heading_0 > 1800) {
		path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
	} else {
		path_heading = (uint16_t)(heading_0 + heading_1) >> 1;
	}

	x = Map_GetXPos();
	y = Map_GetYPos();

	pos_x = robot::instance()->robot_get_position_x() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->robot_get_position_y() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_SetPosition(pos_x, pos_y);
	if (x != Map_GetXPos() || y != Map_GetYPos()) {
		for (c = 1; c >= -1; --c) {
			for (d = 1; d >= -1; --d) {
				CM_count_normalize(path_heading, CELL_SIZE * c, CELL_SIZE * d, &i, &j);
				e = Map_GetCell(MAP, countToCell(i), countToCell(j));

				if (e == BLOCKED_OBS || e == BLOCKED_BUMPER) {
					printf("%s, %d: warning, reset bumper/obs value, (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(i), countToCell(j));
					Map_SetCell(MAP, i, j, CLEANED);
				}
			}
		}
		robot::instance() -> pub_clean_markers();
	}

#if (ROBOT_SIZE == 5)

	CM_count_normalize(heading_0, -CELL_SIZE_2, CELL_SIZE, &i, &j);
	if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_BOUNDARY) {
		//printf("%s, %d: warning, setting boundary.\n", __FUNCTION__, __LINE__);
	} else {
		Map_SetCell(MAP, i, j, CLEANED);
	}

	for (c = 1; c >= -1; --c) {
		CM_count_normalize(heading_0, c * CELL_SIZE, CELL_SIZE, &i, &j);
		Map_SetCell(MAP, i, j, CLEANED);
	}

	CM_count_normalize(heading_0, CELL_SIZE_2, CELL_SIZE, &i, &j);
	if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_BOUNDARY) {
		//printf("%s, %d: warning, setting boundary.\n", __FUNCTION__, __LINE__);
	} else {
		Map_SetCell(MAP, i, j, CLEANED);
	}

	if (Get_OBS_Status() & Status_Left_OBS) {
		CM_count_normalize(0, heading_0, CELL_SIZE_3, CELL_SIZE, &i, &j);
		if (Get_Wall_ADC(0) > 200) {
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BUMPER) {
				Map_SetCell(MAP, i, j, BLOCKED_BUMPER); //BLOCKED_OBS);
			}
#if 0
		} else {
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_OBS) {
				/* Shall we reset the cell by using the wall sensor if it was marked? */
				Map_SetCell(MAP, i, j, UNCLEAN);
			}
#endif
		}
	}

#else
	for (c = 1; c >= -1; --c) {
		CM_count_normalize(heading_0, c * CELL_SIZE, CELL_SIZE, &i, &j);
		if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_BOUNDARY) {
			//printf("%s, %d: warning, setting boundary.\n", __FUNCTION__, __LINE__);
		} else {
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_BUMPER) {
				printf("%s, %d: warning, setting bumper (%d, %d).\n", __FUNCTION__, __LINE__, countToCell(i), countToCell(j));
			}
			Map_SetCell(MAP, i, j, CLEANED);
		}
	}

	if (Get_OBS_Status() & Status_Left_OBS) {
		CM_count_normalize(heading_0, CELL_SIZE_2, CELL_SIZE, &i, &j);
		if (Get_Wall_ADC(0) > 200) {
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BUMPER) {
				Map_SetCell(MAP, i, j, BLOCKED_BUMPER); //BLOCKED_OBS);
			}
#if 0
		} else {
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_OBS) {
				/* Shall we reset the cell by using the wall sensor if it was marked? */
				Map_SetCell(MAP, i, j, UNCLEAN);
			}
#endif
		}
	}

#endif

	for (c = 0; c < 3; ++c) {
		i = SHRT_MAX;
		switch (c) {
			case 0:
				i = Get_OBS_Status() & Status_Right_OBS;
				break;
			case 1:
				i = Get_OBS_Status() & Status_Front_OBS;
				break;
			case 2:
				i = Get_OBS_Status() & Status_Left_OBS;
				break;
		}

#if (ROBOT_SIZE == 5)
		CM_count_normalize(Gyro_GetAngle(0), (c - 1) * CELL_SIZE, CELL_SIZE_3, &j, &k);
		if (i && Map_GetCell(MAP, countToCell(j), countToCell(k)) != BLOCKED_BUMPER) {
			Map_SetCell(MAP, j, k, BLOCKED_OBS);
		} else if (Map_GetCell(MAP, countToCell(j), countToCell(k)) == BLOCKED_OBS) {
			Map_SetCell(MAP, j, k, UNCLEAN);
		}

#else
		CM_count_normalize(Gyro_GetAngle(0), (c - 1) * CELL_SIZE, CELL_SIZE_2, &j, &k);
		if (i && Map_GetCell(MAP, countToCell(j), countToCell(k)) != BLOCKED_BUMPER) {
			Map_SetCell(MAP, j, k, BLOCKED_OBS);
		} else if (Map_GetCell(MAP, countToCell(j), countToCell(k)) == BLOCKED_OBS) {
			Map_SetCell(MAP, j, k, UNCLEAN);
		}
#endif
	}
}

void CM_update_map_bumper(ActionType action, uint8_t bumper)
{
	int16_t	c;
	int32_t	x_tmp, y_tmp;

#if (ROBOT_SIZE == 5)
	//bumper = Get_Bumper_Status();
	if ((bumper & RightBumperTrig) && (bumper & LeftBumperTrig)) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	} else if (bumper & LeftBumperTrig) {
		if (action == ACTION_LT) {
			CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_3, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			//CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_3, CELL_SIZE, &x_tmp, &y_tmp);
			//Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		} else {
			for (c = 1; c <= 2; ++c) {
				CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_2, CELL_SIZE_3, &x_tmp, &y_tmp);
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	} else if (bumper & RightBumperTrig) {
		if (action == ACTION_RT) {
			CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_3, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			//CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_3, CELL_SIZE, &x_tmp, &y_tmp);
			//Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		} else {
			for (c = -2; c <= -1; ++c) {
				CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE_2, CELL_SIZE_3, &x_tmp, &y_tmp);
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	}
#else

	//bumper = Get_Bumper_Status();
	if ((bumper & RightBumperTrig) && (bumper & LeftBumperTrig)) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	} else if (bumper & LeftBumperTrig) {
		CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_2, CELL_SIZE_2, &x_tmp, &y_tmp);
		printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		if (action == ACTION_LT) {
			//CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			//Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		} else {
			CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			if ((positions[0].x == positions[1].x) && (positions[0].y == positions[1].y) && (positions[0].dir == positions[1].dir) &&
			    (positions[0].x == positions[2].x) && (positions[0].y == positions[2].y) && (positions[0].dir == positions[2].dir)) {
				CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x_tmp, &y_tmp);
				printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	} else if (bumper & RightBumperTrig) {
		CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_2, CELL_SIZE_2, &x_tmp, &y_tmp);
		printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		if (action == ACTION_RT) {
			//CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			//Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		} else {
			CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			if ((positions[0].x == positions[1].x) && (positions[0].y == positions[1].y) && (positions[0].dir == positions[1].dir) &&
			    (positions[0].x == positions[2].x) && (positions[0].y == positions[2].y) && (positions[0].dir == positions[2].dir)) {
				CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x_tmp, &y_tmp);
				printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	}

#endif
}

void CM_update_map(ActionType action, uint8_t bumper) {
	int16_t	c;
	uint16_t i;
	int32_t	x_tmp, y_tmp;

	for (c = 0; c < 3; ++c) {
		i = SHRT_MAX;
		switch (c) {
			case 0:
				i = Get_OBS_Status() & Status_Right_OBS;
				break;
			case 1:
				i = Get_OBS_Status() & Status_Front_OBS;
				break;
			case 2:
				i = Get_OBS_Status() & Status_Left_OBS;
				break;
		}

		CM_count_normalize(Gyro_GetAngle(0), (c - 1) * CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
		if (i) {

#if (ROBOT_SIZE == 5)

			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) != BLOCKED_BUMPER) {
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_OBS);
			}
		} else {
			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) == BLOCKED_OBS) {
				Map_SetCell(MAP, x_tmp, y_tmp, UNCLEAN);
			}
		}
	}

	CM_update_map_bumper(action, bumper);

	if (Get_Cliff_Trig() & Status_Cliff_Front) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(0, Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (Get_Cliff_Trig() & Status_Cliff_Left) {
		CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_2, CELL_SIZE_3, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE_3, CELL_SIZE_2, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
	}
	if (Get_Cliff_Trig() & Status_Cliff_Right) {
		CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_2, CELL_SIZE_3, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE_3, CELL_SIZE_2, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
	}

#else
			CM_count_normalize(Gyro_GetAngle(0), (c - 1) * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) != BLOCKED_BUMPER) {
				printf("%s %d: marking (%d, %d)\n", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_OBS);
			}
		} else {
			CM_count_normalize(Gyro_GetAngle(0), (c - 1) * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) == BLOCKED_OBS) {
				Map_SetCell(MAP, x_tmp, y_tmp, UNCLEAN);
			}
		}
	}

	CM_update_map_bumper(action, bumper);

	if (Get_Cliff_Trig() & Status_Cliff_Front) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (Get_Cliff_Trig() & Status_Cliff_Left) {
		for (c = 1; c <= 2; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (Get_Cliff_Trig() & Status_Cliff_Right) {
		for (c = -2; c <= -1; ++c) {
			CM_count_normalize(Gyro_GetAngle(0), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
#endif
}

/*--------------------------Head Angle--------------------------------*/
void CM_HeadToCourse(uint8_t Speed, int16_t Angle)
{
	int16_t Diff = 0;
	uint32_t SpeedUp, Tick;
	ActionType action = ACTION_NONE;
	uint8_t isBumperTriggered;
	uint8_t Motor_Check_Code = 0;
	static int16_t	angle_turned = 0;

	SpeedUp = Tick = 0;

	Diff = Angle - Gyro_GetAngle(0);

	printf("\n%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\tBias: %d\tTemp: %d\tScale: %d\n",
	         __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff, (Angle - Gyro_GetAngle(0)), Gyro_GetXAcc(), Gyro_GetYAcc(), Gyro_GetZAcc());

	while (Diff >= 1800) {
		Diff = Diff - 3600;
	}

	while (Diff <= (-1800)) {
		Diff = Diff + 3600;
	}

	if ((Diff < 10) && (Diff > (-10))) {
		return;
	}

	printf("\n%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\tangle_turned: %d\tBias: %d\tTemp: %d\tScale: %d\n",
	         __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff, (Angle - Gyro_GetAngle(0)), angle_turned, Gyro_GetXAcc(), Gyro_GetYAcc(), Gyro_GetZAcc());

	if (((Diff <= 1800 && Diff >= 1700) || (Diff >= -1800 && Diff <= -1700))) {
		if (Diff <= 1800 && Diff >= 1700) {
			if (angle_turned < 0) {
				printf("%s %d: Turn Left\n", __FUNCTION__, __LINE__);

				Set_Dir_Left();
				action = ACTION_LT;
				angle_turned += Diff;
			} else {
				printf("%s %d: Turn Right\n", __FUNCTION__, __LINE__);

				Set_Dir_Right();
				action = ACTION_RT;
				angle_turned += (Diff - 3600);
			}
		} else {
			if (angle_turned > 0) {
				printf("%s %d: Turn Right\n", __FUNCTION__, __LINE__);

				Set_Dir_Right();
				action = ACTION_RT;
				angle_turned += Diff;
			} else {
				printf("%s %d: Turn Left\n", __FUNCTION__, __LINE__);

				Set_Dir_Left();
				action = ACTION_LT;
				angle_turned += (3600 + Diff);
			}
		}
	} else {
		if ((Diff >= 0) && (Diff <= 1800)) {	// turn right
			printf("%s %d: Turn Left\n", __FUNCTION__, __LINE__);

			Set_Dir_Left();
			action = ACTION_LT;
		} else if ((Diff <= 0) && (Diff >= (-1800))) {
			printf("%s %d: Turn Right\n", __FUNCTION__, __LINE__);

			Set_Dir_Right();
			action = ACTION_RT;
		}
		angle_turned += Diff;
	}

	Stop_Brifly();
	Reset_TempPWM();
	Set_Wheel_Speed(0, 0);

	SpeedUp = 4;
	while (1) {
		Motor_Check_Code = Check_Motor_Current();
		if (Motor_Check_Code) {
			if (Self_Check(Motor_Check_Code)) {
				Stop_Brifly();
				CM_TouringCancel();
				Set_Touch();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				printf("%s %d: motor(s) error break!\n", __FUNCTION__, __LINE__);
				return;
			}
		}

		if (Touch_Detect()) {
			Set_Touch();
			CM_TouringCancel();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			printf("%s %d: touch detect break!\n", __FUNCTION__, __LINE__);
			return;
		}
		if (Remote_Key(Remote_Max)) {
			if (lowBattery == 0) {
				Switch_VacMode();
			}
		}
		if (Remote_Key(Remote_Home) && go_home == 0) {
			Stop_Brifly();
			Set_BLDC_Speed(Vac_Speed_NormalL);
			Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power);
			Set_Clean_Mode(Clean_Mode_GoHome);
			printf("%s %d: remote home is pressed.\n", __FUNCTION__, __LINE__);

			CM_SetGoHome(1);
			return;
		}

		if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right)) {
			printf("%s %d: robot is taken up break! \n", __FUNCTION__, __LINE__);
			return;
		}

		isBumperTriggered = Get_Bumper_Status();
		if (isBumperTriggered) {
			Stop_Brifly();
			CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));
			CM_update_map(action, isBumperTriggered);

			printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
			CM_CorBack(COR_BACK_20MM);
			Stop_Brifly();
			CM_update_map(action, isBumperTriggered);

			Diff = Angle - Gyro_GetAngle(0);
			while (Diff >= 1800) {
				Diff = Diff - 3600;
			}

			while (Diff <= (-1800)) {
				Diff = Diff + 3600;
			}

			printf("\n%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\n", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff, (Angle - Gyro_GetAngle(0)));
			if ((Diff >= 0) && (Diff <= 1800)) {	// turn right
				printf("Turn Left\n");

				Set_Dir_Left();
				action = ACTION_LT;
			} else if ((Diff <= 0) && (Diff >= (-1800))) {
				printf("Turn Right\n");

				Set_Dir_Right();
				action = ACTION_RT;
			}

			Reset_TempPWM();
			Set_Wheel_Speed(0, 0);

#ifdef BUMPER_ERROR
			if (Get_Bumper_Status()) {
				printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_20MM);

				Diff = Angle - Gyro_GetAngle(0);
				while (Diff >= 1800) {
					Diff = Diff - 3600;
				}

				while (Diff <= (-1800)) {
					Diff = Diff + 3600;
				}

				printf("\n%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\n", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff, (Angle - Gyro_GetAngle(0)));
				if ((Diff >= 0) && (Diff <= 1800)) {	// turn right
					printf("Turn Left\n");

					Set_Dir_Left();
					action = ACTION_LT;
				} else if ((Diff <= 0) && (Diff >= (-1800))) {
					printf("Turn Right\n");

					Set_Dir_Right();
					action = ACTION_RT;
				}
				if (Get_Bumper_Status()) {
					CM_CorBack(COR_BACK_20MM);

					Diff = Angle - Gyro_GetAngle(0);
					while (Diff >= 1800) {
						Diff = Diff - 3600;
					}

					while (Diff <= (-1800)) {
						Diff = Diff + 3600;
					}

					printf("\n%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\n", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff, (Angle - Gyro_GetAngle(0)));
					if ((Diff >= 0) && (Diff <= 1800)) {	// turn right
						printf("Turn Left\n");

						Set_Dir_Left();
						action = ACTION_LT;
					} else if ((Diff <= 0) && (Diff >= (-1800))) {
						printf("Turn Right\n");

						Set_Dir_Right();
						action = ACTION_RT;
					}
					if (Get_Bumper_Status()) {
						printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);
						Set_Error_Code(Error_Code_Bumper);
						Stop_Brifly();
						return;
					}
				}
			}

#endif
		}

		Diff = CM_ABS(Angle, Gyro_GetAngle(0));
		Diff = Diff > 1800 ? 3600 - Diff : Diff;
		if ((Diff < 10) && (Diff > (-10))) {
			Stop_Brifly();
			CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));

			printf("%s %d: Angle: %d\tGyro: %d\tDiff: %d\n", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(0), Diff);
			return;
		}

		Tick++;
		if (Tick > 2) {
			Tick = 0;
			if (Diff > 350) {// acceleration and deceleration angle = 10
				SpeedUp += 1;
				SpeedUp = SpeedUp > Speed ? Speed : SpeedUp;
			} else {
				SpeedUp -= 1;
				SpeedUp = SpeedUp < 7 ? 7 : SpeedUp;
			}
		}
		Set_Wheel_Speed(SpeedUp, SpeedUp);

		usleep(10000);
	}
}

// Target:	robot coordinate
MapTouringType CM_LinearMoveToPoint(Point32_t Target, int32_t speed_max, bool stop_is_needed, bool rotate_is_needed)
{
	int32_t Target_Course, Rotate_Angle, Integrated, Left_Speed, Right_Speed, Base_Speed, distance, Dis_From_Init;
	uint8_t Integration_Cycle, boundary_reach;
	uint32_t Tick = 0;
	ActionType action = ACTION_NONE;

	uint8_t Motor_Check_Code = 0;

	uint8_t isBumperTriggered;

	int8_t	HomeT, HomeL, HomeR, home_hit;
	//int32_t	front_obs_val = 0;
	uint32_t Temp_Rcon_Status;

	int8_t	slow_down;
	int16_t	i;
	//int8_t c;
	int32_t x, y;

        int32_t Init_Pose_X, Init_Pose_Y;
        int16_t Limited_Distance = 16107;//21476 = 4M 16107 = 3M
        int8_t Limited_Flag = 0;
	bool Dynamic_Flag = 1;//Dynamic adjust speed when exploring

	MapTouringType	retval = MT_None;

	Point32_t position;

	position.X = Map_GetXCount();
	position.Y = Map_GetYCount();

	// Reset the Bumper_Status_For_Rounding.
	Bumper_Status_For_Rounding = 0;

	should_follow_wall = 0;

	set_gyro(1, 0);
	usleep(10000);
	Set_IMU_Status();
	//10 second

	Reset_Rcon_Status();
	HomeT = HomeL = HomeR = home_hit = slow_down = 0;
	Integration_Cycle = 0;
	Target_Course = Rotate_Angle = Integrated = Left_Speed = Right_Speed = 0;
	Base_Speed = BASE_SPEED;

	CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));

	if (rotate_is_needed == true) {
		Target_Course = course2dest(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y);
		printf("Target_Course(%d)\n",Target_Course);
		CM_HeadToCourse(ROTATE_TOP_SPEED, Target_Course);	//turn to target position
		printf("leave CM_HeadToCourse\n");
	}

	ROS_DEBUG("%s %d: original target (%d, %d)\n", __FUNCTION__, __LINE__, Target.X, Target.Y);
	if (position.X != Map_GetXCount() && position.X == Target.X) {
		Target.X = Map_GetXCount();
	} else if (position.Y != Map_GetYCount() && position.Y == Target.Y) {
		Target.Y = Map_GetYCount();
	}
	ROS_DEBUG("%s %d: new target (%d, %d)\n", __FUNCTION__, __LINE__, Target.X, Target.Y);

	if (Touch_Detect()) {
		printf("%s %d: Gyro Calibration: %d\n", __FUNCTION__, __LINE__, Gyro_GetCalibration());
		set_gyro(1, 1);
		usleep(10000);

		return MT_Key_Clean;
	}
	if (Get_Clean_Mode() == Clean_Mode_GoHome) {
		printf("%s %d: Gyro Calibration: %d\n", __FUNCTION__, __LINE__, Gyro_GetCalibration());
		set_gyro(1, 1);
		usleep(10000);

		Set_Clean_Mode(Clean_Mode_Navigation);
		return MT_Remote_Home;
	}

	//usleep(1000);
	CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));

	if (Get_LeftBrush_Stall())Set_LeftBrush_Stall(0);
	if (Get_RightBrush_Stall())Set_RightBrush_Stall(0);

	if (go_home == 1) {
		Set_VacMode(Vac_Normal);
		Set_BLDC_Speed(Vac_Speed_NormalL);
	}
	else {
		Set_Vac_Speed();
	}
	
	Init_Pose_X = Map_GetXCount();
	Init_Pose_Y = Map_GetYCount();
	while (1) {
		
#ifdef WALL_DYNAMIC
		Wall_Dynamic_Base(50);
#endif
#ifdef OBS_DYNAMIC_MOVETOTARGET
		/* Dyanmic adjust obs trigger val . */
		OBS_Dynamic_Base(100);
#endif

		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code = Check_Motor_Current();
		if (Motor_Check_Code) {
			if (Self_Check(Motor_Check_Code)) {
				Stop_Brifly();
				CM_TouringCancel();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				ROS_DEBUG("%s %d: Check: Clean Mode! break\n", __FUNCTION__, __LINE__);
				retval = MT_Key_Clean;
				break;
			}
			else
			{
				Base_Speed = BASE_SPEED;
			}
		}

		/* Check low battery event, if battery is low, stop. */
		if (go_home == 1) {
			if (Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power) && go_home != 1 ) {
				Stop_Brifly();
				printf("%s %d: low battery, battery < 1200 is detected.\n", __FUNCTION__, __LINE__);
				retval = MT_Battery;
				break;
			}
		} else {
			if (Check_Bat_SetMotors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power) && go_home != 1 ) {
				Stop_Brifly();
				printf("%s %d: low battery, battery < 1200 is detected.\n", __FUNCTION__, __LINE__);
				retval = MT_Battery;
				break;
			}
		}

		if ((retval = CM_handleExtEvent()) != MT_None) {
			ROS_DEBUG("%s %d: Check: Clean Mode! break\n", __FUNCTION__, __LINE__);
			break;
		}

		/* Check cliff event.*/
		if (Get_Cliff_Trig()) {
			ROS_INFO("Cliff Trig");
			Set_Wheel_Speed(0, 0);
			Set_Dir_Backward();
			usleep(300);
			if (Get_Cliff_Trig()) {
				ROS_INFO("Cliff back 1st time.");
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);

				printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_20MM);
#ifdef CLIFF_ERROR

				if (Get_Cliff_Trig()) {
					ROS_INFO("Cliff back 2nd time.");
					if (Get_Cliff_Trig() == Status_Cliff_All) {
						ROS_DEBUG("%s %d: all cliffs are triggered\n", __FUNCTION__, __LINE__);
						Stop_Brifly();
						retval = MT_Key_Clean;
						break;
					}
					printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
					CM_CorBack(COR_BACK_20MM);
					if (Get_Cliff_Trig()) {
						ROS_INFO("Cliff back 3rd time.");
						if (Get_Cliff_Trig() == Status_Cliff_All) {
							ROS_DEBUG("%s %d: all cliffs are triggered\n", __FUNCTION__, __LINE__);
							Stop_Brifly();
							retval = MT_Key_Clean;
							break;
						}
						printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);
						Set_Error_Code(Error_Code_Cliff);
						Stop_Brifly();
						retval = MT_Key_Clean;
						break;
					}
				}
#endif

				Stop_Brifly();

				should_follow_wall = 1;
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);
				retval = MT_None;
				printf("%s %d: cliff break!\n", __FUNCTION__, __LINE__);
				break;
			}
		}

#ifdef ENABLE_TILTED_DETECT

		if (Get_Left_CLiff_Value() < TILTED_CLIFF_LIMIT && Get_Right_CLiff_Value() < TILTED_CLIFF_LIMIT && Get_Front_CLiff_Value() < TILTED_CLIFF_LIMIT) {
			if (abs((int) (atan(((double)Gyro_GetXAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT ||
				abs((int) (atan(((double)Gyro_GetYAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT) {

				printf("%s %d: possible tiled.\n", __FUNCTION__, __LINE__);

				Set_Wheel_Speed(0, 0);
				Set_Dir_Backward();
				usleep(300);
				CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));

				if (abs((int) (atan(((double)Gyro_GetXAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT ||
					abs((int) (atan(((double)Gyro_GetYAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT) {

					printf("%s %d: confirmed tiled.\n", __FUNCTION__, __LINE__);
					i = 0;
					do {
						printf("%s %d: moving back count: %d\n", __FUNCTION__, __LINE__, i);
						if (abs((int) (atan(((double)Gyro_GetXAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT ||
							abs((int) (atan(((double)Gyro_GetYAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT) {
							CM_CorBack(COR_BACK_100MM);
							Stop_Brifly();

#if (ROBOT_SIZE == 5)

							CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
							CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
							CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);

#else

							CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
							CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
							CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);

#endif
						} else {
							ROS_DEBUG("%s %d: robot is not tiled anymore\n", __FUNCTION__, __LINE__);
							break;
						}
						i++;
					} while (i < 5);

					retval = MT_None;
					break;
				}
			}
		}
#endif

		Temp_Rcon_Status = Get_Rcon_Status() & (RconFL_HomeT | RconFR_HomeT | RconL_HomeT | RconR_HomeT);
		if (go_home == 0 && Temp_Rcon_Status) {
			// It just clear the bits that are 1 in Temp_Rcon_Status has.
			Set_Rcon_Status(Get_Rcon_Status() & (~Temp_Rcon_Status));
			if (Temp_Rcon_Status & (RconFR_HomeT | RconFL_HomeT)) {
				HomeT++;
			}
			if (Temp_Rcon_Status & RconL_HomeT) {
				HomeL++;
			}
			if (Temp_Rcon_Status & RconR_HomeT) {
				HomeR++;
			}

			// If detect charger stub then print the detection info
			if (HomeL || HomeR || HomeT){
				printf("%s %d: home detected (%d %d %d)\n", __FUNCTION__, __LINE__, HomeL, HomeT, HomeR);
			}

			if (HomeR + HomeL + HomeT > 4) {
				home_hit = HomeR > HomeL ? HomeR : HomeL;
				home_hit = home_hit > HomeT ? home_hit : HomeT;

				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);
				if (home_hit == HomeR) {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else if (home_hit == HomeL) {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				}
				Stop_Brifly();
				retval = MT_None;
				// Update the location of charger stub
				CM_SetHome(Map_GetXCount(), Map_GetYCount());

				ROS_DEBUG("%s %d: all charge signal is detected\n", __FUNCTION__, __LINE__);
				break;
			} else if (HomeT == 0 && (HomeR > 2 || HomeL > 2)) {
				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);

				if (HomeR > 2) {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(0), -CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else if (HomeL > 2) {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(0), CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				}
				Stop_Brifly();
				retval = MT_None;
				// Update the location of charger stub
				CM_SetHome(Map_GetXCount(), Map_GetYCount());

				ROS_DEBUG("%s %d: left/right charger signal detected\n", __FUNCTION__, __LINE__);
				break;
			} else if (HomeR == 0 && HomeL == 0 && HomeT > 3) {
				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);

#if (ROBOT_SIZE == 5)
				CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_3, &x, &y);
#else
				CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_2, &x, &y);
				// Mark CELL_SIZE_3 to avoid position jump caused by slam
				CM_count_normalize(Gyro_GetAngle(0), 0, CELL_SIZE_3, &x, &y);
#endif
				Map_SetCell(MAP, x, y, BLOCKED_BUMPER);

				Stop_Brifly();
				retval = MT_None;
				// Update the location of charger stub
				CM_SetHome(Map_GetXCount(), Map_GetYCount());

				ROS_DEBUG("%s %d: top charger signal is detected\n", __FUNCTION__, __LINE__);
				break;
			}
		}

		Reset_Rcon_Status();


		if (Get_FrontOBS() > Get_FrontOBST_Value()) {
			Stop_Brifly();
			isBumperTriggered = Get_Bumper_Status();
			CM_update_map(action, isBumperTriggered);
			retval = MT_None;
			should_follow_wall = 1;
			printf("%s %d: OBS break, val: %d(%d)\n", __FUNCTION__, __LINE__, Get_FrontOBS(), Get_FrontOBST_Value());
			break;
		}

		/* Check bumper event.*/
		isBumperTriggered = Get_Bumper_Status();
		if (isBumperTriggered) {
			// Mark the status for rounding function.
			Bumper_Status_For_Rounding = isBumperTriggered;

			Stop_Brifly();
			CM_update_map_bumper(action, isBumperTriggered);
			robot::instance()->pub_bumper_markers();

			printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
			CM_CorBack(COR_BACK_20MM);

#ifdef BUMPER_ERROR
			if (Get_Bumper_Status()) {
				printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_20MM);
				if (Get_Bumper_Status()) {
					CM_CorBack(COR_BACK_20MM);
					if (Get_Bumper_Status()) {
						printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);
						Set_Error_Code(Error_Code_Bumper);
						Stop_Brifly();
						// If bumper jam, wait for manual release and it can keep on.(Convenient for testing)
						//retval = MT_Key_Clean;
						printf("%s %d: bumper jam break! Please manual release the bumper!\n", __FUNCTION__, __LINE__);
						while (Get_Bumper_Status()){
							// Sleep for 2s and detect again, and beep to alarm in the first 0.5s
							Beep(3, 25, 0, 1);
							usleep(2000000);
						}
						//break;
					}
				}
			}

#endif

			Stop_Brifly();
			//isBumperTriggered = Get_Bumper_Status();
			CM_update_map(action, isBumperTriggered);
			retval = MT_None;
			should_follow_wall = 1;
			printf("%s %d: bumper break!\n", __FUNCTION__, __LINE__);
			break;
		}

		if (CM_ABS(Map_GetXCount(), Target.X) < 150 && CM_ABS(Map_GetYCount(), Target.Y) < 150) {
			ROS_INFO("%s, %d: Reach target.", __FUNCTION__, __LINE__);
			isBumperTriggered = Get_Bumper_Status();
			CM_update_map(action, isBumperTriggered);

			retval = MT_None;
			break;
		}
#if LIMIT_DISTANCE_ENABLE
		/*Check limited distance in one straight movement*/
		if ((Dis_From_Init = TwoPointsDistance(Map_GetXCount(), Map_GetYCount(), Init_Pose_X, Init_Pose_Y)) > Limited_Distance) {
			printf("reach the limited distance\n");
			printf("Map_XCount=%d,Map_YCount=%d,Init_Pose_X=%d,Init_Pose_Y=%d,Dis_From_Init=%d,Limited_Distance=%d\n",Map_GetXCount(),Map_GetYCount(),Init_Pose_X, Init_Pose_Y, Dis_From_Init, Limited_Distance);
			Limited_Flag = 3;//Limit distance flag
			printf("after sleep");
			Init_Pose_X = Map_GetXCount();
			Init_Pose_Y = Map_GetYCount();
		}
#endif

#if EXPLORE_SCOPE_ENABLE 
		/*Check if in exploring status*/
		if (Dynamic_Flag == 1) {//Dynamic adjust speed when exploring
			if (Limited_Flag != 3) {//not in distance limit
				if (bool Explore_Flag = CM_Check_is_exploring() == 1) {
					Limited_Flag = 1;
				} else if (Explore_Flag == 2) {
					Limited_Flag = 2;
				} else if (Explore_Flag == 0) {
					Limited_Flag = 0;
				}
			}
		} else {	//Adjust once when exploring
			if (Limited_Flag != 1) {
				if (Limited_Flag != 3) {	//not in distance limit
					if (bool Explore_Flag = CM_Check_is_exploring() == 1) {
						Limited_Flag = 1;
					} else if (Explore_Flag == 2) {
						Limited_Flag = 2;
					} else if (Explore_Flag == 0) {
						Limited_Flag = 0;
					}
				}
			}
		}
#endif

		CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));

#if 1
		/* Check map boundary. */
		boundary_reach = 0;
		for (i = -1; boundary_reach == 0 && i <= 1; i++) {
#if (ROBOT_SIZE == 5)
			CM_count_normalize(Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_3, &x, &y);
#else

			CM_count_normalize(Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_3, &x, &y);
			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
				slow_down = 1;
			}

			CM_count_normalize(Gyro_GetAngle(0), i * CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif

			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
				ROS_INFO("%s, %d: Blocked boundary.", __FUNCTION__, __LINE__);
				boundary_reach = 1;
				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);
				retval = MT_None;
				break;
			}
		}

		if (boundary_reach == 1) {
			printf("%s %d: boundary break!\n", __FUNCTION__, __LINE__);
			break;
		}
#endif

		/*--------------------------Adjust Move ------------------------------------*/
		Rotate_Angle = course2dest(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y) - Gyro_GetAngle(0);

		if (Rotate_Angle >= 1800) {
			Rotate_Angle -= 3600;
		} else if (Rotate_Angle <= -1800) {
			Rotate_Angle += 3600;
		}
		if (abs(Rotate_Angle) > 300) {
			printf("%s %d: warning: angle is too big, angle: %d\n\n", __FUNCTION__, __LINE__, Rotate_Angle);
			break;
		}

		Integration_Cycle++;
		if (Integration_Cycle > 10) {
			Integration_Cycle = 0;
			Integrated += Rotate_Angle;
			if (Integrated > 150) {
				Integrated = 150;
			} else if (Integrated < -150) {
				Integrated = -150;
			}
		}

		distance = TwoPointsDistance(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y);

		if (Get_OBS_Status()) {
			Integrated = 0;
			Rotate_Angle = 0;
			Base_Speed -= 5;
			Base_Speed = Base_Speed < BASE_SPEED ? BASE_SPEED : Base_Speed;
		}
		else if (Is_OBS_Near()) {
			Integrated = 0;
			Rotate_Angle = 0;
			Base_Speed -= 4;
			Base_Speed = Base_Speed < BASE_SPEED ? BASE_SPEED : Base_Speed;
		}
		else if ((distance < SLOW_DOWN_DISTANCE) || slow_down) {
			Integrated = 0;
			Rotate_Angle = 0;
			Base_Speed -= 3;
			Base_Speed = Base_Speed < BASE_SPEED ? BASE_SPEED : Base_Speed;
		}
		else if (laser::instance()->laser_obstcal_detected(0.2, 0, -1.0) == true) {
			//printf("%s %d: laser detected obstcal, slow down!\n", __FUNCTION__, __LINE__);
			Integrated = 0;
			Rotate_Angle = 0;
			Base_Speed -= 3;
			Base_Speed = Base_Speed < BASE_SPEED ? BASE_SPEED : Base_Speed;
		}
		else if (Base_Speed < (int32_t) speed_max) {
			Tick++;
			if (Tick > 5) {
				Tick = 0;
				Base_Speed += 1;
			}
			Integrated = 0;
		}

		Left_Speed = Base_Speed - Rotate_Angle / 10 - Integrated / 150; // - Delta / 20; // - Delta * 10 ; // - Integrated / 2500;
		Right_Speed = Base_Speed + Rotate_Angle / 10 + Integrated / 150; // + Delta / 20;// + Delta * 10 ; // + Integrated / 2500;

		if (Left_Speed < BASE_SPEED) {
			Left_Speed = BASE_SPEED;
		} else if (Left_Speed > speed_max) {
			Left_Speed = speed_max;
		}

		if (Right_Speed < BASE_SPEED) {
			Right_Speed = BASE_SPEED;
		} else if (Right_Speed > speed_max) {
			Right_Speed = speed_max;
		}

		if (Limited_Flag == 3){
			Move_Forward(Left_Speed / 2, Right_Speed / 2);
		}
		else if (Limited_Flag == 2){
			Move_Forward(Left_Speed, Right_Speed);
		} else{
			if (Limited_Flag == 0){
				Move_Forward(Left_Speed, Right_Speed);
			} else if(Limited_Flag == 1){
				Move_Forward(Left_Speed / 2, Right_Speed / 2);
			} 
		}
		Base_Speed = (Left_Speed + Right_Speed) / 2;

		usleep(10000);
	}

	if (stop_is_needed == true) {
		Stop_Brifly();
	}
	CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));

	printf("%s %d: move to point: %d\tGyro Calibration: %d\n", __FUNCTION__, __LINE__, retval, Gyro_GetCalibration());
	set_gyro(1, 1);
	robot::instance()->robot_display_positions();
	usleep(10000);

	return retval;
}


MapTouringType CM_MoveToPoint(Point32_t target)
{
	MapTouringType mt_state = MT_None;

#ifdef PP_CURVE_MOVE

	if (path_get_path_points_count() >= 3) {
		mt_state = CurveMove_MoveToPoint();
		if (mt_state == MT_CurveMove) {
			mt_state = CM_LinearMoveToPoint(target, RUN_TOP_SPEED, true, true);
		}
	} else {
		printf("%s %d: Normal move to next point at east.\n", __FUNCTION__, __LINE__);
		mt_state = CM_LinearMoveToPoint(target, RUN_TOP_SPEED, true, true);
	}

#else

	printf("%s %d: Normal move to next point at east.\n", __FUNCTION__, __LINE__);
	mt_state = CM_LinearMoveToPoint(target, RUN_TOP_SPEED, true, true);

#endif

	return mt_state;
}

#if (PP_ROUNDING_OBSTACLE_LEFT) || (PP_ROUNDING_OBSTACLE_RIGHT)
uint16_t CM_get_robot_direction()
{
	uint16_t	dir;

	dir = (uint16_t) round(((double)Gyro_GetAngle(0)) / 100);
	switch(dir) {
		case 0:
		case 36:
			dir = NORTH;
			break;
		case 9:
			dir = EAST;
			break;
		case 18:
			dir = SOUTH;
			break;
		case 27:
			dir = WEST;
			break;
		default:
			break;
	}
	return dir;
}

RoundingType CM_get_rounding_direction(Point32_t *Next_Point, Point32_t Target_Point) {
	int32_t		y_coordinate;
	uint16_t	dir;
	RoundingType	rounding_type = ROUNDING_NONE;

	ROS_INFO("Enter rounding detection.");
	dir = CM_get_robot_direction();
	if (should_follow_wall == 0) {
		return rounding_type;
	}

	if (Get_Cliff_Trig()) {
		printf("%s %d, cliff triggered.\n", __FUNCTION__, __LINE__);
	} else if (Get_Bumper_Status()) {
		printf("%s %d, bumper event.\n", __FUNCTION__, __LINE__);
	} else if (Get_OBS_Status()) {
		printf("%s %d, OBS detected.\n", __FUNCTION__, __LINE__);
	} else if (Get_FrontOBS() > Get_FrontOBST_Value()) {
		printf("%s %d, front OBS detected.\n", __FUNCTION__, __LINE__);
	} else if (Get_Wall_ADC(0) > 170) {
		printf("%s %d, wall sensor exceed 170.\n", __FUNCTION__, __LINE__);
	}
	/*                South (Xmin)
	 *                     |
	 *  West (Ymin)  --  robot  --   East (Ymax)
	 *                     |
	 *                North (Xmax)
	**/
	y_coordinate = countToCell(Next_Point->Y);
	if (y_coordinate != Map_GetYPos()) {
		// Robot need to go to new line
		ROS_INFO("Robot need to go to new line");

#if PP_ROUNDING_OBSTACLE_LEFT
		if ((dir == NORTH && y_coordinate < Map_GetYPos() && (y_coordinate == Map_GetYPos() - 1 || y_coordinate == Map_GetYPos() - 2)) ||
			(dir == SOUTH && y_coordinate > Map_GetYPos() && (y_coordinate == Map_GetYPos() + 1 || y_coordinate == Map_GetYPos() + 2 ))) {

			rounding_type = ROUNDING_LEFT;
		}
#endif

#if PP_ROUNDING_OBSTACLE_RIGHT
		if ((dir == NORTH && y_coordinate > Map_GetYPos() && (y_coordinate == Map_GetYPos() + 1 || y_coordinate == Map_GetYPos() + 2)) ||
			(dir == SOUTH && y_coordinate < Map_GetYPos() && (y_coordinate == Map_GetYPos() - 1 || y_coordinate == Map_GetYPos() - 2))) {

			rounding_type = ROUNDING_RIGHT;
		}
#endif
	} else {
		printf("%s %d Robot don't need to go to new line.%d %d %d\n", __FUNCTION__, __LINE__, y_coordinate, SHRT_MAX, SHRT_MIN);
		if (!(y_coordinate == SHRT_MAX || y_coordinate == SHRT_MIN)) {
			y_coordinate = countToCell(Target_Point.Y);
			if (y_coordinate != Map_GetYPos()) {

#if PP_ROUNDING_OBSTACLE_LEFT
				if ((dir == NORTH && y_coordinate < Map_GetYPos() && (y_coordinate == Map_GetYPos() - 1 || y_coordinate == Map_GetYPos() - 2)) || 
					(dir == SOUTH && y_coordinate > Map_GetYPos() && (y_coordinate == Map_GetYPos() + 1 || y_coordinate == Map_GetYPos() + 2 ))) {

					rounding_type = ROUNDING_LEFT;
					Next_Point->Y = Target_Point.Y;
				}
#endif

#if PP_ROUNDING_OBSTACLE_RIGHT
				if ((dir == NORTH && y_coordinate > Map_GetYPos() && (y_coordinate == Map_GetYPos() + 1 || y_coordinate == Map_GetYPos() + 2)) ||
					(dir == SOUTH && y_coordinate < Map_GetYPos() && (y_coordinate == Map_GetYPos() - 1 || y_coordinate == Map_GetYPos() - 2))) {

					rounding_type = ROUNDING_RIGHT;
					Next_Point->Y = Target_Point.Y;
				}
#endif

			}
		}
	}
	return rounding_type;
}

#endif

void start_obstacle_detector(void)
{
	system("roslaunch pp obstacle_detector.launch 2>/dev/null &");
}

void stop_obstacle_detector(void)
{
	system("rosnode kill /obstacle_detector 2>/dev/null &");
}

void start_slam(void)
{
	robot::instance()->start_slam();
}

void show_time(std::function<void(void)> task){
	auto gyro_start = std::chrono::system_clock::now();
	task();
	auto diff = std::chrono::system_clock::now() - gyro_start;
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
	std::cout <<"this task runs:" << ms.count() << " ms" << std::endl;
}

class Motion_controller {
public:
	Motion_controller()
	{
		robot::instance()->Subscriber();
		Work_Motor_Configure();
		if (robot::instance()->align_active() == true)
		{
			start_obstacle_detector();
		}

		robot::instance()->start_lidar();

		if (robot::instance()->align_active() == true)
		{
			robot::instance()->align();
			stop_obstacle_detector();
		}

		enable_slam_offset = 1;
		start_slam();

	};

	~Motion_controller()
	{
		Disable_Motors();
		robot::instance()->stop_lidar();
		if (robot::instance()->align_active())
		{
			robot::instance()->align_exit();
			stop_obstacle_detector();
		}
		show_time(Set_gyro_off);
		Reset_IMU_Status();
		is_line_angle_offset = false;
		enable_slam_offset = 0;
		robot::instance()->stop_slam();
		robot::instance()->UnSubscriber();
	}
};
uint8_t CM_Touring(void)
{
	int8_t	state;
	uint8_t Blink_LED = 0;
	int16_t	i, x, y, x_current, y_current, start, end;
	float	slop, intercept;

	// X, Y in Target_Point are all counts.
	Point32_t	Next_Point, Target_Point;
	Point16_t	tmpPnt, pnt16ArTmp[3];


	MapTouringType	mt_state = MT_None;
	RoundingType	rounding_type;

	// Reset battery status
	lowBattery = 0;
	tiledUpCount = 0;

	Reset_Rcon_Status();
	Reset_Touch();

	station_zone = -1;
	from_station = 0;
	map_touring_cancel = go_home = remote_go_home = 0;

	Reset_Touch();
	Reset_MoveWithRemote();
	Set_LED(100, 0);
	/*Move back from charge station*/
	if (Is_AtHomeBase()) {
		printf("%s %d: calling moving back\n", __FUNCTION__, __LINE__);
		Set_SideBrush_PWM(30, 30);
		// Reset the robot to non charge mode.
		set_stop_charge();
		// Debug
		while (Is_ChargerOn())
		{
			ROS_INFO("Robot Still charging.");
			usleep(20000);
		}
		// Sleep for 30ms to make sure it has sent at least one control message to stop charging.
		usleep(30000);
		if (Is_ChargerOn()){
			printf("[core_move.cpp] Still charging.\n");
		}
		// Beep while moving back.
		Beep(3, 25, 25, 6);
		// Set i < 7 for robot to move back for approximately 500mm.
		for (i = 0; i < 7; i++) {
			// Move back for distance of 72mm, it takes approximately 0.5s.
			Quick_Back(20, 72);
			if (Touch_Detect() || Is_AtHomeBase()) {
				printf("%s %d: move back 100mm and still detect charger or touch event! return 0\n", __FUNCTION__, __LINE__);
				Set_Clean_Mode(Clean_Mode_Userinterface);
				Stop_Brifly();
				Set_SideBrush_PWM(0, 0);
				Beep(3, 100, 25, 5);
				return 0;
			}
		}
		Deceleration();
		Stop_Brifly();
		from_station = 1;
	}

	Blink_LED = 8;
	Reset_Touch();
	/*wati for gyro initialize*/
	ROS_DEBUG("while Blink_LED-----------------------------");
	while (Blink_LED--) {
		if (Touch_Detect()) {
			ROS_DEBUG("Touch_Detect1-----------------------------");
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 0;
		}
		Set_LED(0, 0);
		usleep(200000);
		if (Touch_Detect()) {
			ROS_DEBUG("Touch_Detect2-----------------------------");
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 0;
		}
		Set_LED(100, 0);
		usleep(200000);
	}

	// Set the Work_Timer_Start as current time
	Reset_Work_Time();

	//Initital home point
	Home_Point.clear();
	New_Home_Point.X = New_Home_Point.Y = 0;
	// Push the start point into the home point list
	Home_Point.push_front(New_Home_Point);

	ROS_DEBUG("Map_Initialize-----------------------------");
	Map_Initialize();
	PathPlanning_Initialize(&Home_Point.front().X, &Home_Point.front().Y);

	Reset_Rcon_Status();


	robot::instance()->init_mumber();// for init robot member

	//open gyro, wait for 3s ,if have any stop event (remote clean ...)stop ant return;
	Set_gyro_off();
	if(!Set_gyro_on()){
		Set_gyro_off();
		ROS_INFO("%s %d: Check: Touch Clean Mode! return 0\n", __FUNCTION__, __LINE__);
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return 0;
	}
	Set_IMU_Status();

	Motion_controller motion;
	auto count_n_10ms = 1000;
	while(robot::instance()->map_ready() == false && --count_n_10ms != 0){
		  usleep(10000);
	}
	if(count_n_10ms == 0){
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return 0;
	}

	int16_t	home_angle = robot::instance()->robot_get_home_angle();
		/* usleep for checking whether robot is in the station */
	usleep(700);
	if (from_station == 1 && !robot::instance()->align_active()) {
		printf("%s %d: Turn 45 degree to the wall\n", __FUNCTION__, __LINE__);

		CM_HeadToCourse(ROTATE_TOP_SPEED, Gyro_GetAngle(0) - 450);

		if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			printf("%s %d: Check: Touch Clean Mode! return 0\n", __FUNCTION__, __LINE__);
			return 0;
		}

		from_station = 1;
		station_zone = 0;
	}
	/*****************************************************Cleaning*****************************************************/
	while (ros::ok()) {

		/*************************************2 Cleaning Main Loop*************************************/
		state = -1;
		while (ros::ok()) {
			// Debug
			printf("[core_move.cpp] %s %d: Current Battery level: %d.\n", __FUNCTION__, __LINE__, robot::instance()->robot_get_battery_voltage());
			printf("[core_move.cpp] %s %d: Current work time: %d(s).\n", __FUNCTION__, __LINE__, Get_Work_Time());

			/***************************2.1 Common Process***************************/
			if (map_touring_cancel == 1) {
				return 0;
			}

			/***************************2.1 Common Process End***************************/

			/***************************2.2-1 Go Home***************************/
			if (go_home == 1) {
				//2.2-1.1 Common process
				tmpPnt.X = countToCell(Home_Point.front().X);
				tmpPnt.Y = countToCell(Home_Point.front().Y);
				pnt16ArTmp[0] = tmpPnt;
				path_escape_set_trapped_cell(pnt16ArTmp, 1);

				if (remote_go_home == 1) {
					Set_LED(100, 100);
					SetHomeRemote();
				}
				

				//2.2-1.3 Path to unclean area
				CM_create_home_boundary();

				// Try all the saved home point until it reach the charger stub. (There will be at least one home point (0, 0).)
				tmpPnt.X = countToCell(Home_Point.front().X);
				tmpPnt.Y = countToCell(Home_Point.front().Y);
				// Delete the first home point, it works like a stack.
				Home_Point.pop_front();
				while (ros::ok())
				{
					ROS_INFO("Go home Target: (%d, %d), %u targets left.", tmpPnt.X, tmpPnt.Y, Home_Point.size());
					// Try go to exactly this home point.
					state = CM_MoveToCell( tmpPnt.X, tmpPnt.Y, 2, 0, 1 );
					ROS_INFO("CM_MoveToCell return %d.", state);

					if ( state == -2 && Home_Point.empty()) {
						// If it is the last saved home point, stop the robot.
						Disable_Motors();
						// Beep for the finish signal.
						for (i = 10; i > 0; i--) {
							Beep(i, 6, 0, 1);
							usleep(100000);
						}

						if (from_station >= 1) {
							Set_Clean_Mode(Clean_Mode_GoHome);
						} else {
							Set_Clean_Mode(Clean_Mode_Userinterface);
						}

						printf("%s %d: Finish cleanning but not stop near home, cleaning time: %d(s)\n", __FUNCTION__, __LINE__, Get_Work_Time());
						return 0;

					} else if (state == -3 && Home_Point.empty()) {
						// If it is the last saved home point, stop the robot.
						Disable_Motors();
						mt_state = MT_Battery;
						return 0;
					} else if (state == -5 && Home_Point.empty()) {
						// If it is the last saved home point, stop the robot.
						Disable_Motors();
						// Beep for the finish signal.
						for (i = 10; i > 0; i--) {
							Beep(i, 6, 0, 1);
							usleep(100000);
						}
						Set_Clean_Mode(Clean_Mode_Userinterface);
						printf("%s %d: Finish cleanning, cleaning time: %d(s)\n", __FUNCTION__, __LINE__, Get_Work_Time());
						return 0;
					} else if (state == 1 || state == -7) {
						// Call GoHome() function to try to go to charger stub.
						GoHome();

						// In GoHome() function the clean mode might be set to Clean_Mode_GoHome, but it is not appropriate right here, it will change the state returned by CM_MoveToCell() for next home point to -4 and stop robot from moving to next home point.
						if (Get_Clean_Mode() == Clean_Mode_GoHome)
						{
							Set_Clean_Mode(Clean_Mode_Navigation);
						}
						// Check the clean mode to find out whether it has reach the charger.
						if (Get_Clean_Mode() == Clean_Mode_Charging)
						{
							printf("%s %d: Finish cleanning, cleaning time: %d(s)\n", __FUNCTION__, __LINE__, Get_Work_Time());
							return 0;
						}
						else if (Home_Point.empty())
						{
							// If it is the last point, it means it it now at (0, 0).
							if (from_station == 0) {
								CM_HeadToCourse(ROTATE_TOP_SPEED, -home_angle);

								if (Touch_Detect()) {
									return 0;
								}
							}

							Disable_Motors();
							// Beep for the finish signal.
							for (i = 10; i > 0; i--) {
								Beep(i, 6, 0, 1);
								usleep(100000);
							}

							if (from_station >= 1) {
								Set_Clean_Mode(Clean_Mode_GoHome);
							} else {
								Set_Clean_Mode(Clean_Mode_Userinterface);
							}

							printf("%s %d: Finish cleanning, cleaning time: %d(s)\n", __FUNCTION__, __LINE__, Get_Work_Time());
							return 0;
						}
					}
					else if (state == -4)
					{
						// state == -4 means home key was pressed. It continues going to current target home point.
						ROS_INFO("%s %d: Home key was pressed, keep going to this target.", __FUNCTION__, __LINE__);
						continue;
					}
					else if (state == -6)
					{
						// state == -6 means it detecteds low battery go home when go_home != 1, that's not possible.
						ROS_INFO("%s %d: Robot detecteds low battery go home when go_home != 1, this message should not be printed, please check.", __FUNCTION__, __LINE__);
						continue;
					}

					if (!Home_Point.empty())
					{
						// Get next home point 
						tmpPnt.X = countToCell(Home_Point.front().X);
						tmpPnt.Y = countToCell(Home_Point.front().Y);
						Home_Point.pop_front();

						// In GoHome() function, it may set the clean mode to Clean_Mode_GoHome. But it is not appropriate here, because it might affect the mode detection in CM_MoveToCell() and make it return -4.
						if (Get_Clean_Mode() == Clean_Mode_GoHome)
						{
							Set_Clean_Mode(Clean_Mode_Navigation);
						}
					}
					else
					{
						ROS_INFO("No home targets left and last target return %d, this message should not be printed, please check.", state);
						break;
					}
				}

			}
			/***************************2.2-1 Go Home End***************************/

			/***************************2.2-2 Normal Cleaning***************************/
			else {
				//State -1: Path next
				//State  0: Target list is empty
				//State  2: Robot is trapped

				/***************************2.2-2.1 Common State*************************/
				/***************************2.2-2.1 Common State End*************************/

				/***************************2.2-2.2-1 Start State -1*************************/
				//Find path
				if (state == -1) {

					ROS_DEBUG("Find path-----------------------------");

					x_current = Map_GetXPos();
					y_current = Map_GetYPos();
					state = path_next(&Next_Point.X, &Next_Point.Y, &Target_Point);
					ROS_INFO("Next point is (%d, %d)", countToCell(Next_Point.X), countToCell(Next_Point.Y));

					printf("State: %d", state);
					if (CM_handleExtEvent() != MT_None) {
						return 0;
					}
				}
				/***************************2.2-2.2-1 Start State -1 End*************************/

				/***************************2.2-2.2-2 Start State 0*************************/
				//No target point
				else if (state == 0) {
					//printf("State 0!");
					/***************************2.2-2.2-2.1 Common Process*************************/
					go_home = 1;
					break;
				}
				/***************************2.2-2.2-2 Start State 0 End*************************/

				/***************************2.2-2.2-3 Start State 1*************************/
				//Move to target
				else if (state == 1) {
					//printf("State 1!");

					ROS_DEBUG("Move to target-----------------------------");

					rounding_type = ROUNDING_NONE;

#if (PP_ROUNDING_OBSTACLE_LEFT) || (PP_ROUNDING_OBSTACLE_RIGHT)
					rounding_type = CM_get_rounding_direction(&Next_Point, Target_Point);
#endif

					if (rounding_type != ROUNDING_NONE) {
						printf("%s %d: Rounding %s.\n", __FUNCTION__, __LINE__, rounding_type == ROUNDING_LEFT ? "left" : "right");
						rounding(rounding_type, Next_Point, Bumper_Status_For_Rounding);
					} else {
						mt_state = CM_MoveToPoint(Next_Point);
					}

					if (y_current == countToCell(Next_Point.Y)) {
						x = Map_GetXPos();
						y = Map_GetYPos();
						if (x_current != x ) {
							slop = (((float)y_current) - ((float)y)) / (((float)x_current) - ((float)x));
							intercept = ((float)(y)) - slop *  ((float)(x));

							start = x > x_current ? x_current : x;
							end = x > x_current ? x: x_current;

							for (x = start; x <= end; x++) {
								y = (int16_t) (slop * (x) + intercept);

								printf("%s %d: marking (%d, %d) (%d, %d) (%d, %d)\n", __FUNCTION__, __LINE__, x, y - 1, x, y, x, y + 1);
								Map_SetCell(MAP, cellToCount(x), cellToCount(y - 1), CLEANED);
								Map_SetCell(MAP, cellToCount(x), cellToCount(y), CLEANED);
								Map_SetCell(MAP, cellToCount(x), cellToCount(y + 1), CLEANED);
							}
						}
					}

					if (mt_state == MT_Battery) {
						return 0;
					} else if (mt_state == MT_Remote_Home) {
						go_home = 1;
						Stop_Brifly();
					} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
						Disable_Motors();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					} else if (mt_state == MT_Battery_Home) {
						Display_Battery_Status(Display_Low);
						go_home = 1;
						Stop_Brifly();
					}

					state = -1;
				}
				/***************************2.2-2.2-3 Start State 1 End*************************/

				/***************************2.2-2.2-4 Start State 2*************************/
				//Trapped, escape mode
				else if (state == 2) {
					//printf("State 2!");
					state = Map_Wall_Follow(Map_Wall_Follow_Escape_Trapped);

					if ( map_touring_cancel == 1 ) {
						return 0;
					}

					if ( go_home == 1 ) {
						break;
					}

					if (state == 2) {
						Disable_Motors();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					}

					state = -1;
				}
				/***************************2.2-2.2-4 Start State 2 End*************************/

				/***************************2.2-2.2-5 Start Other State*************************/
				else {
					state = -1;
				}
				/***************************2.2-2.2-5 Start Other State End*************************/
			}
			/***************************2.2-2 Normal Cleaning End***************************/

			/***************************2.3 Last Common Process*************************/
			if (mt_state == MT_Battery) {
				return 0;
			} else if (mt_state == MT_Remote_Home) {
				go_home = 1;
				Stop_Brifly();
			} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
				Disable_Motors();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;
			} else if (mt_state == MT_Battery_Home) {
				Display_Battery_Status(Display_Low);
				go_home = 1;
				Stop_Brifly();
			}
			/***************************2.3 Last Common Process End*************************/
		}
		/*************************************2 Cleaning Main Loop End*************************************/
	}
	/*****************************************************Cleaning End*****************************************************/

}

/*
 * Robot move to target cell
 * @param x	cell x
 * @param y	cell y
 * @param mode 2: Dynamic change cells near target cell
 *             1: with escape mode, not finish
 *             0: no escape mode
 * @return	-2: Robot is trapped
 *		-1: Robot cannot move to target cell
 *		1: Robot arrive target cell
 */
int8_t CM_MoveToCell( int16_t x, int16_t y, uint8_t mode, uint8_t length, uint8_t step )
{
	Point32_t		Next_Point;
	int8_t		pathFind;
	int16_t		i, j, k;
	uint16_t	last_dir, offsetIdx = 0;
	Point16_t	tmp, pos;
	MapTouringType	mt_state = MT_None;

	if (is_block_accessible(x, y) == 0) {
		printf("%s %d: target is blocked.\n\n", __FUNCTION__, __LINE__);
		Map_Set_Cells(ROBOT_SIZE, x, y, CLEANED);
	}

	//Escape mode
	//TODO: Escape
	if ( mode ==  1 ) {
		printf("%s %d Path Find: Escape Mode\n", __FUNCTION__, __LINE__);

		pos.X = x;
		pos.Y = y;
		pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(),  &tmp.X, &tmp.Y, 0 );

		return 0;
	} else if ( mode == 2 ) {
		i = j = k = offsetIdx = 0;

		Point16_t relativePosTmp = {0, 0};
		printf("%s %d Path Find: Dynamic Target Mode, target: (%d, %d)\n", __FUNCTION__, __LINE__, x, y);

		relativePos[k].X = 0;
		relativePos[k].Y = 0;
		k = 1;
		for ( i = -length; i <= length; i += step ) {
			for ( j = -length; j <= length; j += step ) {
				if ( x + i <= xMax && x + i >= xMin &&  y + j <= yMax && y + j >= yMin ) {
					if ( i == 0 && j == 0 ) {
						continue;
					}
					relativePos[k].X = i;
					relativePos[k].Y = j;
					printf("Id: %d\tPoint: (%d, %d)\n", k, relativePos[k].X, relativePos[k].Y);
					++k;
				}
			}
		}
		printf("%s %d Size: %d\n", __FUNCTION__, __LINE__, k);

		//Position sort, two case: 1. sort for the previous half size of point; 2. sort the rest.
		//Sort from the nearest point to the farest point, refer to the middle point
		for ( i = 1 ; i < k; ++i) {
			for ( j = 1; j < k - i; ++j ) {
				if ( TwoPointsDistance( relativePos[j].X * 1000,     relativePos[j].Y * 1000,     0, 0 ) >
				     TwoPointsDistance( relativePos[j + 1].X * 1000, relativePos[j + 1].Y * 1000, 0, 0 ) ) {
					relativePosTmp = relativePos[j + 1];
					relativePos[j + 1] = relativePos[j];
					relativePos[j] = relativePosTmp;
				}
			}
		}

		printf("Bubble sort:\n");
		for ( i = 0; i < k; i++ ) {
			printf("Id: %d\tPoint: (%d, %d)\tDis:%d\n", i, relativePos[i].X, relativePos[i].Y,
			         TwoPointsDistance( relativePos[i].X * 1000, relativePos[i].Y * 1000, 0, 0 ));
		}

		last_dir = path_get_robot_direction();

		pos.X = x + relativePos[0].X;
		pos.Y = y + relativePos[0].Y;
		pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &tmp.X, &tmp.Y, 0 );

		//Set cell
		Map_Set_Cells(ROBOT_SIZE, x + relativePos[0].X, y + relativePos[0].Y, CLEANED);

		printf("%s %d Path Find: %d\n", __FUNCTION__, __LINE__, pathFind);
		printf("%s %d Target need to go: x:%d\ty:%d\n", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		printf("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
		while (1) {
			if ( pathFind == 1 || pathFind == SCHAR_MAX ) {
				path_set_current_pos();

				printf("%s %d Move to target...\n", __FUNCTION__, __LINE__ );
				Next_Point.X = cellToCount(tmp.X);
				Next_Point.Y = cellToCount(tmp.Y);

#if ENABLE_DEBUG
				debug_map(MAP, tmp.X, tmp.Y);
#endif

				mt_state = CM_MoveToPoint(Next_Point);
				printf("%s %d Arrive Target! Now: (%d, %d)\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());

				if (mt_state == MT_Battery) {
					printf("%s %d: low battery is detected, battery < 1200\n", __FUNCTION__, __LINE__);
					return -3;
				} else if (mt_state == MT_Remote_Home) {
					printf("%s %d: home is pressed\n", __FUNCTION__, __LINE__);
					return -4;
				} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
					printf("%s %d: remote is pressed, clean key is pressed,  or cliff is reached\n", __FUNCTION__, __LINE__);
					return -5;
				} else if (mt_state == MT_Battery_Home) {
					printf("%s %d: low battery is detected, battery < 1300\n", __FUNCTION__, __LINE__);
					return -6;
				} else if ( mt_state == MT_None ) {
					if ( go_home == 1 && Is_Station() == 1 ) {
						return -7;
					}
				}

				//Arrive exit cell, set < 3 when ROBOT_SIZE == 5
				if ( TwoPointsDistance( x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y, Map_GetXPos(), Map_GetYPos() ) < ROBOT_SIZE / 2 + 1 ) {
					printf("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
					printf("%s %d Destination: x:%d\ty:%d\n", __FUNCTION__, __LINE__, x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);
					return 1;
				}

				if (is_block_accessible(x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y) == 0) {
					printf("%s %d: Target is blocked. Try to find new target.\n", __FUNCTION__, __LINE__);
					pathFind = -2;
					continue;
				}

				last_dir = path_get_robot_direction();

				pos.X = x + relativePos[offsetIdx].X;
				pos.Y = y + relativePos[offsetIdx].Y;
				pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(),
				                                      &tmp.X, &tmp.Y, last_dir );

				if (CM_CheckLoopBack(tmp) == 1) {
					pathFind = -2;
				}
				printf("%s %d Path Find: %d, target: (%d, %d)\n", __FUNCTION__, __LINE__, pathFind,
				         x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);
				printf("%s %d Target need to go: x:%d\ty:%d\n", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
				printf("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
			} else if ( pathFind == -2 || pathFind == -1 ) {
				//Add offset
				offsetIdx++;
				if ( relativePos[offsetIdx].X == 0 && relativePos[offsetIdx].Y == 0 )
					offsetIdx++;

				if ( offsetIdx >= k ) {
					return -2;
				}

				last_dir = path_get_robot_direction();

				pos.X = x + relativePos[offsetIdx].X;
				pos.Y = y + relativePos[offsetIdx].Y;
				pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(),
				                                      &tmp.X, &tmp.Y, last_dir );

				if (Touch_Detect()) {
					Set_Touch();
					CM_TouringCancel();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return -5;
				}

				if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right)) {
					printf("%s %d: robot is taken up.\n", __FUNCTION__, __LINE__);
					Stop_Brifly();
					return -5;
				}

				printf("%s %d Path Find: %d, %d Target Offset: (%d, %d)\n", __FUNCTION__, __LINE__, pathFind, offsetIdx,
				         relativePos[offsetIdx].X, relativePos[offsetIdx].Y);
				printf("%s %d Path Find: %d, target: (%d, %d)\n", __FUNCTION__, __LINE__, pathFind,
				         x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);

			} else {
				return pathFind;
			}
		}
	}
	//Normal mode
	else {
		printf("%s %d Path Find: Normal Mode, target: (%d, %d)\n", __FUNCTION__, __LINE__, x, y);
		last_dir = path_get_robot_direction();

		pos.X = x;
		pos.Y = y;
		pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &tmp.X, &tmp.Y, 0 );

		printf("%s %d Path Find: %d\n", __FUNCTION__, __LINE__, pathFind);
		printf("%s %d Target need to go: x:%d\ty:%d\n", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		printf("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());

		//Note that path_move_to_unclean_area only can get the next cell to the destination cell
		while ( pathFind == 1 || pathFind == SCHAR_MAX ) {
			path_set_current_pos();

			printf("%s %d Move to target...\n", __FUNCTION__, __LINE__ );
			Next_Point.X = cellToCount(tmp.X);
			Next_Point.Y = cellToCount(tmp.Y);
#if ENABLE_DEBUG
			debug_map(MAP, tmp.X, tmp.Y);
#endif

			mt_state = CM_MoveToPoint(Next_Point);
			printf("%s %d Arrive Target! Now: (%d, %d)\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());

			if (mt_state == MT_Battery) {
				printf("%s %d: low battery is detected, battery < 1200\n", __FUNCTION__, __LINE__);
				return -3;
			} else if (mt_state == MT_Remote_Home) {
				printf("%s %d: home is pressed\n", __FUNCTION__, __LINE__);
				return -4;
			} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
				printf("%s %d: remote is pressed, clean key is pressed,  or cliff is reached\n", __FUNCTION__, __LINE__);
				return -5;
			} else if (mt_state == MT_Battery_Home) {
				printf("%s %d: low battery is detected, battery < 1300\n", __FUNCTION__, __LINE__);
				return -6;
			} else if ( mt_state == MT_None ) {
				if ( go_home == 1 && Is_Station() == 1 ) {
					return -7;
				}
			}

			//Arrive exit cell, set < 3 when ROBOT_SIZE == 5
			if ( TwoPointsDistance( x, y, Map_GetXPos(), Map_GetYPos() ) < ROBOT_SIZE / 2 + 1 ) {
				printf("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
				printf("%s %d Destination: x:%d\ty:%d\n", __FUNCTION__, __LINE__, x, y);
				return 1;
			}

			if (is_block_accessible(x, y) == 0) {
				printf("%s %d: target is blocked\n", __FUNCTION__, __LINE__);
				return 0;
			}

			last_dir = path_get_robot_direction();

			pos.X = x;
			pos.Y = y;
			pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &tmp.X, &tmp.Y, last_dir );

			printf("%s %d Path Find: %d, target: (%d, %d)\n", __FUNCTION__, __LINE__, pathFind, x, y);
			printf("%s %d Target need to go: x:%d\ty:%d\n", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
			printf("%s %d Now: x:%d\ty:%d\n", __FUNCTION__, __LINE__, countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
		}
		return pathFind;
	}
}

/*-------------- Move Back -----------------------------*/
void CM_CorBack(uint16_t dist)
{
	float pos_x, pos_y, distance;
	uint32_t SP = 10;
	uint16_t Counter_Watcher = 0;

	printf("%s %d: Moving back...\n", __FUNCTION__, __LINE__);
	Stop_Brifly();
	CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));
	Set_Dir_Backward();
	Set_Wheel_Speed(8, 8);
	Reset_Wheel_Step();
	Counter_Watcher = 0;

	pos_x = robot::instance()->robot_get_odom_position_x();
	pos_y = robot::instance()->robot_get_odom_position_y();

	while (1) {
		distance = sqrtf(powf(pos_x - robot::instance()->robot_get_odom_position_x(), 2) + powf(pos_y - robot::instance()->robot_get_odom_position_y(), 2));
		if (fabsf(distance) > 0.02f) {
			break;
		}

		CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));
		usleep(10000);
		Counter_Watcher++;
		SP = 8 + Counter_Watcher / 100;
		SP = (SP > 18) ? 18 : SP;

		Set_Wheel_Speed(SP, SP);
		if (Counter_Watcher > 3000) {
			if (Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			break;
		}
		if (Touch_Detect()) {
			break;
		}

		if ((Check_Motor_Current() == Check_Left_Wheel) || (Check_Motor_Current() == Check_Right_Wheel)) {
			break;
		}
	}
	CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));
	Reset_TempPWM();
	Stop_Brifly();
	printf("%s %d: Moving back done!\n", __FUNCTION__, __LINE__);
}

void CM_SetGoHome(uint8_t remote) {
	go_home = 1;
	if (remote == 1) {
		remote_go_home = 1;
	}
}

void CM_TouringCancel(void)
{
	map_touring_cancel = 1;
}

void CM_SetGyroOffset(int16_t offset)
{
	map_gyro_offset = offset;
}

void CM_SetHome(int32_t x, int32_t y) {
	printf("%s %d: Push new reachable home: (%d, %d) to home point list.\n", __FUNCTION__, __LINE__, countToCell(x), countToCell(y));
	New_Home_Point.X = x;
	New_Home_Point.Y = y;
	Home_Point.push_front(New_Home_Point);
}



uint8_t CM_IsLowBattery(void) {
	return lowBattery;
}

uint8_t CM_CheckLoopBack( Point16_t target ) {
	uint8_t retval = 0;
	if ( target.X == positions[1].x && target.Y == positions[1].y &&
	     target.X == positions[3].x && target.Y == positions[3].y ) {
		printf("%s %d Possible loop back (%d, %d)\n", __FUNCTION__, __LINE__, target.X, target.Y);
		retval  = 1;
	}

	return retval;
}

void CM_Matrix_Rotate(int32_t x_in, int32_t y_in, int32_t *x_out, int32_t *y_out, double theta)
{
	double dd, de;

	dd = (double) x_in;
	de = (double) y_in;

	Matrix_Rotate(&dd, &de, theta);

	*x_out = (int32_t)dd;
	*y_out = (int32_t)de;
}

MapTouringType CM_handleExtEvent()
{
	/* Check low battery event, if battery is low, go home directly. */
	if ((Check_Bat_Home() == 1) && go_home != 1) {
		lowBattery = 1;
		if ( Get_VacMode() == Vac_Max ) {
			Switch_VacMode();
		}
		Stop_Brifly();
		printf("%s %d: low battery, battery < 13.2v is detected.\n", __FUNCTION__, __LINE__);
		remote_go_home = 1;
		return MT_Battery_Home;
	}

	/* Check key press events. */
	if (Touch_Detect()) {
		Stop_Brifly();
		printf("%s %d: clean key is pressed.\n", __FUNCTION__, __LINE__);
		Beep(5, 6, 6, 1);
		Reset_Touch();
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return MT_Key_Clean;
	}

	/* Check remote events. */
	if (Get_Rcon_Remote() != 0) {
		/* Check remote home key press event, if home key is pressed, go home directly. */
		if (Remote_Key(Remote_Home) && (go_home == 0)) {
			Set_BLDC_Speed(Vac_Speed_NormalL);
			Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power);
			Stop_Brifly();
			printf("%s %d: remote home is pressed.\n", __FUNCTION__, __LINE__);
			remote_go_home = 1;
			return MT_Remote_Home;
		}


		/*
		 * Check remote spot key press event, if spot key is pressed,
		 * change to spot mode, after spot mode finished, back to zig-zag clean.
		 */
#if 0
		if (Remote_Key(Remote_Spot)) {
			Stop_Brifly();
			printf("%s %d: remote spot is pressed.\n", __FUNCTION__, __LINE__);
			Spot_Mode();
			Switch_VacMode();
			printf("%s %d: remote spot ends.\n", __FUNCTION__, __LINE__);
			return MT_None;
		}
#endif

		if (Remote_Key(Remote_Max)) {
			if (lowBattery == 0) {
				Switch_VacMode();
			}
		}


		/* Check remote clean key press event, if clean key is pressed, stop robot directly. */
		if (Remote_Key(Remote_Clean)) {
			Stop_Brifly();
			printf("%s %d: remote clean is pressed.\n", __FUNCTION__, __LINE__);
			return MT_Remote_Clean;
		}
		Reset_Rcon_Remote();
	}

	/* Check whether robot is taken up. */
	if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right)) {
		printf("%s %d: robot is taken up.\n", __FUNCTION__, __LINE__);
		Stop_Brifly();
		return MT_Cliff;
	}

	return MT_None;
}

void CM_create_home_boundary(void)
{
	int16_t i, j, k;
	int16_t xMinSearch, xMaxSearch, yMinSearch, yMaxSearch;

	k = 3;
	xMinSearch = xMaxSearch = yMinSearch = yMaxSearch = SHRT_MAX;
	for (i = xMin; xMinSearch == SHRT_MAX; i++) {
		for (j = yMin; j <= yMax; j++) {
			if (Map_GetCell(MAP, i, j) != UNCLEAN) {
				xMinSearch = i - k;
				break;
			}
		}
	}
	for (i = xMax; xMaxSearch == SHRT_MAX; i--) {
		for (j = yMin; j <= yMax; j++) {
			if (Map_GetCell(MAP, i, j) != UNCLEAN) {
				xMaxSearch = i + k;
				break;
			}
		}
	}
	for (i = yMin; yMinSearch == SHRT_MAX; i++) {
		for (j = xMin; j <= xMax; j++) {
			if (Map_GetCell(MAP, j, i) != UNCLEAN) {
				yMinSearch = i - k;
				break;
			}
		}
	}
	for (i = yMax; yMaxSearch == SHRT_MAX; i--) {
		for (j = xMin; j <= xMax; j++) {
			if (Map_GetCell(MAP, j, i) != UNCLEAN) {
				yMaxSearch = i + k;
				break;
			}
		}
	}
	printf("%s %d: x: %d - %d\ty: %d - %d\n", __FUNCTION__, __LINE__, xMinSearch, xMaxSearch, yMinSearch, yMaxSearch);
	for (i = xMinSearch; i <= xMaxSearch; i++) {
		if (i == xMinSearch || i == xMaxSearch) {
			for (j = yMinSearch; j <= yMaxSearch; j++) {
				Map_SetCell(MAP, cellToCount(i), cellToCount(j), BLOCKED_BUMPER);
			}
		} else {
			Map_SetCell(MAP, cellToCount(i), cellToCount(yMinSearch), BLOCKED_BUMPER);
			Map_SetCell(MAP, cellToCount(i), cellToCount(yMaxSearch), BLOCKED_BUMPER);
		}
	}
}
