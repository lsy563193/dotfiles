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
#include "spot.h"

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

#include <wav.h>
//#include "../include/obstacle_detector.h"
#include <motion_controler.h>
//#include "obstacle_detector.h"
//using namespace obstacle_detector;

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


// This list is for storing the position that robot sees the charger stub.
std::list <Point32_t> Home_Point;
// This is for adding new point to Home Point list.
Point32_t New_Home_Point;

#if CONTINUE_CLEANING_AFTER_CHARGE
// This is for the continue point for robot to go after charge.
Point32_t Continue_Point;
#endif

uint8_t map_touring_cancel = 0;

uint8_t	go_home = 0;
uint8_t	remote_go_home = 0;
uint8_t	from_station = 0;
uint8_t lowBattery = 0;
int16_t map_gyro_offset = 0;

uint8_t	should_follow_wall = 0;

Point16_t relativePos[MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH] = {{0, 0}};

extern PositionType positions[];

extern int16_t xMin, xMax, yMin, yMax;

extern volatile uint8_t cleaning_mode;

// This status is for rounding function to decide the angle it should turn.
uint8_t Bumper_Status_For_Rounding;

extern bool Is_Slam_Ready;//For checking if the slam is initialized finish

// This time count is for checking how many times of 20ms did the user press the key.
uint16_t Press_Time = 0;

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

	uint32_t	width,	height;

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
	if (std::abs(yaw) <= (M_PI / 2) ){
		plus_sign = 1;
		a_max = (plus_sign * std::abs(int(round(search_length * sin(std::abs(yaw)) / 0.05))));
		for (int a = 0; a <= a_max; a = a + 1) {		//n = (search_length * cos(yaw)) / resolution
			int c = 0;
			float x = position_x + (a / tan(std::abs(yaw))) * 0.05;
			float y = position_y + a * 0.05;
			for (int b = -int((round(search_width / 0.05)) / 2) + c; b <= int((round(search_width / 0.05)) / 2); b = b + 1){
				float x_1 = x + b * 0.05;
				/*over map scope*/
				if ((x_1 < origin_x ) or (x_1 > (width * 0.05) ) or (y < origin_y) or (y > (height * 0.05))){
					ROS_WARN("over scope");
				} else {
					if ((*p_map_data)[CM_Get_grid_index(x_1, y, width, height, resolution, origin_x, origin_y)] == 100) {
						c++;	//add one grid wall
						if (c >= (search_width / 0.05)){
							ROS_INFO("exist wall");
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
		a_max = (plus_sign * std::abs(int(round(search_length * sin(std::abs(yaw)) / 0.05))));
		for (int a = 0; a >= a_max; a = a - 1) {	//n = (search_length * cos(yaw)) / resolution
			int c = 0;
			float x = position_x + (a / tan(std::abs(yaw))) * 0.05;
			float y = position_y + a * 0.05;
			for (int b = -int((round(search_width / 0.05)) / 2) + c; b <= int((round(search_width / 0.05)) / 2); b = b + 1) {
				float x_1 = x + b * 0.05;
				/*over map scope*/
				if ((x_1 < origin_x ) or (x_1 > (width * 0.05) ) or (y < origin_y) or (y > (height * 0.05))) {
					ROS_WARN("over scope");
				} else {
					if ((*p_map_data)[CM_Get_grid_index(x_1, y, width, height, resolution, origin_x, origin_y)] == 100) {
						c++;
						if (c >= (search_width / 0.05)) {
							ROS_INFO("exist wall");
							return 2;
						}
					} else {
						if ((*p_map_data)[CM_Get_grid_index(x_1, y, width, height, resolution, origin_x, origin_y)] == -1) {
							ROS_WARN("exist unkown");
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

void CM_update_position(uint16_t heading) {
	int8_t	e;
	int16_t c, d, x, y;
	int32_t i, j, k;

	float	pos_x, pos_y;
	x = Map_GetXPos();
	y = Map_GetYPos();

	pos_x = robot::instance()->robot_get_position_x() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->robot_get_position_y() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_SetPosition(pos_x, pos_y);
	if (x != Map_GetXPos() || y != Map_GetYPos()) {
		for (c = 1; c >= -1; --c) {
			for (d = 1; d >= -1; --d) {
				CM_count_normalize(heading, CELL_SIZE * c, CELL_SIZE * d, &i, &j);
				e = Map_GetCell(MAP, countToCell(i), countToCell(j));

				if (e == BLOCKED_OBS || e == BLOCKED_BUMPER) {
					ROS_WARN("%s %d: warning, reset bumper/obs value, (%d, %d)", __FUNCTION__, __LINE__, countToCell(i), countToCell(j));
					Map_SetCell(MAP, i, j, CLEANED);
				}
			}
		}
		robot::instance() -> pub_clean_markers();
	}

#if (ROBOT_SIZE == 5)

	CM_count_normalize(heading, -CELL_SIZE_2, CELL_SIZE, &i, &j);
	if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_BOUNDARY) {
		//ROS_WARN("%s %d: warning, setting boundary.", __FUNCTION__, __LINE__);
	} else {
		Map_SetCell(MAP, i, j, CLEANED);
	}

	for (c = 1; c >= -1; --c) {
		CM_count_normalize(heading, c * CELL_SIZE, CELL_SIZE, &i, &j);
		Map_SetCell(MAP, i, j, CLEANED);
	}

	CM_count_normalize(heading, CELL_SIZE_2, CELL_SIZE, &i, &j);
	if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_BOUNDARY) {
		//ROS_WARN("%s %d: warning, setting boundary.", __FUNCTION__, __LINE__);
	} else {
		Map_SetCell(MAP, i, j, CLEANED);
	}

	if (Get_OBS_Status() & Status_Left_OBS) {
		CM_count_normalize(0, heading, CELL_SIZE_3, CELL_SIZE, &i, &j);
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
		CM_count_normalize(heading, c * CELL_SIZE, CELL_SIZE, &i, &j);
		if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_BOUNDARY) {
			//ROS_WARN("%s %d: warning, setting boundary.", __FUNCTION__, __LINE__);
		} else {
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) == BLOCKED_BUMPER) {
				ROS_WARN("%s %d: warning, setting bumper (%d, %d).", __FUNCTION__, __LINE__, countToCell(i), countToCell(j));
			}
			Map_SetCell(MAP, i, j, CLEANED);
		}
	}

	if (Get_OBS_Status() & Status_Left_OBS) {
		CM_count_normalize(heading, CELL_SIZE_2, CELL_SIZE, &i, &j);
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
		CM_count_normalize(Gyro_GetAngle(), (c - 1) * CELL_SIZE, CELL_SIZE_3, &j, &k);
		if (i && Map_GetCell(MAP, countToCell(j), countToCell(k)) != BLOCKED_BUMPER) {
			Map_SetCell(MAP, j, k, BLOCKED_OBS);
		} else if (Map_GetCell(MAP, countToCell(j), countToCell(k)) == BLOCKED_OBS) {
			Map_SetCell(MAP, j, k, UNCLEAN);
		}

#else
		CM_count_normalize(Gyro_GetAngle(), (c - 1) * CELL_SIZE, CELL_SIZE_2, &j, &k);
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
			CM_count_normalize(Gyro_GetAngle(), c * CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	} else if (bumper & LeftBumperTrig) {
		if (action == ACTION_LT) {
			CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_3, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			//CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_3, CELL_SIZE, &x_tmp, &y_tmp);
			//Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		} else {
			for (c = 1; c <= 2; ++c) {
				CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE_3, &x_tmp, &y_tmp);
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	} else if (bumper & RightBumperTrig) {
		if (action == ACTION_RT) {
			CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_3, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			//CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_3, CELL_SIZE, &x_tmp, &y_tmp);
			//Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		} else {
			for (c = -2; c <= -1; ++c) {
				CM_count_normalize(Gyro_GetAngle(), c * CELL_SIZE_2, CELL_SIZE_3, &x_tmp, &y_tmp);
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	}
#else

	//bumper = Get_Bumper_Status();
	if ((bumper & RightBumperTrig) && (bumper & LeftBumperTrig)) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(Gyro_GetAngle(), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	} else if (bumper & LeftBumperTrig) {
		CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE_2, &x_tmp, &y_tmp);
		ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		if (action == ACTION_LT) {
			//CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			//Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		} else {
			CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			if ((positions[0].x == positions[1].x) && (positions[0].y == positions[1].y) && (positions[0].dir == positions[1].dir) &&
				(positions[0].x == positions[2].x) && (positions[0].y == positions[2].y) && (positions[0].dir == positions[2].dir)) {
				CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x_tmp, &y_tmp);
				ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
			}
		}
	} else if (bumper & RightBumperTrig) {
		CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE_2, &x_tmp, &y_tmp);
		ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		if (action == ACTION_RT) {
			//CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			//Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		} else {
			CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE, &x_tmp, &y_tmp);
			ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

			if ((positions[0].x == positions[1].x) && (positions[0].y == positions[1].y) && (positions[0].dir == positions[1].dir) &&
				(positions[0].x == positions[2].x) && (positions[0].y == positions[2].y) && (positions[0].dir == positions[2].dir)) {
				CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x_tmp, &y_tmp);
				ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
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

		CM_count_normalize(Gyro_GetAngle(), (c - 1) * CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
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
			CM_count_normalize(0, Gyro_GetAngle(), c * CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (Get_Cliff_Trig() & Status_Cliff_Left) {
		CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE_3, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_3, CELL_SIZE_2, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
	}
	if (Get_Cliff_Trig() & Status_Cliff_Right) {
		CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_3, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE_3, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);

		CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_3, CELL_SIZE_2, &x_tmp, &y_tmp);
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
	}

#else
			CM_count_normalize(Gyro_GetAngle(), (c - 1) * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) != BLOCKED_BUMPER) {
				ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_OBS);
			}
		} else {
			CM_count_normalize(Gyro_GetAngle(), (c - 1) * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) == BLOCKED_OBS) {
				Map_SetCell(MAP, x_tmp, y_tmp, UNCLEAN);
			}
		}
	}

	CM_update_map_bumper(action, bumper);

	if (Get_Cliff_Trig() & Status_Cliff_Front) {
		for (c = -1; c <= 1; ++c) {
			CM_count_normalize(Gyro_GetAngle(), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (Get_Cliff_Trig() & Status_Cliff_Left) {
		for (c = 1; c <= 2; ++c) {
			CM_count_normalize(Gyro_GetAngle(), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
			Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
		}
	}
	if (Get_Cliff_Trig() & Status_Cliff_Right) {
		for (c = -2; c <= -1; ++c) {
			CM_count_normalize(Gyro_GetAngle(), c * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
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

	Diff = Angle - Gyro_GetAngle();

	ROS_INFO("%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\tBias: %d\tTemp: %d\tScale: %d",
			 __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(), Diff, (Angle - Gyro_GetAngle()), Gyro_GetXAcc(), Gyro_GetYAcc(), Gyro_GetZAcc());

	while (Diff >= 1800) {
		Diff = Diff - 3600;
	}

	while (Diff <= (-1800)) {
		Diff = Diff + 3600;
	}

	if ((Diff < 10) && (Diff > (-10))) {
		return;
	}

	ROS_INFO("%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)\tangle_turned: %d\tBias: %d\tTemp: %d\tScale: %d",
			 __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(), Diff, (Angle - Gyro_GetAngle()), angle_turned, Gyro_GetXAcc(), Gyro_GetYAcc(), Gyro_GetZAcc());

	if (((Diff <= 1800 && Diff >= 1700) || (Diff >= -1800 && Diff <= -1700))) {
		if (Diff <= 1800 && Diff >= 1700) {
			if (angle_turned < 0) {
				ROS_INFO("%s %d: Turn Left", __FUNCTION__, __LINE__);

				Set_Dir_Left();
				action = ACTION_LT;
				angle_turned += Diff;
			} else {
				ROS_INFO("%s %d: Turn Right", __FUNCTION__, __LINE__);

				Set_Dir_Right();
				action = ACTION_RT;
				angle_turned += (Diff - 3600);
			}
		} else {
			if (angle_turned > 0) {
				ROS_INFO("%s %d: Turn Right", __FUNCTION__, __LINE__);

				Set_Dir_Right();
				action = ACTION_RT;
				angle_turned += Diff;
			} else {
				ROS_INFO("%s %d: Turn Left", __FUNCTION__, __LINE__);

				Set_Dir_Left();
				action = ACTION_LT;
				angle_turned += (3600 + Diff);
			}
		}
	} else {
		if ((Diff >= 0) && (Diff <= 1800)) {	// turn right
			ROS_INFO("%s %d: Turn Left", __FUNCTION__, __LINE__);

			Set_Dir_Left();
			action = ACTION_LT;
		} else if ((Diff <= 0) && (Diff >= (-1800))) {
			ROS_INFO("%s %d: Turn Right", __FUNCTION__, __LINE__);

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
				ROS_WARN("%s %d: motor(s) error break!", __FUNCTION__, __LINE__);
				return;
			}
		}

		if (Touch_Detect()) {
			Stop_Brifly();
			ROS_WARN("%s %d: touch detect break!", __FUNCTION__, __LINE__);
			return;
		}

		if (Get_Rcon_Remote() > 0) {
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (Get_Rcon_Remote() & (Remote_Clean | Remote_Home | Remote_Max)) {
				if (Remote_Key(Remote_Max)) {
					if (lowBattery == 0) {
						Switch_VacMode();
					}
					Reset_Rcon_Remote();
				}
				if (Remote_Key(Remote_Home) && go_home == 0) {
					Stop_Brifly();
					Set_BLDC_Speed(Vac_Speed_NormalL);
					Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power);
					Set_Clean_Mode(Clean_Mode_GoHome);
					ROS_WARN("%s %d: remote home is pressed.", __FUNCTION__, __LINE__);

					CM_SetGoHome(1);
					Reset_Rcon_Remote();
					return;
				}
				Reset_Rcon_Remote();
			} else {
				Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
				Reset_Rcon_Remote();
			}
		}

		if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right)) {
			ROS_WARN("%s %d: robot is taken up break!", __FUNCTION__, __LINE__);
			return;
		}

		isBumperTriggered = Get_Bumper_Status();
		if (isBumperTriggered) {
			Stop_Brifly();
			CM_update_position(Gyro_GetAngle());
			CM_update_map(action, isBumperTriggered);

			ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
			CM_CorBack(COR_BACK_20MM);

			if (Touch_Detect())
			{
				Stop_Brifly();
				ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
				return;
			}
			Stop_Brifly();
			CM_update_map(action, isBumperTriggered);

			Diff = Angle - Gyro_GetAngle();
			while (Diff >= 1800) {
				Diff = Diff - 3600;
			}

			while (Diff <= (-1800)) {
				Diff = Diff + 3600;
			}

			ROS_INFO("%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(), Diff, (Angle - Gyro_GetAngle()));
			if ((Diff >= 0) && (Diff <= 1800)) {	// turn right
				ROS_INFO("Turn Left");

				Set_Dir_Left();
				action = ACTION_LT;
			} else if ((Diff <= 0) && (Diff >= (-1800))) {
				ROS_INFO("Turn Right");

				Set_Dir_Right();
				action = ACTION_RT;
			}

			Reset_TempPWM();
			Set_Wheel_Speed(0, 0);

#ifdef BUMPER_ERROR
			if (Get_Bumper_Status()) {
				ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_20MM);

				if (Touch_Detect())
				{
					Stop_Brifly();
					ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
					return;
				}

				Diff = Angle - Gyro_GetAngle();
				while (Diff >= 1800) {
					Diff = Diff - 3600;
				}

				while (Diff <= (-1800)) {
					Diff = Diff + 3600;
				}

				ROS_INFO("%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(), Diff, (Angle - Gyro_GetAngle()));
				if ((Diff >= 0) && (Diff <= 1800)) {	// turn right
					ROS_INFO("Turn Left");

					Set_Dir_Left();
					action = ACTION_LT;
				} else if ((Diff <= 0) && (Diff >= (-1800))) {
					ROS_INFO("Turn Right");

					Set_Dir_Right();
					action = ACTION_RT;
				}
				if (Get_Bumper_Status()) {
					CM_CorBack(COR_BACK_20MM);

					if (Touch_Detect())
					{
						Stop_Brifly();
						ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
						return;
					}

					Diff = Angle - Gyro_GetAngle();
					while (Diff >= 1800) {
						Diff = Diff - 3600;
					}

					while (Diff <= (-1800)) {
						Diff = Diff + 3600;
					}

					ROS_INFO("%s %d: Angle: %d\tGyro: %d\tDiff: %d(%d)", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(), Diff, (Angle - Gyro_GetAngle()));
					if ((Diff >= 0) && (Diff <= 1800)) {	// turn right
						ROS_INFO("Turn Left");

						Set_Dir_Left();
						action = ACTION_LT;
					} else if ((Diff <= 0) && (Diff >= (-1800))) {
						ROS_INFO("Turn Right");

						Set_Dir_Right();
						action = ACTION_RT;
					}
					if (Get_Bumper_Status()) {
						ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);

						if (Touch_Detect())
						{
							Stop_Brifly();
							ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
							return;
						}

						Set_Error_Code(Error_Code_Bumper);
						Stop_Brifly();
						return;
					}
				}
			}

#endif
		}

		Diff = CM_ABS(Angle, Gyro_GetAngle());
		Diff = Diff > 1800 ? 3600 - Diff : Diff;
		if ((Diff < 10) && (Diff > (-10))) {
			Stop_Brifly();
			CM_update_position(Gyro_GetAngle());

			ROS_INFO("%s %d: Angle: %d\tGyro: %d\tDiff: %d", __FUNCTION__, __LINE__, Angle, Gyro_GetAngle(), Diff);
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

	usleep(10000);
	//10 second

	Reset_Rcon_Status();
	HomeT = HomeL = HomeR = home_hit = slow_down = 0;
	Integration_Cycle = 0;
	Target_Course = Rotate_Angle = Integrated = Left_Speed = Right_Speed = 0;
	Base_Speed = BASE_SPEED;

	CM_update_position(Gyro_GetAngle());

	if (rotate_is_needed == true) {
		Target_Course = course2dest(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y);
		ROS_INFO("Target_Course: %d", Target_Course);
		CM_HeadToCourse(ROTATE_TOP_SPEED, Target_Course);	//turn to target position
		ROS_INFO("leave CM_HeadToCourse");
	}

	ROS_INFO("%s %d: original target (%d, %d)", __FUNCTION__, __LINE__, Target.X, Target.Y);
	if (position.X != Map_GetXCount() && position.X == Target.X) {
		Target.X = Map_GetXCount();
	} else if (position.Y != Map_GetYCount() && position.Y == Target.Y) {
		Target.Y = Map_GetYCount();
	}
	ROS_INFO("%s %d: new target (%d, %d)", __FUNCTION__, __LINE__, Target.X, Target.Y);

	if (Touch_Detect()) {
		ROS_INFO("%s %d: Gyro Calibration: %d", __FUNCTION__, __LINE__, Gyro_GetCalibration());
		ROS_INFO("%s %d: Touch_Detect in CM_HeadToCourse()", __FUNCTION__, __LINE__);
		// Key release detection, if user has not release the key, don't do anything.
		//ROS_WARN("%s %d: Press_Time = %d", __FUNCTION__, __LINE__,  Press_Time);
		while (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: User hasn't release key.", __FUNCTION__, __LINE__);
			usleep(20000);
#if MANUAL_PAUSE_CLEANING
			Press_Time++;
			if (Press_Time == 151)
			{
				Beep(1, 5, 0, 1);
			}
		}
		if (Press_Time > 150)
		{
			if (robot::instance()->Is_Cleaning_Manual_Paused())
			{
				robot::instance()->Reset_Cleaning_Manual_Pause();
			}
		}
		else
		{
			Press_Time = 0;
#endif
		}
		usleep(10000);
		Stop_Brifly();
		return MT_Key_Clean;
	}
	if (Get_Clean_Mode() == Clean_Mode_GoHome) {
		ROS_INFO("%s %d: Gyro Calibration: %d", __FUNCTION__, __LINE__, Gyro_GetCalibration());
		usleep(10000);

		Set_Clean_Mode(Clean_Mode_Navigation);
		return MT_Remote_Home;
	}

	//usleep(1000);
	CM_update_position(Gyro_GetAngle());

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
				ROS_WARN("%s %d: Check: Clean Mode! break", __FUNCTION__, __LINE__);
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
				ROS_WARN("%s %d: low battery, battery < 1200 is detected.", __FUNCTION__, __LINE__);
				retval = MT_Battery;
				break;
			}
		} else {
			if (Check_Bat_SetMotors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power) && go_home != 1 ) {
				Stop_Brifly();
				ROS_WARN("%s %d: low battery, battery < 1200 is detected.", __FUNCTION__, __LINE__);
				retval = MT_Battery;
				break;
			}
		}

		if ((retval = CM_handleExtEvent()) != MT_None) {
			ROS_WARN("%s %d: Check: Clean Mode! break.", __FUNCTION__, __LINE__);
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

				ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_20MM);
				if (Touch_Detect())
				{
					Stop_Brifly();
					ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
					retval = MT_Key_Clean;
					break;
				}

#ifdef CLIFF_ERROR

				if (Get_Cliff_Trig()) {
					ROS_INFO("Cliff back 2nd time.");
					if (Get_Cliff_Trig() == Status_Cliff_All) {
						ROS_WARN("%s %d: all cliffs are triggered", __FUNCTION__, __LINE__);
						Stop_Brifly();
						retval = MT_Key_Clean;
						break;
					}
					ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
					CM_CorBack(COR_BACK_20MM);
					if (Touch_Detect())
					{
						Stop_Brifly();
						ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
						retval = MT_Key_Clean;
						break;
					}
					if (Get_Cliff_Trig()) {
						ROS_INFO("Cliff back 3rd time.");
						if (Get_Cliff_Trig() == Status_Cliff_All) {
							ROS_WARN("%s %d: all cliffs are triggered", __FUNCTION__, __LINE__);
							Stop_Brifly();
							retval = MT_Key_Clean;
							break;
						}
						ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);
						if (Touch_Detect())
						{
							ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
						}
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
				ROS_WARN("%s %d: cliff break!", __FUNCTION__, __LINE__);
				break;
			}
		}

#ifdef ENABLE_TILTED_DETECT

		if (Get_Left_CLiff_Value() < TILTED_CLIFF_LIMIT && Get_Right_CLiff_Value() < TILTED_CLIFF_LIMIT && Get_Front_CLiff_Value() < TILTED_CLIFF_LIMIT) {
			if (abs((int) (atan(((double)Gyro_GetXAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT ||
				abs((int) (atan(((double)Gyro_GetYAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT) {

				ROS_WARN("%s %d: possible tiled.", __FUNCTION__, __LINE__);

				Set_Wheel_Speed(0, 0);
				Set_Dir_Backward();
				usleep(300);
				CM_update_position(Gyro_GetAngle());

				if (abs((int) (atan(((double)Gyro_GetXAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT ||
					abs((int) (atan(((double)Gyro_GetYAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT) {

					ROS_WARN("%s %d: confirmed tiled.", __FUNCTION__, __LINE__);
					i = 0;
					do {
						ROS_WARN("%s %d: moving back count: %d", __FUNCTION__, __LINE__, i);
						if (abs((int) (atan(((double)Gyro_GetXAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT ||
							abs((int) (atan(((double)Gyro_GetYAcc()) / Gyro_GetZAcc()) * 1800 / PI) * (-1)) > TILTED_ANGLE_LIMIT) {
							CM_CorBack(COR_BACK_100MM);
							if (Touch_Detect())
							{
								Stop_Brifly();
								ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
								break;
							}
							Stop_Brifly();

#if (ROBOT_SIZE == 5)

							CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
							CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
							CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);

#else

							CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
							CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
							CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x, &y);
							Map_SetCell(MAP, x, y, BLOCKED_BUMPER);

#endif
						} else {
							ROS_INFO("%s %d: robot is not tiled anymore", __FUNCTION__, __LINE__);
							break;
						}
						i++;
					} while (i < 5);

					if (Touch_Detect())
					{
						ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
						retval = MT_Key_Clean;
						break;
					}
					retval = MT_None;
					break;
				}
			}
		}
#endif

		Temp_Rcon_Status = Get_Rcon_Status() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT | RconL_HomeT | RconR_HomeT);
		if (go_home == 0 && Temp_Rcon_Status) {
			// It just clear the bits that are 1 in Temp_Rcon_Status has.
			Set_Rcon_Status(Get_Rcon_Status() & (~Temp_Rcon_Status));
			if (Temp_Rcon_Status & (RconFR_HomeT | RconFL_HomeT)) {
				HomeT++;
			}
			if (Temp_Rcon_Status & (RconFL2_HomeT | RconL_HomeT)) {
				HomeL++;
			}
			if (Temp_Rcon_Status & (RconFR2_HomeT | RconR_HomeT)) {
				HomeR++;
			}

			// If detect charger stub then print the detection info
			if (HomeL || HomeR || HomeT){
				ROS_WARN("%s %d: home detected (%d %d %d)", __FUNCTION__, __LINE__, HomeL, HomeT, HomeR);
			}

			if (HomeR + HomeL + HomeT > 4) {
				home_hit = HomeR > HomeL ? HomeR : HomeL;
				home_hit = home_hit > HomeT ? home_hit : HomeT;

				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);
				if (home_hit == HomeR) {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else if (home_hit == HomeL) {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				}
				Stop_Brifly();
				retval = MT_None;
				// Update the location of charger stub
				CM_SetHome(Map_GetXCount(), Map_GetYCount());

				ROS_WARN("%s %d: all charge signal is detected", __FUNCTION__, __LINE__);
				break;
			} else if (HomeT == 0 && (HomeR > 2 || HomeL > 2)) {
				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);

				if (HomeR > 2) {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				} else if (HomeL > 2) {
#if (ROBOT_SIZE == 5)
					CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_3, &x, &y);
#else
					CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x, &y);
#endif
					Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
				}
				Stop_Brifly();
				retval = MT_None;
				// Update the location of charger stub
				CM_SetHome(Map_GetXCount(), Map_GetYCount());

				ROS_WARN("%s %d: left/right charger signal detected", __FUNCTION__, __LINE__);
				break;
			} else if (HomeR == 0 && HomeL == 0 && HomeT > 3) {
				Stop_Brifly();
				isBumperTriggered = Get_Bumper_Status();
				CM_update_map(action, isBumperTriggered);

#if (ROBOT_SIZE == 5)
				CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_3, &x, &y);
#else
				CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
				// Mark CELL_SIZE_3 to avoid position jump caused by slam
				CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_3, &x, &y);
#endif
				Map_SetCell(MAP, x, y, BLOCKED_BUMPER);

				Stop_Brifly();
				retval = MT_None;
				// Update the location of charger stub
				CM_SetHome(Map_GetXCount(), Map_GetYCount());

				ROS_WARN("%s %d: top charger signal is detected", __FUNCTION__, __LINE__);
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
			ROS_WARN("%s %d: OBS break, val: %d(%d)", __FUNCTION__, __LINE__, Get_FrontOBS(), Get_FrontOBST_Value());
			break;
		}

		/* Check bumper event.*/
		isBumperTriggered = Get_Bumper_Status();
		if (isBumperTriggered) {
			// Mark the status for rounding function.
			Bumper_Status_For_Rounding = isBumperTriggered;

			Stop_Brifly();
			CM_update_map_bumper(action, isBumperTriggered);
			//robot::instance()->pub_bumper_markers();

			ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
			CM_CorBack(COR_BACK_20MM);
			if (Touch_Detect())
			{
				ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
				retval = MT_Key_Clean;
				break;
			}

#ifdef BUMPER_ERROR
			if (Get_Bumper_Status()) {
				ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_20MM);
				if (Touch_Detect())
				{
					ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
					retval = MT_Key_Clean;
					break;
				}
				if (Get_Bumper_Status()) {
					CM_CorBack(COR_BACK_20MM);
					if (Touch_Detect())
					{
						ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
						retval = MT_Key_Clean;
						break;
					}
					if (Get_Bumper_Status()) {
						ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);
						if (Touch_Detect())
						{
							ROS_WARN("%s %d: Touch detect in CM_CorBack!", __FUNCTION__, __LINE__);
							retval = MT_Key_Clean;
							break;
						}
						Set_Error_Code(Error_Code_Bumper);
						Stop_Brifly();
						// If bumper jam, wait for manual release and it can keep on.(Convenient for testing)
						//retval = MT_Key_Clean;
						ROS_WARN("%s %d: bumper jam break! Please manual release the bumper!", __FUNCTION__, __LINE__);
						while (Get_Bumper_Status()){
							// Sleep for 2s and detect again, and beep to alarm in the first 0.5s
//							Beep(3, 25, 0, 1);
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
			ROS_WARN("%s %d: bumper break!", __FUNCTION__, __LINE__);
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
			ROS_INFO("reach the limited distance");
			ROS_INFO("Map_XCount = %d, Map_YCount = %d, Init_Pose_X = %d, Init_Pose_Y = %d, Dis_From_Init = %d, Limited_Distance = %d",
				Map_GetXCount(), Map_GetYCount(), Init_Pose_X, Init_Pose_Y, Dis_From_Init, Limited_Distance);

			Limited_Flag = 3;//Limit distance flag
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

		CM_update_position(Gyro_GetAngle());

#if 1
		/* Check map boundary. */
		boundary_reach = 0;
		for (i = -1; boundary_reach == 0 && i <= 1; i++) {
#if (ROBOT_SIZE == 5)
			CM_count_normalize(Gyro_GetAngle(), i * CELL_SIZE, CELL_SIZE_3, &x, &y);
#else

			CM_count_normalize(Gyro_GetAngle(), i * CELL_SIZE, CELL_SIZE_3, &x, &y);
			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
				slow_down = 1;
			}

			CM_count_normalize(Gyro_GetAngle(), i * CELL_SIZE, CELL_SIZE_2, &x, &y);
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
			ROS_WARN("%s %d: boundary break!", __FUNCTION__, __LINE__);
			break;
		}
#endif

		/*--------------------------Adjust Move ------------------------------------*/
		Rotate_Angle = course2dest(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y) - Gyro_GetAngle();

		if (Rotate_Angle >= 1800) {
			Rotate_Angle -= 3600;
		} else if (Rotate_Angle <= -1800) {
			Rotate_Angle += 3600;
		}
		if (std::abs(Rotate_Angle) > 300) {
			ROS_WARN("%s %d: warning: angle is too big, angle: %d", __FUNCTION__, __LINE__, Rotate_Angle);
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
			//ROS_INFO("%s %d: laser detected obstcal, slow down!", __FUNCTION__, __LINE__);
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
	CM_update_position(Gyro_GetAngle());

	ROS_INFO("%s %d: move to point: %d\tGyro Calibration: %d", __FUNCTION__, __LINE__, retval, Gyro_GetCalibration());
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
		ROS_INFO("%s %d: Normal move to next point at east.", __FUNCTION__, __LINE__);
		mt_state = CM_LinearMoveToPoint(target, RUN_TOP_SPEED, true, true);
	}

#else

	ROS_INFO("%s %d: Normal move to next point at east.", __FUNCTION__, __LINE__);
	mt_state = CM_LinearMoveToPoint(target, RUN_TOP_SPEED, true, true);

#endif

	return mt_state;
}

#if (PP_ROUNDING_OBSTACLE_LEFT) || (PP_ROUNDING_OBSTACLE_RIGHT)

RoundingType CM_get_rounding_direction(Point32_t *Next_Point, Point32_t Target_Point, uint16_t dir) {
	int32_t		y_coordinate;
	RoundingType	rounding_type = ROUNDING_NONE;

	ROS_INFO("Enter rounding detection.");
	if (should_follow_wall == 0 || Next_Point->Y == Map_GetYCount()) {
		return rounding_type;
	}

	if (Get_Cliff_Trig()) {
		ROS_INFO("%s %d, cliff triggered.", __FUNCTION__, __LINE__);
	} else if (Get_Bumper_Status()) {
		ROS_INFO("%s %d, bumper event.", __FUNCTION__, __LINE__);
	} else if (Get_OBS_Status()) {
		ROS_INFO("%s %d, OBS detected.", __FUNCTION__, __LINE__);
	} else if (Get_FrontOBS() > Get_FrontOBST_Value()) {
		ROS_INFO("%s %d, front OBS detected.", __FUNCTION__, __LINE__);
	} else if (Get_Wall_ADC(0) > 170) {
		ROS_INFO("%s %d, wall sensor exceed 170.", __FUNCTION__, __LINE__);
	}
	/*					South (Xmin)
	 *						|
	 *	West (Ymin)  --  robot	--	 East (Ymax)
	 *						|
	 *					North (Xmax)
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
		ROS_INFO("%s %d Robot don't need to go to new line. y: %d", __FUNCTION__, __LINE__, y_coordinate);
		if (!(countToCell(Next_Point->X) == SHRT_MAX || countToCell(Next_Point->X) == SHRT_MIN)) {
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

uint8_t CM_resume_cleaning()
{
#if CONTINUE_CLEANING_AFTER_CHARGE
	int8_t	state_for_continue_cleaning;

	// Handle Continue Cleaning
	if (go_home == 0 && robot::instance()->Is_Cleaning_Low_Bat_Paused())
	{
		ROS_INFO("Go to continue point: (%d, %d), targets left.", countToCell(Continue_Point.X), countToCell(Continue_Point.Y));
		lowBattery = 0;

		// Reset the cleaning pause flag.
		CM_reset_cleaning_low_bat_pause();
		// Try go to exactly this home point.
		state_for_continue_cleaning = CM_MoveToCell(countToCell(Continue_Point.X), countToCell(Continue_Point.Y), 2, 0, 1 );
		ROS_INFO("CM_MoveToCell return %d.", state_for_continue_cleaning);

		if (state_for_continue_cleaning == 1)
		{
			ROS_INFO("Robot has reach the continue point, continue cleaning.");
		}
		else if (state_for_continue_cleaning == -1 || state_for_continue_cleaning == -2)
		{
			ROS_INFO("Robot can't reach the continue point, directly continue cleaning.");
		}
		else if (state_for_continue_cleaning == -3 || state_for_continue_cleaning == -5)
		{
			if (state_for_continue_cleaning == -3)
			{
				ROS_INFO("Robot battery < 1200, stop it.");
				return 0;
			}
			else
			{
				ROS_INFO("Touch_Detect.");
				return 0;
			}
		}

		else if (state_for_continue_cleaning == -4)
		{
			ROS_INFO("Remote home pressed, go home.");
			remote_go_home = 1;
			go_home = 1;
		}
		else if (state_for_continue_cleaning == -6)
		{
			ROS_INFO("Low battery go home. go_home = %d", go_home);
			// go_home has been set to 1.
		}
		else if (state_for_continue_cleaning == -7)
		{
			ROS_INFO("Go home and near home now.");
			// go_home has been set to 1.
		}
	}
#endif
	return 1;
}

int CM_cleaning()
{
	bool	quit;
	float	slop, intercept;
	int8_t	state, retval;
	int16_t	x, y, x_current, y_current, start, end;

	uint16_t	last_dir;

	// X, Y in Target_Point are all counts.
	Point32_t	Next_Point, Target_Point;

	RoundingType	rounding_type;
	MapTouringType	mt_state = MT_None;

	retval = 0;
	quit = false;
	while (ros::ok() && quit == false) {
		if (map_touring_cancel == 1) {
			quit = true;
			retval = -1;
			continue;
		}

		//State -1: Path next
		//State  0: Target list is empty
		//State  2: Robot is trapped
		ROS_INFO("Find path-----------------------------");

		last_dir = path_get_robot_direction();

		x_current = Map_GetXPos();
		y_current = Map_GetYPos();
		state = path_next(&Next_Point.X, &Next_Point.Y, &Target_Point);
		ROS_INFO("Next point is (%d, %d)", countToCell(Next_Point.X), countToCell(Next_Point.Y));

		ROS_INFO("State: %d", state);
		ROS_INFO("[core_move.cpp] %s %d: Current Battery level: %d.", __FUNCTION__, __LINE__, GetBatteryVoltage());
		ROS_INFO("[core_move.cpp] %s %d: Current work time: %d(s).", __FUNCTION__, __LINE__, Get_Work_Time());

		if (state == 0) {		//No target point
			go_home = 1;
			quit = true;
		} else if (state == 1) {
			ROS_INFO("Move to target-----------------------------");

			rounding_type = ROUNDING_NONE;

#if (PP_ROUNDING_OBSTACLE_LEFT) || (PP_ROUNDING_OBSTACLE_RIGHT)
			rounding_type = CM_get_rounding_direction(&Next_Point, Target_Point, last_dir);
#endif

			if (rounding_type != ROUNDING_NONE) {
				ROS_INFO("%s %d: Rounding %s.", __FUNCTION__, __LINE__, rounding_type == ROUNDING_LEFT ? "left" : "right");
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

						ROS_INFO("%s %d: marking (%d, %d) (%d, %d) (%d, %d)", __FUNCTION__, __LINE__, x, y - 1, x, y, x, y + 1);
						Map_SetCell(MAP, cellToCount(x), cellToCount(y - 1), CLEANED);
						Map_SetCell(MAP, cellToCount(x), cellToCount(y), CLEANED);
						Map_SetCell(MAP, cellToCount(x), cellToCount(y + 1), CLEANED);
					}
				}
			}

			if (mt_state == MT_Battery) {
				quit = true;
				retval = -1;
			} else if (mt_state == MT_Remote_Home) {
				go_home = 1;
				Stop_Brifly();
				quit = true;
			} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
				Disable_Motors();
				quit = true;
				retval = -1;
			} else if (mt_state == MT_Battery_Home) {
				go_home = 1;
				Stop_Brifly();
				quit = true;
			}
		} else if (state == 2) {		// Trapped
			state = Map_Wall_Follow(Map_Wall_Follow_Escape_Trapped);

			if ( map_touring_cancel == 1 ) {
				quit = true;
				retval = -1;
			}

			if ( go_home == 1 ) {
				quit = true;
			}

			if (state == 2) {
				Disable_Motors();
				quit = true;
				retval = -1;
			}
		}
	}

	return retval;
}

void CM_reset_cleaning_low_bat_pause()
{
#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->Is_Cleaning_Low_Bat_Paused()) {
		// Due to robot can't successfully go back to charger stub, exit conintue cleaning.
		robot::instance()->Reset_Cleaning_Low_Bat_Pause();
	}
#endif
}

void CM_go_home()
{

	if(robot::instance()->Is_Cleaning_Low_Bat_Paused())
		wav_play(WAV_BATTERY_LOW);
	wav_play(WAV_BACK_TO_CHARGER);

	int8_t	state;
	int16_t	i;

	Point32_t	Next_Point, Target_Point;
	Point16_t	tmpPnt, pnt16ArTmp[3];

	state = -1;
	while (ros::ok()) {
		if (map_touring_cancel == 1) {
			return;
		}

		//2.2-1.1 Common process
		tmpPnt.X = countToCell(Home_Point.front().X);
		tmpPnt.Y = countToCell(Home_Point.front().Y);
		pnt16ArTmp[0] = tmpPnt;
		path_escape_set_trapped_cell(pnt16ArTmp, 1);

		if (remote_go_home == 1) {
			SetHomeRemote();
		}

#if CONTINUE_CLEANING_AFTER_CHARGE
		if (!robot::instance()->Is_Cleaning_Low_Bat_Paused())
		{
			//2.2-1.3 Path to unclean area
			CM_create_home_boundary();
		}
#else
		CM_create_home_boundary();
#endif

		if (Home_Point.empty())
		{
			ROS_ERROR("Miss start point (0, 0). But will continue after 10s.");
			tmpPnt.X = 0;
			tmpPnt.Y = 0;
			usleep(10000000);
		}
		else
		{
			// Try all the saved home point until it reach the charger stub. (There will be at least one home point (0, 0).)
			tmpPnt.X = countToCell(Home_Point.front().X);
			tmpPnt.Y = countToCell(Home_Point.front().Y);
			// Delete the first home point, it works like a stack.
			ROS_WARN("%s, %d: Go home Target: (%d, %d), %u targets left.", __FUNCTION__, __LINE__, tmpPnt.X, tmpPnt.Y, Home_Point.size());
			Home_Point.pop_front();
		}
		while (ros::ok())
		{
			// Try go to exactly this home point.
			state = CM_MoveToCell(tmpPnt.X, tmpPnt.Y, 2, 0, 1 );
			ROS_INFO("%s, %d: CM_MoveToCell for home point return %d.", __FUNCTION__, __LINE__, state);
			ROS_INFO("[core_move.cpp] %s %d: Current Battery level: %d.", __FUNCTION__, __LINE__, GetBatteryVoltage());
			ROS_INFO("[core_move.cpp] %s %d: Current work time: %d(s).", __FUNCTION__, __LINE__, Get_Work_Time());

			if ( state == -2 && Home_Point.empty()) {
				// state == -2 means it is trapped and can't go to the saved point.
				// If it is the last saved home point, stop the robot.
				Disable_Motors();
				// Beep for the finish signal.
				for (i = 10; i > 0; i--) {
//					Beep(i, 6, 0, 1);
					usleep(100000);
				}

				if (from_station >= 1) {
					Set_Clean_Mode(Clean_Mode_GoHome);
				} else {
					Set_Clean_Mode(Clean_Mode_Userinterface);
				}

				CM_reset_cleaning_low_bat_pause();

				ROS_WARN("%s %d: Finish cleanning but not stop near home, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
				return;
			} else if (state == -3) {
				// state == -3 means battery too low, battery < LOW_BATTERY_STOP_VOLTAGE (1200)
				Disable_Motors();
				// Beep for the finish signal.
//				for (i = 10; i > 0; i--) {
//					Beep(i, 6, 0, 1);
//					usleep(100000);
//				}
				Set_Clean_Mode(Clean_Mode_Sleep);

				CM_reset_cleaning_low_bat_pause();

				ROS_WARN("%s %d: Battery too low, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
				return;
			} else if (state == -5) {
				// state = -5 means clean key is pressed or cliff is triggered or remote key clean is pressed.
				Disable_Motors();
				// Beep for the finish signal.
//				for (i = 10; i > 0; i--) {
//					Beep(i, 6, 0, 1);
//					usleep(100000);
//				}
				Set_Clean_Mode(Clean_Mode_Userinterface);

				CM_reset_cleaning_low_bat_pause();

				// The current target home point is still valid, so push it back to the home point list.
				New_Home_Point.X = cellToCount(tmpPnt.X);
				New_Home_Point.Y = cellToCount(tmpPnt.Y);
				Home_Point.push_front(New_Home_Point);

				ROS_INFO("%s %d: Pause cleanning, cleaning time: %d(s), Home_Point list size: %u.", __FUNCTION__, __LINE__, Get_Work_Time(), Home_Point.size());
				return;
			} else if (state == 1 || state == -7) {
				// state == 1 means robot has reached the saved point.
				// state = -7 means go_home == 1 and it is near the charger stub.
				// Call GoHome() function to try to go to charger stub.
				GoHome();

				// In GoHome() function the clean mode might be set to Clean_Mode_GoHome, it should keep try GoHome().
				while (Get_Clean_Mode() == Clean_Mode_GoHome)
				{
					GoHome();
				}
				// Check the clean mode to find out whether it has reach the charger.
				if (Get_Clean_Mode() == Clean_Mode_Charging)
				{
#if CONTINUE_CLEANING_AFTER_CHARGE
					if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
					{
						ROS_WARN("%s %d: Pause cleaning for low battery, will continue cleaning when charge finished. Current cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
						return;
					}
#endif
					ROS_INFO("%s %d: Finish cleaning and stop in charger stub, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
					return;
				}
				else if (Get_Clean_Mode() == Clean_Mode_Sleep)
				{
					// Battery too low.
					Disable_Motors();
					// Beep for the finish signal.
//					for (i = 10; i > 0; i--) {
//						Beep(i, 6, 0, 1);
//						usleep(100000);
//					}
					Set_Clean_Mode(Clean_Mode_Sleep);

					CM_reset_cleaning_low_bat_pause();

					ROS_WARN("%s %d: Battery too low, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
					return;
				}
				else if (Touch_Detect())
				{
					Disable_Motors();
					// Beep for the finish signal.
//					for (i = 10; i > 0; i--) {
//						Beep(i, 6, 0, 1);
//						usleep(100000);
//					}
					Set_Clean_Mode(Clean_Mode_Userinterface);

					CM_reset_cleaning_low_bat_pause();

					Reset_Touch();
					ROS_INFO("%s %d: Finish cleanning, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
					return;
				}
				else if (Home_Point.empty())
				{
					// If it is the last point, it means it it now at (0, 0).
					if (from_station == 0) {
						CM_HeadToCourse(ROTATE_TOP_SPEED, -robot::instance()->robot_get_home_angle());

						if (Touch_Detect())
						{
							Stop_Brifly();
//							Beep(5, 20, 0, 1);
							ROS_INFO("%s %d: Touch detected in CM_HeadToCourse().", __FUNCTION__, __LINE__);
							// Key release detection, if user has not release the key, don't do anything.
							//ROS_WARN("%s %d: Press_Time = %d", __FUNCTION__, __LINE__,  Press_Time);
							while (Get_Key_Press() & KEY_CLEAN)
							{
								ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
								usleep(20000);
#if MANUAL_PAUSE_CLEANING
								Press_Time++;
								if (Press_Time == 151)
								{
									Beep(1, 5, 0, 1);
								}
							}
							if (Press_Time > 150)
							{
								if (robot::instance()->Is_Cleaning_Manual_Paused())
								{
									robot::instance()->Reset_Cleaning_Manual_Pause();
								}
							}
							else
							{
								Press_Time = 0;
#endif
							}
							// Key relaesed, then the touch status should be cleared.
							Reset_Touch();
						}
					}
					Disable_Motors();
					// Beep for the finish signal.
//					for (i = 10; i > 0; i--) {
//						Beep(i, 6, 0, 1);
//						usleep(100000);
//					}

					if (from_station >= 1) {
						Set_Clean_Mode(Clean_Mode_GoHome);
					} else {
						Set_Clean_Mode(Clean_Mode_Userinterface);
					}

#if CONTINUE_CLEANING_AFTER_CHARGE
					if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
					{
						ROS_WARN("%s %d: Can not go to charger stub after going to all home points. Finish cleaning, cleaning time: %d(s).", __FUNCTION__, __LINE__, Get_Work_Time());
						CM_reset_cleaning_low_bat_pause();
						return;
					}
#endif
					ROS_INFO("%s %d: Finish cleaning but can't go to charger stub, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
					return;
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
				ROS_WARN("%s, %d: Go home Target: (%d, %d), %u targets left.", __FUNCTION__, __LINE__, tmpPnt.X, tmpPnt.Y, Home_Point.size());

				/*
				// In GoHome() function, it may set the clean mode to Clean_Mode_GoHome. But it is not appropriate here, because it might affect the mode detection in CM_MoveToCell() and make it return -4.
				if (Get_Clean_Mode() == Clean_Mode_GoHome)
				{
					Set_Clean_Mode(Clean_Mode_Navigation);
				}
				*/
			}
			else
			{
				ROS_INFO("No home targets left and last target return %d, this message should not be printed, please check.", state);
				break;
			}
		}
	}
}

uint8_t CM_Touring(void)
{
	uint8_t Blink_LED = 0;
	int16_t	i;

	// Reset battery status
	lowBattery = 0;

	Reset_Rcon_Status();

	from_station = 0;
	map_touring_cancel = go_home = remote_go_home = 0;

	Set_LED(100,0);
	Reset_MoveWithRemote();
	Reset_Touch();

	Press_Time = 0;

#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
	{
		wav_play(WAV_CLEANING_CONTINUE);
	}
	else
#endif
	{
#if MANUAL_PAUSE_CLEANING
		if (robot::instance()->Is_Cleaning_Manual_Paused())
		{
			ROS_WARN("Restore from manual pause");
			wav_play(WAV_CLEANING_CONTINUE);
		}
		else
#endif
		{
			// Restart the gyro.
			Set_Gyro_Off();
			// Wait for 30ms to make sure the off command has been effectived.
			usleep(30000);
			// Set gyro on before wav_play can save the time for opening the gyro.
			Set_Gyro_On();
			wav_play(WAV_CLEANING_START);

			if (!Wait_For_Gyro_On())
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;
			}
		}
	}

	/*Move back from charge station*/
	if (Is_AtHomeBase()) {
		// Beep while moving back.
//		Beep(3, 25, 25, 6);
		// Key release detection, if user has not release the key, don't do anything.
		while (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
		// Key relaesed, then the touch status should be cleared.
		Reset_Touch();
		ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
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
			ROS_WARN("[core_move.cpp] Still charging.");
		}
		// Set i < 7 for robot to move back for approximately 500mm.
		for (i = 0; i < 7; i++) {
			// Move back for distance of 72mm, it takes approximately 0.5s.
			Quick_Back(20, 72);
			if (Touch_Detect() || Is_AtHomeBase()) {
				Set_Clean_Mode(Clean_Mode_Userinterface);
				Stop_Brifly();
				Set_SideBrush_PWM(0, 0);
//				Beep(5, 20, 0, 1);
				if (Is_AtHomeBase())
				{
					ROS_WARN("%s %d: move back 100mm and still detect charger! Or touch event. return 0", __FUNCTION__, __LINE__);
				}
				if (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_WARN("%s %d: touch event! return 0", __FUNCTION__, __LINE__);
					Stop_Brifly();
					// Key release detection, if user has not release the key, don't do anything.
					while (Get_Key_Press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					Reset_Touch();
				}
#if CONTINUE_CLEANING_AFTER_CHARGE
				if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
				{
					ROS_WARN("%s %d: fail to leave charger stub when continue to clean.", __FUNCTION__, __LINE__);
					// Quit continue cleaning.
					CM_reset_cleaning_low_bat_pause();
				}
#endif
#if MANUAL_PAUSE_CLEANING
				if (robot::instance()->Is_Cleaning_Manual_Paused())
				{
					robot::instance()->Reset_Cleaning_Manual_Pause();
				}
#endif
				return 0;
			}
		}
		Deceleration();
		Stop_Brifly();
		from_station = 1;
	}
	else
	{
		// Key release detection, if user has not release the key, don't do anything.
		while (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
		// Key relaesed, then the touch status should be cleared.
		Reset_Touch();
	}

	Blink_LED = 8;
	Reset_Touch();
	/*
	ROS_INFO("while Blink_LED-----------------------------");
	while (Blink_LED--) {
		if (Touch_Detect()) {
			ROS_INFO("Touch_Detect1-----------------------------");
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 0;
		}
		usleep(200000);
		if (Touch_Detect()) {
			ROS_INFO("Touch_Detect2-----------------------------");
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 0;
		}
		usleep(200000);
	}
	*/

	New_Home_Point.X = New_Home_Point.Y = 0;

#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
	{
		if (Get_Rcon_Status())
		{
			// Save the current coordinate as a new home point.
			New_Home_Point.X = Map_GetXCount();
			New_Home_Point.Y = Map_GetYCount();

			// Push the start point into the home point list.
			Home_Point.push_front(New_Home_Point);
		}

		Reset_Rcon_Status();
	}
	else
#endif
	{
#if MANUAL_PAUSE_CLEANING
		if (robot::instance()->Is_Cleaning_Manual_Paused())
		{
			// Don't initialize the map, etc.
		}
		else
#endif
		{
			// Set the Work_Timer_Start as current time
			Reset_Work_Time();

			//Initital home point
			Home_Point.clear();

			// Push the start point into the home point list
			Home_Point.push_front(New_Home_Point);

			ROS_INFO("Map_Initialize-----------------------------");
			Map_Initialize();
			PathPlanning_Initialize(&Home_Point.front().X, &Home_Point.front().Y);

			Reset_Rcon_Status();

			/* usleep for checking whether robot is in the station */
			usleep(20000);

			robot::instance()->init_mumber();// for init robot member

#if CONTINUE_CLEANING_AFTER_CHARGE
			// If it it the first time cleaning, initialize the Continue_Point.
			Continue_Point.X = Continue_Point.Y = 0;
#endif
		}

	}

	Motion_controller motion;

//	Set_Clean_Mode(Clean_Mode_Navigation);
//	return 0;
#if MANUAL_PAUSE_CLEANING
	// Clear the pause status.
	if (robot::instance()->Is_Cleaning_Manual_Paused())
	{
		robot::instance()->Reset_Cleaning_Manual_Pause();
	}
#endif

	if(except_event()){
		while (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
		ROS_WARN("%s %d: Check: Touch Clean Mode! return 0\n", __FUNCTION__, __LINE__);
		Set_Clean_Mode(Clean_Mode_Userinterface);
#if CONTINUE_CLEANING_AFTER_CHARGE
		// Reset continue cleaning status
		CM_reset_cleaning_low_bat_pause();
#endif
#if MANUAL_PAUSE_CLEANING
		if (robot::instance()->Is_Cleaning_Manual_Paused())
		{
			robot::instance()->Reset_Cleaning_Manual_Pause();
		}
#endif
		return 0;
	}

	//Check if slam is ok
	if (Is_Slam_Ready) {
		Is_Slam_Ready = 0;
	} else {
		Is_Slam_Ready = 0;
		Set_Error_Code(Error_Code_Slam);
		Set_Clean_Mode(Clean_Mode_Userinterface);
		wav_play(WAV_TEST_LIDAR);
		return 0;
	}
		/* usleep for checking whether robot is in the station */
	usleep(700);
	if (from_station == 1 && !robot::instance()->align_active()) {
		ROS_INFO("%s %d: Turn 45 degree to the wall", __FUNCTION__, __LINE__);

		CM_HeadToCourse(ROTATE_TOP_SPEED, Gyro_GetAngle() - 450);

		if (Touch_Detect()) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			ROS_WARN("%s %d: Check: Touch Clean Mode! return 0", __FUNCTION__, __LINE__);
//			Beep(5, 20, 0, 1);
			Stop_Brifly();
			// Key release detection, if user has not release the key, don't do anything.
			//ROS_WARN("%s %d: Press_Time = %d", __FUNCTION__, __LINE__,  Press_Time);
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
#if MANUAL_PAUSE_CLEANING
				Press_Time++;
				if (Press_Time == 151)
				{
					Beep(1, 5, 0, 1);
				}
			}
			if (Press_Time > 150)
			{
				if (robot::instance()->Is_Cleaning_Manual_Paused())
				{
					robot::instance()->Reset_Cleaning_Manual_Pause();
				}
			}else
			{
				Press_Time = 0;
#endif
			}
			Reset_Touch();
			return 0;
		}

		from_station = 1;
	}

	if (CM_resume_cleaning())
	{
		if (CM_cleaning() == 0) {
			CM_go_home();
		}
		else
		{
			// Cleaning shuted down, battery too low or touch detected.
			Set_Clean_Mode(Clean_Mode_Userinterface);
			// Reset continue cleaning status
			CM_reset_cleaning_low_bat_pause();
		}
	}
	else
	{
		// Resume cleaning failed, battery too low or touch detected.
		Set_Clean_Mode(Clean_Mode_Userinterface);
		// Reset continue cleaning status
		CM_reset_cleaning_low_bat_pause();
	}

#if CONTINUE_CLEANING_AFTER_CHARGE
	if (!robot::instance()->Is_Cleaning_Low_Bat_Paused())
#endif
	{
#if MANUAL_PAUSE_CLEANING
		if (!robot::instance()->Is_Cleaning_Manual_Paused())
#endif
		{
			Home_Point.clear();
		}
	}
	return 0;
}

/*
 * Robot move to target cell
 * @param x	cell x
 * @param y	cell y
 * @param mode 2: Dynamic change cells near target cell
 *			   1: with escape mode, not finish
 *			   0: no escape mode
 * @return	-2: Robot is trapped
 *		-1: Robot cannot move to target cell
 *		1: Robot arrive target cell
 */
int8_t CM_MoveToCell( int16_t x, int16_t y, uint8_t mode, uint8_t length, uint8_t step )
{
	Point32_t		Next_Point;
	int8_t		pathFind;
	int16_t		i, j, k;
	uint16_t	offsetIdx = 0;
	Point16_t	tmp, pos;
	MapTouringType	mt_state = MT_None;

	if (is_block_accessible(x, y) == 0) {
		ROS_WARN("%s %d: target is blocked.\n", __FUNCTION__, __LINE__);
		Map_Set_Cells(ROBOT_SIZE, x, y, CLEANED);
	}

	//Escape mode
	//TODO: Escape
	if ( mode ==  1 ) {
		ROS_WARN("%s %d: Path Find: Escape Mode", __FUNCTION__, __LINE__);

		pos.X = x;
		pos.Y = y;
		pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(),  &tmp.X, &tmp.Y);

		return 0;
	} else if ( mode == 2 ) {
		i = j = k = offsetIdx = 0;

		Point16_t relativePosTmp = {0, 0};
		ROS_INFO("%s %d: Path Find: Dynamic Target Mode, target: (%d, %d)", __FUNCTION__, __LINE__, x, y);

		relativePos[k].X = 0;
		relativePos[k].Y = 0;
		k = 1;
		for ( i = -length; i <= length; i += step ) {
			for ( j = -length; j <= length; j += step ) {
				if ( x + i <= xMax && x + i >= xMin &&	y + j <= yMax && y + j >= yMin ) {
					if ( i == 0 && j == 0 ) {
						continue;
					}
					relativePos[k].X = i;
					relativePos[k].Y = j;
					ROS_INFO("%s %d: Id: %d\tPoint: (%d, %d)", __FUNCTION__, __LINE__, k, relativePos[k].X, relativePos[k].Y);
					++k;
				}
			}
		}
		ROS_INFO("%s %d: Size: %d", __FUNCTION__, __LINE__, k);

		//Position sort, two case: 1. sort for the previous half size of point; 2. sort the rest.
		//Sort from the nearest point to the farest point, refer to the middle point
		for ( i = 1 ; i < k; ++i) {
			for ( j = 1; j < k - i; ++j ) {
				if ( TwoPointsDistance( relativePos[j].X * 1000,	 relativePos[j].Y * 1000,	  0, 0 ) >
					 TwoPointsDistance( relativePos[j + 1].X * 1000, relativePos[j + 1].Y * 1000, 0, 0 ) ) {
					relativePosTmp = relativePos[j + 1];
					relativePos[j + 1] = relativePos[j];
					relativePos[j] = relativePosTmp;
				}
			}
		}

		ROS_INFO("%s %d: Bubble sort:", __FUNCTION__, __LINE__);
		for ( i = 0; i < k; i++ ) {
			ROS_INFO("%s %d: Id: %d\tPoint: (%d, %d)\tDis: %d", __FUNCTION__, __LINE__, i, relativePos[i].X, relativePos[i].Y,
					 TwoPointsDistance( relativePos[i].X * 1000, relativePos[i].Y * 1000, 0, 0 ));
		}

		pos.X = x + relativePos[0].X;
		pos.Y = y + relativePos[0].Y;
		pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &tmp.X, &tmp.Y);

		//Set cell
		Map_Set_Cells(ROBOT_SIZE, x + relativePos[0].X, y + relativePos[0].Y, CLEANED);

		ROS_INFO("%s %d: Path Find: %d", __FUNCTION__, __LINE__, pathFind);
		ROS_INFO("%s %d: Target need to go: x: %d\ty: %d", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		ROS_INFO("%s %d: Now: x: %d\ty: %d", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
		while (1) {
			if ( pathFind == 1 || pathFind == SCHAR_MAX ) {
				path_set_current_pos();

				ROS_INFO("%s %d: Move to target...", __FUNCTION__, __LINE__ );
				Next_Point.X = cellToCount(tmp.X);
				Next_Point.Y = cellToCount(tmp.Y);

#if ENABLE_DEBUG
				debug_map(MAP, tmp.X, tmp.Y);
#endif

				mt_state = CM_MoveToPoint(Next_Point);
				ROS_INFO("%s %d: Arrive Target! Now: (%d, %d)", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());

				if (mt_state == MT_Battery) {
					ROS_WARN("%s %d: low battery is detected, battery < 1200", __FUNCTION__, __LINE__);
					return -3;
				} else if (mt_state == MT_Remote_Home) {
					ROS_WARN("%s %d: home is pressed", __FUNCTION__, __LINE__);
					return -4;
				} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
					ROS_WARN("%s %d: remote is pressed, clean key is pressed,  or cliff is reached", __FUNCTION__, __LINE__);
					return -5;
				} else if (mt_state == MT_Battery_Home) {
					ROS_WARN("%s %d: low battery is detected, battery < 1300", __FUNCTION__, __LINE__);
					return -6;
				} else if ( mt_state == MT_None ) {
					if ( go_home == 1 && Is_Station() == 1 ) {
						return -7;
					}
				}

				//Arrive exit cell, set < 3 when ROBOT_SIZE == 5
				if ( TwoPointsDistance( x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y, Map_GetXPos(), Map_GetYPos() ) < ROBOT_SIZE / 2 + 1 ) {
					ROS_WARN("%s %d: Now: x: %d\ty: %d", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
					ROS_WARN("%s %d: Destination: x: %d\ty: %d", __FUNCTION__, __LINE__, x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);
					return 1;
				}

				if (is_block_accessible(x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y) == 0) {
					ROS_WARN("%s %d: Target is blocked. Try to find new target.", __FUNCTION__, __LINE__);
					pathFind = -2;
					continue;
				}

				pos.X = x + relativePos[offsetIdx].X;
				pos.Y = y + relativePos[offsetIdx].Y;
				pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &tmp.X, &tmp.Y);

				if (CM_CheckLoopBack(tmp) == 1) {
					pathFind = -2;
				}
				ROS_INFO("%s %d: Path Find: %d, target: (%d, %d)", __FUNCTION__, __LINE__, pathFind,
						 x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);
				ROS_INFO("%s %d: Target need to go: x: %d\ty: %d", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
				ROS_INFO("%s %d: Now: x: %d\ty: %d", __FUNCTION__, __LINE__, countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
			} else if ( pathFind == -2 || pathFind == -1 ) {
				//Add offset
				offsetIdx++;
				if ( relativePos[offsetIdx].X == 0 && relativePos[offsetIdx].Y == 0 )
					offsetIdx++;

				if ( offsetIdx >= k ) {
					return -2;
				}

				pos.X = x + relativePos[offsetIdx].X;
				pos.Y = y + relativePos[offsetIdx].Y;
				pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &tmp.X, &tmp.Y);

				if (Touch_Detect()) {
					ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
					// Set touch status to make sure this event can be detected by main process while loop.
					Stop_Brifly();
					Set_Touch();
					return -5;
				}

				if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right)) {
					ROS_WARN("%s %d: robot is taken up.", __FUNCTION__, __LINE__);
					Stop_Brifly();
					return -5;
				}

				ROS_INFO("%s %d: Path Find: %d, %d Target Offset: (%d, %d)", __FUNCTION__, __LINE__, pathFind, offsetIdx,
						 relativePos[offsetIdx].X, relativePos[offsetIdx].Y);
				ROS_INFO("%s %d: Path Find: %d, target: (%d, %d)", __FUNCTION__, __LINE__, pathFind,
						 x + relativePos[offsetIdx].X, y + relativePos[offsetIdx].Y);

			} else {
				return pathFind;
			}
		}
	}
	//Normal mode
	else {
		ROS_INFO("%s %d: Path Find: Normal Mode, target: (%d, %d)", __FUNCTION__, __LINE__, x, y);
		pos.X = x;
		pos.Y = y;
		pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &tmp.X, &tmp.Y);

		ROS_INFO("%s %d: Path Find: %d", __FUNCTION__, __LINE__, pathFind);
		ROS_INFO("%s %d: Target need to go: x: %d\ty: %d", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		ROS_INFO("%s %d: Now: x: %d\ty: %d", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());

		//Note that path_move_to_unclean_area only can get the next cell to the destination cell
		while ( pathFind == 1 || pathFind == SCHAR_MAX ) {
			path_set_current_pos();

			ROS_INFO("%s %d: Move to target...", __FUNCTION__, __LINE__ );
			Next_Point.X = cellToCount(tmp.X);
			Next_Point.Y = cellToCount(tmp.Y);
#if ENABLE_DEBUG
			debug_map(MAP, tmp.X, tmp.Y);
#endif

			mt_state = CM_MoveToPoint(Next_Point);
			ROS_INFO("%s %d: Arrive Target! Now: (%d, %d)", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());

			if (mt_state == MT_Battery) {
				ROS_INFO("%s %d: low battery is detected, battery < 1200", __FUNCTION__, __LINE__);
				return -3;
			} else if (mt_state == MT_Remote_Home) {
				ROS_INFO("%s %d: home is pressed", __FUNCTION__, __LINE__);
				return -4;
			} else if (mt_state == MT_Remote_Clean || mt_state == MT_Cliff || mt_state == MT_Key_Clean) {
				ROS_INFO("%s %d: remote is pressed, clean key is pressed,  or cliff is reached", __FUNCTION__, __LINE__);
				return -5;
			} else if (mt_state == MT_Battery_Home) {
				ROS_INFO("%s %d: low battery is detected, battery < 1300", __FUNCTION__, __LINE__);
				return -6;
			} else if ( mt_state == MT_None ) {
				if ( go_home == 1 && Is_Station() == 1 ) {
					return -7;
				}
			}

			//Arrive exit cell, set < 3 when ROBOT_SIZE == 5
			if ( TwoPointsDistance( x, y, Map_GetXPos(), Map_GetYPos() ) < ROBOT_SIZE / 2 + 1 ) {
				ROS_INFO("%s %d: Now: x:%d \ty: %d", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos());
				ROS_INFO("%s %d: Destination: x: %d\ty: %d", __FUNCTION__, __LINE__, x, y);
				return 1;
			}

			if (is_block_accessible(x, y) == 0) {
				ROS_INFO("%s %d: target is blocked", __FUNCTION__, __LINE__);
				return 0;
			}

			pos.X = x;
			pos.Y = y;
			pathFind = path_move_to_unclean_area(pos, Map_GetXPos(), Map_GetYPos(), &tmp.X, &tmp.Y);

			ROS_INFO("%s %d: Path Find: %d, target: (%d, %d)", __FUNCTION__, __LINE__, pathFind, x, y);
			ROS_INFO("%s %d: Target need to go: x: %d\ty: %d", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
			ROS_INFO("%s %d: Now: x: %d\ty: %d", __FUNCTION__, __LINE__, countToCell(Map_GetXCount()), countToCell(Map_GetYCount()));
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

	ROS_INFO("%s %d: Moving back...", __FUNCTION__, __LINE__);
	Stop_Brifly();
	CM_update_position(Gyro_GetAngle());
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

		CM_update_position(Gyro_GetAngle());
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
			ROS_INFO("%s %d: Touch detected!", __FUNCTION__, __LINE__);
			Stop_Brifly();
			// Set touch status to make sure this event can be detected by main process while loop.
			Set_Touch();
			break;
		}
		uint8_t octype = Check_Motor_Current();
		if ((octype == Check_Left_Wheel) || ( octype  == Check_Right_Wheel)) {
			ROS_INFO("%s ,%d, motor over current",__FUNCTION__,__LINE__);
			break;
		}
	}
	CM_update_position(Gyro_GetAngle());
	Reset_TempPWM();
	Stop_Brifly();
	ROS_INFO("%s %d: Moving back done!", __FUNCTION__, __LINE__);
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
	bool found = false;

	ROS_INFO("%s %d: Push new reachable home: (%d, %d) to home point list.", __FUNCTION__, __LINE__, countToCell(x), countToCell(y));
	New_Home_Point.X = x;
	New_Home_Point.Y = y;

	for (list<Point32_t>::iterator it = Home_Point.begin(); found == false && it != Home_Point.end(); ++it) {
		if (it->X == x && it->Y == y) {
			found = true;
		}
	}
	if (found == false) {
		Home_Point.push_front(New_Home_Point);
	}
}

#if CONTINUE_CLEANING_AFTER_CHARGE
void CM_SetContinuePoint(int32_t x, int32_t y)
{
	ROS_INFO("%s %d: Set continue point: (%d, %d).", __FUNCTION__, __LINE__, countToCell(x), countToCell(y));
	Continue_Point.X = x;
	Continue_Point.Y = y;
}
#endif

uint8_t CM_IsLowBattery(void) {
	return lowBattery;
}

uint8_t CM_CheckLoopBack( Point16_t target ) {
	uint8_t retval = 0;
	if ( target.X == positions[1].x && target.Y == positions[1].y &&
		 target.X == positions[3].x && target.Y == positions[3].y ) {
		ROS_WARN("%s %d Possible loop back (%d, %d)", __FUNCTION__, __LINE__, target.X, target.Y);
		retval	= 1;
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
		// Robot battery below LOW_BATTERY_GO_HOME_VOLTAGE (1320).
		lowBattery = 1;
		if ( Get_VacMode() == Vac_Max ) {
			Switch_VacMode();
		}
		Stop_Brifly();
		ROS_WARN("%s %d: low battery, battery < 13.2v is detected.", __FUNCTION__, __LINE__);
		CM_SetGoHome(0);
#if CONTINUE_CLEANING_AFTER_CHARGE
		CM_SetContinuePoint(Map_GetXCount(), Map_GetYCount());
		robot::instance()->Set_Cleaning_Low_Bat_Pause();
#endif
		return MT_Battery_Home;
	}
	/*
	//for testing
	if (Remote_Key(Remote_Left) && go_home != 1) {
		// Robot battery below LOW_BATTERY_GO_HOME_VOLTAGE (1320).
		lowBattery = 1;
		if ( Get_VacMode() == Vac_Max ) {
			Switch_VacMode();
		}
		Stop_Brifly();
		ROS_WARN("%s %d: (For test, left key pressed) low battery, battery < 13.2v is detected.", __FUNCTION__, __LINE__);
		CM_SetGoHome(0);
#if CONTINUE_CLEANING_AFTER_CHARGE
		CM_SetContinuePoint(Map_GetXCount(), Map_GetYCount());
		robot::instance()->Set_Cleaning_Low_Bat_Pause();
#endif
		Reset_Rcon_Remote();
		return MT_Battery_Home;
	}
	*/

	/* Check key press events. */
	if (Touch_Detect()) {
		Stop_Brifly();
		ROS_WARN("%s %d: Touch_Detect in CM_handleExtEvent.", __FUNCTION__, __LINE__);
//		Beep(5, 20, 0, 1);
		// Key release detection, if user has not release the key, don't do anything.
		//ROS_WARN("%s %d: Press_Time = %d", __FUNCTION__, __LINE__,  Press_Time);
		while (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: User hasn't release key.", __FUNCTION__, __LINE__);
			usleep(20000);
#if MANUAL_PAUSE_CLEANING
			Press_Time++;
			if (Press_Time == 151)
			{
				Beep(1, 5, 0, 1);
			}
		}
		if (Press_Time > 150)
		{
			if (robot::instance()->Is_Cleaning_Manual_Paused())
			{
				robot::instance()->Reset_Cleaning_Manual_Pause();
			}
		}
		else
		{
			Press_Time = 0;
#endif
		}
		Reset_Touch();
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return MT_Key_Clean;
	}

	/* Check remote events. */
	if (Get_Rcon_Remote() > 0) {
		ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
		if (Get_Rcon_Remote() & (Remote_Clean | Remote_Home | Remote_Max | Remote_Spot)) {
			/* Check remote home key press event, if home key is pressed, go home directly. */
			if (Remote_Key(Remote_Home) && (go_home == 0)) {
				Set_BLDC_Speed(Vac_Speed_NormalL);
				Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power);
				Stop_Brifly();
				ROS_WARN("%s %d: remote home is pressed.", __FUNCTION__, __LINE__);
				CM_SetGoHome(1);
				Reset_Rcon_Remote();
				return MT_Remote_Home;
			}


			/*
			 * Check remote spot key press event, if spot key is pressed,
			 * change to spot mode, after spot mode finished, back to zig-zag clean.
			 */

			if (Remote_Key(Remote_Spot)) {
				Stop_Brifly();
				Reset_Rcon_Remote();
				ROS_WARN("%s %d: remote spot is pressed.", __FUNCTION__, __LINE__);
				Spot_Mode(CleanSpot);
				Set_VacMode(Vac_Max);
				Switch_VacMode();
				ROS_WARN("%s %d: remote spot ends.", __FUNCTION__, __LINE__);
				return MT_None;
			}
	

			if (Remote_Key(Remote_Max)) {
				if (lowBattery == 0) {
					Switch_VacMode();
				}
				Reset_Rcon_Remote();
			}


			/* Check remote clean key press event, if clean key is pressed, stop robot directly. */
			if (Remote_Key(Remote_Clean)) {
				Stop_Brifly();
				ROS_WARN("%s %d: remote clean key pressed.", __FUNCTION__, __LINE__);
				Reset_Rcon_Remote();
				return MT_Remote_Clean;
			}
			Reset_Rcon_Remote();
		} else {
			Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
			Reset_Rcon_Remote();
		}
	}

	/* Check whether robot is taken up. */
	if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right)) {
		ROS_INFO("%s %d: robot is taken up.\n", __FUNCTION__, __LINE__); 
		Stop_Brifly();
		wav_play(WAV_ERROR_LIFT_UP);
		return MT_Cliff;
	}
    /* check plan setting*/
	if(Get_Plan_Status())
	{
		Set_Plan_Status(false);
//		wav_play(WAV_APPOINTMENT_DONE);
		Beep(Beep_Error_Sounds, 2, 0, 1);
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
	ROS_INFO("%s %d: x: %d - %d\ty: %d - %d", __FUNCTION__, __LINE__, xMinSearch, xMaxSearch, yMinSearch, yMaxSearch);
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
