/*
 ******************************************************************************
 * @file	AI Cleaning Robot
 * @author	ILife Team Dxsong
 * @version V1.0
 * @date	17-Nov-2011
 * @brief	Move near the wall on the left in a certain distance
 ******************************************************************************
 * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include "robot.hpp"
#include "movement.h"
#include "core_move.h"
#include "gyro.h"
#include "map.h"
#include "mathematics.h"
#include "path_planning.h"
#include "wall_follow_slam.h"
#include "wall_follow_trapped.h"
#include <ros/ros.h>
#include "debug.h"
#include "rounding.h"
#include <vector>
#include <shortest_path.h>
#include "charger.hpp"
#include "wav.h"
#include "robotbase.h"

#include "motion_manage.h"
//Turn speed
#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed  18
#endif

std::vector<Pose16_t> g_wf_cell;
Pose16_t g_new_wf_point;
// This list is for storing the position that robot sees the charger stub.
extern std::list<Point32_t> g_home_point;
extern uint8_t g_from_station;
extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

extern Cell_t g_trapped_cell[ESCAPE_TRAPPED_REF_CELL_SIZE];
extern uint8_t g_trapped_cell_size;
extern Cell_t g_cell_history[5];
extern uint8_t g_wheel_left_direction;
extern uint8_t g_wheel_right_direction;

//Timer
uint32_t g_wall_follow_timer;
int32_t g_reach_count = 0;
const static int32_t REACH_COUNT_LIMIT = 10;//10 represent the wall follow will end after overlap 10 cells
int32_t g_same_cell_count = 0;
//MFW setting
static const MapWallFollowSetting MFW_SETTING[6] = {{1200, 250, 150},
																										{1200, 250, 150},
																										{1200, 250, 150},
																										{1200, 250, 70},
																										{1200, 250, 150},
																										{1200, 250, 150},};
//------------static
static int8_t wf_push_cell(Pose16_t &pose)
{
	if (!g_wf_cell.empty() && g_wf_cell.back() != pose)
	{
		g_same_cell_count = 0;
		g_new_wf_point = pose;
		g_wf_cell.push_back(g_new_wf_point);
		return 1;
	} else if (g_wf_cell.back() == pose)
	{
		g_same_cell_count++;//for checking if the robot is traped
		return 0;//it means still in the same cell
	}
}

static bool is_reach(void)
{
	if ( g_reach_count < REACH_COUNT_LIMIT)
		return false;

	int16_t th_diff;
	int16_t DIFF_LIMIT = 1500;//1500 means 150 degrees, it is used by angle check.
	int8_t pass_count = 0;
	int8_t sum = REACH_COUNT_LIMIT;
	bool fail_flag = 0;
	try
	{
		for (int i = 1; i <= REACH_COUNT_LIMIT; i++)
		{
			Pose16_t cell = g_wf_cell[g_wf_cell.size() - i];
			fail_flag = 0;
			for (std::vector<Pose16_t>::reverse_iterator r_iter = (g_wf_cell.rbegin() + i); r_iter != g_wf_cell.rend(); ++r_iter)
			{
				if (*r_iter == cell)
				{
					th_diff = (abs(r_iter->TH - cell.TH));

					if (th_diff > 1800)
						th_diff = 3600 - th_diff;

					if (th_diff <= DIFF_LIMIT)
					{
						pass_count++;
						break;
					} else
						fail_flag = 1;
				}
				/*in case of the g_wf_cell no second same point, the g_reach_count++ caused by cleanning the block obstacle which
				 cost is 2 or 3, it will be set 1(CLEANED), at this situation the sum should sum--, it will happen when the robot
				 get close to the left wall but there is no wall exist, then the robot can judge it is in the isolate island.*/
				if ((r_iter == (g_wf_cell.rend() - 1)) && (fail_flag == 0))
					sum--;
			}
		}

		if (sum < REACH_COUNT_LIMIT) return 0;

		if (pass_count < REACH_COUNT_LIMIT) return 0; else return 1;
	}
	catch (const std::out_of_range &oor)
	{
		std::cerr << "Out of range error:" << oor.what() << '\n';
	}
	return 0;
}

static bool is_trap(void)
{
	if (g_same_cell_count >= 1000)
	{
		ROS_WARN("Maybe the robot is trapped! Checking!");
		g_same_cell_count = 0;
/*		if (wf_is_isolate())
		{
			ROS_WARN("Not trapped!");
		} else
		{
			ROS_WARN("Trapped!");
			set_clean_mode(Clean_Mode_Userinterface);
			return 0;
		}*/
		return 0;
	}
	return false;
}

static bool is_time_out(void)
{
	if ((time(NULL) - g_wall_follow_timer) > WALL_FOLLOW_TIME)
	{
		ROS_INFO("Wall Follow time longer than 60 minutes");
		ROS_INFO("time now : %d", (int(time(NULL)) - g_wall_follow_timer));
		wf_clear();
		return 1;
	}
	return 0;
}


static void wf_update_cleaned()
{

	int16_t x, y;
	auto heading = Gyro_GetAngle();
	if (x != map_get_x_cell() || y != map_get_y_cell())
	{
		for (auto c = 1; c >= -1; --c)
		{
			for (auto d = 1; d >= -1; --d)
			{
				auto i = map_get_relative_x(heading, CELL_SIZE * c, CELL_SIZE * d);
				auto j = map_get_relative_y(heading, CELL_SIZE * c, CELL_SIZE * d);
				auto e = map_get_cell(MAP, count_to_cell(i), count_to_cell(j));

				if (e == BLOCKED_OBS || e == BLOCKED_BUMPER || e == BLOCKED_BOUNDARY)
				{
					map_set_cell(MAP, i, j, CLEANED);
				}
			}
		}
	}
}

static bool wf_is_reach_cleaned(void)
{
	if (! g_wf_cell.empty())
		if ( map_get_cell(MAP, g_wf_cell.back().X, (g_wf_cell.back().Y) == CLEANED) )
			return true;
	return false;
}

static void wf_mark_home_point(void)
{
	//path_planning_initialize(&, &g_home_point.front().Y);
	int32_t x, y;
	int i, j;
	std::list<Point32_t> WF_Home_Point;

	WF_Home_Point = g_home_point;

	while (!WF_Home_Point.empty())
	{
		x = WF_Home_Point.front().X;
		y = WF_Home_Point.front().Y;
		ROS_INFO("%s %d: WF_Home_Point.front().X = %d, WF_Home_Point.front().Y = %d, WF_Home_Point.size() = %d",
						 __FUNCTION__, __LINE__, x, y, (uint) WF_Home_Point.size());
		ROS_INFO("%s %d: g_x_min = %d, g_x_max = %d", __FUNCTION__, __LINE__, g_x_min, g_x_max);
		WF_Home_Point.pop_front();

		for (i = -2; i <= 2; i++)
		{
			for (j = -2; j <= 2; j++)
			{
				map_set_cell(MAP, cell_to_count(x + i), cell_to_count(y + j), CLEANED);//0, -1
				//ROS_INFO("%s %d: x + i = %d, y + j = %d", __FUNCTION__, __LINE__, x + i, y + j);
			}
		}
	}
}

/*------------------------------------------------------------------ Wall Follow Mode--------------------------*/
uint8_t wf_break_wall_follow(void)
{
	/*****************************************Release Memory************************************/
	g_home_point.clear();
	g_wf_cell.clear();
	std::vector<Pose16_t>(g_wf_cell).swap(g_wf_cell);
	set_clean_mode(Clean_Mode_Userinterface);
	return 0;
}

uint8_t wf_clear(void)
{
	robot::instance()->setBaselinkFrameType( Map_Position_Map_Angle);//inorder to use the slam angle to finsh the shortest path to home;
	cm_update_position();
	wf_mark_home_point();
	wf_break_wall_follow();
	return 0;
}

void wf_update_map()
{
	auto heading = Gyro_GetAngle();
	extern Point32_t g_next_point, g_target_point;

	cm_update_position();

	wf_update_cleaned();

	auto cell_x = map_get_x_cell();
	auto cell_y = map_get_y_cell();
	Pose16_t curr_cell{cell_x, cell_y, Gyro_GetAngle()};
	if (wf_push_cell(curr_cell))
	{
		g_reach_count = (wf_is_reach_cleaned()) ? g_reach_count + 1 : 0;
		int size = (g_wf_cell.size() - 2);
		if (size >= 0)
			map_set_cell(MAP, cell_to_count(g_wf_cell[size].X), cell_to_count(g_wf_cell[size].Y), CLEANED);
	}

	cell_x = map_get_relative_x(heading, CELL_SIZE_2, 0);
	cell_y = map_get_relative_y(heading, CELL_SIZE_2, 0);
	if (map_get_cell(MAP, count_to_cell(cell_x), count_to_cell(cell_y)) != BLOCKED_BOUNDARY)
		map_set_cell(MAP, cell_x, cell_y, BLOCKED_OBS);
}

bool wf_is_start()
{
	static bool is_first_time = true;
	if(is_first_time){
		is_first_time = false;
		return true;
	}
	return false;
}
bool wf_is_isolate() {


	path_update_cell_history();
	int16_t	val = 0;
	uint16_t i = 0;
	int16_t remote_x = 0, remote_y = 0;
	int16_t x_min, x_max, y_min, y_max;
	Point32_t	Remote_Point;
	Cell_t   tmpPnt, pnt16ArTmp[3];
	Remote_Point.X = 0;
	Remote_Point.Y = 0;
	//tmpPnt.X = countToCell(Remote_Point.X);
	//tmpPnt.Y = countToCell(Remote_Point.Y);
	path_get_range(&x_min, &x_max, &y_min, &y_max);
	tmpPnt.X = x_max + 1;
	tmpPnt.Y = y_max + 1;
	ROS_INFO("tmpPnt.X = %d, tmpPnt.Y = %d\n",tmpPnt.X, tmpPnt.Y);
	pnt16ArTmp[0] = tmpPnt;

	path_escape_set_trapped_cell(pnt16ArTmp, 1);

	if ( g_trapped_cell[0].X != remote_x || g_trapped_cell[0].Y != remote_y ){
		for ( i = 0; i < g_trapped_cell_size; ++i ) {
			ROS_INFO("%s %d Check %d trapped reference cell: x: %d, y: %d\n", __FUNCTION__, __LINE__,
			         i, g_trapped_cell[i].X, g_trapped_cell[i].Y);
			/*if (is_block_accessible(trappedCell[i].X, trappedCell[i].Y) == 0) {
				Map_Set_Cells(ROBOT_SIZE, trappedCell[i].X, trappedCell[i].Y, CLEANED);
			}*/

			//val = WF_path_find_shortest_path( g_positions[0].x, g_positions[0].y, trappedCell[i].X, trappedCell[i].Y, 0);
			val = WF_path_find_shortest_path( g_cell_history[0].X, g_cell_history[0].Y, g_trapped_cell[i].X, g_trapped_cell[i].Y, 0);
			ROS_INFO("%s %d: val %d\n", __FUNCTION__, __LINE__, val);
			ROS_INFO("SCHAR_MAX = %d\n", SCHAR_MAX);
			if (val < 0 || val == SCHAR_MAX) {
				/* No path to home, which is set when path planning is initialized. */
				val = 0;//not isolated
			} else {
				val = 1;//isolated
				break;
			}
		}
	} else {
		if (is_block_accessible(0, 0) == 1) {
			val = WF_path_find_shortest_path(g_cell_history[0].X, g_cell_history[0].Y, 0, 0, 0);
#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
			ROS_INFO("%s %d: pos (%d, %d)\tval: %d\n", __FUNCTION__, __LINE__, g_cell_history[0].X, g_cell_history[0].Y, val);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = WF_path_find_shortest_path(g_cell_history[0].X, g_cell_history[0].Y, remote_x, remote_y, 0);
				ROS_INFO("%s %d: val %d\n", __FUNCTION__, __LINE__, val);

#if DEBUG_MAP
				debug_map(MAP, remote_x, remote_y);
#endif

				if (val < 0 || val == SCHAR_MAX) {
					val = 0;
				} else {
					val = 1;
				};
			} else {
				val = 1;
			}
		} else {
			val = WF_path_find_shortest_path(g_cell_history[0].X, g_cell_history[0].Y, remote_x, remote_y, 0);
			ROS_INFO("%s %d: val %d\n", __FUNCTION__, __LINE__, val);

#if DEBUG_SM_MAP
			debug_map(SPMAP, 0, 0);
#endif
#if DEBUG_MAP
			debug_map(MAP, remote_x, remote_y);
#endif

			if (val < 0 || val == SCHAR_MAX) {
				/* No path to home, which is set when path planning is initialized. */
				val = 0;
			} else {
				val = 1;
			}
		}
	}

	ROS_INFO("%s %d: val %d\n", __FUNCTION__, __LINE__, val);
	return val != 0;
}

bool wf_is_end()
{
	if (is_reach() || is_time_out() || is_trap())
			return true;
	return false;
}

