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
#include "charger.hpp"
#include "wav.h"
#include "robotbase.h"

#include "motion_manage.h"
//Turn speed
#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed  18
#endif

std::vector<Cell_t> g_cells;
// This list is for storing the position that robot sees the charger stub.
extern std::list<Point32_t> g_home_point;
extern uint8_t g_from_station;
extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;
//Timer
uint32_t g_wall_follow_timer;
uint32_t g_bumper_interval_timer;
bool g_reach_continuous_state;
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

bool wf_check_isolate(void)
{
	float pos_x, pos_y;
	int16_t val;
	uint32_t left_speed, right_speed;
	static int16_t current_x = 0, current_y = 0;

	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	map_set_position(pos_x, pos_y);
	//Map_set_cell(MAP, pos_x, pos_y, CLEANED);

	current_x = map_get_x_cell();
	current_y = map_get_y_cell();

	//ROS_INFO("%s %d: escape thread is up!\n", __FUNCTION__, __LINE__);
	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	map_set_position(pos_x, pos_y);
	//Map_set_cell(MAP, pos_x, pos_y, CLEANED);


	path_update_cell_history();
	//ROS_INFO("%s %d: escape thread checking: pos: (%d, %d) (%d, %d)!\n", __FUNCTION__, __LINE__, current_x, current_y, Map_get_x_cell(), Map_get_y_cell());
	val = WF_path_escape_trapped();
	if (val == 0)
	{
		return 0;//not isolated
	} else
	{
		return 1;//isolated
	}
	current_x = map_get_x_cell();
	current_y = map_get_y_cell();
}

typedef struct WallFollowSpeedRegulator_ {

	int32_t previous = 0;
	int32_t speed_base = 0;

	void adjustSpeed(int32_t &l_speed, int32_t &r_speed, const int wall_distance);
} WallFollowSpeedRegulator;

void WallFollowSpeedRegulator_::adjustSpeed(int32_t &l_speed, int32_t &r_speed, const int wall_distance)
{
	speed_base = 15 + get_wall_accelerate() / 150;
	if (speed_base > 28)speed_base = 28;

	auto proportion = robot::instance()->getLeftWall();

	proportion = proportion * 100 / wall_distance;

	proportion -= 100;

	auto delta = proportion - previous;

	if (wall_distance > 200)
	{//over left
		l_speed = speed_base + proportion / 8 + delta / 3; //12
		r_speed = speed_base - proportion / 9 - delta / 3; //10

		if (speed_base < 26)
		{
			if (r_speed > speed_base + 6)
			{
				r_speed = 34;
				l_speed = 4;
			} else if (l_speed > speed_base + 10)
			{
				r_speed = 5;
				l_speed = 30;
			}
		} else
		{
			if (r_speed > 35)
			{
				r_speed = 35;
				l_speed = 4;
			}
		}
	} else
	{
		l_speed = speed_base + proportion / 10 + delta / 3;//16
		r_speed = speed_base - proportion / 10 - delta / 4; //11

		if (speed_base < 26)
		{
			if (r_speed > speed_base + 4)
			{
				r_speed = 34;
				l_speed = 4;
			}
		} else
		{
			if (r_speed > 32)
			{
				r_speed = 36;
				l_speed = 4;
			}
		}
	}

	previous = proportion;

	if (l_speed > 39)l_speed = 39;
	if (l_speed < 0)l_speed = 0;
	if (r_speed > 35)r_speed = 35;
	if (r_speed < 5)r_speed = 5;
}

void adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	auto speed = 0;
	if (get_left_wheel_step() < 500)
	{
		if (get_wall_accelerate() < 100)
			speed = 10;
		else
			speed = 15;
	} else
		speed = 23;
	l_speed = r_speed =speed;
}

/*------------------------------------------------------------------ Wall Follow Mode--------------------------*/
uint8_t wall_follow(MapWallFollowType follow_type)
{

	uint8_t temp_counter = 0, jam = 0;
	int ret;
	int16_t left_wall_buffer[3] = {0};
	WallFollowSpeedRegulator regulator;
	int32_t l_speed = 0, r_speed = 0;
	uint8_t isolated_flag;
	int16_t isolated_count = 0;

	auto wall_distance = Wall_High_Limit;

	volatile int32_t straight_distance = 200;
	l_speed = 15;

	MotionManage motion;
	if (!motion.initSucceeded())
	{
		set_clean_mode(Clean_Mode_Userinterface);
		reset_stop_event_status();
		return 0;
	}

	ROS_INFO("%s %d: Start wall follow now.", __FUNCTION__, __LINE__);
	g_wall_follow_timer = time(NULL);
	g_bumper_interval_timer = time(NULL);
	move_forward(25, 25);

	while (ros::ok())
	{
		while (ros::ok())
		{
			wall_dynamic_base(30);
			robotbase_obs_adjust_count(300);
			wf_update_position();
			if (get_bumper_status() || (get_front_obs() > get_front_obs_value()) || get_cliff_trig())
				break;
		}

		while (ros::ok())
		{
			if ((time(NULL) - g_wall_follow_timer) > WALL_FOLLOW_TIME)
			{
				ROS_INFO("Wall Follow time longer than 60 minutes");
				ROS_INFO("time now : %d", (int(time(NULL)) - g_wall_follow_timer));
				wf_end_wall_follow();
				return 1;
			}
			/*------------------------------------WF_Map_Update---------------------------------------------------*/
			//wf_update_position();
			wf_check_loop_closed(Gyro_GetAngle());
			if (g_reach_count >= REACH_COUNT_LIMIT)
			{
				if (wf_check_angle())
				{//wf_check_angle succeed,it proves that the robot is not in the narrow space
					g_reach_count = 0;
					stop_brifly();
					if (wf_check_isolate())
					{
						isolated_flag = 1;
						break;
					} else
					{
						isolated_flag = 0;
					}
					wf_end_wall_follow();
					break;
				} else
				{
					g_reach_count = 0;//reset reach_cout because wf_check_angle fail, it proves that the robot is in the narrow space
				}
			}
			//Check if the robot is trapped
			if (g_same_cell_count >= 1000)
			{
				ROS_WARN("Maybe the robot is trapped! Checking!");
				g_same_cell_count = 0;
				stop_brifly();
				if (wf_check_isolate())
				{
					ROS_WARN("Not trapped!");
				} else
				{
					ROS_WARN("Trapped!");
					wf_break_wall_follow();
					set_clean_mode(Clean_Mode_Userinterface);
					return 0;
				}
			}
			robotbase_obs_adjust_count(100);
			/*---------------------------------------------------Bumper Event-----------------------*/
			if (get_bumper_status() & RightBumperTrig)
			{
				ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
				set_wheel_speed(0, 0);
				wf_check_loop_closed(Gyro_GetAngle());
				WFM_move_back(350);
				if (time(NULL) - g_bumper_interval_timer > 15)
				{
					usleep(500000);
					ROS_WARN("wait for adjust the gyro.");
					g_bumper_interval_timer = time(NULL);
				}
				wf_check_loop_closed(Gyro_GetAngle());
				wall_distance = std::min(wall_distance + 300 , Wall_High_Limit);
				wf_turn_right(Turn_Speed - 5, 920);
				wf_check_loop_closed(Gyro_GetAngle());
				move_forward(10, 10);
				reset_wall_accelerate();
				reset_wheel_step();
			}
			if (get_bumper_status() & LeftBumperTrig)
			{
				ROS_WARN("%s %d: left bumper triggered", __FUNCTION__, __LINE__);
				set_wheel_speed(0, 0);
				wf_check_loop_closed(Gyro_GetAngle());
				if (get_bumper_status() & RightBumperTrig)
				{
					ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
					WFM_move_back(100);
					if (time(NULL) - g_bumper_interval_timer > 15)
					{
						usleep(500000);
						ROS_WARN("wait for adjust the gyro.");
						g_bumper_interval_timer = time(NULL);
					}
					wf_check_loop_closed(Gyro_GetAngle());
					wf_turn_right(Turn_Speed - 5, 850);
					wall_distance = std::min(wall_distance + 300 , Wall_High_Limit);
				} else
				{
					wall_distance = std::max(wall_distance - 100 , Wall_High_Limit);
					WFM_move_back(350);
					if (time(NULL) - g_bumper_interval_timer > 15)
					{
						usleep(500000);
						ROS_WARN("wait for adjust the gyro.");
						g_bumper_interval_timer = time(NULL);
					}
					wf_check_loop_closed(Gyro_GetAngle());
					if (is_bumper_jamed())
					{
						reset_stop_event_status();
						set_clean_mode(Clean_Mode_Userinterface);
						wf_break_wall_follow();
						ROS_INFO("%s %d: Check: Bumper 3! break", __FUNCTION__, __LINE__);
						return 0;
					}
					if (jam < 3)
					{
						if (wall_distance < 200)
						{
							if (get_left_obs() > (get_left_obs_value() - 200))
							{
								wall_distance = Wall_High_Limit;
								wf_turn_right(Turn_Speed - 5, 300);
							} else
								wf_turn_right(Turn_Speed - 5, 200);
						} else
							wf_turn_right(Turn_Speed - 5, 300);
					} else
						wf_turn_right(Turn_Speed - 5, 200);
				}
				jam = (get_wall_accelerate() < 2000) ? jam+1 : 0;
				reset_wall_accelerate();
				wf_check_loop_closed(Gyro_GetAngle());

				move_forward(10, 10);

				for (temp_counter = 0; temp_counter < 3; temp_counter++)
				{
					left_wall_buffer[temp_counter] = 0;
				}
				reset_wheel_step();
			}
			/*------------------------------------------------------Short Distance Move-----------------------*/
			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
			if (get_wall_accelerate() < (uint32_t) straight_distance)
			{
				adjustSpeed(l_speed, r_speed);
				move_forward(l_speed, r_speed);
			}
			else
			{
				if (get_front_obs() < get_front_obs_value())
				{
					regulator.adjustSpeed(l_speed, r_speed, wall_distance);
					move_forward(l_speed, r_speed);
				} else
				{
					stop_brifly();
					if (get_wall_accelerate() < 2000)
						jam++;
					wf_turn_right(Turn_Speed - 5, 920);
					stop_brifly();
					move_forward(15, 15);
					reset_wheel_step();
					wall_distance = Wall_High_Limit;
				}
			}
			wf_check_loop_closed(Gyro_GetAngle());
			usleep(10000);
		}
		if (isolated_flag)
		{
			isolated_flag = 0;
			isolated_count++;
			if (isolated_count > 3)
			{
				ROS_WARN("%s %d: Isolate islands more than 3, break", __FUNCTION__, __LINE__);
				wf_end_wall_follow();
				break;
			}
			//Map_Initialize();
			map_reset(MAP);
			g_cells.clear();
			Turn_Right(Turn_Speed, 900);
			continue;
		}
		else
		{
			ROS_WARN("%s %d: Not in isolate island, finish, break", __FUNCTION__, __LINE__);
			break;
		}
	}//the biggest loop end

	stop_brifly();
	move_forward(0, 0);
	return ret;
}

uint8_t wf_end_wall_follow(void)
{
	int16_t i;
	int8_t state;
	stop_brifly();
	robot::instance()->setBaselinkFrameType(
					Map_Position_Map_Angle);//inorder to use the slam angle to finsh the shortest path to home;
	cm_update_position();
	wf_mark_home_point();
	cm_go_home();

	/*****************************************Release Memory************************************/
	g_home_point.clear();
	g_cells.clear();
	std::vector<Cell_t>(g_cells).swap(g_cells);
	debug_map(MAP, 0, 0);
	debug_map(SPMAP, 0, 0);
	set_clean_mode(Clean_Mode_Userinterface);
	return 0;
}

uint8_t wf_break_wall_follow(void)
{
	/*****************************************Release Memory************************************/
	g_home_point.clear();
	g_cells.clear();
	std::vector<Cell_t>(g_cells).swap(g_cells);
	debug_map(MAP, 0, 0);
	debug_map(SPMAP, 0, 0);
	set_clean_mode(Clean_Mode_Userinterface);
	return 0;
}

void wf_update_position(void)
{
	float pos_x, pos_y;
	int16_t x, y;

	x = map_get_x_cell();
	y = map_get_y_cell();

	//Map_move_to(dd * cos(deg2rad(heading, 10)), dd * sin(deg2rad(heading, 10)));
	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	map_set_position(pos_x, pos_y);
}

/**************************************************************
Function:WF_Check_Loop_Closed
Description:
 *1.push a point
 *2.check last cell if cleaned, cleaned->break, uncleaned->continue
 *3.check if in same cell, same->loop until leave this cell, not same-> marked last cell as cleaned
 ***************************************************************/
void wf_check_loop_closed(uint16_t heading)
{
	cm_update_position();
//	g_same_cell_count = is_new_cell() ? g_same_cell_count + 1 : 0;
	is_start_cell() ? g_reach_count++ : g_reach_count = 0;

	auto dx = map_get_relative_x(heading, CELL_SIZE_2, 0);
	auto dy = map_get_relative_y(heading, CELL_SIZE_2, 0);
	if (map_get_cell(MAP, count_to_cell(dx), count_to_cell(dy)) != BLOCKED_BOUNDARY)
		map_set_cell(MAP, dx, dy, BLOCKED_OBS);
}

bool is_start_cell(void)
{
	if (!g_cells.empty())
		if (map_get_cell(MAP, g_cells.back().X, g_cells.back().Y) == CLEANED)
			return true;
	return false;
}

bool is_new_cell()
{
	Cell_t curr = {map_get_x_cell(), map_get_x_cell()};
	if (g_cells.empty() || g_cells.back() != curr){
		g_cells.push_back(curr);
		return 1;
	}
	return 0;
}

void wf_mark_home_point(void)
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

bool wf_check_angle(void)
{
/*
	int32_t x, y;
	int16_t th, former_th;
	int16_t th_diff;
	int16_t DIFF_LIMIT = 1500;//1500 means 150 degrees, it is used by angle check.
	int8_t pass_count = 0;
	int8_t sum = REACH_COUNT_LIMIT;
	bool fail_flag = 0;
	for (int i = 1; i <= REACH_COUNT_LIMIT; i++)
	{
		Cell_t curr =g_cells[g_cells.size() - i];
		fail_flag = 0;

		for (auto it_cell = (g_cells.rbegin() + i); it_cell != g_cells.rend(); ++it_cell)
		{
			if (it_cell->X == x && it_cell->Y == y)
			{
				th_diff = (abs(former_th - th));

				if (th_diff > 1800)
				{
					th_diff = 3600 - th_diff;
				}

				if (th_diff <= DIFF_LIMIT)
				{
					pass_count++;
					ROS_WARN("th_diff = %d <= %d, pass angle check!", th_diff, DIFF_LIMIT);
					break;
				} else
				{
					fail_flag = 1;
					ROS_WARN("th_diff = %d > %d, fail angle check!", th_diff, DIFF_LIMIT);
				}
			}
			if ((it_cell == (g_cells.rend() - 1)) && (fail_flag == 0))
			{
				sum--;
				ROS_WARN("Can't find second same pose in g_cells! sum--");
			}
		}
	}

	if (sum < REACH_COUNT_LIMIT || pass_count < sum)
		return 0;
*/

	return 1;
}
