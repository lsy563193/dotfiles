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

extern std::list<Point32_t> g_home_point;
extern uint8_t g_from_station;
extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;
//Timer
uint32_t g_wall_follow_timer;
uint32_t g_bumper_interval;

static Cell_t g_start_cell;
static auto g_start_flag=0;

bool is_check_isolate(void)
{
//	path_update_cell_history();
//	return WF_path_escape_trapped();
	return false;
}

typedef struct WallFollowSpeedRegulator_ {

	int32_t previous = 0;
	int32_t speed_base = 0;

	void adjustSpeed(int32_t &l_speed, int32_t &r_speed, const int wall_distance);
} WallFollowSpeedRegulator;

void WallFollowSpeedRegulator_::adjustSpeed(int32_t &l_speed, int32_t &r_speed, const int wall_distance)
{
	speed_base = 15 + get_right_wheel_step() / 150;
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
		if (get_right_wheel_step() < 100)
			speed = 10;
		else
			speed = 15;
	} else
		speed = 23;
	l_speed = r_speed =speed;
}

uint32_t bumper_interval()
{
	if (time(NULL) - g_bumper_interval > 15)
	{
		usleep(500000);
		ROS_WARN("wait for adjust the gyro.");
		g_bumper_interval = time(NULL);
	}
	return g_bumper_interval;
}
/*------------------------------------------------------------------ Wall Follow Mode--------------------------*/
uint8_t wall_follow(MapWallFollowType follow_type)
{

	uint8_t temp_counter = 0, jam = 0;
	int ret;
	int16_t left_wall_buffer[3] = {0};
	WallFollowSpeedRegulator regulator;
	int32_t l_speed = 0, r_speed = 0;

	auto wall_distance = Wall_High_Limit;

	const volatile int32_t STRAIGHT_DISTANCE = 200;
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
	g_bumper_interval = time(NULL);
	move_forward(25, 25);

	while (ros::ok())
	{
		while (ros::ok())
		{
			wall_dynamic_base(30);
			robotbase_obs_adjust_count(300);
			if (get_bumper_status() || (get_front_obs() > get_front_obs_value()) || get_cliff_trig()){
				g_start_cell = map_get_curr_cell();
				g_start_flag = true;
				ROS_INFO("g_start_cell(%d,%d):",g_start_cell.X, g_start_cell.Y);
				break;
			}
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
			if (map_get_curr_cell() == g_start_cell && g_start_flag == 10)
			{
				stop_brifly();
				if (is_check_isolate())
					break;
				wf_end_wall_follow();
				break;
			}

			if (map_get_curr_cell() != g_start_cell){
				g_start_flag++;
				if(g_start_flag >= 10)
					g_start_flag=10;
			}
			robotbase_obs_adjust_count(100);
			/*---------------------------------------------------Bumper Event-----------------------*/
			if (get_bumper_status() & RightBumperTrig)
			{
				ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
				WFM_move_back(350);
				g_bumper_interval = bumper_interval();
				wall_distance = std::min(wall_distance + 300 , Wall_High_Limit);
				wf_turn_right(Turn_Speed - 5, 920);
				move_forward(10, 10);
				reset_wheel_step();
			}
			if (get_bumper_status() & LeftBumperTrig)
			{
				ROS_WARN("%s %d: left bumper triggered", __FUNCTION__, __LINE__);
				if (get_bumper_status() & RightBumperTrig)
				{
					ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
					WFM_move_back(100);
					g_bumper_interval = bumper_interval();
					wf_turn_right(Turn_Speed - 5, 850);
					wall_distance = std::min(wall_distance + 300 , Wall_High_Limit);
				} else
				{
					wall_distance = std::max(wall_distance - 100 , Wall_Low_Limit);
					WFM_move_back(350);
					g_bumper_interval = bumper_interval();
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
				jam = (get_right_wheel_step() < 2000) ? jam+1 : 0;
				move_forward(10, 10);
				for (temp_counter = 0; temp_counter < 3; temp_counter++)
					left_wall_buffer[temp_counter] = 0;
				reset_wheel_step();
			}
			/*------------------------------------------------------Short Distance Move-----------------------*/
			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
			if (get_right_wheel_step() < (uint32_t) STRAIGHT_DISTANCE)
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
					if (get_right_wheel_step() < 2000)
						jam++;
					wf_turn_right(Turn_Speed - 5, 920);
					stop_brifly();
					move_forward(15, 15);
					reset_wheel_step();
					wall_distance = Wall_High_Limit;
				}
			}
			usleep(10000);
		}
		if (is_check_isolate())
			continue;

			ROS_WARN("%s %d: Not in isolate island, finish, break", __FUNCTION__, __LINE__);
			break;
	}

	stop_brifly();
	move_forward(0, 0);
	return ret;
}

uint8_t wf_end_wall_follow(void)
{
	stop_brifly();
	robot::instance()->setBaselinkFrameType(
					Map_Position_Map_Angle);//inorder to use the slam angle to finsh the shortest path to home;
	cm_update_map();
	wf_mark_home_point();
	cm_go_home();

	/*****************************************Release Memory************************************/
	g_home_point.clear();
	debug_map(MAP, 0, 0);
	debug_map(SPMAP, 0, 0);
	set_clean_mode(Clean_Mode_Userinterface);
	return 0;
}

uint8_t wf_break_wall_follow(void)
{
	/*****************************************Release Memory************************************/
	g_home_point.clear();
	debug_map(MAP, 0, 0);
	debug_map(SPMAP, 0, 0);
	set_clean_mode(Clean_Mode_Userinterface);
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

