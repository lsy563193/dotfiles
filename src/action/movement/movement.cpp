//
// Created by lsy563193 on 6/28/17.
//

#include <map.h>
#include <gyro.h>
#include <movement.h>
#include <robot.hpp>
#include <core_move.h>
#include "regulator.h"
#include "ros/ros.h"
#include <event_manager.h>
#include <mathematics.h>
#include <motion_manage.h>
#include <wav.h>
#include <spot.h>
#include <robotbase.h>
#include <path_planning.h>
#include <clean_state.h>
#include <pp.h>
#include <bumper.h>
#include <obs.h>
#include <beep.h>
#include <wall_follow.h>
#include <odom.h>
#include "clean_mode.h"


int g_wall_distance = WALL_DISTANCE_LOW_LIMIT;
double bumper_turn_factor = 0.85;
bool line_is_found;
int16_t g_turn_angle;

double g_time_straight = 0.3;
double time_start_straight = 0;

bool g_slip_backward = false;

int16_t bumper_turn_angle()
{
	static int bumper_jam_cnt_ = 0;
	auto get_wheel_step = (mt.is_left()) ? &Wheel::getRightStep : &Wheel::getLeftStep;
	auto get_obs = (mt.is_left()) ? &Obs::get_left : &Obs::get_right;
	auto get_obs_value = (mt.is_left()) ? &Obs::get_left_trig_value : &Obs::get_right_trig_value;
	auto status = ev.bumper_triggered;
	auto diff_side = (mt.is_left()) ? BLOCK_RIGHT : BLOCK_LEFT;
	auto same_side = (mt.is_left()) ? BLOCK_LEFT : BLOCK_RIGHT;

	if (status == BLOCK_ALL)
	{
		g_turn_angle = -600;
		bumper_jam_cnt_ = (wheel.*get_wheel_step)() < 2000 ? ++bumper_jam_cnt_ : 0;
		g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
	} else if (status == diff_side)
	{
		g_turn_angle = -850;
		g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
	} else if (status == same_side)
	{
		g_wall_distance = bumper_turn_factor * g_wall_distance;
		if(g_wall_distance < 330)
			g_wall_distance = WALL_DISTANCE_LOW_LIMIT;
		g_turn_angle =0;
		if (!cs.is_trapped()) {
			g_turn_angle = (bumper_jam_cnt_ >= 3 || (obs.*get_obs)() <= (obs.*get_obs_value)()) ? -180 : -280;
		} else {
			g_turn_angle = (bumper_jam_cnt_ >= 3 || (obs.*get_obs)() <= (obs.*get_obs_value)()) ? -100 : -200;
		}
		//ROS_INFO("%s, %d: g_turn_angle(%d)",__FUNCTION__,__LINE__, g_turn_angle);

		bumper_jam_cnt_ = (wheel.*get_wheel_step)() < 2000 ? ++bumper_jam_cnt_ : 0;
	}
	//ROS_INFO("%s %d: g_wall_distance in bumper_turn_angular: %d", __FUNCTION__, __LINE__, g_wall_distance);
	wheel.resetStep();
	if(mt.is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

int16_t cliff_turn_angle()
{
	g_turn_angle = -750;
	if(mt.is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

int16_t tilt_turn_angle()
{
	auto tmp_status = gyro.getTiltCheckingStatus();
	if (mt.is_left())
	{
		if (tmp_status | TILT_LEFT)
			g_turn_angle = -600;
		if (tmp_status | TILT_FRONT)
			g_turn_angle = -850;
		if (tmp_status | TILT_RIGHT)
			g_turn_angle = -1100;
	}
	else
	{
		if (tmp_status | TILT_RIGHT)
			g_turn_angle = 600;
		if (tmp_status | TILT_FRONT)
			g_turn_angle = 850;
		if (tmp_status | TILT_LEFT)
			g_turn_angle = 1100;
	}
	return g_turn_angle;
}

int16_t obs_turn_angle()
{
	auto diff_side = (mt.is_left()) ? BLOCK_RIGHT : BLOCK_LEFT;
	auto same_side = (mt.is_left()) ? BLOCK_LEFT : BLOCK_RIGHT;
	if(ev.obs_triggered == BLOCK_FRONT)
		g_turn_angle = -850;
	else if(ev.obs_triggered == diff_side)
		g_turn_angle = -920;
	else if(ev.obs_triggered == same_side)
		g_turn_angle = -300;

	if(mt.is_right())
		g_turn_angle = -g_turn_angle;
//	ROS_WARN("g_turn_angle(%d)",g_turn_angle);
	return g_turn_angle;
}

int16_t rcon_turn_angle()
{
	enum {left,fl2,fl,fr,fr2,right};
	int16_t left_angle[] =   {-300,-600,-850,-850,-950,-1100};
	int16_t right_angle[] =  {1100, 950, 850, 850, 600, 300};
	if(mt.is_left())
		g_turn_angle = left_angle[ev.rcon_triggered-1];
	else if(mt.is_right())
		g_turn_angle = right_angle[ev.rcon_triggered-1];

	return g_turn_angle;
}

static int double_scale_10(double line_angle)
{
	int angle;
	if (line_angle > 0)
	{
		angle = int((180 - line_angle) * 10);
	} else
	{
		angle = int(fabs(line_angle) * 10);
	}
	return angle;
}

static bool _lidar_turn_angle(int16_t& turn_angle, int lidar_min, int lidar_max, int angle_min,int angle_max,double dis_limit=0.217)
{
//	ROS_INFO("%s,%d,bumper (\033[32m%d\033[0m)!",__FUNCTION__,__LINE__,bumper.get_status());
	double line_angle;
	double distance;
//	auto RESET_WALL_DIS = 100;
	line_is_found = lidar.lidarGetFitLine(lidar_min, lidar_max, -1.0, dis_limit, &line_angle, &distance);
//	RESET_WALL_DIS = int(distance * 1000);

//	ROS_INFO("line_distance = %lf", distance);
//	ROS_INFO("line_angle_raw = %lf", line_angle);
	auto angle = double_scale_10(line_angle);

	if (mt.is_right())
		angle  = 1800-angle;

//	ROS_INFO("line_angle = %d", angle);
	if (line_is_found && angle >= angle_min && angle < angle_max)
	{
//		ROS_ERROR("distance: %f",(distance*100.0-16.7));
		line_angle=fabs(line_angle);
		if(line_angle < 90)
			robot_to_wall_distance=g_back_distance*100*sin(line_angle*3.1415/180.0);
		else
			robot_to_wall_distance=g_back_distance*100*sin((180-line_angle)*3.1415/180.0);
//		ROS_ERROR("left_x= %f  left_angle= %lf",x,line_angle);
		turn_angle = mt.is_right() ? angle : -angle;
//		ROS_INFO("lidar generate turn angle(%d)!",turn_angle);
		return true;
	}
	return false;
}

bool lidar_turn_angle(int16_t& turn_angle)
{
//	ROS_INFO("%s,%d: mt.is_fw",__FUNCTION__, __LINE__);
	wheel.stop();

	if (ev.obs_triggered != 0)
	{
//		ROS_INFO("%s %d: \033[32mfront obs trigger.\033[0m", __FUNCTION__, __LINE__);
		return _lidar_turn_angle(turn_angle, 90, 270, 450, 1800, 0.25);
	}
	else if(ev.bumper_triggered != 0)
	{
		int angle_min, angle_max;
		if (mt.is_left() ^ (ev.bumper_triggered == BLOCK_LEFT))
		{
			angle_min = 600;
			angle_max = 1800;
		}
		else
		{
			angle_min = 180;
			angle_max = 900;
		}

		if (ev.bumper_triggered == BLOCK_ALL) {
//			ROS_INFO("%s %d: AllBumper trigger.", __FUNCTION__, __LINE__);
			return _lidar_turn_angle(turn_angle, 90, 270, 900, 1800);
		}
		else if (ev.bumper_triggered == BLOCK_RIGHT) {
//			ROS_INFO("%s %d: RightBumper trigger.", __FUNCTION__, __LINE__);
			return _lidar_turn_angle(turn_angle, 90, 180, angle_min, angle_max);
		}
		else if (ev.bumper_triggered == BLOCK_LEFT) {
//			ROS_INFO("%s %d: LeftBumper trigger.", __FUNCTION__, __LINE__);
			return _lidar_turn_angle(turn_angle, 180, 270, angle_min, angle_max);
		}
	}
	return false;
}

Point32_t Movement::s_target_p = {0,0};
Point32_t Movement::s_origin_p = {0,0};
int16_t Movement::s_target_angle = 0;
float Movement::s_pos_x = 0;
float Movement::s_pos_y = 0;
Point32_t Movement::s_curr_p = {0,0};


bool Movement::isExit()
{
	if (ev.fatal_quit || ev.key_clean_pressed || ev.charge_detect)
	{
		ROS_WARN("%s %d: fatal_quit(%d), key_clean_pressed(%d), charge_detect(%d)",
				 __FUNCTION__, __LINE__, ev.fatal_quit, ev.key_clean_pressed, ev.charge_detect);
		return true;
	}
	return false;
}

bool Movement::isStop()
{
	if (ev.battery_home || ev.remote_spot || (!cs.is_going_home() && ev.remote_home) || cm_should_self_check())
	{
		ROS_WARN("%s %d: battery_home(%d), remote_spot(%d), remote_home(%d), should_self_check(%d)",
				 __FUNCTION__, __LINE__, ev.battery_home, ev.remote_spot, ev.remote_home, cm_should_self_check());
		return true;
	}
	return false;
}

