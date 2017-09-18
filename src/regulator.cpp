//
// Created by lsy563193 on 6/28/17.
//

#include <map.h>
#include <gyro.h>
#include <movement.h>
#include <wall_follow_slam.h>
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

extern uint16_t g_old_dir;
extern uint16_t g_new_dir;
extern Cell_t g_cell_history[];
int g_wall_distance = WALL_DISTANCE_LOW_LIMIT;
double bumper_turn_factor=0.85;
bool line_is_found;
double robot_to_wall_distance=0.8;
int16_t g_turn_angle;
float g_back_distance = 0.01;
// Back distance for go to charger regulator
bool g_go_to_charger_back_30cm = false;
bool g_go_to_charger_back_10cm = false;
bool g_go_to_charger_back_0cm = false;


extern bool g_exploration_home;

static int16_t bumper_turn_angle()
{
	static int bumper_jam_cnt_ = 0;
	auto get_wheel_step = (mt_is_left()) ? get_right_wheel_step : get_left_wheel_step;
	auto get_obs = (mt_is_left()) ? get_left_obs : get_right_obs;
	auto get_obs_value = (mt_is_left()) ? get_left_obs_value : get_right_obs_value;
	auto status = g_bumper_triggered;
	auto diff_side = (mt_is_left()) ? RightBumperTrig : LeftBumperTrig;
	auto same_side = (mt_is_left()) ? LeftBumperTrig : RightBumperTrig;

	if (status == AllBumperTrig)
	{
		g_turn_angle = -600;
		g_straight_distance = 150; //150;
		bumper_jam_cnt_ = get_wheel_step() < 2000 ? ++bumper_jam_cnt_ : 0;
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
		ROS_WARN("%s, %d: g_turn_angle(%d)",__FUNCTION__,__LINE__, g_turn_angle);
		if (g_trapped_mode != 1) {
			g_turn_angle = (bumper_jam_cnt_ >= 3 || (get_obs() <= get_obs_value() - 200)) ? -180 : -280;
		} else {
			g_turn_angle = (bumper_jam_cnt_ >= 3 || (get_obs() <= get_obs_value() - 200)) ? -100 : -200;
		}
		ROS_WARN("%s, %d: g_turn_angle(%d)",__FUNCTION__,__LINE__, g_turn_angle);

		g_straight_distance = 250; //250;
		bumper_jam_cnt_ = get_wheel_step() < 2000 ? ++bumper_jam_cnt_ : 0;
	}
	ROS_INFO("g_wall_distance in bumper_turn_angular: %d",g_wall_distance);
	g_straight_distance = 200;
	reset_wheel_step();
	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

static int16_t cliff_turn_angle()
{
	g_turn_angle = -750;
	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

static int16_t tilt_turn_angle()
{
	auto tmp_status = get_tilt_status();
	if (mt_is_left())
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

static int16_t obs_turn_angle()
{
	auto diff_side = (mt_is_left()) ? RightBumperTrig : LeftBumperTrig;
	auto same_side = (mt_is_left()) ? LeftBumperTrig : RightBumperTrig;
	if(g_obs_triggered == Status_Front_OBS)
		g_turn_angle = -850;
	else if(g_obs_triggered == diff_side)
		g_turn_angle = -920;
	else if(g_obs_triggered == same_side)
		g_turn_angle = -300;

	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
//	ROS_WARN("g_turn_angle(%d)",g_turn_angle);
	return g_turn_angle;
}

static int16_t rcon_turn_angle()
{
	enum {left,fl2,fl,fr,fr2,right};
	int16_t left_angle[] =   {-300,-600,-850,-850,-950,-1100};
	int16_t right_angle[] =  {1100, 950, 850, 850, 600, 300};
	if(mt_is_left())
		g_turn_angle = left_angle[g_rcon_triggered-1];
	else if(mt_is_right())
		g_turn_angle = right_angle[g_rcon_triggered-1];

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

static int16_t _laser_turn_angle(int laser_min, int laser_max, int angle_min,int angle_max,double dis_limit=0.217)
{
	ROS_INFO("bumper (%d)!", get_bumper_status());
	double line_angle;
	double distance;
//	auto RESET_WALL_DIS = 100;
	line_is_found = MotionManage::s_laser->getLaserDistance(laser_min, laser_max, -1.0, dis_limit, &line_angle, &distance);
//	RESET_WALL_DIS = int(distance * 1000);

	ROS_INFO("line_distance = %lf", distance);
	ROS_INFO("line_angle_raw = %lf", line_angle);
	auto angle = double_scale_10(line_angle);

	if (mt_is_right())
		angle  = 1800-angle;

	ROS_INFO("line_angle = %d", angle);
	if (line_is_found && angle >= angle_min && angle < angle_max)
	{
//		ROS_ERROR("distance: %f",(distance*100.0-16.7));
		line_angle=fabs(line_angle);
		if(line_angle < 90)
			robot_to_wall_distance=g_back_distance*100*sin(line_angle*3.1415/180.0);
		else
			robot_to_wall_distance=g_back_distance*100*sin((180-line_angle)*3.1415/180.0);
//		ROS_ERROR("left_x= %f  left_angle= %lf",x,line_angle);
		g_turn_angle = mt_is_right() ? angle : -angle;
		ROS_INFO("laser generate turn angle(%d)!",g_turn_angle);
	}
	return g_turn_angle;
}

static int16_t laser_turn_angle()
{
	stop_brifly();

	if (g_obs_triggered != 0)
	{
		ROS_INFO("%s %d: \033[32mfront obs trigger.\033[0m", __FUNCTION__, __LINE__);
		return _laser_turn_angle(90, 270, 450, 1800, 0.25);
	}
		else if(g_bumper_triggered != 0)
	{
		int angle_min, angle_max;
		if (mt_is_left() ^ g_bumper_triggered == LeftBumperTrig)
		{
			angle_min = 600;
			angle_max = 1800;
		}
		else
		{
			angle_min = 180;
			angle_max = 900;
		}

		if (g_bumper_triggered == AllBumperTrig) {
			ROS_INFO("%s %d: AllBumper trigger.", __FUNCTION__, __LINE__);
			return _laser_turn_angle(90, 270, 900, 1800);
		}
		else if (g_bumper_triggered == RightBumperTrig) {
			ROS_INFO("%s %d: RightBumper trigger.", __FUNCTION__, __LINE__);
			return _laser_turn_angle(90, 180, angle_min, angle_max);
		}
		else if (g_bumper_triggered == LeftBumperTrig) {
			ROS_INFO("%s %d: LeftBumper trigger.", __FUNCTION__, __LINE__);
			return _laser_turn_angle(180, 270, angle_min, angle_max);
		}
	}

	return g_turn_angle;
}
static int16_t _get_obs_value()
{
	if(get_front_obs() > get_front_obs_value())
		return Status_Front_OBS;
	if(get_left_obs() > get_left_obs_value())
		return Status_Left_OBS;
	if(get_right_obs() > get_right_obs_value())
		return Status_Right_OBS;
	return 0;
}

Point32_t RegulatorBase::s_target = {0,0};
Point32_t RegulatorBase::s_origin = {0,0};
int16_t RegulatorBase::s_origin_angle = 0;
int16_t RegulatorBase::s_target_angle = 0;
float RegulatorBase::s_pos_x = 0;
float RegulatorBase::s_pos_y = 0;
Point32_t RegulatorBase::s_curr_p = {0,0};

bool RegulatorBase::isExit(){
	return g_fatal_quit_event || g_key_clean_pressed ;
}

bool RegulatorBase::_isStop()
{
	bool ret = false;
//	ROS_INFO("reg_base _isStop");

/*for exploration mark the map and detect the rcon signal*/
	if (get_clean_mode() == Clean_Mode_Exploration) {
		if (get_rcon_status()) {
			g_exploration_home = true;
			ret = true;
		}
	}
	ret |=  g_battery_home || g_remote_spot || (!g_go_home && g_remote_home) || cm_should_self_check() ; 
	return ret;
}


BackRegulator::BackRegulator() : speed_(8), counter_(0)
{
//	ROS_INFO("%s, %d: ", __FUNCTION__, __LINE__);
}

void BackRegulator::setTarget()
{
	s_pos_x = robot::instance()->getOdomPositionX();
	s_pos_y = robot::instance()->getOdomPositionY();
	if (g_robot_slip){
		g_back_distance = 0.30;
		g_slip_triggered = true;
		g_robot_slip = false;
	}
	else if (g_tilt_triggered)
		g_back_distance = 0.05;
	else if (g_go_to_charger_back_10cm)
		g_back_distance = 0.10;
	else if (g_go_to_charger_back_30cm)
		g_back_distance = 0.30;
	else if (g_go_to_charger_back_0cm)
		g_back_distance = 0.0;
	else
		g_back_distance = 0.01;
	ROS_INFO("%s %d: Set back distance: %f.", __FUNCTION__, __LINE__, g_back_distance);
}

bool BackRegulator::isReach()
{
	auto distance = sqrtf(powf(s_pos_x - robot::instance()->getOdomPositionX(), 2) +
				powf(s_pos_y - robot::instance()->getOdomPositionY(), 2));
	ROS_DEBUG("%s, %d: BackRegulator distance %f", __FUNCTION__, __LINE__, distance);
	if(fabsf(distance) > g_back_distance)
	{
		/*---------slip detect------*/
		if(g_slip_cnt >=1){ //this should be judge first
			g_slip_cnt = 0;
			g_robot_stuck = true;
			ROS_WARN("%s,%d,\033[1mrobot stuck detect!!\033[0m",__FUNCTION__,__LINE__);
			beep_for_command(false);
			g_robot_slip = false;
			g_slip_triggered = false;
			return true;
		}
		else if(g_robot_slip){
			ROS_WARN("%s,%d,\033[1mrobot slip again!!\033[0m",__FUNCTION__,__LINE__);
			setTarget();
			g_robot_slip = false;
			g_slip_cnt ++;
			return false;
		}
		else if(g_slip_triggered){
			g_slip_triggered = false;
			ROS_WARN("%s,%d,\033[1mrobot slip backward reach!! distance(%f),back_distance(%f)\033[0m",__FUNCTION__,__LINE__,distance,g_back_distance);
			g_slip_cnt = 0;
			return true;
		}
		/*-------slip detect end-------*/

		ROS_INFO("\033[32m%s\033[0m, %d: \033[33mBackRegulator\033[0m ", __FUNCTION__, __LINE__);
		g_bumper_cnt =get_bumper_status() == 0 ? 0 : g_bumper_cnt+1 ;
		g_cliff_cnt = get_cliff_status() == 0 ? 0 : g_cliff_cnt+1 ;

		if (g_bumper_cnt == 0 && g_cliff_cnt == 0 && !get_tilt_status())
		{
			if (mt_is_go_to_charger())
			{
				g_go_to_charger_back_30cm = false;
				g_go_to_charger_back_10cm = false;
				g_go_to_charger_back_0cm = false;
			}
			return true;
		}
		if (g_cliff_cnt >= 2)
		{
			g_cliff_jam = true;
			return false;
		}
		else if (g_bumper_cnt >= 2)
		{
			g_bumper_jam = true;
			return false;
		}
		else
			setTarget();
	}
	return false;
}

bool BackRegulator::isSwitch()
{
//	ROS_INFO("BackRegulator::isSwitch");
	if(mt_is_follow_wall() || mt_is_go_to_charger())
		return isReach();
	if(mt_is_linear())
		return false;

	return false;
}

bool BackRegulator::_isStop()
{
	MotionManage::s_laser->laserMarker(false);
	bool ret = false;
	return ret;
}

void BackRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
//	ROS_INFO("BackRegulator::adjustSpeed");
	set_dir_backward();
	if (get_clean_mode() != Clean_Mode_WallFollow)
	{
		speed_ += ++counter_;
		speed_ = (speed_ > BACK_MAX_SPEED) ? BACK_MAX_SPEED : speed_;
	}
	reset_wheel_step();
	l_speed = r_speed = speed_;
}


TurnRegulator::TurnRegulator(int16_t angle) : speed_(ROTATE_LOW_SPEED)
{
	accurate_ = ROTATE_TOP_SPEED > 30 ? 30 : 15;
	s_target_angle = angle;
	ROS_WARN("%s %d: Init, s_target_angle: %d", __FUNCTION__, __LINE__, s_target_angle);
}

bool TurnRegulator::isReach()
{
	if (abs(s_target_angle - gyro_get_angle()) < accurate_){
		ROS_INFO("%s, %d: TurnRegulator target angle: %d, current angle: %d.", __FUNCTION__, __LINE__, s_target_angle, gyro_get_angle());

		/*********************************************For wall follow**********************************************/
		if(line_is_found)
		{
			g_wall_distance = (mt_is_left()) ? robot::instance()->getLeftWall() : robot::instance()->getRightWall();
/*			if(g_wall_distance < 10)	//set g_wall_distance in U round
			{
				g_wall_distance=last_g_wall_distance;
				ROS_ERROR("g_wall_distance: %d",g_wall_distance);
				return true;
			}*/

			ROS_INFO("g_wall_distance: %d",g_wall_distance);
			if(g_wall_distance < 150)  //150 is the experience value by testing in the closest position to black wall
			{
				g_wall_distance += (150 - g_wall_distance) / 4 * 3;
				if(g_wall_distance < WALL_DISTANCE_BLACK_MIN)
					g_wall_distance = WALL_DISTANCE_BLACK_MIN;
				ROS_INFO("g_wall_distance_distance_adjust: %d",g_wall_distance);
			} else if(g_wall_distance > 250 && g_wall_distance < 320) // boardline invagination
			{
				g_wall_distance += (400 - g_wall_distance) / 5 * 4;
				ROS_INFO("boradline invagination，g_wall_distance: %d",g_wall_distance);
			} else if(g_wall_distance > 320 && g_wall_distance < 620)  //620 is the experience value by testing in the closest position to white wall
			{
				g_wall_distance += (620 - g_wall_distance) / 4 * 3;
				if(g_wall_distance < WALL_DISTANCE_WHITE_MIN)
					g_wall_distance = WALL_DISTANCE_WHITE_MIN;
				ROS_INFO("g_wall_distance_distance_adjust: %d",g_wall_distance);
			}else if(g_wall_distance > WALL_DISTANCE_WHITE_MAX)
			{
				g_wall_distance = WALL_DISTANCE_WHITE_MAX + 0.3 * (g_wall_distance - WALL_DISTANCE_WHITE_MAX);
				ROS_INFO("g_wall_distance_distance_adjust: %d",g_wall_distance);
			}
//			last_g_wall_distance=g_wall_distance;
			line_is_found = false;
		}
		else
			ROS_INFO("turn regulator,%s,%d,\033[32mline is not found\033[0m",__FUNCTION__,__LINE__);
		return true;
	}
		/**********************************************END**********************************************************/

	return false;
}

bool TurnRegulator::isSwitch()
{
//	ROS_INFO("TurnRegulator::isSwitch");

	if(isReach() ||(! g_bumper_triggered  && get_bumper_status()) || (! g_cliff_triggered && get_cliff_status()) || (!g_tilt_triggered && get_tilt_status()) || g_robot_slip)
	{
		ROS_INFO("%s, %d: TurnRegulator should switch.", __FUNCTION__, __LINE__);
		g_bumper_triggered = get_bumper_status();
		g_cliff_triggered = get_cliff_status();
		g_tilt_triggered = get_tilt_status();
		reset_sp_turn_count();
		reset_wheel_step();
		reset_rcon_status();
		return true;
	}
	return false;
}

bool TurnRegulator::_isStop()
{
	bool ret = false;
	MotionManage::s_laser->laserMarker(false);
	return ret;
}

void TurnRegulator::setTarget()
{
	if(LASER_FOLLOW_WALL && g_trapped_mode != 1)
	{
		set_wheel_speed(0, 0);
		delay_sec(0.25);
/*		do
		{
			set_wheel_speed(0, 0);
			usleep(300000);
		} while (robot::instance()->isMoving());*/
		g_turn_angle = laser_turn_angle();
	}
	s_target_angle = ranged_angle(gyro_get_angle() + g_turn_angle);
	// Reset the speed.
	speed_ = ROTATE_LOW_SPEED;
	ROS_INFO("%s %d: TurnRegulator, s_target_angle: %d", __FUNCTION__, __LINE__, s_target_angle);
}

void TurnRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	auto diff = ranged_angle(s_target_angle - gyro_get_angle());
//	ROS_INFO("TurnRegulator::adjustSpeed diff(%d),(%d,%d)", diff,s_target_angle, gyro_get_angle());
	ROS_DEBUG("%s %d: TurnRegulator diff: %d, s_target_angle: %d, current angle: %d.", __FUNCTION__, __LINE__, diff, s_target_angle, gyro_get_angle());
	(diff >= 0) ? set_dir_left() : set_dir_right();

//	ROS_INFO("TurnRegulator::adjustSpeed");

	if (std::abs(diff) > 200){
		speed_ += 1;
		speed_ = std::min(speed_, ROTATE_TOP_SPEED);
	}
	else if (std::abs(diff) > 50){
		speed_ -= 2;
		uint8_t low_speed = ROTATE_LOW_SPEED + 5;
		speed_ = std::max(speed_, low_speed);
		ROS_DEBUG("%s %d: 50 - 200, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}
	else{
		speed_ -= 2;
		speed_ = std::max(speed_, ROTATE_LOW_SPEED);
		ROS_DEBUG("%s %d: 0 - 50, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}
	l_speed = r_speed = speed_;

}

bool TurnSpeedRegulator::adjustSpeed(int16_t diff, uint8_t& speed)
{
	if ((diff >= 0) && (diff <= 1800))
		set_dir_left();
	else if ((diff <= 0) && (diff >= (-1800)))
		set_dir_right();

	tick_++;
	if (tick_ > 2)
	{
		tick_ = 0;
		if (std::abs(diff) > 350){
			speed_ = std::min(++speed_, speed_max_);
		}
		else{
			--speed_;
			speed_ = std::max(--speed_, speed_min_);
		}
	}
	speed = speed_;
	return true;
}


LinearRegulator::LinearRegulator(Point32_t target, const PPTargetType& path):
				integrated_(0),base_speed_(LINEAR_MIN_SPEED),integration_cycle_(0),tick_(0),turn_speed_(4)
{
//	g_is_should_follow_wall = false;
	s_target = target;
	path_ = path;
	//ROS_INFO("%s %d: current cell(%d,%d), target cell(%d,%d) ", __FUNCTION__, __LINE__, map_get_x_cell(),map_get_y_cell(), count_to_cell(s_target.X), count_to_cell(s_target.Y));
}

bool LinearRegulator::isReach()
{
	auto curr_p = (IS_X_AXIS(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target = (IS_X_AXIS(g_new_dir)) ? s_target.X : s_target.Y;

#if LINEAR_MOVE_WITH_PATH
//	ROS_WARN("%s, %d: LinearRegulator2:g_new_dir(%d),is_x_axis(%d),is_pos(%d),curr_p(%d),target(%d)", __FUNCTION__, __LINE__,g_new_dir,IS_X_AXIS(g_new_dir),IS_POS_AXIS(g_new_dir),curr_p, target);
	if (path_.cells.empty() || path_.cells.size() == 1)
	{
		//ROS_INFO("\033[32m" "cells.empty(%s),cells.size(%d)""\033[0m",path_.cells.empty()?"true":"false", path_.cells.size());
		if (std::abs(s_curr_p.X - s_target.X) < CELL_COUNT_MUL_1_2 && std::abs(s_curr_p.Y - s_target.Y) < CELL_COUNT_MUL_1_2)
		{
			ROS_INFO("\033[1m""%s, %d: LinearRegulator, reach the target cell (%d,%d)!!""\033[0m", __FUNCTION__, __LINE__ ,path_.target.X,path_.target.Y);
			return true;
		}
	}
	else
	{
		if( (IS_POS_AXIS(g_new_dir) && (curr_p > target - 1.5 * CELL_COUNT_MUL)) ||
			(! IS_POS_AXIS(g_new_dir) && (curr_p < target + 1.5 * CELL_COUNT_MUL)))
		{
			path_.cells.pop_front();
			g_next_cell = path_.cells.front();
			s_target = map_cell_to_point(g_next_cell);
			if (std::abs(s_curr_p.X - s_target.X) < std::abs(s_curr_p.Y - s_target.Y))
				g_new_dir = s_curr_p.Y > s_target.Y ? NEG_Y : POS_Y;
			else
				g_new_dir = s_curr_p.X > s_target.X ? NEG_X : POS_X;
			//ROS_WARN("%s %d: Curr(%d, %d), switch next cell(%d, %d), new dir(%d).", __FUNCTION__, __LINE__, map_get_x_cell(), map_get_y_cell(), g_next_cell.X, g_next_cell.Y, g_new_dir);
			//MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell, path_.cells);
		}
	}
#else
	if (std::abs(s_curr_p.X - s_target.X) < 150 && std::abs(s_curr_p.Y - s_target.Y) < 150)
	{
		ROS_INFO("\033[32m""%s, %d: LinearRegulator, reach the target cell!!""\033[0m", __FUNCTION__, __LINE__);
		return true;
	}
#endif
	curr_p = (IS_X_AXIS(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	target = (IS_X_AXIS(g_new_dir)) ? s_target.X : s_target.Y;
//	ROS_WARN("%s, %d: LinearRegulator2:g_new_dir(%d),is_x_axis(%d),is_pos(%d),curr_p(%d),target(%d)", __FUNCTION__, __LINE__,g_new_dir,IS_X_AXIS(g_new_dir),IS_POS_AXIS(g_new_dir),curr_p, target);
	if( (IS_POS_AXIS(g_new_dir) && (curr_p > target + CELL_COUNT_MUL/4)) ||
			(! IS_POS_AXIS(g_new_dir) && (curr_p < target - CELL_COUNT_MUL/4))
		){
		ROS_INFO("%s, %d: LinearRegulator2: g_new_dir(\033[32m%d\033[0m),is_x_axis(\033[32m%d\033[0m),is_pos(\033[32m%d\033[0m),curr_p(\033[32m%d\033[0m),target(\033[32m%d\033[0m)", __FUNCTION__, __LINE__,g_new_dir,IS_X_AXIS(g_new_dir),IS_POS_AXIS(g_new_dir),curr_p, target);
		return true;
	}

	return false;
}

bool LinearRegulator::isSwitch()
{

	if ((! g_bumper_triggered && get_bumper_status())
		|| (! g_cliff_triggered && get_cliff_status())
		|| (! g_tilt_triggered && get_tilt_status()) || g_robot_slip)
	{
//		g_is_should_follow_wall = true;
		ROS_INFO("%s, %d,g_bumper_triggered(\033[32m%d\033[0m) g_cliff_triggered(\033[32m%d\033[0m) g_tilt_triggered(\033[32m%d\033[0m) g_robot_slip(\033[32m%d\033[0m).", __FUNCTION__, __LINE__,g_bumper_triggered,g_cliff_triggered,g_tilt_triggered,g_robot_slip);
		if(get_bumper_status())
			g_bumper_triggered = get_bumper_status();
		if(get_cliff_status())
			g_cliff_triggered = get_cliff_status();
		if(get_tilt_status())
			g_tilt_triggered = get_tilt_status();

		SpotType spt = SpotMovement::instance() -> getSpotType();
		if(spt == CLEAN_SPOT || spt == NORMAL_SPOT)
			SpotMovement::instance()->setOBSTrigger();

//		mt_set(CM_FOLLOW_LEFT_WALL);
//		if(g_bumper_triggered)
//			g_turn_angle = bumper_turn_angle();
//		else
//			g_turn_angle = cliff_turn_angle();
//		mt_set(CM_LINEARMOVE);

		return true;
	}

	return false;
}

bool LinearRegulator::_isStop()
{
	auto rcon_tmp = get_rcon_trig();
	bool obs_tmp;
	if(get_clean_mode()==Clean_Mode_WallFollow)
		 obs_tmp = LASER_MARKER ?  MotionManage::s_laser->laserMarker(true,0.14,0.20): _get_obs_value();
	else
		 obs_tmp = LASER_MARKER ?  MotionManage::s_laser->laserMarker(true): _get_obs_value();

	//if (obs_tmp == Status_Front_OBS || rcon_tmp)
	if (obs_tmp != 0 || rcon_tmp )
	{
		//if(obs_tmp == Status_Front_OBS)
		if(obs_tmp != 0)
			g_obs_triggered = obs_tmp;

		if(rcon_tmp){
//			if(g_cell_history[0] != map_get_curr_cell()){
				g_rcon_triggered = rcon_tmp;
				if (g_go_home)
					g_rcon_during_go_home = true;
				else
					path_set_home(map_get_curr_cell());
//			}
//			else{
//				ROS_ERROR("%s, %d: g_rcon_triggered but curr(%d,%d),g_h0=g_h1(%d,%d).", __FUNCTION__, __LINE__,map_get_curr_cell().X,map_get_curr_cell().Y,g_cell_history[0].X, g_cell_history[0].Y);
//				stop_brifly();
//				sleep(5);
//				return false;
//			}
		}
		if(g_obs_triggered)
			g_turn_angle = obs_turn_angle();
		else
			g_turn_angle = rcon_turn_angle();
		ROS_INFO("%s, %d: LinearRegulator, g_obs_triggered(%d) g_rcon_triggered(%d).", __FUNCTION__, __LINE__,g_obs_triggered, g_rcon_triggered);
		SpotType spt = SpotMovement::instance()->getSpotType();
		if (spt == CLEAN_SPOT || spt == NORMAL_SPOT)
			SpotMovement::instance()->setOBSTrigger();
		return true;
	}

	if (is_map_front_block(2))
	{
		ROS_INFO("%s, %d: LinearRegulator, Blocked boundary.", __FUNCTION__, __LINE__);
		return true;
	}

#if !LINEAR_MOVE_WITH_PATH
	auto diff = ranged_angle(
					course_to_dest(s_curr_p.X, s_curr_p.Y, s_target.X, s_target.Y) - gyro_get_angle());
	if ( std::abs(diff) > 300)
	{
		ROS_WARN("%s %d: LinearRegulator, warning: angle is too big, angle: %d", __FUNCTION__, __LINE__, diff);
		return true;
	}
#endif

//if ((IS_X_AXIS(g_new_dir) && std::abs(s_target.Y - s_curr_p.Y) > CELL_COUNT_MUL_1_2/5*4)
//	||  (IS_Y_AXIS(g_new_dir) && std::abs(s_target.X - s_curr_p.X) > CELL_COUNT_MUL_1_2/5*4))
//	{
//		ROS_INFO("%s %d: LinearRegulator, warning: angle is too big, angle: %d", __FUNCTION__, __LINE__, diff);
//		ROS_ERROR("%s %d: d_angle %d, d_Y(%d)", __FUNCTION__, __LINE__, diff, s_target.Y - s_curr_p.Y);
//		return true;
//	}
	return false;
}

void LinearRegulator::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	ROS_INFO("BackRegulator::adjustSpeed");
	set_dir_forward();

	auto angle_diff = ranged_angle(
					course_to_dest(s_curr_p.X, s_curr_p.Y, s_target.X, s_target.Y) - gyro_get_angle());

	auto dis_diff = IS_X_AXIS(g_new_dir) ? s_curr_p.Y - s_target.Y : s_curr_p.X - s_target.X;
	dis_diff = IS_POS_AXIS(g_new_dir) ^ IS_X_AXIS(g_new_dir) ? dis_diff :  -dis_diff;

	if (integration_cycle_++ > 10)
	{
		integration_cycle_ = 0;
		integrated_ += angle_diff;
		check_limit(integrated_, -150, 150);
	}

	auto distance = two_points_distance(s_curr_p.X, s_curr_p.Y, s_target.X, s_target.Y);
	auto laser_detected = MotionManage::s_laser->laserObstcalDetected(0.2, 0, -1.0);

	if (get_obs_status() || is_obs_near() || (distance < SLOW_DOWN_DISTANCE) || is_map_front_block(3) || laser_detected )
	{
		if (distance < SLOW_DOWN_DISTANCE)
			angle_diff = 0;
		integrated_ = 0;
		if (base_speed_ > (int32_t) LINEAR_MIN_SPEED)
			base_speed_--;
	}else
	if (base_speed_ < (int32_t) LINEAR_MAX_SPEED)
	{
		if (tick_++ > 1)
		{
			tick_ = 0;
			base_speed_++;
		}
		integrated_ = 0;
	}

	if((FORCE_MOVE_LINE && std::abs(dis_diff) > CELL_COUNT_MUL/4 && distance > SLOW_DOWN_DISTANCE*4) && (get_clean_mode() != Clean_Mode_WallFollow))
	{
		integrated_ = 0;

//		auto diff = (dis_diff > 0) ? (dis_diff - CELL_COUNT_MUL/4) : (dis_diff + CELL_COUNT_MUL/4);
//		float cell = (diff/CELL_COUNT_MUL);

//		auto diff2 = (std::abs(dis_diff) < CELL_COUNT_MUL/4) ? 1 : 3;
//		if (dis_diff < 0)  diff2 = - diff2;
		left_speed = base_speed_ - dis_diff / (CELL_COUNT_MUL/4)/* - diff2*/ /*- integrated_ / 150*/; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
		right_speed = base_speed_ + dis_diff / (CELL_COUNT_MUL/4) /*+ diff2 *//*+ integrated_ / 150*/; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
//		ROS_WARN("left_speed(%d),right_speed(%d),dis_diff(%d),diff1(%d),diff2(%d)",left_speed, right_speed, dis_diff, diff,diff2);
		//ROS_WARN("left_speed(%d),right_speed(%d),dis_diff(%d),diff(%d)",left_speed, right_speed, dis_diff, dis_diff / (CELL_COUNT_MUL/4));
	}
	else{
		left_speed = base_speed_ - angle_diff / 20 - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
		right_speed = base_speed_ + angle_diff / 20 + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
		//ROS_ERROR("left_speed(%d),right_speed(%d),angle_diff(%d), intergrated_(%d)",left_speed, right_speed, angle_diff, integrated_);
	}

#if LINEAR_MOVE_WITH_PATH
	check_limit(left_speed, 0, LINEAR_MAX_SPEED);
	check_limit(right_speed, 0, LINEAR_MAX_SPEED);
#else
	check_limit(left_speed, LINEAR_MIN_SPEED, LINEAR_MAX_SPEED);
	check_limit(right_speed, LINEAR_MIN_SPEED, LINEAR_MAX_SPEED);
#endif
	base_speed_ = (left_speed + right_speed) / 2;
}

FollowWallRegulator::FollowWallRegulator(Point32_t start_point, Point32_t target) : previous_(0), seen_charger_counter(0)
{
	extern bool g_keep_on_wf;
	if (!g_keep_on_wf) {
		g_straight_distance = 300;
		s_origin = start_point;
		s_origin_angle = gyro_get_angle();
		s_target = target;
		g_is_left_start = false;
		map_init(WFMAP);
		ROS_INFO("%s, %d: ", __FUNCTION__, __LINE__);
	} else {
		g_keep_on_wf = false;
		ROS_INFO("reset g_keep_on_wf");
	}
}

bool FollowWallRegulator::isReach()
{
//	ROS_INFO("target_(%d,%d)",s_target.X,s_target.Y);
	//map_set_realtime();
	if (get_clean_mode() != Clean_Mode_WallFollow) {
//		MotionManage::s_laser->laserMarker(true);
		map_set_laser();
		map_set_obs();
//		map_set_obs();
	}
	bool ret = false;
	if (get_clean_mode() == Clean_Mode_WallFollow)
	{
		if (wf_is_end())
		{
			//wf_break_wall_follow();
			ret = true;
		}
	} else if (get_clean_mode() == Clean_Mode_Navigation || get_clean_mode() == Clean_Mode_Exploration)
	{
		if (g_trapped_mode != 0)
		{
			if ((time(NULL) - g_escape_trapped_timer) > ESCAPE_TRAPPED_TIME)
			//if ((time(NULL) - g_escape_trapped_timer) > ESCAPE_TRAPPED_TIME || wf_is_end())
			{
				ROS_WARN("%s %d: Escape trapped timeout.", __FUNCTION__, __LINE__);
				g_fatal_quit_event = true;
				ret = true;
			}
			if (trapped_is_end()) {
				ROS_WARN("%s %d: Trapped wall follow is loop closed. ", __FUNCTION__, __LINE__);
				g_trapped_mode = 0;
				extern int g_isolate_count;
				if (g_isolate_count > 3) {
					g_fatal_quit_event = true;
				}
				ret = true;
			}
			if (g_trapped_mode == 2)
			{
				wf_clear();
//				wav_play(WAV_CLEANING_START);
				ROS_WARN("%s:%d: out of esc", __FUNCTION__, __LINE__);
				g_trapped_mode = 0;
				// This led light is for debug.
				set_led_mode(LED_STEADY, LED_GREEN);
				ret = true;
			}
		} else
		{
			auto curr = map_point_to_cell(s_curr_p);
			auto target = map_point_to_cell(s_target);
			auto origin = map_point_to_cell(s_origin);
			if ((s_origin.Y < s_target.Y ^ s_curr_p.Y < s_target.Y))
			{
				auto dx = (s_origin.Y < s_target.Y  ^ mt_is_left()) ? +2 : -2;
				if(is_block_blocked(count_to_cell(s_curr_p.X)+dx, count_to_cell(s_curr_p.Y)))
				{
					ROS_WARN("%s %d: is_map_front_block", __FUNCTION__, __LINE__);
					ret = true;
				}
				if(std::abs(s_origin.Y - s_curr_p.Y) > CELL_COUNT_MUL*3)
				{
					ROS_WARN("%s %d: reach the target, CELL_COUNT_MUL*3, s_origin.Y(%d), target.Y(%d),curr_y(%d)", __FUNCTION__, __LINE__, count_to_cell(s_origin.Y), count_to_cell(s_target.Y), count_to_cell(s_curr_p.Y));
					ret = true;
				}
			}else
			if ((s_target.Y > s_origin.Y && (s_origin.Y - s_curr_p.Y) > 120) ||
					(s_target.Y < s_origin.Y && (s_curr_p.Y - s_origin.Y) > 120))
			{
//				auto dy = (s_origin.Y < s_target.Y  ^ mt_is_left()) ? +2 : -2;
				PPTargetType path_;
				path_.cells.clear();
				MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell, path_.cells);
//				if(!is_block_blocked_x_axis(count_to_cell(s_curr_p.X), count_to_cell(s_curr_p.Y/*+dy*/)))
//				{
//					ROS_WARN("%s %d: is_map_front_block", __FUNCTION__, __LINE__);
//					ret = true;
//				}
//				auto angle_diff = ranged_angle( gyro_get_angle());
				auto target_angel  = (s_target.Y > s_origin.Y) ? -900 : 900;
//				ROS_INFO("%s %d: target_angel(%d),curr(%d)diff(%d)", __FUNCTION__, __LINE__, target_angel, gyro_get_angle(), target_angel - gyro_get_angle());
				if(std::abs(gyro_get_angle()-target_angel) <100 || is_block_cleaned_unblock(curr.X,curr.Y))
				{
					ROS_WARN("%s %d: opposite direcition, old_dir(%d) s_origin.Y(%d), target.Y(%d),curr_y(%d)", __FUNCTION__, __LINE__, g_old_dir, count_to_cell(s_origin.Y), count_to_cell(s_target.Y), count_to_cell(s_curr_p.Y));
					ROS_WARN("%s %d: is_map_front_block", __FUNCTION__, __LINE__);
					ret = true;
				}
				s_target.Y += s_curr_p.Y - s_origin.Y;
				s_origin.Y = s_curr_p.Y;
//				ROS_WARN("%s %d: opposite direcition, origin.Y(%d), s_target_y(%d)", __FUNCTION__, __LINE__, s_origin.Y, s_target.Y);
			}
			else{
//			if(s_curr_p.X < std::min(s_origin.X , s_target.X) - CELL_COUNT_MUL*2 || s_curr_p.X > std::max(s_origin.X , s_target.X) + CELL_COUNT_MUL*2)
//				if(is_block_cleaned_unblock(curr.X,curr.Y))
//					ret = true;
				if(std::abs(gyro_get_angle() - s_origin_angle) > 900 && is_block_cleaned_unblock(curr.X,curr.Y))
				{
					ROS_WARN("%s %d: curr_angle(%d), origin_angle(%d),diff(%d)", __FUNCTION__, __LINE__,gyro_get_angle(), s_origin_angle,std::abs(gyro_get_angle() - s_origin_angle));
					ret = true;
				}
			}
		}
	}
	return ret;
}

bool FollowWallRegulator::isSwitch()
{
//	ROS_INFO("FollowWallRegulator isSwitch");
	if( g_bumper_triggered || get_bumper_status()){
		if(! g_bumper_triggered)
			g_bumper_triggered = get_bumper_status();
		g_turn_angle = bumper_turn_angle();
		ROS_INFO("%s %d: g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
	if( g_cliff_triggered || get_cliff_status()){
		if(! g_cliff_triggered)
			g_cliff_triggered = get_cliff_status();
		g_turn_angle = cliff_turn_angle();
		ROS_INFO("%s %d: g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
	if( g_tilt_triggered || get_tilt_status()){
		if(! g_tilt_triggered)
			g_tilt_triggered = get_tilt_status();
		g_turn_angle = tilt_turn_angle();
		ROS_INFO("%s %d: g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
//	if(get_rcon_status() != 0)
//		ROS_ERROR("%s,%d: rcon_status(%x)",__FUNCTION__, __LINE__, get_rcon_status());
#if 0
	auto rcon_tmp = get_rcon_trig();
//	if(rcon_tmp != 0)
//		ROS_WARN("rcon_tmp(%d)",rcon_tmp);

	/*---rcon handled in adjustSpeed()---*/
	if( g_rcon_triggered || rcon_tmp){
		if(! g_rcon_triggered)
			g_rcon_triggered = rcon_tmp;
		path_set_home(map_get_curr_cell());
		g_turn_angle = rcon_turn_angle();
		g_straight_distance = 80;
		ROS_INFO("%s %d: g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
#endif
	auto obs_tmp = LASER_MARKER ?  MotionManage::s_laser->laserMarker(true,0.14,wall_follow_detect_distance): (get_front_obs() > get_front_obs_value());
	if(obs_tmp) {
		if( g_bumper_triggered || get_bumper_status()){
			if(! g_bumper_triggered)
				g_bumper_triggered = get_bumper_status();
			g_turn_angle = bumper_turn_angle();
			ROS_INFO("%s %d: g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
			return true;
		}
//		ROS_INFO("Laser Stop in wall follow");
		if(! g_obs_triggered )
			g_obs_triggered = Status_Front_OBS;
		if(g_trapped_mode == 1)
			g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
		g_turn_angle = obs_turn_angle();
		g_straight_distance = 100;
		ROS_INFO("%s %d: g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
	if(g_robot_slip)
	{
		return true;
	}
	return false;
}

bool FollowWallRegulator::_isStop()
{
//	ROS_INFO("FollowWallRegulator _isStop");
	bool ret = false;
	return ret;
}

void FollowWallRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	ROS_DEBUG("%s %d: FollowWallRegulator.", __FUNCTION__, __LINE__);
	set_dir_forward();
//	uint32_t same_dist = (get_right_wheel_step() / 100) * 11 ;
	uint32_t rcon_status = 0;
	auto _l_step = get_left_wheel_step();
	auto _r_step = get_right_wheel_step();
	auto &same_dist = (mt_is_left()) ? _l_step : _r_step;
	auto &diff_dist = (mt_is_left()) ? _r_step : _l_step;
	auto &same_speed = (mt_is_left()) ? l_speed : r_speed;
	auto &diff_speed = (mt_is_left()) ? r_speed : l_speed;
	wall_buffer[2]=wall_buffer[1];
	wall_buffer[1]=wall_buffer[0];
	wall_buffer[0]=(mt_is_left()) ? robot::instance()->getLeftWall() : robot::instance()->getRightWall();

	rcon_status = get_rcon_status();
	/*---only use a part of the Rcon signal---*/
	rcon_status &= (RconFL2_HomeT|RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT);
	if(rcon_status)
	{
//		g_rcon_triggered = get_rcon_trig();
//		map_set_rcon();
		int32_t linear_speed = 24;
		/* angular speed notes						*
		 * larger than 0 means move away from wall	*
		 * less than 0 means move close to wall		*/
		int32_t angular_speed = 0;

		seen_charger_counter = 30;
		if(rcon_status & (RconFR_HomeT|RconFL_HomeT))
		{
			angular_speed = 12;
		}
		else if(rcon_status & RconFR2_HomeT)
		{
			if(mt_is_left())
				angular_speed = 15;
			else if(mt_is_right())
				angular_speed = 10;
		}
		else if(rcon_status & RconFL2_HomeT)
		{
			if(mt_is_left())
				angular_speed = 10;
			else if(mt_is_right())
				angular_speed = 15;
		}
		reset_rcon_status();
		/*---check if should eloud the charger---*/
		if(seen_charger_counter)
		{
			same_speed = linear_speed + angular_speed;
			diff_speed = linear_speed - angular_speed;
			return ;
		}
	}
	if(seen_charger_counter)
	{
		seen_charger_counter--;
		return ;
	}

//	ROS_INFO("same_dist: %d < g_straight_distance : %d", same_dist, g_straight_distance);
	if ((same_dist) < (uint32_t) g_straight_distance)
	{
		int32_t speed;
		if (same_dist < 500)
			speed = (same_dist < 100)? 10 :15;
		else
			speed = 23;
		same_speed = diff_speed = speed;
//		ROS_INFO("same_dist: %d < g_straight_distance : %d", same_dist, g_straight_distance);
	}
	else
	{
		auto wheel_speed_base = 17 + diff_dist / 150;
		if (wheel_speed_base > 28)wheel_speed_base = 28;

		auto adc_value = (mt_is_left()) ? robot::instance()->getLeftWall() : robot::instance()->getRightWall();

		auto proportion = (adc_value - g_wall_distance) * 100 / g_wall_distance;

		auto delta = proportion - previous_;

		if (robot_to_wall_distance > 0.8 || abs(adc_value - g_wall_distance) > 150 )
		{//over left
			same_speed = wheel_speed_base + proportion / 7 + delta/2; //
			diff_speed = wheel_speed_base - proportion / 7 - delta/2; //

			if (wheel_speed_base < 26)
			{
				reset_sp_turn_count();
					if (diff_speed > wheel_speed_base + 6)
				{
					diff_speed = 32;
					same_speed = 6;
//				ROS_INFO("Wf_2, same_speed = %d, diff_speed = %d", same_speed, diff_speed);
				}
				else if (same_speed > wheel_speed_base + 10)
				{
					diff_speed = 6;
					same_speed = 28;
//					ROS_INFO("Wf_3, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
				}
			}
			else
			{
				if (diff_speed > 35)
				{
					add_sp_turn_count();
					diff_speed = 34;
					same_speed = 6;
//					ROS_INFO("Wf_4, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
//					ROS_INFO("get_sp_turn_count() = %d", get_sp_turn_count());
				}
				else
				{
					reset_sp_turn_count();
				}
			}
		}
		else
		{
			same_speed = wheel_speed_base + proportion / 11 + delta/2;//16
			diff_speed = wheel_speed_base - proportion / 11 - delta/2; //11
//			ROS_INFO("Wf_4.1, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
			if (wheel_speed_base < 26)
			{
				reset_sp_turn_count();
				if (diff_speed > wheel_speed_base + 4)
				{
					diff_speed = 32;
					same_speed = 5;
//					ROS_INFO("Wf_5, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
				}
			}
			else
			{
				if (diff_speed > 32)
				{
					add_sp_turn_count();
					diff_speed = 35;
					same_speed = 6;
//					ROS_INFO("Wf_6, same_speed = %d, diff_speed = %d, g_robot_to_wall_distance = %d", same_speed, diff_speed, g_robot_to_wall_distance);
//					ROS_INFO("g_sp_turn_count() = %d",get_sp_turn_count());
				}
				else
				{
					reset_sp_turn_count();
				}
			}
		}

		/****************************************************turn a right angular***************************************************/
		if (wall_buffer[0] < 100)
		{
			if(g_wall_distance > 250)
				turn_right_angle_factor = 50;				//white
			else
				turn_right_angle_factor = 13;				//black

//			ROS_WARN("g_wall_distance: %d",g_wall_distance);
//			ROS_WARN("wall_buffer[1] - wall_buffer[0]: %d",wall_buffer[1] - wall_buffer[0]);
			if ((wall_buffer[1] - wall_buffer[0]) >= g_wall_distance / turn_right_angle_factor)
			{
//				ROS_WARN("wall_buffer[2] - wall_buffer[1]: %d",wall_buffer[2] - wall_buffer[1]);
				if ((wall_buffer[2] - wall_buffer[1]) >= g_wall_distance / turn_right_angle_factor)
				{
					if (same_dist>200)
					{
						if ((diff_speed-same_speed) >= -3)
						{
							// Away from the wall.
//						ROS_WARN("%s,%d: delay_sec(0.22) to walk straight",__FUNCTION__,__LINE__);
//							if(ros::Time::now().toSec()-transit_time < 0.8 && (g_wall_distance != last_transit_g_wall_distance))
//							{
//								g_wall_distance=last_transit_g_wall_distance;
//								ROS_WARN("set back g_wall_distance: %d",g_wall_distance);
//							}
							if(mt_is_left()) {
								if (is_decelerate_wall()) {
									move_forward((2 * 300 * (wall_follow_detect_distance - 0.167) + (20 - 15)) / 2,
															 (2 * 300 * (wall_follow_detect_distance - 0.167) - (20 - 15)) / 2);
									delay_sec(0.4);
//									ROS_WARN("delay_decelerate");
								} else {
									move_forward(20, 15);
									delay_sec(0.22);
									ROS_WARN("%s,%d: delay_sec(0.22) to walk straight",__FUNCTION__,__LINE__);
								}
							}
							else
							{
								if(is_decelerate_wall()) {
									move_forward((2 * 300 * (wall_follow_detect_distance - 0.167) - (20 - 15)) / 2,
															 (2 * 300 * (wall_follow_detect_distance - 0.167) + (20 - 15)) / 2);
									delay_sec(0.3);
								}else {
									move_forward(15, 20);
									delay_sec(0.22);
								}
							}
//							g_wall_distance+=15;
							g_straight_distance = 250;
						}
					}
				}
			}
		}
		/****************************************************END**************************************************************/
#if 0
		/*********************reset g_wall_distance when transits from white wall to black wall**********************************/
//		ROS_WARN("wall_buffer[0]:%d,wall_buffer[1]=%d,wall_buffer[2]:%d",wall_buffer[0],wall_buffer[1],wall_buffer[2]);
/*		if (wall_buffer[0] >150 && wall_buffer[0] < 300 && wall_buffer[1] > 200 &&wall_buffer[2] > 200 && wall_buffer[1] < wall_buffer[2] )
		{
//			ROS_WARN("g_wall_distance: %d",g_wall_distance);
//			ROS_WARN("wall_buffer[1] - wall_buffer[0]: %d",wall_buffer[1] - wall_buffer[0]);
			if ((wall_buffer[1] - wall_buffer[0]) >= g_wall_distance/30)
			{
//				ROS_WARN("wall_buffer[2] - wall_buffer[1]: %d",wall_buffer[2] - wall_buffer[1]);
				if ((wall_buffer[2] - wall_buffer[1]) >= g_wall_distance/30 && ((wall_buffer[1] - wall_buffer[0]+10) > (wall_buffer[2] - wall_buffer[1])))
				{
//					ROS_WARN("same_dist: %d",same_dist);
					if (same_dist>200)
					{
//						ROS_WARN("get_right_wheel_speed-get_left_wheel_speed: %d",get_right_wheel_speed()-get_left_wheel_speed());
						if ((diff_speed-same_speed) >= -3)
						{
							last_transit_g_wall_distance=g_wall_distance;
							transit_time=ros::Time::now().toSec();
							g_wall_distance=wall_buffer[0];
//							ROS_WARN("set g_wall_distance from white wall to black wall: %d",g_wall_distance);
							if(g_wall_distance < 300)  //620 is the experience value by testing in the closest position to white wall
							{
								g_wall_distance+=(150-g_wall_distance)/4*3;
								if(g_wall_distance < WALL_DISTANCE_BLACK_MIN)
									g_wall_distance=WALL_DISTANCE_BLACK_MIN;
								else if(g_wall_distance > WALL_DISTANCE_BLACK_MAX)
									g_wall_distance=WALL_DISTANCE_BLACK_MAX;
								ROS_WARN("transit from white wall to black wall,reset g_wall_distance: %d",g_wall_distance);
							}
						}
					}
				}
			}
		}*/
		/****************************************************END**************************************************************/

		/***************reset g_wall_distance when transits from black wall to white wall**********************/
/*		if(wall_buffer[0] > 350 && wall_buffer[1] < 420 && wall_buffer[2] < 420)
		{
//			ROS_WARN("wall_buffer[2]: %d,wall_buffer[1]: %d",wall_buffer[2],wall_buffer[0]);
//			ROS_WARN("g_wall_distance: %d",g_wall_distance);
//			ROS_WARN("wall_buffer[0] - wall_buffer[1]: %d",wall_buffer[0] - wall_buffer[1]);
			if(wall_buffer[0]-wall_buffer[1] >=(g_wall_distance / 5))
			{
//				ROS_WARN("wall_buffer[1] - wall_buffer[2]: %d",wall_buffer[1] - wall_buffer[2]);
				if(wall_buffer[1]-wall_buffer[2] >= 0)
					if(diff_dist > 200)
						if((diff_speed-same_speed) >= -3)
						{
							g_wall_distance=wall_buffer[0];
							if(g_wall_distance < 620)  //620 is the experience value by testing in the closest position to white wall
							{
								g_wall_distance+=(620-g_wall_distance)/4*3;
								if(g_wall_distance < WALL_DISTANCE_WHITE_MIN)
									g_wall_distance=WALL_DISTANCE_WHITE_MIN;
								ROS_WARN("transit from black wall to white wall,reset g_wall_distance: %d",g_wall_distance);
							}
						}
			}
		}*/
		/******************************************END******************************************************/
#endif
//		ROS_ERROR("same_speed:%d,diff_speed:%d",same_speed,diff_speed);

		previous_ = proportion;

		if (same_speed > 39)same_speed = 39;
		if (same_speed < 0)same_speed = 0;
		if (diff_speed > 35)diff_speed = 35;
		if (diff_speed < 5)diff_speed = 5;
	}

	if (is_decelerate_wall()) {
		old_same_speed = same_speed;
		old_diff_speed = diff_speed;
		if (next_linear_speed > (300 * (wall_follow_detect_distance - 0.167))){
			if(next_linear_speed == INT_MAX)
				next_linear_speed = (old_same_speed + old_diff_speed) / 2 - 1;
			same_speed = (2 * next_linear_speed + next_linear_speed * (old_same_speed - old_diff_speed) / (old_same_speed + old_diff_speed)) / 2;
			diff_speed = (2 * next_linear_speed - next_linear_speed * (old_same_speed - old_diff_speed) / (old_same_speed + old_diff_speed)) / 2;
			next_linear_speed = (same_speed + diff_speed) / 2 - 1;
//			ROS_ERROR("decelerate:same_speed:%d,diff_speed:%d,next_linear_speed:%d",same_speed,diff_speed,next_linear_speed);
		} else{
			//the first parameter 300 must below 638 to ensure the linear velocity below to the calculating linear velocity
				same_speed = (2 * (300 * (wall_follow_detect_distance - 0.167)) + old_same_speed - old_diff_speed) / 2;
				diff_speed = (2 * (300 * (wall_follow_detect_distance - 0.167)) + old_diff_speed - old_same_speed) / 2;
//				ROS_ERROR("continue:same_speed:%d,diff_speed:%d,linear_speed:%d",same_speed,diff_speed,(same_speed + diff_speed) / 2);
		}
		if(same_speed < 0) {
			diff_speed -= same_speed;
			same_speed = 0;
//			ROS_ERROR("below zero by same_speed:same_speed:%d,diff_speed:%d",same_speed,diff_speed);
		}
		else if(diff_speed < 0)
		{
			same_speed -= diff_speed;
			diff_speed = 0;
//			ROS_ERROR("below zero by diff_speed:same_speed:%d,diff_speed:%d",same_speed,diff_speed);
		}
	} else{
		next_linear_speed = INT_MAX;
	}
}

GoToChargerRegulator::GoToChargerRegulator()
{
	ROS_INFO("%s %d: Init", __FUNCTION__, __LINE__);
	go_home_state_now = GO_TO_CHARGER_INIT;
}

bool GoToChargerRegulator::isReach()
{
	if (g_charge_detect)
		return true;
	return false;
}

bool GoToChargerRegulator::isSwitch()
{
	if (go_home_state_now == GO_TO_CHARGER_INIT)
	{
		resetGoToChargerVariables();
		g_go_to_charger_back_30cm = false;
		g_go_to_charger_back_10cm = false;
		g_go_to_charger_back_0cm = false;
		go_home_state_now = CHECK_NEAR_CHARGER_STATION;
	}
	if (go_home_state_now == CHECK_NEAR_CHARGER_STATION)
	{
		if(no_signal_cnt < 10)
		{
			receive_code = get_rcon_trig();
			ROS_INFO("%s, %d: check near home, receive_code: %8x", __FUNCTION__, __LINE__, receive_code);
			if(receive_code&RconAll_Home_T)
			{
				ROS_INFO("receive LR");
				if(receive_code&(RconFL_HomeT|RconFR_HomeT|RconFL2_HomeT|RconFR2_HomeT))
				{
					ROS_INFO("turn 180");
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 1800;
				}
				else if(receive_code&RconR_HomeT)
				{
					ROS_INFO("turn left 90");
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 900;
				}
				else if(receive_code&RconL_HomeT)
				{
					ROS_INFO("turn right 90");
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 900;
				}
				go_home_state_now = AWAY_FROM_CHARGER_STATION;
				resetGoToChargerVariables();
				return true;
			}
			else
				no_signal_cnt++;
		}
		else
		{
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL;
			resetGoToChargerVariables();
		}
	}
	if (go_home_state_now == AWAY_FROM_CHARGER_STATION)
	{
		g_bumper_triggered = get_bumper_status();
		if(g_bumper_triggered)
		{
			ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL;
			g_turn_angle = 0;
			resetGoToChargerVariables();
			return true;
		}
		g_cliff_triggered = get_cliff_status();
		if(g_cliff_triggered)
		{
			ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL;
			g_turn_angle = 0;
			resetGoToChargerVariables();
			return true;
		}
		if(move_away_from_charger_cnt++ > 20)
		{
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL;
			resetGoToChargerVariables();
		}
	}
	if (go_home_state_now == TURN_FOR_CHARGER_SIGNAL)
	{
		if(gyro_step < 360)
		{
			// Handle for angle
			current_angle = robot::instance()->getAngle();
			angle_offset = static_cast<float>(ranged_angle((current_angle - last_angle) * 10)) / 10;
			ROS_DEBUG("Current_Angle = %f, Last_Angle = %f, Angle_Offset = %f, Gyro_Step = %f.", current_angle, last_angle, angle_offset, gyro_step);
			if (angle_offset < 0)
				gyro_step += (-angle_offset);
			last_angle = current_angle;

			// Handle for bumper and cliff
			g_bumper_triggered = get_bumper_status();
			if(g_bumper_triggered)
			{
				ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
				g_turn_angle = 0;
				resetGoToChargerVariables();
				return true;
			}
			g_cliff_triggered = get_cliff_status();
			if(g_cliff_triggered)
			{
				ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
				resetGoToChargerVariables();
				g_turn_angle = 0;
				go_home_state_now = GO_TO_CHARGER_INIT;
				return true;
			}

			// Handle for rcon signal
			receive_code = get_rcon_trig();
			if (receive_code)
				go_home_state_now = AROUND_CHARGER_STATION_INIT;
			else
				g_turn_angle = 0;

			// HomeL
			if(receive_code & (RconFL_HomeL | RconFR_HomeL))
			{
				g_turn_angle = -900;
				around_charger_stub_dir = 1;
				if(receive_code & RconFL_HomeL)//FL H_L
					ROS_INFO("Start with FL-L.");
				else if(receive_code&RconFR_HomeL)//FR H_L
					ROS_INFO("Start with FR-L.");
			}
			else if(receive_code&RconFL2_HomeL)//FL2 H_L
			{
				ROS_INFO("Start with FL2-L.");
				g_turn_angle = -600;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR2_HomeL)//FR2 H_L
			{
				ROS_INFO("Start with FR2-L.");
				g_turn_angle = -850;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconL_HomeL)// L  H_L
			{
				ROS_INFO("Start with L-L.");
				g_turn_angle = 0;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconR_HomeL)// R  H_L
			{
				ROS_INFO("Start with R-L.");
				g_turn_angle = -1500;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBL_HomeL)//BL H_L
			{
				ROS_INFO("Start with BL-L.");
				g_turn_angle = 800;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBR_HomeL)//BL H_L
			{
				ROS_INFO("Start with BR-L.");
				g_turn_angle = -800;
				around_charger_stub_dir = 0;
			}
			// HomeR
			else if(receive_code & (RconFL_HomeR | RconFR_HomeR))
			{
				g_turn_angle = 900;
				around_charger_stub_dir = 0;
				if(receive_code&RconFL_HomeR)//FL H_R
					ROS_INFO("Start with FL-R.");
				else if(receive_code&RconFR_HomeR)//FR H_R
					ROS_INFO("Start with FR-R.");
			}
			else if(receive_code&RconFL2_HomeR)//FL2 H_R
			{
				ROS_INFO("Start with FL2-R.");
				g_turn_angle = 850;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconFR2_HomeR)//FR2 H_R
			{
				ROS_INFO("Start with FR2-R.");
				g_turn_angle = 600;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconL_HomeR)// L  H_R
			{
				ROS_INFO("Start with L-R.");
				g_turn_angle = 1500;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconR_HomeR)// R  H_R
			{
				ROS_INFO("Start with R-R.");
				g_turn_angle = 0;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconBR_HomeR)//BR H_R
			{
				ROS_INFO("Start with BR-R.");
				g_turn_angle = -800;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconBL_HomeR)//BL H_R
			{
				ROS_INFO("Start with BL-R.");
				g_turn_angle = 800;
				around_charger_stub_dir = 1;
			}
			// HomeT
			else if(receive_code&RconFL_HomeT)//FL H_T
			{
				ROS_INFO("Start with FL-T.");
				g_turn_angle = -600;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR_HomeT)//FR H_T
			{
				ROS_INFO("Start with FR-T.");
				g_turn_angle = -800;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFL2_HomeT)//FL2 H_T
			{
				ROS_INFO("Start with FL2-T.");
				g_turn_angle = -600;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR2_HomeT)//FR2 H_T
			{
				ROS_INFO("Start with FR2-T.");
				g_turn_angle = -800;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconL_HomeT)// L  H_T
			{
				ROS_INFO("Start with L-T.");
				g_turn_angle = -1200;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconR_HomeT)// R  H_T
			{
				ROS_INFO("Start with R-T.");
				g_turn_angle = -1200;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBL_HomeT)//BL H_T
			{
				ROS_INFO("Start with BL-T.");
				g_turn_angle = 300;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBR_HomeT)//BR H_T
			{
				ROS_INFO("Start with BR-T.");
				g_turn_angle = -300;
				around_charger_stub_dir = 0;
			}

			if (g_turn_angle != 0)
			{
				//g_go_to_charger_back_0cm = true;
				return true;
			}
		}
		// gyro_step > 360 is handled in GoToChargerRegulator::_isStop()
	}
	if (go_home_state_now == AROUND_CHARGER_STATION_INIT)
	{
		go_home_bumper_cnt = 0;
		//move_forward(9, 9);
		reset_rcon_status();
		ROS_INFO("%s, %d: Call Around_ChargerStation with dir = %d.", __FUNCTION__, __LINE__, around_charger_stub_dir);
		go_home_state_now = AROUND_CHARGER_STATION;
		around_move_cnt = 5;
	}
	if (go_home_state_now == AROUND_CHARGER_STATION)
	{
		g_cliff_triggered = get_cliff_status();
		if(g_cliff_triggered)
		{
			ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
			g_turn_angle = 1750;
			go_home_state_now = GO_TO_CHARGER_INIT;
			return true;
		}
		g_bumper_triggered = get_bumper_status();
		if(g_bumper_triggered)
		{
			ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
			around_charger_stub_dir = 1 - around_charger_stub_dir;
			g_turn_angle = 1800;
			if(++go_home_bumper_cnt > 1)
				go_home_state_now = GO_TO_CHARGER_INIT;
			return true;
		}

		if (--around_move_cnt <= 0)
		{
			around_move_cnt = 5;
			receive_code = get_rcon_trig();
			if(receive_code)
				no_signal_cnt = 0;
			else if(++no_signal_cnt > 60)
			{
				ROS_WARN("%s %d:No charger signal received.", __FUNCTION__, __LINE__);
				go_home_state_now = GO_TO_CHARGER_INIT;
			}

			//ROS_DEBUG("%s %d Check DIR: %d, and do something", __FUNCTION__, __LINE__, around_charger_stub_dir);
			if(around_charger_stub_dir == 1)//10.30
			{
				if(receive_code&(RconFR_HomeR|RconFL_HomeR))
				{
					//go_home_state_now = BY_PATH_INIT;
					g_turn_angle = 0;
					if(receive_code&RconFR_HomeR)
						ROS_INFO("%s, %d: Detect FR-R, call By_Path().", __FUNCTION__, __LINE__);
					else if(receive_code&RconFL_HomeR)
						ROS_INFO("%s, %d: Detect FL-R, call By_Path().", __FUNCTION__, __LINE__);
					// Debug
					beep_for_command(true);
				}
				else if(receive_code&RconL_HomeR)
				{
					ROS_INFO("%s, %d: Detect L-R, Check position.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_LEFT;
					//go_home_state_now = CHECK_POSITION_INIT;
					g_turn_angle = 0;
					// Debug
					beep_for_command(true);
				}
				else if(receive_code&(RconFL_HomeL|RconFL_HomeT))
				{
					g_turn_angle = -500;
					if(receive_code&RconFL_HomeL)//FL_HL
						ROS_DEBUG("%s, %d: Detect FL-L.", __FUNCTION__, __LINE__);
					else if(receive_code&RconFL_HomeT)//FL_HT
						ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconFR_HomeT)//FR_HT
				{
					ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
					g_turn_angle = -800;
				}
				else if(receive_code&RconFR2_HomeT)//FR2_T
				{
					ROS_DEBUG("%s, %d: Detect FR2-T.", __FUNCTION__, __LINE__);
					g_turn_angle = -900;
				}
				else if(receive_code&RconR_HomeT)//R_HT
				{
					ROS_DEBUG("%s, %d: Detect R-T.", __FUNCTION__, __LINE__);
					g_turn_angle = -1100;
					around_charger_stub_dir = 0;
				}
				else
					g_turn_angle = 0;

				if (g_turn_angle != 0)
				{
					//g_go_to_charger_back_0cm = true;
					return true;
				}
			}
			else //around_charger_stub_dir == 0
			{
				if(receive_code&(RconFL_HomeL|RconFR_HomeL))
				{
					//go_home_state_now = BY_PATH_INIT;
					g_turn_angle = 0;
					if(receive_code&(RconFL_HomeL))
						ROS_INFO("%s, %d: Detect FL-L, call By_Path().", __FUNCTION__, __LINE__);
					else if(receive_code&(RconFR_HomeL))
						ROS_INFO("%s, %d: Detect FR-L, call By_Path().", __FUNCTION__, __LINE__);
					// Debug
					beep_for_command(true);
				}
				else if(receive_code&(RconR_HomeL))
				{
					ROS_INFO("%s, %d: Detect R-L, check position.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_RIGHT;
					//go_home_state_now = CHECK_POSITION_INIT;
					g_turn_angle = 0;
					// Debug
					beep_for_command(true);
				}
				else if(receive_code&(RconFR_HomeR|RconFR_HomeT))
				{
					g_turn_angle = 500;
					if(receive_code&RconFR_HomeR)
						ROS_DEBUG("%s, %d: Detect FR-R.", __FUNCTION__, __LINE__);
					else if(receive_code&RconFR_HomeT)
						ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconFL_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
					g_turn_angle = 800;
				}
				else if(receive_code&RconFL2_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
					g_turn_angle = 900;
				}
				else if(receive_code&RconL_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
					g_turn_angle = 1100;
					around_charger_stub_dir = 1;
				}
				else
					g_turn_angle = 0;

				if (g_turn_angle != 0)
				{
					//g_go_to_charger_back_0cm = true;
					return true;
				}
			}
		}
	}

	return false;
}

bool GoToChargerRegulator::_isStop()
{
	bool ret = false;
	if(g_robot_stuck)
		ret = true;
	if (go_home_state_now == TURN_FOR_CHARGER_SIGNAL && gyro_step > 360)
		ret = true;
	return ret;
}

void GoToChargerRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	/*---check if near charger station---*/
	if (go_home_state_now == CHECK_NEAR_CHARGER_STATION)
		l_speed = r_speed = 0;
	else if (go_home_state_now == AWAY_FROM_CHARGER_STATION)
	{
		set_dir_forward();
		l_speed = r_speed = 30;
	}
	else if (go_home_state_now == TURN_FOR_CHARGER_SIGNAL)
	{
		set_dir_right();
		l_speed = r_speed = 10;
	}
	else if (go_home_state_now == AROUND_CHARGER_STATION_INIT)
	{
		set_dir_forward();
		l_speed = r_speed = 9;
		around_move_cnt = 5;
	}
	else if (go_home_state_now == AROUND_CHARGER_STATION)
	{
		if (around_move_cnt == 5 && around_charger_stub_dir == 1)
		{
			if(receive_code&RconL_HomeT)
			{
				ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 22;
				r_speed = 12;
			}
			else if(receive_code&RconL_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect L-L.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 22;
				r_speed = 12;
			}
			else if(receive_code&RconFL2_HomeT)
			{
				ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 23;
				r_speed = 14;
			}
			else if(receive_code&RconFL2_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect FL2-L.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 20;
				r_speed = 14;
			}
			else if(receive_code&RconFL2_HomeR)
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 15;
				r_speed = 21;
			}
			else if (--around_move_cnt == 0)
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 14;
				r_speed = 21;
			}
		}
		else if (around_move_cnt == 5 && around_charger_stub_dir == 0)
		{
			if(receive_code&RconR_HomeT)
			{
				ROS_DEBUG("%s %d Detect R-T.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 12;
				r_speed = 22;
			}
			else if(receive_code&RconR_HomeR)
			{
				ROS_DEBUG("%s %d Detect R-R.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 12;
				r_speed = 22;
			}
			else if(receive_code&RconFR2_HomeT)
			{
				ROS_DEBUG("%s %d Detect FR2-T.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 14;
				r_speed = 23;
			}
			else if(receive_code&RconFR2_HomeR)
			{
				ROS_DEBUG("%s %d Detect FR2-R.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 14;
				r_speed = 20;
			}
			else if(receive_code&RconFR2_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 21;
				r_speed = 15;
			}
			else if(--around_move_cnt == 0)
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				set_dir_forward();
				l_speed = 21;
				r_speed = 14;
				around_move_cnt = 5;
			}
		}
	}
}

void SelfCheckRegulator::adjustSpeed(uint8_t bumper_jam_state)
{
	uint8_t left_speed;
	uint8_t right_speed;
	if (g_oc_suction)
		left_speed = right_speed = 0;
	else if (g_oc_wheel_left || g_oc_wheel_right)
	{
		if (g_oc_wheel_right) {
			set_dir_right();
		} else {
			set_dir_left();
		}
		left_speed = 30;
		right_speed = 30;
	}
	else if (g_cliff_jam)
	{
		set_dir_backward();
		left_speed = right_speed = 18;
	}
	else if (g_bumper_jam)
	{
		switch (bumper_jam_state)
		{
			case 1:
			case 2:
			case 3:
			{
				// Quickly move back for a distance.
				set_dir_backward();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 4:
			{
				// Quickly turn right for 90 degrees.
				set_dir_right();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 5:
			{
				// Quickly turn left for 180 degrees.
				set_dir_left();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
		}
	}
	else if(g_omni_notmove)
	{
		//set_dir_backward();
		//left_speed = right_speed = RUN_TOP_SPEED;
	}
	else if(g_robot_stuck)
	{
	//	set_dir_backward();
		//left_speed = right_speed = RUN_TOP_SPEED;
	}

	set_wheel_speed(left_speed, right_speed);
}

//RegulatorManage
RegulatorManage::RegulatorManage(const Cell_t& start_cell, const Cell_t& target_cell, const PPTargetType& path)
{
#if FORCE_MOVE_LINE
	auto origin = map_cell_to_point(start_cell);
#else
	s_curr_p.X = map_get_x_count();
	s_curr_p.Y = map_get_y_count();
#endif
	g_wall_distance=WALL_DISTANCE_HIGH_LIMIT;
	bumper_turn_factor=0.85;
	auto target = map_cell_to_point(target_cell);
	ROS_INFO("%s %d: start cell\033[33m(%d, %d)\033[0m, target\033[33m(%d, %d)\033[0m.", __FUNCTION__, __LINE__, start_cell.X, start_cell.Y, count_to_cell(target.X), count_to_cell(target.Y));
	g_bumper_cnt = g_cliff_cnt =0;
	g_slip_cnt = 0;
	g_rcon_during_go_home = false;
	reset_rcon_status();

	back_reg_ = new BackRegulator();

	if(mt_is_follow_wall())
	{
		mt_reg_ = new FollowWallRegulator(s_curr_p, target);
		ROS_INFO("%s %d: obs(\033[32m%d\033[0m), rcon(\033[32m%d\033[0m), bum(\033[32m%d\033[0m), cliff(\033[32m%d\033[0m), tilt(\033[32m%d\033[0m),slip(\033[32m%d\033[0m)",__FUNCTION__, __LINE__, g_obs_triggered, g_rcon_triggered, g_bumper_triggered, g_cliff_triggered, g_tilt_triggered,g_robot_slip);
		int16_t block_angle=0;
		if (g_obs_triggered)
			block_angle = obs_turn_angle();
		else if (g_bumper_triggered)
			block_angle = bumper_turn_angle();
		else if (g_cliff_triggered)
			block_angle = cliff_turn_angle();
		else if (g_tilt_triggered)
			block_angle = tilt_turn_angle();
	//	else if (g_rcon_triggered)
	//		block_angle = rcon_turn_angle();
		else
			block_angle = 0;
		if (LASER_FOLLOW_WALL)
			block_angle = laser_turn_angle();
//		int16_t target_angle = ranged_angle( course_to_dest(s_curr_p.X, s_curr_p.Y, s_target.X, s_target.Y) - gyro_get_angle());
//		g_turn_angle = std::max(std::abs(block_angle), std::abs(target_angle));
//		g_turn_angle = std::max(std::abs(block_angle), std::abs(target_angle));
		g_turn_angle = ranged_angle( course_to_dest(s_curr_p.X, s_curr_p.Y, s_target.X, s_target.Y) - gyro_get_angle());
		if (!g_is_left_start)
			s_origin_angle = gyro_get_angle() + g_turn_angle;
	}
	else if(mt_is_linear())
	{
		mt_reg_ = new LinearRegulator(target, path);
		g_turn_angle = ranged_angle(
					course_to_dest(s_curr_p.X, s_curr_p.Y, s_target.X, s_target.Y) - gyro_get_angle());
	}
	else if (mt_is_go_to_charger())
	{
		mt_reg_ = new GoToChargerRegulator();
		g_turn_angle = 0;
	}

	ROS_INFO("%s, %d: g_turn_angle(\033[32m%d\033[0m)",__FUNCTION__,__LINE__, g_turn_angle);
	turn_reg_ = new TurnRegulator(ranged_angle(gyro_get_angle() + g_turn_angle));
	p_reg_ = turn_reg_;

	robot::instance()->obsAdjustCount(50);
	cm_set_event_manager_handler_state(true);

	ROS_INFO("%s, %d: RegulatorManage finish",__FUNCTION__,__LINE__);
}

RegulatorManage::~RegulatorManage()
{
	delete turn_reg_;
	delete back_reg_;
	delete mt_reg_;
	set_wheel_speed(0,0);
	cm_set_event_manager_handler_state(false);
}
void RegulatorManage::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	if (p_reg_ != nullptr)
		p_reg_->adjustSpeed(left_speed, right_speed);
}

bool RegulatorManage::isReach()
{
	if ( (mt_is_linear() && (p_reg_ == back_reg_ || p_reg_ == mt_reg_)) || (mt_is_follow_wall() && p_reg_ == mt_reg_) ){
		return p_reg_->isReach();
	}
	return false;
}

bool RegulatorManage::isSwitch()
{
	if (p_reg_ != nullptr)
		return p_reg_->isSwitch();
	return false;
}

bool RegulatorManage::_isStop()
{
	if (p_reg_ != nullptr)
		return p_reg_->_isStop();
	return false;
}

void RegulatorManage::switchToNext()
{
	if (p_reg_ == turn_reg_)
	{
		if(g_robot_slip || g_bumper_triggered || g_cliff_triggered || g_tilt_triggered ){
			p_reg_ = back_reg_;
			ROS_INFO("%s %d: From turn_reg_ to back_reg_.", __FUNCTION__, __LINE__);
		}
		else /*if(g_obs_triggered || g_rcon_triggered)*/
		{
			p_reg_ = mt_reg_;
			ROS_INFO("%s %d: From turn_reg_ to mt_reg_.", __FUNCTION__, __LINE__);
		}
	}
	else if (p_reg_ == back_reg_)
	{
		p_reg_ = turn_reg_;
		ROS_INFO("%s %d: From back_reg_ to turn_reg_.", __FUNCTION__, __LINE__);
	}
	else if (p_reg_ == mt_reg_)
	{
		if (g_robot_slip || g_bumper_triggered || g_cliff_triggered || g_tilt_triggered 
			|| (mt_is_go_to_charger() && g_go_to_charger_back_10cm))
		{
			p_reg_ = back_reg_;
			ROS_INFO("%s %d: From mt_reg_ to back_reg_.", __FUNCTION__, __LINE__);
		}
		else/* if (g_obs_triggered || g_rcon_triggered)*/
		{
			p_reg_ = turn_reg_;
			ROS_INFO("%s %d: From mt_reg_ to turn_reg_.", __FUNCTION__, __LINE__);
		}
	}
	ROS_INFO("%s %d: g_obs_triggered(\033[32m%d\033[0m), g_rcon_triggered(\033[32m%d\033[0m), g_bumper_hitted(\033[32m%d\033[0m), g_cliff_triggered(\033[32m%d\033[0m), g_tilt_triggered(\033[32m%d\033[0m),g_robot_slip(\033[32m%d\033[0m)",__FUNCTION__, __LINE__, g_obs_triggered, g_rcon_triggered, g_bumper_triggered, g_cliff_triggered, g_tilt_triggered,g_robot_slip);
	setTarget();
	if(p_reg_ != back_reg_){//note: save when robot leave move_to
		g_rcon_triggered = 0;
		g_bumper_triggered = 0;
		g_obs_triggered = 0;
		g_cliff_triggered = 0;
		g_tilt_triggered = 0;
	}
}

