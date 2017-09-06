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
int jam=0;
int16_t wall_buffer[3]={0};
int strength = STRENGTH_LOW_LIMIT;
//int last_strength=150;
//int last_transit_strength=150;
//double transit_time=0;
int factor=15;
double bumper_turn_factor=0.85;
bool line_is_found;
double wall_distance=0.8;
float back_distance=0.01;
uint8_t seen_charger_counter = 0;
CMMoveType last_move_type;
//bool g_is_should_follow_wall;

static int16_t bumper_turn_angle()
{
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
		jam = get_wheel_step() < 2000 ? ++jam : 0;
		strength = STRENGTH_HIGHT_LIMIT;
		} else if (status == diff_side)
	{
		g_turn_angle = -850;
		strength = STRENGTH_HIGHT_LIMIT;
	} else if (status == same_side)
	{
		strength = bumper_turn_factor * strength;
		if(strength < 330)
			strength = STRENGTH_LOW_LIMIT;
		g_turn_angle =0;
		ROS_WARN("%s, %d: g_turn_angle(%d)",__FUNCTION__,__LINE__, g_turn_angle);
		if (g_trapped_mode != 1) {
			g_turn_angle = (jam >= 3 || (get_obs() <= get_obs_value() - 200)) ? -180 : -280;
		} else {
			g_turn_angle = (jam >= 3 || (get_obs() <= get_obs_value() - 200)) ? -100 : -200;
		}
		ROS_WARN("%s, %d: g_turn_angle(%d)",__FUNCTION__,__LINE__, g_turn_angle);

		g_straight_distance = 100; //250;
		jam = get_wheel_step() < 2000 ? ++jam : 0;
	}
	ROS_INFO("strength in bumper_turn_angular: %d",strength);
	g_straight_distance = 100;
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
			wall_distance=back_distance*100*sin(line_angle*3.1415/180.0);
		else
			wall_distance=back_distance*100*sin((180-line_angle)*3.1415/180.0);
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
//	ROS_INFO("reg_base _isStop");
	return g_battery_home || g_remote_spot || (!g_go_home && g_remote_home) || cm_should_self_check();
}


BackRegulator::BackRegulator() : speed_(8), counter_(0)
{
//	ROS_INFO("%s, %d: ", __FUNCTION__, __LINE__);
}

bool BackRegulator::isReach()
{
	auto distance = sqrtf(powf(s_pos_x - robot::instance()->getOdomPositionX(), 2) +
			   	powf(s_pos_y - robot::instance()->getOdomPositionY(), 2));
	if (g_tilt_triggered)
		back_distance = 0.05f;
	else
		back_distance = 0.01f;
	if(fabsf(distance) > back_distance){
		if (g_tilt_triggered && get_tilt_status())
		{
			// Still tilt.
			BackRegulator::setTarget();
			return false;
		}
		ROS_INFO("%s, %d: BackRegulator ", __FUNCTION__, __LINE__);
		g_bumper_cnt =get_bumper_status() == 0 ? 0 : g_bumper_cnt+1 ;
		g_cliff_cnt = get_cliff_status() == 0 ? 0 : g_cliff_cnt+1 ;
		if(g_bumper_cnt == 0 && g_cliff_cnt == 0)
			return true;
		if(g_bumper_cnt >= 2 || g_cliff_cnt >= 2){
			if(g_cliff_cnt >= 2)
				g_cliff_jam = true;
			else
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
	if(mt_is_follow_wall())
		return isReach();
	if(mt_is_linear())
		return false;

	return false;
}

bool BackRegulator::_isStop()
{
	MotionManage::s_laser->laserMarker(false);
	bool ret = false;
	if(g_robot_stuck)
		ret = true;
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
			strength = (mt_is_left()) ? robot::instance()->getLeftWall() : robot::instance()->getRightWall();
/*			if(strength < 10)	//set strength in U round
			{
				strength=last_strength;
				ROS_ERROR("strength: %d",strength);
				return true;
			}*/

			ROS_INFO("strength: %d",strength);
			if(strength < 150)  //150 is the experience value by testing in the closest position to black wall
			{
				strength+=(150-strength)/4*3;
				if(strength < STRENGTH_BLACK_MIN)
					strength=STRENGTH_BLACK_MIN;
				ROS_INFO("strength_distance_adjust: %d",strength);
			}
			else if(strength >250 && strength < 320) // boardline invagination
			{
				strength +=(400-strength)/5*4;
				ROS_INFO("boradline invagination，strength: %d",strength);
			}
			else if(strength > 320 && strength < 620)  //620 is the experience value by testing in the closest position to white wall
			{
				strength+=(620-strength)/4*3;
				if(strength < STRENGTH_WHITE_MIN)
					strength=STRENGTH_WHITE_MIN;
				else if(strength > STRENGTH_WHITE_MAX)
					strength=0.95*strength;
				ROS_INFO("strength_distance_adjust: %d",strength);
			}
//			last_strength=strength;
			line_is_found=false;
		}
		else
			ROS_INFO("line is not found");
		return true;
	}
		/**********************************************END**********************************************************/

		return false;
}

bool TurnRegulator::isSwitch()
{
//	ROS_INFO("TurnRegulator::isSwitch");

	if(isReach() ||(! g_bumper_triggered  && get_bumper_status()) || (! g_cliff_triggered && get_cliff_status()) || (!g_tilt_triggered && get_tilt_status()))
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
	if(g_robot_stuck)
		ret = true;
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
	ROS_INFO("%s %d: current cell(%d,%d), target cell(%d,%d) ", __FUNCTION__, __LINE__, map_get_x_cell(),map_get_y_cell(), count_to_cell(s_target.X), count_to_cell(s_target.Y));
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
			//ROS_INFO("\033[31m" "%s,%d,g_next_cell(%d,%d)" "\033[0m",__FUNCTION__,__LINE__,g_next_cell.X,g_next_cell.Y);
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
		ROS_INFO("\033[31m""%s, %d: LinearRegulator2:g_new_dir(%d),is_x_axis(%d),is_pos(%d),curr_p(%d),target(%d)""\033[0m", __FUNCTION__, __LINE__,g_new_dir,IS_X_AXIS(g_new_dir),IS_POS_AXIS(g_new_dir),curr_p, target);
		return true;
	}

	return false;
}

bool LinearRegulator::isSwitch()
{

	if ((! g_bumper_triggered && get_bumper_status())
		|| (! g_cliff_triggered && get_cliff_status())
		|| (! g_tilt_triggered && get_tilt_status()))
	{
//		g_is_should_follow_wall = true;
		ROS_INFO("%s, %d:LinearRegulator g_bumper_triggered || g_cliff_triggered || g_tilt_triggered.", __FUNCTION__, __LINE__);
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
		mt_set(CM_LINEARMOVE);

		return true;
	}

	return false;
}

bool LinearRegulator::_isStop()
{
	auto rcon_tmp = get_rcon_trig();
	bool obs_tmp;
		if(g_robot_stuck)
		return true;

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


FollowWallRegulator::FollowWallRegulator(Point32_t start_point, Point32_t target) : previous_(0)
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
		MotionManage::s_laser->laserMarker(true);
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
	} else if (get_clean_mode() == Clean_Mode_Navigation)
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
			if ((s_origin.Y < s_target.Y ^ s_curr_p.Y < s_target.Y))
			{
				ROS_WARN("%s %d: reach the target, s_origin.Y(%d), target.Y(%d),curr_y(%d)", __FUNCTION__, __LINE__,
								 count_to_cell(s_origin.Y), count_to_cell(s_target.Y), count_to_cell(s_curr_p.Y));
				auto dx = (s_origin.Y < s_target.Y  ^ mt_is_left()) ? +2 : -2;
				if(is_block_blocked(count_to_cell(s_curr_p.X)+dx, count_to_cell(s_curr_p.Y)))
				{
					ROS_WARN("%s %d: is_map_front_block", __FUNCTION__, __LINE__);
					ret = true;
				}
				if(std::abs(s_origin.Y - s_curr_p.Y) > CELL_COUNT_MUL*3)
					ret = true;
			}
			if ((s_target.Y > s_origin.Y && (s_origin.Y - s_curr_p.Y) > 120) ||
					(s_target.Y < s_origin.Y && (s_curr_p.Y - s_origin.Y) > 120))
			{
				ROS_WARN("%s %d: opposite direcition, old_dir(%d) s_origin.Y(%d), target.Y(%d),curr_y(%d)", __FUNCTION__, __LINE__,
								 g_old_dir, count_to_cell(s_origin.Y), count_to_cell(s_target.Y), count_to_cell(s_curr_p.Y));

//				auto dy = (s_origin.Y < s_target.Y  ^ mt_is_left()) ? +2 : -2;
				PPTargetType path_;
				path_.cells.clear();
				MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell, path_.cells);
//				if(!is_block_blocked_x_axis(count_to_cell(s_curr_p.X), count_to_cell(s_curr_p.Y/*+dy*/)))
//				{
//					ROS_WARN("%s %d: is_map_front_block", __FUNCTION__, __LINE__);
//					ret = true;
//				}
				auto angle_diff = ranged_angle( gyro_get_angle());
				auto target_angel  = (s_target.Y > s_origin.Y) ? -900 : 900;
				ROS_INFO("%s %d: target_angel(%d),curr(%d)diff(%d)", __FUNCTION__, __LINE__, target_angel, gyro_get_angle(), target_angel - gyro_get_angle());
				if(std::abs(gyro_get_angle()-target_angel) <100)
				{
					ROS_WARN("%s %d: is_map_front_block", __FUNCTION__, __LINE__);
					ret = true;
				}
//				ROS_INFO("%s %d: opposite direcition, origin.Y(%d), s_target_y(%d)", __FUNCTION__, __LINE__, s_origin.Y, s_target.Y);
				s_target.Y += s_curr_p.Y - s_origin.Y;
				s_origin.Y = s_curr_p.Y;
//				ROS_WARN("%s %d: opposite direcition, origin.Y(%d), s_target_y(%d)", __FUNCTION__, __LINE__, s_origin.Y, s_target.Y);
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
	if( g_obs_triggered  || get_front_obs() >= get_front_obs_value()+1000){
		if(! g_obs_triggered)
			g_obs_triggered = Status_Front_OBS;
		if(g_trapped_mode == 1)
			strength = STRENGTH_HIGHT_LIMIT;
		g_turn_angle = obs_turn_angle();
		g_straight_distance = 100;
		ROS_INFO("%s %d: g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	return false;
}

bool FollowWallRegulator::_isStop()
{
//	ROS_INFO("FollowWallRegulator isSwitch");

	return false;
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
	#if 0
	rcon_status &= (RconFL2_HomeT|RconFL2_HomeL|RconFL2_HomeR|
					RconFR_HomeT|RconFR_HomeL|RconFR_HomeR|
					RconFL_HomeT|RconFL_HomeL|RconFL_HomeR|
					RconFR2_HomeT|RconFR2_HomeL|RconFR2_HomeR|
					RconL_HomeL|RconL_HomeR|
					RconR_HomeL|RconR_HomeR);
	#endif
	rcon_status &= (RconFL2_HomeT|RconFL2_HomeL|RconFL2_HomeR|
					RconFR_HomeT|RconFR_HomeL|RconFR_HomeR|
					RconFL_HomeT|RconFL_HomeL|RconFL_HomeR|
					RconFR2_HomeT|RconFR2_HomeL|RconFR2_HomeR);
	if(rcon_status)
	{
		int32_t linear_speed = 24;
		/* angular speed notes						*
		 * larger than 0 means move away from wall	*
		 * less than 0 means move close to wall		*/
		int32_t angular_speed = 0;

		seen_charger_counter = 30;
		if(rcon_status & (RconFR_HomeT|RconFR_HomeL|RconFR_HomeR|RconFL_HomeT|RconFL_HomeL|RconFL_HomeR))
		{
			angular_speed = 12;
		}
		else if(rcon_status & (RconFR2_HomeT|RconFR2_HomeL|RconFR2_HomeR))
		{
			if(mt_is_left())
				angular_speed = 15;
			else if(mt_is_right())
				angular_speed = 10;
		}
		else if(rcon_status & (RconFL2_HomeT|RconFL2_HomeL|RconFL2_HomeR))
		{
			if(mt_is_left())
				angular_speed = 10;
			else if(mt_is_right())
				angular_speed = 15;
		}
		#if 0
		else if(rcon_status & (RconL_HomeL|RconL_HomeR))
		{
			if(mt_is_left())
				angular_speed = -3;
			else if(mt_is_right())
				seen_charger_counter = 0;/*---no use to elude the charger---*/
		}
		else if(rcon_status & (RconR_HomeL|RconR_HomeR))
		{
			if(mt_is_left())
				seen_charger_counter = 0;/*---no use to elude the charger---*/
			else if(mt_is_right())
				angular_speed = -3;
		}
		#endif
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

		auto proportion = (adc_value - strength) * 100 / strength;

		auto delta = proportion - previous_;

		if (wall_distance > 0.8 || abs(adc_value - strength) > 150 )
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
//					ROS_INFO("Wf_3, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
				}
			}
			else
			{
				if (diff_speed > 35)
				{
					add_sp_turn_count();
					diff_speed = 34;
					same_speed = 6;
//					ROS_INFO("Wf_4, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
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
//			ROS_INFO("Wf_4.1, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
			if (wheel_speed_base < 26)
			{
				reset_sp_turn_count();
				if (diff_speed > wheel_speed_base + 4)
				{
					diff_speed = 32;
					same_speed = 5;
//					ROS_INFO("Wf_5, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
				}
			}
			else
			{
				if (diff_speed > 32)
				{
					add_sp_turn_count();
					diff_speed = 35;
					same_speed = 6;
//					ROS_INFO("Wf_6, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
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
			if(strength > 250)
				factor=50;				//white
			else
				factor=13;				//black

//			ROS_WARN("strength: %d",strength);
//			ROS_WARN("wall_buffer[1] - wall_buffer[0]: %d",wall_buffer[1] - wall_buffer[0]);
			if ((wall_buffer[1] - wall_buffer[0]) >= strength/factor)
			{
//				ROS_WARN("wall_buffer[2] - wall_buffer[1]: %d",wall_buffer[2] - wall_buffer[1]);
				if ((wall_buffer[2] - wall_buffer[1]) >= strength/factor)
				{
					if (same_dist>200)
					{
						if ((diff_speed-same_speed) >= -3)
						{
							// Away from the wall.
							ROS_WARN("%s,%d: delay_sec(0.22) to walk straight",__FUNCTION__,__LINE__);
//							if(ros::Time::now().toSec()-transit_time < 0.8 && (strength != last_transit_strength))
//							{
//								strength=last_transit_strength;
//								ROS_WARN("set back strength: %d",strength);
//							}
							if(mt_is_left())
								move_forward(20,15);
							else
								move_forward(15,20);
//							strength+=15;
							delay_sec(0.22);
							g_straight_distance = 250;
						}
					}
				}
			}
		}
		/****************************************************END**************************************************************/
#if 0
		/*********************reset strength when transits from white wall to black wall**********************************/
//		ROS_WARN("wall_buffer[0]:%d,wall_buffer[1]=%d,wall_buffer[2]:%d",wall_buffer[0],wall_buffer[1],wall_buffer[2]);
/*		if (wall_buffer[0] >150 && wall_buffer[0] < 300 && wall_buffer[1] > 200 &&wall_buffer[2] > 200 && wall_buffer[1] < wall_buffer[2] )
		{
//			ROS_WARN("strength: %d",strength);
//			ROS_WARN("wall_buffer[1] - wall_buffer[0]: %d",wall_buffer[1] - wall_buffer[0]);
			if ((wall_buffer[1] - wall_buffer[0]) >= strength/30)
			{
//				ROS_WARN("wall_buffer[2] - wall_buffer[1]: %d",wall_buffer[2] - wall_buffer[1]);
				if ((wall_buffer[2] - wall_buffer[1]) >= strength/30 && ((wall_buffer[1] - wall_buffer[0]+10) > (wall_buffer[2] - wall_buffer[1])))
				{
//					ROS_WARN("same_dist: %d",same_dist);
					if (same_dist>200)
					{
//						ROS_WARN("get_right_wheel_speed-get_left_wheel_speed: %d",get_right_wheel_speed()-get_left_wheel_speed());
						if ((diff_speed-same_speed) >= -3)
						{
							last_transit_strength=strength;
							transit_time=ros::Time::now().toSec();
							strength=wall_buffer[0];
//							ROS_WARN("set strength from white wall to black wall: %d",strength);
							if(strength < 300)  //620 is the experience value by testing in the closest position to white wall
							{
								strength+=(150-strength)/4*3;
								if(strength < STRENGTH_BLACK_MIN)
									strength=STRENGTH_BLACK_MIN;
								else if(strength > STRENGTH_BLACK_MAX)
									strength=STRENGTH_BLACK_MAX;
								ROS_WARN("transit from white wall to black wall,reset strength: %d",strength);
							}
						}
					}
				}
			}
		}*/
		/****************************************************END**************************************************************/

		/***************reset strength when transits from black wall to white wall**********************/
/*		if(wall_buffer[0] > 350 && wall_buffer[1] < 420 && wall_buffer[2] < 420)
		{
//			ROS_WARN("wall_buffer[2]: %d,wall_buffer[1]: %d",wall_buffer[2],wall_buffer[0]);
//			ROS_WARN("strength: %d",strength);
//			ROS_WARN("wall_buffer[0] - wall_buffer[1]: %d",wall_buffer[0] - wall_buffer[1]);
			if(wall_buffer[0]-wall_buffer[1] >=(strength / 5))
			{
//				ROS_WARN("wall_buffer[1] - wall_buffer[2]: %d",wall_buffer[1] - wall_buffer[2]);
				if(wall_buffer[1]-wall_buffer[2] >= 0)
					if(diff_dist > 200)
						if((diff_speed-same_speed) >= -3)
						{
							strength=wall_buffer[0];
							if(strength < 620)  //620 is the experience value by testing in the closest position to white wall
							{
								strength+=(620-strength)/4*3;
								if(strength < STRENGTH_WHITE_MIN)
									strength=STRENGTH_WHITE_MIN;
								ROS_WARN("transit from black wall to white wall,reset strength: %d",strength);
							}
						}
			}
		}*/
		/******************************************END******************************************************/
#endif
//		ROS_WARN("same_speed:%d,diff_speed:%d",same_speed,diff_speed);

		previous_ = proportion;

		if (same_speed > 39)same_speed = 39;
		if (same_speed < 0)same_speed = 0;
		if (diff_speed > 35)diff_speed = 35;
		if (diff_speed < 5)diff_speed = 5;
	}
		if(get_clean_mode()==Clean_Mode_Navigation)
		{
				if(mt_get()!=last_move_type)
						wall_buffer[0]=wall_buffer[1]=wall_buffer[2]=0;
				last_move_type=mt_get();
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
	else if(g_omni_notmove){
		//set_dir_backward();
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
	auto target = map_cell_to_point(target_cell);
	ROS_INFO("%s %d: start cell(%d, %d), target(%d, %d).", __FUNCTION__, __LINE__, start_cell.X, start_cell.Y, count_to_cell(target.X), count_to_cell(target.Y));
	g_bumper_cnt = g_cliff_cnt =0;
	g_rcon_during_go_home = false;
	reset_rcon_status();

	back_reg_ = new BackRegulator();

	if (mt_is_follow_wall())
		mt_reg_ = new FollowWallRegulator(s_curr_p, target);
	else
		mt_reg_ = new LinearRegulator(target, path);

	if(mt_is_follow_wall())
	{
		ROS_INFO("%s %d: obs(\033[32m%d\033[0m), rcon(\033[32m%d\033[0m), bum(\033[32m%d\033[0m), cliff(\033[32m%d\033[0m), tilt(\033[32m%d\033[0m)",__FUNCTION__, __LINE__, g_obs_triggered, g_rcon_triggered, g_bumper_triggered, g_cliff_triggered, g_tilt_triggered);
		if (g_obs_triggered)
			g_turn_angle = obs_turn_angle();
		else if (g_bumper_triggered)
			g_turn_angle = bumper_turn_angle();
		else if (g_cliff_triggered)
			g_turn_angle = cliff_turn_angle();
		else if (g_tilt_triggered)
			g_turn_angle = tilt_turn_angle();
	//	else if (g_rcon_triggered)
	//		g_turn_angle = rcon_turn_angle();
		else
			g_turn_angle = 0;
		if (LASER_FOLLOW_WALL)
			g_turn_angle = laser_turn_angle();
		if (!g_is_left_start)
			s_origin_angle = gyro_get_angle() + g_turn_angle;
	}else if(mt_is_linear())
		g_turn_angle = ranged_angle(
					course_to_dest(s_curr_p.X, s_curr_p.Y, s_target.X, s_target.Y) - gyro_get_angle());

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
	strength=STRENGTH_HIGHT_LIMIT;
	bumper_turn_factor=0.85;
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
		if(g_bumper_triggered || g_cliff_triggered || g_tilt_triggered){
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
		if (g_obs_triggered || g_rcon_triggered)
		{
			p_reg_ = turn_reg_;
			ROS_INFO("%s %d: From mt_reg_ to turn_reg_.", __FUNCTION__, __LINE__);
		}
		else if (g_bumper_triggered || g_cliff_triggered || g_tilt_triggered )
		{
			p_reg_ = back_reg_;
			ROS_INFO("%s %d: From mt_reg_ to back_reg_.", __FUNCTION__, __LINE__);
		}
	}
	ROS_INFO("%s %d: g_obs_triggered(%d), g_rcon_triggered(%d), g_bumper_hitted(%d), g_cliff_triggered(%d), g_tilt_triggered(%d)",__FUNCTION__, __LINE__, g_obs_triggered, g_rcon_triggered, g_bumper_triggered, g_cliff_triggered, g_tilt_triggered);
	setTarget();
	if(p_reg_ != back_reg_){
		g_rcon_triggered = 0;
		g_bumper_triggered = 0;
		g_obs_triggered = 0;
		g_cliff_triggered = 0;
		g_tilt_triggered = 0;
	}
}

