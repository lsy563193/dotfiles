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
#include <tilt.h>
#include <beep.h>
#include <wall_follow.h>
#include "clean_mode.h"

#define TURN_REGULATOR_WAITING_FOR_LASER 1
#define TURN_REGULATOR_TURNING 2

extern PPTargetType g_plan_path;
//extern uint16_t g_old_dir;
extern int16_t g_new_dir;
extern Cell_t g_cell_history[];
int g_wall_distance = WALL_DISTANCE_LOW_LIMIT;
double bumper_turn_factor = 0.85;
bool line_is_found;
double robot_to_wall_distance = 0.8;
int16_t g_turn_angle;
float g_back_distance = 0.01;
// Back distance for go to charger regulator
bool g_go_to_charger_back_30cm = false;
bool g_go_to_charger_back_10cm = false;
bool g_go_to_charger_back_0cm = false;

double g_time_straight = 0.3;
double time_start_straight = 0;

bool g_slip_backward = false;

int16_t bumper_turn_angle()
{
	static int bumper_jam_cnt_ = 0;
	auto get_wheel_step = (mt_is_left()) ? &Wheel::get_right_step : &Wheel::get_left_step;
	auto get_obs = (mt_is_left()) ? &Obs::get_left : &Obs::get_right;
	auto get_obs_value = (mt_is_left()) ? &Obs::get_left_trig_value : &Obs::get_right_trig_value;
	auto status = ev.bumper_triggered;
	auto diff_side = (mt_is_left()) ? BLOCK_RIGHT : BLOCK_LEFT;
	auto same_side = (mt_is_left()) ? BLOCK_LEFT : BLOCK_RIGHT;

	if (status == BLOCK_ALL)
	{
		g_turn_angle = -600;
		g_straight_distance = 150; //150;
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
		if (!cs_is_trapped()) {
			g_turn_angle = (bumper_jam_cnt_ >= 3 || (obs.*get_obs)() <= (obs.*get_obs_value)()) ? -180 : -280;
		} else {
			g_turn_angle = (bumper_jam_cnt_ >= 3 || (obs.*get_obs)() <= (obs.*get_obs_value)()) ? -100 : -200;
		}
		//ROS_INFO("%s, %d: g_turn_angle(%d)",__FUNCTION__,__LINE__, g_turn_angle);

		g_straight_distance = 250; //250;
		bumper_jam_cnt_ = (wheel.*get_wheel_step)() < 2000 ? ++bumper_jam_cnt_ : 0;
	}
	//ROS_INFO("%s %d: g_wall_distance in bumper_turn_angular: %d", __FUNCTION__, __LINE__, g_wall_distance);
	g_straight_distance = 200;
	wheel.reset_step();
	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

int16_t cliff_turn_angle()
{
	g_turn_angle = -750;
	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

int16_t tilt_turn_angle()
{
	auto tmp_status = tilt.get_status();
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

int16_t obs_turn_angle()
{
	auto diff_side = (mt_is_left()) ? BLOCK_RIGHT : BLOCK_LEFT;
	auto same_side = (mt_is_left()) ? BLOCK_LEFT : BLOCK_RIGHT;
	if(ev.obs_triggered == BLOCK_FRONT)
		g_turn_angle = -850;
	else if(ev.obs_triggered == diff_side)
		g_turn_angle = -920;
	else if(ev.obs_triggered == same_side)
		g_turn_angle = -300;

	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
//	ROS_WARN("g_turn_angle(%d)",g_turn_angle);
	return g_turn_angle;
}

int16_t rcon_turn_angle()
{
	enum {left,fl2,fl,fr,fr2,right};
	int16_t left_angle[] =   {-300,-600,-850,-850,-950,-1100};
	int16_t right_angle[] =  {1100, 950, 850, 850, 600, 300};
	if(mt_is_left())
		g_turn_angle = left_angle[ev.rcon_triggered-1];
	else if(mt_is_right())
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

static bool _laser_turn_angle(int16_t& turn_angle, int laser_min, int laser_max, int angle_min,int angle_max,double dis_limit=0.217)
{
//	ROS_INFO("%s,%d,bumper (\033[32m%d\033[0m)!",__FUNCTION__,__LINE__,bumper.get_status());
	double line_angle;
	double distance;
//	auto RESET_WALL_DIS = 100;
	line_is_found = MotionManage::s_laser->laserGetFitLine(laser_min, laser_max, -1.0, dis_limit, &line_angle, &distance);
//	RESET_WALL_DIS = int(distance * 1000);

//	ROS_INFO("line_distance = %lf", distance);
//	ROS_INFO("line_angle_raw = %lf", line_angle);
	auto angle = double_scale_10(line_angle);

	if (mt_is_right())
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
		turn_angle = mt_is_right() ? angle : -angle;
//		ROS_INFO("laser generate turn angle(%d)!",turn_angle);
		return true;
	}
	return false;
}

bool laser_turn_angle(int16_t& turn_angle)
{
//	ROS_INFO("%s,%d: mt_is_fw",__FUNCTION__, __LINE__);
	wheel.stop();

	if (ev.obs_triggered != 0)
	{
//		ROS_INFO("%s %d: \033[32mfront obs trigger.\033[0m", __FUNCTION__, __LINE__);
		return _laser_turn_angle(turn_angle, 90, 270, 450, 1800, 0.25);
	}
	else if(ev.bumper_triggered != 0)
	{
		int angle_min, angle_max;
		if (mt_is_left() ^ (ev.bumper_triggered == BLOCK_LEFT))
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
			return _laser_turn_angle(turn_angle, 90, 270, 900, 1800);
		}
		else if (ev.bumper_triggered == BLOCK_RIGHT) {
//			ROS_INFO("%s %d: RightBumper trigger.", __FUNCTION__, __LINE__);
			return _laser_turn_angle(turn_angle, 90, 180, angle_min, angle_max);
		}
		else if (ev.bumper_triggered == BLOCK_LEFT) {
//			ROS_INFO("%s %d: LeftBumper trigger.", __FUNCTION__, __LINE__);
			return _laser_turn_angle(turn_angle, 180, 270, angle_min, angle_max);
		}
	}
	return false;
}

Point32_t RegulatorBase::s_target_p = {0,0};
Point32_t RegulatorBase::s_origin_p = {0,0};
int16_t RegulatorBase::s_target_angle = 0;
float RegulatorBase::s_pos_x = 0;
float RegulatorBase::s_pos_y = 0;
Point32_t RegulatorBase::s_curr_p = {0,0};

bool sp_turn_over(const Cell_t& curr)
{
	ROS_INFO("  %s %d:?? curr(%d,%d,%d)",__FUNCTION__,__LINE__, curr.X, curr.Y, curr.TH);
	/*check if spot turn*/
	if(get_sp_turn_count() > 400) {
		reset_sp_turn_count();
		ROS_WARN("  yes! sp_turn over 400");
		return true;
	}
	return false;
}

bool RegulatorBase::isExit()
{
	if (ev.fatal_quit || ev.key_clean_pressed || ev.charge_detect)
	{
		ROS_WARN("%s %d: ", __FUNCTION__, __LINE__);
		return true;
	}
	return false;
}

bool RegulatorBase::isStop()
{
	if (ev.battery_home || ev.remote_spot || (!cs_is_going_home() && ev.remote_home) || cm_should_self_check())
	{
		ROS_WARN("%s %d: ", __FUNCTION__, __LINE__);
		return true;
	}
	return false;
}

BackRegulator::BackRegulator() : counter_(0), speed_(BACK_MAX_SPEED), distance(0)
{
//	ROS_INFO("%s, %d: ", __FUNCTION__, __LINE__);
}

void BackRegulator::setTarget()
{
	s_pos_x = robot::instance()->getOdomPositionX();
	s_pos_y = robot::instance()->getOdomPositionY();
	if (g_robot_slip){
		g_back_distance = 0.30;
		g_slip_backward= true;
		g_robot_slip = false;
	}
	else if (ev.tilt_triggered)
		g_back_distance = 0.05;
	else if (g_go_to_charger_back_10cm)
		g_back_distance = 0.10;
	else if (g_go_to_charger_back_30cm)
		g_back_distance = 0.30;
	else if (g_go_to_charger_back_0cm)
		g_back_distance = 0.0;
	//else if (g_lidar_bumper)
	//	g_back_distnce = 0.15;
	else
		g_back_distance = 0.015;
	ROS_INFO("%s %d: Set back distance: %f.", __FUNCTION__, __LINE__, g_back_distance);
}

bool BackRegulator::isReach()
{
	distance = sqrtf(powf(s_pos_x - robot::instance()->getOdomPositionX(), 2) +
				powf(s_pos_y - robot::instance()->getOdomPositionY(), 2));
	ROS_DEBUG("%s, %d: BackRegulator distance %f", __FUNCTION__, __LINE__, distance);
	/*---------slip detect------*/
	if(g_robot_slip && g_slip_cnt >= 2){
		g_robot_slip = false;
		g_slip_backward = false;
		ROS_WARN("%s,%d,\033[1mrobot slip %d times!!\033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
		beep_for_command(false);
		return true;
	}
	if(fabsf(distance) >= g_back_distance)
	{
		if(g_slip_backward){
			ROS_WARN("%s,%d,\033[1mrobot slip backward reach!! distance(%f),back_distance(%f)\033[0m",__FUNCTION__,__LINE__,distance,g_back_distance);
			g_slip_backward= false;
			return true;
		}

		g_bumper_cnt = bumper.get_status() == 0 ? 0 : g_bumper_cnt+1 ;
		g_cliff_cnt = cliff.get_status() == 0 ? 0 : g_cliff_cnt+1 ;
		ev.tilt_triggered = tilt.get_status();
		//g_lidar_bumper_cnt = robot::instance()->getLidarBumper() == 0? 0:g_lidar_bumper_cnt+1;

		if (g_bumper_cnt == 0 && g_cliff_cnt == 0 && !ev.tilt_triggered)
		{
			if (mt_is_go_to_charger())
			{
				g_go_to_charger_back_30cm = false;
				g_go_to_charger_back_10cm = false;
				g_go_to_charger_back_0cm = false;
			}
			ROS_INFO("%s, %d: BackRegulator reach target.", __FUNCTION__, __LINE__);
			return true;
		}
		if (g_cliff_cnt >= 2)
		{
			ev.cliff_jam = true;
			ROS_WARN("%s, %d: Cliff jam.", __FUNCTION__, __LINE__);
			ev.cliff_jam = true;
			return false;
		}
		else if (g_bumper_cnt >= 2)
		{
			ev.bumper_jam = true;
			ROS_WARN("%s, %d: Bumper jam.", __FUNCTION__, __LINE__);
			ev.bumper_jam = true;
			return false;
		}
		//else if (g_lidar_bumper_cnt >= 2)
		//{
		//	g_lidar_bumper_jam = true;
		//	return false;
		//}
		else
			setTarget();
	}
	return false;
}

bool BackRegulator::isLaserStop()
{
	auto obstacle_distance = MotionManage::s_laser->getObstacleDistance(1, ROBOT_RADIUS);
	if (g_back_distance >= 0.05 && obstacle_distance < 0.03)
	{
		ROS_WARN("%s, %d: obstacle_distance:%f.", __FUNCTION__, __LINE__, obstacle_distance);
		return true;
	}

	return false;
}

void BackRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
//	ROS_INFO("BackRegulator::adjustSpeed");
	wheel.set_dir_backward();
	if (!cm_is_follow_wall())
	{
		speed_ += ++counter_;
		speed_ = (speed_ > BACK_MAX_SPEED) ? BACK_MAX_SPEED : speed_;
	}
	wheel.reset_step();
	/*if (fabsf(distance) >= g_back_distance * 0.8)
	{
		l_speed = r_speed = speed_--;
		check_limit(l_speed, BACK_MIN_SPEED, BACK_MAX_SPEED);
		check_limit(r_speed, BACK_MIN_SPEED, BACK_MAX_SPEED);
	}
	else*/
		l_speed = r_speed = speed_;
}

TurnRegulator::TurnRegulator(int16_t angle) : speed_(ROTATE_LOW_SPEED), stage_(TURN_REGULATOR_WAITING_FOR_LASER), waiting_finished_(false)
{
	accurate_ = ROTATE_TOP_SPEED > 30 ? 30 : 15;
	waiting_start_sec_ = ros::Time::now().toSec();
	s_target_angle = angle;
	if (mt_is_follow_wall())
	{
		wait_sec_ = 1;
		stage_ = TURN_REGULATOR_WAITING_FOR_LASER;
		skip_laser_turn_angle_cnt_ = 2;
	}
	else
	{
		wait_sec_ = 0;
		stage_ = TURN_REGULATOR_TURNING;
		skip_laser_turn_angle_cnt_ = 0;
	}
	ROS_INFO("%s %d: Init, \033[32ms_target_angle: %d\033[0m", __FUNCTION__, __LINE__, s_target_angle);
}
bool TurnRegulator::isReach()
{
	if (stage_ == TURN_REGULATOR_WAITING_FOR_LASER)
		setTarget();
	else if (abs(ranged_angle(s_target_angle - gyro_get_angle())) < accurate_){

		/*********************************************For wall follow**********************************************/
		if(line_is_found)
		{
			g_wall_distance = (mt_is_left()) ? wall.getLeft() : wall.getRight();
/*			if(g_wall_distance < 10)	//set g_wall_distance in U round
			{
				g_wall_distance=last_g_wall_distance;
				ROS_ERROR("g_wall_distance: %d",g_wall_distance);
				return true;
			}*/

			ROS_INFO("%s, %d: TurnRegulator target angle: \033[32m%d\033[0m, current angle: \033[32m%d\033[0m, g_wall_distance:%d."
					, __FUNCTION__, __LINE__, s_target_angle, gyro_get_angle(), g_wall_distance);
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
			ROS_INFO("%s, %d: TurnRegulator target angle: \033[32m%d\033[0m, current angle: \033[32m%d\033[0m, line is not found."
					, __FUNCTION__, __LINE__, s_target_angle, gyro_get_angle());
		time_start_straight = ros::Time::now().toSec();
		return true;
	}
		/**********************************************END**********************************************************/
	return false;
}

bool TurnRegulator::shouldMoveBack()
{
	// Robot should move back for these cases.
	ev.bumper_triggered = bumper.get_status();
	ev.cliff_triggered = cliff.get_status();
	ev.tilt_triggered = tilt.get_status();

	if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || g_robot_slip)
	{
		ROS_WARN("%s, %d,TurnRegulator, ev.bumper_triggered(\033[32m%d\033[0m) ev.cliff_triggered(\033[32m%d\033[0m) ev.tilt_triggered(\033[32m%d\033[0m) g_robot_slip(\033[32m%d\033[0m)."
				, __FUNCTION__, __LINE__,ev.bumper_triggered,ev.cliff_triggered,ev.tilt_triggered,g_robot_slip);
		return true;
	}

	return false;

}

void TurnRegulator::setTarget()
{
	if(cs_is_going_home() && map_point_to_cell(s_curr_p) == g_zero_home)
	{
		s_target_angle = g_home_point.TH;
	}
	else if(LASER_FOLLOW_WALL && !cs_is_trapped() && mt_is_follow_wall() && skip_laser_turn_angle_cnt_ >= 2)
	{
		// Use laser data for generating target angle in every 3 times of turning.
		if (waiting_finished_) {
			stage_ = TURN_REGULATOR_WAITING_FOR_LASER;
			waiting_finished_ = false;
			waiting_start_sec_ = ros::Time::now().toSec();
			wait_sec_ = 0.33;
			s_target_angle = gyro_get_angle();
			ROS_INFO("%s %d: TurnRegulator, start waiting for %fs.", __FUNCTION__, __LINE__, wait_sec_);
		}
		else
		{
			// Wait for specific time, for a new scan and gyro dynamic adjustment.
			double tmp_sec = ros::Time::now().toSec() - waiting_start_sec_;
			//ROS_INFO("%s %d: Has been wait for %f sec.", __FUNCTION__, __LINE__, tmp_sec);
			if (tmp_sec > wait_sec_) {
				waiting_finished_ = true;
				stage_ = TURN_REGULATOR_TURNING;
				laser_turn_angle(g_turn_angle);
				s_target_angle = ranged_angle(gyro_get_angle() + g_turn_angle);
				// Reset the speed.
				speed_ = ROTATE_LOW_SPEED;
				ROS_INFO("%s %d: TurnRegulator, current angle:%d, \033[33ms_target_angle: \033[32m%d\033[0m, after %fs waiting."
						, __FUNCTION__, __LINE__, gyro_get_angle(), s_target_angle, tmp_sec);
				skip_laser_turn_angle_cnt_ = 0;
			}
		}
	}
	else
	{
		s_target_angle = ranged_angle(gyro_get_angle() + g_turn_angle);
		stage_ = TURN_REGULATOR_TURNING;
		// Reset the speed.
		speed_ = ROTATE_LOW_SPEED;
		skip_laser_turn_angle_cnt_++;
		ROS_INFO("%s %d: TurnRegulator, \033[33ms_target_angle: \033[32m%d\033[0m, skip_laser_turn_angle_cnt_: %d.", __FUNCTION__, __LINE__, s_target_angle, skip_laser_turn_angle_cnt_);
	}
}

void TurnRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	if (stage_ == TURN_REGULATOR_WAITING_FOR_LASER)
	{
		l_speed = r_speed = 0;
		return;
	}

	auto diff = ranged_angle(s_target_angle - gyro_get_angle());
//	ROS_INFO("TurnRegulator::adjustSpeed diff(%d),(%d,%d)", diff,s_target_angle, gyro_get_angle());
	ROS_DEBUG("%s %d: TurnRegulator diff: %d, s_target_angle: %d, current angle: %d.", __FUNCTION__, __LINE__, diff, s_target_angle, gyro_get_angle());
	(diff >= 0) ? wheel.set_dir_left() : wheel.set_dir_right();

//	ROS_INFO("TurnRegulator::adjustSpeed");
	if (std::abs(diff) > 200){
		speed_ += 1;
		speed_ = std::min(speed_, ROTATE_TOP_SPEED);
	}
	else if (std::abs(diff) > 100){
		speed_ -= 2;
		uint8_t low_speed = ROTATE_LOW_SPEED + 5;
		speed_ = std::max(speed_, low_speed);
		ROS_DEBUG("%s %d: 100 - 200, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}
	else{
		speed_ -= 2;
		speed_ = std::max(speed_, ROTATE_LOW_SPEED);
		ROS_DEBUG("%s %d: 0 - 100, speed = %d.", __FUNCTION__, __LINE__, speed_);
	}

	l_speed = r_speed = speed_;

}

bool TurnSpeedRegulator::adjustSpeed(int16_t diff, uint8_t& speed)
{
	if ((diff >= 0) && (diff <= 1800))
		wheel.set_dir_left();
	else if ((diff <= 0) && (diff >= (-1800)))
		wheel.set_dir_right();

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
				integrated_(0),base_speed_(LINEAR_MIN_SPEED),integration_cycle_(0),tick_(0),turn_speed_(4),odom_y_start(0.0),odom_x_start(0.0)
{
//	g_is_should_follow_wall = false;
//	s_target = target;
//	path_ = path;
	//ROS_INFO("%s %d: current cell(%d,%d), target cell(%d,%d) ", __FUNCTION__, __LINE__, map_get_x_cell(),map_get_y_cell(), count_to_cell(s_target.X), count_to_cell(s_target.Y));
}

bool LinearRegulator::isCellReach()
{
	// Checking if robot has reached target cell.
	auto curr = (IS_X_AXIS(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = map_cell_to_point(g_plan_path.back());
	auto target = (IS_X_AXIS(g_new_dir)) ? target_p.X : target_p.Y;
	if (std::abs(s_curr_p.X - target_p.X) < CELL_COUNT_MUL_1_2 &&
		std::abs(s_curr_p.Y - target_p.Y) < CELL_COUNT_MUL_1_2)
	{
		ROS_INFO("\033[1m""%s, %d: LinearRegulator, reach the target cell (%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
						 g_plan_path.back().X, g_plan_path.back().Y);
		g_turn_angle = ranged_angle(g_new_dir - gyro_get_angle());
		return true;
	}

	return false;
}

bool LinearRegulator::isPoseReach()
{
	// Checking if robot has reached target cell and target angle.
	auto target_angle = g_plan_path.back().TH;
	if (isCellReach() && std::abs(ranged_angle(gyro_get_angle() - target_angle)) < 200)
	{
		ROS_INFO("\033[1m""%s, %d: LinearRegulator, reach the target cell and pose(%d,%d,%d)!!""\033[0m", __FUNCTION__, __LINE__,
				 g_plan_path.back().X, g_plan_path.back().Y, g_plan_path.back().TH);
		return true;
	}
	return false;
}

bool LinearRegulator::isNearTarget()
{
	auto curr = (IS_X_AXIS(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = map_cell_to_point(g_plan_path.front());
	auto &target = (IS_X_AXIS(g_new_dir)) ? target_p.X : target_p.Y;
	//ROS_INFO("%s %d: s_curr_p(%d, %d), target_p(%d, %d), dir(%d)",
	//		 __FUNCTION__, __LINE__, s_curr_p.X, s_curr_p.Y, target_p.X, target_p.Y, g_new_dir);
	if ((IS_POS_AXIS(g_new_dir) && (curr > target - 1.5 * CELL_COUNT_MUL)) ||
		(!IS_POS_AXIS(g_new_dir) && (curr < target + 1.5 * CELL_COUNT_MUL))) {
		if(g_plan_path.size() > 1)
		{
			// Switch to next target for smoothly turning.
			g_new_dir = g_plan_path.front().TH;
			g_plan_path.pop_front();
			ROS_INFO("%s %d: Curr(%d, %d), switch next cell(%d, %d), new dir(%d).", __FUNCTION__, __LINE__, map_get_x_cell(),
					 map_get_y_cell(), g_plan_path.front().X, g_plan_path.front().Y, g_new_dir);
		}
		else if(g_plan_path.front() != g_zero_home && g_allow_check_path_in_advance)
		{
			g_check_path_in_advance = true;
			ROS_INFO("%s %d: Curr(%d, %d), target(%d, %d), dir(%d), g_check_path_in_advance(%d)", __FUNCTION__, __LINE__, map_get_x_cell(),
					 map_get_y_cell(), g_plan_path.front().X, g_plan_path.front().Y, g_new_dir, g_check_path_in_advance);
			return true;
		}
	}
	return false;
}

bool LinearRegulator::shouldMoveBack()
{
	// Robot should move back for these cases.
	ev.bumper_triggered = bumper.get_status();
	ev.cliff_triggered = cliff.get_status();
	ev.tilt_triggered = tilt.get_status();

	if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || g_robot_slip)
	{
		ROS_WARN("%s, %d,ev.bumper_triggered(%d) ev.cliff_triggered(%d) ev.tilt_triggered(%d) g_robot_slip(%d)."
				, __FUNCTION__, __LINE__,ev.bumper_triggered,ev.cliff_triggered,ev.tilt_triggered,g_robot_slip);
		return true;
	}

	return false;
}

bool LinearRegulator::isRconStop()
{
	ev.rcon_triggered = c_rcon.get_trig();
	if(ev.rcon_triggered)
	{
		g_turn_angle = rcon_turn_angle();
		ROS_INFO("%s, %d: ev.rcon_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.rcon_triggered, g_turn_angle);
		return true;
	}

	return false;
}

bool LinearRegulator::isOBSStop()
{
	// Now OBS sensor is just for slowing down.
	return false;
/*
	ev.obs_triggered = obs.get_status(200, 1700, 200);
	if(ev.obs_triggered)
	{
		g_turn_angle = obs_turn_angle();
		ROS_INFO("%s, %d: ev.obs_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.obs_triggered, g_turn_angle);
		return true;
	}

	return false;*/
}

bool LinearRegulator::isLaserStop()
{
	ev.laser_triggered = get_laser_status();
	if (ev.laser_triggered)
	{
		// Temporary use OBS to get angle.
		ev.obs_triggered = ev.laser_triggered;
		g_turn_angle = obs_turn_angle();
		ROS_WARN("%s, %d: ev.laser_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.laser_triggered, g_turn_angle);
		return true;
	}

	return false;
}

bool LinearRegulator::isBoundaryStop()
{
	if (is_map_front_block(2))
	{
		ROS_INFO("%s, %d: LinearRegulator, Blocked boundary.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

bool LinearRegulator::isPassTargetStop()
{
	// Checking if robot has reached target cell.
	auto curr = (IS_X_AXIS(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = map_cell_to_point(g_plan_path.back());
	auto target = (IS_X_AXIS(g_new_dir)) ? target_p.X : target_p.Y;
	if ((IS_POS_AXIS(g_new_dir) && (curr > target + CELL_COUNT_MUL / 4)) ||
		(!IS_POS_AXIS(g_new_dir) && (curr < target - CELL_COUNT_MUL / 4)))
	{
		ROS_INFO("%s, %d: LinearRegulator, pass target: g_new_dir(\033[32m%d\033[0m),is_x_axis(\033[32m%d\033[0m),is_pos(\033[32m%d\033[0m),curr(\033[32m%d\033[0m),target(\033[32m%d\033[0m)",
				 __FUNCTION__, __LINE__, g_new_dir, IS_X_AXIS(g_new_dir), IS_POS_AXIS(g_new_dir), curr, target);
		return true;
	}
	return false;
}

void LinearRegulator::setTarget()
{
//	g_turn_angle = ranged_angle(
//						course_to_dest(s_curr_p.X, s_curr_p.Y, s_target_p.X, s_target_p.Y) - gyro_get_angle());
	s_target_p = map_cell_to_point(g_plan_path.back());
//	path_ = g_plan_path;
}

void LinearRegulator::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	ROS_WARN("%s,%d: g_path_size(%d)",__FUNCTION__, __LINE__,g_plan_path.size());
	wheel.set_dir_forward();
	auto curr = (IS_X_AXIS(g_new_dir)) ? s_curr_p.X : s_curr_p.Y;
	auto target_p = map_cell_to_point(g_plan_path.front());
	auto &target = (IS_X_AXIS(g_new_dir)) ? target_p.X : target_p.Y;


	int16_t angle_diff = 0;
	int16_t dis = std::min(std::abs(curr - target), (int32_t) (1.5 * CELL_COUNT_MUL));
	if (!IS_POS_AXIS(g_new_dir))
		dis *= -1;
	target = curr + dis;

	angle_diff = ranged_angle(
					course_to_dest(s_curr_p.X, s_curr_p.Y, target_p.X, target_p.Y) - gyro_get_angle());

//	ROS_WARN("curr(%d),x?(%d),pos(%d),dis(%d), target_p(%d,%d)", curr, IS_X_AXIS(g_new_dir), IS_POS_AXIS(g_new_dir), dis, target_p.X, target_p.Y);
//	auto dis_diff = IS_X_AXIS(g_new_dir) ? s_curr_p.Y - s_target_p.Y : s_curr_p.X - s_target_p.X;
//	dis_diff = IS_POS_AXIS(g_new_dir) ^ IS_X_AXIS(g_new_dir) ? dis_diff :  -dis_diff;

	if (integration_cycle_++ > 10) {
		auto t = map_point_to_cell(target_p);
		MotionManage::pubCleanMapMarkers(MAP, g_plan_path, &t);
		integration_cycle_ = 0;
		integrated_ += angle_diff;
		check_limit(integrated_, -150, 150);
	}
	auto distance = two_points_distance(s_curr_p.X, s_curr_p.Y, target_p.X, target_p.Y);
	auto obstalce_distance_front = MotionManage::s_laser->getObstacleDistance(0,ROBOT_RADIUS);
	uint8_t obs_state = obs.get_status();
	if (obs_state > 0 || (distance < SLOW_DOWN_DISTANCE) || is_map_front_block(3) || (obstalce_distance_front < 0.25))
	{
//		ROS_WARN("decelarate");
		if (distance < SLOW_DOWN_DISTANCE)
			angle_diff = 0;
		integrated_ = 0;
		if (base_speed_ > (int32_t) LINEAR_MIN_SPEED){
			base_speed_--;
			/*if(obstalce_distance_front > 0.025 && obstalce_distance_front < 0.125 && (left_speed > 20 || right_speed > 20)) {
				base_speed_ -= 2;
			}
			else if(obs_state & BLOCK_FRONT)
				base_speed_ -=2;
			else if(obs_state & (BLOCK_LEFT | BLOCK_RIGHT))
				base_speed_ --;*/
		}
	}
	else if (base_speed_ < (int32_t) LINEAR_MAX_SPEED) {
		if (tick_++ > 1) {
			tick_ = 0;
			base_speed_++;
		}
		integrated_ = 0;
	}

	left_speed =
					base_speed_ - angle_diff / 20 - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
	right_speed =
					base_speed_ + angle_diff / 20 + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
//		ROS_ERROR("left_speed(%d),right_speed(%d),angle_diff(%d), intergrated_(%d)",left_speed, right_speed, angle_diff, integrated_);

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
	if (!g_keep_on_wf) {
		g_straight_distance = 300;
		s_origin_p = start_point;
		s_target_p = target;
		map_init(WFMAP);
		ROS_INFO("%s, %d: ", __FUNCTION__, __LINE__);
	} else {
		g_keep_on_wf = false;
		ROS_INFO("reset g_keep_on_wf");
	}
}

bool FollowWallRegulator::isNewLineReach()
{
	auto ret = false;
	auto is_pos_dir = s_target_p.Y - s_origin_p.Y > 0;
	// The limit is CELL_COUNT_MUL / 8 * 3 further than target line center.
	auto target_limit = s_target_p.Y + CELL_COUNT_MUL / 8 * 3 * is_pos_dir;
	if (is_pos_dir ^ s_curr_p.Y < target_limit) // Robot has reached the target line limit.
	{
		ROS_WARN("%s %d: Reach the target limit, s_origin_p.Y(%d), target.Y(%d),curr_y(%d)",
				 __FUNCTION__, __LINE__, count_to_cell(s_origin_p.Y), count_to_cell(s_target_p.Y),
				 count_to_cell(s_curr_p.Y));
		ret = true;
	}
	else if (is_pos_dir ^ s_curr_p.Y < s_target_p.Y)
	{
		// Robot has reached the target line center but still not reach target line limit.
		// Check if the wall side has blocks on the map.
		auto dx = (is_pos_dir ^ mt_is_left()) ? +2 : -2;
		if (is_block_blocked(count_to_cell(s_curr_p.X) + dx, count_to_cell(s_curr_p.Y))) {
			ROS_WARN("%s %d: Already has block at the wall side, s_origin_p.Y(%d), target.Y(%d),curr_y(%d)",
					 __FUNCTION__, __LINE__, count_to_cell(s_origin_p.Y), count_to_cell(s_target_p.Y),
					 count_to_cell(s_curr_p.Y));
			ret = true;
		}
	}

	return ret;
}

bool FollowWallRegulator::isClosure(uint8_t closure_cnt)
{
	if (g_wf_reach_count >= closure_cnt) {
		ROS_WARN("%s %d: Trapped wall follow is loop closed. reach_count(%d) ", __FUNCTION__, __LINE__, g_wf_reach_count);
		return true;
	}
	return false;
}

bool FollowWallRegulator::isIsolate()
{
	return false;
}

bool FollowWallRegulator::isTimeUp()
{
	if (fw_is_time_up()) {
		ROS_WARN("%s %d: curr(%d),start(%d),diff(%d)",__FUNCTION__, __LINE__, time(NULL), g_wf_start_timer, g_wf_diff_timer);
		ev.fatal_quit = true;
		ROS_INFO("%s %d: curr(%d),start(%d),diff(%d)",__FUNCTION__, __LINE__, time(NULL), g_wf_start_timer, g_wf_diff_timer);
		ev.fatal_quit = true;
		return true;
	}

	return false;
}

bool FollowWallRegulator::shouldMoveBack()
{
	ev.bumper_triggered = bumper.get_status();
	if (ev.bumper_triggered) {
		g_turn_angle = bumper_turn_angle();
		ROS_WARN("%s %d: Bumper triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
	ev.cliff_triggered = cliff.get_status();
	if (ev.cliff_triggered) {
		g_turn_angle = cliff_turn_angle();
		ROS_WARN("%s %d: Cliff triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}
	ev.tilt_triggered = tilt.get_status();
	if (ev.tilt_triggered) {
		g_turn_angle = tilt_turn_angle();
		ROS_WARN("%s %d: Tilt triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	if(g_robot_slip)
	{
		// Temporary use obs as laser triggered.
		ev.obs_triggered = BLOCK_FRONT;
		g_turn_angle = obs_turn_angle();
		ROS_WARN("%s %d: slip triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	return false;
}

bool FollowWallRegulator::shouldTurn()
{
	ev.laser_triggered = get_laser_status();
	if (ev.laser_triggered)
	{
		// Temporary use bumper as laser triggered.
		ev.bumper_triggered = ev.laser_triggered;
		g_turn_angle = bumper_turn_angle();
		ev.bumper_triggered = 0;
		ROS_WARN("%s %d: Laser triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	ev.obs_triggered = (obs.get_front() > obs.get_front_trig_value() + 1700);
	if (ev.obs_triggered)
	{
		ev.obs_triggered = BLOCK_FRONT;
		g_turn_angle = obs_turn_angle();
		ROS_WARN("%s %d: OBS triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	return false;
}

bool FollowWallRegulator::isBlockCleared()
{
	if (!is_block_accessible(map_get_x_cell(), map_get_y_cell())) // Robot has step on blocks.
	{
		ROS_WARN("%s %d: Laser triggered, g_turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
		return true;
	}

	return false;
}

bool FollowWallRegulator::isOverOriginLine()
{
	auto curr = map_point_to_cell(s_curr_p);
	if ((s_target_p.Y > s_origin_p.Y && (s_origin_p.Y - s_curr_p.Y) > 120)
		|| (s_target_p.Y < s_origin_p.Y && (s_curr_p.Y - s_origin_p.Y) > 120))
	{
		auto target_angel = (s_target_p.Y > s_origin_p.Y) ? -900 : 900;
		//ROS_INFO("%s %d: target_angel(%d),curr(%d)diff(%d)", __FUNCTION__, __LINE__, target_angel, gyro_get_angle(), target_angel - gyro_get_angle());
		if (std::abs(ranged_angle(gyro_get_angle() - target_angel)) < 50) // If robot is directly heading to the opposite side of target line, stop.
		{
			ROS_WARN("%s %d: Opposite to target angle.", __FUNCTION__, __LINE__);
			return true;
		}
		else if (is_block_cleaned_unblock(curr.X, curr.Y)) // If robot covers a big block, stop.
		{
			ROS_WARN("%s %d: Back to cleaned place, current(%d, %d).", __FUNCTION__, __LINE__, curr.X, curr.Y);
			return true;
		}
		else{
			// Dynamic adjust the origin line and target line, so it can smoothly follow the wall to clean.
			s_target_p.Y += s_curr_p.Y - s_origin_p.Y;
			s_origin_p.Y = s_curr_p.Y;
		}
	}

	return false;
}

void FollowWallRegulator::setTarget()
{
	// No need to set target here, it is set in path_next().
}

void FollowWallRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	ROS_DEBUG("%s %d: FollowWallRegulator.", __FUNCTION__, __LINE__);
	wheel.set_dir_forward();
//	uint32_t same_dist = (wheel.get_right_step() / 100) * 11 ;
	uint32_t rcon_status = 0;
	auto _l_step = wheel.get_left_step();
	auto _r_step = wheel.get_right_step();
	auto &same_dist = (mt_is_left()) ? _l_step : _r_step;
	auto &diff_dist = (mt_is_left()) ? _r_step : _l_step;
	auto &same_speed = (mt_is_left()) ? l_speed : r_speed;
	auto &diff_speed = (mt_is_left()) ? r_speed : l_speed;
	wall_buffer[2]=wall_buffer[1];
	wall_buffer[1]=wall_buffer[0];
	wall_buffer[0]=(mt_is_left()) ? wall.getLeft() : wall.getRight();

	rcon_status = c_rcon.get_status();
	/*---only use a part of the Rcon signal---*/
	rcon_status &= (RconFL2_HomeT|RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT);
	if(rcon_status)
	{
//		ev.rcon_triggered = c_rcon.get_trig();
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
		c_rcon.reset_status();
		/*---check if should aloud the charger---*/
		if(seen_charger_counter)
		{
			same_speed = linear_speed + angular_speed;
			diff_speed = linear_speed - angular_speed;
			same_speed_ = same_speed;
			diff_speed_ = diff_speed;
			return ;
		}
	}
	if(seen_charger_counter)
	{
		seen_charger_counter--;
		same_speed = same_speed_;
		diff_speed = diff_speed_;
		return ;
	}

//	ROS_INFO("same_dist: %d < g_straight_distance : %d", same_dist, g_straight_distance);
	if (ros::Time::now().toSec() - time_start_straight < g_time_straight)
	{
		auto tmp = ros::Time::now().toSec() - time_start_straight;
		if(tmp < (g_time_straight / 3)) {
			if(same_speed < 8 )
				same_speed = diff_speed += 1;
			else
				same_speed = diff_speed = 8;
		}
		else if(tmp < (2 * g_time_straight / 3)) {
			if(same_speed < 8)
				same_speed = diff_speed = 8;
			if(same_speed < 13)
				same_speed = diff_speed += 1;
			else
				same_speed = diff_speed = 13;
		}
		else {
			if (same_speed < 13)
				same_speed = diff_speed = 13;
			if(same_speed < 18)
			same_speed = diff_speed += 1;
			else
				same_speed = diff_speed = 18;
		}
	}
	else
	{
		auto wheel_speed_base = 17 + diff_dist / 150;
		if (wheel_speed_base > 28)wheel_speed_base = 28;

		auto adc_value = (mt_is_left()) ? wall.getLeft() : wall.getRight();

		auto proportion = (adc_value - g_wall_distance) * 100 / g_wall_distance;

		auto delta = proportion - previous_;

		previous_ = proportion;
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

			if ((wall_buffer[1] - wall_buffer[0]) >= g_wall_distance / turn_right_angle_factor &&
					(wall_buffer[2] - wall_buffer[1]) >= g_wall_distance / turn_right_angle_factor &&
					same_dist > 200 &&
					(diff_speed-same_speed) >= -3) {
				is_right_angle = true;
			}
		}

		if(is_right_angle)
		{
			if(time_right_angle == 0) {
				time_right_angle = ros::Time::now().toSec();
				ROS_WARN("%s,%d: delay_sec(0.44) to walk straight", __FUNCTION__, __LINE__);
			}
			if(obs.is_wall_front() || is_map_front_block(3) ) {
				if(ros::Time::now().toSec() - time_right_angle < 0.4) {
					same_speed = 2 * 300 * (wall_follow_detect_distance - 0.167) + (20 - 15) / 2;
					diff_speed = 2 * 300 * (wall_follow_detect_distance - 0.167) - (20 - 15) / 2;
					return;
				}else {
					time_right_angle = 0;
					is_right_angle = 0;
				}
			}else {
				if(ros::Time::now().toSec() - time_right_angle < 0.44) {
					same_speed = 15;
					diff_speed = 15;
					return;
				}else{
					time_right_angle = 0;
					is_right_angle = 0;
					ROS_INFO("reset time_right_angle,is_right_angle");
				}
			}
		}
		/****************************************************END**************************************************************/
//		ROS_ERROR("same_speed:%d,diff_speed:%d",same_speed,diff_speed);

		if (same_speed > 39)same_speed = 39;
		if (same_speed < 0)same_speed = 0;
		if (diff_speed > 35)diff_speed = 35;
		if (diff_speed < 5)diff_speed = 5;

		if (obs.is_wall_front() || is_map_front_block(3)) {
//			ROS_WARN("decelarate");
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

		if(same_speed > diff_speed && diff_speed < (0.210 * same_speed)) {
			diff_speed = 0.210 * same_speed;
		}if(same_speed < diff_speed && same_speed < (0.210 * diff_speed)) {
			same_speed = 0.210 * diff_speed;
		}
	}
}

GoToChargerRegulator::GoToChargerRegulator()
{
	ROS_INFO("%s %d: Init", __FUNCTION__, __LINE__);
	go_home_state_now = GO_TO_CHARGER_INIT;
}

bool GoToChargerRegulator::isReach()
{
	if (ev.charge_detect)
		return true;
	return false;
}

bool GoToChargerRegulator::isChargerReach()
{
	if (ev.charge_detect)
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
		extern bool g_charge_turn_connect_fail;
		if(g_charge_turn_connect_fail && no_signal_cnt < 10)
		{
			receive_code = c_rcon.get_trig();
			ROS_INFO("%s, %d: check near home, receive_code: %8x", __FUNCTION__, __LINE__, receive_code);
			if(receive_code&RconAll_Home_T)
			{
				ROS_INFO("receive LR");
				if(receive_code&(RconFL_HomeT|RconFR_HomeT|RconFL2_HomeT|RconFR2_HomeT))
				{
					ROS_INFO("%s %d: turn 180", __FUNCTION__, __LINE__);
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 1800;
				}
				else if(receive_code&RconR_HomeT)
				{
					ROS_INFO("%s %d: turn left 90", __FUNCTION__, __LINE__);
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 900;
				}
				else if(receive_code&RconL_HomeT)
				{
					ROS_INFO("%s %d: turn right 90", __FUNCTION__, __LINE__);
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 900;
				}
				else //receive_code&RconBL_HomeT || receive_code&RconBR_HomeT
				{
					ROS_INFO("%s %d: go straight", __FUNCTION__, __LINE__);
					g_go_to_charger_back_0cm = true;
					g_turn_angle = 0;
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
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			resetGoToChargerVariables();
			g_charge_turn_connect_fail = false;
		}
	}
	if (go_home_state_now == AWAY_FROM_CHARGER_STATION)
	{
		ev.bumper_triggered = bumper.get_status();
		if(ev.bumper_triggered)
		{
			ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			g_turn_angle = 0;
			resetGoToChargerVariables();
			return true;
		}
		ev.cliff_triggered = cliff.get_status();
		if(ev.cliff_triggered)
		{
			ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			g_turn_angle = 0;
			resetGoToChargerVariables();
			return true;
		}
		if(move_away_from_charger_cnt++ > 50)
		{
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			resetGoToChargerVariables();
		}
	}
	if (go_home_state_now == TURN_FOR_CHARGER_SIGNAL_INIT)
	{
		resetGoToChargerVariables();
		g_go_to_charger_back_30cm = false;
		g_go_to_charger_back_10cm = false;
		g_go_to_charger_back_0cm = false;
		go_home_state_now = TURN_FOR_CHARGER_SIGNAL;
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
			ev.bumper_triggered = bumper.get_status();
			if(ev.bumper_triggered)
			{
				ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
				g_turn_angle = 0;
				resetGoToChargerVariables();
				return true;
			}
			ev.cliff_triggered = cliff.get_status();
			if(ev.cliff_triggered)
			{
				ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
				resetGoToChargerVariables();
				g_turn_angle = 0;
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
				return true;
			}

			// Handle for rcon signal
			receive_code = c_rcon.get_trig();
			if (receive_code)
			{
				go_home_state_now = AROUND_CHARGER_STATION_INIT;
			}
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
				g_go_to_charger_back_0cm = true;
				return true;
			}
		}
		// gyro_step > 360 is handled in GoToChargerRegulator::_isStop()
	}
	if (go_home_state_now == AROUND_CHARGER_STATION_INIT)
	{
		go_home_bumper_cnt = 0;
		//wheel.move_forward(9, 9);
		c_rcon.reset_status();
		ROS_INFO("%s, %d: Call Around_ChargerStation with dir = %d.", __FUNCTION__, __LINE__, around_charger_stub_dir);
		go_home_state_now = AROUND_CHARGER_STATION;
		around_move_cnt = 0;
	}
	else if (go_home_state_now == AROUND_CHARGER_STATION)
	{
		ev.cliff_triggered = cliff.get_status();
		if(ev.cliff_triggered)
		{
			ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
			g_turn_angle = 1750;
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}
		ev.bumper_triggered = bumper.get_status();
		if(ev.bumper_triggered)
		{
			ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
			around_charger_stub_dir = 1 - around_charger_stub_dir;
			g_turn_angle = 1800;
			if(++go_home_bumper_cnt > 1)
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}

		if (--around_move_cnt <= 0)
		{
			around_move_cnt = 7;
			receive_code = c_rcon.get_trig();
			if(receive_code)
				no_signal_cnt = 0;
			else if(++no_signal_cnt > 60)
			{
				ROS_WARN("%s %d:No charger signal received.", __FUNCTION__, __LINE__);
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			}

			//ROS_DEBUG("%s %d Check DIR: %d, and do something", __FUNCTION__, __LINE__, around_charger_stub_dir);
			if(around_charger_stub_dir == 1)//10.30
			{
				if(receive_code&(RconFR_HomeR|RconFL_HomeR))
				{
					go_home_state_now = BY_PATH_INIT;
					g_turn_angle = 0;
					if(receive_code&RconFR_HomeR)
						ROS_INFO("%s, %d: Detect FR-R, call By_Path().", __FUNCTION__, __LINE__);
					else if(receive_code&RconFL_HomeR)
						ROS_INFO("%s, %d: Detect FL-R, call By_Path().", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconL_HomeR)
				{
					ROS_INFO("%s, %d: Detect L-R, Check position left.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_LEFT;
					go_home_state_now = CHECK_POSITION_INIT;
					g_turn_angle = 0;
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
					g_go_to_charger_back_0cm = true;
					return true;
				}
			}
			else //around_charger_stub_dir == 0
			{
				if(receive_code&(RconFL_HomeL|RconFR_HomeL))
				{
					go_home_state_now = BY_PATH_INIT;
					g_turn_angle = 0;
					if(receive_code&(RconFL_HomeL))
						ROS_INFO("%s, %d: Detect FL-L, call By_Path().", __FUNCTION__, __LINE__);
					else if(receive_code&(RconFR_HomeL))
						ROS_INFO("%s, %d: Detect FR-L, call By_Path().", __FUNCTION__, __LINE__);
				}
				else if(receive_code&(RconR_HomeL))
				{
					ROS_INFO("%s, %d: Detect R-L, check position right.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_RIGHT;
					go_home_state_now = CHECK_POSITION_INIT;
					g_turn_angle = 0;
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
					g_go_to_charger_back_0cm = true;
					return true;
				}
			}
		}
	}
	if (go_home_state_now == CHECK_POSITION_INIT)
	{
		resetGoToChargerVariables();
		go_home_state_now = CHECK_POSITION;
	}
	if (go_home_state_now == CHECK_POSITION)
	{
		ev.bumper_triggered = bumper.get_status();
		if(ev.bumper_triggered)
		{
			ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
			around_charger_stub_dir = 1 - around_charger_stub_dir;
			if(++go_home_bumper_cnt > 1)
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			else
				go_home_state_now = AROUND_CHARGER_STATION_INIT;
			g_turn_angle = 1800;
			return true;
		}
		ev.cliff_triggered = cliff.get_status();
		if(ev.cliff_triggered)
		{
			ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			g_turn_angle = 1800;
			return true;
		}

		if(gyro_step < 360)
		{
			current_angle = robot::instance()->getAngle();
			angle_offset = static_cast<float>(ranged_angle((current_angle - last_angle) * 10)) / 10;
			ROS_DEBUG("%s %d: Current_Angle = %f, Last_Angle = %f, Angle_Offset = %f, Gyro_Step = %f.", __FUNCTION__, __LINE__, current_angle, last_angle, angle_offset, gyro_step);
			if (check_position_dir == ROUND_LEFT && angle_offset > 0)
				gyro_step += angle_offset;
			if (check_position_dir == ROUND_RIGHT && angle_offset < 0)
				gyro_step += (-angle_offset);
			last_angle = current_angle;

			ROS_DEBUG("%s %d: Check_Position c_rcon.get_status() == %8x, R... == %8x.", __FUNCTION__, __LINE__,
								c_rcon.get_status(), RconFrontAll_Home_LR);
			receive_code = (c_rcon.get_trig()&RconFrontAll_Home_LR);
			ROS_DEBUG("%s %d: receive code: %8x.", __FUNCTION__, __LINE__, receive_code);
			if(receive_code)
			{
				if (receive_code & RconL_HomeL)ROS_DEBUG("Check_Position get L-L");
				if (receive_code & RconL_HomeR)ROS_DEBUG("Check_Position get L-R");
				if (receive_code & RconFL_HomeL)ROS_DEBUG("Check_Position get FL-L");
				if (receive_code & RconFL_HomeR)ROS_DEBUG("Check_Position get FL-R");
				if (receive_code & RconFL2_HomeL)ROS_DEBUG("Check_Position get FL2-L");
				if (receive_code & RconFL2_HomeR)ROS_DEBUG("Check_Position get FL2-R");
				if (receive_code & RconR_HomeL)ROS_DEBUG("Check_Position get R-L");
				if (receive_code & RconR_HomeR)ROS_DEBUG("Check_Position get R-R");
				if (receive_code & RconFR_HomeL)ROS_DEBUG("Check_Position get FR-L");
				if (receive_code & RconFR_HomeR)ROS_DEBUG("Check_Position get FR-R");
				if (receive_code & RconFR2_HomeL)ROS_DEBUG("Check_Position get FR2-L");
				if (receive_code & RconFR2_HomeR)ROS_DEBUG("Check_Position get FR2-R");
			}

			if(receive_code & (RconFR_HomeL|RconFR_HomeR) && check_position_dir == ROUND_LEFT)
				go_home_state_now = BY_PATH_INIT;
			if(receive_code & (RconFL_HomeL|RconFL_HomeR) && check_position_dir == ROUND_RIGHT)
				go_home_state_now = BY_PATH_INIT;
		}
		if(gyro_step >= 360)
		{
			ROS_INFO("%s, %d: Robot can't see charger, restart go to charger process.", __FUNCTION__, __LINE__);
			g_turn_angle = 1000;
			g_go_to_charger_back_0cm = true;
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}
	}
	if (go_home_state_now == BY_PATH_INIT)
	{
		resetGoToChargerVariables();
		go_home_state_now = BY_PATH;
	}
	if (go_home_state_now == BY_PATH)
	{
		ev.bumper_triggered = bumper.get_status();
		if(ev.bumper_triggered)
		{
			ROS_INFO("bumper in by path!");
			if(!position_far)
			{
				go_home_state_now = TURN_CONNECT;
				turn_connect_dir = ROUND_RIGHT;
				turn_connect_cnt = 0;
			}
			else
			{
				//if((c_rcon.get_status()&RconFront_Home_LR) == 0)
				//	go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
				g_go_to_charger_back_10cm = true;
				//if(ev.bumper_triggered & BLOCK_LEFT)
				//	g_turn_angle = -1100;
				//else
				//	g_turn_angle = 1100;
				ROS_WARN("%d: quick_back in position_far", __LINE__);
				return true;
			}
		}
		ev.cliff_triggered = cliff.get_status();
		if(ev.cliff_triggered)
		{
			g_turn_angle = 1750;
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}

		if (--by_path_move_cnt < 0)
		{
			by_path_move_cnt = 25;
			receive_code = c_rcon.get_trig();
			if(receive_code)
			{
				if(receive_code&RconFR_HomeT && receive_code&RconFL_HomeT)
				{
					position_far = false;
					ROS_DEBUG("%s, %d: Robot face HomeT, position_far = false.", __FUNCTION__, __LINE__);
				}
				else if(receive_code&(RconFL2_HomeT|RconFR2_HomeT|RconL_HomeT|RconR_HomeT))
				{
					position_far = false;
					ROS_DEBUG("%s, %d: Robot side face HomeT, position_far = false.", __FUNCTION__, __LINE__);
				}
				if(receive_code&RconFrontAll_Home_T)
				{
					if(++near_counter > 1)
					{
						position_far = false;
						ROS_DEBUG("%s, %d: Robot near HomeT counter > 1, position_far = false.", __FUNCTION__, __LINE__);
					}
					else
						near_counter = 0;
					if((receive_code&RconFront_Home_LR) == 0 && ++side_counter > 5)
					{
						ROS_INFO("%s, %d: Robot away from the front of charger stub, back to gohome mode_.", __FUNCTION__, __LINE__);
						go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
						g_go_to_charger_back_0cm = true;
						g_turn_angle = 0;
						return true;
					}
					else
						side_counter = 0;
				}

				if(receive_code&RconFL_HomeL && receive_code&RconFR_HomeR)
				{
					ROS_DEBUG("%s, %d: Robot sees HomeL or HomeR, position_far = false.", __FUNCTION__, __LINE__);
					position_far = false;
				}

				auto temp_code = receive_code;
				temp_code &= RconFrontAll_Home_LR;
				if (temp_code)
				{
					g_go_to_charger_back_0cm = true;
					if(position_far)
					{
						switch(temp_code)
						{
							case (RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -350;
								return true;
							case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -400;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -450;
								return true;
							case (RconR_HomeR|RconR_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/R_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -550;
								return true;
							case (RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -500;
								return true;
							case (RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 350;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 400;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 450;
								return true;
							case (RconL_HomeR|RconL_HomeL):
								ROS_DEBUG("%s, %d: position_far, L_R/L_L.", __FUNCTION__, __LINE__);
								g_turn_angle = 500;
								return true;
							case (RconL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 550;
								return true;
							case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeR|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 500;
								return true;
						}
					}
					else
					{
						switch(temp_code)
						{
							case (RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -350;
								return true;
							case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -400;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -450;
								return true;
							case (RconR_HomeR|RconR_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/R_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -500;
								return true;
							case (RconR_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -550;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -500;
								return true;
							case (RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 350;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 400;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 450;
								return true;
							case (RconL_HomeR|RconL_HomeL):
								ROS_DEBUG("%s, %d: !position_far, L_R/L_L.", __FUNCTION__, __LINE__);
								g_turn_angle = 500;
								return true;
							case (RconL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeR|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 500;
								return true;
						}
					}
					g_go_to_charger_back_0cm = false;
				}
				no_signal_cnt = 0;
			}
			else
			{
				near_counter = 0;
				if(++no_signal_cnt > 1)
				{
					ROS_INFO("%s %d: No signal in by path, switch to check position.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_LEFT;
					go_home_state_now = CHECK_POSITION_INIT;
				}
			}
		}
	}
	if (go_home_state_now == TURN_CONNECT)
	{
		if (turn_connect_dir == ROUND_RIGHT && ++turn_connect_cnt > 50)
		{
			turn_connect_dir = ROUND_LEFT;
			turn_connect_cnt = 0;
		}
		if (turn_connect_dir == ROUND_LEFT && ++turn_connect_cnt > 50)
		{
			turn_connect_cnt = 0;
			g_go_to_charger_back_30cm = true;
			ROS_WARN("%s %d: Turn connect failed, move back for 0.1m.", __FUNCTION__, __LINE__);
			g_turn_angle = 0;
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}
	}

	return false;
}

bool GoToChargerRegulator::_isStop()
{
	if (g_robot_stuck || (go_home_state_now == TURN_FOR_CHARGER_SIGNAL && gyro_step > 360))
	{
		ROS_WARN("%s %d: Stop here", __FUNCTION__, __LINE__);
		cs_set(CS_CLEAN);
		return true;
	}
	return false;
}

void GoToChargerRegulator::setTarget()
{
	g_turn_angle = 0;
}

void GoToChargerRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	/*---check if near charger station---*/
	if (go_home_state_now == CHECK_NEAR_CHARGER_STATION)
	{
		wheel.set_dir_forward();
		l_speed = r_speed = 0;
	}
	else if (go_home_state_now == AWAY_FROM_CHARGER_STATION)
	{
		wheel.set_dir_forward();
		l_speed = r_speed = 30;
	}
	else if (go_home_state_now == TURN_FOR_CHARGER_SIGNAL)
	{
		wheel.set_dir_right();
		l_speed = r_speed = 10;
	}
	else if (go_home_state_now == AROUND_CHARGER_STATION_INIT)
	{
		wheel.set_dir_forward();
		l_speed = r_speed = 9;
		around_move_cnt = 0;
	}
	else if (go_home_state_now == AROUND_CHARGER_STATION)
	{
		if (around_move_cnt == 7 && around_charger_stub_dir == 1)
		{
			if(receive_code&RconL_HomeT)
			{
				ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 22;
				r_speed = 12;
			}
			else if(receive_code&RconL_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect L-L.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 22;
				r_speed = 12;
			}
			else if(receive_code&RconFL2_HomeT)
			{
				ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 23;
				r_speed = 14;
			}
			else if(receive_code&RconFL2_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect FL2-L.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 20;
				r_speed = 14;
			}
			else if(receive_code&RconFL2_HomeR)
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 15;
				r_speed = 21;
			}
			else
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 14;
				r_speed = 21;
			}
		}
		else if (around_move_cnt == 7 && around_charger_stub_dir == 0)
		{
			if(receive_code&RconR_HomeT)
			{
				ROS_DEBUG("%s %d Detect R-T.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 12;
				r_speed = 22;
			}
			else if(receive_code&RconR_HomeR)
			{
				ROS_DEBUG("%s %d Detect R-R.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 12;
				r_speed = 22;
			}
			else if(receive_code&RconFR2_HomeT)
			{
				ROS_DEBUG("%s %d Detect FR2-T.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 14;
				r_speed = 23;
			}
			else if(receive_code&RconFR2_HomeR)
			{
				ROS_DEBUG("%s %d Detect FR2-R.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 14;
				r_speed = 20;
			}
			else if(receive_code&RconFR2_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 21;
				r_speed = 15;
			}
			else
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				wheel.set_dir_forward();
				l_speed = 21;
				r_speed = 14;
			}
		}
		else
		{
			wheel.set_dir_forward();
			l_speed = left_speed_;
			r_speed = right_speed_;
		}
	}
	else if (go_home_state_now == CHECK_POSITION)
	{
		ROS_DEBUG("%s, %d: Check position dir: %d.", __FUNCTION__, __LINE__, check_position_dir);
		if(check_position_dir == ROUND_LEFT)
			wheel.set_dir_left();
		else if(check_position_dir == ROUND_RIGHT)
			wheel.set_dir_right();
		l_speed = r_speed = 10;
	}
	else if (go_home_state_now == BY_PATH)
	{
		wheel.set_dir_forward();
		auto temp_code = receive_code;
		temp_code &= RconFrontAll_Home_LR;
		if (by_path_move_cnt == 25 && temp_code)
		{
			if(position_far)
			{
				switch(temp_code)
				{
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 9;
						break;
					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 8;
						break;
					case (RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 8;
						break;
					case (RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FR_L.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 8;
						break;
					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 7;
						break;
					case (RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 7;
						break;
					case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 7;
						break;
					case (RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 6;
						break;
					case (RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 10;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 10;
						break;
					case (RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 7;
						break;
					case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 10;
						break;
					case (RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 8;
						break;
					case (RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 12;
						break;
					case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 2;
						break;
					case (RconR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 0;
						break;
					case (RconL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 1;
						break;
					case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 6;
						break;
					case (RconR_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 0;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 14;
						r_speed = 4;
						break;
					case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 13;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 13;
						break;
					case (RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 13;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 13;
						break;
					case (RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 14;
						break;
					case (RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 12;
						break;
					case (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL_L.", __FUNCTION__, __LINE__);
						l_speed = 2;
						r_speed = 9;
						break;
					case (RconL_HomeL|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 0;
						r_speed = 9;
						break;
					case (RconR_HomeR|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, R_R/FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 1;
						r_speed = 9;
						break;
					case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 9;
						break;
					case (RconL_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L.", __FUNCTION__, __LINE__);
						l_speed = 0;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					default:
						ROS_DEBUG("%s, %d: position_far, else:%x.", __FUNCTION__, __LINE__, temp_code);
						l_speed = 10;
						r_speed = 12;
				}
			}
			else
			{
				switch(temp_code)
				{
					case (RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL-L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 8;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 6;
						break;
					case (RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FR_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 4;
						break;
					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 3;
						break;
					case (RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 5;
						break;
					case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 3;
						break;
					case (RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 3;
						break;
					case (RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 7;
						break;
					case (RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 4;
						break;
					case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 7;
						break;
					case (RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 3;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 3;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 6;
						break;
					case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 2;
						break;
					case (RconR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 0;
						break;
					case (RconL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 1;
						break;
					case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 6;
						break;
					case (RconR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 0;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 2;
						break;
					case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R.", __FUNCTION__, __LINE__);
						l_speed = 4;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 5;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 8;
						break;
					case (RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 9;
						break;
					case (RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 9;
						break;
					case (RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 4;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 9;
						break;
					case (RconFL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 9;
						break;
					case (RconL_HomeL|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 0;
						r_speed = 9;
						break;
					case (RconR_HomeR|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_R/FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 1;
						r_speed = 9;
						break;
					case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 9;
						break;
					case (RconL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, L_L.", __FUNCTION__, __LINE__);
						l_speed = 0;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 2;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					default:
						ROS_DEBUG("%s, %d: !position_far, else:%x.", __FUNCTION__, __LINE__, temp_code);
						l_speed = 7;
						r_speed = 8;
						break;
				}
			}
		}
		else
		{
			wheel.set_dir_forward();
			l_speed = left_speed_;
			r_speed = right_speed_;
		}
	}
	else if (go_home_state_now == TURN_CONNECT)
	{
		if (turn_connect_dir == ROUND_RIGHT)
			wheel.set_dir_right();
		else if (turn_connect_dir == ROUND_LEFT)
			wheel.set_dir_left();
		l_speed = r_speed = 5;
	}
	left_speed_ = l_speed;
	right_speed_ = r_speed;
}

void SelfCheckRegulator::adjustSpeed(uint8_t bumper_jam_state)
{
	uint8_t left_speed;
	uint8_t right_speed;
	if (ev.oc_suction)
		left_speed = right_speed = 0;
	else if (ev.oc_wheel_left || ev.oc_wheel_right)
	{
		if (ev.oc_wheel_right) {
			wheel.set_dir_right();
		} else {
			wheel.set_dir_left();
		}
		left_speed = 30;
		right_speed = 30;
	}
	else if (ev.cliff_jam)
	{
		wheel.set_dir_backward();
		left_speed = right_speed = 18;
	}
	else if (ev.bumper_jam)
	{
		switch (bumper_jam_state)
		{
			case 1:
			case 2:
			case 3:
			{
				// Quickly move back for a distance.
				wheel.set_dir_backward();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 4:
			{
				// Quickly turn right for 90 degrees.
				wheel.set_dir_right();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
			case 5:
			{
				// Quickly turn left for 180 degrees.
				wheel.set_dir_left();
				left_speed = right_speed = RUN_TOP_SPEED;
				break;
			}
		}
	}
	else if(g_omni_notmove)
	{
		//wheel.set_dir_backward();
		//left_speed = right_speed = RUN_TOP_SPEED;
	}
	else if(g_slip_cnt>=2)
	{
		if(g_slip_cnt <3)
			wheel.set_dir_left();
		else if(g_slip_cnt <4)
			wheel.set_dir_right();
		left_speed = right_speed = ROTATE_TOP_SPEED;
	}
	else if (ev.laser_stuck)
	{
		wheel.set_dir_backward();
		left_speed = right_speed = 2;
	}

	wheel.set_speed(left_speed, right_speed);
}

