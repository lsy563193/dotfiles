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

int jam=0;

void cm_block_charger_stub()
{
	enum {left,right,fl,fr,fl2,fr2};
	ROS_WARN("%s %d: Robot meet charger stub, stop and mark the block.", __FUNCTION__, __LINE__);
	int dx=0,dy=0;
	int dx2=0,dy2=0;
	switch (g_rcon_triggered-1)
	{
		case left: dx = 1, dy = 2;
			break;
		case fl2: dx = 1,dy = 2; dx2 = 2,dy2 = 1;
			break;
		case fl:
		case fr: dx = 0, dy = 2;
			break;
		case fr2: dx = 1, dy = -2; dx2 = 2, dy2 = -1;
			break;
		case right: dx = -2, dy = 1;
			break;
	}
	if(get_clean_mode() != Clean_Mode_WallFollow)
	{
		map_set_cell(MAP, CELL_SIZE * dx, CELL_SIZE * dy, BLOCKED_BUMPER);
		if (dx2 != 0)
			map_set_cell(MAP, CELL_SIZE * dx2, CELL_SIZE * dy2, BLOCKED_BUMPER);
	}
	cm_set_home(map_get_x_count(), map_get_y_count());

}

static int16_t bumper_turn_angle()
{
	auto get_wheel_step = (mt_is_left()) ? get_right_wheel_step : get_left_wheel_step;
	auto get_obs = (mt_is_left()) ? get_left_obs : get_right_obs;
	auto get_obs_value = (mt_is_left()) ? get_left_obs_value : get_right_obs_value;
	auto status = get_bumper_status();
	auto diff_side = (mt_is_left()) ? RightBumperTrig : LeftBumperTrig;
	auto same_side = (mt_is_left()) ? LeftBumperTrig : RightBumperTrig;

	if (status == AllBumperTrig)
	{
		g_turn_angle = 850;
		g_straight_distance = 150; //150;
		g_wall_distance = std::min(g_wall_distance + 300, Wall_High_Limit);
		jam = get_wheel_step() < 2000 ? ++jam : 0;
	} else if (status == diff_side)
	{
		g_wall_distance = std::min(g_wall_distance + 300, Wall_High_Limit);
		g_turn_angle = 920;
	} else if (status == same_side)
	{
		g_wall_distance = std::max(g_wall_distance - 100, Wall_Low_Limit);

		g_turn_angle = (jam >= 3 || (g_wall_distance < 200 && get_obs() <= get_obs_value() - 200)) ? 200 : 300;

		g_wall_distance = (jam < 3 && g_wall_distance<200 && get_obs()>(get_obs_value() - 200))
											? Wall_High_Limit : g_wall_distance;

		g_straight_distance = 250; //250;
		jam = get_wheel_step() < 2000 ? ++jam : 0;
	}
	g_straight_distance = 200;
//	g_left_buffer = {0, 0, 0};
	reset_wheel_step();
//	ROS_INFO("705, g_turn_angle(%d), g_straight_distance(%d),g_wall_distance(%d),jam(%d),get_right_wheel_step(%d) ", g_turn_angle,     g_straight_distance,    g_wall_distance,    jam,    get_right_wheel_step()  );
	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

static int16_t cliff_turn_angle()
{
	g_turn_angle = 750;
	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

static int16_t obs_turn_angle()
{
	g_turn_angle = 920;
	if(mt_is_right())
		g_turn_angle = -g_turn_angle;
	return g_turn_angle;
}

static int16_t rcon_turn_angle()
{
	enum {left,right,fl,fr,fl2,fr2};
	int16_t left_angle[] =   {300,1100,850,850,600,950};
	int16_t right_angle[] =  {-1100, -300, -850, -850, -950, -600};
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
	bool is_found;
	double line_angle;
	const auto RESET_WALL_DIS = 100;
	is_found = MotionManage::s_laser->getLaserDistance(laser_min, laser_max, -1.0, dis_limit, &line_angle);
	ROS_INFO("line_angle_raw = %lf", line_angle);
	uint16_t angle = double_scale_10(line_angle);

	if (mt_is_right())
		angle  = 1800-angle;

	ROS_INFO("line_angle = %d", angle);
	if (is_found && angle >= angle_min && angle < angle_max)
	{
		g_turn_angle = angle;
		g_wall_distance = RESET_WALL_DIS;
		ROS_WARN("laser generate turn angle(%d)!",g_turn_angle);
	}
	return g_turn_angle;
}

static int16_t laser_turn_angle()
{
	stop_brifly();

	if (g_obs_triggered == 1)
	{
		ROS_INFO("front obs trigger");
		return _laser_turn_angle(90, 270, 450, 1800, 0.25);
	}
	else if(g_bumper_hitted != 0)
	{
		int angle_min, angle_max;
		if (mt_is_left() ^ g_bumper_hitted == LeftBumperTrig)
		{
			angle_min = 450;
			angle_max = 1800;
		}
		else
		{
			angle_min = 200;
			angle_max = 900;
		}

		if (g_bumper_hitted == AllBumperTrig)
			return _laser_turn_angle(90, 270, 900, 1800);
		else if (g_bumper_hitted == RightBumperTrig)
			return _laser_turn_angle(90, 180, angle_min, angle_max);
		else if (g_bumper_hitted == LeftBumperTrig)
			return _laser_turn_angle(180, 270, angle_min, angle_max);
	}

	return g_turn_angle;
}
static int16_t _get_obs_value()
{
	if(get_front_obs() > get_front_obs_value())
		return 1;
	if(get_left_obs() > get_left_obs_value())
		return 2;
	if(get_right_obs() > get_right_obs_value())
		return 3;
	return 0;
}

Point32_t RegulatorBase::s_target = {0,0};
Point32_t RegulatorBase::s_origin = {0,0};
int16_t RegulatorBase::s_angle = 0;
float RegulatorBase::s_pos_x = 0;
float RegulatorBase::s_pos_y = 0;

bool RegulatorBase::isExit(){
	return g_fatal_quit_event || g_key_clean_pressed;
}

bool RegulatorBase::_isStop()
{
//	ROS_INFO("reg_base _isStop");
	return g_battery_home || (!g_go_home && g_remote_home) || cm_should_self_check();
}


BackRegulator::BackRegulator() : speed_(8), counter_(0)
{
//	ROS_WARN("%s, %d: ", __FUNCTION__, __LINE__);
}

bool BackRegulator::isReach()
{
	auto distance = sqrtf(powf(s_pos_x - robot::instance()->getOdomPositionX(), 2) +
			   	powf(s_pos_y - robot::instance()->getOdomPositionY(), 2));
	if(fabsf(distance) > 0.02f){
//		ROS_WARN("%s, %d: BackRegulator ");
		g_bumper_cnt = get_bumper_status() == 0 ? 0 : g_bumper_cnt+1 ;
		g_cliff_cnt = get_bumper_status() != 0 ? 0 : g_cliff_cnt+1 ;

		if((g_bumper_cnt == 0 && g_cliff_cnt == 0) || g_bumper_cnt >= 3 || g_cliff_cnt >= 3)
			return true;
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
	return false;
}

void BackRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
//	ROS_INFO("BackRegulator::adjustSpeed");
	set_dir_backward();
	speed_ += counter_ / 100;
	speed_ = (speed_ > 18) ? 18 : speed_;
	reset_wheel_step();
	l_speed = r_speed = speed_;
}


TurnRegulator::TurnRegulator(int16_t angle) : speed_max_(13)
{
//	ROS_WARN("%s, %d: ", __FUNCTION__, __LINE__);
	accurate_ = speed_max_ > 30 ? 30 : 10;
	s_angle = angle;
}

bool TurnRegulator::isReach()
{
	if (abs(s_angle - gyro_get_angle()) < accurate_){
		ROS_WARN("%s, %d: TurnRegulator target,curr (%d,%d)", __FUNCTION__, __LINE__,s_angle, gyro_get_angle());
		return true;
	}

	return false;
}

bool TurnRegulator::isSwitch()
{
//	ROS_INFO("TurnRegulator::isSwitch");
	if(isReach() ||(! g_bumper_hitted  && get_bumper_status()) || (! g_cliff_triggered && get_cliff_trig()))
	{
		g_bumper_hitted = get_bumper_status();
		g_cliff_triggered = get_cliff_trig();
		reset_sp_turn_count();
		reset_wheel_step();
		return true;
	}
	return false;
}

bool TurnRegulator::_isStop()
{
	return false;
}

void TurnRegulator::setTarget()
{
	s_angle = ranged_angle(gyro_get_angle() + laser_turn_angle());
}

void TurnRegulator::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{

	auto diff = ranged_angle(s_angle - gyro_get_angle());
//	ROS_WARN("TurnRegulator::adjustSpeed diff(%d)", diff);
	if(mt_is_follow_wall())
		(mt_is_left()) ? set_dir_right() : set_dir_left();
	else
	{
		if ((diff >= 0) && (diff <= 1800))
			set_dir_left();
		else if ((diff <= 0) && (diff >= (-1800)))
			set_dir_right();
	}

//	ROS_INFO("TurnRegulator::adjustSpeed");
	auto speed = speed_max_;

	if (std::abs(diff) < 50)
	{
		speed = std::min((uint16_t) 5, speed);
	} else if (std::abs(diff) < 200)
	{
		speed = std::min((uint16_t) 10, speed);
	}
	l_speed = r_speed = speed;
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


LinearRegulator::LinearRegulator(Point32_t target):
				speed_max_(40),integrated_(0),base_speed_(BASE_SPEED),integration_cycle_(0),tick_(0),turn_speed_(4)
{
	s_target = target;
	g_turn_angle = ranged_angle(
					course_to_dest(map_get_x_count(), map_get_y_count(), s_target.X, s_target.Y) - gyro_get_angle());
	ROS_WARN("%s %d: angle(%d),curr(%d,%d),targ(%d,%d) ", __FUNCTION__, __LINE__, g_turn_angle, map_get_x_count(),map_get_y_count(), s_target.X,s_target.Y);
	ROS_ERROR("angle:%d(%d,%d) ", g_turn_angle,
						course_to_dest(map_get_x_count(), map_get_y_count(), s_target.X, s_target.Y), gyro_get_angle());
}

bool LinearRegulator::isReach()
{
	if (std::abs(map_get_x_count() - s_target.X) < 150 && std::abs(map_get_y_count() - s_target.Y) < 150)
	{
		ROS_INFO("%s, %d: LinearRegulator.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

bool LinearRegulator::isSwitch()
{
	if ((! g_bumper_hitted && get_bumper_status())
			|| (! g_cliff_triggered && get_cliff_trig()))
	{
		ROS_WARN("%s, %d:LinearRegulator g_bumper_hitted || g_cliff_triggered.", __FUNCTION__, __LINE__);
		g_bumper_hitted = get_bumper_status();
		g_cliff_triggered = get_cliff_trig();

		mt_set(CM_FOLLOW_LEFT_WALL);
		g_turn_angle = bumper_turn_angle();
		mt_set(CM_LINEARMOVE);

		return true;
	}

	return false;
}

bool LinearRegulator::_isStop()
{
	if (_get_obs_value() || get_rcon_status())
	{
		if(get_rcon_status()) cm_block_charger_stub();

		ROS_WARN("%s, %d: g_obs_triggered || g_rcon_triggered.", __FUNCTION__, __LINE__);
		SpotType spt = SpotMovement::instance()->getSpotType();
		if (spt == CLEAN_SPOT || spt == NORMAL_SPOT)
			SpotMovement::instance()->setDirectChange();
		return true;
	}

	if (is_map_front_block(2))
	{
		ROS_WARN("%s, %d: Blocked boundary.", __FUNCTION__, __LINE__);
		return true;
	}

	auto diff = ranged_angle(
					course_to_dest(map_get_x_count(), map_get_y_count(), s_target.X, s_target.Y) - gyro_get_angle());
	if ( std::abs(diff) > 300)
	{
		ROS_WARN("%s %d: warning: angle is too big, angle: %d", __FUNCTION__, __LINE__, diff);
		return true;
	}

	if(g_remote_spot)
	{
		ROS_WARN("%s, %d: g_remote_spot.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

void LinearRegulator::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
//	ROS_INFO("BackRegulator::adjustSpeed");
	set_dir_forward();

	auto diff = ranged_angle(
					course_to_dest(map_get_x_count(), map_get_y_count(), s_target.X, s_target.Y) - gyro_get_angle());

//	ROS_WARN("%s %d: angle %d(%d,%d) ", __FUNCTION__, __LINE__, diff,course_to_dest(map_get_x_count(), map_get_y_count(), s_target.X, s_target.Y),gyro_get_angle());
	if (integration_cycle_++ > 10)
	{
		integration_cycle_ = 0;
		integrated_ += diff;
		check_limit(integrated_, -150, 150);
	}

	auto distance = two_points_distance(map_get_x_count(), map_get_y_count(), s_target.X, s_target.Y);
	auto obstcal_detected = MotionManage::s_laser->laserObstcalDetected(0.2, 0, -1.0);

	if (get_obs_status() || is_obs_near() || (distance < SLOW_DOWN_DISTANCE) || is_map_front_block(3) || obstcal_detected)
	{
		integrated_ = 0;
		diff = 0;
		base_speed_ -= 1;
		base_speed_ = base_speed_ < BASE_SPEED ? BASE_SPEED : base_speed_;
	} else
	if (base_speed_ < (int32_t) speed_max_)
	{
		if (tick_++ > 5)
		{
			tick_ = 0;
			base_speed_++;
		}
		integrated_ = 0;
	}

	left_speed = base_speed_ - diff / 20 - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
	right_speed = base_speed_ + diff / 20 + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;

	check_limit(left_speed, BASE_SPEED, speed_max_);
	check_limit(right_speed, BASE_SPEED, speed_max_);
	base_speed_ = (left_speed + right_speed) / 2;
//	ROS_ERROR("left_speed(%d),right_speed(%d), base_speed_(%d), slow_down(%d),diff(%d)",left_speed, right_speed, base_speed_, is_map_front_block(3),diff);
}


FollowWallRegulator::FollowWallRegulator(Point32_t origin, Point32_t target) : previous_(0)
{
	g_wall_distance = 400;
	g_straight_distance = 300;
	s_origin = origin;
	s_target = target;
	ROS_WARN("%s, %d: ", __FUNCTION__, __LINE__);
}

bool FollowWallRegulator::isReach()
{
//	ROS_INFO("FollowWallRegulator isReach");
//	ROS_INFO("target_(%d,%d)",s_target.X,s_target.Y);
	bool ret = false;
	auto start_y = s_origin.Y;
	if (get_clean_mode() == Clean_Mode_WallFollow)
	{
		if (wf_is_end())
		{
			//wf_break_wall_follow();
			ret = true;
		}
	} else if (get_clean_mode() == Clean_Mode_Navigation)
	{

		extern int g_trapped_mode;
		if (g_trapped_mode != 0)
		{
			extern uint32_t g_escape_trapped_timer;
			if (g_trapped_mode == 2 || (time(NULL) - g_escape_trapped_timer) > ESCAPE_TRAPPED_TIME)
			{
//				wav_play(WAV_CLEANING_START);
//				g_trapped_mode = 0;
				ret = true;
			}
		} else
		{
			if ((start_y < s_target.Y ^ map_get_y_count() < s_target.Y))
			{
				ROS_WARN("%s, %d: BackRegulator ");
				ROS_WARN("Robot has reach the target.");
				ROS_WARN("%s %d:start_y(%d), target.Y(%d),curr_y(%d)", __FUNCTION__, __LINE__, start_y, s_target.Y,
								 map_get_y_count());
//			if(s_origin.X == map_get_x_count() && s_origin.Y == map_get_y_count()){
//				ROS_WARN("direcition is wrong, swap");
//				extern uint16_t g_last_dir;
//				g_last_dir = (g_last_dir == POS_X) ? NEG_X : POS_X;
//			}
				ret = true;
			}

			if ((s_target.Y > start_y && (start_y - map_get_y_count()) > 120) ||
					(s_target.Y < start_y && (map_get_y_count() - start_y) > 120))
			{
				ROS_WARN("%s, %d: BackRegulator ");
				ROS_WARN("Robot has round to the opposite direcition.");
				ROS_WARN("%s %d:start_y(%d), target.Y(%d),curr_y(%d)", __FUNCTION__, __LINE__, start_y, s_target.Y,
								 map_get_y_count());
				map_set_cell(MAP, map_get_relative_x(gyro_get_angle(), CELL_SIZE_3, 0),
										 map_get_relative_y(gyro_get_angle(), CELL_SIZE_3, 0), CLEANED);

//			if(s_origin.X == map_get_x_count() && s_origin.Y == map_get_y_count()){
//				ROS_WARN("direcition is wrong, swap");
//				extern uint16_t g_last_dir;
//				g_last_dir = (g_last_dir == POS_X) ? NEG_X : POS_X;
//			}
				ret = true;
			}
		}
	}
	return ret;
}

bool FollowWallRegulator::isSwitch()
{
//	ROS_INFO("FollowWallRegulator isSwitch");
	if(! g_bumper_hitted && get_bumper_status()){
		g_bumper_hitted = get_bumper_status();
		g_turn_angle = bumper_turn_angle();
		return true;
	}
	if(! g_cliff_triggered && get_cliff_trig()){
		g_cliff_triggered = get_cliff_trig();
		g_turn_angle = cliff_turn_angle();
		return true;
	}
	if(! g_rcon_triggered && get_rcon_trig()){
		g_rcon_triggered = get_rcon_status();
		g_turn_angle = rcon_turn_angle();
		g_straight_distance = 80;
		return true;
	}
	if(! g_obs_triggered  && get_front_obs() >= get_front_obs_value()){
		g_obs_triggered = 1;
		g_turn_angle = obs_turn_angle();
		g_wall_distance = Wall_High_Limit;
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
	ROS_INFO("FollowWallRegulator::adjustSpeed");
	set_dir_forward();
//	uint32_t same_dist = (get_right_wheel_step() / 100) * 11 ;
	auto _l_step = get_left_wheel_step();
	auto _r_step = get_right_wheel_step();
	auto &same_dist = (mt_is_left()) ? _l_step : _r_step;
	auto &diff_dist = (mt_is_left()) ? _r_step : _l_step;
	auto &same_speed = (mt_is_left()) ? l_speed : r_speed;
	auto &diff_speed = (mt_is_left()) ? r_speed : l_speed;
//	ROS_INFO("FollowWallRegulator adjustSpeed");
//	ROS_INFO("same_dist: %d < g_straight_distance : %d", same_dist, g_straight_distance);
	if ((same_dist) < (uint32_t) g_straight_distance)
	{
		int32_t speed;
		if (same_dist < 500)
			speed = (same_dist < 100)? 10 :15;
		else
			speed = 23;
		same_speed = diff_speed = speed;
//		ROS_WARN("same_dist: %d < g_straight_distance : %d", same_dist, g_straight_distance);
//		ROS_INFO("Wf_1, speed = %d, g_wall_distance = %d", speed, g_wall_distance, g_wall_distance);
	}
	else
	{
		auto wheel_speed_base = 15 + diff_dist / 150;
		if (wheel_speed_base > 28)wheel_speed_base = 28;

		auto proportion = (mt_is_left()) ? robot::instance()->getLeftWall() : robot::instance()->getRightWall();

		proportion = proportion * 100 / g_wall_distance;

		proportion -= 100;

		auto delta = proportion - previous_;

		if (g_wall_distance > 200)
		{//over left

			same_speed = wheel_speed_base + proportion / 8 + delta / 3; //12
			diff_speed = wheel_speed_base - proportion / 9 - delta / 3; //10

			if (wheel_speed_base < 26)
			{
				reset_sp_turn_count();
				if (diff_speed > wheel_speed_base + 6)
				{
					diff_speed = 34;
					same_speed = 4;
//						ROS_INFO("Wf_2, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
				}
				else if (same_speed > wheel_speed_base + 10)
				{
					diff_speed = 5;
					same_speed = 30;
//						ROS_INFO("Wf_3, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
				}
			}
			else
			{
				if (diff_speed > 35)
				{
					add_sp_turn_count();
					diff_speed = 35;
					same_speed = 4;
//						ROS_INFO("Wf_4, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
//						ROS_WARN("get_sp_turn_count() = %d", get_sp_turn_count());
				}
				else
				{
					reset_sp_turn_count();
				}
			}
		}
		else
		{

			same_speed = wheel_speed_base + proportion / 10 + delta / 3;//16
			diff_speed = wheel_speed_base - proportion / 10 - delta / 4; //11

			if (wheel_speed_base < 26)
			{
				reset_sp_turn_count();
				if (diff_speed > wheel_speed_base + 4)
				{
					diff_speed = 34;
					same_speed = 4;
//						ROS_INFO("Wf_5, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
				}
			}
			else
			{
				if (diff_speed > 32)
				{
					add_sp_turn_count();
					diff_speed = 36;
					same_speed = 4;
//						ROS_INFO("Wf_6, same_speed = %d, diff_speed = %d, g_wall_distance = %d", same_speed, diff_speed, g_wall_distance);
//						ROS_WARN("g_sp_turn_count() = %d",get_sp_turn_count());
				}
				else
				{
					reset_sp_turn_count();
				}
			}
		}

		previous_ = proportion;

		if (same_speed > 39)same_speed = 39;
		if (same_speed < 0)same_speed = 0;
		if (diff_speed > 35)diff_speed = 35;
		if (diff_speed < 5)diff_speed = 5;
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
	else //if (g_bumper_jam)
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

	set_wheel_speed(left_speed, right_speed);
}


//RegulatorManage
RegulatorProxy::RegulatorProxy(Point32_t origin, Point32_t target)
{
	g_bumper_cnt = g_cliff_cnt =0;
	reset_rcon_status();

	back_reg_ = new BackRegulator();

	if (mt_is_follow_wall())
		mt_reg_ = new FollowWallRegulator(origin, target);
	else
		mt_reg_ = new LinearRegulator(target);

	turn_reg_ = new TurnRegulator(ranged_angle(gyro_get_angle() + g_turn_angle));
	p_reg_ = turn_reg_;

	robotbase_obs_adjust_count(50);
	cm_set_event_manager_handler_state(true);

}

RegulatorProxy::~RegulatorProxy()
{
	delete turn_reg_;
	delete back_reg_;
	delete mt_reg_;
	set_wheel_speed(0,0);
	cm_set_event_manager_handler_state(false);
}
void RegulatorProxy::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	if (p_reg_ != nullptr)
		p_reg_->adjustSpeed(left_speed, right_speed);
}

bool RegulatorProxy::isReach()
{
	if ( (mt_is_linear() && p_reg_ == back_reg_) || (mt_is_follow_wall() && p_reg_ == mt_reg_) )
		return p_reg_->isReach();
	return false;
}

bool RegulatorProxy::isSwitch()
{
	if (p_reg_ != nullptr)
		return p_reg_->isSwitch();
	return false;
}

bool RegulatorProxy::_isStop()
{
	if (p_reg_ != nullptr)
		return p_reg_->_isStop();
	return false;
}

void RegulatorProxy::switchToNext()
{
	if (p_reg_ == turn_reg_)
	{
		if(g_bumper_hitted || g_cliff_triggered)
			p_reg_ = back_reg_;
		else /*if(g_obs_triggered || g_rcon_triggered)*/
			p_reg_ = mt_reg_;
	}
	else if (p_reg_ == back_reg_)
		p_reg_ = turn_reg_;
	else if (p_reg_ == mt_reg_)
	{
		if (g_obs_triggered || g_rcon_triggered)
			p_reg_ = turn_reg_;
		else if (g_bumper_hitted || g_cliff_triggered)
			p_reg_ = back_reg_;
	}
	g_obs_triggered = g_rcon_triggered = 0;
	g_bumper_hitted = g_cliff_triggered = 0;
	setTarget();
}
