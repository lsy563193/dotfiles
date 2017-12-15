//
// Created by lsy563193 on 12/14/17.
//
#include "ros/ros.h"
#include <cstdint>
#include <mathematics.h>
#include <deque>
#include <pp.h>
#include <error.h>
Cell_t g_zero_home;
bool	g_start_point_seen_charger;
bool g_have_seen_charger;
std::deque<Cell_t> g_passed_path;
bool g_move_back_finished;
static uint8_t g_cleaning_mode = 0;
bool line_is_found;
bool g_motion_init_succeeded;
bool g_resume_cleaning;
//boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;
//boost::shared_ptr<IMoveType> ACleanMode::sp_move_type_ = nullptr;
uint32_t g_wf_start_timer;
uint32_t g_wf_diff_timer;
int g_wf_reach_count;
//SpotMovement::instance()
double g_time_straight;
int g_wall_distance;
int16_t g_turn_angle;
bool g_allow_check_path_in_advance;
bool g_check_path_in_advance;
bool g_slip_backward;
time_t last_time_remote_spot = time(NULL);
std::vector<Cell_t> g_homes;
std::vector<int> g_home_way_list;
std::vector<int>::iterator g_home_way_it;
bool g_go_home_by_remote = false;
Cell_t g_home_point;

std::deque <Cell_t> path_points;
double bumper_turn_factor = 0.85;

void path_display_path_points(const std::deque<Cell_t>& path)
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": Path size(" + std::to_string(path.size()) + "):";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ", " + std::to_string(it->TH) + ")->";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}
bool fw_is_time_up()
{
	return ((uint32_t)difftime(time(NULL), g_wf_start_timer)) > g_wf_diff_timer;
}

void cm_set(uint8_t mode)
{
	g_cleaning_mode = mode;
}

bool cm_is_go_charger()
{
	return g_cleaning_mode == Clean_Mode_Go_Charger;
}

uint8_t cm_get()
{
	return g_cleaning_mode;
}

bool is_fobbit_free() {
	//NOTE: g_home_way_it should last of g_home_point,for g_homeway_list may empty.
	return (/*cs.is_going_home() &&*/ *g_home_way_it % HOMEWAY_NUM == USE_CLEANED);
}


bool cm_should_self_check(void)
{
	return (ev.oc_wheel_left || ev.oc_wheel_right || ev.bumper_jam || ev.cliff_jam || ev.oc_suction || g_slip_cnt >= 2 || ev.lidar_stuck);
}

void path_set_home(const Cell_t& curr)
{
	bool is_found = false;

	for (const auto& it : g_homes) {
		ROS_INFO("%s %d: curr\033[33m(%d, %d)\033[0m home_it\033[33m(%d,%d)\033[0m.", __FUNCTION__, __LINE__, curr.X, curr.Y,it.X,it.Y);
		if (it == curr) {
			is_found = true;
			break;
		}
	}
	if (!is_found) {
		ROS_INFO("%s %d: Push new reachable home:\033[33m (%d, %d)\033[0m to home point list.", __FUNCTION__, __LINE__, curr.X, curr.Y);
		if(cm_get() != Clean_Mode_Spot)
			g_have_seen_charger = true;
		// If curr near (0, 0)
		if (abs(curr.X) >= 5 || abs(curr.Y) >= 5)
		{
			if(g_homes.size() >= ESCAPE_TRAPPED_REF_CELL_SIZE+1)//escape_count + zero_home = 3+1 = 4
			{
				std::copy(g_homes.begin() + 2, g_homes.end(), g_homes.begin()+1);//shift 1 but save zero_home
				g_homes.pop_back();
			}
			g_homes.push_back(curr);
		}
	}
	else if(curr == g_zero_home && cm_get() != Clean_Mode_Spot)
	{
		g_start_point_seen_charger = true;
		g_have_seen_charger = true;
	}
}

uint8_t cs_self_check(uint8_t Check_Code)
{
	static time_t mboctime;
	static time_t vacoctime;
	static uint8_t mbrushchecking = 0;
	uint8_t Time_Out = 0;
	int32_t Wheel_Current_Summary = 0;
	uint8_t Left_Wheel_Slow = 0;
	uint8_t Right_Wheel_Slow = 0;

/*
	if(cm_is_navigation())
		cm_move_back_(COR_BACK_20MM);
	else
		quick_back(30,20);
*/
	cs_disable_motors();
	usleep(10000);
	/*------------------------------Self Check right wheel -------------------*/
	if (Check_Code == Check_Right_Wheel)
	{
		Right_Wheel_Slow = 0;
		if (wheel.getDirection() == DIRECTION_LEFT)
		{
			wheel.setDirectionRight();
		} else
		{
			wheel.setDirectionLeft();
		}
		wheel.setPidTargetSpeed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) wheel.getRightWheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			cs_disable_motors();
			ROS_WARN("%s,%d right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
			error.set(ERROR_CODE_RIGHTWHEEL);
			error.alarm();
			return 1;

		}
		/*
		if(Right_Wheel_Slow>100)
		{
			cs_disable_motors();
			error.set(Error_Code_RightWheel);
			return 1;
		}
		*/
		wheel.stop();
		//turn_right(Turn_Speed,1800);
	}
		/*---------------------------Self Check left wheel -------------------*/
	else if (Check_Code == Check_Left_Wheel)
	{
		Left_Wheel_Slow = 0;
		if (wheel.getDirection() == DIRECTION_RIGHT)
		{
			wheel.setDirectionLeft();
		} else
		{
			wheel.setDirectionRight();
		}
		wheel.setPidTargetSpeed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) wheel.getLeftWheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			cs_disable_motors();
			ROS_WARN("%s %d,left wheel stall maybe, please check!!", __FUNCTION__, __LINE__);
			error.set(ERROR_CODE_LEFTWHEEL);
			error.alarm();
			return 1;
		}
		/*
		if(Left_Wheel_Slow>100)
		{
			cs_disable_motors();
			error.set(ERROR_CODE_RIGHTWHEEL);
			return 1;
		}
		*/
		wheel.stop();
		//turn_left(Turn_Speed,1800);
	} else if (Check_Code == Check_Main_Brush)
	{
		if (!mbrushchecking)
		{
			brush.setMainPwm(0);
			mbrushchecking = 1;
			mboctime = time(NULL);
		} else if ((uint32_t) difftime(time(NULL), mboctime) >= 3)
		{
			mbrushchecking = 0;
			error.set(ERROR_CODE_MAINBRUSH);
			cs_disable_motors();
			error.alarm();
			return 1;
		}
		return 0;
	} else if (Check_Code == Check_Vacuum)
	{
#ifndef BLDC_INSTALL
		ROS_INFO("%s, %d: Vacuum Over Current!!", __FUNCTION__, __LINE__);
		ROS_INFO("%d", vacuum.getSelfCheckStatus());
		while (vacuum.getSelfCheckStatus() != 0x10)
		{
			/*-----wait until self check begin-----*/
			vacuum.startSelfCheck();
		}
		ROS_INFO("%s, %d: Vacuum Self checking", __FUNCTION__, __LINE__);
		/*-----reset command for start self check-----*/
		vacuum.resetSelfCheck();
		/*-----wait for the end of self check-----*/
		while (vacuum.getSelfCheckStatus() == 0x10);
		ROS_INFO("%s, %d: end of Self checking", __FUNCTION__, __LINE__);
		if (vacuum.getSelfCheckStatus() == 0x20)
		{
			ROS_INFO("%s, %d: Vacuum error", __FUNCTION__, __LINE__);
			/*-----vacuum error-----*/
			error.set(ERROR_CODE_FAN_H);
			cs_disable_motors();
			error.alarm();
			vacuum.resetSelfCheck();
			return 1;
		}
		vacuum.resetSelfCheck();
#else
		Disable_Motors();
		//wheel.stop();
		Set_Vac_Speed();
		usleep(100000);
		vacoctime = time(NULL);
		uint16_t tmpnoc_n = 0;
		while((uint32_t)difftime(time(NULL),vacoctime)<=3){
			if(!robot::instance()->robot_get_vacuum_oc()){
				tmpnoc_n++;
				if(tmpnoc_n>20){
					Work_Motor_Configure();
					tmpnoc_n = 0;
					return 0;
				}
			}
			usleep(50000);
		}
		error.set(Error_Code_Fan_H);
		cs_disable_motors();
		Alarm_Error();
		return 1;
#endif
	} else if (Check_Code == Check_Left_Brush)
	{
		error.set(ERROR_CODE_LEFTBRUSH);
		cs_disable_motors();
		error.alarm();
		return 1;
	} else if (Check_Code == Check_Right_Brush)
	{
		error.set(ERROR_CODE_RIGHTBRUSH);
		cs_disable_motors();
		error.alarm();
		return 1;
	}
	wheel.stop();
	Left_Wheel_Slow = 0;
	Right_Wheel_Slow = 0;
	cs_work_motor(false);
	//wheel.moveForward(5,5);
	return 0;
}

void cs_disable_motors(void)
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
}


void cs_work_motor(bool st_is_go_home)
{
	if (st_is_go_home)
	{
		// Set the vacuum to a normal mode
		vacuum.setMode(Vac_Normal, false);
		// Turn on the main brush and side brush
		brush.setSidePwm(30, 30);
	} else {
		vacuum.setMode(Vac_Save);
		// Turn on the main brush and side brush
		brush.setSidePwm(50, 50);
	}

	brush.setMainPwm(30);
}


bool check_pub_scan()
{
	//ROS_INFO("%s %d: get_left_wheel.speed() = %d, get_right_wheel.speed() = %d.", __FUNCTION__, __LINE__, wheel.getLeftSpeedAfterPid(), wheel.getRightSpeedAfterPid());
	if (g_motion_init_succeeded &&
		((fabs(wheel.getLeftWheelActualSpeed() - wheel.getRightWheelActualSpeed()) > 0.1)
		|| (wheel.getLeftWheelActualSpeed() * wheel.getRightWheelActualSpeed() < 0)
		|| bumper.get_status() || gyro.getTiltCheckingStatus()
		|| abs(wheel.getLeftSpeedAfterPid() - wheel.getRightSpeedAfterPid()) > 100
		|| wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() < 0))
		return false;
	else
		return true;
}

