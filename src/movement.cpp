#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <time.h>
#include <fcntl.h>
#include <motion_manage.h>
#include <move_type.h>
#include <ctime>
#include <clean_state.h>
#include <vacuum.h>
#include <cliff.h>
#include <brush.h>
#include <bumper.h>
#include <controller.h>
#include <obs.h>

#include "gyro.h"
#include "key.h"
#include "robot.hpp"
#include "movement.h"
#include "crc8.h"
#include "serial.h"
#include "robotbase.h"
#include "config.h"
#include "core_move.h"
#include "wall_follow_trapped.h"
#include "wav.h"
#include "slam.h"
#include "event_manager.h"
#include "laser.hpp"
#include "clean_mode.h"


static int16_t g_leftwall_obs_trig_vale = 500;
uint8_t g_wheel_left_direction = FORWARD;
uint8_t g_wheel_right_direction = FORWARD;
uint32_t g_average_move = 0;
uint32_t g_average_counter = 0;
uint32_t g_max_move = 0;
uint32_t g_auto_work_time = 2800;
uint32_t g_room_work_time = 3600;
uint8_t g_room_mode = 0;
uint8_t g_sleep_mode_flag = 0;

static uint32_t g_wall_accelerate = 0;
static int16_t g_left_wheel_speed = 0;
static int16_t g_right_wheel_speed = 0;
static int32_t g_left_wheel_step = 0;
static int32_t g_right_wheel_step = 0;
static uint32_t g_leftwall_step = 0;
static uint32_t g_rightwall_step = 0;

static int32_t g_move_step_counter = 0;
static uint32_t g_mobility_step = 0;
static uint8_t g_direction_flag = 0;
// Variable for vacuum mode_

//static uint8_t g_cleaning_mode = 0;
ros::Time g_lw_t, g_rw_t; // these variable is used for calculate wheel step

volatile uint8_t g_r_h_flag = 0;

// Value for wall sensor offset.
volatile int16_t g_left_wall_baseline = 50;
volatile int16_t g_right_wall_baseline = 50;

// Variable for stop event status.
volatile uint8_t g_stop_event_status = 0;

// Error code for exception case
volatile uint8_t g_error_code = 0;

//Variable for checking spot turn in wall follow mode_
volatile int32_t g_wf_sp_turn_count;

uint8_t g_tilt_status = 0;

// For wheel PID adjustment
struct pid_argu_struct argu_for_pid = {REG_TYPE_NONE,0,0,0};
struct pid_struct left_pid = {0,0,0,0,0,0,0,0}, right_pid = {0,0,0,0,0,0,0,0};
boost::mutex pid_lock;

uint8_t get_direction_flag(void)
{
	return g_direction_flag;
}


/*----------------------- Set error functions--------------------------*/
void set_error_code(uint8_t code)
{
	g_error_code = code;
}

uint8_t get_error_code()
{
	return g_error_code;
}

void alarm_error(void)
{
	switch (get_error_code())
	{
		case Error_Code_LeftWheel:
		{
			wav_play(WAV_ERROR_LEFT_WHEEL);
			break;
		}
		case Error_Code_RightWheel:
		{
			wav_play(WAV_ERROR_RIGHT_WHEEL);
			break;
		}
		case Error_Code_LeftBrush:
		{
			wav_play(WAV_ERROR_LEFT_BRUSH);
			break;
		}
		case Error_Code_RightBrush:
		{
			wav_play(WAV_ERROR_RIGHT_BRUSH);
			break;
		}
		case Error_Code_MainBrush:
		{
			wav_play(WAV_ERROR_MAIN_BRUSH);
			break;
		}
		case Error_Code_Fan_H:
		{
			wav_play(WAV_ERROR_SUCTION_FAN);
			break;
		}
		case Error_Code_Cliff:
		{
			wav_play(WAV_ERROR_CLIFF);
			break;
		}
		case Error_Code_Bumper:
		{
			wav_play(WAV_ERROR_BUMPER);
			break;
		}
		case Error_Code_Omni:
		{
			wav_play(WAV_ERROR_MOBILITY_WHEEL);
			break;
		}
		case Error_Code_Laser:
		{
			wav_play(WAV_TEST_LIDAR);
			break;
		}
		case Error_Code_Stuck:
		{
			wav_play(WAV_ROBOT_STUCK);
			break;
		}
		default:
		{
			break;
		}
	}

}

bool check_error_cleared(uint8_t error_code)
{
	bool error_cleared = true;
	switch (error_code)
	{
		case Error_Code_LeftWheel:
		case Error_Code_RightWheel:
		case Error_Code_LeftBrush:
		case Error_Code_RightBrush:
		case Error_Code_MainBrush:
		case Error_Code_Fan_H:
			break;
		case Error_Code_Cliff:
		{
			if (cliff.get_status())
			{
				ROS_WARN("%s %d: Cliff still triggered.", __FUNCTION__, __LINE__);
				error_cleared = false;
			}
			break;
		}
		case Error_Code_Bumper:
		{
			if (bumper.get_status())
			{
				ROS_WARN("%s %d: Bumper still triggered.", __FUNCTION__, __LINE__);
				error_cleared = false;
			}
			break;
		}
		case Error_Code_Omni:
		{
			if(g_omni_notmove)
			{
				ROS_WARN("%s %d: Omni still triggered.", __FUNCTION__, __LINE__);
				error_cleared = false;
			}
			break;
		}
		default:
			break;
	}

	return error_cleared;
}

int32_t get_right_wheel_step(void)
{
	double t, step;
	double rwsp;
	if (g_right_wheel_speed < 0)
		rwsp = (double) g_right_wheel_speed * -1;
	else
		rwsp = (double) g_right_wheel_speed;
	t = (ros::Time::now() - g_rw_t).toSec();
	step = rwsp * t / 0.12;//origin 0.181
	g_right_wheel_step = (uint32_t) step;
	return g_right_wheel_step;
}

int32_t get_left_wheel_step(void)
{
	double t, step;
	double lwsp;
	if (g_left_wheel_speed < 0)
		lwsp = (double) g_left_wheel_speed * -1;
	else
		lwsp = (double) g_left_wheel_speed;
	t = (double) (ros::Time::now() - g_lw_t).toSec();
	step = lwsp * t / 0.12;//origin 0.181
	g_left_wheel_step = (uint32_t) step;
	return g_left_wheel_step;
}

void reset_wheel_step(void)
{
	g_lw_t = ros::Time::now();
	g_rw_t = ros::Time::now();
	g_right_wheel_step = 0;
	g_left_wheel_step = 0;
}

int32_t get_wall_adc(int8_t dir)
{
	if (dir == 0)
	{
		return (int32_t) (int16_t) robot::instance()->getLeftWall();
	} else
	{
		return (int32_t) (int16_t) robot::instance()->getRightWall();
	}
}

void set_dir_backward(void)
{
	g_wheel_left_direction = BACKWARD;
	g_wheel_right_direction = BACKWARD;
}

void set_dir_forward(void)
{
	g_wheel_left_direction = FORWARD;
	g_wheel_right_direction = FORWARD;
}

void wall_dynamic_base(uint32_t Cy)
{
	//ROS_INFO("Run wall_dynamic_base.");
	static int32_t Left_Wall_Sum_Value = 0, Right_Wall_Sum_Value = 0;
	static int32_t Left_Wall_Everage_Value = 0, Right_Wall_Everage_Value = 0;
	static int32_t Left_Wall_E_Counter = 0, Right_Wall_E_Counter = 0;
	static int32_t Left_Temp_Wall_Buffer = 0, Right_Temp_Wall_Buffer = 0;
	// Dynamic adjust for left wall sensor.
	Left_Temp_Wall_Buffer = get_wall_adc(0);
	Left_Wall_Sum_Value += Left_Temp_Wall_Buffer;
	Left_Wall_E_Counter++;
	Left_Wall_Everage_Value = Left_Wall_Sum_Value / Left_Wall_E_Counter;
	double obstacle_distance_left = DBL_MAX;
	double obstacle_distance_right = DBL_MAX;

	obstacle_distance_left = MotionManage::s_laser->getObstacleDistance(2,ROBOT_RADIUS);
	obstacle_distance_right = MotionManage::s_laser->getObstacleDistance(3,ROBOT_RADIUS);

	if (abs_minus(Left_Wall_Everage_Value, Left_Temp_Wall_Buffer) > 20 || obstacle_distance_left < (ROBOT_RADIUS + 0.30) || robot::instance()->getLeftWall() > 300)
	{
//		ROS_ERROR("left_reset");
		Left_Wall_Everage_Value = 0;
		Left_Wall_E_Counter = 0;
		Left_Wall_Sum_Value = 0;
		Left_Temp_Wall_Buffer = 0;
	}
	if ((uint32_t) Left_Wall_E_Counter > Cy)
	{
		// Get the wall base line for left wall sensor.
		Left_Wall_Everage_Value += get_wall_base(0);
		if (Left_Wall_Everage_Value > 300)Left_Wall_Everage_Value = 300;//set a limit
		// Adjust the wall base line for left wall sensor.
		set_wall_base(0, Left_Wall_Everage_Value);
//		ROS_ERROR("left_wall_value:%d",Left_Wall_Everage_Value);
		Left_Wall_Everage_Value = 0;
		Left_Wall_E_Counter = 0;
		Left_Wall_Sum_Value = 0;
		Left_Temp_Wall_Buffer = 0;
		//ROS_INFO("Set Left Wall base value as: %d.", get_wall_base(0));
	}

	// Dynamic adjust for right wall sensor.
	Right_Temp_Wall_Buffer = get_wall_adc(1);
	Right_Wall_Sum_Value += Right_Temp_Wall_Buffer;
	Right_Wall_E_Counter++;
	Right_Wall_Everage_Value = Right_Wall_Sum_Value / Right_Wall_E_Counter;

	if (abs_minus(Right_Wall_Everage_Value, Right_Temp_Wall_Buffer) > 20 || obstacle_distance_right < (ROBOT_RADIUS + 0.30) || robot::instance()->getRightWall() > 300)
	{
//		ROS_ERROR("right_reset");
		Right_Wall_Everage_Value = 0;
		Right_Wall_E_Counter = 0;
		Right_Wall_Sum_Value = 0;
		Right_Temp_Wall_Buffer = 0;
	}
	if ((uint32_t) Right_Wall_E_Counter > Cy)
	{
		// Get the wall base line for right wall sensor.
		Right_Wall_Everage_Value += get_wall_base(1);
		if (Right_Wall_Everage_Value > 300)Right_Wall_Everage_Value = 300;//set a limit
		// Adjust the wall base line for right wall sensor.
//		ROS_ERROR("right_wall_value:%d",Right_Wall_Everage_Value);
		set_wall_base(1, Right_Wall_Everage_Value);
		//ROS_INFO("%s,%d:right_wall_value: \033[31m%d\033[0m",__FUNCTION__,__LINE__,Right_Wall_Everage_Value);
		Right_Wall_Everage_Value = 0;
		Right_Wall_E_Counter = 0;
		Right_Wall_Sum_Value = 0;
		Right_Temp_Wall_Buffer = 0;
		//ROS_INFO("Set Right Wall base value as: %d.", get_wall_base(0));
	}

}

void set_wall_base(int8_t dir, int32_t data)
{
	if (dir == 0)
	{
		g_left_wall_baseline = data;
	} else
	{
		g_right_wall_baseline = data;
	}
}

int32_t get_wall_base(int8_t dir)
{
	if (dir == 0)
	{
		return g_left_wall_baseline;
	} else
	{
		return g_right_wall_baseline;
	}
}

void quick_back(uint8_t speed, uint16_t distance)
{
	// The distance is for mm.
	float saved_x, saved_y;
	saved_x = robot::instance()->getOdomPositionX();
	saved_y = robot::instance()->getOdomPositionY();
	// Quickly move back for a distance.
	g_wheel_left_direction = BACKWARD;
	g_wheel_right_direction = BACKWARD;
	reset_wheel_step();
	set_wheel_speed(speed, speed);
	while (sqrtf(powf(saved_x - robot::instance()->getOdomPositionX(), 2) +
							 powf(saved_y - robot::instance()->getOdomPositionY(), 2)) < (float) distance / 1000)
	{
		ROS_DEBUG("%s %d: saved_x: %f, saved_y: %f current x: %f, current y: %f.", __FUNCTION__, __LINE__, saved_x, saved_y,
							robot::instance()->getOdomPositionX(), robot::instance()->getOdomPositionY());
		if (ev.fatal_quit || ev.key_clean_pressed || ev.charge_detect || ev.cliff_all_triggered)
			break;
		usleep(20000);
	}
	ROS_INFO("quick_back finished.");
}

/*-------------------------------Check if at charger stub------------------------------------*/
bool is_on_charger_stub(void)
{
	// 1: On charger stub and charging.
	// 2: On charger stub but not charging.
	if (robot::instance()->getChargeStatus() == 2 || robot::instance()->getChargeStatus() == 1)
		return true;
	else
		return false;
}

bool is_direct_charge(void)
{
	// 3: Direct connect to charge line but not charging.
	// 4: Direct connect to charge line and charging.
	if (robot::instance()->getChargeStatus() == 3 || robot::instance()->getChargeStatus() == 4)
		return true;
	else
		return false;
}

void set_argu_for_pid(uint8_t reg_type, float Kp, float Ki, float Kd)
{
	boost::mutex::scoped_lock(pid_lock);
	argu_for_pid.reg_type = reg_type;
	argu_for_pid.Kp = Kp;
	argu_for_pid.Ki = Ki;
	argu_for_pid.Kd = Kd;
}
void wheels_pid(void)
{
	boost::mutex::scoped_lock(pid_lock);
#if 0
	left_pid.delta = left_pid.target_speed - left_pid.actual_speed;
	/*---target speed changed, reset err_sum---*/
	if(left_pid.last_target_speed != left_pid.target_speed)
		left_pid.delta_sum = 0;
	left_pid.delta_sum += left_pid.delta;

	/*---pid---*/
	left_pid.variation = argu_for_pid.Kp*left_pid.delta + argu_for_pid.Ki*left_pid.delta_sum + argu_for_pid.Kd*(left_pid.delta - left_pid.delta_last);
	left_pid.actual_speed += left_pid.variation;

	/*---update status---*/
	left_pid.last_target_speed = left_pid.target_speed;
	left_pid.delta_last = left_pid.delta;

	right_pid.delta = right_pid.target_speed - right_pid.actual_speed;
	/*---target speed changed, reset err_sum---*/
	if(right_pid.last_target_speed != right_pid.target_speed)
		right_pid.delta_sum = 0;
	right_pid.delta_sum += right_pid.delta;

	/*---pid---*/
	right_pid.variation = argu_for_pid.Kp*right_pid.delta + argu_for_pid.Ki*right_pid.delta_sum + argu_for_pid.Kd*(right_pid.delta - right_pid.delta_last);
	right_pid.actual_speed += right_pid.variation;

	/*---update status---*/
	right_pid.last_target_speed = right_pid.target_speed;
	right_pid.delta_last = right_pid.delta;
#else
	if (argu_for_pid.reg_type != REG_TYPE_NONE && (left_pid.last_reg_type != argu_for_pid.reg_type || right_pid.last_reg_type != argu_for_pid.reg_type))
	{
#if 1
		if(argu_for_pid.reg_type == REG_TYPE_BACK)
		{
			/*---brake when turn to back regulator---*/
			left_pid.actual_speed = 0;
			right_pid.actual_speed = 0;
		}
		else
#endif
		{
			//ROS_INFO("%s %d: Slowly reset the speed to zero.", __FUNCTION__, __LINE__);
			if (left_pid.actual_speed < 0)
				left_pid.actual_speed -= static_cast<int8_t>(left_pid.actual_speed) >= -6 ? left_pid.actual_speed : floor(left_pid.actual_speed / 10.0);
			else if (left_pid.actual_speed > 0)
				left_pid.actual_speed -= static_cast<int8_t>(left_pid.actual_speed) <= 6 ? left_pid.actual_speed : ceil(left_pid.actual_speed / 10.0);
			if (right_pid.actual_speed < 0)
				right_pid.actual_speed -= static_cast<int8_t>(right_pid.actual_speed) >= -6 ? right_pid.actual_speed : floor(right_pid.actual_speed / 10.0);
			else if (right_pid.actual_speed > 0)
				right_pid.actual_speed -= static_cast<int8_t>(right_pid.actual_speed) <= 6 ? right_pid.actual_speed : ceil(right_pid.actual_speed / 10.0);
		}

		if (left_pid.actual_speed == 0 || right_pid.actual_speed == 0)
		{
			left_pid.actual_speed = 0;
			right_pid.actual_speed = 0;
			left_pid.last_reg_type = right_pid.last_reg_type = argu_for_pid.reg_type;
			//ROS_INFO("%s %d: Switch PID type to %d.", __FUNCTION__, __LINE__, argu_for_pid.reg_type);
		}
	}
	else if(argu_for_pid.reg_type == REG_TYPE_NONE || argu_for_pid.reg_type == REG_TYPE_WALLFOLLOW)
	{
		left_pid.actual_speed = left_pid.target_speed;
		right_pid.actual_speed = right_pid.target_speed;
		left_pid.last_reg_type = right_pid.last_reg_type = argu_for_pid.reg_type;
	}
	else
	{
	#if 0
		/*---if one of the wheels should change direction, set both target_speed to 0 first---*/
		if((left_pid.actual_speed * left_pid.target_speed < 0) || (right_pid.actual_speed * right_pid.target_speed < 0))
		{
			left_pid.target_speed = 0;
			right_pid.target_speed = 0;
		}
		left_pid.variation = left_pid.target_speed - left_pid.actual_speed;
		right_pid.variation = right_pid.target_speed - right_pid.actual_speed;
		/*---set variation limit---*/
		float variation_limit = 0;
		if(argu_for_pid.reg_type == REG_TYPE_LINEAR)
			variation_limit = 1;
		else if(argu_for_pid.reg_type == REG_TYPE_CURVE)
			variation_limit = 8;
		else if(argu_for_pid.reg_type == REG_TYPE_TURN)
			variation_limit = 4;
		else if(argu_for_pid.reg_type == REG_TYPE_BACK)
			variation_limit = 20;
		/*---adjust speed---*/
		if(left_pid.variation > variation_limit)left_pid.variation = variation_limit;
		else if(left_pid.variation < -variation_limit)left_pid.variation = -variation_limit;
		if(right_pid.variation > variation_limit)right_pid.variation = variation_limit;
		else if(right_pid.variation < -variation_limit)right_pid.variation = -variation_limit;

		left_pid.actual_speed += left_pid.variation;
		right_pid.actual_speed += right_pid.variation;
	#endif
		if(fabsf(left_pid.actual_speed) <= 6)
		{
			if(left_pid.target_speed > 0)
				left_pid.actual_speed = (left_pid.target_speed > 7) ? 7 : left_pid.target_speed;
			else
				left_pid.actual_speed = (left_pid.target_speed < -7) ? -7 : left_pid.target_speed;
		}
		else if(fabsf(left_pid.actual_speed) <= 15)
		{
			// For low actual speed cases.
			left_pid.delta = left_pid.target_speed - left_pid.actual_speed;
			if(left_pid.delta > 0)
				left_pid.actual_speed += left_pid.actual_speed > 0 ? 1 : 0.5;
			else if(left_pid.delta < 0)
				left_pid.actual_speed -= left_pid.actual_speed < 0 ? 1 : 0.5;
		}
		else if (fabsf(left_pid.target_speed) <= 10)
		{
			// For high actual speed cases.
			left_pid.delta = left_pid.target_speed - left_pid.actual_speed;
			if(left_pid.delta > 0)
				left_pid.actual_speed += 4;
			else if(left_pid.delta < 0)
				left_pid.actual_speed -= 4;
		}
		else
			left_pid.actual_speed = left_pid.target_speed;

		if(fabsf(right_pid.actual_speed) <= 6)
		{
			if(right_pid.target_speed > 0)
				right_pid.actual_speed = (right_pid.target_speed > 7) ? 7 : right_pid.target_speed;
			else
				right_pid.actual_speed = (right_pid.target_speed < -7) ? -7 : right_pid.target_speed;
		}
		else if(fabsf(right_pid.actual_speed) <= 15)
		{
			// For low actual speed cases.
			right_pid.delta = right_pid.target_speed - right_pid.actual_speed;
			if(right_pid.delta > 0)
				right_pid.actual_speed += right_pid.actual_speed > 0 ? 1 : 0.5;
			else if(right_pid.delta < 0)
				right_pid.actual_speed -= right_pid.actual_speed < 0 ? 1 : 0.5;
		}
		else if (fabsf(right_pid.target_speed) <= 10)
		{
			// For high actual speed cases.
			right_pid.delta = right_pid.target_speed - right_pid.actual_speed;
			if(right_pid.delta > 0)
				right_pid.actual_speed += 4;
			else if(right_pid.delta < 0)
				right_pid.actual_speed -= 4;
		}
		else
			right_pid.actual_speed = right_pid.target_speed;

		if(left_pid.actual_speed > RUN_TOP_SPEED)left_pid.actual_speed = (int8_t)RUN_TOP_SPEED;
		else if(left_pid.actual_speed < -RUN_TOP_SPEED)left_pid.actual_speed = -(int8_t)RUN_TOP_SPEED;
		if(right_pid.actual_speed > RUN_TOP_SPEED)right_pid.actual_speed = (int8_t)RUN_TOP_SPEED;
		else if(right_pid.actual_speed < -RUN_TOP_SPEED)right_pid.actual_speed = -(int8_t)RUN_TOP_SPEED;
	}
	//ROS_INFO("%s %d: real speed: %f, %f, target speed: %f, %f, reg_type: %d.", __FUNCTION__, __LINE__, left_pid.actual_speed, right_pid.actual_speed, left_pid.target_speed, right_pid.target_speed, argu_for_pid.reg_type);

	/*---update status---*/
	left_pid.last_target_speed = left_pid.target_speed;
	right_pid.last_target_speed = right_pid.target_speed;
#endif
}
void set_wheel_speed(uint8_t Left, uint8_t Right, uint8_t reg_type, float PID_p, float PID_i, float PID_d)
{
	int8_t signed_left_speed = (int8_t)Left;
	int8_t signed_right_speed = (int8_t)Right;
	set_argu_for_pid(reg_type, PID_p, PID_i, PID_d);

	if(g_wheel_left_direction == BACKWARD)
		signed_left_speed *= -1;
	if(g_wheel_right_direction == BACKWARD)
		signed_right_speed *= -1;
	left_pid.target_speed = (float)signed_left_speed;
	right_pid.target_speed = (float)signed_right_speed;
}
void set_left_wheel_speed(uint8_t speed)
{
	int16_t l_speed;
	speed = speed > RUN_TOP_SPEED ? RUN_TOP_SPEED : speed;
	l_speed = (int16_t) (speed * SPEED_ALF);
	g_left_wheel_speed = l_speed;
	if (g_wheel_left_direction == BACKWARD)
	{
		l_speed |= 0x8000;
		g_left_wheel_speed *= -1;
	}
	controller.set(CTL_WHEEL_LEFT_HIGH, (l_speed >> 8) & 0xff);
	controller.set(CTL_WHEEL_LEFT_LOW, l_speed & 0xff);

}

void set_right_wheel_speed(uint8_t speed)
{
	int16_t r_speed;
	speed = speed > RUN_TOP_SPEED ? RUN_TOP_SPEED : speed;
	r_speed = (int16_t) (speed * SPEED_ALF);
	g_right_wheel_speed = r_speed;
	if (g_wheel_right_direction == BACKWARD)
	{
		r_speed |= 0x8000;
		g_right_wheel_speed *= -1;
	}
	controller.set(CTL_WHEEL_RIGHT_HIGH, (r_speed >> 8) & 0xff);
	controller.set(CTL_WHEEL_RIGHT_LOW, r_speed & 0xff);
}

int16_t get_left_wheel_speed(void)
{
	return g_left_wheel_speed;
}

int16_t get_right_wheel_speed(void)
{
	return g_right_wheel_speed;
}

void work_motor_configure(void)
{
	if (cs_is_going_home())
	{
		// Set the vacuum to a normal mode_
		vacuum.mode(Vac_Normal, false);
	} else {
		vacuum.mode(Vac_Save);
	}

	// Trun on the main brush and side brush
	brush.set_side_pwm(50, 50);
	brush.set_main_pwm(30);
}

/*-----------------------------------------------------------Self Check-------------------*/
uint8_t self_check(uint8_t Check_Code)
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
	disable_motors();
	usleep(10000);
	/*------------------------------Self Check right wheel -------------------*/
	if (Check_Code == Check_Right_Wheel)
	{
		Right_Wheel_Slow = 0;
		if (get_direction_flag() == Direction_Flag_Left)
		{
			set_dir_right();
		} else
		{
			set_dir_left();
		}
		set_wheel_speed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) robot::instance()->getRwheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			disable_motors();
			ROS_WARN("%s,%d right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
			set_error_code(Error_Code_RightWheel);
			alarm_error();
			return 1;

		}
		/*
		if(Right_Wheel_Slow>100)
		{
			disable_motors();
			set_error_code(Error_Code_RightWheel);
			return 1;
		}
		*/
		stop_brifly();
		//turn_right(Turn_Speed,1800);
	}
		/*---------------------------Self Check left wheel -------------------*/
	else if (Check_Code == Check_Left_Wheel)
	{
		Left_Wheel_Slow = 0;
		if (get_direction_flag() == Direction_Flag_Right)
		{
			set_dir_left();
		} else
		{
			set_dir_right();
		}
		set_wheel_speed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) robot::instance()->getLwheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			disable_motors();
			ROS_WARN("%s %d,left wheel stall maybe, please check!!", __FUNCTION__, __LINE__);
			set_error_code(Error_Code_LeftWheel);
			alarm_error();
			return 1;
		}
		/*
		if(Left_Wheel_Slow>100)
		{
			disable_motors();
			set_error_code(Error_Code_RightWheel);
			return 1;
		}
		*/
		stop_brifly();
		//turn_left(Turn_Speed,1800);
	} else if (Check_Code == Check_Main_Brush)
	{
		if (!mbrushchecking)
		{
			brush.set_main_pwm(0);
			mbrushchecking = 1;
			mboctime = time(NULL);
		} else if ((uint32_t) difftime(time(NULL), mboctime) >= 3)
		{
			mbrushchecking = 0;
			set_error_code(Error_Code_MainBrush);
			disable_motors();
			alarm_error();
			return 1;
		}
		return 0;
	} else if (Check_Code == Check_Vacuum)
	{
#ifndef BLDC_INSTALL
		ROS_INFO("%s, %d: Vacuum Over Current!!", __FUNCTION__, __LINE__);
		ROS_INFO("%d", get_self_check_vacuum_status());
		while (get_self_check_vacuum_status() != 0x10)
		{
			/*-----wait until self check begin-----*/
			start_self_check_vacuum();
		}
		ROS_INFO("%s, %d: Vacuum Self checking", __FUNCTION__, __LINE__);
		/*-----reset command for start self check-----*/
		reset_self_check_vacuum_controler();
		/*-----wait for the end of self check-----*/
		while (get_self_check_vacuum_status() == 0x10);
		ROS_INFO("%s, %d: end of Self checking", __FUNCTION__, __LINE__);
		if (get_self_check_vacuum_status() == 0x20)
		{
			ROS_INFO("%s, %d: Vacuum error", __FUNCTION__, __LINE__);
			/*-----vacuum error-----*/
			set_error_code(Error_Code_Fan_H);
			disable_motors();
			alarm_error();
			reset_self_check_vacuum_controler();
			return 1;
		}
		reset_self_check_vacuum_controler();
#else
		Disable_Motors();
		//stop_brifly();
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
		set_error_code(Error_Code_Fan_H);
		disable_motors();
		Alarm_Error();
		return 1;
#endif
	} else if (Check_Code == Check_Left_Brush)
	{
		set_error_code(Error_Code_LeftBrush);
		disable_motors();
		alarm_error();
		return 1;
	} else if (Check_Code == Check_Right_Brush)
	{
		set_error_code(Error_Code_RightBrush);
		disable_motors();
		alarm_error();
		return 1;
	}
	stop_brifly();
	Left_Wheel_Slow = 0;
	Right_Wheel_Slow = 0;
	work_motor_configure();
	//move_forward(5,5);
	return 0;
}

uint8_t get_self_check_vacuum_status(void)
{
	return (uint8_t) robot::instance()->getVacuumSelfCheckStatus();
}

void move_forward(uint8_t Left_Speed, uint8_t Right_Speed)
{
	set_dir_forward();
	set_wheel_speed(Left_Speed, Right_Speed);
}

void set_dir_left(void)
{
	set_direction_flag(Direction_Flag_Left);
	g_wheel_left_direction = BACKWARD;
	g_wheel_right_direction = FORWARD;
}

void set_dir_right(void)
{
	set_direction_flag(Direction_Flag_Right);
	g_wheel_left_direction = FORWARD;
	g_wheel_right_direction = BACKWARD;
}

void set_led(uint16_t G, uint16_t R)
{
	// Set the brightnesss of the LED within range(0, 100).
	G = G < 100 ? G : 100;
	R = R < 100 ? R : 100;
	controller.set(CTL_LED_RED, R & 0xff);
	controller.set(CTL_LED_GREEN, G & 0xff);
}

void stop_brifly(void)
{
	//ROS_INFO("%s %d: stopping robot.", __FUNCTION__, __LINE__);
	do
	{
		set_wheel_speed(0, 0, REG_TYPE_LINEAR);
		usleep(15000);
		//ROS_INFO("%s %d: linear speed: (%f, %f, %f)", __FUNCTION__, __LINE__,
		//	robot::instance()->getLinearX(), robot::instance()->getLinearY(), robot::instance()->getLinearZ());
	} while (robot::instance()->isMoving());
	//ROS_INFO("%s %d: robot is stopped.", __FUNCTION__, __LINE__);
}

void reset_stop_event_status(void)
{
	g_stop_event_status = 0;
	// For key release checking.
	key.reset();
}

uint8_t is_water_tank(void)
{
	return 0;
}


//void cm_set(uint8_t mode_)
//{
//	g_cleaning_mode = mode_;
//}

void beep(uint8_t Sound_Code, int Sound_Time_Ms, int Silence_Time_Ms, int Total_Time_Count)
{
	// Sound_Code means the interval of the speaker sounding, higher interval makes lower sound.
	robotbase_sound_code = Sound_Code;
	// Total_Time_Count means how many loops of speaker sound loop will it sound.
	robotbase_speaker_sound_loop_count = Total_Time_Count;
	// A speaker sound loop contains one sound time and one silence time
	// Sound_Time_Count means how many loops of g_send_stream loop will it sound in one speaker sound loop
	robotbase_speaker_sound_time_count = Sound_Time_Ms / 20;
	// Silence_Time_Count means how many loops of g_send_stream loop will it be silence in one speaker sound loop, -1 means consistently beep.
	robotbase_speaker_silence_time_count = Silence_Time_Ms / 20;
	// Trigger the update flag to start the new beep action
	robotbase_beep_update_flag = true;
}


void disable_motors(void)
{
	// Disable all the motors, including brush, wheels, and vacuum.
	// Stop the wheel
	set_wheel_speed(0, 0);
	// Stop the side brush
	brush.set_side_pwm(0, 0);
	// Stop the main brush
	brush.set_main_pwm(0);
	// Stop the vacuum, directly stop the BLDC
	vacuum.stop();
}

void set_start_charge(void)
{
	// This function will turn on the charging function.
	controller.set(CTL_CHARGER, 0x01);
}

void set_stop_charge(void)
{
	// Set the flag to false so that it can quit charger mode_.
	controller.set(CTL_CHARGER, 0x00);
}

void set_main_pwr_byte(uint8_t val)
{
	controller.set(CTL_MAIN_PWR, val & 0xff);
}

uint8_t get_main_pwr_byte()
{
	return controller.get(CTL_MAIN_PWR);
}

void start_self_check_vacuum(void)
{
	uint8_t omni_reset_byte = controller.get(CTL_OMNI_RESET);
	controller.set(CTL_OMNI_RESET, omni_reset_byte | 0x02);
}

void reset_self_check_vacuum_controler(void)
{
	uint8_t omni_reset_byte = controller.get(CTL_OMNI_RESET);
	controller.set(CTL_OMNI_RESET, omni_reset_byte & ~0x06);
}

void set_direction_flag(uint8_t flag)
{
	g_direction_flag = flag;
}

void reset_mobility_step()
{
	uint8_t omni_reset_byte = controller.get(CTL_OMNI_RESET);
	controller.set(CTL_OMNI_RESET, omni_reset_byte | 0x01);
}

void clear_reset_mobility_step()
{
	uint8_t omni_reset_byte = controller.get(CTL_OMNI_RESET);
	controller.set(CTL_OMNI_RESET, omni_reset_byte & ~0x01);
}

int32_t abs_minus(int32_t A, int32_t B)
{
	if (A > B)
	{
		return A - B;
	}
	return B - A;
}

uint8_t get_sleep_mode_flag()
{
	return g_sleep_mode_flag;
}

void set_sleep_mode_flag()
{
	g_sleep_mode_flag = 1;
}

void reset_sleep_mode_flag()
{
	g_sleep_mode_flag = 0;
}

void beep_for_command(bool valid)
{
	if (valid)
		beep(2, 40, 0, 1);
	else
		beep(5, 40, 0, 1);
}

void reset_sp_turn_count()
{
	g_wf_sp_turn_count = 0;
}

int32_t get_sp_turn_count()
{
	return g_wf_sp_turn_count;
}

void add_sp_turn_count()
{
	g_wf_sp_turn_count++;
}

void set_led_mode(uint8_t type, uint8_t color, uint16_t time_ms)
{
	robotbase_led_type = type;
	robotbase_led_color = color;
	robotbase_led_cnt_for_switch = time_ms / 20;
	live_led_cnt_for_switch = 0;
	robotbase_led_update_flag = true;
}

int16_t get_front_acc()
{
#if GYRO_FRONT_X_POS
	return -robot::instance()->getXAcc();
#elif GYRO_FRONT_X_NEG
	return robot::instance()->getXAcc();
#elif GYRO_FRONT_Y_POS
	return -robot::instance()->getYAcc();
#elif GYRO_FRONT_Y_NEG
	return robot::instance()->getYAcc();
#endif
}

int16_t get_left_acc()
{
#if GYRO_FRONT_X_POS
	return -robot::instance()->getYAcc();
#elif GYRO_FRONT_X_NEG
	return robot::instance()->getYAcc();
#elif GYRO_FRONT_Y_POS
	return robot::instance()->getXAcc();
#elif GYRO_FRONT_Y_NEG
	return -robot::instance()->getXAcc();
#endif
}

int16_t get_right_acc()
{
#if GYRO_FRONT_X_POS
	return -robot::instance()->getYAcc();
#elif GYRO_FRONT_X_NEG
	return robot::instance()->getYAcc();
#elif GYRO_FRONT_Y_POS
	return robot::instance()->getXAcc();
#elif GYRO_FRONT_Y_NEG
	return -robot::instance()->getXAcc();
#endif
}

int16_t get_front_init_acc()
{
#if GYRO_FRONT_X_POS
	return -robot::instance()->getInitXAcc();
#elif GYRO_FRONT_X_NEG
	return robot::instance()->getInitXAcc();
#elif GYRO_FRONT_Y_POS
	return -robot::instance()->getInitYAcc();
#elif GYRO_FRONT_Y_NEG
	return robot::instance()->getInitYAcc();
#endif
}

int16_t get_left_init_acc()
{
#if GYRO_FRONT_X_POS
	return -robot::instance()->getInitYAcc();
#elif GYRO_FRONT_X_NEG
	return robot::instance()->getInitYAcc();
#elif GYRO_FRONT_Y_POS
	return robot::instance()->getInitXAcc();
#elif GYRO_FRONT_Y_NEG
	return -robot::instance()->getInitXAcc();
#endif
}

int16_t get_right_init_acc()
{
#if GYRO_FRONT_X_POS
	return -robot::instance()->getInitYAcc();
#elif GYRO_FRONT_X_NEG
	return robot::instance()->getInitYAcc();
#elif GYRO_FRONT_Y_POS
	return robot::instance()->getInitXAcc();
#elif GYRO_FRONT_Y_NEG
	return -robot::instance()->getInitXAcc();
#endif
}

uint8_t check_tilt()
{
	static uint16_t front_tilt_count = 0;
	static uint16_t left_tilt_count = 0;
	static uint16_t right_tilt_count = 0;
	static uint16_t z_tilt_count = 0;
	uint8_t tmp_tilt_status = 0;

	if (g_tilt_enable)
	{
		if (get_front_acc() - get_front_init_acc() > FRONT_TILT_LIMIT)
		{
			front_tilt_count += 2;
			//ROS_WARN("%s %d: front(%d)\tfront init(%d), front cnt(%d).", __FUNCTION__, __LINE__, get_front_acc(), get_front_init_acc(), front_tilt_count);
		}
		else
		{
			if (front_tilt_count > 0)
				front_tilt_count--;
			else
				front_tilt_count = 0;
		}
		if (get_left_acc() - get_left_init_acc() > LEFT_TILT_LIMIT)
		{
			left_tilt_count++;
			//ROS_WARN("%s %d: left(%d)\tleft init(%d), left cnt(%d).", __FUNCTION__, __LINE__, get_left_acc(), get_left_init_acc(), left_tilt_count);
		}
		else
		{
			if (left_tilt_count > 0)
				left_tilt_count--;
			else
				left_tilt_count = 0;
		}
		if (get_right_acc() - get_right_init_acc() > RIGHT_TILT_LIMIT)
		{
			right_tilt_count++;
			//ROS_WARN("%s %d: right(%d)\tright init(%d), right cnt(%d).", __FUNCTION__, __LINE__, get_right_acc(), get_right_init_acc(), right_tilt_count);
		}
		else
		{
			if (right_tilt_count > 0)
				right_tilt_count--;
			else
				right_tilt_count = 0;
		}
		if (abs(robot::instance()->getZAcc() - robot::instance()->getInitZAcc()) > DIF_TILT_Z_VAL)
		{
			z_tilt_count++;
			//ROS_WARN("%s %d: z(%d)\tzi(%d).", __FUNCTION__, __LINE__, robot::instance()->getZAcc(), robot::instance()->getInitZAcc());
		}
		else
		{
			if (z_tilt_count > 1)
				z_tilt_count -= 2;
			else
				z_tilt_count = 0;
		}

		//if (left_tilt_count > 7 || front_tilt_count > 7 || right_tilt_count > 7 || z_tilt_count > 7)
			//ROS_WARN("%s %d: tilt_count left:%d, front:%d, right:%d, z:%d", __FUNCTION__, __LINE__, left_tilt_count, front_tilt_count, right_tilt_count, z_tilt_count);

		if (front_tilt_count + left_tilt_count + right_tilt_count + z_tilt_count > TILT_COUNT_REACH)
		{
			ROS_INFO("\033[47;34m" "%s,%d,robot tilt !!" "\033[0m",__FUNCTION__,__LINE__);
			if (left_tilt_count > TILT_COUNT_REACH / 3)
				tmp_tilt_status |= TILT_LEFT;
			if (right_tilt_count > TILT_COUNT_REACH / 3)
				tmp_tilt_status |= TILT_RIGHT;

			if (front_tilt_count > TILT_COUNT_REACH / 3 || !tmp_tilt_status)
				tmp_tilt_status |= TILT_FRONT;
			set_tilt_status(tmp_tilt_status);
			front_tilt_count /= 3;
			left_tilt_count /= 3;
			right_tilt_count /= 3;
			z_tilt_count /= 3;
		}
		else if (front_tilt_count + left_tilt_count + right_tilt_count + z_tilt_count < TILT_COUNT_REACH / 4)
			set_tilt_status(0);
	}
	else{
		front_tilt_count = 0;
		left_tilt_count = 0;
		right_tilt_count = 0;
		z_tilt_count = 0;
		set_tilt_status(0);
	}

	return tmp_tilt_status;
}

void set_tilt_status(uint8_t status)
{
	g_tilt_status = status;
}

uint8_t get_tilt_status()
{
	return g_tilt_status;
}

bool check_pub_scan()
{
	//ROS_INFO("%s %d: get_left_wheel_speed() = %d, get_right_wheel_speed() = %d.", __FUNCTION__, __LINE__, get_left_wheel_speed(), get_right_wheel_speed());
	if (g_motion_init_succeeded &&
		((fabs(robot::instance()->getLeftWheelSpeed() - robot::instance()->getRightWheelSpeed()) > 0.1)
		|| (robot::instance()->getLeftWheelSpeed() * robot::instance()->getRightWheelSpeed() < 0)
		|| bumper.get_status() || get_tilt_status()
		|| abs(get_left_wheel_speed() - get_right_wheel_speed()) > 100
		|| get_left_wheel_speed() * get_right_wheel_speed() < 0))
		return false;
	else
		return true;
}

uint8_t is_robot_slip()
{
	uint8_t ret = 0;
	if(MotionManage::s_laser != nullptr && MotionManage::s_laser->isScan2Ready() && MotionManage::s_laser->isRobotSlip()){
		ROS_INFO("\033[35m""%s,%d,robot slip!!""\033[0m",__FUNCTION__,__LINE__);
		ret = 1;
	}
	return ret;
}

bool is_clean_paused()
{
	bool ret = false;
	if(g_is_manual_pause || g_robot_stuck)
	{
		ret= true;
	}
	return ret;
}

void reset_clean_paused(void)
{
	if (g_is_manual_pause || g_robot_stuck)
	{
		g_robot_stuck = false;
		// These are all the action that ~MotionManage() won't do if isManualPaused() returns true.
		ROS_WARN("Reset manual/stuck pause status.");
		wav_play(WAV_CLEANING_STOP);
		g_is_manual_pause = false;
		robot::instance()->savedOffsetAngle(0);
		if (MotionManage::s_slam != nullptr)
		{
			delete MotionManage::s_slam;
			MotionManage::s_slam = nullptr;
		}
		cm_reset_go_home();
		g_resume_cleaning = false;
	}
}

bool is_decelerate_wall(void)
{
	auto status = (obs.get_front() > obs.get_front_trig_value());
	return is_map_front_block(3) || status;
}
bool check_laser_stuck()
{
	if (MotionManage::s_laser != nullptr && !MotionManage::s_laser->laserCheckFresh(3, 2))
		return true;
	return false;
}

uint8_t get_laser_status()
{
	if (MotionManage::s_laser != nullptr)
		return MotionManage::s_laser->laserMarker(0.20);
	return 0;
}
