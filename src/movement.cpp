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

#include "gyro.h"
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

static int16_t obs_left_trig_value = 100;
static int16_t obs_front_trig_value = 100;
static int16_t obs_right_trig_value = 100;
int16_t g_obs_left_baseline = 100;
int16_t g_obs_front_baseline = 100;
int16_t g_obs_right_baseline = 100;
static int16_t cliff_left_trig_value = CLIFF_LIMIT;
static int16_t cliff_front_trig_value = CLIFF_LIMIT;
static int16_t cliff_right_trig_value = CLIFF_LIMIT;
int16_t g_cliff_left_baseline = 100;
int16_t g_cliff_front_baseline = 100;
int16_t g_cliff_right_baseline = 100;
static int16_t g_leftwall_obs_trig_vale = 500;
uint8_t g_wheel_left_direction = FORWARD;
uint8_t g_wheel_right_direction = FORWARD;
static uint8_t g_remote_move_flag = 0;
static uint8_t g_home_remote_flag = 0;
uint32_t movement_rcon_status;
uint32_t g_average_move = 0;
uint32_t g_average_counter = 0;
uint32_t g_max_move = 0;
uint32_t g_auto_work_time = 2800;
uint32_t g_room_work_time = 3600;
uint8_t g_room_mode = 0;
uint8_t g_sleep_mode_flag = 0;
extern Eigen::MatrixXd laser_matrix;

static uint32_t g_wall_accelerate = 0;
static int16_t g_left_wheel_speed = 0;
static int16_t g_right_wheel_speed = 0;
static int32_t g_left_wheel_step = 0;
static int32_t g_right_wheel_step = 0;
static uint32_t g_leftwall_step = 0;
static uint32_t g_rightwall_step = 0;

//Value for saving SideBrush_PWM
static uint16_t g_l_brush_pwm = 0;
static uint16_t g_r_brush_pwm = 0;

static int32_t g_move_step_counter = 0;
static uint32_t g_mobility_step = 0;
static uint8_t g_direction_flag = 0;
// Variable for vacuum mode

volatile uint8_t g_vac_mode;
volatile uint8_t g_vac_mode_save;
//static uint8_t g_cleaning_mode = 0;
static uint8_t g_sendflag = 0;
static time_t g_start_work_time;
ros::Time g_lw_t, g_rw_t; // these variable is used for calculate wheel step

// Flag for homeremote
volatile uint8_t g_r_h_flag = 0;

// Counter for bumper error
volatile uint8_t g_bumper_error = 0;

// Value for wall sensor offset.
volatile int16_t g_left_wall_baseline = 50;
volatile int16_t g_right_wall_baseline = 50;

// Variable for key status, key may have many key types.
volatile uint8_t g_key_status = 0;
// Variable for touch status, touch status is just for KEY_CLEAN.
volatile uint8_t g_touch_status = 0;
// Variable for remote status, remote status is just for remote controller.
volatile uint8_t g_remote_status = 0;
// Variable for stop event status.
volatile uint8_t g_stop_event_status = 0;
// Variable for plan status
volatile uint8_t g_plan_status = 0;

// Error code for exception case
volatile uint8_t g_error_code = 0;

//Variable for checking spot turn in wall follow mode
volatile int32_t g_wf_sp_turn_count;

bool g_reset_lbrush_oc = false;
bool g_reset_rbrush_oc = false;

uint8_t g_tilt_status = 0;

// For wheel PID adjustment
struct pid_argu_struct argu_for_pid = {REG_TYPE_NONE,0,0,0};
struct pid_struct left_pid = {0,0,0,0,0,0,0,0}, right_pid = {0,0,0,0,0,0,0,0};
boost::mutex pid_lock;

/*----------------------- Work Timer functions--------------------------*/
void reset_work_time()
{
	g_start_work_time = time(NULL);
}

uint32_t get_work_time()
{
	return (uint32_t) difftime(time(NULL), g_start_work_time);
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
			if (get_cliff_status())
			{
				ROS_WARN("%s %d: Cliff still triggered.", __FUNCTION__, __LINE__);
				error_cleared = false;
			}
			break;
		}
		case Error_Code_Bumper:
		{
			if (get_bumper_status())
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

void reset_wall_step(void)
{
	g_leftwall_step = 0;
	g_rightwall_step = 0;
}

uint32_t get_left_wall_step(void)
{
	return g_leftwall_step = get_left_wheel_step();
}

uint32_t get_right_wall_step(void)
{
	return g_rightwall_step = get_right_wheel_step();
}

void set_wheel_step(uint32_t Left, uint32_t Right)
{
	g_left_wheel_step = Left;
	g_right_wheel_step = Right;
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

uint8_t is_encoder_fail(void)
{
	return 0;
}

void set_right_brush_stall(uint8_t R)
{
	R = R;
}

void set_left_brush_stall(uint8_t L)
{
	L = L;
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

void turn_left(uint16_t speed, int16_t angle)
{
	auto target_angle = ranged_angle(gyro_get_angle() + angle);
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(),
					 speed);

	set_dir_left();

	set_wheel_speed(speed, speed);

	uint8_t oc = 0;
	uint8_t accurate;
	accurate = 10;
	if (speed > 30) accurate = 30;
	while (ros::ok())
	{
		// For GoHome(), if reach the charger stub during turning, should stop immediately.
		if (is_charge_on())
		{
			ROS_DEBUG("Reach charger while turn left.");
			stop_brifly();
			break;
		}
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < accurate)
		{
			break;
		}
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < 50)
		{
			auto speed_ = std::min((uint16_t) 5, speed);
			set_wheel_speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), 5);
		} else if (abs(ranged_angle(target_angle - gyro_get_angle())) < 200)
		{
			auto speed_ = std::min((uint16_t) 5, speed);
			set_wheel_speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), 10);
		} else
		{
			set_wheel_speed(speed, speed);
		}
		oc = check_motor_current();
		if (oc == Check_Left_Wheel || oc == Check_Right_Wheel)
			break;
		if (stop_event())
			break;
		//prompt for useless remote command
		if (get_rcon_remote() > 0)
		{
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (get_rcon_remote() & (Remote_Clean))
			{
			} else
			{
				beep_for_command(INVALID);
				reset_rcon_remote();
			}
		}
		/* check plan setting*/
		if (get_plan_status() == 1)
		{
			set_plan_status(0);
			beep_for_command(INVALID);
		}
		/*if(is_turn_remote())
			break;*/
		if (get_bumper_status())
		{
			break;
		}
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d,diff = %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed,target_angle - gyro_get_angle());
	}
	g_wheel_left_direction = FORWARD;
	g_wheel_right_direction = FORWARD;

	set_wheel_speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle());
}

void turn_right(uint16_t speed, int16_t angle)
{
	auto target_angle = ranged_angle(gyro_get_angle() - angle);
	if (target_angle < 0)
	{
		target_angle = 3600 + target_angle;
	}
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(),
					 speed);

	set_dir_right();

	set_wheel_speed(speed, speed);
	uint8_t oc = 0;

	uint8_t accurate;
	accurate = 10;
	if (speed > 30) accurate = 30;
	while (ros::ok())
	{
		// For GoHome(), if reach the charger stub during turning, should stop immediately.
		if (is_charge_on())
		{
			ROS_DEBUG("Reach charger while turn right.");
			stop_brifly();
			break;
		}
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < accurate)
		{
			break;
		}
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < 50)
		{
			auto speed_ = std::min((uint16_t) 5, speed);
			set_wheel_speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), 5);
		} else if (abs(ranged_angle(target_angle - gyro_get_angle())) < 200)
		{
			auto speed_ = std::min((uint16_t) 5, speed);
			set_wheel_speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), 10);
		} else
		{
			set_wheel_speed(speed, speed);
		}
		oc = check_motor_current();
		if (oc == Check_Left_Wheel || oc == Check_Right_Wheel)
			break;
		if (stop_event())
			break;
		//prompt for useless remote command
		if (get_rcon_remote() > 0)
		{
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (get_rcon_remote() & (Remote_Clean))
			{
			} else
			{
				beep_for_command(INVALID);
				reset_rcon_remote();
			}
		}
		/* check plan setting*/
		if (get_plan_status() == 1)
		{
			set_plan_status(0);
			beep_for_command(INVALID);
		}
		/*if(is_turn_remote())
			break;*/
		if (get_bumper_status())
		{
			break;
		}
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), speed);
	}
	g_wheel_left_direction = FORWARD;
	g_wheel_right_direction = FORWARD;

	set_wheel_speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle());
}

void jam_turn_left(uint16_t speed, int16_t angle)
{
	auto target_angle = ranged_angle(gyro_get_angle() + angle);
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle,
					 gyro_get_angle(), speed);

	set_dir_left();

	set_wheel_speed(speed, speed);

	uint8_t oc = 0;
	uint8_t accurate;
	accurate = 10;
	if (speed > 30) accurate = 30;
	while (ros::ok())
	{
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < accurate)
		{
			break;
		}
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < 50)
		{
			auto speed_ = std::min((uint16_t) 5, speed);
			set_wheel_speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), 5);
		} else if (abs(ranged_angle(target_angle - gyro_get_angle())) < 200)
		{
			auto speed_ = std::min((uint16_t) 5, speed);
			set_wheel_speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), 10);
		} else
		{
			set_wheel_speed(speed, speed);
		}
		oc = check_motor_current();
		if (oc == Check_Left_Wheel || oc == Check_Right_Wheel)
			break;
		if (stop_event())
			break;
		if (!get_bumper_status())
			break;
		/*if(is_turn_remote())
			break;*/
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d,diff = %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed,target_angle - gyro_get_angle());
	}
	g_wheel_left_direction = FORWARD;
	g_wheel_right_direction = FORWARD;

	set_wheel_speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle());
}

void jam_turn_right(uint16_t speed, int16_t angle)
{
	auto target_angle = ranged_angle(gyro_get_angle() - angle);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle,
					 gyro_get_angle(), speed);

	set_dir_right();

	set_wheel_speed(speed, speed);
	uint8_t oc = 0;

	uint8_t accurate;
	accurate = 10;
	if (speed > 30) accurate = 30;
	while (ros::ok())
	{
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < accurate)
		{
			break;
		}
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < 50)
		{
			auto speed_ = std::min((uint16_t) 5, speed);
			set_wheel_speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), 5);
		} else if (abs(ranged_angle(target_angle - gyro_get_angle())) < 200)
		{
			auto speed_ = std::min((uint16_t) 5, speed);
			set_wheel_speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), 10);
		} else
		{
			set_wheel_speed(speed, speed);
		}
		oc = check_motor_current();
		if (oc == Check_Left_Wheel || oc == Check_Right_Wheel)
			break;
		if (stop_event())
			break;
		if (!get_bumper_status())
			break;
		/*if(is_turn_remote())
			break;*/
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle(), speed);
	}
	g_wheel_left_direction = FORWARD;
	g_wheel_right_direction = FORWARD;

	set_wheel_speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, gyro_get_angle());
}

uint8_t get_bumper_status(void)
{
	uint8_t Temp_Status = 0;

	if (robot::instance()->getBumperLeft())
	{
		Temp_Status |= BLOCK_LEFT;
	}
	if (robot::instance()->getBumperRight())
	{
		Temp_Status |= BLOCK_RIGHT;
	}
	if (robot::instance()->getLidarBumper())
	{
		//Temp_Status |= LidarBumperTrig;
		Temp_Status |= BLOCK_FRONT;
	}
	if(Temp_Status == (BLOCK_LEFT | BLOCK_RIGHT) || (Temp_Status & BLOCK_FRONT) !=0)
		Temp_Status = BLOCK_ALL;
	return Temp_Status;
}

uint8_t get_cliff_status(void)
{
	uint8_t status = 0x00;

	if (get_left_cliff() < get_left_cliff_trig_value())
		status |= BLOCK_LEFT;

	if (get_front_cliff() < get_front_cliff_trig_value())
		status |= BLOCK_FRONT;

	if (get_right_cliff() < get_right_cliff_trig_value())
		status |= BLOCK_RIGHT;

	//if (status != 0x00){
	//	ROS_WARN("%s %d: Return Cliff status:%x.", __FUNCTION__, __LINE__, status);
	//	beep_for_command(true);
	//}
	return status;
}

//--------------------------------------Cliff Dynamic adjust----------------------
void cliff_dynamic_base(uint16_t count)
{
//	count = 20;
//	enum {front,left,right};
	static uint16_t cliff_cnt[] = {0,0,0};
	static int16_t cliff_sum[] = {0,0,0};
	const int16_t cliff_dynamic_limit = 500;
	int16_t* p_cliff_baseline[] = {&g_cliff_front_baseline, &g_cliff_left_baseline, &g_cliff_right_baseline};
	typedef int16_t(*Func_t)(void);
	Func_t p_get_cliff[] = {&get_front_cliff, &get_left_cliff, &get_right_cliff};
//	if(count == 0)
//		return ;
	for(int i =0;i<3;i++)
	{
//		if(i == 0)
//			ROS_WARN("front-------------------------");
//		if(i == 1)
//			ROS_WARN("left-------------------------");
//		if(i == 2)
//			ROS_WARN("right-------------------------");

		auto p_cliff_baseline_ = p_cliff_baseline[i];
		auto cliff_get = p_get_cliff[i]();

//		ROS_WARN("cliff_trig_val(%d),cliff_get(%d)", *p_cliff_baseline_, cliff_get);
		cliff_sum[i] += cliff_get;
		cliff_cnt[i]++;
		int16_t cliff_avg = cliff_sum[i] / cliff_cnt[i];
//		ROS_WARN("cliff_avg(%d), (%d / %d), ",cliff_avg, cliff_sum[i], cliff_cnt[i]);
		auto diff = abs_minus(cliff_avg , cliff_get);
		if (diff > 50)
		{
//			ROS_WARN("diff = (%d) > 50.", diff);
			cliff_cnt[i] = 0;
			cliff_sum[i] = 0;
		}
		if (cliff_cnt[i] > count)
		{
			cliff_cnt[i] = 0;
			cliff_sum[i] = 0;
			cliff_get = (cliff_avg + *p_cliff_baseline_) / 2;
			if (cliff_get > cliff_dynamic_limit)
				cliff_get = cliff_dynamic_limit;

			*p_cliff_baseline_ = cliff_get;
//			if(i == 0)
//				ROS_WARN("cliff front baseline = %d.", *p_cliff_baseline_);
//			else if(i == 1)
//				ROS_WARN("cliff left baseline = %d.", *p_cliff_baseline_);
//			else if(i == 2)
//				ROS_WARN("cliff right baseline = %d.", *p_cliff_baseline_);
		}
	}
}

int16_t get_front_cliff_trig_value(void)
{
	return cliff_front_trig_value;
}

int16_t get_left_cliff_trig_value(void)
{
	return cliff_left_trig_value;
}

int16_t get_right_cliff_trig_value(void)
{
	return cliff_right_trig_value;
}

int16_t get_front_cliff(void)
{
	return robot::instance()->getCliffFront();
}

int16_t get_left_cliff(void)
{
	return robot::instance()->getCliffLeft();
}

int16_t get_right_cliff(void)
{
	return robot::instance()->getCliffRight();
}


int get_rcon_trig_()
{
	enum {left,fl1,fl2,fr2,fr1,right};
	static int8_t cnt[6]={0,0,0,0,0,0};
	const int MAX_CNT = 1;
//	if(get_rcon_status() != 0)
//		ROS_WARN("get_rcon_status(%d)",get_rcon_status());
	if (get_rcon_status() & RconL_HomeT)
		cnt[left]++;
	if (get_rcon_status() & RconFL_HomeT)
		cnt[fl1]++;
	if (get_rcon_status() & RconFL2_HomeT)
		cnt[fl2]++;
	if (get_rcon_status() & RconFR2_HomeT)
		cnt[fr2]++;
	if (get_rcon_status() & RconFR_HomeT)
		cnt[fr1]++;
	if (get_rcon_status() & RconR_HomeT)
		cnt[right]++;
	auto ret = 0;
	for(int i=0;i<6;i++)
		if(cnt[i] > MAX_CNT)
		{
			cnt[left] = cnt[fl1] = cnt[fl2] = cnt[fr2] = cnt[fr1] = cnt[right] = 0;
			ret = i+1;
			break;
		}
	reset_rcon_status();
	return ret;
}

int get_rcon_trig(void)
{
//	if (cs_is_going_home()) {
////		ROS_WARN("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
//		reset_rcon_status();
//		return 0;
//	}
	if(mt_is_follow_wall()){
//		ROS_WARN("%s %d: rcon(%d).", __FUNCTION__, __LINE__, (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT));
//		ROS_WARN("%s %d: ~rcon(%d).", __FUNCTION__, __LINE__, ~(RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT));
//		ROS_WARN("%s %d: rcon_status(%d).", __FUNCTION__, __LINE__, (get_rcon_status() & (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)));
		if (!(get_rcon_status() & (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT))){
			reset_rcon_status();
			return 0;
		}
		else
			return get_rcon_trig_();
	}
	else if (mt_is_linear()){
//		ROS_WARN("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		if (cm_is_exploration())
		{
			auto rcon_status = get_rcon_status() & RconAll_Home_T;
			reset_rcon_status();
			return rcon_status;
		}
		// Since we have front left 2 and front right 2 rcon receiver, seems it is not necessary to handle left or right rcon receives home signal.
		else if (!(get_rcon_status() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT))){
			reset_rcon_status();
			return 0;
		}
		else
			return get_rcon_trig_();
	}
	else if (mt_is_go_to_charger())
	{
		auto rcon_status = get_rcon_status();
		reset_rcon_status();
		return rcon_status;
	}

	return 0;
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
void set_home_remote(void)
{
	g_home_remote_flag = 1;
}

void reset_home_remote(void)
{
	g_home_remote_flag = 0;
}

uint8_t is_home_remote(void)
{
	return g_r_h_flag;
}

uint8_t is_move_with_remote(void)
{
	return g_remote_move_flag;
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
			ROS_INFO("%s %d: Update the last_reg_type.", __FUNCTION__, __LINE__);
			left_pid.actual_speed = 0;
			right_pid.actual_speed = 0;
			left_pid.last_reg_type = right_pid.last_reg_type = argu_for_pid.reg_type;
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
	control_set(CTL_WHEEL_LEFT_HIGH, (l_speed >> 8) & 0xff);
	control_set(CTL_WHEEL_LEFT_LOW, l_speed & 0xff);

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
	control_set(CTL_WHEEL_RIGHT_HIGH, (r_speed >> 8) & 0xff);
	control_set(CTL_WHEEL_RIGHT_LOW, r_speed & 0xff);
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
		// Set the vacuum to a normal mode
		set_vacmode(Vac_Normal, false);
		set_vac_speed();
	} else {
		set_vacmode(Vac_Save);
		set_vac_speed();
	}

	// Trun on the main brush and side brush
	set_side_brush_pwm(50, 50);
	set_main_brush_pwm(30);
}

uint8_t check_motor_current(void)
{
	static uint8_t lwheel_oc_count = 0;
	static uint8_t rwheel_oc_count = 0;
	static uint8_t vacuum_oc_count = 0;
	static uint8_t mbrush_oc_count = 0;
	if ((uint32_t) robot::instance()->getLwheelCurrent() > Wheel_Stall_Limit)
	{
		lwheel_oc_count++;
		if (lwheel_oc_count > 40)
		{
			lwheel_oc_count = 0;
			ROS_WARN("%s,%d,left wheel over current,%u mA\n", __FUNCTION__, __LINE__,
							 (uint32_t) robot::instance()->getLwheelCurrent());
			return Check_Left_Wheel;
		}
	} else
		lwheel_oc_count = 0;
	if ((uint32_t) robot::instance()->getRwheelCurrent() > Wheel_Stall_Limit)
	{
		rwheel_oc_count++;
		if (rwheel_oc_count > 40)
		{
			rwheel_oc_count = 0;
			ROS_WARN("%s,%d,right wheel over current,%u mA", __FUNCTION__, __LINE__,
							 (uint32_t) robot::instance()->getRwheelCurrent());
			return Check_Right_Wheel;
		}
	} else
		rwheel_oc_count = 0;
	if (robot::instance()->getMbrushOc())
	{
		mbrush_oc_count++;
		if (mbrush_oc_count > 40)
		{
			mbrush_oc_count = 0;
			ROS_WARN("%s,%d,main brush over current", __FUNCTION__, __LINE__);
			return Check_Main_Brush;
		}
	}
	if (robot::instance()->getVacuumOc())
	{
		vacuum_oc_count++;
		if (vacuum_oc_count > 40)
		{
			vacuum_oc_count = 0;
			ROS_WARN("%s,%d,vacuum over current", __FUNCTION__, __LINE__);
			return Check_Vacuum;
		}
	}
	return 0;
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
			set_main_brush_pwm(0);
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

uint8_t check_bat_home(void)
{
	// Check if battary is lower than the low battery go home voltage value.
	if (get_battery_voltage() > 0 && get_battery_voltage() < LOW_BATTERY_GO_HOME_VOLTAGE)
	{
		return 1;
	}
	return 0;
}

uint8_t check_bat_full(void)
{
	// Check if battary is higher than the battery full voltage value.
	if (get_battery_voltage() > BATTERY_FULL_VOLTAGE)
	{
		return 1;
	}
	return 0;
}

uint8_t check_bat_ready_to_clean(void)
{
	uint16_t battery_limit;
	if (cm_get() == Clean_Mode_Charging)
	{
		battery_limit = BATTERY_READY_TO_CLEAN_VOLTAGE + 60;
	} else
	{
		battery_limit = BATTERY_READY_TO_CLEAN_VOLTAGE;
	}
	//ROS_INFO("%s %d: Battery limit is %d.", __FUNCTION__, __LINE__, battery_limit);
	// Check if battary is lower than the low battery go home voltage value.
	if (get_battery_voltage() >= battery_limit)
	{
		return 1;
	}
	return 0;
}

//uint8_t cm_get(void)
//{
//	return g_cleaning_mode;
//}

void set_vacmode(uint8_t mode, bool is_save)
{
	// Set the mode for vacuum.
	// The data should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_NormalL.
	g_vac_mode = g_vac_mode_save;
	if (mode != Vac_Save)
	{
		g_vac_mode = mode;
		if (is_save)
			g_vac_mode_save = g_vac_mode;
	}

	ROS_INFO("%s ,%d g_vac_mode(%d),g_vac_mode_save(%d)", __FUNCTION__, __LINE__, g_vac_mode, g_vac_mode_save);
}

void set_bldc_speed(uint32_t S)
{
	// Set the power of BLDC, S should be in range(0, 100).
	S = S < 100 ? S : 100;
	control_set(CTL_VACCUM_PWR, S & 0xff);
}

void set_vac_speed(void)
{
	// Set the power of BLDC according to different situation
	// Stop the BLDC if rGobot carries the water tank
	if (is_water_tank())
	{
		set_bldc_speed(0);
	} else
	{
		// Set the BLDC power to max if robot in max mode
		if (get_vac_mode() == Vac_Max)
		{
			set_bldc_speed(Vac_Speed_Max);
		} else
		{
			// If work time less than 2 hours, the BLDC should be in normal level, but if more than 2 hours, it should slow down a little bit.
			if (get_work_time() < Two_Hours)
			{
				set_bldc_speed(Vac_Speed_Normal);
			} else
			{
				//ROS_INFO("%s %d: Work time more than 2 hours.", __FUNCTION__, __LINE__);
				set_bldc_speed(Vac_Speed_NormalL);
			}
		}
	}
}

//--------------------------------------Obs Dynamic adjust----------------------
void obs_dynamic_base(uint16_t count)
{
//	count = 20;
//	enum {front,left,right};
	static uint16_t obs_cnt[] = {0,0,0};
	static int32_t obs_sum[] = {0,0,0};
	const int16_t obs_dynamic_limit = 2000;
	int16_t* p_obs_baseline[] = {&g_obs_front_baseline, &g_obs_left_baseline, &g_obs_right_baseline};
	typedef int16_t(*Func_t)(void);
	Func_t p_get_obs[] = {&get_front_obs,&get_left_obs,&get_right_obs};
//	if(count == 0)
//		return ;
	for(int i =0;i<3;i++)
	{
//		if(i == 0)
//			ROS_WARN("front-------------------------");
//		if(i == 1)
//			ROS_WARN("left-------------------------");
//		if(i == 2)
//			ROS_WARN("right-------------------------");

		auto p_obs_baseline_ = p_obs_baseline[i];
		auto obs_get = p_get_obs[i]();

//		ROS_WARN("obs_trig_val(%d),obs_get(%d)", *p_obs_baseline_, obs_get);
		obs_sum[i] += obs_get;
		obs_cnt[i]++;
		int16_t obs_avg = obs_sum[i] / obs_cnt[i];
//		ROS_WARN("i = %d, obs_avg(%d), (%d / %d), ",i, obs_avg, obs_sum[i], obs_cnt[i]);
		auto diff = abs_minus(obs_avg , obs_get);
		if (diff > 50)
		{
//			ROS_WARN("i = %d, diff = (%d) > 50.", i, diff);
			obs_cnt[i] = 0;
			obs_sum[i] = 0;
		}
		if (obs_cnt[i] > count)
		{
			obs_cnt[i] = 0;
			obs_sum[i] = 0;
			obs_get = obs_avg / 2 + *p_obs_baseline_;
			if (obs_get > obs_dynamic_limit)
				obs_get = obs_dynamic_limit;

			*p_obs_baseline_ = obs_get;
//			if(i == 0)
//				ROS_WARN("obs front baseline = %d.", *p_obs_baseline_);
//			else if(i == 1)
//				ROS_WARN("obs left baseline = %d.", *p_obs_baseline_);
//			else if(i == 2)
//				ROS_WARN("obs right baseline = %d.", *p_obs_baseline_);
		}
	}
}

int16_t get_front_obs_trig_value(void)
{
	return obs_front_trig_value;
}

int16_t get_left_obs_trig_value(void)
{
	return obs_left_trig_value;
}

int16_t get_right_obs_trig_value(void)
{
	return obs_right_trig_value;
}

int16_t get_front_obs(void)
{
	return robot::instance()->getObsFront();
}

int16_t get_left_obs(void)
{
	return robot::instance()->getObsLeft();
}

int16_t get_right_obs(void)
{
	return robot::instance()->getObsRight();
}

uint8_t get_obs_status(int16_t left_obs_offset, int16_t front_obs_offset, int16_t right_obs_offset)
{
	uint8_t status = 0;

	if (get_left_obs() > get_left_obs_trig_value() + left_obs_offset)
		status |= BLOCK_LEFT;

	if (get_front_obs() > get_front_obs_trig_value() + front_obs_offset)
		status |= BLOCK_FRONT;

	if (get_right_obs() > get_right_obs_trig_value() + right_obs_offset)
		status |= BLOCK_RIGHT;

	return status;
}

void move_forward(uint8_t Left_Speed, uint8_t Right_Speed)
{
	set_dir_forward();
	set_wheel_speed(Left_Speed, Right_Speed);
}

uint8_t get_vac_mode(void)
{
	// Return the vacuum mode
	return g_vac_mode;
}

void set_vac_mode(uint8_t mode)
{
	if(mode <= Vac_Save)
		g_vac_mode = mode;
	else{
		ROS_ERROR("%s,%d, variable error",__FUNCTION__,__LINE__);
	}
}

void switch_vac_mode(bool is_save)
{
	// Switch the vacuum mode between Max and Normal
	if (get_vac_mode() == Vac_Normal)
	{
		set_vacmode(Vac_Max, is_save);
	} else
	{
		set_vacmode(Vac_Normal, is_save);
	}
	// Process the vacuum mode
	set_vac_speed();
}

void set_rcon_status(uint32_t code)
{
	movement_rcon_status = code;
}

void reset_rcon_status(void)
{
	movement_rcon_status = 0;
}

uint32_t get_rcon_status()
{
	extern Cell_t g_stub_cell;
	if(!cs_is_going_home() && g_from_station && g_motion_init_succeeded && !mt_is_go_to_charger()  && !mt_is_follow_wall()){//check if robot start from charge station
		if(two_points_distance(g_stub_cell.X,g_stub_cell.Y,map_get_x_cell(),map_get_y_cell()) <= 20){
			g_in_charge_signal_range = true;
			reset_rcon_status();
			return 0;
		}
		else{
			g_in_charge_signal_range = false;
			return movement_rcon_status;
		}
	}
	else
		return movement_rcon_status;
}

/*----------------------------------------Remote--------------------------------*/
uint8_t remote_key(uint8_t key)
{
	// Debug
	if (g_remote_status > 0)
	{
		ROS_DEBUG("%s, %d g_remote_status = %x", __FUNCTION__, __LINE__, g_remote_status);
	}
	if (g_remote_status & key)
	{
		return 1;
	} else
	{
		return 0;
	}
}

void set_rcon_remote(uint8_t cmd)
{
	g_remote_status |= cmd;
}

void reset_rcon_remote(void)
{
	g_remote_status = 0;
}

uint8_t get_rcon_remote(void)
{
	return g_remote_status;
}

void set_move_with_remote(void)
{
	g_remote_move_flag = 1;
}

void reset_move_with_remote(void)
{
	g_remote_move_flag = 0;
}

uint8_t check_bat_set_motors(uint32_t Vacuum_Voltage, uint32_t Side_Brush, uint32_t Main_Brush)
{
	static uint8_t low_acc = 0;
	if (check_bat_stop())
	{
		if (low_acc < 255)
			low_acc++;
		if (low_acc > 50)
		{
			low_acc = 0;
			uint16_t t_vol = get_battery_voltage();
			uint8_t v_pwr = Vacuum_Voltage / t_vol;
			uint8_t s_pwr = Side_Brush / t_vol;
			uint8_t m_pwr = Main_Brush / t_vol;

			set_bldc_speed(v_pwr);
			set_side_brush_pwm(s_pwr, s_pwr);
			set_main_brush_pwm(m_pwr);
			return 1;

		} else
			return 0;
	} else
	{
		return 0;
	}
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
	control_set(CTL_LED_RED, R & 0xff);
	control_set(CTL_LED_GREEN, G & 0xff);
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

void set_main_brush_pwm(uint16_t PWM)
{
	// Set main brush PWM, the value of PWM should be in range (0, 100).
	PWM = PWM < 100 ? PWM : 100;
	control_set(CTL_BRUSH_MAIN, PWM & 0xff);
}

void set_side_brush_pwm(uint16_t L, uint16_t R)
{
	// Set left and right brush PWM, the value of L/R should be in range (0, 100).
	L = L < 100 ? L : 100;
	g_l_brush_pwm = L;
	control_set(CTL_BRUSH_LEFT, L & 0xff);
	R = R < 100 ? R : 100;
	g_r_brush_pwm = R;
	control_set(CTL_BRUSH_RIGHT, R & 0xff);
}

void set_left_brush_pwm(uint16_t L)
{
	L = L < 100 ? L : 100;
	control_set(CTL_BRUSH_LEFT, L & 0xff);
}

void set_right_brush_pwm(uint16_t R)
{
	R = R < 100 ? R : 100;
	control_set(CTL_BRUSH_RIGHT, R & 0xff);
}

uint8_t get_left_brush_stall(void)
{
	return 0;
}

uint8_t get_right_brush_stall(void)
{
	return 0;
}


uint8_t get_touch_status(void)
{
	return g_touch_status;
}

void reset_touch(void)
{
	g_touch_status = 0;
}

void set_touch(void)
{
	g_touch_status = 1;
}

void reset_stop_event_status(void)
{
	g_stop_event_status = 0;
	// For key release checking.
	reset_touch();
}

uint8_t stop_event(void)
{
	// If it has already had a g_stop_event_status, then no need to check.
	if (!g_stop_event_status)
	{
		// Get the key value from robot sensor
		if (get_touch_status())
		{
			ROS_WARN("Touch status == 1");
#if MANUAL_PAUSE_CLEANING
			if (cm_is_navigation())
				robot::instance()->setManualPause();
#endif
			reset_touch();
			g_stop_event_status = 1;
		}
		if (remote_key(Remote_Clean))
		{
			ROS_WARN("remote_key clean.");
			reset_rcon_remote();
#if MANUAL_PAUSE_CLEANING
			if (cm_is_navigation())
				robot::instance()->setManualPause();
#endif
			g_stop_event_status = 2;
		}
		if (get_cliff_status() == 0x07)
		{
			ROS_WARN("Cliff triggered.");
			g_stop_event_status = 3;
		}

		if (get_error_code())
		{
			ROS_WARN("Detects Error: %x!", get_error_code());
			if (get_error_code() == Error_Code_Slam)
			{
				stop_brifly();
				// Check if it is really stopped.
				uint8_t slam_error_count = 0;
				tf::StampedTransform transform;
				for (uint8_t i = 0; i < 3; i++)
				{
					try
					{
						robot::instance()->robot_tf_->lookupTransform("/map", "base_link", ros::Time(0), transform);
					} catch (tf::TransformException e)
					{
						ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
						slam_error_count++;
					}
					if (slam_error_count > 0)
						break;
					i++;
					usleep(20000);
				}
				if (slam_error_count > 0)
				{
					// beep for debug
					//beep(3, 500, 500, 3);
					system("rosnode kill /slam_karto &");
					usleep(3000000);
					system("roslaunch pp karto_slam.launch &");
					robotbase_restore_slam_correction();
					MotionManage::s_slam->isMapReady(false);
					while (!MotionManage::s_slam->isMapReady())
					{
						ROS_WARN("Slam not ready yet.");
						MotionManage::s_slam->enableMapUpdate();
						usleep(500000);
					}
					ROS_WARN("Slam restart successed.");
					// Wait for 0.5s to make sure it has process the first scan.
					usleep(500000);
				}
				set_error_code(Error_Code_None);
			} else
			{
				g_stop_event_status = 4;
			}
		}
		if (is_direct_charge())
		{
			ROS_WARN("Detect direct charge!");
			g_stop_event_status = 5;
		}
	}
	return g_stop_event_status;
}

uint8_t is_station(void)
{
	if (get_rcon_status() & RconAll_Home_TLR) // It means eight rcon accepters receive any of the charger stub signal.
	{
		return 1;
	}
	return 0;
}

bool is_charge_on(void)
{
	// 1: On charger stub and charging.
	// 4: Direct connect to charge line and charging.
	if (robot::instance()->getChargeStatus() == 1 || robot::instance()->getChargeStatus() == 4)
		return true;
	else
		return false;
}

uint8_t is_water_tank(void)
{
	return 0;
}


//void cm_set(uint8_t mode)
//{
//	g_cleaning_mode = mode;
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

void initialize_motor(void)
{
#ifdef BLDC_INSTALL
	Clear_BLDC_Fail();
	BLDC_OFF;
	delay(5000);
	Set_BLDC_TPWM(40);
	Set_Vac_Speed();
#endif
	set_main_brush_pwm(50);
	set_side_brush_pwm(60, 60);
	set_bldc_speed(40);
//	move_forward(0,0);
	stop_brifly();
//	Left_Wheel_Slow=0;
//	Right_Wheel_Slow=0;
//	reset_bumper_error();
}

void disable_motors(void)
{
	// Disable all the motors, including brush, wheels, and vacuum.
	// Stop the wheel
	set_wheel_speed(0, 0);
	// Stop the side brush
	set_side_brush_pwm(0, 0);
	// Stop the main brush
	set_main_brush_pwm(0);
	// Stop the vacuum, directly stop the BLDC
	set_bldc_speed(0);
}

void set_start_charge(void)
{
	// This function will turn on the charging function.
	control_set(CTL_CHARGER, 0x01);
}

void set_stop_charge(void)
{
	// Set the flag to false so that it can quit charger mode.
	control_set(CTL_CHARGER, 0x00);
}

void set_main_pwr_byte(uint8_t val)
{
	control_set(CTL_MAIN_PWR, val & 0xff);
}

uint8_t get_main_pwr_byte()
{
	return control_get(CTL_MAIN_PWR);
}

void set_clean_tool_power(uint8_t vacuum_val, uint8_t left_brush_val, uint8_t right_brush_val, uint8_t main_brush_val)
{
	int vacuum_pwr = vacuum_val;
	vacuum_pwr = vacuum_pwr > 0 ? vacuum_pwr : 0;
	vacuum_pwr = vacuum_pwr < 100 ? vacuum_pwr : 100;
	control_set(CTL_VACCUM_PWR, vacuum_pwr & 0xff);

	int brush_left = left_brush_val;
	control_set(CTL_BRUSH_LEFT, brush_left & 0xff);

	int brush_right = right_brush_val;
	control_set(CTL_BRUSH_RIGHT, brush_right & 0xff);

	int brush_main = main_brush_val;
	control_set(CTL_BRUSH_MAIN, brush_main & 0xff);
}

void start_self_check_vacuum(void)
{
	uint8_t omni_reset_byte = control_get(CTL_OMNI_RESET);
	control_set(CTL_OMNI_RESET, omni_reset_byte | 0x02);
}

void end_self_check_vacuum(void)
{
	uint8_t omni_reset_byte = control_get(CTL_OMNI_RESET);
	control_set(CTL_OMNI_RESET, omni_reset_byte | 0x04);
}

void reset_self_check_vacuum_controler(void)
{
	uint8_t omni_reset_byte = control_get(CTL_OMNI_RESET);
	control_set(CTL_OMNI_RESET, omni_reset_byte & ~0x06);
}

void control_set(uint8_t type, uint8_t val)
{
	set_send_flag();
	if (type >= CTL_WHEEL_LEFT_HIGH && type <= CTL_GYRO)
	{
		g_send_stream[type] = val;
	}
	reset_send_flag();
}

uint8_t control_get(uint8_t seq)
{
	uint8_t tmp_data;
	g_send_stream_mutex.lock();
	tmp_data = g_send_stream[seq];
	g_send_stream_mutex.unlock();
	return tmp_data;
}

void control_append_crc()
{
	//set_send_flag();
	uint8_t tmp_data;
	uint8_t buf[SEND_LEN];
	g_send_stream_mutex.lock();
	memcpy(buf, g_send_stream, sizeof(uint8_t) * SEND_LEN);
	g_send_stream_mutex.unlock();
	tmp_data = calc_buf_crc8(buf, SEND_LEN - 3);
	control_set(CTL_CRC, tmp_data);
	//reset_send_flag();
}

void control_stop_all(void)
{
	uint8_t i;
	//set_send_flag();
	for (i = 2; i < (SEND_LEN) - 2; i++)
	{
		if (i == CTL_MAIN_PWR)
			control_set(i, 0x01);
		else
			control_set(i, 0x00);
	}
	//reset_send_flag();
	//g_send_stream[SEND_LEN-3] = calc_buf_crc8(g_send_stream, SEND_LEN-3);
	//serial_write(SEND_LEN, g_send_stream);
}

int control_get_sign(uint8_t *key, uint8_t *sign, uint8_t key_length, int sequence_number)
{
	int num_send_packets = key_length / KEY_DOWNLINK_LENGTH;
	uint8_t ptr[RECEI_LEN], buf[SEND_LEN];

	//Set random seed.
	//srand(time(NULL));
	//Send random key to robot.
	for (int i = 0; i < num_send_packets; i++)
	{
		//Populate dummy.
		for (int j = 0; j < DUMMY_DOWNLINK_LENGTH; j++)
			control_set(j + DUMMY_DOWNLINK_OFFSET, (uint8_t) (rand() % 256));

		//Populate Sequence number.
		for (int j = 0; j < SEQUENCE_DOWNLINK_LENGTH; j++)
			control_set(j + DUMMY_DOWNLINK_OFFSET, (uint8_t) ((sequence_number >> 8 * j) % 256));

		//Populate key.
#if VERIFY_DEBUG
		printf("appending key: ");
#endif

		for (int k = 0; k < KEY_DOWNLINK_LENGTH; k++)
		{
			control_set(k + KEY_DOWNLINK_OFFSET, key[i * KEY_DOWNLINK_LENGTH + k]);

#if VERIFY_DEBUG
			printf("%02x ", g_send_stream[k + KEY_DOWNLINK_OFFSET]);
			if (k == KEY_DOWNLINK_LENGTH - 1)
				printf("\n");
#endif

		}

		//Fill command field
		switch (i)
		{
			case 0:
				control_set(SEND_LEN - 4, CMD_KEY1);
				break;
			case 1:
				control_set(SEND_LEN - 4, CMD_KEY2);
				break;
			case 2:
				control_set(SEND_LEN - 4, CMD_KEY3);
				break;
			default:

#if VERIFY_DEBUG
				printf("control_get_sign : Error! key_length too large.");
#endif

				return -1;
				//break;
		}

		for (int i = 0; i < 40; i++)
		{  //200ms (round trip takes at leat 15ms)
			int counter = 0, ret;

			g_send_stream_mutex.lock();
			memcpy(buf, g_send_stream, sizeof(uint8_t) * SEND_LEN);
			g_send_stream_mutex.unlock();
			buf[CTL_CRC] = calc_buf_crc8(buf, SEND_LEN - 3);
			serial_write(SEND_LEN, buf);

#if VERIFY_DEBUG
			printf("sending data to robot: i: %d\n", i);
			for (int j = 0; j < SEND_LEN; j++) {
				printf("%02x ", buf[j]);
			}
			printf("\n");
#endif

			while (counter < 200)
			{

				ret = serial_read(1, ptr);
				if (ptr[0] != 0xAA)
					continue;

				ret = serial_read(1, ptr);
				if (ptr[0] != 0x55)
					continue;

				ret = serial_read(RECEI_LEN - 2, ptr);
				if (RECEI_LEN - 2 != ret)
				{

#if VERIFY_DEBUG
					printf("%s %d: receive count error: %d\n", __FUNCTION__, __LINE__, ret);
#endif

					usleep(100);
					counter++;
				} else
				{
					break;
				}
			}
			if (counter < 200)
			{

#if VERIFY_DEBUG
				printf("%s %d: counter: %d\tdata count: %d\treceive cmd: 0x%02x\n", __FUNCTION__, __LINE__, counter, ret, ptr[CMD_UPLINK_OFFSET]);

				printf("receive from robot: %d\n");
				for (int j = 0; j < RECEI_LEN - 2; j++) {
					printf("%02x ", ptr[j]);
				}
				printf("\n");
#endif

				if (ptr[CMD_UPLINK_OFFSET - 2] == CMD_NCK)   //robot received bronen packet
					continue;

				if (ptr[CMD_UPLINK_OFFSET - 2] == CMD_ACK)
				{  //set finished
					//printf("Downlink command ACKed!!\n");
					control_set(CTL_CMD, 0x00);
					break;
				}
			} else
			{

#if VERIFY_DEBUG
				printf("%s %d: max read count reached: %d\n", counter);
#endif

			}
			usleep(500);
		}
	}

	//Block and wait for signature.
	for (int i = 0; i < 400; i++)
	{                              //200ms (round trip takes at leat 15ms)
		int counter = 0, ret;
		while (counter < 400)
		{
			ret = serial_read(1, ptr);
			if (ptr[0] != 0xAA)
				continue;

			ret = serial_read(1, ptr);
			if (ptr[0] != 0x55)
				continue;

			ret = serial_read(RECEI_LEN - 2, ptr);

#if VERIFY_DEBUG
			printf("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, ret, RECEI_LEN - 2, counter);
#endif

			if (RECEI_LEN - 2 != ret)
			{
				usleep(100);
				counter++;
			} else
			{
				break;
			}
		}

#if VERIFY_DEBUG
		for (int j = 0; j < RECEI_LEN - 2; j++) {
			printf("%02x ", ptr[j]);
		}
		printf("\n");
#endif

		if (counter < 400 && ptr[CMD_UPLINK_OFFSET - 3] == CMD_ID)
		{
			//set finished

#if VERIFY_DEBUG
			printf("Signature received!!\n");
#endif

			for (int j = 0; j < KEY_UPLINK_LENGTH; j++)
				sign[j] = ptr[KEY_UPLINK_OFFSET - 2 + j];

			//Send acknowledge back to MCU.
			control_set(CTL_CMD, CMD_ACK);
			for (int k = 0; k < 20; k++)
			{
				g_send_stream_mutex.lock();
				memcpy(buf, g_send_stream, sizeof(uint8_t) * SEND_LEN);
				g_send_stream_mutex.unlock();
				buf[CTL_CRC] = calc_buf_crc8(buf, SEND_LEN - 3);
				serial_write(SEND_LEN, buf);

				usleep(500);
			}
			control_set(CTL_CMD, 0x00);

#if VERIFY_DEBUG
			printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

			return KEY_UPLINK_LENGTH;
		}
		usleep(500);
	}
	control_set(CTL_CMD, 0x00);

#if VERIFY_DEBUG
	printf("%s %d: exit\n", __FUNCTION__, __LINE__);
#endif

	return -1;
}

uint8_t is_flag_set(void)
{
	return g_sendflag;
}

void set_send_flag(void)
{
	g_sendflag = 1;
}

void reset_send_flag(void)
{
	g_sendflag = 0;
}

void random_back(void)
{
	stop_brifly();
	quick_back(12, 30);

}

void move_back(void)
{
	stop_brifly();
	quick_back(18, 30);
}

void cliff_move_back()
{
	stop_brifly();
	quick_back(18, 60);
}

void set_right_wheel_step(uint32_t step)
{
	g_right_wheel_step = step;
}

void set_left_wheel_step(uint32_t step)
{
	g_left_wheel_step = step;
}

void reset_right_wheel_step()
{
	g_rw_t = ros::Time::now();
	g_right_wheel_step = 0;
}

void reset_left_wheel_step()
{
	g_lw_t = ros::Time::now();
	g_left_wheel_step = 0;
}

uint16_t get_battery_voltage()
{
	return robot::instance()->getBatteryVoltage();
}

uint8_t check_bat_stop()
{
	if (get_battery_voltage() < LOW_BATTERY_STOP_VOLTAGE)
		return 1;
	else
		return 0;
}

void set_key_press(uint8_t key)
{
	g_key_status |= key;
}

void reset_key_press(uint8_t key)
{
	g_key_status &= ~key;
}

uint8_t get_key_press(void)
{
	return g_key_status;
}

uint8_t is_virtual_wall(void)
{
	return 0;
}

uint8_t is_turn_remote(void)
{
	if (remote_key(Remote_Max | Remote_Home | Remote_Spot | Remote_Wall_Follow))
	{
		reset_rcon_remote();
		return 1;
	} else
	{
		return 0;
	}
}

uint8_t get_direction_flag(void)
{
	return g_direction_flag;
}

void set_direction_flag(uint8_t flag)
{
	g_direction_flag = flag;
}

uint8_t is_direction_right(void)
{
	if (get_direction_flag() == Direction_Flag_Right)return 1;
	return 0;
}

uint8_t is_direction_left(void)
{
	if (get_direction_flag() == Direction_Flag_Left)return 1;
	return 0;
}

uint8_t is_left_wheel_reach(int32_t step)
{
	if (g_left_wheel_step > (uint32_t) step)
		return 1;
	else
		return 0;
}

uint8_t is_right_wheel_reach(int32_t step)
{
	if (g_right_wheel_step > (uint32_t) step)
		return 1;
	else
		return 0;
}

void wall_move_back(void)
{
	uint16_t count = 0;
	uint16_t tp = 0;
	uint8_t mc = 0;
	set_dir_backward();
	set_wheel_speed(3, 3);
	usleep(20000);
	reset_wheel_step();
	while (((g_left_wheel_step < 100) || (g_right_wheel_step < 100)) && ros::ok())
	{
		tp = g_left_wheel_step / 3 + 8;
		if (tp > 12)tp = 12;
		set_wheel_speed(tp, tp);
		usleep(1000);

		if (stop_event())
			return;
		count++;
		if (count > 3000);
		return;
		mc = check_motor_current();
		if (mc == Check_Left_Wheel || mc == Check_Right_Wheel)
			return;
	}
	set_dir_forward();
	set_wheel_speed(0, 0);

}

void reset_move_distance()
{
	g_move_step_counter = 0;
}

uint8_t is_move_finished(int32_t distance)
{
	g_move_step_counter = get_left_wheel_step();
	g_move_step_counter += get_right_wheel_step();
	if ((g_move_step_counter / 2) > distance)
		return 1;
	else
		return 0;
}

uint32_t get_move_distance(void)
{
	g_move_step_counter = get_left_wheel_step();
	g_move_step_counter += get_right_wheel_step();

	if (g_move_step_counter > 0)return (uint32_t) (g_move_step_counter / 2);
	return 0;
}

void obs_turn_left(uint16_t speed, uint16_t angle)
{
	uint16_t counter = 0;
	uint8_t oc = 0;
	set_dir_left();
	reset_rcon_remote();
	set_wheel_speed(speed, speed);
	reset_left_wheel_step();
	while (ros::ok() && g_left_wheel_step < angle)
	{
		counter++;
		if (counter > 3000)
			return;
		if (is_turn_remote())
			return;
		oc = check_motor_current();
		if (oc == Check_Left_Wheel || oc == Check_Right_Wheel)
			return;
		usleep(1000);
	}

}

void obs_turn_right(uint16_t speed, uint16_t angle)
{
	uint16_t counter = 0;
	uint8_t oc = 0;
	set_dir_right();
	reset_rcon_remote();
	set_wheel_speed(speed, speed);
	reset_right_wheel_step();
	while (ros::ok() && g_right_wheel_step < angle)
	{
		counter++;
		if (counter > 3000)
			//return;
			break;
		if (is_turn_remote())
			//return;
			break;
		oc = check_motor_current();
		if (oc == Check_Left_Wheel || oc == Check_Right_Wheel)
			//return;
			break;
		usleep(1000);
	}

}

void cliff_turn_left(uint16_t speed, uint16_t angle)
{
	uint16_t Counter_Watcher = 0;
	//Left_Wheel_Step=0;
	set_wheel_speed(speed, speed);
	Counter_Watcher = 0;
	reset_rcon_remote();
	int16_t target_angle;
	// This decides whether robot should stop when left cliff triggered.
	bool right_cliff_triggered = false;

	if (get_cliff_status() & BLOCK_RIGHT)
	{
		right_cliff_triggered = true;
	}

	target_angle = ranged_angle(gyro_get_angle() + angle);

	set_dir_left();
	set_wheel_speed(speed, speed);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle,
					 gyro_get_angle(), speed);
	while (ros::ok())
	{
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < 20)
		{
			break;
		}
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < 50)
		{
			set_wheel_speed(speed / 2, speed / 2);
		} else
		{
			set_wheel_speed(speed, speed);
		}
		//delay(10);
		usleep(1000);
		Counter_Watcher++;
		if (Counter_Watcher > 3000)
		{
			if (is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
			}
			return;
		}
		if (is_turn_remote())return;
		if (!right_cliff_triggered && (get_cliff_status() & BLOCK_RIGHT))
		{
			stop_brifly();
			return;
		}
		if (stop_event())
		{
			return;
		}
		if ((check_motor_current() == Check_Left_Wheel) || (check_motor_current() == Check_Right_Wheel))return;
	}
}

void cliff_turn_right(uint16_t speed, uint16_t angle)
{
	uint16_t Counter_Watcher = 0;
	//Left_Wheel_Step=0;
	set_wheel_speed(speed, speed);
	Counter_Watcher = 0;
	reset_rcon_remote();
	int16_t target_angle;
	// This decides whether robot should stop when left cliff triggered.
	bool left_cliff_triggered = false;

	if (get_cliff_status() & BLOCK_LEFT)
	{
		left_cliff_triggered = true;
	}

	target_angle = ranged_angle(gyro_get_angle() - angle);

	set_dir_right();
	set_wheel_speed(speed, speed);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle,
					 gyro_get_angle(), speed);
	while (ros::ok())
	{
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < 20)
		{
			break;
		}
		if (abs(ranged_angle(target_angle - gyro_get_angle())) < 50)
		{
			set_wheel_speed(speed / 2, speed / 2);
		} else
		{
			set_wheel_speed(speed, speed);
		}
		//delay(10);
		usleep(1000);
		Counter_Watcher++;
		if (Counter_Watcher > 3000)
		{
			if (is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
			}
			return;
		}
		if (is_turn_remote())return;
		if (!left_cliff_triggered && (get_cliff_status() & BLOCK_LEFT))
		{
			stop_brifly();
			return;
		}
		if (stop_event())
		{
			return;
		}
		if ((check_motor_current() == Check_Left_Wheel) || (check_motor_current() == Check_Right_Wheel))return;
	}
}

uint8_t get_random_factor(void)
{
	srand(time(0));
	return (uint8_t) rand() % 100;
}

uint8_t is_near_station(void)
{
	static uint32_t s_count = 0;
	static uint32_t no_s = 0;
	static uint32_t s_rcon = 0;
	s_rcon = get_rcon_status();
	if (s_rcon & 0x00000f00)
	{
		return 1;
	}
	if (s_rcon & 0x0330ff)
	{
		no_s = 0;
		s_count++;
		reset_rcon_status();
		if (s_count > 3)
		{
			s_count = 0;
			return 1;
		}
	} else
	{
		no_s++;
		if (no_s > 50)
		{
			no_s = 0;
			s_count = 0;
		}
	}
	return 0;
}

void set_mobility_step(uint32_t Steps)
{
	g_mobility_step = Steps;
}

void reset_mobility_step()
{
	uint8_t omni_reset_byte = control_get(CTL_OMNI_RESET);
	control_set(CTL_OMNI_RESET, omni_reset_byte | 0x01);
}

void clear_reset_mobility_step()
{
	uint8_t omni_reset_byte = control_get(CTL_OMNI_RESET);
	control_set(CTL_OMNI_RESET, omni_reset_byte & ~0x01);
}

uint32_t get_mobility_step()
{
	return g_mobility_step;
}

void check_mobility(void)
{

}

void add_average(uint32_t data)
{
	g_average_move += data;
	g_average_counter++;
	if (data > g_max_move)
		g_max_move = data;

}

uint32_t get_average_move(void)
{
	return (g_average_move / g_average_counter);
}

uint32_t reset_average_counter(void)
{
	g_average_move = 0;
	g_average_counter = 0;
}

void reset_virtual_wall(void)
{

}

uint8_t virtual_wall_turn_right()
{
	return 0;
}

uint8_t virtual_wall_turn_left()
{
	return 0;
}

uint8_t is_work_finish(uint8_t m)
{
	static uint8_t bat_count = 0;
	uint32_t wt = get_work_time();
	if (m)
	{
		if (wt > g_auto_work_time)return 1;
	} else if (wt > Const_Work_Time)return 1;
	if (get_battery_voltage() < 1420)
	{
		bat_count++;
		if (bat_count > 50)return 1;
	} else
		bat_count = 0;
	return 0;
}

uint8_t get_room_mode()
{
	return g_room_mode;
}

void set_room_mode(uint8_t m)
{
	if (m)
		g_room_mode = 1;
	else
		g_room_mode = 0;
}

void reset_wall_accelerate()
{
	g_wall_accelerate = 0;
}

uint32_t get_wall_accelerate()
{
	return g_wall_accelerate = get_right_wheel_step();
}

uint8_t is_virtual_wall_()
{
	return 0;
}

int32_t abs_minus(int32_t A, int32_t B)
{
	if (A > B)
	{
		return A - B;
	}
	return B - A;
}

uint8_t check_left_brush_stall(void)
{
	static time_t time_lbrush;
	static uint8_t lbrush_error_counter = 0;
	static uint8_t left_brush_status = 1;
	/*---------------------------------Left Brush Stall---------------------------------*/
	if (g_reset_lbrush_oc)
	{
		lbrush_error_counter = 0;
		left_brush_status = 1;
		g_reset_lbrush_oc = false;
		//ROS_WARN("%s %d: Reset left brush.", __FUNCTION__, __LINE__);
		return 0;
	}

	switch (left_brush_status)
	{
		case 1:
		{
			if (robot::instance()->getLbrushOc())
			{
				if (g_oc_brush_left_cnt < 200)
					g_oc_brush_left_cnt++;
			}
			else
				g_oc_brush_left_cnt = 0;

			if (g_oc_brush_left_cnt > 10)
			{
				/*-----Left Brush is stall, stop the brush-----*/
				set_left_brush_pwm(0);
				left_brush_status = 2;
				time_lbrush = time(NULL);
				ROS_WARN("%s %d: Stop the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 2:
		{
			/*-----brush should stop for 5s-----*/
			if ((time(NULL) - time_lbrush) >= 5)
			{
				// Then restart brush and let it fully operated.
				set_left_brush_pwm(100);
				left_brush_status = 3;
				time_lbrush = time(NULL);
				ROS_WARN("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}

		case 3:
		{
			if (robot::instance()->getLbrushOc())
			{
				if (g_oc_brush_left_cnt < 200)
					g_oc_brush_left_cnt++;
			} else
			{
				g_oc_brush_left_cnt = 0;
			}

			if (g_oc_brush_left_cnt > 10)
			{
				/*-----Brush is still stall, stop the brush and increase error counter -----*/
				set_left_brush_pwm(0);
				left_brush_status = 2;
				time_lbrush = time(NULL);
				lbrush_error_counter++;
				if (lbrush_error_counter > 2)
				{
					left_brush_status = 1;
					g_oc_brush_left_cnt = 0;
					lbrush_error_counter = 0;
					return 1;
				}
				break;
			}
			else
			{
				ROS_DEBUG("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
				if ((time(NULL) - time_lbrush) >= 5)
				{
					ROS_WARN("%s %d: Restore from fully operated.", __FUNCTION__, __LINE__);
					/*-----brush is in max mode more than 5s, turn to normal mode and reset error counter-----*/
					set_left_brush_pwm(g_l_brush_pwm);
					left_brush_status = 1;
					g_oc_brush_left_cnt = 0;
					lbrush_error_counter = 0;
				}
			}
			break;
		}
	}
	return 0;
}
uint8_t check_right_brush_stall(void)
{
	static time_t time_rbrush;
	static uint8_t rbrush_error_counter = 0;
	static uint8_t right_brush_status = 1;
	/*---------------------------------Left Brush Stall---------------------------------*/

	if (g_reset_rbrush_oc)
	{
		rbrush_error_counter = 0;
		right_brush_status = 1;
		g_reset_rbrush_oc = false;
		//ROS_WARN("%s %d: Reset right brush.", __FUNCTION__, __LINE__);
		return 0;
	}

	switch (right_brush_status)
	{
		case 1:
		{
			if (robot::instance()->getRbrushOc())
			{
				if (g_oc_brush_right_cnt < 200)
					g_oc_brush_right_cnt++;
			}
			else
				g_oc_brush_right_cnt = 0;

			if (g_oc_brush_right_cnt > 10)
			{
				/*-----Left Brush is stall, stop the brush-----*/
				set_right_brush_pwm(0);
				right_brush_status = 2;
				time_rbrush = time(NULL);
				ROS_WARN("%s %d: Stop the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 2:
		{
			/*-----brush should stop for 5s-----*/
			if ((time(NULL) - time_rbrush) >= 5)
			{
				// Then restart brush and let it fully operated.
				set_right_brush_pwm(100);
				right_brush_status = 3;
				time_rbrush = time(NULL);
				ROS_WARN("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
			}
			break;
		}

		case 3:
		{
			if (robot::instance()->getRbrushOc())
			{
				if (g_oc_brush_right_cnt < 200)
					g_oc_brush_right_cnt++;
			} else
			{
				g_oc_brush_right_cnt = 0;
			}

			if (g_oc_brush_right_cnt > 10)
			{
				/*-----Brush is still stall, stop the brush and increase error counter -----*/
				set_right_brush_pwm(0);
				right_brush_status = 2;
				time_rbrush = time(NULL);
				rbrush_error_counter++;
				if (rbrush_error_counter > 2)
				{
					right_brush_status = 1;
					g_oc_brush_right_cnt = 0;
					rbrush_error_counter = 0;
					return 1;
				}
				break;
			}
			else
			{
				ROS_DEBUG("%s %d: Fully operate the brush for 5s.", __FUNCTION__, __LINE__);
				if ((time(NULL) - time_rbrush) >= 5)
				{
					ROS_WARN("%s %d: Restore from fully operated.", __FUNCTION__, __LINE__);
					/*-----brush is in max mode more than 5s, turn to normal mode and reset error counter-----*/
					set_right_brush_pwm(g_r_brush_pwm);
					right_brush_status = 1;
					g_oc_brush_right_cnt = 0;
					rbrush_error_counter = 0;
				}
			}
			break;
		}
	}
	return 0;
}

void set_plan_status(uint8_t Status)
{
	g_plan_status = Status;
	if (g_plan_status != 0)
		ROS_DEBUG("Plan status return %d.", g_plan_status);
}

uint8_t get_plan_status()
{
	return g_plan_status;
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

void delay_sec(double s)
{
	auto start=ros::Time::now().toSec();
	auto now=start;
	while((now-start) < s)
	{
		now=ros::Time::now().toSec();
	}
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
		|| get_bumper_status() || get_tilt_status()
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
	if(robot::instance()->isManualPaused() || g_robot_stuck)
	{
		ret= true;
	}
	return ret;
}

void reset_clean_paused(void)
{
	if (robot::instance()->isManualPaused() || g_robot_stuck)
	{
		g_robot_stuck = false;
		// These are all the action that ~MotionManage() won't do if isManualPaused() returns true.
		ROS_WARN("Reset manual/stuck pause status.");
		wav_play(WAV_CLEANING_STOP);
		robot::instance()->resetManualPause();
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
	auto status = (robot::instance()->getObsFront() > get_front_obs_trig_value());
	return is_map_front_block(3) || status;
}

static int lidar_bumper_fd = -1;
static uint8_t is_lidar_bumper_init = 0;

int8_t lidar_bumper_init(const char* device)
{
	if(lidar_bumper_fd != -1)
		return 0;
	char buf[128];
	sprintf(buf,"%s",device);
	lidar_bumper_fd = open(buf, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(lidar_bumper_fd > 0)
	{
		is_lidar_bumper_init = 1;	
		ROS_INFO("%s,%d,open %s success!",__FUNCTION__,__LINE__,buf);
		return 1;
	}
	else if(lidar_bumper_fd <= 0 )
	{
		lidar_bumper_fd = -1;
		is_lidar_bumper_init = 0;
		ROS_INFO("%s,%d,no such file \033[32m %s \033[0m open fail!!",__FUNCTION__,__LINE__,buf);
		return -1;
	}

}

int8_t get_lidar_bumper_status()
{
	if(is_lidar_bumper_init && lidar_bumper_fd != -1)
	{
		uint8_t temp_buf[32] = {0};
		int reval;
		reval = read(lidar_bumper_fd,temp_buf,32);
		if(reval >=0){
			ROS_INFO("read lidar bumper value:\033[32m%d\033[0m",temp_buf[12]);
			return temp_buf[12];
		}
		else
			return -1;
	}
	return 0;
}

int8_t lidar_bumper_deinit()
{
	if(lidar_bumper_fd == -1)
		return 0;
	int c_ret;
	c_ret = close(lidar_bumper_fd);
	if(c_ret > 0)
	{
		lidar_bumper_fd = -1;
		is_lidar_bumper_init = 0;
	}
	return c_ret;
}

bool check_laser_stuck()
{
	if (MotionManage::s_laser != nullptr && !MotionManage::s_laser->laserCheckFresh(3, 2))
		return true;
	return false;
}

uint8_t estimate_charger_position(Rcon_Point_t rcon_point_a, Rcon_Point_t rcon_point_b,Cell_t *pos)
{
	float charger_dist_est_a,charger_dist_est_b;
	int32_t ax = rcon_point_a.x;//point a postition
	int32_t ay = rcon_point_a.y;
	float angle_a = (float)deg_to_rad(rcon_point_a.sensor_angle*1.0,1);

	int32_t bx = rcon_point_b.x;//point b position
	int32_t by = rcon_point_b.y;
	float angle_b = (float)deg_to_rad(rcon_point_b.sensor_angle*1.0,1);

	float diff_angle_a = atanf((ax - bx)/(ay - by));
	float diff_angle_b = PI - diff_angle_a;

	float angle_A = PI/2 - angle_a - diff_angle_a;
	float angle_B = PI - angle_b - diff_angle_b;

	uint32_t dist = two_points_distance(ax,ay,bx,by);

	charger_dist_est_a = dist * sinf(angle_B)/sinf(PI-(angle_B+angle_A)); 
	charger_dist_est_b = dist * sinf(angle_A)/sinf(PI-(angle_B+angle_A));

	pos->X = count_to_cell(cosf(angle_a)*charger_dist_est_a + ax);
	pos->Y = count_to_cell(sinf(angle_a)*charger_dist_est_a + ay);

	ROS_INFO("%s,%d,estimate charger stub position on (%d,%d)",__FUNCTION__,__LINE__,pos->X,pos->Y);
	return 1;
}
