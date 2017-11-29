#include <pthread.h>
#include <ros/ros.h>
#include <pp.h>
#include <battery.h>
#include <brush.h>
#include <bumper.h>
#include <clean_timer.h>
#include <remote.h>
#include <charger.h>
#include <beep.h>

#include "config.h"
#include "serial.h"

#include "movement.h"
#include "robot.hpp"
#include "robotbase.h"
#include "motion_manage.h"

#include "event_manager.h"
#include "obs.h"
#include "error.h"


/* Events variables */
/* The fatal quit event includes any of the following case:
 *  ev.bumper_jam
 * 	ev.cliff_all_triggered
 * 	ev.oc_brush_main
 * 	ev.oc_wheel_left
 * 	ev.oc_wheel_right
 * 	ev.oc_suction
 * 	ev.battery_low
 */
/* Bumper */
int g_bumper_cnt = 0;
/* OBS */
uint8_t g_cliff_all_cnt = 0;
int g_cliff_cnt = 0;
/* RCON */
//int ev.rcon_triggered = 0;
/* Over Current */
uint8_t g_oc_wheel_left_cnt = 0;
uint8_t g_oc_wheel_right_cnt = 0;
uint8_t g_oc_suction_cnt = 0;

uint8_t g_battery_low_cnt = 0;
uint8_t g_charge_detect_cnt = 0;
bool g_plan_activated = false;


/* robot slip & stuck */
uint8_t g_slip_cnt = 0;
bool g_robot_slip = false;
bool g_robot_slip_enable = false;
bool g_robot_stuck = false;

Ev_t ev;
/* lidar bumper */
//bool g_lidar_bumper = false;
//bool g_lidar_bumper_jam =false;
//int g_lidar_bumper_cnt = 0;

// laser stuck

static int bumper_all_cnt, bumper_left_cnt, bumper_right_cnt;

//static EventModeType evt_mgr_mode = EVT_MODE_USER_INTERFACE;

static EventActionType	eat;

pthread_mutex_t	new_event_mtx;
pthread_cond_t new_event_cond = PTHREAD_COND_INITIALIZER;

bool g_event_manager_enabled = false;
bool g_event_handler_status = false;
pthread_mutex_t	event_handler_mtx;
pthread_cond_t event_handler_cond = PTHREAD_COND_INITIALIZER;

static bool g_new_event_status[EVT_MAX];

PEHF p_handler[EVT_MAX];
EventHandle deh;
void event_manager_init()
{
	int	j;

	g_event_manager_enabled = g_event_handler_status = false;

	bumper_all_cnt = bumper_left_cnt = bumper_right_cnt = 0;
	event_manager_reset_status();

		for (j = 0; j < EVT_MAX; j++) {
//			eat.handler[j] = NULL;
//			p_handler[j] = NULL;
//			eat.handler_enabled[j] = false;
			g_new_event_status[j] = false;
		}
	p_handler[EVT_BUMPER_ALL] = &EventHandle::bumper_all;
	p_handler[EVT_BUMPER_LEFT] = &EventHandle::bumper_left;
	p_handler[EVT_BUMPER_RIGHT] = &EventHandle::bumper_right;

	p_handler[EVT_OBS_FRONT] = &EventHandle::obs_front;
	p_handler[EVT_OBS_LEFT] = &EventHandle::obs_left;
	p_handler[EVT_OBS_RIGHT] = &EventHandle::obs_right;
	p_handler[EVT_OBS_WALL_LFET] = &EventHandle::obs_wall_left;
	p_handler[EVT_OBS_WALL_RIGHT] = &EventHandle::obs_wall_right;

	p_handler[EVT_CLIFF_ALL] = &EventHandle::cliff_all;
	p_handler[EVT_CLIFF_FRONT_LEFT] = &EventHandle::cliff_front_left;
	p_handler[EVT_CLIFF_FRONT_RIGHT] = &EventHandle::cliff_front_right;
	p_handler[EVT_CLIFF_LEFT_RIGHT] = &EventHandle::cliff_left_right;
	p_handler[EVT_CLIFF_LEFT] = &EventHandle::cliff_left;
	p_handler[EVT_CLIFF_RIGHT] = &EventHandle::cliff_right;
	p_handler[EVT_CLIFF_FRONT] = &EventHandle::cliff_front;

	p_handler[EVT_RCON] = &EventHandle::rcon;

	p_handler[EVT_OVER_CURRENT_BRUSH_LEFT] = &EventHandle::over_current_brush_left;
	p_handler[EVT_OVER_CURRENT_BRUSH_MAIN] = &EventHandle::over_current_brush_main;
	p_handler[EVT_OVER_CURRENT_BRUSH_RIGHT] = &EventHandle::over_current_brush_right;
	p_handler[EVT_OVER_CURRENT_WHEEL_LEFT] = &EventHandle::over_current_wheel_left;
	p_handler[EVT_OVER_CURRENT_WHEEL_RIGHT] = &EventHandle::over_current_wheel_right;
	p_handler[EVT_OVER_CURRENT_SUCTION] = &EventHandle::over_current_suction;

	p_handler[EVT_KEY_CLEAN] = &EventHandle::key_clean;

	p_handler[EVT_REMOTE_PLAN] = &EventHandle::remote_plan;
	p_handler[EVT_REMOTE_CLEAN] = &EventHandle::remote_clean;
	p_handler[EVT_REMOTE_HOME] = &EventHandle::remote_home;

//	handler[EVT_REMOTE_DIRECTION_BACKWARD]=handler_remote_direction_forward;
	p_handler[EVT_REMOTE_DIRECTION_FORWARD] = &EventHandle::remote_direction_forward;
	p_handler[EVT_REMOTE_DIRECTION_LEFT] = &EventHandle::remote_direction_left;
	p_handler[EVT_REMOTE_DIRECTION_RIGHT] = &EventHandle::remote_direction_right;

	p_handler[EVT_REMOTE_WALL_FOLLOW] = &EventHandle::remote_wall_follow;
	p_handler[EVT_REMOTE_SPOT] = &EventHandle::remote_spot;

	p_handler[EVT_REMOTE_MAX] = &EventHandle::remote_max;
//	handler[EVT_REMOTE_TIMER]=handler_remote_max;
//	handler[EVT_WATER_TANK]=handler_remote_max;

	p_handler[EVT_BATTERY_HOME] = &EventHandle::battery_home;
	p_handler[EVT_BATTERY_LOW] = &EventHandle::battery_low;

	p_handler[EVT_CHARGE_DETECT] = &EventHandle::charge_detect;

	p_handler[EVT_SLAM_ERROR] = &EventHandle::slam_error;

	p_handler[EVT_ROBOT_SLIP] = &EventHandle::robot_slip;

//	handler[EVT_LIDAR_BUMPER]=handler_laser_stuck;
	p_handler[EVT_LASER_STUCK] = &EventHandle::laser_stuck;
	eat.peh = &deh;
}

void event_manager_set_enable(bool enable)
{
	g_event_manager_enabled = enable;
	if (!enable)
	{
		//ROS_WARN("%s %d: Disable all event under manager mode_:%d", __FUNCTION__, __LINE__, evt_mgr_mode);
		for (int i = 0; i < EVT_MAX; i++) {
//			eat.handler[i] = NULL;
//			eat.handler_enabled[i] = false;
			g_new_event_status[i] = false;
		}
	}
}


void *event_manager_thread(void *data)
{
	bool set = false;
	bool status[EVT_MAX];
	auto evt_set_status_x = [&](EventType x)
	{
		status[x] = true;
		set = true;
	};

	pthread_detach(pthread_self());

	while (ros::ok()) {
		if (g_event_manager_enabled == false) {
			usleep(10000);
			continue;
		}

		set = false;
		for (int i = 0; i < EVT_MAX; i++) {
			status[i] = false;
		}

		pthread_mutex_lock(&serial_data_ready_mtx);
		pthread_cond_wait(&serial_data_ready_cond, &serial_data_ready_mtx);
		pthread_mutex_unlock(&serial_data_ready_mtx);

		//ROS_DEBUG("%s %d: wake up by serial data arrive", __FUNCTION__, __LINE__);


		/* Bumper */
		if (bumper.get_status() == BLOCK_ALL) {
			ROS_DEBUG("%s %d: setting event:all bumper trig ", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_ALL);
		} else if (bumper.get_status() & BLOCK_LEFT) {
			ROS_DEBUG("%s %d: setting event: left bumper trig", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_LEFT);
		} else if (bumper.get_status() & BLOCK_RIGHT) {
			ROS_DEBUG("%s %d: setting event: right bumper trig", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_RIGHT);
		}

		/* OBS */
		if (obs.get_front() > obs.get_front_trig_value() + 1700) {
			ROS_DEBUG("%s %d: setting event: front obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_FRONT);
		}
		if (obs.get_left() > obs.get_left_trig_value() + 200) {
			ROS_DEBUG("%s %d: setting event: left obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_LEFT);
		}
		if (obs.get_right() > obs.get_right_trig_value() + 200) {
			ROS_DEBUG("%s %d: setting event: right obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_RIGHT);
		}

		/* Cliff */
		if (cliff.get_status() == BLOCK_ALL) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_ALL);
		} else if (cliff.get_status() == (BLOCK_FRONT | BLOCK_LEFT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT_LEFT);
		} else if (cliff.get_status() == (BLOCK_FRONT | BLOCK_RIGHT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT_RIGHT);
		} else if (cliff.get_status() == (BLOCK_LEFT | BLOCK_RIGHT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_LEFT_RIGHT);
		} else if (cliff.get_status() == (BLOCK_FRONT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT);
		} else if (cliff.get_status() == (BLOCK_LEFT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_LEFT);
		} else if (cliff.get_status() == (BLOCK_RIGHT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_RIGHT);
		}

		/* RCON */
		if (c_rcon.getStatus()) {
			//ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON);
		}
/*
		if ((Get_Rcon_Status() & RconFL_HomeT) == RconFL_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_FRONT_LEFT)
		} else if ((Get_Rcon_Status() & RconFL2_HomeT) == RconFL2_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_FRONT_LEFT2)
		} else if ((Get_Rcon_Status() & RconFR_HomeT) == RconFR_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_FRONT_RIGHT)
		} else if ((Get_Rcon_Status() & RconFR2_HomeT) == RconFR2_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_FRONT_RIGHT2)
		} else if ((Get_Rcon_Status() & RconL_HomeT) == RconL_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_LEFT)
		} else if ((c_rcon.getStatus() & RconR_HomeT) == RconR_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_RIGHT)
		}
*/

		/* Over Current */
		if (1/* || robot::instance()->getLbrushOc()*/) {
			//ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_LEFT);
		}
		if (brush.getMainOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_MAIN);
		}
		if (1/* || robot::instance()->getRbrushOc()*/) {
			//ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_RIGHT);
		}
		if ((uint32_t) wheel.getLeftWheelCurrent() > Wheel_Stall_Limit) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_WHEEL_LEFT);
		}
		if ((uint32_t) wheel.getRightWheelCurrent() > Wheel_Stall_Limit) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_WHEEL_RIGHT);
		}
		if (vacuum.getVacuumOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_SUCTION);
		}

		/* Key */
		if (key.getTriggerStatus()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_KEY_CLEAN);
		}

		if (timer.get_status()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_PLAN);
		}

		/* Remote */
		if (remote.key(Remote_Clean)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_CLEAN);
		}
		if (remote.key(Remote_Home)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_HOME);
		}
		if (remote.key(Remote_Forward)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_FORWARD);
		}
		if (remote.key(Remote_Left)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_LEFT);
		}
		if (remote.key(Remote_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_RIGHT);
		}
		if (remote.key(Remote_Spot)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_SPOT);
		}
		if (remote.key(Remote_Max)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_MAX);
		}
		if (remote.key(Remote_Wall_Follow)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_WALL_FOLLOW);
		}

		/* Battery */
		if (battery.getVoltage() && battery.getVoltage() < LOW_BATTERY_GO_HOME_VOLTAGE) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BATTERY_HOME);
		}
		if (battery.getVoltage() < LOW_BATTERY_STOP_VOLTAGE) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BATTERY_LOW);
		}

		/* Charge Status */
		if (charger.getChargeStatus()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CHARGE_DETECT);
		}

		/* Slam Error */
		if (ev.slam_error) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_SLAM_ERROR);
		}
		/* robot slip */
		if(laser_is_robot_slip()){
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_ROBOT_SLIP);
		}
		/*
		if(robot::instance()->getLidarBumper()){
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_LIDAR_BUMPER)
		}
		*/

		// Laser stuck
		if (laser_is_stuck()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_LASER_STUCK);
		}

		if (set) {
			//ROS_INFO("%s %d: going to broadcase new event", __FUNCTION__, __LINE__);
			pthread_mutex_lock(&new_event_mtx);

			memcpy(g_new_event_status, status, sizeof(bool) * EVT_MAX);
			pthread_cond_broadcast(&new_event_cond);

			pthread_mutex_unlock(&new_event_mtx);
		}
	}

	ROS_ERROR("%s %d: exit\n", __FUNCTION__, __LINE__);
}

void *event_handler_thread(void *data) {
	bool status_now[EVT_MAX], status_last[EVT_MAX];

	auto evt_handle_event_x = [&](EventType x)
	{
//		if (eat.handler_enabled[x]) {
//			if (eat.handler[x] == NULL) {
				(eat.peh->*p_handler[x])(status_now[x], status_last[x]);
//			} else {
//				eat.handler[x](status_now[x], status_last[x]);
//			}
//		}
	};

	auto evt_handle_check_event = [&](EventType x)
	{
		if (status_now[x]) {
			evt_handle_event_x(x);
		}
	};

	pthread_detach(pthread_self());

	while (ros::ok()) {
		pthread_mutex_lock(&new_event_mtx);
		pthread_cond_wait(&new_event_cond, &new_event_mtx);

		memcpy(status_last, status_now, sizeof(bool) * EVT_MAX);
		memcpy(status_now, g_new_event_status, sizeof(bool) * EVT_MAX);
		for (int i = 0; i < EVT_MAX; i++) {
			g_new_event_status[i] = false;
		}

		pthread_mutex_unlock(&new_event_mtx);

		pthread_mutex_lock(&event_handler_mtx);
		g_event_handler_status = true;
		pthread_cond_broadcast(&event_handler_cond);
		pthread_mutex_unlock(&event_handler_mtx);

		//ROS_DEBUG("%s %d: handler thread is up, new event to handle", __FUNCTION__, __LINE__);

		/* Bumper */
		if (status_now[EVT_BUMPER_ALL]) {
			evt_handle_event_x(EVT_BUMPER_ALL);
		} else if (status_now[EVT_BUMPER_RIGHT]) {
			evt_handle_event_x(EVT_BUMPER_RIGHT);
		} else if (status_now[EVT_BUMPER_LEFT]) {
			evt_handle_event_x(EVT_BUMPER_LEFT);
		}

		/* OBS */
		evt_handle_check_event(EVT_OBS_FRONT);
		evt_handle_check_event(EVT_OBS_LEFT);
		evt_handle_check_event(EVT_OBS_RIGHT);

		/* Cliff */
		if (status_now[EVT_CLIFF_ALL]) {
            evt_handle_event_x(EVT_CLIFF_ALL);
		} else if (status_now[EVT_CLIFF_FRONT_LEFT]) {
            evt_handle_event_x(EVT_CLIFF_FRONT_LEFT);
		} else if (status_now[EVT_CLIFF_FRONT_RIGHT]) {
            evt_handle_event_x(EVT_CLIFF_FRONT_RIGHT);
		} else if (status_now[EVT_CLIFF_LEFT_RIGHT]) {
            evt_handle_event_x(EVT_CLIFF_LEFT_RIGHT);
		} else if (status_now[EVT_CLIFF_FRONT]) {
            evt_handle_event_x(EVT_CLIFF_FRONT);
		} else if (status_now[EVT_CLIFF_LEFT]) {
            evt_handle_event_x(EVT_CLIFF_LEFT);
		} else if (status_now[EVT_CLIFF_RIGHT]) {
            evt_handle_event_x( EVT_CLIFF_RIGHT);
		}

		/* RCON */
		evt_handle_check_event(EVT_RCON);
/*
		if (status_now[EVT_RCON_FRONT_LEFT] == true) {
            evt_handle_event_x(rcon_front_left, EVT_RCON_FRONT_LEFT)
		} else if (status_now[EVT_RCON_FRONT_LEFT2] == true) {
            evt_handle_event_x(rcon_front_left2, EVT_RCON_FRONT_LEFT2)
		} else if (status_now[EVT_RCON_FRONT_RIGHT] == true) {
            evt_handle_event_x(rcon_front_right, EVT_RCON_FRONT_RIGHT)
		} else if (status_now[EVT_RCON_FRONT_RIGHT2] == true) {
            evt_handle_event_x(rcon_front_right2, EVT_RCON_FRONT_RIGHT2)
		} else if (status_now[EVT_RCON_LEFT] == true) {
            evt_handle_event_x(rcon_left, EVT_RCON_LEFT)
		} else if (status_now[EVT_RCON_RIGHT] == true) {
            evt_handle_event_x(rcon_right, EVT_RCON_RIGHT)
		}
*/

		/* Over Current */
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_LEFT);
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_MAIN);
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_RIGHT);
		evt_handle_check_event(EVT_OVER_CURRENT_WHEEL_LEFT);
		evt_handle_check_event(EVT_OVER_CURRENT_WHEEL_RIGHT);
		evt_handle_check_event(EVT_OVER_CURRENT_SUCTION);
		
		/* Key */
		evt_handle_check_event(EVT_KEY_CLEAN);

		/* Remote */
		evt_handle_check_event(EVT_REMOTE_PLAN);
		evt_handle_check_event(EVT_REMOTE_CLEAN);
		evt_handle_check_event(EVT_REMOTE_HOME);
		evt_handle_check_event(EVT_REMOTE_DIRECTION_FORWARD);
		evt_handle_check_event(EVT_REMOTE_DIRECTION_LEFT);
		evt_handle_check_event(EVT_REMOTE_DIRECTION_RIGHT);
		evt_handle_check_event(EVT_REMOTE_SPOT);
		evt_handle_check_event(EVT_REMOTE_MAX);
		evt_handle_check_event(EVT_REMOTE_WALL_FOLLOW);

		/* Battery */
		evt_handle_check_event(EVT_BATTERY_HOME);
		evt_handle_check_event(EVT_BATTERY_LOW);

		/* Charge Status */
		evt_handle_check_event(EVT_CHARGE_DETECT);

		/* Slam Error */
		evt_handle_check_event(EVT_SLAM_ERROR);

		/* robot slip*/
		evt_handle_check_event(EVT_ROBOT_SLIP);

		/* lidar bumper*/
		//evt_handle_check_event(EVT_LIDAR_BUMPER,lidar_bumper)

		// Laser stuck
		evt_handle_check_event(EVT_LASER_STUCK);

		pthread_mutex_lock(&event_handler_mtx);
		g_event_handler_status = false;
		pthread_cond_broadcast(&event_handler_cond);
		pthread_mutex_unlock(&event_handler_mtx);
	}
	ROS_ERROR("%s %d: exit\n", __FUNCTION__, __LINE__);
}

//void event_manager_set_current_mode(EventModeType mode_)
//{
//	evt_mgr_mode = mode_;
//}

void event_manager_register_handler(EventHandle* eh)
{
	eat.peh = eh;
}

uint8_t event_manager_check_event(bool *p_eh_status_now, bool *p_eh_status_last)
{
	struct timespec	ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	timespec_add_ns(ts, _10MS);

	pthread_mutex_lock(&event_handler_mtx);
	pthread_cond_timedwait(&event_handler_cond, &event_handler_mtx, &ts);

	*p_eh_status_last = *p_eh_status_now;
	*p_eh_status_now = g_event_handler_status;
	pthread_mutex_unlock(&event_handler_mtx);

	if (*p_eh_status_now == true) {
		if (*p_eh_status_last == false) {
//			ROS_WARN("%s %d: event manager handling event, process pause!\n", __FUNCTION__, __LINE__);
		}
		return 1;
	} else
	if (*p_eh_status_last == true) {
//		ROS_WARN("%s %d: event manager finished handling event, process resume!\n", __FUNCTION__, __LINE__);
	}

	return 0;
}

void event_manager_reset_status(void)
{
	ev.fatal_quit = false;
	/* Bumper */
	ev.bumper_triggered = false;
	ev.bumper_jam = false;
	g_bumper_cnt = 0;
	/* OBS */
//	ev.obs_triggered = false;
	/* Cliff */
	ev.cliff_all_triggered = false;
	ev.cliff_triggered = 0;
	ev.cliff_jam = false;
	g_cliff_all_cnt = 0;
	g_cliff_cnt = 0;
	/* RCON */
	/* Over Current */
	ev.oc_brush_main = false;
	ev.oc_wheel_left = false;
	ev.oc_wheel_right = false;
	ev.oc_suction = false;
	brush.oc_left_cnt_ = 0;
	brush.oc_main_cnt_ = 0;
	brush.oc_right_cnt_ = 0;
	g_oc_wheel_left_cnt = 0;
	g_oc_wheel_right_cnt = 0;
	g_oc_suction_cnt = 0;
	/* Key */
	ev.key_clean_pressed = false;
	/* Remote */
	ev.remote_home = false;
	ev.remote_spot = false;
	ev.remote_wallfollow = false;
	ev.remote_direction_keys = false;
	/* Battery */
	ev.battery_home = false;
	ev.battery_low = false;
	g_battery_low_cnt = 0;
	/* Charge Status */
	ev.charge_detect = 0;
	g_charge_detect_cnt = 0;
	/* Slam Error */
	ev.slam_error = false;
	/* robot stuck */
	//g_robot_stuck = false;
	g_robot_slip = false;
	g_slip_cnt = 0;
	/* tilt switch*/
	gyro.TiltCheckingEnable(false);
	ev.tilt_triggered = false;
	/* lidar bumper */
	//g_lidar_bumper = false;
	//g_lidar_bumper_cnt =0;
	//g_lidar_bumper_jam = false;
	// laser stuck
	ev.laser_stuck = false;
}

/* Below are the internal functions. */

/* Bumper */
void EventHandle::bumper_all(bool state_now, bool state_last)
{
/*	if (state_now == true && state_last == true) {
		bumper_all_cnt++;

		if (bumper_all_cnt > 2) {
			ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_all_cnt);
		}
	} else {
		bumper_all_cnt = 0;
	}

	move_back();
	wheel_stop();

	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, bumper.get_status());*/
}

void EventHandle::bumper_left(bool state_now, bool state_last)
{
/*	if (state_now == true && state_last == true) {
		bumper_left_cnt++;

		if (bumper_left_cnt > 2) {
			ROS_WARN("%s %d: left bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_left_cnt);
		}
	} else {
		bumper_left_cnt = 0;
	}

	move_back();
	wheel_stop();
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, bumper.get_status());*/
}

void EventHandle::bumper_right(bool state_now, bool state_last)
{
/*
	if (state_now == true && state_last == true) {
		bumper_right_cnt++;

		if (bumper_right_cnt > 2) {
			ROS_WARN("%s %d: right bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_right_cnt);
		}
	} else {
		bumper_right_cnt = 0;
	}

	move_back();
	wheel_stop();
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, bumper.get_status());*/
}

/* OBS */
void EventHandle::obs_front(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

void EventHandle::obs_left(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

void EventHandle::obs_right(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

void EventHandle::obs_wall_left(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

void EventHandle::obs_wall_right(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

/* Cliff */
void EventHandle::cliff_all(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliff_front_left(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliff_front_right(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliff_left_right(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliff_front(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliff_left(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliff_right(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* RCON */
void EventHandle::rcon(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
//	c_rcon.resetStatus();
}
void df_rcon(bool state_now, bool state_last)
{
	c_rcon.resetStatus();
}
/*
void EventHandle::rcon_front_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_front_left2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_front_right2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}
*/

/* Over Current */
void EventHandle::over_current_brush_left(bool state_now, bool state_last)
{

}
void df_over_current_brush_left(bool state_now, bool state_last)
{
	//ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (!ev.fatal_quit && brush.leftIsStall())
	{
		error.set(ERROR_CODE_LEFTBRUSH);
		ev.fatal_quit = true;
		ROS_WARN("%s %d: Left brush stall, please check.", __FUNCTION__, __LINE__);
	}
}


void EventHandle::over_current_brush_main(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::over_current_brush_right(bool state_now, bool state_last)
{}
void df_over_current_brush_right(bool state_now, bool state_last)
{
	//ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (!ev.fatal_quit && brush.rightIsStall())
	{
		error.set(ERROR_CODE_RIGHTBRUSH);
		ev.fatal_quit = true;
		ROS_WARN("%s %d: Right brush stall, please check.", __FUNCTION__, __LINE__);
	}
}

void EventHandle::over_current_wheel_left(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::over_current_wheel_right(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::over_current_suction(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Key */
void EventHandle::key_clean(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Remote */
void EventHandle::remote_plan(bool state_now, bool state_last)
{

}
void df_remote_plan(bool state_now, bool state_last)
{
	if (timer.get_status() == 1 || timer.get_status() == 2)
	{
		ROS_WARN("%s %d: Remote plan is pressed.", __FUNCTION__, __LINE__);
		beeper.play_for_command(INVALID);
	}
	else if (timer.get_status() == 3)
		ROS_WARN("%s %d: Plan is activated.", __FUNCTION__, __LINE__);

	timer.set_status(0);
	remote.reset();
}

void EventHandle::remote_clean(bool state_now, bool state_last)
{}
void df_remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote clean is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remote_home(bool state_now, bool state_last)
{}
void df_remote_home(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote home is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remote_direction_forward(bool state_now, bool state_last)
{}
void df_remote_direction_forward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote forward is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remote_wall_follow(bool state_now, bool state_last)
{}
void df_remote_wall_follow(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote wall_follow is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remote_direction_left(bool state_now, bool state_last)
{}
void df_remote_direction_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote left is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remote_direction_right(bool state_now, bool state_last)
{}
void df_remote_direction_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote right is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remote_spot(bool state_now, bool state_last)
{}
void df_remote_spot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote spot is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remote_max(bool state_now, bool state_last)
{}
void df_remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

/* Battery */
void EventHandle::battery_home(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::battery_low(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::charge_detect(bool state_now, bool state_last)
{}
void df_charge_detect(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (g_charge_detect_cnt++ > 25)
	{
		ev.charge_detect = charger.getChargeStatus();
		ROS_WARN("%s %d: ev.charge_detect has been set to %d.", __FUNCTION__, __LINE__, ev.charge_detect);
		g_charge_detect_cnt = 0;
	}
}

/* Slam Error */
void EventHandle::slam_error(bool state_now, bool state_last)
{}
void df_slam_error(bool state_now, bool state_last)
{
/*	static time_t slam_error_kill_timer_;
	static bool relaunch = false;

	if (!state_last)
	{
		ROS_ERROR("%s %d: Slam process may be dead.", __FUNCTION__, __LINE__);
		relaunch = false;
		slam_error_kill_timer_ = time(NULL);
		system("rosnode kill /slam_karto &");
	}

	// Wait for 3s to make sure node has been killed.
	if (time(NULL) - slam_error_kill_timer_ < 1)
		return;

	if (!relaunch)
	{
		system("roslaunch pp karto_slam.launch &");
		//robotbase_restore_slam_correction();
		MotionManage::s_slam->isMapReady(false);
		relaunch = true;
		led.set_mode(LED_FLASH, LED_GREEN, 1000);
	}

	if (MotionManage::s_slam != nullptr)
	{
		if (!MotionManage::s_slam->isMapReady())
		{
			ROS_WARN("Slam not ready yet.");
			MotionManage::s_slam->enableMapUpdate();
			usleep(100000);
			return;
		}
	}
	led.set_mode(LED_STEADY, LED_GREEN);
	// Wait for 0.2s to make sure it has process the first scan.
	usleep(200000);
	ROS_WARN("Slam restart successed.");
	ev.slam_error = false;*/
}

void EventHandle::robot_slip(bool state_new,bool state_last)
{}
void df_robot_slip(bool state_new,bool state_last)
{
	ROS_WARN("\033[32m%s,%d,set robot slip!! \033[0m",__FUNCTION__,__LINE__);
	beeper.play_for_command(true);
	g_robot_slip = true;
	g_slip_cnt ++;
}
/*
void EventHandle::lidar_bumper(bool state_new,bool state_last)
{
	g_lidar_bumper = robot::instance()->getLidarBumper();
}
*/

// Laser stuck
void EventHandle::laser_stuck(bool state_new,bool state_last)
{}
void df_laser_stuck(bool state_new,bool state_last)
{
	beeper.play_for_command(true);
	//ROS_WARN("\033[32m%s %d: Laser stuck.\033[0m", __FUNCTION__, __LINE__);
	//ev.laser_stuck = true;
}

///* Default: empty hanlder */
//void EventHandle::empty(bool state_now, bool state_last)
//{
////	ROS_INFO("%s %d: is called", __FUNCTION__, __LINE__);
//}
