#include <pthread.h>
#include <ros/ros.h>

#include "config.h"
#include "serial.h"

#include "movement.h"
#include "robot.hpp"
#include "robotbase.h"
#include "motion_manage.h"

#include "event_manager.h"


/* Events variables */
/* The fatal quit event includes any of the following case:
 *  g_bumper_jam
 * 	g_cliff_all_triggered
 * 	g_oc_brush_main
 * 	g_oc_wheel_left
 * 	g_oc_wheel_right
 * 	g_oc_suction
 * 	g_battery_low
 */
bool g_fatal_quit_event = false;
/* Bumper */
int g_bumper_triggered = false;
bool g_bumper_jam = false;
int g_bumper_cnt = 0;
/* OBS */
int g_obs_triggered = 0;
/* Cliff */
bool g_cliff_all_triggered = false;
int g_cliff_triggered = 0;
bool g_cliff_jam = false;
uint8_t g_cliff_all_cnt = 0;
int g_cliff_cnt = 0;
/* RCON */
//int g_rcon_triggered = 0;
/* Over Current */
bool g_oc_brush_main = false;
bool g_oc_wheel_left = false;
bool g_oc_wheel_right = false;
bool g_oc_suction = false;
uint8_t g_oc_brush_left_cnt = 0;
uint8_t g_oc_brush_main_cnt = 0;
uint8_t g_oc_brush_right_cnt = 0;
uint8_t g_oc_wheel_left_cnt = 0;
uint8_t g_oc_wheel_right_cnt = 0;
uint8_t g_oc_suction_cnt = 0;
/* Key */
bool g_key_clean_pressed = false;
/* Remote */
bool g_remote_home = false;
bool g_remote_spot = false;
bool g_remote_wallfollow = false;
bool g_remote_direction_keys = false;
/* Battery */
bool g_battery_home = false;
bool g_battery_low = false;
uint8_t g_battery_low_cnt = 0;
/* Charge Status */
uint8_t g_charge_detect = 0;
uint8_t g_charge_detect_cnt = 0;
/* Slam Error */
bool g_slam_error = false;
/* Plan */
bool g_plan_activated = false;

/* Omni wheel*/
bool g_omni_notmove = false;

static int bumper_all_cnt, bumper_left_cnt, bumper_right_cnt;

static EventModeType evt_mgr_mode = EVT_MODE_USER_INTERFACE;

static EventActionType	eat[EVT_MODE_MAX];

pthread_mutex_t	new_event_mtx;
pthread_cond_t new_event_cond = PTHREAD_COND_INITIALIZER;

bool g_event_manager_enabled = false;
bool g_event_handler_status = false;
pthread_mutex_t	event_handler_mtx;
pthread_cond_t event_handler_cond = PTHREAD_COND_INITIALIZER;

static bool gs_new_event_status[EVT_MAX];

void event_manager_init()
{
	int	i, j;

	g_event_manager_enabled = g_event_handler_status = false;

	bumper_all_cnt = bumper_left_cnt = bumper_right_cnt = 0;
	event_manager_reset_status();

	for (i = 0; i < EVT_MODE_MAX; i++) {
		for (j = 0; j < EVT_MAX; j++) {
			eat[i].handler[j] = NULL;
			eat[i].handler_enabled[j] = false;
		}

		gs_new_event_status[i] = false;
	}
}

void event_manager_set_enable(bool enable)
{
	g_event_manager_enabled = enable;
}

void *event_manager_thread(void *data)
{
	bool set = false;
	bool status[EVT_MAX];

	pthread_detach(pthread_self());

	while (ros::ok()) {
		if (g_event_manager_enabled == false) {
			usleep(10000);
		}

		set = false;
		for (int i = 0; i < EVT_MAX; i++) {
			status[i] = false;
		}

		pthread_mutex_lock(&serial_data_ready_mtx);
		pthread_cond_wait(&serial_data_ready_cond, &serial_data_ready_mtx);
		pthread_mutex_unlock(&serial_data_ready_mtx);

//		ROS_DEBUG("%s %d: wake up by serial data arrive", __FUNCTION__, __LINE__);

#define	evt_set_status_x(x)	\
	{ 						\
		status[x] = true;	\
		set = true;			\
	}

		/* Bumper */
		if (get_bumper_status() == AllBumperTrig) {
			ROS_DEBUG("%s %d: setting event:all bumper trig ", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_ALL)
		} else if (get_bumper_status() & LeftBumperTrig) {
			ROS_DEBUG("%s %d: setting event: left bumper trig", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_LEFT)
		} else if (get_bumper_status() & RightBumperTrig) {
			ROS_DEBUG("%s %d: setting event: right bumper trig", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_RIGHT)
		}

		/* OBS */
		if (get_front_obs() > get_front_obs_value()) {
			ROS_DEBUG("%s %d: setting event: front obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_FRONT)
		}
		if (get_left_obs() > get_left_obs_value()) {
			ROS_DEBUG("%s %d: setting event: left obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_LEFT)
		}
		if (get_right_obs() > get_right_obs_value()) {
			ROS_DEBUG("%s %d: setting event: right obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_RIGHT)
		}

		/* Cliff */
		if (get_cliff_status() == Status_Cliff_All) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_ALL)
		} else if (get_cliff_status() == (Status_Cliff_Front | Status_Cliff_Left)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT_LEFT)
		} else if (get_cliff_status() == (Status_Cliff_Front | Status_Cliff_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT_RIGHT)
		} else if (get_cliff_status() == (Status_Cliff_Left | Status_Cliff_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_LEFT_RIGHT)
		} else if (get_cliff_status() == (Status_Cliff_Front)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT)
		} else if (get_cliff_status() == (Status_Cliff_Left)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_LEFT)
		} else if (get_cliff_status() == (Status_Cliff_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_RIGHT)
		}

		/* RCON */
		if (get_rcon_status()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON)
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
		} else if ((get_rcon_status() & RconR_HomeT) == RconR_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_RIGHT)
		}
*/

		/* Over Current */
		if (1/* || robot::instance()->getLbrushOc()*/) {
			//ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_LEFT)
		}
		if (robot::instance()->getMbrushOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_MAIN)
		}
		if (1/* || robot::instance()->getRbrushOc()*/) {
			//ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_RIGHT)
		}
		if ((uint32_t)robot::instance()->getLwheelCurrent() > Wheel_Stall_Limit) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_WHEEL_LEFT)
		}
		if ((uint32_t)robot::instance()->getRwheelCurrent() > Wheel_Stall_Limit) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_WHEEL_RIGHT)
		}
		if (robot::instance()->getVacuumOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_SUCTION)
		}

		/* Key */
		if (get_touch_status()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_KEY_CLEAN)
		}

		if (get_plan_status()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_PLAN)
		}

		/* Remote */
		if (remote_key(Remote_Clean)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_CLEAN)
		}
		if (remote_key(Remote_Home)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_HOME)
		}
		if (remote_key(Remote_Forward)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_FORWARD)
		}
		if (remote_key(Remote_Left)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_LEFT)
		}
		if (remote_key(Remote_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_RIGHT)
		}
		if (remote_key(Remote_Spot)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_SPOT)
		}
		if (remote_key(Remote_Max)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_MAX)
		}
		if (remote_key(Remote_Wall_Follow)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_WALL_FOLLOW)
		}

		/* Battery */
		if (robot::instance()->getBatteryVoltage() && robot::instance()->getBatteryVoltage() < LOW_BATTERY_GO_HOME_VOLTAGE) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BATTERY_HOME)
		}
		if (robot::instance()->getBatteryVoltage() < LOW_BATTERY_STOP_VOLTAGE) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BATTERY_LOW)
		}

		/* Charge Status */
		if (robot::instance()->getChargeStatus()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CHARGE_DETECT)
		}

		/* Slam Error */
		if (g_slam_error) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_SLAM_ERROR)
		}
#undef evt_set_status_x

		if (set) {
			//ROS_INFO("%s %d: going to broadcase new event", __FUNCTION__, __LINE__);
			pthread_mutex_lock(&new_event_mtx);

			memcpy(gs_new_event_status, status, sizeof(bool) * EVT_MAX);
			pthread_cond_broadcast(&new_event_cond);

			pthread_mutex_unlock(&new_event_mtx);
		}
	}

	ROS_ERROR("%s %d: exit\n", __FUNCTION__, __LINE__);
}

void *event_handler_thread(void *data) {
	bool status_now[EVT_MAX], status_last[EVT_MAX];

	pthread_detach(pthread_self());

	while (ros::ok()) {
		pthread_mutex_lock(&new_event_mtx);
		pthread_cond_wait(&new_event_cond, &new_event_mtx);

		memcpy(status_last, status_now, sizeof(bool) * EVT_MAX);
		memcpy(status_now, gs_new_event_status, sizeof(bool) * EVT_MAX);
		for (int i = 0; i < EVT_MAX; i++) {
			gs_new_event_status[i] = false;
		}

		pthread_mutex_unlock(&new_event_mtx);

		pthread_mutex_lock(&event_handler_mtx);
		g_event_handler_status = true;
		pthread_cond_broadcast(&event_handler_cond);
		pthread_mutex_unlock(&event_handler_mtx);

		ROS_DEBUG("%s %d: handler thread is up, new event to handle", __FUNCTION__, __LINE__);

#define	evt_handle_event_x(name, y)	\
	{																			\
		if (eat[evt_mgr_mode].handler_enabled[y] == true) {						\
			if (eat[evt_mgr_mode].handler[y] == NULL) {							\
				em_default_handle_ ## name(status_now[y], status_last[y]);		\
			} else {															\
				eat[evt_mgr_mode].handler[y](status_now[y], status_last[y]);	\
			}																	\
		}																		\
	}

#define evt_handle_check_event(x, y) \
	{									\
		if (status_now[x] == true) {	\
			evt_handle_event_x(y, x)	\
		}								\
	}

		/* Bumper */
		if (status_now[EVT_BUMPER_ALL] == true) {
			evt_handle_event_x(bumper_all, EVT_BUMPER_ALL)
		} else if (status_now[EVT_BUMPER_RIGHT] == true) {
			evt_handle_event_x(bumper_right, EVT_BUMPER_RIGHT)
		} else if (status_now[EVT_BUMPER_LEFT] == true) {
			evt_handle_event_x(bumper_left, EVT_BUMPER_LEFT)
		}

		/* OBS */
		evt_handle_check_event(EVT_OBS_FRONT, obs_front)
		evt_handle_check_event(EVT_OBS_LEFT, obs_left)
		evt_handle_check_event(EVT_OBS_RIGHT, obs_right)

		/* Cliff */
		if (status_now[EVT_CLIFF_ALL] == true) {
            evt_handle_event_x(cliff_all, EVT_CLIFF_ALL)
		} else if (status_now[EVT_CLIFF_FRONT_LEFT] == true) {
            evt_handle_event_x(cliff_front_left, EVT_CLIFF_FRONT_LEFT)
		} else if (status_now[EVT_CLIFF_FRONT_RIGHT] == true) {
            evt_handle_event_x(cliff_front_right, EVT_CLIFF_FRONT_RIGHT)
		} else if (status_now[EVT_CLIFF_LEFT_RIGHT] == true) {
            evt_handle_event_x(cliff_left_right, EVT_CLIFF_LEFT_RIGHT)
		} else if (status_now[EVT_CLIFF_FRONT] == true) {
            evt_handle_event_x(cliff_front, EVT_CLIFF_FRONT)
		} else if (status_now[EVT_CLIFF_LEFT] == true) {
            evt_handle_event_x(cliff_left, EVT_CLIFF_LEFT)
		} else if (status_now[EVT_CLIFF_RIGHT] == true) {
            evt_handle_event_x(cliff_right, EVT_CLIFF_RIGHT)
		}

		/* RCON */
		if (status_now[EVT_RCON] == true) {
			evt_handle_event_x(rcon, EVT_RCON)
		}
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
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_LEFT, over_current_brush_left)
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_MAIN, over_current_brush_main)
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_RIGHT, over_current_brush_right)
		evt_handle_check_event(EVT_OVER_CURRENT_WHEEL_LEFT, over_current_wheel_left)
		evt_handle_check_event(EVT_OVER_CURRENT_WHEEL_RIGHT, over_current_wheel_right)
		evt_handle_check_event(EVT_OVER_CURRENT_SUCTION, over_current_suction)
		
		/* Key */
		evt_handle_check_event(EVT_KEY_CLEAN, key_clean)

		/* Remote */
		evt_handle_check_event(EVT_REMOTE_PLAN, remote_plan)
		evt_handle_check_event(EVT_REMOTE_CLEAN, remote_clean)
		evt_handle_check_event(EVT_REMOTE_HOME, remote_home)
		evt_handle_check_event(EVT_REMOTE_DIRECTION_FORWARD, remote_direction_forward)
		evt_handle_check_event(EVT_REMOTE_DIRECTION_LEFT, remote_direction_left)
		evt_handle_check_event(EVT_REMOTE_DIRECTION_RIGHT, remote_direction_right)
		evt_handle_check_event(EVT_REMOTE_SPOT, remote_spot)
		evt_handle_check_event(EVT_REMOTE_MAX, remote_max)
		evt_handle_check_event(EVT_REMOTE_WALL_FOLLOW, remote_wall_follow)

		/* Battery */
		evt_handle_check_event(EVT_BATTERY_HOME, battery_home)
		evt_handle_check_event(EVT_BATTERY_LOW, battery_low)

		/* Charge Status */
		evt_handle_check_event(EVT_CHARGE_DETECT, charge_detect)

		/* Slam Error */
		evt_handle_check_event(EVT_SLAM_ERROR, slam_error)

#undef evt_handle_event_x

		pthread_mutex_lock(&event_handler_mtx);
		g_event_handler_status = false;
		pthread_cond_broadcast(&event_handler_cond);
		pthread_mutex_unlock(&event_handler_mtx);
	}
	ROS_ERROR("%s %d: exit\n", __FUNCTION__, __LINE__);
}

void event_manager_set_current_mode(EventModeType mode)
{
	evt_mgr_mode = mode;
}

void event_manager_register_handler(EventType type, void (*func)(bool state_now, bool state_last))
{
	eat[evt_mgr_mode].handler[type] = func;
}

void event_manager_enable_handler(EventType type, bool enabled)
{
	eat[evt_mgr_mode].handler_enabled[type] = enabled;
	if ((type == EVT_OVER_CURRENT_BRUSH_LEFT || type == EVT_OVER_CURRENT_BRUSH_RIGHT) && !enabled)
	{
		extern bool g_reset_lbrush_oc;
		extern bool g_reset_rbrush_oc;
		g_reset_lbrush_oc = true;
		g_reset_rbrush_oc = true;
	}
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
	g_fatal_quit_event = false;
	/* Bumper */
	g_bumper_triggered = false;
	g_bumper_jam = false;
	g_bumper_cnt = 0;
	/* OBS */
//	g_obs_triggered = false;
	/* Cliff */
	g_cliff_all_triggered = false;
	g_cliff_triggered = 0;
	g_cliff_jam = false;
	g_cliff_all_cnt = 0;
	g_cliff_cnt = 0;
	/* RCON */
//	g_rcon_triggered = false;
	/* Over Current */
	g_oc_brush_main = false;
	g_oc_wheel_left = false;
	g_oc_wheel_right = false;
	g_oc_suction = false;
	g_oc_brush_left_cnt = 0;
	g_oc_brush_main_cnt = 0;
	g_oc_brush_right_cnt = 0;
	g_oc_wheel_left_cnt = 0;
	g_oc_wheel_right_cnt = 0;
	g_oc_suction_cnt = 0;
	/* Key */
	g_key_clean_pressed = false;
	/* Remote */
	g_remote_home = false;
	g_remote_spot = false;
	g_remote_wallfollow = false;
	g_remote_direction_keys = false;
	/* Battery */
	g_battery_home = false;
	g_battery_low = false;
	g_battery_low_cnt = 0;
	/* Charge Status */
	g_charge_detect = 0;
	g_charge_detect_cnt = 0;
	/* Slam Error */
	g_slam_error = false;
}

/* Below are the internal functions. */

/* Bumper */
void em_default_handle_bumper_all(bool state_now, bool state_last)
{
	if (state_now == true && state_last == true) {
		bumper_all_cnt++;

		if (bumper_all_cnt > 2) {
			ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_all_cnt);
		}
	} else {
		bumper_all_cnt = 0;
	}

	move_back();
	stop_brifly();

	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, get_bumper_status());
}

void em_default_handle_bumper_left(bool state_now, bool state_last)
{
	if (state_now == true && state_last == true) {
		bumper_left_cnt++;

		if (bumper_left_cnt > 2) {
			ROS_WARN("%s %d: left bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_left_cnt);
		}
	} else {
		bumper_left_cnt = 0;
	}

	move_back();
	stop_brifly();
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, get_bumper_status());
}

void em_default_handle_bumper_right(bool state_now, bool state_last)
{

	if (state_now == true && state_last == true) {
		bumper_right_cnt++;

		if (bumper_right_cnt > 2) {
			ROS_WARN("%s %d: right bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_right_cnt);
		}
	} else {
		bumper_right_cnt = 0;
	}

	move_back();
	stop_brifly();
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, get_bumper_status());
}

/* OBS */
void em_default_handle_obs_front(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//stop_brifly();
}

void em_default_handle_obs_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//stop_brifly();
}

void em_default_handle_obs_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//stop_brifly();
}

void em_default_handle_obs_wall_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//stop_brifly();
}

void em_default_handle_obs_wall_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//stop_brifly();
}

/* Cliff */
void em_default_handle_cliff_all(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_cliff_front_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_cliff_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_cliff_left_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_cliff_front(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_cliff_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_cliff_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* RCON */
void em_default_handle_rcon(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}
/*
void em_default_handle_rcon_front_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_rcon_front_left2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_rcon_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_rcon_front_right2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_rcon_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_rcon_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}
*/

/* Over Current */
void em_default_handle_over_current_brush_left(bool state_now, bool state_last)
{
	//ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (!g_fatal_quit_event && check_left_brush_stall())
	{
		/*-----Set error-----*/
		set_error_code(Error_Code_LeftBrush);
		g_fatal_quit_event = true;
		ROS_WARN("%s %d: Left brush stall, please check.", __FUNCTION__, __LINE__);
	}
}

void em_default_handle_over_current_brush_main(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_over_current_brush_right(bool state_now, bool state_last)
{
	//ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (!g_fatal_quit_event && check_right_brush_stall())
	{
		/*-----Set error-----*/
		set_error_code(Error_Code_RightBrush);
		g_fatal_quit_event = true;
		ROS_WARN("%s %d: Right brush stall, please check.", __FUNCTION__, __LINE__);
	}
}

void em_default_handle_over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_over_current_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Key */
void em_default_handle_key_clean(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Remote */
void em_default_handle_remote_plan(bool state_now, bool state_last)
{
	if (get_plan_status() == 1 || get_plan_status() == 2)
	{
		ROS_WARN("%s %d: Remote plan is pressed.", __FUNCTION__, __LINE__);
		beep_for_command(INVALID);
	}
	else if (get_plan_status() == 3)
		ROS_WARN("%s %d: Plan is activated.", __FUNCTION__, __LINE__);

	set_plan_status(0);
	reset_rcon_remote();
}

void em_default_handle_remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote clean is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void em_default_handle_remote_home(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote home is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void em_default_handle_remote_direction_forward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote forward is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void em_default_handle_remote_wall_follow(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote wall_follow is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void em_default_handle_remote_direction_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote left is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void em_default_handle_remote_direction_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote right is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void em_default_handle_remote_spot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote spot is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void em_default_handle_remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

/* Battery */
void em_default_handle_battery_home(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_battery_low(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handle_charge_detect(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (g_charge_detect_cnt++ > 25)
	{
		g_charge_detect = robot::instance()->getChargeStatus();
		ROS_WARN("%s %d: g_charge_detect has been set to %d.", __FUNCTION__, __LINE__, g_charge_detect);
		g_charge_detect_cnt = 0;
	}
}

/* Slam Error */
void em_default_handle_slam_error(bool state_now, bool state_last)
{
	static time_t slam_error_kill_timer_;
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
		robotbase_restore_slam_correction();
		MotionManage::s_slam->isMapReady(false);
		relaunch = true;
		set_led_mode(LED_FLASH, LED_GREEN, 1000);
	}

	if (MotionManage::s_slam != nullptr)
	{
		if (!MotionManage::s_slam->isMapReady())
		{
			//MotionManage::s_laser->stopShield();
			MotionManage::s_laser->lidarShieldDetect(false);
			ROS_WARN("Slam not ready yet.");
			MotionManage::s_slam->enableMapUpdate();
			usleep(100000);
			return;
		}
		else
			//MotionManage::s_laser->startShield();
			MotionManage::s_laser->lidarShieldDetect(true);
	}
	set_led_mode(LED_STEADY, LED_GREEN);
	// Wait for 0.2s to make sure it has process the first scan.
	usleep(200000);
	ROS_WARN("Slam restart successed.");
	g_slam_error = false;
}

/* Default: empty hanlder */
void em_default_handle_empty(bool state_now, bool state_last)
{
	ROS_INFO("%s %d: is called", __FUNCTION__, __LINE__);
}
