#include <pthread.h>
#include <ros/ros.h>

#include "config.h"
#include "serial.h"

#include "movement.h"
#include "robot.hpp"

#include "event_manager.h"

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
		if (Get_Bumper_Status() == AllBumperTrig) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_ALL)
		} else if (Get_Bumper_Status() & LeftBumperTrig) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_LEFT)
		} else if (Get_Bumper_Status() & RightBumperTrig) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_RIGHT)
		}

		/* OBS */
		if (Get_FrontOBS() > Get_FrontOBST_Value()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_FRONT)
		}
		if (Get_LeftOBS() > Get_LeftOBST_Value()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_LEFT)
		}
		if (Get_RightOBS() > Get_RightOBST_Value()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_RIGHT)
		}

		/* Cliff */
		if (Get_Cliff_Trig() == Status_Cliff_All) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_ALL)
		} else if (Get_Cliff_Trig() == (Status_Cliff_Front | Status_Cliff_Left)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT_LEFT)
		} else if (Get_Cliff_Trig() == (Status_Cliff_Front | Status_Cliff_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT_RIGHT)
		} else if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_LEFT_RIGHT)
		} else if (Get_Cliff_Trig() == (Status_Cliff_Front)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT)
		} else if (Get_Cliff_Trig() == (Status_Cliff_Left)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_LEFT)
		} else if (Get_Cliff_Trig() == (Status_Cliff_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_RIGHT)
		}

		/* RCON */
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
		} else if ((Get_Rcon_Status() & RconR_HomeT) == RconR_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_RIGHT)
		}

		/* Over Current */
		if (robot::instance()->getLbrushOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_LEFT)
		}
		if (robot::instance()->getMbrushOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_MAIN)
		}
		if (robot::instance()->getRbrushOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
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
		if (Get_Touch_Status()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_KEY_CLEAN)
		}

		if (Get_Plan_Status() == 1) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_APPOINMENT)
		}

		if (Remote_Key(Remote_Clean)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_CLEAN)
		}
		if (Remote_Key(Remote_Home)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_HOME)
		}
		if (Remote_Key(Remote_Left)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_LEFT)
		}
		if (Remote_Key(Remote_Right)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_RIGHT)
		}
		if (Remote_Key(Remote_Spot)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_MODE_SPOT)
		}
		if (Remote_Key(Remote_Max)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_SUCTION)
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
				em_default_handler_ ## name(status_now[y], status_last[y]);		\
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
		evt_handle_check_event(EVT_REMOTE_APPOINMENT, remote_plan)
		evt_handle_check_event(EVT_REMOTE_CLEAN, remote_clean)
		evt_handle_check_event(EVT_REMOTE_HOME, remote_home)
		evt_handle_check_event(EVT_REMOTE_DIRECTION_LEFT, remote_direction_left)
		evt_handle_check_event(EVT_REMOTE_DIRECTION_RIGHT, remote_direction_right)
		evt_handle_check_event(EVT_REMOTE_MODE_SPOT, remote_mode_spot)
		evt_handle_check_event(EVT_REMOTE_SUCTION, remote_suction)

		/* Battery */
		evt_handle_check_event(EVT_BATTERY_HOME, battery_home)
		evt_handle_check_event(EVT_BATTERY_LOW, battery_low)

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

/* Below are the internal functions. */

/* Bumper */
void em_default_handler_bumper_all(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
	if (state_now == true && state_last == true) {
		bumper_all_cnt++;

		if (bumper_all_cnt > 2) {
			ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_all_cnt);
		}
	} else {
		bumper_all_cnt = 0;
	}

	Move_Back();
	Stop_Brifly();

	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
}

void em_default_handler_bumper_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
	if (state_now == true && state_last == true) {
		bumper_left_cnt++;

		if (bumper_left_cnt > 2) {
			ROS_WARN("%s %d: left bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_left_cnt);
		}
	} else {
		bumper_left_cnt = 0;
	}

	Move_Back();
	Stop_Brifly();
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
}

void em_default_handler_bumper_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());

	if (state_now == true && state_last == true) {
		bumper_right_cnt++;

		if (bumper_right_cnt > 2) {
			ROS_WARN("%s %d: right bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_right_cnt);
		}
	} else {
		bumper_right_cnt = 0;
	}

	Move_Back();
	Stop_Brifly();
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
}

/* OBS */
void em_default_handler_obs_front(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	Move_Back();
	Stop_Brifly();
}

void em_default_handler_obs_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	Move_Back();
	Stop_Brifly();
}

void em_default_handler_obs_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	Move_Back();
	Stop_Brifly();
}

void em_default_handler_obs_wall_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	Move_Back();
	Stop_Brifly();
}

void em_default_handler_obs_wall_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	Move_Back();
	Stop_Brifly();
}

/* Cliff */
void em_default_handler_cliff_all(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_cliff_front_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_cliff_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_cliff_left_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_cliff_front(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_cliff_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_cliff_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* RCON */
void em_default_handler_rcon_front_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_rcon_front_left2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_rcon_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_rcon_front_right2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_rcon_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_rcon_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Over Current */
void em_default_handler_over_current_brush_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_over_current_brush_main(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_over_current_brush_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_over_current_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Key */
void em_default_handler_key_clean(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Remote */
void em_default_handler_remote_plan(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_remote_clean(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_remote_home(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_remote_direction_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	Reset_Rcon_Remote();
}

void em_default_handler_remote_direction_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	Reset_Rcon_Remote();
}

void em_default_handler_remote_mode_spot(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_remote_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	Switch_VacMode(false);
}

/* Battery */
void em_default_handler_battery_home(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void em_default_handler_battery_low(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Default: empty hanlder */
void em_default_handler_empty(bool state_now, bool state_last)
{
	ROS_INFO("%s %d: is called", __FUNCTION__, __LINE__);
}
