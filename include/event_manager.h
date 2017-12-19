#ifndef __EVENT_MANAGER__
#define __EVENT_MANAGER__

/* MS in nano */
#define _1MS			(1 * 1000 * 1000)
#define _10MS			(10 * _1MS)
#define _100MS			(10 * _10MS)
#define _1S				(10 * _100MS)

#define NANO_SEC_MAX	(999999999)

#define timespec_rationalize(x)					\
	do {										\
		while (x.tv_nsec > NANO_SEC_MAX)		\
			x.tv_nsec -= _1S, x.tv_sec += 1;	\
	} while (0)

#define timespec_add_ns(x, ns)		\
	do {							\
		x.tv_nsec += ns;			\
		timespec_rationalize(x);	\
	} while (0)

#define check_limit(val, min, max)	\
	if (val < min) {				\
		val = min;					\
	} else if (val > max) {			\
		val = max;					\
	}

#include "stdint.h"
#include <pthread.h>

/* Events variables */
/* The fatal quit event includes any of the following case:
 *  ev.bumper_jam
 * 	g_cliff_all_triggered
 * 	g_oc_brush_main
 * 	g_oc_wheel_left
 * 	g_oc_wheel_right
 * 	g_oc_suction
 * 	g_battery_low
 */

typedef struct {
		bool remote_home;
		bool battery_home;
		bool battery_low;
		bool remote_spot;

		bool battrey_home;
		bool fatal_quit;
		uint8_t bumper_triggered;
		int rcon_triggered;
		uint8_t obs_triggered;
		bool bumper_jam;
		uint8_t cliff_triggered;
		bool cliff_jam;

		bool oc_brush_main;
		bool oc_wheel_left;
		bool oc_wheel_right;
		bool oc_suction;
		bool key_clean_pressed;
		bool key_long_pressed;
		bool remote_wallfollow;
		bool remote_direction_left;
		bool remote_direction_right;
		bool remote_direction_forward;
		bool remote_direction_back;
		bool slam_error;
		bool tilt_triggered;
		uint8_t charge_detect;
		bool lidar_stuck = false;
		uint8_t lidar_triggered;
		bool cliff_all_triggered;
}Ev_t;
class EventHandle{
public:
virtual void bumperAll(bool state_now, bool state_last);

virtual void bumperLeft(bool state_now, bool state_last);

virtual void bumperRight(bool state_now, bool state_last);

/* OBS */
virtual void obsFront(bool state_now, bool state_last);

virtual void obsLeft(bool state_now, bool state_last);

virtual void obsRight(bool state_now, bool state_last);

virtual void obsWallLeft(bool state_now, bool state_last);

virtual void obsWallRight(bool state_now, bool state_last);

/* Cliff */
virtual void cliffAll(bool state_now, bool state_last);

virtual void cliffFrontLeft(bool state_now, bool state_last);

virtual void cliffFrontRight(bool state_now, bool state_last);

virtual void cliffLeftRight(bool state_now, bool state_last);

virtual void cliffFront(bool state_now, bool state_last);

virtual void cliffLeft(bool state_now, bool state_last);

virtual void cliffRight(bool state_now, bool state_last);

/* RCON */
virtual void rcon(bool state_now, bool state_last);
/*
void rcon_front_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void rcon_front_left2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void rcon_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void rcon_front_right2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void rcon_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void rcon_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}
*/

/* Over Current */
virtual void overCurrentBrushLeft(bool state_now, bool state_last);

virtual void overCurrentBrushMain(bool state_now, bool state_last);

virtual void overCurrentBrushRight(bool state_now, bool state_last);

virtual void overCurrentWheelLeft(bool state_now, bool state_last);

virtual void overCurrentWheelRight(bool state_now, bool state_last);

virtual void overCurrentSuction(bool state_now, bool state_last);

/* Key */
virtual void keyClean(bool state_now, bool state_last);

/* Remote */
virtual void remotePlan(bool state_now, bool state_last);

virtual void remoteClean(bool state_now, bool state_last);

virtual void remoteHome(bool state_now, bool state_last);

virtual void remoteDirectionForward(bool state_now, bool state_last);

virtual void remoteWallFollow(bool state_now, bool state_last);

virtual void remoteDirectionLeft(bool state_now, bool state_last);

virtual void remoteDirectionRight(bool state_now, bool state_last);

virtual void remoteSpot(bool state_now, bool state_last);

virtual void remoteMax(bool state_now, bool state_last);

/* Battery */
virtual void batteryHome(bool state_now, bool state_last);

virtual void batteryLow(bool state_now, bool state_last);

virtual void chargeDetect(bool state_now, bool state_last);

virtual void robotSlip(bool state_new, bool state_last);
/*
void lidar_bumper(bool state_new,bool state_last)
{
	g_lidar_bumper = robot::instance()->getLidarBumper();
}
*/

// Lidar stuck
virtual void lidarStuck(bool state_new, bool state_last);
/* Default: empty hanlder */
//void empty(bool state_now, bool state_last);

};

/* Bumper */
extern int g_bumper_cnt;
/* OBS */
/* Cliff */
extern bool g_cliff_all_triggered;
extern uint8_t g_cliff_all_cnt;
extern int g_cliff_cnt;
/* RCON */
extern bool g_rcon_during_go_home;
/* Over Current */

extern uint8_t g_oc_wheel_left_cnt;
extern uint8_t g_oc_wheel_right_cnt;
extern uint8_t g_oc_suction_cnt;
/* Key */
extern bool g_key_clean_pressed;
/* Remote */
extern Ev_t ev;
/* Battery */
extern uint8_t g_battery_low_cnt;
/* Charge status */
extern uint8_t g_charge_detect_cnt;
/* Plan */
extern bool g_plan_activated;
/* tilt enable */
extern bool g_tilt_triggered;
/* robot stuck */
extern bool g_robot_stuck;
extern bool g_robot_slip;
extern bool g_wf_is_reach;
extern bool g_robot_slip_enable;
extern uint8_t g_slip_cnt;

typedef enum {
	EVT_BUMPER_ALL = 0,
	EVT_BUMPER_LEFT,
	EVT_BUMPER_RIGHT,

	EVT_OBS_FRONT,
	EVT_OBS_LEFT,
	EVT_OBS_RIGHT,

	EVT_OBS_WALL_LFET,
	EVT_OBS_WALL_RIGHT,

	EVT_CLIFF_ALL,
	EVT_CLIFF_FRONT_LEFT,
	EVT_CLIFF_FRONT_RIGHT,
	EVT_CLIFF_LEFT_RIGHT,
	EVT_CLIFF_FRONT,
	EVT_CLIFF_LEFT,
	EVT_CLIFF_RIGHT,

	EVT_RCON,
/*
	EVT_RCON_FRONT_LEFT,
	EVT_RCON_FRONT_LEFT2,
	EVT_RCON_FRONT_RIGHT,
	EVT_RCON_FRONT_RIGHT2,
	EVT_RCON_LEFT,
	EVT_RCON_RIGHT,
*/

	EVT_OVER_CURRENT_BRUSH_LEFT,
	EVT_OVER_CURRENT_BRUSH_MAIN,
	EVT_OVER_CURRENT_BRUSH_RIGHT,
	EVT_OVER_CURRENT_WHEEL_LEFT,
	EVT_OVER_CURRENT_WHEEL_RIGHT,
	EVT_OVER_CURRENT_SUCTION,

	EVT_KEY_CLEAN,
	EVT_KEY_RANDOM,

	EVT_REMOTE_PLAN,

	EVT_REMOTE_CLEAN,
	EVT_REMOTE_HOME,

	EVT_REMOTE_DIRECTION_BACKWARD,
	EVT_REMOTE_DIRECTION_FORWARD,
	EVT_REMOTE_DIRECTION_LEFT,
	EVT_REMOTE_DIRECTION_RIGHT,

	EVT_REMOTE_WALL_FOLLOW,
	EVT_REMOTE_RANDOM,
	EVT_REMOTE_SPOT,

	EVT_REMOTE_MAX,

	EVT_REMOTE_TIMER,

	EVT_WATER_TANK,

	EVT_BATTERY_HOME,
	EVT_BATTERY_LOW,

	EVT_CHARGE_DETECT,

	EVT_ROBOT_SLIP,

	EVT_LIDAR_BUMPER,

	EVT_LIDAR_STUCK,

	EVT_MAX,
} EventType;


/*typedef enum {
	EVT_MODE_USER_INTERFACE = 0,
	EVT_MODE_CLEAN,
	EVT_MODE_HOME,
	EVT_MODE_NAVIGATION,
	EVT_MODE_WALL_FOLLOW_TRAPPED,
	EVT_MODE_WALL_FOLLOW,
	EVT_MODE_RANDOM,
	EVT_MODE_SPOT,
	EVT_MODE_CHARGE,
	EVT_MODE_SLEEP,
	EVT_MODE_REMOTE,
	EVT_MODE_MAX,
} EventModeType;*/
typedef void(EventHandle::*PEHF)(bool state_now, bool state_last);
typedef void(*event_handle_t)(bool state_now, bool state_last);
typedef struct {
//	EventModeType	emt;
//	bool	handler_enabled[EVT_MAX];
	EventHandle* peh;
//	event_handle_t handler[EVT_MAX];
} EventActionType;

extern bool	event_handler_status;
extern pthread_mutex_t event_handler_mtx;
extern pthread_cond_t event_handler_cond;

void event_manager_init();

void event_manager_set_enable(bool enable);

void event_manager_thread_cb();

void event_handler_thread_cb();

//void event_manager_set_current_mode(EventModeType mode);

void event_manager_register_handler(EventHandle* eh);

void event_manager_enable_handler(EventType type, bool enabled);

uint8_t event_manager_check_event(bool *eh_status_now, bool *eh_status_last);

void event_manager_reset_status(void);

/* Below are the internal functions. */
/* Below are the internal functions. */

/* Bumper */

#endif
