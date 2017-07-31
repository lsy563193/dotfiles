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
extern bool g_fatal_quit_event;
/* Bumper */
extern int g_bumper_triggered;
extern bool g_bumper_jam;
extern int g_bumper_cnt;
/* OBS */
extern int g_obs_triggered;
/* Cliff */
extern bool g_cliff_all_triggered;
extern int g_cliff_triggered;
extern bool g_cliff_jam;
extern uint8_t g_cliff_all_cnt;
extern int g_cliff_cnt;
/* RCON */
extern int g_rcon_triggered;
extern bool g_rcon_during_go_home;
/* Over Current */
extern bool g_oc_brush_main;
extern bool g_oc_wheel_left;
extern bool g_oc_wheel_right;
extern bool g_oc_suction;
extern uint8_t g_oc_brush_left_cnt;
extern uint8_t g_oc_brush_main_cnt;
extern uint8_t g_oc_brush_right_cnt;
extern uint8_t g_oc_wheel_left_cnt;
extern uint8_t g_oc_wheel_right_cnt;
extern uint8_t g_oc_suction_cnt;
/* Key */
extern bool g_key_clean_pressed;
/* Remote */
extern bool g_remote_home;
extern bool g_go_home;
extern bool g_remote_spot;
extern bool g_remote_wallfollow;
extern bool g_remote_direction_keys;
/* Battery */
extern bool g_battery_home;
extern bool g_battery_low;
extern uint8_t g_battery_low_cnt;
/* Charge status */
extern uint8_t g_charge_detect;
extern uint8_t g_charge_detect_cnt;
/* Slam Error */
extern bool g_slam_error;
/* Plan */
extern bool g_plan_activated;
/* Omni wheel*/
extern bool g_omni_notmove;

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

	EVT_SLAM_ERROR,

	EVT_MAX,
} EventType;


typedef enum {
	EVT_MODE_USER_INTERFACE = 0,
	EVT_MODE_CLEAN,
	EVT_MODE_HOME,
	EVT_MODE_NAVIGATION,
	EVT_MODE_WALL_FOLLOW_TRAPPED,
	EVT_MODE_WALL_FOLLOW,
	EVT_MODE_RANDOM,
	EVT_MODE_SPOT,
	EVT_MODE_MAX,
	EVT_MODE_CHARGE,
	EVT_MODE_SLEEP,
	EVT_MODE_REMOTE,
} EventModeType;

typedef struct {
	EventModeType	emt;

	bool	handler_enabled[EVT_MAX];
	void	(*handler[EVT_MAX])(bool state_now, bool state_last);
} EventActionType;

extern bool	event_handler_status;
extern pthread_mutex_t event_handler_mtx;
extern pthread_cond_t event_handler_cond;

void event_manager_init();

void event_manager_set_enable(bool enable);

void *event_manager_thread(void *data);

void *event_handler_thread(void *data);

void event_manager_set_current_mode(EventModeType mode);

void event_manager_register_handler(EventType type, void (*func)(bool state_now, bool state_last));

void event_manager_enable_handler(EventType type, bool enabled);

uint8_t event_manager_check_event(bool *eh_status_now, bool *eh_status_last);

void event_manager_reset_status(void);

/* Below are the internal functions. */

#define define_em_handler_func(name) \
	void em_default_handle_ ## name(bool state_now, bool state_last);

/* Bumper */
define_em_handler_func(bumper_all)
define_em_handler_func(bumper_left)
define_em_handler_func(bumper_right)

/* OBS */
define_em_handler_func(obs_front)
define_em_handler_func(obs_left)
define_em_handler_func(obs_right)
define_em_handler_func(obs_wall_left)
define_em_handler_func(obs_wall_right)

/* Cliff */
define_em_handler_func(cliff_all)
define_em_handler_func(cliff_front_left)
define_em_handler_func(cliff_front_right)
define_em_handler_func(cliff_left_right)
define_em_handler_func(cliff_front)
define_em_handler_func(cliff_left)
define_em_handler_func(cliff_right)

/* RCON */
define_em_handler_func(rcon)
/*
define_em_handler_func(rcon_front_left)
define_em_handler_func(rcon_front_left2)
define_em_handler_func(rcon_front_right)
define_em_handler_func(rcon_front_right2)
define_em_handler_func(rcon_left)
define_em_handler_func(rcon_right)
*/

/* Over Current */
define_em_handler_func(over_current_brush_left)
define_em_handler_func(over_current_brush_main)
define_em_handler_func(over_current_brush_right)
define_em_handler_func(over_current_wheel_left)
define_em_handler_func(over_current_wheel_right)
define_em_handler_func(over_current_suction)

/* Key */
define_em_handler_func(key_clean)

/* Remote */
define_em_handler_func(remote_plan)
define_em_handler_func(remote_clean)
define_em_handler_func(remote_home)
define_em_handler_func(remote_direction_forward)
define_em_handler_func(remote_direction_left)
define_em_handler_func(remote_direction_right)
define_em_handler_func(remote_spot)
define_em_handler_func(remote_max)
define_em_handler_func(remote_wall_follow)

/* Battery */
define_em_handler_func(battery_home)
define_em_handler_func(battery_low)

/* Charge Status */
define_em_handler_func(charge_detect)

/* Slam Error */
define_em_handler_func(slam_error)

define_em_handler_func(empty)

#endif
