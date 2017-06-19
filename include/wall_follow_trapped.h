//#ifndef __WALL_FOLLOW_TRAPPED_H__
//#define __WALL_FOLLOW_TRAPPED_H__
//
//#include "config.h"
//
//#include "mathematics.h"
//#include "debug.h"
//#include <functional>
//#include <future>
//typedef enum {
//	Map_Wall_Follow_None = 0,
//	Map_Wall_Follow_Escape_Trapped,
//} MapWallFollowType;
//
//typedef enum {
//	Map_Escape_Trapped_Escaped,
//	Map_Escape_Trapped_Trapped,
//	Map_Escape_Trapped_Timeout,
//} MapEscapeTrappedType;
//
//typedef struct {
//	int32_t front_obs_val;
//	int32_t left_bumper_val;
//	int32_t right_bumper_val;
//} MapWallFollowSetting;
//
//uint8_t Map_Wall_Follow(MapWallFollowType follow_type);
//#endif


#ifndef __WALL_FOLLOW_TRAPPED_H__
#define __WALL_FOLLOW_TRAPPED_H__

#include "config.h"

#include "mathematics.h"
#include "debug.h"

typedef enum {
	Escape_Trapped_Escaped,
	Escape_Trapped_Trapped,
	Escape_Trapped_Timeout,
	Escape_Trapped_Fatal,
	Escape_Trapped_Key_Clean,
} EscapeTrappedType;

typedef enum {
	Map_Wall_Follow_None = 0,
	Map_Wall_Follow_Escape_Trapped,
} MapWallFollowType;

typedef struct {
	int32_t front_obs_val;
	int32_t left_bumper_val;
	int32_t right_bumper_val;
} MapWallFollowSetting;

EscapeTrappedType Wall_Follow_Trapped();

/* Event handler functions. */
void WFT_regist_events(void);
void WFT_unregist_events(void);

#define define_wft_handle_func(name) \
	void WFT_handle_ ## name(bool state_now, bool state_last);

/* Bumper */
define_wft_handle_func(bumper_all)
define_wft_handle_func(bumper_left)
define_wft_handle_func(bumper_right)

/* Cliff */
define_wft_handle_func(cliff_all)
define_wft_handle_func(cliff_front_left)
define_wft_handle_func(cliff_front_right)
define_wft_handle_func(cliff_left_right)
define_wft_handle_func(cliff_front)
define_wft_handle_func(cliff_left)
define_wft_handle_func(cliff_right)

/* RCON */
define_wft_handle_func(rcon)
/*
define_wft_handle_func(rcon_front_left)
define_wft_handle_func(rcon_front_left2)
define_wft_handle_func(rcon_front_right)
define_wft_handle_func(rcon_front_right2)
define_wft_handle_func(rcon_left)
define_wft_handle_func(rcon_right)
*/

/* Over Current */
define_wft_handle_func(over_current_brush_left)
define_wft_handle_func(over_current_brush_main)
define_wft_handle_func(over_current_brush_right)
define_wft_handle_func(over_current_wheel_left)
define_wft_handle_func(over_current_wheel_right)
define_wft_handle_func(over_current_suction)

/* Key */
define_wft_handle_func(key_clean)

/* Remote */
define_wft_handle_func(remote_clean)
define_wft_handle_func(remote_max)

/* Battery */
define_wft_handle_func(battery_low)


#endif
