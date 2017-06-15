#ifndef __CORMOVE_H__
#define __CORMOVE_H__

#include "mathematics.h"
#include "debug.h"
#include <vector>
#include <bitset>

#define MS_Clear 		0x00
#define MS_OBS   		0x01
#define MS_Bumper		0x02
#define MS_Cliff 		0x04
#define MS_User 		0x08
#define MS_Home			0x10
#define MS_Clean 		0x20
#define MS_Spot 		0x40
#define MS_Error 		0x80

#define TILT_PITCH_LIMIT	(100)
#define TILT_ROLL_LIMIT		(100)

#define COR_BACK_20MM		(120)
#define COR_BACK_100MM		(600)
#define COR_BACK_500MM		(3000)

typedef enum {
	ROUNDING_NONE = 0,
	ROUNDING_LEFT,
	ROUNDING_RIGHT,
} RoundingType;

typedef enum {
	CM_LINEARMOVE = 0,
	CM_CURVEMOVE,
	CM_ROUNDING,
} CMMoveType;

typedef enum {
	MT_None = 0,
	MT_Battery,
	MT_Remote_Home,
	MT_Remote_Clean,
	MT_Remote_Spot,
	MT_Cliff,
	MT_Bumper,
	MT_OBS,
	MT_Boundary,
	MT_CurveMove,
	MT_Key_Clean,
	MT_Battery_Home,
} MapTouringType;

typedef enum {
	ACTION_NONE	= 0x01,
	ACTION_GO	= 0x02,
	ACTION_BACK	= 0x04,
	ACTION_LT	= 0x08,
	ACTION_RT	= 0x10,
} ActionType;

typedef struct {
	Cell_t	pos;
} VWType;

void CM_TouringCancel(void);
void cm_reset_go_home(void);
void cm_head_to_course(uint8_t Speed, int16_t Angle);

MapTouringType CM_LinearMoveToPoint(Point32_t target);
bool cm_linear_move_to_point(Point32_t Target, int32_t speed_max, bool stop_is_needed, bool rotate_is_needed);

int cm_get_grid_index(float position_x, float position_y, uint32_t width, uint32_t height, float resolution,
											double origin_x, double origin_y);
bool CM_Check_is_exploring();
uint8_t CM_MoveForward(void);

uint8_t cm_touring(void);

void cm_update_position(bool is_turn = false);
void cm_update_map();
void cm_update_map_bumper();
void cm_update_map_cliff();
bool cm_curve_move_to_point();

void cm_count_normalize(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);

bool cm_move_to_cell(int16_t x, int16_t y);
//int8_t CM_MoveToCell( int16_t x, int16_t y);

void cm_move_back(uint16_t dist);

void cm_set_home(int32_t x, int32_t y);
void cm_go_home(void);
bool cm_go_to_charger(Cell_t current_home_cell);
//void CM_SetStationHome(void);

// This function is for setting the continue point for robot to go after charge.
void cm_set_continue_point(int32_t x, int32_t y);

void CM_ResetBoundaryBlocks(void);

void CM_AddTargets(Cell_t zone);

uint8_t cm_check_loop_back(Cell_t target);

MapTouringType cm_handle_ext_event(void);

void cm_create_home_boundary(void);

/* Event handler functions. */
void cm_regist_events(void);
void cm_unregist_events(void);

void cm_set_event_manager_handler_state(bool state);

void cm_event_manager_turn(bool state);

#define define_cm_handle_func(name) \
	void cm_handle_ ## name(bool state_now, bool state_last);

/* Bumper */
define_cm_handle_func(bumper_all)

define_cm_handle_func(bumper_left)

define_cm_handle_func(bumper_right)

/* OBS */
define_cm_handle_func(obs_front)

define_cm_handle_func(obs_left)

define_cm_handle_func(obs_right)

/* Cliff */
define_cm_handle_func(cliff_all)

define_cm_handle_func(cliff_front_left)

define_cm_handle_func(cliff_front_right)

define_cm_handle_func(cliff_left_right)

define_cm_handle_func(cliff_front)

define_cm_handle_func(cliff_left)

define_cm_handle_func(cliff_right)

/* RCON */
define_cm_handle_func(rcon_front_left)

define_cm_handle_func(rcon_front_left2)

define_cm_handle_func(rcon_front_right)

define_cm_handle_func(rcon_front_right2)

define_cm_handle_func(rcon_left)

define_cm_handle_func(rcon_right)

/* Over Current */
define_cm_handle_func(over_current_brush_left)

define_cm_handle_func(over_current_brush_main)

define_cm_handle_func(over_current_brush_right)

define_cm_handle_func(over_current_wheel_left)

define_cm_handle_func(over_current_wheel_right)

define_cm_handle_func(over_current_suction)

/* Key */
define_cm_handle_func(key_clean)

/* Remote */
define_cm_handle_func(remote_plan)

define_cm_handle_func(remote_clean)

define_cm_handle_func(remote_home)

define_cm_handle_func(remote_mode_spot)

define_cm_handle_func(remote_suction)

/* Battery */
define_cm_handle_func(battery_home)

define_cm_handle_func(battery_low)

#endif

