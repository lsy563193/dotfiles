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

enum {
	EXIT_CLEAN=-1,
	NO_REATH_TARGET=0,
	REATH_TARGET=1,
};

typedef struct {
	Cell_t	pos;
} VWType;

extern float saved_pos_x, saved_pos_y;
extern bool g_move_back_finished;
extern uint16_t g_straight_distance;
extern bool g_is_left_start;
extern bool g_finish_cleaning_go_home;
extern bool g_from_station;
extern int g_trapped_mode;
extern uint32_t g_wf_start_timer;
extern uint32_t g_wf_diff_timer;
extern bool g_motion_init_succeeded;
extern bool g_go_home_by_remote;
extern int g_rcon_triggered;
extern Cell_t g_next_cell, g_target_cell;
extern bool g_resume_cleaning;
extern bool g_exploration_home;

uint8_t angle_to_bumper_status(void);
int16_t calc_target(int16_t);
int16_t uranged_angle(int16_t angle);
extern int16_t ranged_angle(int16_t angle);
bool is_map_front_block(int dx);

void CM_TouringCancel(void);
void cm_reset_go_home(void);
bool cm_head_to_course(uint8_t Speed, int16_t Angle);

void cm_follow_wall_turn(uint16_t speed, int16_t angle);
void linear_mark_clean(const Cell_t &start, const Cell_t &target);
MapTouringType CM_LinearMoveToPoint(Point32_t target);

int cm_get_grid_index(float position_x, float position_y, uint32_t width, uint32_t height, float resolution,
											double origin_x, double origin_y);
bool CM_Check_is_exploring();
uint8_t CM_MoveForward(void);

uint8_t cm_touring(void);
void cm_cleaning(void);

void cm_check_should_go_home(void);
void cm_check_temp_spot(void);

Cell_t cm_update_position(bool is_turn = false);
//void cm_update_map();
bool cm_curve_move_to_point();

void cm_world_to_point(int16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);
void cm_world_to_point_accurate(int16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);
void cm_world_to_cell(int16_t heading, int16_t offset_lat, int16_t offset_long, int16_t &x, int16_t &y);

void mark_offset(int16_t dx, int16_t dy, CellState status);

void cm_move_back_(uint16_t dist);

	/* Robot will try to go to the cells in g_home_point_old_path list
	 * first, and it will only go through the CLEANED area. If the
	 * cell in g_home_point_new_path is unreachable through the
	 * CLEANED area, it will be push into g_home_point_new_path list.
	 * When all the cells in g_home_point_old_path list are unreachable
	 * or failed to go to charger, robot will start to go to cells in
	 * g_home_point_new_path through the UNCLEAN area (If there is a
	 * way like this).
	 */

void cm_go_home(void);
bool cm_go_to_charger(void);
bool cm_is_continue_go_to_charger(void);
//void CM_SetStationHome(void);

void CM_ResetBoundaryBlocks(void);

void CM_AddTargets(Cell_t zone);

bool cm_check_loop_back(Cell_t target);

MapTouringType cm_handle_ext_event(void);

void cm_create_home_boundary(void);

void cm_self_check(void);
bool cm_should_self_check(void);

uint8_t cm_turn_and_check_charger_signal(void);
/* Event handler functions. */
void cm_register_events(void);
void cm_unregister_events(void);

void cm_set_event_manager_handler_state(bool state);

void cm_event_manager_turn(bool state);

void cm_block_charger_stub(int8_t direction);
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
define_cm_handle_func(rcon)
/*
define_cm_handle_func(rcon_front_left)

define_cm_handle_func(rcon_front_left2)

define_cm_handle_func(rcon_front_right)

define_cm_handle_func(rcon_front_right2)

define_cm_handle_func(rcon_left)

define_cm_handle_func(rcon_right)
*/

/* Over Current */
//define_cm_handle_func(over_current_brush_left)

define_cm_handle_func(over_current_brush_main)

//define_cm_handle_func(over_current_brush_right)

define_cm_handle_func(over_current_wheel_left)

define_cm_handle_func(over_current_wheel_right)

define_cm_handle_func(over_current_suction)

/* Key */
define_cm_handle_func(key_clean)

/* Remote */
define_cm_handle_func(remote_plan)

define_cm_handle_func(remote_clean)

define_cm_handle_func(remote_home)

define_cm_handle_func(remote_spot)

define_cm_handle_func(remote_wallfollow);

define_cm_handle_func(remote_max)

define_cm_handle_func(remote_direction)

/* Battery */
define_cm_handle_func(battery_home)

define_cm_handle_func(battery_low)

/* Charge Status */
define_cm_handle_func(charge_detect)
#endif

