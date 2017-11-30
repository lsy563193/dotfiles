#ifndef __CORMOVE_H__
#define __CORMOVE_H__

#include <vector>
#include <deque>
#include <bitset>
#include "mathematics.h"
#include "event_manager.h"

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
extern bool g_is_left_start;
extern bool g_from_charger;

extern uint32_t g_wf_start_timer;
extern uint32_t g_wf_diff_timer;
extern bool g_motion_init_succeeded;
extern bool g_go_home_by_remote;
//extern Cell_t g_next_cell;
//extern Cell_t g_target_cell;
extern bool g_resume_cleaning;
extern bool g_have_seen_charger;
extern bool	g_start_point_seen_charger;
extern bool g_exploration_home;
extern std::deque<Cell_t> g_passed_path;
extern std::deque<Cell_t> g_plan_path;

uint8_t angle_to_bumper_status(void);
int16_t calc_target(int16_t);
int16_t uranged_angle(int16_t angle);

void CM_TouringCancel(void);
void cm_reset_go_home(void);
bool cm_head_to_course(uint8_t Speed, int16_t Angle);

void cm_follow_wall_turn(uint16_t speed, int16_t angle);
void linear_mark_clean(const Cell_t &start, const Cell_t &target);
MapTouringType CM_LinearMoveToPoint(Point32_t target);

bool CM_Check_is_exploring();
uint8_t CM_MoveForward(void);

uint8_t cm_touring(void);
void cm_cleaning(void);

void cm_apply_cs(int next);

	/* Robot will try to go to the cells in g_home_point_old_path list
	 * first, and it will only go through the CLEANED area. If the
	 * cell in g_home_point_new_path is unreachable through the
	 * CLEANED area, it will be push into g_home_point_new_path list.
	 * When all the cells in g_home_point_old_path list are unreachable
	 * or failed to go to charger, robot will start to go to cells in
	 * g_home_point_new_path through the UNCLEAN area (If there is a
	 * way like this).
	 */

bool cm_go_to_charger(void);

void cm_create_home_boundary(void);

void cm_self_check(void);

bool cm_should_self_check(void);
bool cm_should_go_to_charger(void);

/* Event handler functions. */
void cm_register_events(void);
void cm_unregister_events(void);

void cm_set_event_manager_handler_state(bool state);

void cm_block_charger_stub(int8_t direction);


class CM_EventHandle:public EventHandle {

	void bumper_all(bool state_now, bool state_last);

	void bumper_left(bool state_now, bool state_last);

	void bumper_right(bool state_now, bool state_last);

/* OBS */
	void obs_front(bool state_now, bool state_last);

	void obs_left(bool state_now, bool state_last);

	void obs_right(bool state_now, bool state_last);

/* Cliff */
	void cliff_all(bool state_now, bool state_last);

	void cliff_front_left(bool state_now, bool state_last);

	void cliff_front_right(bool state_now, bool state_last);

	void cliff_left_right(bool state_now, bool state_last);

	void cliff_front(bool state_now, bool state_last);

	void cliff_left(bool state_now, bool state_last);

	void cliff_right(bool state_now, bool state_last);

/* RCON */
	void rcon(bool state_now, bool state_last);
/* Over Current */
/*
void over_current_brush_left(bool state_now, bool state_last)
{
	static uint8_t stop_cnt = 0;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	if(!robot::instance()->getLbrushOc()) {
		g_oc_brush_left_cnt = 0;
		if (stop_cnt++ > 250) {
			set_left_brush_pwm(30);
		}
		return;
	}

	stop_cnt = 0;
	if (g_oc_brush_left_cnt++ > 40) {
		g_oc_brush_left_cnt = 0;
		set_left_brush_pwm(0);
		ROS_WARN("%s %d: left brush over current", __FUNCTION__, __LINE__);
	}
}
*/

	void over_current_brush_main(bool state_now, bool state_last);

	void over_current_brush_left(bool state_now, bool state_last) { df_over_current_brush_left(state_now, state_last); }

	void over_current_brush_right(bool state_now, bool state_last) { df_over_current_brush_right(state_now, state_last); }

/*

//void over_current_brush_right(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//
//	if(!robot::instance()->getRbrushOc()) {
//		g_oc_brush_right_cnt = 0;
//		if (stop_cnt++ > 250) {
//			set_right_brush_pwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush_right_cnt++ > 40) {
//		g_oc_brush_right_cnt = 0;
//		set_right_brush_pwm(0);
//		ROS_WARN("%s %d: reft brush over current", __FUNCTION__, __LINE__);
//	}
//}
*/

	void over_current_wheel_left(bool state_now, bool state_last);

	void over_current_wheel_right(bool state_now, bool state_last);

	void over_current_suction(bool state_now, bool state_last);

/* Key */
	void key_clean(bool state_now, bool state_last);

/* Remote */
	void remote_clean(bool state_now, bool state_last);

	void remote_home(bool state_now, bool state_last);

	void remote_spot(bool state_now, bool state_last);

	void remote_wall_follow(bool state_now, bool state_last) { df_remote_wall_follow(state_now, state_last); }

	void remote_max(bool state_now, bool state_last);

	void remote_direction_left(bool state_now, bool state_last) { df_remote_direction_left(state_now, state_last); }

	void remote_direction_right(bool state_now, bool state_last) { df_remote_direction_right(state_now, state_last); }

	void remote_direction_forward(bool state_now, bool state_last) { df_remote_direction_forward(state_now, state_last); }

	void remote_plan(bool state_now, bool state_last) { df_remote_plan(state_now, state_last); }

/* Battery */
	void battery_home(bool state_now, bool state_last);

	void battery_low(bool state_now, bool state_last);

	void charge_detect(bool state_now, bool state_last);

	void robot_slip(bool state_now, bool state_last) { df_robot_slip(state_now, state_last); }

	void slam_error(bool state_now, bool state_last) { df_slam_error(state_now, state_last); }

	void lidar_stuck(bool state_now, bool state_last) { df_lidar_stuck(state_now, state_last); }
};
#endif

