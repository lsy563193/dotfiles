/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V0.0
  * @date    11-July-2011
  * @brief   System Initialize
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

#ifndef __Remote_Mode_H
#define __Remote_Mode_H

#include "event_manager.h"
typedef enum {
	REMOTE_MODE_FORWARD = 0,
	REMOTE_MODE_LEFT,
	REMOTE_MODE_RIGHT,
	REMOTE_MODE_BACKWARD,
	REMOTE_MODE_STAY,
} RemoteModeMoveType;

void remote_mode(void);
void remote_move(void);
void set_move_flag_(RemoteModeMoveType flag);
RemoteModeMoveType get_move_flag_(void);
void remote_mode_register_events(void);
void remote_mode_unregister_events(void);

#define define_func(name) \
	void  ## name(bool state_now, bool state_last);


class RM_EventHandle:public EventHandle {

	void bumper(bool state_now, bool state_last);

	void bumper_all(bool state_now, bool state_last);

	void bumper_right(bool state_now, bool state_last);

	void bumper_left(bool state_now, bool state_last);

	void cliff_all(bool state_now, bool state_last);

	void cliff(bool state_now, bool state_last);

	void cliff_front(bool state_now, bool state_last);

	void cliff_left(bool state_now, bool state_last);

	void cliff_right(bool state_now, bool state_last);

	void cliff_left_right(bool state_now, bool state_last);

	void cliff_front_left(bool state_now, bool state_last);

	void cliff_front_right(bool state_now, bool state_last);

	void obs(bool state_now, bool state_last);

	void obs_front(bool state_now, bool state_last);

	void obs_left(bool state_now, bool state_last);

	void obs_right(bool state_now, bool state_last);

	void remote_direction_forward(bool state_now, bool state_last);

	void remote_direction_left(bool state_now, bool state_last);

	void remote_direction_right(bool state_now, bool state_last);

	void remote_max(bool state_now, bool state_last);

	void remote_exit(bool state_now, bool state_last);

	void remote_clean(bool state_now, bool state_last);

	void remote_home(bool state_now, bool state_last);

	void remote_spot(bool state_now, bool state_last);

	void remote_wall_follow(bool state_now, bool state_last);

	void rcon(bool state_now, bool state_last);

	void key_clean(bool state_now, bool state_last);

	void charge_detect(bool state_now, bool state_last);

	void over_current_wheel_left(bool state_now, bool state_last);

	void over_current_wheel_right(bool state_now, bool state_last);

	void over_current_suction(bool state_now, bool state_last);

	void over_current_brush_left(bool state_now, bool state_last) { df_over_current_brush_left(state_now, state_last); }

	void over_current_brush_right(bool state_now, bool state_last) { df_over_current_brush_right(state_now, state_last); }

	void battery_low(bool state_now, bool state_last);

	void remote_plan(bool state_now, bool state_last) { df_remote_plan(state_now, state_last); }
};

#endif /*----Behaviors------*/
