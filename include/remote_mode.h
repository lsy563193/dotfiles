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

	void bumperAll(bool state_now, bool state_last);

	void bumperRight(bool state_now, bool state_last);

	void bumperLeft(bool state_now, bool state_last);

	void cliffAll(bool state_now, bool state_last);

	void cliff(bool state_now, bool state_last);

	void cliffFront(bool state_now, bool state_last);

	void cliffLeft(bool state_now, bool state_last);

	void cliffRight(bool state_now, bool state_last);

	void cliffLeftRight(bool state_now, bool state_last);

	void cliffFrontLeft(bool state_now, bool state_last);

	void cliffFrontRight(bool state_now, bool state_last);

	void obs(bool state_now, bool state_last);

	void obsFront(bool state_now, bool state_last);

	void obsLeft(bool state_now, bool state_last);

	void obsRight(bool state_now, bool state_last);

	void remoteDirectionForward(bool state_now, bool state_last);

	void remoteDirectionLeft(bool state_now, bool state_last);

	void remoteDirectionRight(bool state_now, bool state_last);

	void remoteMax(bool state_now, bool state_last);

	void remote_exit(bool state_now, bool state_last);

	void remoteClean(bool state_now, bool state_last);

	void remoteHome(bool state_now, bool state_last);

	void remoteSpot(bool state_now, bool state_last);

	void remoteWallFollow(bool state_now, bool state_last);

	void rcon(bool state_now, bool state_last);

	void keyClean(bool state_now, bool state_last);

	void chargeDetect(bool state_now, bool state_last);

	void overCurrentWheelLeft(bool state_now, bool state_last);

	void overCurrentWheelRight(bool state_now, bool state_last);

	void overCurrentSuction(bool state_now, bool state_last);

	void overCurrentBrushLeft(bool state_now, bool state_last) { df_over_current_brush_left(state_now, state_last); }

	void overCurrentBrushRight(bool state_now, bool state_last) { df_over_current_brush_right(state_now, state_last); }

	void batteryLow(bool state_now, bool state_last);

	void remotePlan(bool state_now, bool state_last) { df_remote_plan(state_now, state_last); }
};

#endif /*----Behaviors------*/
