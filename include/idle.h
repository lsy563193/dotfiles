/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V0.0
  * @date    11-July-2011
  * @brief   Display Fuctions
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

#ifndef __UserInterface_H
#define __UserInterface_H

#define Set_Clock_Flag	0x01
#define Set_Plan_Flag   0x02
#define Set_Hours				0x03
#define Set_Minutes     0x04

#define Status_Hour   1
#define Status_Minute 2

#define Edit_Add      1
#define Edit_Sub      2

#include "stdint.h"
#include "event_manager.h"

void idle(void);

typedef struct
{
	uint8_t Hours;
	uint8_t Minutes;
}Time_Struct;

typedef struct
{
	uint32_t Mon;
	uint32_t Tue;
	uint32_t Wed;
	uint32_t Thu;
	uint32_t Fri;
	uint32_t Sat;
	uint32_t Sun;
}Plan_Struct;

class Idle_EventHandle:public EventHandle {
	void cliff_(bool state_now, bool state_last);

	void cliff_left(bool state_now, bool state_last);

	void cliff_left_right(bool state_now, bool state_last);

	void cliff_right(bool state_now, bool state_last);

	void cliff_front(bool state_now, bool state_last);

	void cliff_front_left(bool state_now, bool state_last);

	void cliff_front_right(bool state_now, bool state_last);

	void rcon(bool state_now, bool state_last);

	void battery_low(bool state_now, bool state_last);

	void remote_cleaning(bool state_now, bool state_last);
	void remote_direction_left(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_direction_right(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_direction_forward(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_home(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_spot(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_wall_follow(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_clean(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}

	void remote_plan(bool state_now, bool state_last);

	void key_clean(bool state_now, bool state_last);

	void charge_detect(bool state_now, bool state_last);

	void remote_max(bool stat_now, bool state_last) { df_remote_max(stat_now, state_last); }
};

#endif /* __DISPLAY_H */

