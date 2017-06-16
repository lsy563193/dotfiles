/*
 ******************************************************************************
 * @file	AI Cleaning Robot
 * @author	ILife Team Dxsong
 * @version V1.0
 * @date	17-Nov-2011
 * @brief	Move near the wall on the left in a certain distance
 ******************************************************************************
 * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
//#include <math.h>
//#include <stdio.h>
//#include <time.h>
//#include <unistd.h>
//#include <pthread.h>
//
//#include "robot.hpp"
//#include "movement.h"
//#include "core_move.h"
//#include "gyro.h"
//#include "map.h"
//#include "mathematics.h"
//#include "path_planning.h"
//#include "wall_follow_trapped.h"
//#include <ros/ros.h>
//#include "debug.h"
//#include "charger.hpp"
//#include "wav.h"
//
//#include "motion_manage.h"
////Turn speed
//#ifdef Turn_Speed
//#undef Turn_Speed
//#define Turn_Speed	18
//#endif
//
//bool	escape_thread_running = false;
////Timer
//uint32_t escape_trapped_timer;
////MFW setting
//static const MapWallFollowSetting MFW_SETTING[6]= {{1200, 250, 150 },
//	{1200, 250, 150},
//	{1200, 250, 150},
//	{1200, 250, 70},
//	{1200, 250, 150},
//	{1200, 250, 150},};
//
//
//extern int16_t WheelCount_Left, WheelCount_Right;
//
//
///************************************************************************
// * Normal End
// ************************************************************************/
//void WFM_move_back(uint16_t dist)
//{
//	float pos_x, pos_y, distance;
//	uint16_t Counter_Watcher = 0;
//	uint16_t Temp_Speed = 10;
//
//	ROS_INFO("%s %d: ", __FUNCTION__, __LINE__);
//	Stop_Brifly();
//	set_dir_backward();
//	Set_Wheel_Speed(5, 5);
//	Counter_Watcher = 0;
//
//	pos_x = robot::instance()->getOdomPositionX();
//	pos_y = robot::instance()->getOdomPositionY();
//	while (ros::ok()) {
//		distance = sqrtf(powf(pos_x - robot::instance()->getOdomPositionX(), 2) + powf(pos_y -
//																																													 robot::instance()->getOdomPositionY(), 2));
//		if (fabsf(distance) > 0.02f) {
//			break;
//		}
//
//		Temp_Speed = get_left_wheel_step() / 3 + 8;
//		if (Temp_Speed > 12) {
//			Temp_Speed = 12;
//		}
//		Set_Wheel_Speed(Temp_Speed, Temp_Speed);
//
//		usleep(10000);
//		Counter_Watcher++;
//		if (Counter_Watcher > 3000) {
//			if(is_encoder_fail()) {
//				set_error_code(Error_Code_Encoder);
//			}
//			return;
//		}
//		if (Stop_Event()) {
//			return;
//		}
//		uint8_t octype = Check_Motor_Current();
//		if ((octype == Check_Left_Wheel) || ( octype == Check_Right_Wheel)) {
//			ROS_INFO("%s,%d,motor over current",__FUNCTION__,__LINE__);
//			return;
//		}
//	}
//	set_dir_forward();
//	Set_Wheel_Speed(0, 0);
//}
//
//void *WFM_check_trapped(void *data)
//{
//	float			pos_x, pos_y;
//	int16_t			val;
//	uint32_t		left_speed, right_speed;
//	static int16_t	current_x = 0, current_y = 0;
//
//	MapEscapeTrappedType	escaped = Map_Escape_Trapped_Escaped;
//
//	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
//	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
//	map_set_position(pos_x, pos_y);
//	map_set_cell(MAP, pos_x, pos_y, CLEANED);
//
//	current_x = map_get_x_cell();
//	current_y = map_get_y_cell();
//	escape_thread_running = true;
//
//	ROS_INFO("%s %d: escape thread is up!", __FUNCTION__, __LINE__);
//	while (escape_thread_running == true) {
//		pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
//		pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
//		map_set_position(pos_x, pos_y);
//		map_set_cell(MAP, pos_x, pos_y, CLEANED);
//
//		if (abs(current_x - map_get_x_cell()) >= 2 || abs(current_y - map_get_y_cell()) >= 2) {
//			path_update_cell_history();
//			ROS_INFO("%s %d: escape thread checking: pos: (%d, %d) (%d, %d)!", __FUNCTION__, __LINE__, current_x, current_y, map_get_x_cell(), map_get_y_cell());
//			val = path_escape_trapped();
//			if (val == 1) {
//				ROS_INFO("%s %d: escaped, thread is existing!", __FUNCTION__, __LINE__);
//				data = (void *) (&escaped);
//				escape_thread_running = false;
//			}
//			current_x = map_get_x_cell();
//			current_y = map_get_y_cell();
//			} else {
//				usleep(100000);
//			}
//		}
//
//		escape_thread_running = false;
//		return NULL;
//}
//
//void WFM_boundary_check()
//{
//	uint8_t boundary_reach = 0;
//	int16_t	j;
//	int32_t	x, y;
//
//	for (j = -1; boundary_reach == 0 && j <= 1; j++) {
//#if (ROBOT_SIZE == 5)
//		x = map_get_relative_x(Gyro_GetAngle(), j * CELL_SIZE, CELL_SIZE_2);
//		y = map_get_relative_y(Gyro_GetAngle(), j * CELL_SIZE, CELL_SIZE_2);
//#else
//		x = map_get_relative_x(Gyro_GetAngle(), j * CELL_SIZE, CELL_SIZE_2);
//		y = map_get_relative_y(Gyro_GetAngle(), j * CELL_SIZE, CELL_SIZE_2);
//#endif
//
//		if (map_get_cell(MAP, countToCell(x), count_to_cell(y)) == BLOCKED_BOUNDARY) {
//			boundary_reach = 1;
//			Set_Wheel_Speed(0, 0);
//			usleep(10000);
//
//			WFM_move_back(350);
//			Turn_Right(Turn_Speed, 600);
//			Move_Forward(15, 15);
//		}
//	}
//}
//
///*------------------------------------------------------------------ Wall Follow --------------------------*/
//uint8_t Map_Wall_Follow(MapWallFollowType follow_type)
//{
//	uint8_t		Temp_Counter = 0;
//	uint16_t	i = 0;
//
//	int			ret;
//	int16_t		Left_Wall_Buffer[3] = {0};
//	int32_t		Proportion = 0, Delta = 0, Previous = 0, R = 0;
//
//	volatile int32_t		Wall_Straight_Distance = 100, Left_Wall_Speed = 0, Right_Wall_Speed = 0;
//	static volatile int32_t	Wall_Distance = Wall_High_Limit;
//
//	uint32_t Temp_Rcon_Status;
//
//	pthread_t	escape_thread_id;
//
//	MapEscapeTrappedType escape_state = Map_Escape_Trapped_Trapped;
//
//	escape_thread_running = false;
//	ret = pthread_create(&escape_thread_id, 0, WFM_check_trapped, &escape_state);
//	if (ret != 0) {
//		ROS_WARN("%s %d: failed to create escape thread!", __FUNCTION__, __LINE__);
//		return 2;
//	} else {
//		while (escape_thread_running == false) {
//			usleep(10000);
//		}
//		ROS_INFO("%s %d: escape thread is running!", __FUNCTION__, __LINE__);
//	}
//
//	Wall_Distance = Wall_High_Limit;
//
//	if (Wall_Distance > Wall_High_Limit) {
//		Wall_Distance = Wall_High_Limit;
//	}
//
//	if (Wall_Distance < Wall_Low_Limit) {
//		Wall_Distance = Wall_Low_Limit;
//	}
//
//	Move_Forward(25, 25);
//
//	Wall_Straight_Distance = 300;
//
//	Left_Wall_Speed = 15;
//
//
//	while (ros::ok()) {
//		if (escape_thread_running == false) {
//			ROS_INFO("%s %d: quit due to thread exit", __FUNCTION__, __LINE__);
//			break;
//		}
//
//		if(Is_OBS_Near()) {
//			Left_Wall_Speed = 15;
//		} else {
//			i++;
//			if (i > 10) {
//				i = 0;
//				if (Left_Wall_Speed < 30) {
//					Left_Wall_Speed++;
//				}
//			}
//		}
//		if (Left_Wall_Speed < 15) {
//			Left_Wall_Speed = 15;
//		}
//
//		Move_Forward(Left_Wall_Speed, Left_Wall_Speed);
//
//#ifdef WALL_DYNAMIC
//		wall_dynamic_base(30);
//#endif
//#ifdef OBS_DYNAMIC
//		robotbase_OBS_adjust_count(300);
//#endif
//
//		//WFM_boundary_check();
//
//		Temp_Rcon_Status = Get_Rcon_Status();
//		Reset_Rcon_Status();
//		if(Temp_Rcon_Status & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT | RconL_HomeT | RconR_HomeT)){
//			break;
//		}
//
//		if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value()) || Get_Cliff_Trig()) {
//			ROS_WARN("%s %d: Check: Get_Bumper_Status! Break!", __FUNCTION__, __LINE__);
//			break;
//		}
//		usleep(10000);
//	}
//
//	//cm_head_to_course(Rotate_TopSpeed, Gyro_GetAngle() + 900);
//
//	/* Set escape trapped timer when it is in Map_Wall_Follow_Escape_Trapped mode. */
//	escape_trapped_timer = time(NULL);
//
//	while (ros::ok()) {
//		if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME || escape_thread_running == false) {
//			ROS_INFO("%s %d: quit due to %s", __FUNCTION__, __LINE__, escape_thread_running == false ? "thread exit" : "timeout");
//			break;
//		}
//
//#ifdef OBS_DYNAMIC
//		robotbase_OBS_adjust_count(100);
//#endif
//
//		//WFM_boundary_check();
//		/*------------------------------------------------------Cliff Event-----------------------*/
//		if(Get_Cliff_Trig()){
//			Set_Wheel_Speed(0,0);
//			set_dir_backward();
//			usleep(15000);
//			Cliff_Move_Back();
//			if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right)){
//				Stop_Brifly();
//				Set_Clean_Mode(Clean_Mode_Userinterface);
//				break;
//			}
//			if(Get_Cliff_Trig()){
//				if(Cliff_Escape()){
//					Set_Clean_Mode(Clean_Mode_Userinterface);
//					return 1;
//				}
//			}
//
//			Turn_Right(Turn_Speed-10,750);
//			Stop_Brifly();
//			Move_Forward(15,15);
//			Reset_WallAccelerate();
//			//reset_wheel_step();
//			Wall_Straight_Distance=375;
//		}
//
//		/*------------------------------------------------------Home Station Event------------------------*/
//		//Temp_Rcon_Status = Get_Rcon_Status();
//		Temp_Rcon_Status = Get_Rcon_Status();
//		Reset_Rcon_Status();
//		if (Temp_Rcon_Status){
//			Reset_Rcon_Status();
//			if (Temp_Rcon_Status & RconFrontAll_Home_TLR) {
//				/*
//				if (Is_WorkFinish(Get_Room_Mode())) {
//					Set_Clean_Mode(Clean_Mode_GoHome);
//					ResetHomeRemote();
//					USPRINTF_ZZ("%s %d: Check: Virtual! break\n", __FUNCTION__, __LINE__);
//					break;
//				}
//				*/
//			}
//			if (Temp_Rcon_Status & RconFrontAll_Home_T) {
//				if (Is_MoveWithRemote()){
//					Set_Clean_Mode(Clean_Mode_GoHome);
//					//ResetHomeRemote();
//					//USPRINTF_ZZ("%s %d: Check: Virtual 2! break\n", __FUNCTION__, __LINE__);
//					ROS_INFO("Check: Virtual 2! break");
//					break;
//				}
//				Stop_Brifly();
//				if(Temp_Rcon_Status & RconFR_HomeT){
//					Turn_Right(Turn_Speed,850);
//				} else if(Temp_Rcon_Status & RconFL_HomeT){
//					Turn_Right(Turn_Speed,850);
//				} else if(Temp_Rcon_Status & RconL_HomeT){
//					Turn_Right(Turn_Speed,300);
//				} else if(Temp_Rcon_Status & RconFL2_HomeT){
//					Turn_Right(Turn_Speed,600);
//				} else if(Temp_Rcon_Status & RconFR2_HomeT){
//					Turn_Right(Turn_Speed,950);
//				} else if(Temp_Rcon_Status & RconR_HomeT){
//					Turn_Right(Turn_Speed,1100);
//				}
//				Stop_Brifly();
//				Move_Forward(10, 10);
//				Reset_Rcon_Status();
//				Wall_Straight_Distance = 80;
//				Reset_WallAccelerate();
//			}
//		}
//
//		/*---------------------------------------------------Bumper Event-----------------------*/
//		if (Get_Bumper_Status() & RightBumperTrig) {
//			ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
//			Stop_Brifly();
//
//			WFM_move_back(350);
//
//			Stop_Brifly();
//			if(Is_Bumper_Jamed())
//			{
//				return 2;
//			}
//			Turn_Right(Turn_Speed, 700);
//
//			Stop_Brifly();
//
//			Move_Forward(15, 15);
//			Wall_Straight_Distance = 375;
//		}
//
//		if (Get_Bumper_Status() & LeftBumperTrig) {
//			ROS_WARN("%s %d: left bumper triggered", __FUNCTION__, __LINE__);
//			Set_Wheel_Speed(0, 0);
//			usleep(10000);
//
//			if (robot::instance()->getLeftWall() > (Wall_Low_Limit)) {
//				Wall_Distance = robot::instance()->getLeftWall() / 3;
//			} else {
//				Wall_Distance += 200;
//			}
//
//			if (Wall_Distance < Wall_Low_Limit) {
//				Wall_Distance = Wall_Low_Limit;
//			}
//
//			if (Wall_Distance > Wall_High_Limit) {
//				Wall_Distance=Wall_High_Limit;
//			}
//
//			if (Get_Bumper_Status() & RightBumperTrig) {
//				WFM_move_back(100);
//				if(Is_Bumper_Jamed())
//				{
//					return 2;
//				}
//				ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
//				Stop_Brifly();
//				Turn_Right(Turn_Speed, 600);
//
//				Wall_Straight_Distance = MFW_SETTING[follow_type].right_bumper_val; //150;
//			} else {
//				WFM_move_back(350);
//				if(Is_Bumper_Jamed())
//				{
//					return 2;
//				}
//				ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
//				Stop_Brifly();
//				Turn_Right(Turn_Speed, 150);
//				Wall_Straight_Distance = MFW_SETTING[follow_type].left_bumper_val; //250;
//			}
//
//			Wall_Straight_Distance = 200;
//			ROS_INFO("%s %d: ", __FUNCTION__, __LINE__);
//			Stop_Brifly();
//			Move_Forward(10, 10);
//
//			for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
//				Left_Wall_Buffer[Temp_Counter] = 0;
//			}
//		}
//
//		if (Wall_Distance >= 200) {
//			Left_Wall_Buffer[2] = Left_Wall_Buffer[1];
//			Left_Wall_Buffer[1] = Left_Wall_Buffer[0];
//			Left_Wall_Buffer[0] = robot::instance()->getLeftWall();
//			if (Left_Wall_Buffer[0] < 100) {
//				if ((Left_Wall_Buffer[1] - Left_Wall_Buffer[0]) > (Wall_Distance / 25)) {
//					if ((Left_Wall_Buffer[2] - Left_Wall_Buffer[1]) > (Wall_Distance / 25)) {
//						ROS_INFO("%s %d: set wall distance to 350", __FUNCTION__, __LINE__);
//						Move_Forward(18, 16);
//						usleep(10000);
//						Wall_Straight_Distance = 350;
//					}
//				}
//			}
//		}
//
//		/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
//		if (Get_FrontOBS() < Get_FrontOBST_Value()) {
//
//			Proportion = robot::instance()->getLeftWall();
//
//			Proportion = Proportion * 100 / Wall_Distance;
//
//			Proportion -= 100;
//
//			Delta = Proportion - Previous;
//
//			if (Wall_Distance > 300){//over left
//				Left_Wall_Speed = 25 + Proportion / 12 + Delta / 5;
//				Right_Wall_Speed = 25 - Proportion / 10 - Delta / 5;
//
//				if (Right_Wall_Speed > 33) {
//					Left_Wall_Speed = 9;
//					Right_Wall_Speed = 33;
//				}
//			} else if (Wall_Distance > 150){//over left
//				Left_Wall_Speed = 22 + Proportion / 15 + Delta / 7;
//				Right_Wall_Speed = 22 - Proportion / 12 - Delta / 7;
//
//				if (Right_Wall_Speed > 27) {
//					Left_Wall_Speed = 8;
//					Right_Wall_Speed = 30;
//				}
//			} else{
//				Left_Wall_Speed = 15 + Proportion / 22 + Delta / 10;
//				Right_Wall_Speed = 15 - Proportion / 18 - Delta / 10;
//
//				if (Right_Wall_Speed > 18) {
//					Left_Wall_Speed = 5;
//					Right_Wall_Speed = 18;
//				}
//
//				if (Left_Wall_Speed > 20) {
//					Left_Wall_Speed = 20;
//				}
//				if (Left_Wall_Speed < 4) {
//					Left_Wall_Speed = 4;
//				}
//				if (Right_Wall_Speed < 4) {
//					Right_Wall_Speed = 4;
//				}
//				if((Left_Wall_Speed - Right_Wall_Speed) > 5) {
//					Left_Wall_Speed = Right_Wall_Speed + 5;
//				}
//			}
//			/*slow move if left obs near a wall*/
//			if (Get_LeftOBS() > Get_LeftOBST_Value()){
//				if (Wall_Distance < Wall_High_Limit) {
//					Wall_Distance++;
//				}
//			}
//			if (Is_WallOBS_Near()){
//				Left_Wall_Speed = Left_Wall_Speed / 2;
//				Right_Wall_Speed = Right_Wall_Speed / 2;
//			}
//
//			Previous = Proportion;
//
//			if (Left_Wall_Speed < 0) {
//				Left_Wall_Speed = 0;
//			}
//			if (Left_Wall_Speed > 40) {
//				Left_Wall_Speed = 40;
//			}
//			if (Right_Wall_Speed < 0) {
//				Right_Wall_Speed = 0;
//			}
//
//			//ROS_INFO("%s %d: left wall speed: %d\tright wall speed: %d", __FUNCTION__, __LINE__, Left_Wall_Speed, Right_Wall_Speed);
//			Move_Forward(Left_Wall_Speed, Right_Wall_Speed);
//
//			//If turing around at the same point
//			if (R > 7500) {
//				ROS_WARN("%s %d:Isolated Wall Follow!", __FUNCTION__, __LINE__);
//				ROS_WARN("%s %d: Check: Isolated Wall Follow! break", __FUNCTION__, __LINE__);
//				break;
//			}
//
//		} else {
//			ROS_INFO("%s %d: ", __FUNCTION__, __LINE__);
//			Stop_Brifly();
//			Turn_Right(Turn_Speed, 750);
//			ROS_INFO("%s %d: ", __FUNCTION__, __LINE__);
//			Stop_Brifly();
//			Move_Forward(15, 15);
//
//			Wall_Distance = Wall_High_Limit;
//
//		}
//		usleep(10000);
//	}
//
//	if (escape_thread_running == true) {
//		escape_thread_running = false;
//	}
//
//	pthread_join(escape_thread_id, NULL);
//
//	ret = 0;
//	if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME) {
//		ROS_WARN("%s %d: escape timeout %d(%d, %d), state 2", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME, (int)time(NULL), escape_trapped_timer);
//		ret = 2;
//	} else if (escape_state == Map_Escape_Trapped_Escaped) {
//		ROS_WARN("%s %d: escaped, state 0", __FUNCTION__, __LINE__);
//		ret = 0;;
//	} else {
//		ROS_WARN("%s %d: escaped, state 1", __FUNCTION__, __LINE__);
//		ret = 1;
//	}
//
//	Stop_Brifly();
//	Move_Forward(0, 0);
//	return ret;
//}


/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include <ros/ros.h>

#include "charger.hpp"
#include "debug.h"
#include "event_manager.h"
#include "gyro.h"
#include "map.h"
#include "mathematics.h"
#include "movement.h"
#include "path_planning.h"
#include "robot.hpp"
#include "robotbase.h"

#include "wall_follow_trapped.h"

#ifdef Turn_Speed
#undef Turn_Speed
#endif

#define Turn_Speed	18

#define WFT_MOVE_BACK_20MM	(120)

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

bool escape_thread_running = false;
EscapeTrappedType escape_state = Escape_Trapped_Trapped;

uint32_t escape_trapped_timer;

int16_t	Left_Wall_Buffer[3] = {0};
int32_t Wall_Distance;
int32_t wall_straight_distance_tmp = 0;

int16_t wft_turn_angle = 0;

void WFT_turn_right(uint16_t speed, int16_t angle)
{
	uint8_t		accurate;
	int16_t		target_angle;
	uint16_t	speed_;

	bool eh_status_now, eh_status_last;

	eh_status_now = eh_status_last = false;

	Stop_Brifly();

	target_angle = Gyro_GetAngle() - angle + (Gyro_GetAngle() - angle < 0 ? 3600 : 0);
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);

	accurate = speed > 30 ? 30 : 10;
	while (ros::ok()) {
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}
		if (g_fatal_quit_event == true || g_key_clean_pressed == true) {
			break;
		}

		if (abs(target_angle - Gyro_GetAngle()) < accurate) {
			break;
		}

		Set_Dir_Right();

		speed_ = speed;
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			speed_ = std::min((uint16_t)5, speed);
		} else if (abs(target_angle - Gyro_GetAngle()) < 200) {
			speed_ = std::min((uint16_t)10, speed);
		}
		ROS_DEBUG("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed_);
		Set_Wheel_Speed(speed_, speed_);
	}

	set_dir_forward();
	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle());
}

void WFT_move_back(uint16_t dist)
{
	float		pos_x, pos_y, distance;
	uint16_t	cnt = 0;
	uint32_t	SP = 10;

	ROS_INFO("%s %d: Moving back...", __FUNCTION__, __LINE__);
	set_dir_backward();
	Set_Wheel_Speed(8, 8);
	reset_wheel_step();

	pos_x = robot::instance()->getOdomPositionX();
	pos_y = robot::instance()->getOdomPositionY();

	while (ros::ok()) {
		distance = sqrtf(powf(pos_x - robot::instance()->getOdomPositionX(), 2) + powf(pos_y -
																																													 robot::instance()->getOdomPositionY(), 2));
		if (fabsf(distance) > 0.02f) {
			ROS_INFO("%s %d: moving back distance!", __FUNCTION__, __LINE__);
			break;
		}

		usleep(10000);
		cnt++;
		SP = 8 + cnt / 100;
		SP = (SP > 18) ? 18 : SP;

		Set_Wheel_Speed(SP, SP);
		if (cnt > 3000) {
			if (is_encoder_fail()) {
				set_error_code(Error_Code_Encoder);
			}
			ROS_INFO("%s %d: Counter Wathcher!", __FUNCTION__, __LINE__);
			break;
		}
		if (g_key_clean_pressed == true) {
			ROS_INFO("%s %d: key clean pressed!", __FUNCTION__, __LINE__);
			break;
		}

		if (g_fatal_quit_event == true) {
			ROS_INFO("%s %d: fatal event quit", __FUNCTION__, __LINE__);
			break;
		}
	}
	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: Moving back done!", __FUNCTION__, __LINE__);
}

void *WFT_check_trapped(void *data)
{
	float			pos_x, pos_y;
	int16_t			val;
	uint32_t		left_speed, right_speed;
	static int16_t	current_x = 0, current_y = 0;

	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	map_set_position(pos_x, pos_y);
	map_set_cell(MAP, pos_x, pos_y, CLEANED);

	current_x = map_get_x_cell();
	current_y = map_get_y_cell();
	escape_thread_running = true;

	ROS_INFO("%s %d: escape thread is up!", __FUNCTION__, __LINE__);
	while (escape_thread_running == true) {
		pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		map_set_position(pos_x, pos_y);
		map_set_cell(MAP, pos_x, pos_y, CLEANED);

		if (abs(current_x - map_get_x_cell()) >= 2 || abs(current_y - map_get_y_cell()) >= 2) {
			path_update_cell_history();
			ROS_INFO("%s %d: escape thread checking: pos: (%d, %d) (%d, %d)!", __FUNCTION__, __LINE__, current_x, current_y,
							 map_get_x_cell(), map_get_y_cell());
			val = path_escape_trapped();
			if (val == 1) {
				ROS_INFO("%s %d: escaped, thread is existing!", __FUNCTION__, __LINE__);
				escape_state = Escape_Trapped_Escaped;
				escape_thread_running = false;
			}
			current_x = map_get_x_cell();
			current_y = map_get_y_cell();
		} else {
			usleep(100000);
		}
	}

	escape_thread_running = false;
	return NULL;
}

void WFT_boundary_check()
{
	uint8_t boundary_reach = 0;
	int16_t	j;
	int32_t	x, y;

	for (j = -1; boundary_reach == 0 && j <= 1; j++) {
		x = map_get_relative_x(Gyro_GetAngle(), j * CELL_SIZE, CELL_SIZE_2);
		y = map_get_relative_y(Gyro_GetAngle(), j * CELL_SIZE, CELL_SIZE_2);

		if (map_get_cell(MAP, count_to_cell(x), count_to_cell(y)) == BLOCKED_BOUNDARY) {
			boundary_reach = 1;
			Set_Wheel_Speed(0, 0);
			usleep(10000);

			WFT_move_back(350);
			WFT_turn_right(Turn_Speed, 600);
			Move_Forward(15, 15);
		}
	}
}

/*------------------------------------------------------------------ Wall Follow --------------------------*/
EscapeTrappedType Wall_Follow_Trapped()
{
	int			ret;
	bool		eh_status_now, eh_status_last;
	int32_t		Proportion, Delta, Previous, R, Wall_Straight_Distance, Left_Wall_Speed, Right_Wall_Speed;
	uint16_t	i = 0;
	uint32_t	Temp_Rcon_Status;
	pthread_t	escape_thread_id;

	escape_state = Escape_Trapped_Trapped;

	eh_status_now = eh_status_last = false;

	g_fatal_quit_event = false;
	g_bumper_jam = g_bumper_hitted = false;
	g_cliff_all_triggered = g_cliff_jam = g_cliff_triggered = false;
	g_rcon_triggered = false;
	g_oc_brush_main = g_oc_wheel_left = g_oc_wheel_right = g_oc_suction = false;
	g_key_clean_pressed = false;
	g_remote_home = false;
	g_battery_low = false;

	g_bumper_cnt = 0;
	g_cliff_cnt = 0;
	wft_turn_angle = 0;
	g_oc_brush_left_cnt = g_oc_brush_main_cnt = g_oc_brush_right_cnt = g_oc_wheel_left_cnt = g_oc_wheel_right_cnt = g_oc_suction_cnt = 0;
	g_battery_low_cnt = 0;

	Wall_Straight_Distance = Proportion = Delta =  Previous = R = 0;
	Wall_Straight_Distance = 300;
	Wall_Distance = Wall_High_Limit;

	escape_thread_running = false;
	ret = pthread_create(&escape_thread_id, 0, WFT_check_trapped, &escape_state);
	if (ret != 0) {
		ROS_WARN("%s %d: failed to create escape thread!", __FUNCTION__, __LINE__);
		return Escape_Trapped_Trapped;
	} else {
		while (escape_thread_running == false) {
			usleep(10000);
		}
		ROS_INFO("%s %d: escape thread is running!", __FUNCTION__, __LINE__);
	}

	Move_Forward(25, 25);

	Left_Wall_Speed = 15;
	while (ros::ok()) {
		if (escape_thread_running == false) {
			ROS_INFO("%s %d: quit due to thread exit", __FUNCTION__, __LINE__);
			break;
		}

		if (Is_OBS_Near()) {
			Left_Wall_Speed = 15;
		} else {
			if (i++ > 10) {
				i = 0;
				Left_Wall_Speed += (Left_Wall_Speed < 30 ? 1 : 0);
			}
		}
		Move_Forward(Left_Wall_Speed, Left_Wall_Speed);

#ifdef WALL_DYNAMIC
		wall_dynamic_base(30);
#endif
#ifdef OBS_DYNAMIC
		robotbase_OBS_adjust_count(300);
#endif

		WFT_boundary_check();

		Temp_Rcon_Status = Get_Rcon_Status();
		Reset_Rcon_Status();
		if (Temp_Rcon_Status & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT | RconL_HomeT | RconR_HomeT)) {
			break;
		}

		if (Get_Bumper_Status() || (Get_FrontOBS() > Get_FrontOBST_Value()) || Get_Cliff_Trig()) {
			ROS_WARN("%s %d: Check: Get_Bumper_Status! Break!", __FUNCTION__, __LINE__);
			break;
		}
		usleep(10000);
	}

	WFT_regist_events();

	escape_trapped_timer = time(NULL);
	while (ros::ok()) {

#ifdef OBS_DYNAMIC
		robotbase_OBS_adjust_count(100);
#endif

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event == true || g_key_clean_pressed == true) {
            break;
        }

		if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME || escape_thread_running == false) {
			ROS_INFO("%s %d: quit due to %s", __FUNCTION__, __LINE__, escape_thread_running == false ? "thread exit" : "timeout");
			break;
		}

		WFT_boundary_check();

		if (g_cliff_triggered == true) {
			g_cliff_triggered = false;
			Reset_WallAccelerate();
			Wall_Straight_Distance = 375;
		}

		if (g_bumper_hitted == true) {
			g_bumper_hitted = false;
			Wall_Straight_Distance = wall_straight_distance_tmp;
		}

		if (wft_turn_angle != 0) {
			WFT_turn_right(Turn_Speed, wft_turn_angle);
			wft_turn_angle = 0;
			Move_Forward(10, 10);
			Wall_Straight_Distance = 80;
			Reset_WallAccelerate();
		}

		if (Wall_Distance >= 200) {
			Left_Wall_Buffer[2] = Left_Wall_Buffer[1];
			Left_Wall_Buffer[1] = Left_Wall_Buffer[0];
			Left_Wall_Buffer[0] = robot::instance()->getLeftWall();
			if (Left_Wall_Buffer[0] < 100) {
				if ((Left_Wall_Buffer[1] - Left_Wall_Buffer[0]) > (Wall_Distance / 25)) {
					if ((Left_Wall_Buffer[2] - Left_Wall_Buffer[1]) > (Wall_Distance / 25)) {
						ROS_INFO("%s %d: set wall distance to 350", __FUNCTION__, __LINE__);
						Move_Forward(18, 16);
						usleep(10000);
						Wall_Straight_Distance = 350;
					}
				}
			}
		}

		/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
		if (Get_FrontOBS() < Get_FrontOBST_Value()) {
			Proportion = robot::instance()->getLeftWall() * 100 / Wall_Distance - 100;
			Delta = Proportion - Previous;

			if (Wall_Distance > 300) { //over left
				Left_Wall_Speed = 25 + Proportion / 12 + Delta / 5;
				Right_Wall_Speed = 25 - Proportion / 10 - Delta / 5;

				if (Right_Wall_Speed > 33) {
					Left_Wall_Speed = 9;
					Right_Wall_Speed = 33;
				}
			} else if (Wall_Distance > 150) { //over left
				Left_Wall_Speed = 22 + Proportion / 15 + Delta / 7;
				Right_Wall_Speed = 22 - Proportion / 12 - Delta / 7;

				if (Right_Wall_Speed > 27) {
					Left_Wall_Speed = 8;
					Right_Wall_Speed = 30;
				}
			} else{
				Right_Wall_Speed = 15 - Proportion / 18 - Delta / 10;
				Left_Wall_Speed = Right_Wall_Speed > 18 ? 5: (15 + Proportion / 22 + Delta / 10);

				check_limit(Left_Wall_Speed, 4, 20);
                check_limit(Right_Wall_Speed, 4, 18);

				Left_Wall_Speed = (Left_Wall_Speed - Right_Wall_Speed) > 5 ? Right_Wall_Speed + 5 : Left_Wall_Speed;
			}
			/*slow move if left obs near a wall*/
			if (Get_LeftOBS() > Get_LeftOBST_Value() && Wall_Distance < Wall_High_Limit) {
				Wall_Distance++;
			}
			if (Is_WallOBS_Near()){
				Left_Wall_Speed = Left_Wall_Speed / 2;
				Right_Wall_Speed = Right_Wall_Speed / 2;
			}

			Previous = Proportion;
			check_limit(Left_Wall_Speed, 0, 40);
			Right_Wall_Speed = Right_Wall_Speed < 0 ?  0 : Right_Wall_Speed;

			//ROS_INFO("%s %d: left wall speed: %d\tright wall speed: %d", __FUNCTION__, __LINE__, Left_Wall_Speed, Right_Wall_Speed);
			Move_Forward(Left_Wall_Speed, Right_Wall_Speed);

			if (R > 7500) {		// If turing around at the same point
				ROS_WARN("%s %d: Isolated Wall Follow! break", __FUNCTION__, __LINE__);
				break;
			}
		} else {
			ROS_INFO("%s %d: ", __FUNCTION__, __LINE__);
			Stop_Brifly();
			WFT_turn_right(Turn_Speed, 750);
			Stop_Brifly();
			Move_Forward(15, 15);

			Wall_Distance = Wall_High_Limit;
		}
		usleep(10000);
	}

	if (escape_thread_running == true) {
		escape_thread_running = false;
	}

	pthread_join(escape_thread_id, NULL);

	ROS_WARN("%s %d: escape_state: %d", __FUNCTION__, __LINE__, escape_state);
	if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME) {
		ROS_WARN("%s %d: escape timeout %d(%d, %d), state 2", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME, (int)time(NULL), escape_trapped_timer);
		escape_state = Escape_Trapped_Timeout;
	} else if (escape_state == Escape_Trapped_Escaped) {
		ROS_WARN("%s %d: escaped, state 0", __FUNCTION__, __LINE__);
	} else if (g_fatal_quit_event == true) {
		ROS_WARN("%s %d: fatal quit event", __FUNCTION__, __LINE__);
		escape_state = Escape_Trapped_Fatal;
	} else if (g_key_clean_pressed == true) {
		ROS_WARN("%s %d: key clean pressed", __FUNCTION__, __LINE__);
		escape_state = Escape_Trapped_Key_Clean;
	} else {
		ROS_WARN("%s %d: unknow state", __FUNCTION__, __LINE__);
	}

	Stop_Brifly();
	Move_Forward(0, 0);

	WFT_unregist_events();

	return escape_state;
}

/* Event handler functions. */
void WFT_regist_events()
{
	event_manager_set_current_mode(EVT_MODE_WALL_FOLLOW_TRAPPED);

#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &WFT_handle_ ##name); \
	event_manager_enable_handler(y, enabled);

	/* Bumper */
	event_manager_register_and_enable_x(bumper_all, EVT_BUMPER_ALL, true);
	event_manager_register_and_enable_x(bumper_left, EVT_BUMPER_LEFT, true);
	event_manager_register_and_enable_x(bumper_right, EVT_BUMPER_RIGHT, true);

	/* Cliff */
	event_manager_register_and_enable_x(cliff_all, EVT_CLIFF_ALL, true);
	event_manager_register_and_enable_x(cliff_front_left, EVT_CLIFF_FRONT_LEFT, true);
	event_manager_register_and_enable_x(cliff_front_right, EVT_CLIFF_FRONT_RIGHT, true);
	event_manager_register_and_enable_x(cliff_left_right, EVT_CLIFF_LEFT_RIGHT, true);
	event_manager_register_and_enable_x(cliff_front, EVT_CLIFF_FRONT, true);
	event_manager_register_and_enable_x(cliff_left, EVT_CLIFF_LEFT, true);
	event_manager_register_and_enable_x(cliff_right, EVT_CLIFF_RIGHT, true);

	/* RCON */
	event_manager_register_and_enable_x(rcon, EVT_RCON, true);
	/*
	event_manager_register_and_enable_x(rcon_front_left, EVT_RCON_FRONT_LEFT, true);
	event_manager_register_and_enable_x(rcon_front_left2, EVT_RCON_FRONT_LEFT2, true);
	event_manager_register_and_enable_x(rcon_front_right, EVT_RCON_FRONT_RIGHT, true);
	event_manager_register_and_enable_x(rcon_front_right2, EVT_RCON_FRONT_RIGHT2, true);
	event_manager_register_and_enable_x(rcon_left, EVT_RCON_LEFT, true);
	event_manager_register_and_enable_x(rcon_right, EVT_RCON_RIGHT, true);
	*/

	/* Over Current */
	event_manager_register_and_enable_x(over_current_brush_left, EVT_OVER_CURRENT_BRUSH_LEFT, true);
	event_manager_register_and_enable_x(over_current_brush_main, EVT_OVER_CURRENT_BRUSH_MAIN, true);
	event_manager_register_and_enable_x(over_current_brush_right, EVT_OVER_CURRENT_BRUSH_RIGHT, true);
	event_manager_register_and_enable_x(over_current_wheel_left, EVT_OVER_CURRENT_WHEEL_LEFT, true);
	event_manager_register_and_enable_x(over_current_wheel_right, EVT_OVER_CURRENT_WHEEL_RIGHT, true);
	event_manager_register_and_enable_x(over_current_suction, EVT_OVER_CURRENT_SUCTION, true);

	/* Key */
	event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);

	/* Remote */
	event_manager_register_and_enable_x(remote_clean, EVT_REMOTE_CLEAN, true);
	event_manager_register_and_enable_x(remote_suction, EVT_REMOTE_SUCTION, true);

	/* Battery */
	event_manager_register_handler(EVT_BATTERY_LOW, &WFT_handle_battery_low);

#undef event_manager_register_and_enable_x

	event_manager_set_enable(true);
}

void WFT_unregist_events()
{
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	/* Bumper */
	event_manager_register_and_disable_x(EVT_BUMPER_ALL);
	event_manager_register_and_disable_x(EVT_BUMPER_LEFT);
	event_manager_register_and_disable_x(EVT_BUMPER_RIGHT);

	/* Cliff */
	event_manager_register_and_disable_x(EVT_CLIFF_ALL);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_RIGHT);

	/* RCON */
	event_manager_register_and_disable_x(EVT_RCON);
/*
	event_manager_register_and_disable_x(EVT_RCON_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_LEFT2);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_RIGHT2);
	event_manager_register_and_disable_x(EVT_RCON_LEFT);
	event_manager_register_and_disable_x(EVT_RCON_RIGHT);
*/

	/* Key */
	event_manager_register_and_disable_x(EVT_KEY_CLEAN);

	/* Remote */
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_register_and_disable_x(EVT_REMOTE_SUCTION);


	/* Battery */
	event_manager_register_and_disable_x(EVT_BATTERY_LOW);

#undef event_manager_register_and_disable_x

	event_manager_set_current_mode(EVT_MODE_USER_INTERFACE);

	event_manager_set_enable(false);
}

void WFT_handle_bumper_all(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	Set_Wheel_Speed(0, 0);

    g_bumper_hitted = true;
    ROS_INFO("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, Get_Bumper_Status(), state_now ? "true" : "false", state_last ? "true" : "false");
    if (state_now == true && state_last == true) {
        g_bumper_cnt++;

        if (g_bumper_cnt > 2) {
            ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, g_bumper_cnt);
            g_bumper_jam = g_fatal_quit_event = true;
        }
    } else {
        g_bumper_cnt = 0;
    }

	WFT_move_back(WFT_MOVE_BACK_20MM);
	Set_Wheel_Speed(0, 0);
	wft_turn_angle = 600;
	wall_straight_distance_tmp = 150;

	Wall_Distance = (robot::instance()->getLeftWall() > Wall_Low_Limit) ? robot::instance()->getLeftWall() / 3 : Wall_Distance + 200;

	check_limit(Wall_Distance, Wall_Low_Limit, Wall_High_Limit)

	for (int i = 0; i < 3; i++) {
		Left_Wall_Buffer[i] = 0;
	}

	ROS_INFO("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
	if(Get_Bumper_Status() == 0) g_bumper_cnt = 0;
}

void WFT_handle_bumper_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	Set_Wheel_Speed(0, 0);

    g_bumper_hitted = true;
    ROS_INFO("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, Get_Bumper_Status(), state_now ? "true" : "false", state_last ? "true" : "false");
    if (state_now == true && state_last == true) {
        g_bumper_cnt++;

        if (g_bumper_cnt > 2) {
            ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, g_bumper_cnt);
            g_bumper_jam = g_fatal_quit_event = true;
        }
    } else {
        g_bumper_cnt = 0;
    }

	WFT_move_back(WFT_MOVE_BACK_20MM);
	Set_Wheel_Speed(0, 0);
	wft_turn_angle = 150;
	wall_straight_distance_tmp = 250;

	Wall_Distance = (robot::instance()->getLeftWall() > Wall_Low_Limit) ? robot::instance()->getLeftWall() / 3 : Wall_Distance + 200;

	check_limit(Wall_Distance, Wall_Low_Limit, Wall_High_Limit);

	for (int i = 0; i < 3; i++) {
		Left_Wall_Buffer[i] = 0;
	}

	ROS_INFO("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
	if((Get_Bumper_Status() & LeftBumperTrig) == 0) g_bumper_cnt = 0;
}

void WFT_handle_bumper_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	Set_Wheel_Speed(0, 0);

    g_bumper_hitted = true;
    ROS_INFO("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, Get_Bumper_Status(), state_now ? "true" : "false", state_last ? "true" : "false");
    if (state_now == true && state_last == true) {
        g_bumper_cnt++;

        if (g_bumper_cnt > 2) {
            ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, g_bumper_cnt);
            g_bumper_jam = g_fatal_quit_event = true;
        }
    } else {
        g_bumper_cnt = 0;
    }

	WFT_move_back(WFT_MOVE_BACK_20MM);
	Set_Wheel_Speed(0, 0);
	wft_turn_angle = 700;
	wall_straight_distance_tmp = 375;

	ROS_INFO("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
	if((Get_Bumper_Status() & RightBumperTrig) == 0) g_bumper_cnt = 0;
}

/* Cliff */
void WFT_handle_cliff_all(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	g_cliff_all_triggered = true;
	g_fatal_quit_event = true;
}

void WFT_handle_cliff_front_left(bool state_now, bool state_last)
{
    ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	g_cliff_triggered = true;
	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	WFT_move_back(WFT_MOVE_BACK_20MM);
	wft_turn_angle = 600;
}

void WFT_handle_cliff_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	g_cliff_triggered = true;
	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	WFT_move_back(WFT_MOVE_BACK_20MM);
	wft_turn_angle = 1350;
}

void WFT_handle_cliff_left_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	g_cliff_triggered = true;
	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	wft_turn_angle = 900;
}

void WFT_handle_cliff_front(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	g_cliff_triggered = true;
	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	WFT_move_back(WFT_MOVE_BACK_20MM);
	wft_turn_angle = 900;
}

void WFT_handle_cliff_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	g_cliff_triggered = true;
	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	WFT_move_back(WFT_MOVE_BACK_20MM);
	wft_turn_angle = 300;
}

void WFT_handle_cliff_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	g_cliff_triggered = true;
	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	WFT_move_back(WFT_MOVE_BACK_20MM);
	wft_turn_angle = 1350;
}

/* RCON */
#define wft_hanle_rcons(x)		\
    Reset_Rcon_Status();		\
	Set_Wheel_Speed(0, 0);		\
	g_rcon_triggered = true;		\
	wft_turn_angle = x;

void WFT_handle_rcon(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (Get_Rcon_Status() & RconL_HomeT) {
		wft_hanle_rcons(300);
	}
	else if (Get_Rcon_Status() & RconFL2_HomeT) {
		wft_hanle_rcons(600);
	}
	else if (Get_Rcon_Status() & RconFL_HomeT) {
		wft_hanle_rcons(850);
	}
	else if (Get_Rcon_Status() & RconFR_HomeT) {
		wft_hanle_rcons(850);
	}
	else if (Get_Rcon_Status() & RconFR2_HomeT) {
		wft_hanle_rcons(950);
	}
	else if (Get_Rcon_Status() & RconR_HomeT) {
		wft_hanle_rcons(1100);
	}
}
/*
void WFT_handle_rcon_front_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	wft_hanle_rcons(850);
}

void WFT_handle_rcon_front_left2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	wft_hanle_rcons(600);
}

void WFT_handle_rcon_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	wft_hanle_rcons(850);
}

void WFT_handle_rcon_front_right2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	wft_hanle_rcons(950);
}

void WFT_handle_rcon_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	wft_hanle_rcons(300);
}

void WFT_handle_rcon_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	wft_hanle_rcons(1100);
}
*/
#undef wft_hanle_rcons

/* Over Current */
void WFT_handle_over_current_brush_left(bool state_now, bool state_last)
{
	static uint8_t stop_cnt = 0;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	if (!robot::instance()->getLbrushOc()) {
		g_oc_brush_left_cnt = 0;
		if (stop_cnt++ > 250) {
			Set_LeftBrush_PWM(30);
		}
		return;
	}

	stop_cnt = 0;
	if (g_oc_brush_left_cnt++ > 40) {
		g_oc_brush_left_cnt = 0;
		Set_LeftBrush_PWM(0);
		ROS_WARN("%s %d: left brush over current", __FUNCTION__, __LINE__);
	}
}

void WFT_handle_over_current_brush_main(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getMbrushOc()){
		g_oc_brush_main_cnt = 0;
		return;
	}

	if (g_oc_brush_main_cnt++ > 40) {
		g_oc_brush_main_cnt = 0;
		ROS_WARN("%s %d: main brush over current", __FUNCTION__, __LINE__);

		if (Self_Check(Check_Main_Brush) == 1) {
			g_oc_brush_main = true;
			g_fatal_quit_event = true;
		}
	}
}

void WFT_handle_over_current_brush_right(bool state_now, bool state_last)
{
	static uint8_t stop_cnt = 0;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getRbrushOc()) {
		g_oc_brush_right_cnt = 0;
		if (stop_cnt++ > 250) {
			Set_RightBrush_PWM(30);
		}
		return;
	}

	stop_cnt = 0;
	if (g_oc_brush_right_cnt++ > 40) {
		g_oc_brush_right_cnt = 0;
		Set_RightBrush_PWM(0);
		ROS_WARN("%s %d: reft brush over current", __FUNCTION__, __LINE__);
	}
}

void WFT_handle_over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getLwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, %lu mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getLwheelCurrent());

		if (Self_Check(Check_Left_Wheel) == 1) {
			g_oc_wheel_left = true;
			g_fatal_quit_event = true;
		}
	}
}

void WFT_handle_over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getRwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, %lu mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getRwheelCurrent());

		if (Self_Check(Check_Right_Wheel) == 1) {
			g_oc_wheel_right = true;
			g_fatal_quit_event = true;
		}
	}
}

void WFT_handle_over_current_suction(bool state_now, bool state_last)
{
    ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

    if (!robot::instance()->getVacuumOc()) {
        g_oc_suction_cnt = 0;
        return;
    }

    if (g_oc_suction_cnt++ > 40) {
        g_oc_suction_cnt = 0;
        ROS_WARN("%s %d: vacuum over current", __FUNCTION__, __LINE__);

        if (Self_Check(Check_Vacuum) == 1) {
            g_oc_suction = false;
            g_fatal_quit_event = true;
        }
    }
}

/* Key */
void WFT_handle_key_clean(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (Get_Touch_Status()) {
		Set_Wheel_Speed(0, 0);

		g_key_clean_pressed = true;
		Reset_Touch();
	}
}

/* Remote */
void WFT_handle_remote_clean(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	g_key_clean_pressed = true;
	Reset_Rcon_Remote();
}

void WFT_handle_remote_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	Switch_VacMode(true);
	Reset_Rcon_Remote();
}

/* Battery */
void WFT_handle_battery_low(bool state_now, bool state_last)
{
	uint8_t		s_pwr;
	uint16_t	t_vol;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_battery_low_cnt++ > 50) {
		ROS_WARN("%s %d: low battery, battery < %dmv is detected.", __FUNCTION__, __LINE__,
						 robot::instance()->getBatteryVoltage());
		t_vol = GetBatteryVoltage();
		s_pwr = Clean_SideBrush_Power / t_vol;

		g_battery_low_cnt = 0;
		Set_BLDC_Speed(Clean_Vac_Power / t_vol);
		Set_SideBrush_PWM(s_pwr, s_pwr);
		Set_MainBrush_PWM(Clean_MainBrush_Power / t_vol);

		g_fatal_quit_event = true;
		g_battery_low = true;
	}
}
void WFM_move_back(uint16_t dist)
{
	float pos_x, pos_y, distance;
	uint16_t Counter_Watcher = 0;
	uint16_t Temp_Speed = 10;

	ROS_INFO("%s %d: ", __FUNCTION__, __LINE__);
	Stop_Brifly();
	set_dir_backward();
	Set_Wheel_Speed(5, 5);
	Counter_Watcher = 0;

	pos_x = robot::instance()->getOdomPositionX();
	pos_y = robot::instance()->getOdomPositionY();
	while (ros::ok()) {
		distance = sqrtf(powf(pos_x - robot::instance()->getOdomPositionX(), 2) + powf(pos_y -
																																													 robot::instance()->getOdomPositionY(), 2));
		if (fabsf(distance) > 0.02f) {
			break;
		}

		Temp_Speed = get_left_wheel_step() / 3 + 8;
		if (Temp_Speed > 12) {
			Temp_Speed = 12;
		}
		Set_Wheel_Speed(Temp_Speed, Temp_Speed);

		usleep(10000);
		Counter_Watcher++;
		if (Counter_Watcher > 3000) {
			if(is_encoder_fail()) {
				set_error_code(Error_Code_Encoder);
			}
			return;
		}
		if (Stop_Event()) {
			return;
		}
		uint8_t octype = Check_Motor_Current();
		if ((octype == Check_Left_Wheel) || ( octype == Check_Right_Wheel)) {
			ROS_INFO("%s,%d,motor over current",__FUNCTION__,__LINE__);
			return;
		}
	}
	set_dir_forward();
	Set_Wheel_Speed(0, 0);
}
