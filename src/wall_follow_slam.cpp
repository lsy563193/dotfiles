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
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include "robot.hpp"
#include "movement.h"
#include "core_move.h"
#include "gyro.h"
#include "map.h"
#include "mathematics.h"
#include "path_planning.h"
#include "wall_follow_slam.h"
#include "wall_follow_trapped.h"
#include <ros/ros.h>
#include "debug.h"
#include "rounding.h"
#include <vector>
#include "charger.hpp"
#include "wav.h"

#include "motion_manage.h"
//Turn speed
#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed	18
#endif

std::vector<Pose32_t> WF_Point;
Pose32_t New_WF_Point;
// This list is for storing the position that robot sees the charger stub.
extern std::list <Point32_t> g_home_point;
volatile int32_t Map_Wall_Follow_Distance = 0;
extern uint8_t g_remote_go_home;
extern uint8_t g_from_station;
extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;
//Timer
uint32_t wall_follow_timer;
uint32_t bumper_interval_timer;
bool reach_continuous_state;
int32_t reach_count = 0;
int32_t	REACH_COUNT_LIMIT = 10;//10 represent the wall follow will end after overlap 10 cells
int32_t same_cell_count = 0;
//MFW setting
static const MapWallFollowSetting MFW_Setting[6]= {{1200, 250, 150 },
	{1200, 250, 150},
	{1200, 250, 150},
	{1200, 250, 70},
	{1200, 250, 150},
	{1200, 250, 150},};

extern int16_t WheelCount_Left, WheelCount_Right;

bool WF_check_isolate(void)
{
	float		pos_x, pos_y;
	int16_t		val;
	uint32_t	left_speed, right_speed;
	static int16_t	current_x = 0, current_y = 0;

	EscapeTrappedType	escaped = Escape_Trapped_Escaped;

	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_SetPosition(pos_x, pos_y);
	//Map_SetCell(MAP, pos_x, pos_y, CLEANED);

	current_x = Map_GetXPos();
	current_y = Map_GetYPos();

	//ROS_INFO("%s %d: escape thread is up!\n", __FUNCTION__, __LINE__);
	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_SetPosition(pos_x, pos_y);
	//Map_SetCell(MAP, pos_x, pos_y, CLEANED);


	path_set_current_pos();
	//ROS_INFO("%s %d: escape thread checking: pos: (%d, %d) (%d, %d)!\n", __FUNCTION__, __LINE__, current_x, current_y, Map_GetXPos(), Map_GetYPos());
	val = WF_path_escape_trapped();
	if (val == 0) {
		return 0;//not isolated
	} else {
		return 1;//isolated
	}
	current_x = Map_GetXPos();
	current_y = Map_GetYPos();
}



/*------------------------------------------------------------------ Wall Follow Mode--------------------------*/
uint8_t Wall_Follow(MapWallFollowType follow_type)
{

	uint8_t					Temp_Counter = 0, Jam = 0;
	uint16_t				i = 0;
	int32_t					Wheel_Speed_Base = 0;
	int						ret;
	int16_t					Left_Wall_Buffer[3] = {0};
	int32_t					Proportion = 0, Delta = 0, Previous = 0, R = 0;

	volatile int32_t		Wall_Straight_Distance = 100, Left_Wall_Speed = 0, Right_Wall_Speed = 0;
	static volatile int32_t	Wall_Distance = Wall_High_Limit;
	float					Start_WF_Pose_X, Start_WF_Pose_Y;//the first pose when the wall mode start
	float					Start_Pose_X, Start_Pose_Y;//the first pose hit the wall
	float					Distance_From_WF_Start;
	float					Distance_From_Start;
	float					FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
	uint8_t					First_Time_Flag;
	uint8_t					Isolated_Flag;
	uint32_t				Temp_Rcon_Status;
	int16_t					Isolated_Count = 0;
	uint8_t					octype;//for current check

	Wall_Distance = Wall_High_Limit;
	if (Wall_Distance > Wall_High_Limit) {
		Wall_Distance = Wall_High_Limit;
	}
	if (Wall_Distance < Wall_Low_Limit) {
		Wall_Distance = Wall_Low_Limit;
	}
	Wall_Straight_Distance = 300;
	Left_Wall_Speed = 15;

	MotionManage motion;


	if(!motion.initSucceeded()){
		Set_Clean_Mode(Clean_Mode_Userinterface);
		Reset_Stop_Event_Status();
		return 0;
	}

	ROS_INFO("%s %d: Start wall follow now.", __FUNCTION__, __LINE__);
	wall_follow_timer = time(NULL);
	bumper_interval_timer = time(NULL);
	Move_Forward(25, 25);

	while(ros::ok()){
		/*move to straight to find the wall*/
		while (ros::ok()) {
			//debug_WF_map(MAP, 0, 0);
			//debug_sm_map(SPMAP, 0, 0);

			Start_WF_Pose_X = robot::instance()->getPositionX();
			Start_WF_Pose_Y = robot::instance()->getPositionY();

			if(Is_OBS_Near()) {
				Left_Wall_Speed = 15;
			} else {
				i++;
				if (i > 10) {
					i = 0;
					if (Left_Wall_Speed < 30) {
						Left_Wall_Speed++;
					}
				}
			}
			if (Left_Wall_Speed < 15) {
				Left_Wall_Speed = 15;
			}

			Move_Forward(Left_Wall_Speed, Left_Wall_Speed);

#ifdef WALL_DYNAMIC
			Wall_Dynamic_Base(30);
#endif
#ifdef OBS_DYNAMIC
			OBS_Dynamic_Base(300);
#endif

			//WFM_boundary_check();

			WF_update_position();
			Temp_Rcon_Status = Get_Rcon_Status();
			Reset_Rcon_Status();
			//ROS_INFO("Temp_Rcon_Status = %d", Temp_Rcon_Status);
			if(Temp_Rcon_Status & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT | RconL_HomeT | RconR_HomeT)){
				CM_SetHome(Map_GetXCount(), Map_GetYCount());
				break;
			}

			if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value()) || Get_Cliff_Trig()) {
				ROS_WARN("%s %d: Check: Get_Bumper_Status! Break!", __FUNCTION__, __LINE__);
				break;
			}

			/*------------------------------------------------------Stop event-----------------------*/
			if (Stop_Event()) {
				ROS_WARN("%s %d: Touch", __FUNCTION__, __LINE__);
				Reset_Stop_Event_Status();
				WF_Break_Wall_Follow();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;
			}
			if (Get_Rcon_Remote() > 0) {
				ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
				if (Get_Rcon_Remote() & (Remote_Clean | Remote_Home | Remote_Max)) {
					if (Remote_Key(Remote_Home)) {
						Reset_Rcon_Remote();
						Set_MoveWithRemote();
						WF_End_Wall_Follow();
						return 0;
					}
					if (Remote_Key(Remote_Clean)) {
						Reset_Rcon_Remote();
						Set_MoveWithRemote();
						WF_Break_Wall_Follow();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					}
					if (Remote_Key(Remote_Max)) {
						Reset_Rcon_Remote();
						Switch_VacMode(true);
					}
				} else {
					Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
					Reset_Rcon_Remote();
				}
			}

			/* check plan setting*/
			if(Get_Plan_Status() == 1)
			{
				Set_Plan_Status(0);
				Beep(Beep_Error_Sounds, 2, 0, 1);
			}

			/*------------------------------------------------------Check Current--------------------------------*/
			octype = Check_Motor_Current();
			if (octype) {
				ROS_WARN("%s %d: motor over current ", __FUNCTION__, __LINE__);
				if(Self_Check(octype)){
					WF_Break_Wall_Follow();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return 0;
				}
			}

			/*------------------------------------------------------Distance Check-----------------------*/
			if ((Distance_From_WF_Start = (sqrtf(powf(Start_WF_Pose_X - robot::instance()->getPositionX(), 2) + powf(Start_WF_Pose_Y -
																																																											 robot::instance()->getPositionY(), 2)))) > FIND_WALL_DISTANCE ){
				ROS_INFO("Find wall over the limited distance : %f", FIND_WALL_DISTANCE);
				WF_End_Wall_Follow();
				return 0;
			}
		}

		//CM_HeadToCourse(Rotate_TopSpeed, Gyro_GetAngle() + 900);

		/* Set escape trapped timer when it is in Map_Wall_Follow_Escape_Trapped mode. */
		Start_Pose_X = robot::instance()->getPositionX();
		Start_Pose_Y = robot::instance()->getPositionY();
		First_Time_Flag = 1;
		while (ros::ok()) {
			if ((time(NULL) - wall_follow_timer) > WALL_FOLLOW_TIME) {
				ROS_INFO("Wall Follow time longer than 60 minutes");
				ROS_INFO("time now : %d", (int(time(NULL)) - wall_follow_timer));
				WF_End_Wall_Follow();
				return 1;
			}

#if 0
			/*-------------------------------------------------Start Pose Check------------------------------*/
			if (First_Time_Flag == 0){
				if ((Distance_From_Start = (sqrtf(powf(Start_Pose_X - robot::instance()->robot_get_position_x(), 2) + powf(Start_Pose_Y - robot::instance()->robot_get_position_y(), 2)))) < 0.303 ){
					/*
				   CM_MoveToCell(0, 0, 2, 0, 1);
				   ROS_INFO("In Start Pose, finish wall follow.");
					//Beep for the finish signal.
					for (i = 10; i > 0; i--) {
						Beep(i, 6, 0, 1);
						usleep(100000);
					}
					Wall_Follow_Stop_Slam();
					debug_WF_map(MAP, 0, 0);
					debug_sm_map(SPMAP, 0, 0);
					Set_Clean_Mode(Clean_Mode_Userinterface);
					*/
					WF_End_Wall_Follow();
					break;
				}
			} else{
				if ((Distance_From_Start = (sqrtf(powf(Start_Pose_X - robot::instance()->robot_get_position_x(), 2) + powf(Start_Pose_Y - robot::instance()->robot_get_position_y(), 2)))) > 0.303 ){
					ROS_INFO("Out Start Pose");
					First_Time_Flag = 0;
				}
			}

			//ROS_INFO("Distance_From_Start = %f", Distance_From_Start);
			//ROS_INFO("time now : %d", int(time(NULL)));
#endif
			/*------------------------------------WF_Map_Update---------------------------------------------------*/
			//WF_update_position();
			WF_Check_Loop_Closed(Gyro_GetAngle());
			if(reach_count >= REACH_COUNT_LIMIT){
				if (WF_Check_Angle()) {//WF_Check_Angle succeed,it proves that the robot is not in the narrow space
					reach_count = 0;
					Stop_Brifly();
					if (WF_check_isolate()){
						ROS_WARN("Isolated");
						Isolated_Flag = 1;
						break;
					} else{
						ROS_WARN("Not Isolated");
						Isolated_Flag = 0;
					}
					WF_End_Wall_Follow();
					ROS_WARN("reach_count >= %d", REACH_COUNT_LIMIT);
					break;
				} else {
					reach_count = 0;//reset reach_cout because WF_Check_Angle fail, it proves that the robot is in the narrow space
				}
			}
			//Check if the robot is trapped
			if (same_cell_count >= 1000) {
				ROS_WARN("Maybe the robot is trapped! Checking!");
				same_cell_count = 0;
				Stop_Brifly();
				if (WF_check_isolate()){
					ROS_WARN("Not trapped!");
				} else{
					ROS_WARN("Trapped!");
					WF_Break_Wall_Follow();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return 0;
				}
			}

			//debug_map(MAP, 0, 0);
			//debug_sm_map(SPMAP, 0, 0);

#ifdef OBS_DYNAMIC
			OBS_Dynamic_Base(100);
#endif

			//ROS_INFO("%s %d: wall_following", __FUNCTION__, __LINE__);
			//WFM_boundary_check();
			/*------------------------------------------------------Check Current--------------------------------*/
			octype = Check_Motor_Current();
			if (octype) {
				ROS_WARN("%s %d: motor over current ", __FUNCTION__, __LINE__);
				if(Self_Check(octype)){
					WF_Break_Wall_Follow();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return 0;
				}
			}

			/*------------------------------------------------------Stop event-----------------------*/
			if (Stop_Event()) {
				ROS_WARN("%s %d: Touch", __FUNCTION__, __LINE__);
				Reset_Stop_Event_Status();
				WF_Break_Wall_Follow();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;
			}
			if (Get_Rcon_Remote() > 0) {
				ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
				if (Get_Rcon_Remote() & (Remote_Clean | Remote_Home | Remote_Max)) {
					if (Remote_Key(Remote_Home)) {
						Reset_Rcon_Remote();
						Set_MoveWithRemote();
						WF_End_Wall_Follow();
						return 0;
					}
					if (Remote_Key(Remote_Clean)) {
						Reset_Rcon_Remote();
						Set_MoveWithRemote();
						WF_Break_Wall_Follow();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					}
					if (Remote_Key(Remote_Max)) {
						Reset_Rcon_Remote();
						Switch_VacMode(true);
					}
				} else {
					Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
					Reset_Rcon_Remote();
				}
			}
			/*------------------------------------------------------Check Battery-----------------------*/
			if (Check_Bat_Home() == 1) {
				ROS_WARN("%s %d: low battery, battery < 13.2v is detected, go home.", __FUNCTION__, __LINE__);
				Stop_Brifly();
				Set_LED(100, 100);//it indicate that the robot is in low battery state
				wav_play(WAV_BATTERY_LOW);
				WF_End_Wall_Follow();
				return 0;
			}
			if (Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power)) {
				ROS_WARN("%s %d: low battery, battery < 1200 is detected.", __FUNCTION__, __LINE__);
				Stop_Brifly();
				Set_LED(100, 100);//it indicate that the robot is in low battery state
				wav_play(WAV_BATTERY_LOW);
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;

			}
			/* check plan setting*/
			if(Get_Plan_Status())
			{
				Set_Plan_Status(false);
			//	wav_play(WAV_APPOINTMENT_DONE);
				Beep(Beep_Error_Sounds, 2, 0, 1);
			}
			/*------------------------------------------------------Cliff Event-----------------------*/
			if(Get_Cliff_Trig()){
				Set_Wheel_Speed(0,0);
				Set_Dir_Backward();
				usleep(15000);
				Cliff_Move_Back();
				if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right)){
					Set_Clean_Mode(Clean_Mode_Userinterface);
					WF_Break_Wall_Follow();
					ROS_INFO("Get_Cliff_Trig");
					return 0;
				}
				if(Get_Cliff_Trig()){
					if(Cliff_Escape()){
						Set_Clean_Mode(Clean_Mode_Userinterface);
						WF_Break_Wall_Follow();
						ROS_INFO("Cliff_Escape");
						return 0;
					}
				}

				WF_Turn_Right(Turn_Speed-10,750);
				Stop_Brifly();
				Move_Forward(15,15);
				Reset_WallAccelerate();
				//Reset_Wheel_Step();
				Wall_Straight_Distance=375;
			}


			/*------------------------------------------------------Home Station Event------------------------*/
			Temp_Rcon_Status = Get_Rcon_Status();
			Reset_Rcon_Status();
			//Temp_Rcon_Status = robot::instance()->getRcon();
			//ROS_INFO("Temp_Rcon_Status = %d", Temp_Rcon_Status);
			if(Temp_Rcon_Status & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT | RconL_HomeT | RconR_HomeT)){
				CM_SetHome(Map_GetXCount(), Map_GetYCount());
			}
			if (Temp_Rcon_Status){
				Reset_Rcon_Status();
				if (Temp_Rcon_Status & RconFrontAll_Home_TLR) {	
					/*
					if (Is_WorkFinish(Get_Room_Mode())) {
						Set_Clean_Mode(Clean_Mode_GoHome);
						ResetHomeRemote();
						USPRINTF_ZZ("%s %d: Check: Virtual! break\n", __FUNCTION__, __LINE__);
						break;
					}
					*/
				}
				if (Temp_Rcon_Status & RconFrontAll_Home_T) {
					if (Is_MoveWithRemote()){
						Set_Clean_Mode(Clean_Mode_GoHome);
						//ResetHomeRemote();
						//USPRINTF_ZZ("%s %d: Check: Virtual 2! break\n", __FUNCTION__, __LINE__);
						ROS_INFO("Check: Virtual 2! break");
						break;
					}
					Stop_Brifly();
					if(Temp_Rcon_Status & RconFR_HomeT){
						WF_Turn_Right(Turn_Speed,850);
					} else if(Temp_Rcon_Status & RconFL_HomeT){
						WF_Turn_Right(Turn_Speed,850);
					} else if(Temp_Rcon_Status & RconL_HomeT){
						WF_Turn_Right(Turn_Speed,300);
					} else if(Temp_Rcon_Status & RconFL2_HomeT){
						WF_Turn_Right(Turn_Speed,600);
					} else if(Temp_Rcon_Status & RconFR2_HomeT){
						WF_Turn_Right(Turn_Speed,950);
					} else if(Temp_Rcon_Status & RconR_HomeT){
						WF_Turn_Right(Turn_Speed,1100);
					}
					Stop_Brifly();
					Move_Forward(10, 10);
					Reset_Rcon_Status();
					Wall_Straight_Distance = 80;
					Reset_WallAccelerate();
				}
			}

			/*---------------------------------------------------Bumper Event-----------------------*/
			if (Get_Bumper_Status() & RightBumperTrig) {
				ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
				//Base_Speed = BASE_SPEED;
				//Left_BH_Counter++;
				//Wall_BH_Counter++;

				//STOP_BRIFLY;
				Stop_Brifly();
				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle());

				//WFM_wall_move_back();
				WFM_move_back(350);

				if (time(NULL) - bumper_interval_timer > 15)
				{
					usleep(500000);
					ROS_WARN("wait for adjust the gyro.");
					bumper_interval_timer = time(NULL);
				}

				if (Is_Bumper_Jamed()){
					Reset_Stop_Event_Status();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					//USPRINTF("%s %d: Check: Bumper 2! break\n", __FUNCTION__, __LINE__);
					WF_Break_Wall_Follow();
					ROS_INFO("%s %d: Check: Bumper 2! break", __FUNCTION__, __LINE__);
					return 0;
				}

				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle());

				Wall_Distance+=300;
				if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;	

				//STOP_BRIFLY;
				Stop_Brifly();
				WF_Turn_Right(Turn_Speed-5, 920);

				//bumperCount++;

				//STOP_BRIFLY;
				Stop_Brifly();

				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle());

				Move_Forward(10, 10);
				Reset_WallAccelerate();
				Wall_Straight_Distance = 200;

				Reset_Wheel_Step();
			}

			if (Get_Bumper_Status() & LeftBumperTrig) {
				ROS_WARN("%s %d: left bumper triggered", __FUNCTION__, __LINE__);
				//Base_Speed = BASE_SPEED;
				//L_B_Counter++;
				Set_Wheel_Speed(0, 0);
				Reset_TempPWM();
				//delay(10);
				usleep(1000);
				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle());
				if (Get_Bumper_Status() & RightBumperTrig){
					ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
					//USPRINTF_ZZ("%s %d:Double bumper are trigged!",__func__,__LINE__);
					WFM_move_back(100);

					if (time(NULL) - bumper_interval_timer > 15)
					{
						usleep(500000);
						ROS_WARN("wait for adjust the gyro.");
						bumper_interval_timer = time(NULL);
					}

					//WFM_update();
					WF_Check_Loop_Closed(Gyro_GetAngle());

					if (Is_Bumper_Jamed()){
						Reset_Stop_Event_Status();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						//USPRINTF("%s %d: Check: Bumper 2! break\n", __FUNCTION__, __LINE__);
						WF_Break_Wall_Follow();
						ROS_INFO("%s %d: Check: Bumper 2! break", __FUNCTION__, __LINE__);
						return 0;
					}
					//STOP_BRIFLY;
					Stop_Brifly();
					WF_Turn_Right(Turn_Speed-5, 850);

					Wall_Straight_Distance = MFW_Setting[follow_type].right_bumper_val; //150;
					Wall_Distance+=300;
					if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
				} else {
					Wall_Distance-=100;
					if(Wall_Distance<Wall_Low_Limit)Wall_Distance=Wall_Low_Limit;

					//WFM_wall_move_back();
					WFM_move_back(350);

					if (time(NULL) - bumper_interval_timer > 15)
					{
						usleep(500000);
						ROS_WARN("wait for adjust the gyro.");
						bumper_interval_timer = time(NULL);
					}

					//WFM_update();
					WF_Check_Loop_Closed(Gyro_GetAngle());
					if (Is_Bumper_Jamed()) {
						Reset_Stop_Event_Status();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						WF_Break_Wall_Follow();
						//USPRINTF("%s %d: Check: Bumper 3! break\n", __FUNCTION__, __LINE__);
						ROS_INFO("%s %d: Check: Bumper 3! break", __FUNCTION__, __LINE__);
						return 0;
					}

					//STOP_BRIFLY;
					Stop_Brifly();
					if (Jam < 3) {
						if(Wall_Distance<200){
							if(Get_LeftOBS()>(Get_LeftOBST_Value()-200)){
								Wall_Distance=Wall_High_Limit;
								WF_Turn_Right(Turn_Speed-5, 300);
							} else{
								WF_Turn_Right(Turn_Speed-5, 200);
							}
						} else{
							WF_Turn_Right(Turn_Speed-5, 300);
						}
					} else {
						WF_Turn_Right(Turn_Speed-5, 200);
					}
					Wall_Straight_Distance = MFW_Setting[follow_type].left_bumper_val; //250;
				}

				if (Get_WallAccelerate() < 2000){
					Jam++;
				} else {
					Jam = 0;
				}

				Reset_WallAccelerate();
				Wall_Straight_Distance = 200;
				//STOP_BRIFLY;
				Stop_Brifly();
				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle());

				Move_Forward(10, 10);

				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++){
					Left_Wall_Buffer[Temp_Counter] = 0;
				}
				Reset_Wheel_Step();
			}
			/*------------------------------------------------------Short Distance Move-----------------------*/
			if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance) {
				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle());
				if (Get_LeftWheel_Step() < 500) {
					if (Get_WallAccelerate() < 100) {
						Move_Forward(10, 10);
					} else {
						Move_Forward(15, 15);
					}
				} else {
					Move_Forward(23, 23);
				}
				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle());
			} else {
				/*------------------------------------------------------Wheel Speed adjustment-----------------------*/


#ifdef OBS_DYNAMIC
				if (Get_FrontOBS() < Get_FrontOBST_Value()) {
#else
				if (Get_FrontOBS() < MFW_Setting[follow_type].front_obs_val){
#endif

					Wheel_Speed_Base = 15 + Get_WallAccelerate() / 150;
					if(Wheel_Speed_Base>28)Wheel_Speed_Base = 28;

					Proportion = robot::instance()->getLeftWall();

					Proportion = Proportion*100/Wall_Distance;

					Proportion-=100;

					Delta = Proportion - Previous;

					if(Wall_Distance>200){//over left
						Left_Wall_Speed = Wheel_Speed_Base + Proportion/8 + Delta/3; //12
						Right_Wall_Speed = Wheel_Speed_Base - Proportion/9 - Delta/3; //10

						if(Wheel_Speed_Base<26){
							if(Right_Wall_Speed > Wheel_Speed_Base+6){
								Right_Wall_Speed = 34;
								Left_Wall_Speed = 4;
							} else if(Left_Wall_Speed > Wheel_Speed_Base+10){
								Right_Wall_Speed = 5;
								Left_Wall_Speed = 30;
							}
						} else{
							if(Right_Wall_Speed > 35){
								Right_Wall_Speed = 35;
								Left_Wall_Speed = 4;
							}
						}
					} else {
						Left_Wall_Speed = Wheel_Speed_Base + Proportion/10 + Delta/3;//16
						Right_Wall_Speed = Wheel_Speed_Base - Proportion/10 - Delta/4; //11

						if(Wheel_Speed_Base<26){
							if(Right_Wall_Speed > Wheel_Speed_Base+4){
								Right_Wall_Speed = 34;
								Left_Wall_Speed = 4;
							}
						} else{
							if(Right_Wall_Speed > 32){
								Right_Wall_Speed = 36;
								Left_Wall_Speed = 4;
							}
						}
					}

					Previous = Proportion;

					if(Left_Wall_Speed>39)Left_Wall_Speed=39;
					if(Left_Wall_Speed<0)Left_Wall_Speed=0;
					if(Right_Wall_Speed > 35)Right_Wall_Speed = 35;
					if(Right_Wall_Speed < 5)Right_Wall_Speed = 5;

					Move_Forward(Left_Wall_Speed,Right_Wall_Speed);
					WF_Check_Loop_Closed(Gyro_GetAngle());
				} else {
					Stop_Brifly();
					if (Get_WallAccelerate() < 2000){
						Jam++;
					}
					WF_Turn_Right(Turn_Speed - 5, 920);
					Stop_Brifly();
					//WFM_update();
					WF_Check_Loop_Closed(Gyro_GetAngle());
					Move_Forward(15, 15);
					Reset_Wheel_Step();

					Wall_Distance = Wall_High_Limit;
				}
			}
			usleep(10000);
		}
		if(Isolated_Flag){
			Isolated_Flag = 0;
			Isolated_Count++;
			if (Isolated_Count > 3){
				ROS_WARN("%s %d: Isolate islands more than 3, break", __FUNCTION__, __LINE__);
				WF_End_Wall_Follow();
				break;
			}
			//Map_Initialize();
			Map_Reset(MAP);
			WF_Point.clear();
			Turn_Right(Turn_Speed, 900);
			continue;
		} else{
			ROS_WARN("%s %d: Not in isolate island, finish, break", __FUNCTION__, __LINE__);
			break;
		}
	}//the biggest loop end

	Stop_Brifly();
	Move_Forward(0, 0);
	return ret;
}

uint8_t WF_End_Wall_Follow(void){
	int16_t i;
	int8_t state;
	// X, Y in Target_Point are all counts.
	//Point32_t	Next_Point, Target_Point;
	//Cell_t	tmpPnt, g_pnt16_ar_tmp[3];
	//MapTouringType	mt_state = MT_None;
	//int16_t offsetAngle = robot::instance()->getHomeAngle();
	Stop_Brifly();
	robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);//inorder to use the slam angle to finsh the shortest path to home;
	CM_update_position(Gyro_GetAngle());
	WF_Mark_Home_Point();
	CM_go_home();

	/*****************************************Release Memory************************************/
	g_home_point.clear();
	WF_Point.clear();
	std::vector<Pose32_t>(WF_Point).swap(WF_Point);
	debug_map(MAP, 0, 0);
	debug_map(SPMAP, 0, 0);
	Set_Clean_Mode(Clean_Mode_Userinterface);
	return 0;
}

uint8_t WF_Break_Wall_Follow(void){
	/*****************************************Release Memory************************************/
	g_home_point.clear();
	WF_Point.clear();
	std::vector<Pose32_t>(WF_Point).swap(WF_Point);
	debug_map(MAP, 0, 0);
	debug_map(SPMAP, 0, 0);
	Set_Clean_Mode(Clean_Mode_Userinterface);
	return 0;
}
void WF_update_position(void) {
	float	pos_x, pos_y;
	int16_t	x, y;

	x = Map_GetXPos();
	y = Map_GetYPos();

	//Map_MoveTo(dd * cos(deg2rad(heading, 10)), dd * sin(deg2rad(heading, 10)));
	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_SetPosition(pos_x, pos_y);
}

/**************************************************************
Function:WF_Check_Loop_Closed
Description:
 *1.push a point
 *2.check last cell if cleaned, cleaned->break, uncleaned->continue
 *3.check if in same cell, same->loop until leave this cell, not same-> marked last cell as cleaned
 ***************************************************************/
void WF_Check_Loop_Closed(uint16_t heading) {
	float	pos_x, pos_y;
	int8_t	c, d, e;
	int16_t	x, y;
	int32_t	i, j;
	int8_t	push_state;
	bool	reach_state;

	x = Map_GetXPos();
	y = Map_GetYPos();

	//Map_MoveTo(dd * cos(deg2rad(heading, 10)), dd * sin(deg2rad(heading, 10)));
	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_SetPosition(pos_x, pos_y);

#if (ROBOT_SIZE == 5 || ROBOT_SIZE == 3)

	if (x != Map_GetXPos() || y != Map_GetYPos()) {
		for (c = 1; c >= -1; --c) {
			for (d = 1; d >= -1; --d) {
				i = Map_GetRelativeX(heading, CELL_SIZE * c, CELL_SIZE * d);
				j = Map_GetRelativeY(heading, CELL_SIZE * c, CELL_SIZE * d);
				e = Map_GetCell(MAP, countToCell(i), countToCell(j));

				if (e == BLOCKED_OBS || e == BLOCKED_BUMPER || e == BLOCKED_BOUNDARY ) {
					Map_SetCell(MAP, i, j, CLEANED);
				}
			}
		}
	}

	//Map_SetCell(MAP, Map_GetRelativeX(heading, -CELL_SIZE, CELL_SIZE), Map_GetRelativeY(heading, -CELL_SIZE, CELL_SIZE), CLEANED);
	//Map_SetCell(MAP, Map_GetRelativeX(heading, 0, CELL_SIZE), Map_GetRelativeY(heading, 0, CELL_SIZE), CLEANED);
	i = Map_GetRelativeX(heading, 0, 0);
	j = Map_GetRelativeY(heading, 0, 0);
	push_state = WF_Push_Point(countToCell(i),countToCell(j), Gyro_GetAngle());//push a cell
	if(push_state == 1){
		reach_state = WF_Is_Reach_Cleaned();//check this cell if reached
		if(reach_state == true){//add reach_count
			if (reach_count == 0){
				reach_continuous_state = true;
			}
			if(reach_continuous_state == true){
				reach_count++;
				ROS_WARN("reach_count = %d", reach_count);
			}
		} else{
			reach_continuous_state = false;
			reach_count = 0;
		}
	}


	if(push_state == 1){//mark after letf the same cell
		//Map_SetCell(MAP, i, j, CLEANED);
		int size = (WF_Point.size() - 2);
		if(size >= 0){
			ROS_INFO("WF_Point.size() - 2 = %d", size);
			try{
				Map_SetCell(MAP, cellToCount((WF_Point.at(WF_Point.size() - 2)).X), cellToCount((WF_Point.at(WF_Point.size() - 2)).Y), CLEANED);
			}
			catch(const std::out_of_range& oor){
				std::cerr << "Out of range error:" << oor.what() << '\n';
			}
		}
	}
	i = Map_GetRelativeX(heading, CELL_SIZE_2, 0);
	j = Map_GetRelativeY(heading, CELL_SIZE_2, 0);
	if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
		Map_SetCell(MAP, i, j, BLOCKED_OBS);
	}

#else

	i = Map_GetRelativeX(heading, 0, 0);
	j = Map_GetRelativeY(heading, 0, 0);
	Map_SetCell(MAP, Map_GetRelativeX(heading, 0, CELL_SIZE), Map_GetRelativeY(heading, 0, CELL_SIZE), CLEANED);

	Map_SetCell(MAP, Map_GetRelativeX(heading, CELL_SIZE, 0), Map_GetRelativeY(heading, CELL_SIZE, 0), BLOCKED_OBS);
#endif
}

bool WF_Is_Reach_Cleaned(void){
	int32_t x,y;
	//x = Map_GetXPos();
	//y = Map_GetYPos();

	//CM_count_normalize(Gyro_GetAngle(), 1 * CELL_SIZE_3, CELL_SIZE_3, &x, &y);
	CM_count_normalize(Gyro_GetAngle(), 0, 0, &x, &y);
	try{
		if(WF_Point.empty() == false){
			if((WF_Point.size() - 1) >= 0){
				if (Map_GetCell(MAP, (WF_Point.at(WF_Point.size() - 1)).X, (WF_Point.at(WF_Point.size() - 1)).Y) == CLEANED) {//size() - 2 means last two
					ROS_INFO("Reach X = %d, Reach Y = %d", countToCell(x), countToCell(y));
					//Beep(3, 25, 25, 1);//Beep when it was coincide
					return true;
				} else{
					return false;
				}
			} else{
				return false;
			}
		} else{
			return false;
		}
	}
	catch(const std::out_of_range& oor){
		std::cerr << "Out of range error:" << oor.what() << '\n';
	}
	return false;
}

int8_t WF_Push_Point(int32_t x, int32_t y, int16_t th){
	if (WF_Point.empty() == false){
		if(WF_Point.back().X != x || WF_Point.back().Y != y){
			same_cell_count = 0;
			New_WF_Point.X = x;
			New_WF_Point.Y = y;
			New_WF_Point.TH = th;
			WF_Point.push_back(New_WF_Point);
			ROS_INFO("WF_Point.X = %d, WF_Point.y = %d, size = %d", WF_Point.back().X, WF_Point.back().Y, (uint)WF_Point.size());
			ROS_INFO("WF_Point.X = %d, WF_Point.y = %d, WF_Point.TH = %d, size = %d", WF_Point.back().X, WF_Point.back().Y, WF_Point.back().TH, (uint)WF_Point.size());
			return 1;
		} else{
			same_cell_count++;//for checking if the robot is traped
			//ROS_INFO("same_cell_count = %d, still in the same cell.", same_cell_count);
			return 0;//it means still in the same cell
		}
	} else{
		same_cell_count = 0;
		New_WF_Point.X = x;
		New_WF_Point.Y = y;
		New_WF_Point.TH = th;
		WF_Point.push_back(New_WF_Point);
		ROS_INFO("WF_Point.X = %d, WF_Point.y = %d, WF_Point.TH = %d, size = %d", WF_Point.back().X, WF_Point.back().Y, WF_Point.back().TH, WF_Point.size());
		//ROS_INFO("WF_Point.X = %d, WF_Point.y = %d, size = %d", WF_Point.back().X, WF_Point.back().Y, WF_Point.size());
		return 1;
	}
}

void WF_Mark_Home_Point(void){
	//path_planning_initialize(&, &g_home_point.front().Y);
	int32_t x, y;
	int i, j;
	std::list <Point32_t> WF_Home_Point;

	WF_Home_Point = g_home_point;

	while (!WF_Home_Point.empty()){
		x = WF_Home_Point.front().X;
		y = WF_Home_Point.front().Y;
		ROS_INFO("%s %d: WF_Home_Point.front().X = %d, WF_Home_Point.front().Y = %d, WF_Home_Point.size() = %d", __FUNCTION__, __LINE__, x, y, (uint)WF_Home_Point.size());
		ROS_INFO("%s %d: g_x_min = %d, g_x_max = %d", __FUNCTION__, __LINE__, g_x_min, g_x_max);
		WF_Home_Point.pop_front();

		for (i = -2; i <= 2; i++) {
			for (j = -2;j <= 2; j++) {
				Map_SetCell(MAP, cellToCount(x + i), cellToCount(y + j) , CLEANED);//0, -1
				//ROS_INFO("%s %d: x + i = %d, y + j = %d", __FUNCTION__, __LINE__, x + i, y + j);
			}
		}
	}
}

/**************************************************************
Function:WF_Check_Check_Angle
Description:
 *It mainly for checking whether the angle is same when wall
 *follow end. When it check is loop closed, and is not isolated,
 *it will check whether the angle of last 10 poses in WF_Point
 *is same as the other same pose in the WF_Point except the last
 *10 point. It can prevent from the case that the robot is in the
 *narrow and long space when wall follow, and it will be checked
 *as loop closed.
 ***************************************************************/
bool WF_Check_Angle(void) {
	int32_t x, y;
	int16_t th, former_th;
	int16_t	th_diff;
	int16_t	DIFF_LIMIT = 1500;//1500 means 150 degrees, it is used by angle check.
	int8_t	pass_count = 0;
	int8_t	sum = REACH_COUNT_LIMIT;
	bool	fail_flag = 0;
	try{
		for (int i = 1; i <= REACH_COUNT_LIMIT; i++) {
			x = (WF_Point.at(WF_Point.size() - i)).X;
			y = (WF_Point.at(WF_Point.size() - i)).Y;
			th = (WF_Point.at(WF_Point.size() - i)).TH;
			fail_flag = 0;

			for (std::vector<Pose32_t>::reverse_iterator r_iter =  (WF_Point.rbegin() + i); r_iter != WF_Point.rend(); ++r_iter) {
				if (r_iter->X == x && r_iter->Y == y) {
					former_th = r_iter->TH;
					ROS_INFO("r_iter->X = %d, r_iter->Y = %d, r_iter->TH = %d", r_iter->X, r_iter->Y, r_iter->TH);
					th_diff = (abs(former_th - th));

					if (th_diff > 1800) {
						th_diff = 3600 - th_diff;
					}

					if (th_diff <= DIFF_LIMIT) {
						pass_count++;
						ROS_WARN("th_diff = %d <= %d, pass angle check!", th_diff, DIFF_LIMIT);
						break;
					} else {
						fail_flag = 1;
						ROS_WARN("th_diff = %d > %d, fail angle check!", th_diff, DIFF_LIMIT);
					}
				}
				/*in case of the WF_Point no second same point, the reach_count++ caused by cleanning the block obstacle which 
				 cost is 2 or 3, it will be set 1(CLEANED), at this situation the sum should sum--, it will happen when the robot
				 get close to the left wall but there is no wall exist, then the robot can judge it is in the isolate island.*/
				if ((r_iter == (WF_Point.rend() - 1)) && (fail_flag == 0)) {
					sum--;
					ROS_WARN("Can't find second same pose in WF_Point! sum--");
				}
			}
		}

		if (sum < REACH_COUNT_LIMIT) {
			ROS_WARN("sum = %d < %d, WF_Point is not enough! WF_Check_Angle Failed!", sum, REACH_COUNT_LIMIT);
			return 0;
		}

		if (pass_count < sum) {
			//in case of robot is always in the narrow and long space, when this count bigger than a threshold value, it will return 1
			/*if (WF_check_isolate() == 0) {
				ROS_WARN("Loop closed!return 1.");
				return 1;
			}*/
			ROS_WARN("pass_count = %d, less than sum = %d. WF_Check_Angle Failed!", pass_count, sum);
			return 0;
		} else {
			ROS_WARN("pass_count = %d, equal to sum = %d. WF_Check_Angle Succeed!", pass_count, sum);
			return 1;
		}
	}
	catch(const std::out_of_range& oor){
		std::cerr << "Out of range error:" << oor.what() << '\n';
	}
	return 0;
}
