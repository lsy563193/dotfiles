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
#include "robotbase.h"

#include "motion_manage.h"
//Turn speed
#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed  18
#endif

std::vector<Pose32_t> g_wf_point;
Pose32_t g_new_wf_point;
// This list is for storing the position that robot sees the charger stub.
extern std::list<Point32_t> g_home_point;
extern uint8_t g_from_station;
extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;
//Timer
uint32_t g_wall_follow_timer;
uint32_t g_bumper_interval_timer;
bool g_reach_continuous_state;
int32_t g_reach_count = 0;
const static int32_t REACH_COUNT_LIMIT = 10;//10 represent the wall follow will end after overlap 10 cells
int32_t g_same_cell_count = 0;
//MFW setting
static const MapWallFollowSetting MFW_SETTING[6] = {{1200, 250, 150},
																										{1200, 250, 150},
																										{1200, 250, 150},
																										{1200, 250, 70},
																										{1200, 250, 150},
																										{1200, 250, 150},};

bool wf_check_isolate(void)
{
	float pos_x, pos_y;
	int16_t val;
	uint32_t left_speed, right_speed;
	static int16_t current_x = 0, current_y = 0;

	EscapeTrappedType escaped = Escape_Trapped_Escaped;

	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_set_position(pos_x, pos_y);
	//Map_set_cell(MAP, pos_x, pos_y, CLEANED);

	current_x = Map_get_x_cell();
	current_y = Map_get_y_cell();

	//ROS_INFO("%s %d: escape thread is up!\n", __FUNCTION__, __LINE__);
	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_set_position(pos_x, pos_y);
	//Map_set_cell(MAP, pos_x, pos_y, CLEANED);


	path_set_current_pos();
	//ROS_INFO("%s %d: escape thread checking: pos: (%d, %d) (%d, %d)!\n", __FUNCTION__, __LINE__, current_x, current_y, Map_get_x_cell(), Map_get_y_cell());
	val = WF_path_escape_trapped();
	if (val == 0)
	{
		return 0;//not isolated
	} else
	{
		return 1;//isolated
	}
	current_x = Map_get_x_cell();
	current_y = Map_get_y_cell();
}


/*------------------------------------------------------------------ Wall Follow Mode--------------------------*/
uint8_t wall_follow(MapWallFollowType follow_type)
{

	uint8_t temp_counter = 0, jam = 0;
	uint16_t i = 0;
	int32_t wheel_speed_base = 0;
	int ret;
	int16_t left_wall_buffer[3] = {0};
	int32_t proportion = 0, delta = 0, previous = 0, r = 0;

	volatile int32_t wall_straight_distance = 100, l_speed = 0, r_speed = 0;
	static volatile int32_t Wall_Distance = Wall_High_Limit;
	float Start_WF_Pose_X, Start_WF_Pose_Y;//the first pose when the wall mode start
	float FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
	uint8_t First_Time_Flag;
	uint8_t Isolated_Flag;
	uint32_t Temp_Rcon_Status;
	int16_t Isolated_Count = 0;
	uint8_t octype;//for current check

	Wall_Distance = Wall_High_Limit;
	if (Wall_Distance > Wall_High_Limit)
	{
		Wall_Distance = Wall_High_Limit;
	}
	if (Wall_Distance < Wall_Low_Limit)
	{
		Wall_Distance = Wall_Low_Limit;
	}
	wall_straight_distance = 300;
	l_speed = 15;

	MotionManage motion;


	if (!motion.initSucceeded())
	{
		Set_Clean_Mode(Clean_Mode_Userinterface);
		Reset_Stop_Event_Status();
		return 0;
	}

	ROS_INFO("%s %d: Start wall follow now.", __FUNCTION__, __LINE__);
	g_wall_follow_timer = time(NULL);
	g_bumper_interval_timer = time(NULL);
	Move_Forward(25, 25);

	while (ros::ok())
	{
		/*move to straight to find the wall*/
		while (ros::ok())
		{
			//debug_WF_map(MAP, 0, 0);
			//debug_sm_map(SPMAP, 0, 0);

			Start_WF_Pose_X = robot::instance()->getPositionX();
			Start_WF_Pose_Y = robot::instance()->getPositionY();

			if (Is_OBS_Near())
			{
				l_speed = 15;
			} else
			{
				i++;
				if (i > 10)
				{
					i = 0;
					if (l_speed < 30)
					{
						l_speed++;
					}
				}
			}
			if (l_speed < 15)
			{
				l_speed = 15;
			}

			Move_Forward(l_speed, l_speed);

#ifdef WALL_DYNAMIC
			Wall_Dynamic_Base(30);
#endif
#ifdef OBS_DYNAMIC
			robotbase_OBS_adjust_count(300);
#endif

			//WFM_boundary_check();

			wf_update_position();
			Temp_Rcon_Status = Get_Rcon_Status();
			Reset_Rcon_Status();
			//ROS_INFO("Temp_Rcon_Status = %d", Temp_Rcon_Status);
			if (Temp_Rcon_Status & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT | RconL_HomeT | RconR_HomeT))
			{
				CM_set_home(Map_get_x_count(), Map_get_y_count());
				break;
			}

			if (Get_Bumper_Status() || (Get_FrontOBS() > Get_FrontOBST_Value()) || Get_Cliff_Trig())
			{
				ROS_WARN("%s %d: Check: Get_Bumper_Status! Break!", __FUNCTION__, __LINE__);
				break;
			}

			/*------------------------------------------------------Stop event-----------------------*/
			if (Stop_Event())
			{
				ROS_WARN("%s %d: Touch", __FUNCTION__, __LINE__);
				Reset_Stop_Event_Status();
				wf_break_wall_follow();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;
			}
			if (Get_Rcon_Remote() > 0)
			{
				ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
				if (Get_Rcon_Remote() & (Remote_Clean | Remote_Home | Remote_Max))
				{
					if (Remote_Key(Remote_Home))
					{
						Reset_Rcon_Remote();
						Set_MoveWithRemote();
						wf_end_wall_follow();
						return 0;
					}
					if (Remote_Key(Remote_Clean))
					{
						Reset_Rcon_Remote();
						Set_MoveWithRemote();
						wf_break_wall_follow();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					}
					if (Remote_Key(Remote_Max))
					{
						Reset_Rcon_Remote();
						Switch_VacMode(true);
					}
				} else
				{
					Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
					Reset_Rcon_Remote();
				}
			}

			/* check plan setting*/
			if (Get_Plan_Status() == 1)
			{
				Set_Plan_Status(0);
				Beep(Beep_Error_Sounds, 2, 0, 1);
			}

			/*------------------------------------------------------Check Current--------------------------------*/
			octype = Check_Motor_Current();
			if (octype)
			{
				ROS_WARN("%s %d: motor over current ", __FUNCTION__, __LINE__);
				if (Self_Check(octype))
				{
					wf_break_wall_follow();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return 0;
				}
			}

			/*------------------------------------------------------Distance Check-----------------------*/
			auto Distance_From_WF_Start = (sqrtf( powf(Start_WF_Pose_X - robot::instance()->getPositionX(), 2) + powf(Start_WF_Pose_Y -
																																									robot::instance()->getPositionY(),
																																									2)));
			if (Distance_From_WF_Start > FIND_WALL_DISTANCE)
			{
				ROS_INFO("Find wall over the limited distance : %f", FIND_WALL_DISTANCE);
				wf_end_wall_follow();
				return 0;
			}
		}

		//CM_head_to_course(Rotate_TopSpeed, Gyro_GetAngle() + 900);

		/* Set escape trapped timer when it is in Map_Wall_Follow_Escape_Trapped mode. */
		auto Start_Pose_X = robot::instance()->getPositionX();
		auto Start_Pose_Y = robot::instance()->getPositionY();
		First_Time_Flag = 1;
		while (ros::ok())
		{
			if ((time(NULL) - g_wall_follow_timer) > WALL_FOLLOW_TIME)
			{
				ROS_INFO("Wall Follow time longer than 60 minutes");
				ROS_INFO("time now : %d", (int(time(NULL)) - g_wall_follow_timer));
				wf_end_wall_follow();
				return 1;
			}

#if 0
			/*-------------------------------------------------Start Pose Check------------------------------*/
			if (First_Time_Flag == 0){
				if ((Distance_From_Start = (sqrtf(powf(Start_Pose_X - robot::instance()->robot_get_position_x(), 2) + powf(Start_Pose_Y - robot::instance()->robot_get_position_y(), 2)))) < 0.303 ){
					/*
					 CM_move_to_cell(0, 0, 2, 0, 1);
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
			//wf_update_position();
			wf_check_loop_closed(Gyro_GetAngle());
			if (g_reach_count >= REACH_COUNT_LIMIT)
			{
				if (wf_check_angle())
				{//wf_check_angle succeed,it proves that the robot is not in the narrow space
					g_reach_count = 0;
					Stop_Brifly();
					if (wf_check_isolate())
					{
						ROS_WARN("Isolated");
						Isolated_Flag = 1;
						break;
					} else
					{
						ROS_WARN("Not Isolated");
						Isolated_Flag = 0;
					}
					wf_end_wall_follow();
					ROS_WARN("g_reach_count >= %d", REACH_COUNT_LIMIT);
					break;
				} else
				{
					g_reach_count = 0;//reset reach_cout because wf_check_angle fail, it proves that the robot is in the narrow space
				}
			}
			//Check if the robot is trapped
			if (g_same_cell_count >= 1000)
			{
				ROS_WARN("Maybe the robot is trapped! Checking!");
				g_same_cell_count = 0;
				Stop_Brifly();
				if (wf_check_isolate())
				{
					ROS_WARN("Not trapped!");
				} else
				{
					ROS_WARN("Trapped!");
					wf_break_wall_follow();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return 0;
				}
			}

			//debug_map(MAP, 0, 0);
			//debug_sm_map(SPMAP, 0, 0);

#ifdef OBS_DYNAMIC
			robotbase_OBS_adjust_count(100);
#endif

			//ROS_INFO("%s %d: wall_following", __FUNCTION__, __LINE__);
			//WFM_boundary_check();
			/*------------------------------------------------------Check Current--------------------------------*/
			octype = Check_Motor_Current();
			if (octype)
			{
				ROS_WARN("%s %d: motor over current ", __FUNCTION__, __LINE__);
				if (Self_Check(octype))
				{
					wf_break_wall_follow();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return 0;
				}
			}

			/*------------------------------------------------------Stop event-----------------------*/
			if (Stop_Event())
			{
				ROS_WARN("%s %d: Touch", __FUNCTION__, __LINE__);
				Reset_Stop_Event_Status();
				wf_break_wall_follow();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;
			}
			if (Get_Rcon_Remote() > 0)
			{
				ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
				if (Get_Rcon_Remote() & (Remote_Clean | Remote_Home | Remote_Max))
				{
					if (Remote_Key(Remote_Home))
					{
						Reset_Rcon_Remote();
						Set_MoveWithRemote();
						wf_end_wall_follow();
						return 0;
					}
					if (Remote_Key(Remote_Clean))
					{
						Reset_Rcon_Remote();
						Set_MoveWithRemote();
						wf_break_wall_follow();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return 0;
					}
					if (Remote_Key(Remote_Max))
					{
						Reset_Rcon_Remote();
						Switch_VacMode(true);
					}
				} else
				{
					Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
					Reset_Rcon_Remote();
				}
			}
			/*------------------------------------------------------Check Battery-----------------------*/
			if (Check_Bat_Home() == 1)
			{
				ROS_WARN("%s %d: low battery, battery < 13.2v is detected, go home.", __FUNCTION__, __LINE__);
				Stop_Brifly();
				Set_LED(100, 100);//it indicate that the robot is in low battery state
				wav_play(WAV_BATTERY_LOW);
				wf_end_wall_follow();
				return 0;
			}
			if (Check_Bat_SetMotors(Home_Vac_Power, Home_SideBrush_Power, Home_MainBrush_Power))
			{
				ROS_WARN("%s %d: low battery, battery < 1200 is detected.", __FUNCTION__, __LINE__);
				Stop_Brifly();
				Set_LED(100, 100);//it indicate that the robot is in low battery state
				wav_play(WAV_BATTERY_LOW);
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 0;

			}
			/* check plan setting*/
			if (Get_Plan_Status())
			{
				Set_Plan_Status(false);
				//	wav_play(WAV_APPOINTMENT_DONE);
				Beep(Beep_Error_Sounds, 2, 0, 1);
			}
			/*------------------------------------------------------Cliff Event-----------------------*/
			if (Get_Cliff_Trig())
			{
				Set_Wheel_Speed(0, 0);
				Set_Dir_Backward();
				usleep(15000);
				Cliff_Move_Back();
				if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
					wf_break_wall_follow();
					ROS_INFO("Get_Cliff_Trig");
					return 0;
				}
				if (Get_Cliff_Trig())
				{
					if (Cliff_Escape())
					{
						Set_Clean_Mode(Clean_Mode_Userinterface);
						wf_break_wall_follow();
						ROS_INFO("Cliff_Escape");
						return 0;
					}
				}

				WF_Turn_Right(Turn_Speed - 10, 750);
				Stop_Brifly();
				Move_Forward(15, 15);
				Reset_WallAccelerate();
				//Reset_Wheel_Step();
				wall_straight_distance = 375;
			}


			/*------------------------------------------------------Home Station Event------------------------*/
			Temp_Rcon_Status = Get_Rcon_Status();
			Reset_Rcon_Status();
			//Temp_Rcon_Status = robot::instance()->getRcon();
			//ROS_INFO("Temp_Rcon_Status = %d", Temp_Rcon_Status);
			if (Temp_Rcon_Status & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT | RconL_HomeT | RconR_HomeT))
			{
				CM_set_home(Map_get_x_count(), Map_get_y_count());
			}
			if (Temp_Rcon_Status)
			{
				Reset_Rcon_Status();
				if (Temp_Rcon_Status & RconFrontAll_Home_TLR)
				{
					/*
					if (Is_WorkFinish(Get_Room_Mode())) {
						Set_Clean_Mode(Clean_Mode_GoHome);
						ResetHomeRemote();
						USPRINTF_ZZ("%s %d: Check: Virtual! break\n", __FUNCTION__, __LINE__);
						break;
					}
					*/
				}
				if (Temp_Rcon_Status & RconFrontAll_Home_T)
				{
					if (Is_MoveWithRemote())
					{
						Set_Clean_Mode(Clean_Mode_GoHome);
						//ResetHomeRemote();
						//USPRINTF_ZZ("%s %d: Check: Virtual 2! break\n", __FUNCTION__, __LINE__);
						ROS_INFO("Check: Virtual 2! break");
						break;
					}
					Stop_Brifly();
					if (Temp_Rcon_Status & RconFR_HomeT)
					{
						WF_Turn_Right(Turn_Speed, 850);
					} else if (Temp_Rcon_Status & RconFL_HomeT)
					{
						WF_Turn_Right(Turn_Speed, 850);
					} else if (Temp_Rcon_Status & RconL_HomeT)
					{
						WF_Turn_Right(Turn_Speed, 300);
					} else if (Temp_Rcon_Status & RconFL2_HomeT)
					{
						WF_Turn_Right(Turn_Speed, 600);
					} else if (Temp_Rcon_Status & RconFR2_HomeT)
					{
						WF_Turn_Right(Turn_Speed, 950);
					} else if (Temp_Rcon_Status & RconR_HomeT)
					{
						WF_Turn_Right(Turn_Speed, 1100);
					}
					Stop_Brifly();
					Move_Forward(10, 10);
					Reset_Rcon_Status();
					wall_straight_distance = 80;
					Reset_WallAccelerate();
				}
			}

			/*---------------------------------------------------Bumper Event-----------------------*/
			if (Get_Bumper_Status() & RightBumperTrig)
			{
				ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
				//Base_Speed = BASE_SPEED;
				//Left_BH_Counter++;
				//Wall_BH_Counter++;

				//STOP_BRIFLY;
				Stop_Brifly();
				//WFM_update();
				wf_check_loop_closed(Gyro_GetAngle());

				//WFM_wall_move_back();
				WFM_move_back(350);

				if (time(NULL) - g_bumper_interval_timer > 15)
				{
					usleep(500000);
					ROS_WARN("wait for adjust the gyro.");
					g_bumper_interval_timer = time(NULL);
				}

				if (Is_Bumper_Jamed())
				{
					Reset_Stop_Event_Status();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					//USPRINTF("%s %d: Check: Bumper 2! break\n", __FUNCTION__, __LINE__);
					wf_break_wall_follow();
					ROS_INFO("%s %d: Check: Bumper 2! break", __FUNCTION__, __LINE__);
					return 0;
				}

				//WFM_update();
				wf_check_loop_closed(Gyro_GetAngle());

				Wall_Distance += 300;
				if (Wall_Distance > Wall_High_Limit)Wall_Distance = Wall_High_Limit;

				//STOP_BRIFLY;
				Stop_Brifly();
				WF_Turn_Right(Turn_Speed - 5, 920);

				//bumperCount++;

				//STOP_BRIFLY;
				Stop_Brifly();

				//WFM_update();
				wf_check_loop_closed(Gyro_GetAngle());

				Move_Forward(10, 10);
				Reset_WallAccelerate();
				wall_straight_distance = 200;

				Reset_Wheel_Step();
			}

			if (Get_Bumper_Status() & LeftBumperTrig)
			{
				ROS_WARN("%s %d: left bumper triggered", __FUNCTION__, __LINE__);
				//Base_Speed = BASE_SPEED;
				//L_B_Counter++;
				Set_Wheel_Speed(0, 0);
				Reset_TempPWM();
				//delay(10);
				usleep(1000);
				//WFM_update();
				wf_check_loop_closed(Gyro_GetAngle());
				if (Get_Bumper_Status() & RightBumperTrig)
				{
					ROS_WARN("%s %d: right bumper triggered", __FUNCTION__, __LINE__);
					//USPRINTF_ZZ("%s %d:Double bumper are trigged!",__func__,__LINE__);
					WFM_move_back(100);

					if (time(NULL) - g_bumper_interval_timer > 15)
					{
						usleep(500000);
						ROS_WARN("wait for adjust the gyro.");
						g_bumper_interval_timer = time(NULL);
					}

					//WFM_update();
					wf_check_loop_closed(Gyro_GetAngle());

					if (Is_Bumper_Jamed())
					{
						Reset_Stop_Event_Status();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						//USPRINTF("%s %d: Check: Bumper 2! break\n", __FUNCTION__, __LINE__);
						wf_break_wall_follow();
						ROS_INFO("%s %d: Check: Bumper 2! break", __FUNCTION__, __LINE__);
						return 0;
					}
					//STOP_BRIFLY;
					Stop_Brifly();
					WF_Turn_Right(Turn_Speed - 5, 850);

					wall_straight_distance = MFW_SETTING[follow_type].right_bumper_val; //150;
					Wall_Distance += 300;
					if (Wall_Distance > Wall_High_Limit)Wall_Distance = Wall_High_Limit;
				} else
				{
					Wall_Distance -= 100;
					if (Wall_Distance < Wall_Low_Limit)Wall_Distance = Wall_Low_Limit;

					//WFM_wall_move_back();
					WFM_move_back(350);

					if (time(NULL) - g_bumper_interval_timer > 15)
					{
						usleep(500000);
						ROS_WARN("wait for adjust the gyro.");
						g_bumper_interval_timer = time(NULL);
					}

					//WFM_update();
					wf_check_loop_closed(Gyro_GetAngle());
					if (Is_Bumper_Jamed())
					{
						Reset_Stop_Event_Status();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						wf_break_wall_follow();
						//USPRINTF("%s %d: Check: Bumper 3! break\n", __FUNCTION__, __LINE__);
						ROS_INFO("%s %d: Check: Bumper 3! break", __FUNCTION__, __LINE__);
						return 0;
					}

					//STOP_BRIFLY;
					Stop_Brifly();
					if (jam < 3)
					{
						if (Wall_Distance < 200)
						{
							if (Get_LeftOBS() > (Get_LeftOBST_Value() - 200))
							{
								Wall_Distance = Wall_High_Limit;
								WF_Turn_Right(Turn_Speed - 5, 300);
							} else
							{
								WF_Turn_Right(Turn_Speed - 5, 200);
							}
						} else
						{
							WF_Turn_Right(Turn_Speed - 5, 300);
						}
					} else
					{
						WF_Turn_Right(Turn_Speed - 5, 200);
					}
					wall_straight_distance = MFW_SETTING[follow_type].left_bumper_val; //250;
				}

				if (Get_WallAccelerate() < 2000)
				{
					jam++;
				} else
				{
					jam = 0;
				}

				Reset_WallAccelerate();
				wall_straight_distance = 200;
				//STOP_BRIFLY;
				Stop_Brifly();
				//WFM_update();
				wf_check_loop_closed(Gyro_GetAngle());

				Move_Forward(10, 10);

				for (temp_counter = 0; temp_counter < 3; temp_counter++)
				{
					left_wall_buffer[temp_counter] = 0;
				}
				Reset_Wheel_Step();
			}
			/*------------------------------------------------------Short Distance Move-----------------------*/
			if (Get_WallAccelerate() < (uint32_t) wall_straight_distance)
			{
				//WFM_update();
				wf_check_loop_closed(Gyro_GetAngle());
				if (Get_LeftWheel_Step() < 500)
				{
					if (Get_WallAccelerate() < 100)
					{
						Move_Forward(10, 10);
					} else
					{
						Move_Forward(15, 15);
					}
				} else
				{
					Move_Forward(23, 23);
				}
				//WFM_update();
				wf_check_loop_closed(Gyro_GetAngle());
			} else
			{
				/*------------------------------------------------------Wheel Speed adjustment-----------------------*/


#ifdef OBS_DYNAMIC
				if (Get_FrontOBS() < Get_FrontOBST_Value())
				{
#else
					if (Get_FrontOBS() < MFW_SETTING[follow_type].front_obs_val){
#endif

					wheel_speed_base = 15 + Get_WallAccelerate() / 150;
					if (wheel_speed_base > 28)wheel_speed_base = 28;

					proportion = robot::instance()->getLeftWall();

					proportion = proportion * 100 / Wall_Distance;

					proportion -= 100;

					delta = proportion - previous;

					if (Wall_Distance > 200)
					{//over left
						l_speed = wheel_speed_base + proportion / 8 + delta / 3; //12
						r_speed = wheel_speed_base - proportion / 9 - delta / 3; //10

						if (wheel_speed_base < 26)
						{
							if (r_speed > wheel_speed_base + 6)
							{
								r_speed = 34;
								l_speed = 4;
							} else if (l_speed > wheel_speed_base + 10)
							{
								r_speed = 5;
								l_speed = 30;
							}
						} else
						{
							if (r_speed > 35)
							{
								r_speed = 35;
								l_speed = 4;
							}
						}
					} else
					{
						l_speed = wheel_speed_base + proportion / 10 + delta / 3;//16
						r_speed = wheel_speed_base - proportion / 10 - delta / 4; //11

						if (wheel_speed_base < 26)
						{
							if (r_speed > wheel_speed_base + 4)
							{
								r_speed = 34;
								l_speed = 4;
							}
						} else
						{
							if (r_speed > 32)
							{
								r_speed = 36;
								l_speed = 4;
							}
						}
					}

					previous = proportion;

					if (l_speed > 39)l_speed = 39;
					if (l_speed < 0)l_speed = 0;
					if (r_speed > 35)r_speed = 35;
					if (r_speed < 5)r_speed = 5;

					Move_Forward(l_speed, r_speed);
					wf_check_loop_closed(Gyro_GetAngle());
				} else
				{
					Stop_Brifly();
					if (Get_WallAccelerate() < 2000)
					{
						jam++;
					}
					WF_Turn_Right(Turn_Speed - 5, 920);
					Stop_Brifly();
					//WFM_update();
					wf_check_loop_closed(Gyro_GetAngle());
					Move_Forward(15, 15);
					Reset_Wheel_Step();

					Wall_Distance = Wall_High_Limit;
				}
			}
			usleep(10000);
		}
		if (Isolated_Flag)
		{
			Isolated_Flag = 0;
			Isolated_Count++;
			if (Isolated_Count > 3)
			{
				ROS_WARN("%s %d: Isolate islands more than 3, break", __FUNCTION__, __LINE__);
				wf_end_wall_follow();
				break;
			}
			//Map_Initialize();
			Map_reset(MAP);
			g_wf_point.clear();
			Turn_Right(Turn_Speed, 900);
			continue;
		} else
		{
			ROS_WARN("%s %d: Not in isolate island, finish, break", __FUNCTION__, __LINE__);
			break;
		}
	}//the biggest loop end

	Stop_Brifly();
	Move_Forward(0, 0);
	return ret;
}

uint8_t wf_end_wall_follow(void)
{
	int16_t i;
	int8_t state;
	Stop_Brifly();
	robot::instance()->setBaselinkFrameType(
					Map_Position_Map_Angle);//inorder to use the slam angle to finsh the shortest path to home;
	CM_update_position();
	wf_mark_home_point();
	CM_go_home();

	/*****************************************Release Memory************************************/
	g_home_point.clear();
	g_wf_point.clear();
	std::vector<Pose32_t>(g_wf_point).swap(g_wf_point);
	debug_map(MAP, 0, 0);
	debug_map(SPMAP, 0, 0);
	Set_Clean_Mode(Clean_Mode_Userinterface);
	return 0;
}

uint8_t wf_break_wall_follow(void)
{
	/*****************************************Release Memory************************************/
	g_home_point.clear();
	g_wf_point.clear();
	std::vector<Pose32_t>(g_wf_point).swap(g_wf_point);
	debug_map(MAP, 0, 0);
	debug_map(SPMAP, 0, 0);
	Set_Clean_Mode(Clean_Mode_Userinterface);
	return 0;
}

void wf_update_position(void)
{
	float pos_x, pos_y;
	int16_t x, y;

	x = Map_get_x_cell();
	y = Map_get_y_cell();

	//Map_move_to(dd * cos(deg2rad(heading, 10)), dd * sin(deg2rad(heading, 10)));
	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_set_position(pos_x, pos_y);
}

/**************************************************************
Function:WF_Check_Loop_Closed
Description:
 *1.push a point
 *2.check last cell if cleaned, cleaned->break, uncleaned->continue
 *3.check if in same cell, same->loop until leave this cell, not same-> marked last cell as cleaned
 ***************************************************************/
void wf_check_loop_closed(uint16_t heading)
{
	float pos_x, pos_y;
	int8_t c, d, e;
	int16_t x, y;
	int32_t i, j;
	int8_t push_state;
	bool reach_state;

	x = Map_get_x_cell();
	y = Map_get_y_cell();

	//Map_move_to(dd * cos(deg2rad(heading, 10)), dd * sin(deg2rad(heading, 10)));
	pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_set_position(pos_x, pos_y);

#if (ROBOT_SIZE == 5 || ROBOT_SIZE == 3)

	if (x != Map_get_x_cell() || y != Map_get_y_cell())
	{
		for (c = 1; c >= -1; --c)
		{
			for (d = 1; d >= -1; --d)
			{
				i = Map_get_relative_x(heading, CELL_SIZE * c, CELL_SIZE * d);
				j = Map_get_relative_y(heading, CELL_SIZE * c, CELL_SIZE * d);
				e = Map_get_cell(MAP, count_to_cell(i), count_to_cell(j));

				if (e == BLOCKED_OBS || e == BLOCKED_BUMPER || e == BLOCKED_BOUNDARY)
				{
					Map_set_cell(MAP, i, j, CLEANED);
				}
			}
		}
	}

	//Map_set_cell(MAP, Map_get_relative_x(heading, -CELL_SIZE, CELL_SIZE), Map_get_relative_y(heading, -CELL_SIZE, CELL_SIZE), CLEANED);
	//Map_set_cell(MAP, Map_get_relative_x(heading, 0, CELL_SIZE), Map_get_relative_y(heading, 0, CELL_SIZE), CLEANED);
	i = Map_get_relative_x(heading, 0, 0);
	j = Map_get_relative_y(heading, 0, 0);
	push_state = wf_push_point(count_to_cell(i), count_to_cell(j), Gyro_GetAngle());//push a cell
	if (push_state == 1)
	{
		reach_state = wf_is_reach_cleaned();//check this cell if reached
		if (reach_state == true)
		{//add g_reach_count
			if (g_reach_count == 0)
			{
				g_reach_continuous_state = true;
			}
			if (g_reach_continuous_state == true)
			{
				g_reach_count++;
				ROS_WARN("g_reach_count = %d", g_reach_count);
			}
		} else
		{
			g_reach_continuous_state = false;
			g_reach_count = 0;
		}
	}


	if (push_state == 1)
	{//mark after letf the same cell
		//Map_set_cell(MAP, i, j, CLEANED);
		int size = (g_wf_point.size() - 2);
		if (size >= 0)
		{
			ROS_INFO("g_wf_point.size() - 2 = %d", size);
			try
			{
				Map_set_cell(MAP, cell_to_count((g_wf_point.at(g_wf_point.size() - 2)).X),
										 cell_to_count((g_wf_point.at(g_wf_point.size() - 2)).Y), CLEANED);
			}
			catch (const std::out_of_range &oor)
			{
				std::cerr << "Out of range error:" << oor.what() << '\n';
			}
		}
	}
	i = Map_get_relative_x(heading, CELL_SIZE_2, 0);
	j = Map_get_relative_y(heading, CELL_SIZE_2, 0);
	if (Map_get_cell(MAP, count_to_cell(i), count_to_cell(j)) != BLOCKED_BOUNDARY)
	{
		Map_set_cell(MAP, i, j, BLOCKED_OBS);
	}

#else

	i = Map_GetRelativeX(heading, 0, 0);
	j = Map_GetRelativeY(heading, 0, 0);
	Map_SetCell(MAP, Map_GetRelativeX(heading, 0, CELL_SIZE), Map_GetRelativeY(heading, 0, CELL_SIZE), CLEANED);

	Map_set_cell(MAP, Map_get_relative_x(heading, CELL_SIZE, 0), Map_get_relative_y(heading, CELL_SIZE, 0), BLOCKED_OBS);
#endif
}

bool wf_is_reach_cleaned(void)
{
	int32_t x, y;
	//x = Map_get_x_cell();
	//y = Map_get_y_cell();

	//CM_count_normalize(Gyro_GetAngle(), 1 * CELL_SIZE_3, CELL_SIZE_3, &x, &y);
	CM_count_normalize(Gyro_GetAngle(), 0, 0, &x, &y);
	try
	{
		if (g_wf_point.empty() == false)
		{
			if ((g_wf_point.size() - 1) >= 0)
			{
				if (Map_get_cell(MAP, (g_wf_point.at(g_wf_point.size() - 1)).X, (g_wf_point.at(g_wf_point.size() - 1)).Y) == CLEANED)
				{//size() - 2 means last two
					ROS_INFO("Reach X = %d, Reach Y = %d", count_to_cell(x), count_to_cell(y));
					//Beep(3, 25, 25, 1);//Beep when it was coincide
					return true;
				} else
				{
					return false;
				}
			} else
			{
				return false;
			}
		} else
		{
			return false;
		}
	}
	catch (const std::out_of_range &oor)
	{
		std::cerr << "Out of range error:" << oor.what() << '\n';
	}
	return false;
}

int8_t wf_push_point(int32_t x, int32_t y, int16_t th)
{
	if (g_wf_point.empty() == false)
	{
		if (g_wf_point.back().X != x || g_wf_point.back().Y != y)
		{
			g_same_cell_count = 0;
			g_new_wf_point.X = x;
			g_new_wf_point.Y = y;
			g_new_wf_point.TH = th;
			g_wf_point.push_back(g_new_wf_point);
			ROS_INFO("g_wf_point.X = %d, g_wf_point.y = %d, size = %d", g_wf_point.back().X, g_wf_point.back().Y,
							 (uint) g_wf_point.size());
			ROS_INFO("g_wf_point.X = %d, g_wf_point.y = %d, g_wf_point.TH = %d, size = %d", g_wf_point.back().X, g_wf_point.back().Y,
							 g_wf_point.back().TH, (uint) g_wf_point.size());
			return 1;
		} else
		{
			g_same_cell_count++;//for checking if the robot is traped
			//ROS_INFO("g_same_cell_count = %d, still in the same cell.", g_same_cell_count);
			return 0;//it means still in the same cell
		}
	} else
	{
		g_same_cell_count = 0;
		g_new_wf_point.X = x;
		g_new_wf_point.Y = y;
		g_new_wf_point.TH = th;
		g_wf_point.push_back(g_new_wf_point);
		ROS_INFO("g_wf_point.X = %d, g_wf_point.y = %d, g_wf_point.TH = %d, size = %d", g_wf_point.back().X, g_wf_point.back().Y,
						 g_wf_point.back().TH, g_wf_point.size());
		//ROS_INFO("g_wf_point.X = %d, g_wf_point.y = %d, size = %d", g_wf_point.back().X, g_wf_point.back().Y, g_wf_point.size());
		return 1;
	}
}

void wf_mark_home_point(void)
{
	//path_planning_initialize(&, &g_home_point.front().Y);
	int32_t x, y;
	int i, j;
	std::list<Point32_t> WF_Home_Point;

	WF_Home_Point = g_home_point;

	while (!WF_Home_Point.empty())
	{
		x = WF_Home_Point.front().X;
		y = WF_Home_Point.front().Y;
		ROS_INFO("%s %d: WF_Home_Point.front().X = %d, WF_Home_Point.front().Y = %d, WF_Home_Point.size() = %d",
						 __FUNCTION__, __LINE__, x, y, (uint) WF_Home_Point.size());
		ROS_INFO("%s %d: g_x_min = %d, g_x_max = %d", __FUNCTION__, __LINE__, g_x_min, g_x_max);
		WF_Home_Point.pop_front();

		for (i = -2; i <= 2; i++)
		{
			for (j = -2; j <= 2; j++)
			{
				Map_set_cell(MAP, cell_to_count(x + i), cell_to_count(y + j), CLEANED);//0, -1
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
bool wf_check_angle(void)
{
	int32_t x, y;
	int16_t th, former_th;
	int16_t th_diff;
	int16_t DIFF_LIMIT = 1500;//1500 means 150 degrees, it is used by angle check.
	int8_t pass_count = 0;
	int8_t sum = REACH_COUNT_LIMIT;
	bool fail_flag = 0;
	try
	{
		for (int i = 1; i <= REACH_COUNT_LIMIT; i++)
		{
			x = (g_wf_point.at(g_wf_point.size() - i)).X;
			y = (g_wf_point.at(g_wf_point.size() - i)).Y;
			th = (g_wf_point.at(g_wf_point.size() - i)).TH;
			fail_flag = 0;

			for (std::vector<Pose32_t>::reverse_iterator r_iter = (g_wf_point.rbegin() + i);
					 r_iter != g_wf_point.rend(); ++r_iter)
			{
				if (r_iter->X == x && r_iter->Y == y)
				{
					former_th = r_iter->TH;
					ROS_INFO("r_iter->X = %d, r_iter->Y = %d, r_iter->TH = %d", r_iter->X, r_iter->Y, r_iter->TH);
					th_diff = (abs(former_th - th));

					if (th_diff > 1800)
					{
						th_diff = 3600 - th_diff;
					}

					if (th_diff <= DIFF_LIMIT)
					{
						pass_count++;
						ROS_WARN("th_diff = %d <= %d, pass angle check!", th_diff, DIFF_LIMIT);
						break;
					} else
					{
						fail_flag = 1;
						ROS_WARN("th_diff = %d > %d, fail angle check!", th_diff, DIFF_LIMIT);
					}
				}
				/*in case of the g_wf_point no second same point, the g_reach_count++ caused by cleanning the block obstacle which
				 cost is 2 or 3, it will be set 1(CLEANED), at this situation the sum should sum--, it will happen when the robot
				 get close to the left wall but there is no wall exist, then the robot can judge it is in the isolate island.*/
				if ((r_iter == (g_wf_point.rend() - 1)) && (fail_flag == 0))
				{
					sum--;
					ROS_WARN("Can't find second same pose in g_wf_point! sum--");
				}
			}
		}

		if (sum < REACH_COUNT_LIMIT)
		{
			ROS_WARN("sum = %d < %d, g_wf_point is not enough! wf_check_angle Failed!", sum, REACH_COUNT_LIMIT);
			return 0;
		}

		if (pass_count < sum)
		{
			//in case of robot is always in the narrow and long space, when this count bigger than a threshold value, it will return 1
			/*if (wf_check_isolate() == 0) {
				ROS_WARN("Loop closed!return 1.");
				return 1;
			}*/
			ROS_WARN("pass_count = %d, less than sum = %d. wf_check_angle Failed!", pass_count, sum);
			return 0;
		} else
		{
			ROS_WARN("pass_count = %d, equal to sum = %d. wf_check_angle Succeed!", pass_count, sum);
			return 1;
		}
	}
	catch (const std::out_of_range &oor)
	{
		std::cerr << "Out of range error:" << oor.what() << '\n';
	}
	return 0;
}
