/*
 ******************************************************************************
 * @file    AI Cleaning Robot
 * @author  ILife Team Dxsong
 * @version V1.0
 * @date    17-Nov-2011
 * @brief   Move near the wall on the left in a certain distance
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
#include "wall_follow_multi.h"
#include <ros/ros.h>
#include "debug.h"
#include "rounding.h"
#include <vector>
#include "charger.hpp"
//Turn speed
#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed	18
#endif

std::vector<Point32_t> WF_Point;
Point32_t New_WF_Point;
// This list is for storing the position that robot sees the charger stub.
extern std::list <Point32_t> Home_Point;
// This is for adding new point to Home Point list.
extern Point32_t New_Home_Point;
volatile int32_t Map_Wall_Follow_Distance = 0;
extern uint8_t remote_go_home;
extern uint8_t from_station;

bool	escape_thread_running = false;
//Timer
uint32_t escape_trapped_timer;
bool reach_continuous_state;
int32_t reach_count = 0;

//MFW setting
static const MapWallFollowSetting MFW_Setting[6]= {{1200, 250, 150 },
		{1200, 250, 150},
		{1200, 250, 150},
		{1200, 250, 70},
		{1200, 250, 150},
		{1200, 250, 150},};


extern int16_t WheelCount_Left, WheelCount_Right;


/************************************************************************
 * Normal End
 ************************************************************************/
void WFM_move_back(uint16_t dist)
{
		float pos_x, pos_y, distance;
		uint16_t Counter_Watcher = 0;
		uint16_t Temp_Speed = 10;

		printf("%s %d: \n", __FUNCTION__, __LINE__);
		Stop_Brifly();
		Set_Dir_Backward();
		Set_Wheel_Speed(5, 5);
		Counter_Watcher = 0;

		pos_x = robot::instance()->robot_get_odom_position_x();
		pos_y = robot::instance()->robot_get_odom_position_y();
		while (ros::ok()) {
				distance = sqrtf(powf(pos_x - robot::instance()->robot_get_odom_position_x(), 2) + powf(pos_y - robot::instance()->robot_get_odom_position_y(), 2));
				if (fabsf(distance) > 0.02f) {
						break;
				}

				Temp_Speed = Get_LeftWheel_Step() / 3 +  8;
				if (Temp_Speed > 12) {
						Temp_Speed = 12;
				}
				Set_Wheel_Speed(Temp_Speed, Temp_Speed);

				usleep(10000);
				Counter_Watcher++;
				if (Counter_Watcher > 3000) {
						if(Is_Encoder_Fail()) {
								Set_Error_Code(Error_Code_Encoder);
								Set_Touch();
						}
						return;
				}
				if (Touch_Detect()) {
						return;
				}
				if ((Check_Motor_Current() == Check_Left_Wheel) || (Check_Motor_Current() == Check_Right_Wheel)) {
						return;
				}
		}
		Set_Dir_Forward();
		Set_Wheel_Speed(0, 0);
}

void *WFM_check_trapped(void *data)
{
		float		pos_x, pos_y;
		int16_t		val;
		uint32_t	left_speed, right_speed;
		static int16_t	current_x = 0, current_y = 0;

		MapEscapeTrappedType	escaped = Map_Escape_Trapped_Escaped;

		pos_x = robot::instance()->robot_get_position_x() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		pos_y = robot::instance()->robot_get_position_y() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		Map_SetPosition(pos_x, pos_y);
		Map_SetCell(MAP, pos_x, pos_y, CLEANED);

		current_x = Map_GetXPos();
		current_y = Map_GetYPos();
		escape_thread_running = true;

		printf("%s %d: escape thread is up!\n", __FUNCTION__, __LINE__);
		while (escape_thread_running == true) {
				pos_x = robot::instance()->robot_get_position_x() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
				pos_y = robot::instance()->robot_get_position_y() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
				Map_SetPosition(pos_x, pos_y);
				Map_SetCell(MAP, pos_x, pos_y, CLEANED);

				if (abs(current_x - Map_GetXPos()) >= 2 || abs(current_y - Map_GetYPos()) >= 2) {
						path_set_current_pos();
						printf("%s %d: escape thread checking: pos: (%d, %d) (%d, %d)!\n", __FUNCTION__, __LINE__, current_x, current_y, Map_GetXPos(), Map_GetYPos());
						val = path_escape_trapped();
						if (val == 1) {
								printf("%s %d: escaped, thread is existing!\n", __FUNCTION__, __LINE__);
								data = (void *) (&escaped);
								escape_thread_running = false;
						}
						current_x = Map_GetXPos();
						current_y = Map_GetYPos();
				} else {
						usleep(100000);
				}
		}

		escape_thread_running = false;
		return NULL;
}

void WFM_boundary_check()
{
		uint8_t boundary_reach = 0;
		int16_t	j;
		int32_t	x, y;

		for (j = -1; boundary_reach == 0 && j <= 1; j++) {
#if (ROBOT_SIZE == 5)

				x = Map_GetRelativeX(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_2);
				y = Map_GetRelativeY(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_2);

#else

				x = Map_GetRelativeX(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_2);
				y = Map_GetRelativeY(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_2);

#endif

				if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
						boundary_reach = 1;
						Set_Wheel_Speed(0, 0);
						usleep(10000);

						WFM_move_back(350);
						Turn_Right(Turn_Speed, 600);
						Move_Forward(15, 15);
				}
		}
}

/*------------------------------------------------------------------ Wall Follow --------------------------*/
uint8_t Map_Wall_Follow(MapWallFollowType follow_type)
{

		uint8_t		Temp_Counter = 0;
		uint16_t	i = 0;

		int			ret;
		int16_t		Left_Wall_Buffer[3] = {0};
		int32_t		Proportion = 0, Delta = 0, Previous = 0, R = 0;

		volatile int32_t		Wall_Straight_Distance = 100, Left_Wall_Speed = 0, Right_Wall_Speed = 0;
		static volatile int32_t	Wall_Distance = Wall_High_Limit;
		pthread_t	escape_thread_id;

		MapEscapeTrappedType escape_state = Map_Escape_Trapped_Trapped;

		escape_thread_running = false;
		ret = pthread_create(&escape_thread_id, 0, WFM_check_trapped, &escape_state);
		if (ret != 0) {
				printf("%s %d: failed to create escape thread!\n", __FUNCTION__, __LINE__);
				return 2;
		} else {
				while (escape_thread_running == false) {
						usleep(10000);
				}
				printf("%s %d: escape thread is running!\n", __FUNCTION__, __LINE__);
		}

		Wall_Distance = Wall_High_Limit;

		if (Wall_Distance > Wall_High_Limit) {
				Wall_Distance = Wall_High_Limit;
		}

		if (Wall_Distance < Wall_Low_Limit) {
				Wall_Distance = Wall_Low_Limit;
		}

		Move_Forward(25, 25);

		Wall_Straight_Distance = 300;

		Left_Wall_Speed = 15;


		while (ros::ok()) {
				if (escape_thread_running == false) {
						printf("%s %d: quit due to thread exit\n", __FUNCTION__, __LINE__);
						break;
				}

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

				if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value()) | Get_Cliff_Trig()) {
						printf("%s %d: Check: Get_Bumper_Status! Break!\n", __FUNCTION__, __LINE__);
						break;
				}
				usleep(10000);
		}

		//CM_HeadToCourse(Rotate_TopSpeed, Gyro_GetAngle(0) + 900);

		/* Set escape trapped timer when it is in Map_Wall_Follow_Escape_Trapped mode. */
		escape_trapped_timer = time(NULL);

		while (ros::ok()) {
				if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME || escape_thread_running == false) {
						printf("%s %d: quit due to %s\n", __FUNCTION__, __LINE__, escape_thread_running == false ? "thread exit" : "timeout");
						break;
				}

#ifdef OBS_DYNAMIC

				OBS_Dynamic_Base(100);

#endif




				//WFM_boundary_check();
				/*------------------------------------------------------Cliff Event-----------------------*/
				if(Get_Cliff_Trig())
				{
						Set_Wheel_Speed(0,0);
						Set_Dir_Backward();
						usleep(15000);
						//                      if(Get_Cliff_Trig())
						//                      {
						Cliff_Move_Back();
						if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
						{
								Set_Clean_Mode(Clean_Mode_Userinterface);
								break;
						}
						if(Get_Cliff_Trig())
						{
								if(Cliff_Escape())
								{
										Set_Clean_Mode(Clean_Mode_Userinterface);
										return 1;
								}
						}

						Turn_Right(Turn_Speed-10,750);
						Stop_Brifly();
						Move_Forward(15,15);
						Reset_WallAccelerate();
						//Reset_Wheel_Step();
						Wall_Straight_Distance=375;

						//                              break;
						//                      }
				}
				/*---------------------------------------------------Bumper Event-----------------------*/
				if (Get_Bumper_Status() & RightBumperTrig) {
						printf("%s %d:\n", __FUNCTION__, __LINE__);
						Stop_Brifly();

						WFM_move_back(350);

						printf("%s %d: \n", __FUNCTION__, __LINE__);
						Stop_Brifly();
						Turn_Right(Turn_Speed, 700);

						printf("%s %d: \n", __FUNCTION__, __LINE__);
						Stop_Brifly();

						Move_Forward(15, 15);
						Wall_Straight_Distance = 375;
				}

				if (Get_Bumper_Status() & LeftBumperTrig) {
						printf("%s %d:\n", __FUNCTION__, __LINE__);
						Set_Wheel_Speed(0, 0);
						usleep(10000);

						if (robot::instance()->robot_get_left_wall() > (Wall_Low_Limit)) {
								Wall_Distance = robot::instance()->robot_get_left_wall()  / 3;
						} else {
								Wall_Distance += 200;
						}

						if (Wall_Distance < Wall_Low_Limit) {
								Wall_Distance = Wall_Low_Limit;
						}
						if (Wall_Distance > Wall_High_Limit) {
								Wall_Distance=Wall_High_Limit;
						}

						if (Get_Bumper_Status() & RightBumperTrig) {
								WFM_move_back(100);
								printf("%s %d: \n", __FUNCTION__, __LINE__);
								Stop_Brifly();
								Turn_Right(Turn_Speed, 600);

								Wall_Straight_Distance = MFW_Setting[follow_type].right_bumper_val; //150;
						} else {
								WFM_move_back(350);
								printf("%s %d: \n", __FUNCTION__, __LINE__);
								Stop_Brifly();
								Turn_Right(Turn_Speed, 150);
								Wall_Straight_Distance = MFW_Setting[follow_type].left_bumper_val; //250;
						}

						Wall_Straight_Distance = 200;
						printf("%s %d: \n", __FUNCTION__, __LINE__);
						Stop_Brifly();
						Move_Forward(10, 10);

						for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
								Left_Wall_Buffer[Temp_Counter] = 0;
						}
				}

				if (Wall_Distance >= 200) {
						Left_Wall_Buffer[2] = Left_Wall_Buffer[1];
						Left_Wall_Buffer[1] = Left_Wall_Buffer[0];
						Left_Wall_Buffer[0] = robot::instance()->robot_get_left_wall();
						if (Left_Wall_Buffer[0] < 100) {
								if ((Left_Wall_Buffer[1] - Left_Wall_Buffer[0]) > (Wall_Distance / 25)) {
										if ((Left_Wall_Buffer[2] - Left_Wall_Buffer[1]) > (Wall_Distance / 25)) {
												//if (Get_WallAccelerate() > 300) {
												//if ((Get_RightWheel_Speed() - Get_LeftWheel_Speed()) >= -3) {
												printf("%s %d:\n", __FUNCTION__, __LINE__);
												Move_Forward(18, 16);
												usleep(10000);
												Wall_Straight_Distance = 350;
												//}
												//}
										}
								}
						}
				}
				/*------------------------------------------------------Short Distance Move-----------------------*/
				/*		if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance) {
						if (Get_LeftWheel_Step() < 500) {
						if (Get_WallAccelerate() < 100) {
						Move_Forward(15, 15);
						} else {
						Move_Forward(20, 20);
						}
						} else {
						Move_Forward(25, 25);
						}
						} else */ {
								/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
								if (Get_FrontOBS() < Get_FrontOBST_Value()) {

										Proportion = robot::instance()->robot_get_left_wall();

										Proportion = Proportion * 100 / Wall_Distance;

										Proportion -= 100;

										Delta = Proportion - Previous;

										if (Wall_Distance > 300)//over left
										{
												Left_Wall_Speed = 25 + Proportion / 12 + Delta / 5;
												Right_Wall_Speed = 25 - Proportion / 10 - Delta / 5;
												if (Right_Wall_Speed > 33) {
														Left_Wall_Speed = 9;
														Right_Wall_Speed = 33;
												}
										} else if (Wall_Distance > 150){//over left
												Left_Wall_Speed = 22 + Proportion / 15 + Delta / 7;
												Right_Wall_Speed = 22 - Proportion / 12 - Delta / 7;

												if (Right_Wall_Speed > 27) {
														Left_Wall_Speed = 8;
														Right_Wall_Speed = 30;
												}
										}
										else{
												Left_Wall_Speed = 15 + Proportion / 22 + Delta / 10;
												Right_Wall_Speed = 15 - Proportion / 18 - Delta / 10;

												if (Right_Wall_Speed > 18) {
														Left_Wall_Speed = 5;
														Right_Wall_Speed = 18;
												}

												if (Left_Wall_Speed > 20) {
														Left_Wall_Speed = 20;
												}
												if (Left_Wall_Speed < 4) {
														Left_Wall_Speed = 4;
												}
												if (Right_Wall_Speed < 4) {
														Right_Wall_Speed = 4;
												}
												if((Left_Wall_Speed - Right_Wall_Speed) > 5) {
														Left_Wall_Speed = Right_Wall_Speed + 5;
												}
										}
										/*slow move if left obs near a wall*/
										if (Get_LeftOBS() > Get_LeftOBST_Value()){
												if (Wall_Distance < Wall_High_Limit) {
														Wall_Distance++;
												}
										}
										if (Is_WallOBS_Near()){
												Left_Wall_Speed = Left_Wall_Speed / 2;
												Right_Wall_Speed = Right_Wall_Speed / 2;
										}

										Previous = Proportion;

										if (Left_Wall_Speed < 0) {
												Left_Wall_Speed = 0;
										}
										if (Left_Wall_Speed > 40) {
												Left_Wall_Speed = 40;
										}
										if (Right_Wall_Speed < 0) {
												Right_Wall_Speed = 0;
										}

										//printf("%s %d: %d %d\n", __FUNCTION__, __LINE__, Left_Wall_Speed, Right_Wall_Speed);
										Move_Forward(Left_Wall_Speed, Right_Wall_Speed);

										//If turing around at the same point
										if (R > 7500) {
												printf("Isolated Wall Follow!\n");

												printf("%s %d: Check: Isolated Wall Follow! break\n", __FUNCTION__, __LINE__);
												break;
										}

								} else {
										printf("%s %d: \n", __FUNCTION__, __LINE__);
										Stop_Brifly();
										Turn_Right(Turn_Speed, 750);
										printf("%s %d: \n", __FUNCTION__, __LINE__);
										Stop_Brifly();
										Move_Forward(15, 15);

										Wall_Distance = Wall_High_Limit;

								}
						}
		usleep(10000);
		}

		if (escape_thread_running == true) {
				escape_thread_running = false;
		}

		pthread_join(escape_thread_id, NULL);

		ret = 0;
		if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME) {
				printf("%s %d: escape timeout %d(%d, %d), state 2\n", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME, (int)time(NULL), escape_trapped_timer);
				ret = 2;
		} else if (escape_state == Map_Escape_Trapped_Escaped) {
				printf("%s %d: escaped, state 0\n", __FUNCTION__, __LINE__);
				ret = 0;;
		} else {
				printf("%s %d: escaped, state 1\n", __FUNCTION__, __LINE__);
				ret = 1;
		}

		Stop_Brifly();
		Move_Forward(0, 0);
		return ret;
}

/*------------------------------------------------------------------ Wall Follow Mode--------------------------*/
uint8_t Wall_Follow(MapWallFollowType follow_type)
{

		uint8_t		Temp_Counter = 0, Jam = 0;
		uint16_t	i = 0;
		int32_t		Wheel_Speed_Base = 0;
		int			ret;
		int16_t		Left_Wall_Buffer[3] = {0};
		int32_t		Proportion = 0, Delta = 0, Previous = 0, R = 0;

		volatile int32_t		Wall_Straight_Distance = 100, Left_Wall_Speed = 0, Right_Wall_Speed = 0;
		static volatile int32_t	Wall_Distance = Wall_High_Limit;
		float Start_Pose_X, Start_Pose_Y;
		float Distance_From_Start;
		uint8_t		First_Time_Flag;
		uint32_t Temp_Rcon_Status;
		Reset_MoveWithRemote();
		Wall_Follow_Init_Slam();

		//Initital home point
		Home_Point.clear();
		WF_Point.clear();
		New_Home_Point.X = New_Home_Point.Y = 0;
		// Push the start point into the home point list
		Home_Point.push_front(New_Home_Point);

		Map_Initialize();
		ROS_INFO("grid map initialized");
		PathPlanning_Initialize(&Home_Point.front().X, &Home_Point.front().Y);
		ROS_INFO("path planning initialized");

		//pthread_t	escape_thread_id;
		if (Get_IMU_Status() == 0){
				Set_gyro_on();
				Set_IMU_Status();
				//printf("IMU_Status%d\n", Get_IMU_Status());
		}


		MapEscapeTrappedType escape_state = Map_Escape_Trapped_Trapped;

		escape_thread_running = false;
		//ret = pthread_create(&escape_thread_id, 0, WFM_check_trapped, &escape_state);
		/*
		   if (ret != 0) {
		   printf("%s %d: failed to create escape thread!\n", __FUNCTION__, __LINE__);
		   return 2;
		   } else {
		   while (escape_thread_running == false) {
		   usleep(10000);
		   }
		   printf("%s %d: escape thread is running!\n", __FUNCTION__, __LINE__);
		   }*/

		Wall_Distance = Wall_High_Limit;

		if (Wall_Distance > Wall_High_Limit) {
				Wall_Distance = Wall_High_Limit;
		}

		if (Wall_Distance < Wall_Low_Limit) {
				Wall_Distance = Wall_Low_Limit;
		}

		Move_Forward(25, 25);

		Wall_Straight_Distance = 300;

		Left_Wall_Speed = 15;


		while (ros::ok()) {
				/*------------------------------------WF_Map_Update---------------------------------------------------*/
				//debug_WF_map(MAP, 0, 0);
				//debug_sm_map(SPMAP, 0, 0);

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

				WF_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));
				Temp_Rcon_Status = robot::instance()->robot_get_rcon();
				//ROS_INFO("Temp_Rcon_Status = %d", Temp_Rcon_Status);
				if(Temp_Rcon_Status & (RconFL_HomeT | RconFR_HomeT | RconL_HomeT | RconR_HomeT)){
					CM_SetHome(Map_GetXCount(), Map_GetYCount());
				}

				if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value()) | Get_Cliff_Trig()) {
						printf("%s %d: Check: Get_Bumper_Status! Break!\n", __FUNCTION__, __LINE__);
						break;
				}

				usleep(10000);
		}

		//CM_HeadToCourse(Rotate_TopSpeed, Gyro_GetAngle(0) + 900);

		/* Set escape trapped timer when it is in Map_Wall_Follow_Escape_Trapped mode. */
		escape_trapped_timer = time(NULL);
		Start_Pose_X =  robot::instance()->robot_get_position_x();
		Start_Pose_Y =  robot::instance()->robot_get_position_y();
		First_Time_Flag = 1;
		while (ros::ok()) {
				/*
				   if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME || escape_thread_running == false) {
				   printf("%s %d: quit due to %s\n", __FUNCTION__, __LINE__, escape_thread_running == false ? "thread exit" : "timeout");
				   break;
				   }
				 */
#if 0
				/*-------------------------------------------------Start Pose Check------------------------------*/
				if (First_Time_Flag == 0){
						if ((Distance_From_Start = (sqrtf(powf(Start_Pose_X - robot::instance()->robot_get_position_x(), 2) + powf(Start_Pose_Y - robot::instance()->robot_get_position_y(), 2)))) < 0.303 ){
								/*
								   CM_MoveToCell(0, 0, 2, 0, 1);
								   ROS_INFO("In Start Pose, finish wall follow.");
								// Beep for the finish signal.
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
				}
				else{
						if ((Distance_From_Start = (sqrtf(powf(Start_Pose_X - robot::instance()->robot_get_position_x(), 2) + powf(Start_Pose_Y - robot::instance()->robot_get_position_y(), 2)))) > 0.303 ){
								ROS_INFO("Out Start Pose");
								First_Time_Flag = 0;
						}
				}
#endif
				//ROS_INFO("Distance_From_Start = %f", Distance_From_Start);

				/*------------------------------------WF_Map_Update---------------------------------------------------*/
				//WF_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));
				WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
				if(reach_count >= 10){
						WF_End_Wall_Follow();
						break;
				}

				Temp_Rcon_Status = robot::instance()->robot_get_rcon();
				//ROS_INFO("Temp_Rcon_Status = %d", Temp_Rcon_Status);
				if(Temp_Rcon_Status & (RconFL_HomeT | RconFR_HomeT | RconL_HomeT | RconR_HomeT)){
					CM_SetHome(Map_GetXCount(), Map_GetYCount());
				}
				//CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));
				//update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));
				//rounding_update();
				//debug_WF_map(MAP, 0, 0);
				//debug_sm_map(SPMAP, 0, 0);

#ifdef OBS_DYNAMIC

				OBS_Dynamic_Base(100);

#endif
				/*
				   printf("WF_position_x = %f\n", robot::instance()->robot_get_WF_position_x());
				   printf("WF_position_y = %f\n", robot::instance()->robot_get_WF_position_y());
				   printf("position_x = %f\n", robot::instance()->robot_get_position_x());
				   printf("position_y = %f\n", robot::instance()->robot_get_position_y());
				 */

				//printf("wall_following\n");
				//WFM_boundary_check();
				/*------------------------------------------------------Check Current--------------------------------*/
				if (Check_Motor_Current()) {
						printf("Check_Motor_Current_Error\n");
						Set_Clean_Mode(Clean_Mode_Userinterface);
						break;
				}

				/*------------------------------------------------------Touch and Remote event-----------------------*/
				if (Touch_Detect()) {
						printf("Touch\n");
						Reset_Touch();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						break;
				}
				/*
				   if (Get_Rcon_Remote()) {
				   printf("Rcon\n");
				   Reset_Touch();
				   Set_Clean_Mode(Clean_Mode_Userinterface);
				   break;
				   }*/
				if (Get_Rcon_Remote()) {
						printf("Rcon\n");
						if(Is_MoveWithRemote())
						{
								/*if (Remote_Key(Remote_Random)) {
										Set_Clean_Mode(Clean_Mode_RandomMode);
										break;
								}*/
						
						}
						if (Remote_Key(Remote_Home)) {
								Set_MoveWithRemote();
								Set_Clean_Mode(Clean_Mode_GoHome);
								Move_Forward(10, 10);
								SetHomeRemote();
								break;
						}
						if (Remote_Key(Remote_Spot)) {
								Set_MoveWithRemote();
								Set_Clean_Mode(Clean_Mode_Spot);
								break;
						}
						Reset_Rcon_Remote();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						break;
				}
				/*------------------------------------------------------Check Battery-----------------------*/
				if (Check_Bat_SetMotors(135000, 80000, 100000)) {       //Low Battery Event
						if(Is_MoveWithRemote()){
								Display_Battery_Status(Display_Low);//min_distant_segment low
								usleep(30000);
								Set_Clean_Mode(Clean_Mode_GoHome);
								break;
						}else{
								Set_Clean_Mode(Clean_Mode_GoHome);
								break;
						}
				}
				/*------------------------------------------------------Cliff Event-----------------------*/
				if(Get_Cliff_Trig())
				{
						Set_Wheel_Speed(0,0);
						Set_Dir_Backward();
						usleep(15000);
						//                      if(Get_Cliff_Trig())
						//                      {
						Cliff_Move_Back();
						if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
						{
								Set_Clean_Mode(Clean_Mode_Userinterface);
								break;
						}
						if(Get_Cliff_Trig())
						{
								if(Cliff_Escape())
								{
										Set_Clean_Mode(Clean_Mode_Userinterface);
										return 1;
								}
						}

						Turn_Right(Turn_Speed-10,750);
						Stop_Brifly();
						Move_Forward(15,15);
						Reset_WallAccelerate();
						//Reset_Wheel_Step();
						Wall_Straight_Distance=375;

						//                              break;
						//                      }
				}

#if 0
				/*---------------------------------------------------Bumper Event-----------------------*/
				if (Get_Bumper_Status() & RightBumperTrig) {
						printf("%s %d:\n", __FUNCTION__, __LINE__);
						Stop_Brifly();

						WFM_move_back(350);

						printf("%s %d: \n", __FUNCTION__, __LINE__);
						Stop_Brifly();
						Turn_Right(Turn_Speed, 700);

						printf("%s %d: \n", __FUNCTION__, __LINE__);
						Stop_Brifly();

						Move_Forward(15, 15);
						Wall_Straight_Distance = 375;
				}

				if (Get_Bumper_Status() & LeftBumperTrig) {
						printf("%s %d:\n", __FUNCTION__, __LINE__);
						Set_Wheel_Speed(0, 0);
						usleep(10000);

						if (robot::instance()->robot_get_left_wall() > (Wall_Low_Limit)) {
								Wall_Distance = robot::instance()->robot_get_left_wall()  / 3;
						} else {
								Wall_Distance += 200;
						}

						if (Wall_Distance < Wall_Low_Limit) {
								Wall_Distance = Wall_Low_Limit;
						}
						if (Wall_Distance > Wall_High_Limit) {
								Wall_Distance=Wall_High_Limit;
						}

						if (Get_Bumper_Status() & RightBumperTrig) {
								WFM_move_back(100);
								printf("%s %d: \n", __FUNCTION__, __LINE__);
								Stop_Brifly();
								Turn_Right(Turn_Speed, 600);

								Wall_Straight_Distance = MFW_Setting[follow_type].right_bumper_val; //150;
						} else {
								WFM_move_back(350);
								printf("%s %d: \n", __FUNCTION__, __LINE__);
								Stop_Brifly();
								Turn_Right(Turn_Speed, 150);
								Wall_Straight_Distance = MFW_Setting[follow_type].left_bumper_val; //250;
						}

						Wall_Straight_Distance = 200;
						printf("%s %d: \n", __FUNCTION__, __LINE__);
						Stop_Brifly();
						Move_Forward(10, 10);

						for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
								Left_Wall_Buffer[Temp_Counter] = 0;
						}
				}

				if (Wall_Distance >= 200) {
						Left_Wall_Buffer[2] = Left_Wall_Buffer[1];
						Left_Wall_Buffer[1] = Left_Wall_Buffer[0];
						Left_Wall_Buffer[0] = robot::instance()->robot_get_left_wall();
						if (Left_Wall_Buffer[0] < 100) {
								if ((Left_Wall_Buffer[1] - Left_Wall_Buffer[0]) > (Wall_Distance / 25)) {
										if ((Left_Wall_Buffer[2] - Left_Wall_Buffer[1]) > (Wall_Distance / 25)) {
												//if (Get_WallAccelerate() > 300) {
												//if ((Get_RightWheel_Speed() - Get_LeftWheel_Speed()) >= -3) {
												printf("%s %d:\n", __FUNCTION__, __LINE__);
												Move_Forward(18, 16);
												usleep(10000);
												Wall_Straight_Distance = 350;
												//}
												//}
										}
								}
						}
				}
#endif

		/*------------------------------------------------------Home Station Event------------------------*/
		//Temp_Rcon_Status = Get_Rcon_Status();  
		Temp_Rcon_Status = robot::instance()->robot_get_rcon();
		
		if (Temp_Rcon_Status)
		{
			Reset_Rcon_Status();
#if 0
			if (follow_type == Map_Wall_Follow_Left_Target || follow_type == Map_Wall_Follow_Left_Zone) 
			{
#ifdef ZONE_WALLFOLLOW
				if ( Temp_Rcon_Status & (RconFR_HomeT | RconFL_HomeT | RconFL2_HomeT | RconFR2_HomeT | RconL_HomeT | RconR_HomeT ) ) 	//Wait for modification
				{  
					CM_SetStationHome();
					bumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1;
					USPRINTF("%s %d: BumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1!\n", __FUNCTION__, __LINE__);
					USPRINTF_ZZ("%s %d: Home detected!\n", __FUNCTION__, __LINE__);
				}

				if (Temp_Rcon_Status & RconFrontAll_Home_T) 
				{
					x = Map_GetRelativeX(Gyro_GetAngle(0), CELL_SIZE_2, 0);
					y = Map_GetRelativeY(Gyro_GetAngle(0), CELL_SIZE_2, 0);
					/*------------------ Edit By ZZ ----------------------*/
					Map_SetCell(MAP, x, y, BLOCKED_OBS);
//					Map_SetCell(MAP, x, y, BLOCKED_BOUNDARY);
					/**********************************************************/
					if(Temp_Rcon_Status & RconFR_HomeT)
					{
						Turn_Right(Turn_Speed,850);
					}
					else if(Temp_Rcon_Status & RconFL_HomeT)
					{
						Turn_Right(Turn_Speed,850);
					}
					else if(Temp_Rcon_Status & RconL_HomeT)
					{
						Turn_Right(Turn_Speed,300);
					}
					else if(Temp_Rcon_Status & RconFL2_HomeT)
					{
						Turn_Right(Turn_Speed,600);
					}
					else if(Temp_Rcon_Status & RconFR2_HomeT)
					{
						Turn_Right(Turn_Speed,950);
					}
					else if(Temp_Rcon_Status & RconR_HomeT)
					{
						Turn_Right(Turn_Speed,1100);
					}	
				}

#ifdef VIRTUAL_WALL   
				/*
				 * When checking virtual wall signals, ignore the RconBL_Wall signal.
				 */
				Temp_Rcon_Status = Get_VirtualWall_Value();
				if (Temp_Rcon_Status & (RconL_Wall_T | RconR_Wall_T | RconBR_Wall_T | RconFR_Wall_T | RconFL_Wall_T | RconFL2_Wall_T | RconFR2_Wall_T))
				{
					bumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1;
					x = Map_GetRelativeX(Gyro_GetAngle(0), CELL_SIZE_2, 0);
					y = Map_GetRelativeY(Gyro_GetAngle(0), CELL_SIZE_2, 0);
					Map_SetCell(MAP, x, y, BLOCKED_OBS);	
					
					if (Temp_Rcon_Status & (RconFR_Wall_T | RconFL_Wall_T))
					{
						Turn_Right(Turn_Speed, 800);
					} 
					else if (Temp_Rcon_Status & RconR_Wall_T) 
					{
						Turn_Right(Turn_Speed, 1400);
					}
					else if (Temp_Rcon_Status & RconBR_Wall_T)
					{
						Turn_Right(Turn_Speed, 1500);
					} 
					else if (Temp_Rcon_Status & RconFR2_Wall_T)
					{
						Turn_Right(Turn_Speed, 1300);
					} 
					else if	(Temp_Rcon_Status & RconFL2_Wall_T)
					{
						Turn_Right(Turn_Speed, 450);
					}
					else if (Temp_Rcon_Status & RconL_Wall_T)
					{
						Turn_Right(Turn_Speed, 300);
					}
					Reset_VirtualWall_Value(RconL_Wall_T | RconR_Wall_T | RconBR_Wall_T | RconFR_Wall_T | RconFL_Wall_T | RconFL2_Wall_T | RconFR2_Wall_T);				
				}
				
				if (Temp_Rcon_Status & (RconL_Wall | RconR_Wall | RconBR_Wall | RconFR_Wall | RconFL_Wall | RconFL2_Wall | RconFR2_Wall))
				{
					bumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1;
					USPRINTF_ZZ("%s %d: virtual wall detected! BumperCount = COMPLICATED_AREA_BUMPER_MAX_COUNT + 1! %d\n", __FUNCTION__, __LINE__, Temp_Rcon_Status);

					x = Map_GetRelativeX(Gyro_GetAngle(0), CELL_SIZE_2, 0);
					y = Map_GetRelativeY(Gyro_GetAngle(0), CELL_SIZE_2, 0);
					Map_SetCell(MAP, x, y, BLOCKED_OBS);
					if (Temp_Rcon_Status & (RconFR_Wall | RconFL_Wall))
					{
						Turn_Right(Turn_Speed, 450);
					} 
					else if (Temp_Rcon_Status & RconR_Wall) 
					{
						Turn_Right(Turn_Speed, 300);
					}
					else if (Temp_Rcon_Status & RconFR2_Wall) 
					{
						Turn_Right(Turn_Speed, 400);
					}
					else if (Temp_Rcon_Status & RconBR_Wall)
					{
						Turn_Right(Turn_Speed, 300);
					} 
					else if (Temp_Rcon_Status & RconFL2_Wall)
					{
						Turn_Right(Turn_Speed, 500);
					}				
					else if (Temp_Rcon_Status & RconL_Wall)
					{
						Turn_Right(Turn_Speed, 600);
					}
					Reset_VirtualWall();
				}

#endif

#endif
			}

			else
#endif
			{
				if (Temp_Rcon_Status &  RconFrontAll_Home_TLR) 
				{	
					/*
					if (Is_WorkFinish(Get_Room_Mode())) 
					{
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
						break;
					}
					Stop_Brifly();
					if(Temp_Rcon_Status & RconFR_HomeT)
					{
						Turn_Right(Turn_Speed,850);
					}
					else if(Temp_Rcon_Status & RconFL_HomeT)
					{
						Turn_Right(Turn_Speed,850);
					}
					else if(Temp_Rcon_Status & RconL_HomeT)
					{
						Turn_Right(Turn_Speed,300);
					}
					else if(Temp_Rcon_Status & RconFL2_HomeT)
					{
						Turn_Right(Turn_Speed,600);
					}
					else if(Temp_Rcon_Status & RconFR2_HomeT)
					{
						Turn_Right(Turn_Speed,950);
					}
					else if(Temp_Rcon_Status & RconR_HomeT)
					{
						Turn_Right(Turn_Speed,1100);
					}	
					Stop_Brifly();
					Move_Forward(10, 10);
					Reset_Rcon_Status();
					Wall_Straight_Distance = 80;
					Reset_WallAccelerate();
				}
			}
		}
#if 0
#ifdef ZONE_WALLFOLLOW
		/*------------------------------------- Map Boundary -------------------------------------*/
		if ( wallFoundStatus != Map_Find_Wall_Not_Found ) 							//»¹Ã»ÓÐÕÒµ½µÚÒ»ÌõÖ±Ïß±ß¾Í²»ÊÜ±ß½çÓ°Ïì
		{
			if (follow_type != Map_Wall_Follow_Left_Target && follow_type != Map_Wall_Follow_Left_Zone) 
			{
				WFM_boundary_check();
			}
			else 
			{
				if ( xMax - xMin >= MAP_SIZE - 5 || yMax - yMin >= MAP_SIZE - 5 ) 
				{
					if ( xMax - Map_GetXPos() < 5 || Map_GetXPos() - xMin < 5 ||  yMax - Map_GetYPos() < 5 || Map_GetYPos() - yMin < 5 ) 
					{
						WFM_boundary_check();
					}
				}
			}
		}
#else
		WFM_boundary_check();
#endif
#endif

		/*---------------------------------------------------Bumper Event-----------------------*/
		if (Get_Bumper_Status() & RightBumperTrig) 
		{
			//Base_Speed = BASE_SPEED;
			//Left_BH_Counter++;
			//Wall_BH_Counter++;
			
			//STOP_BRIFLY;
			Stop_Brifly();
			
			//WFM_update();
			WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));

			//WFM_wall_move_back();
			WFM_move_back(350);

			//WFM_update();
			WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));


			Wall_Distance+=300;
			if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;	
			
			//STOP_BRIFLY;
			Stop_Brifly();
			Turn_Right(Turn_Speed-5, 920);

			//bumperCount++;

			//STOP_BRIFLY;
			Stop_Brifly();

			//WFM_update();
			WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
			
			Move_Forward(10, 10);
			Reset_WallAccelerate();
			Wall_Straight_Distance = 200;

			Reset_Wheel_Step();
		}

		if (Get_Bumper_Status() & LeftBumperTrig) 
		{
			//Base_Speed = BASE_SPEED;
			//L_B_Counter++;
			Set_Wheel_Speed(0, 0);
			Reset_TempPWM();
			//delay(10);
			usleep(1000);
			//WFM_update();
			WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
			if (Get_Bumper_Status() & RightBumperTrig)
			{
					//USPRINTF_ZZ("%s %d:Double bumper are trigged!",__func__,__LINE__);
					//WFM_move_back(100);
					WFM_move_back(100);

					//WFM_update();
					WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
					if (Is_Bumper_Jamed())
					{

							Reset_Touch();
							CM_TouringCancel();
							Set_Clean_Mode(Clean_Mode_Userinterface);
							//USPRINTF("%s %d: Check: Bumper 2! break\n", __FUNCTION__, __LINE__);
							break;
					}
					//STOP_BRIFLY;
					Stop_Brifly();
					Turn_Right(Turn_Speed-5, 850);
#if 0
#ifdef ZONE_WALLFOLLOW
					//Detect bumper hitting in a short distance
					if ( TwoPointsDistance(tmpCell3.X, tmpCell3.Y, Map_GetXPos(), Map_GetYPos()) < 3 )
					{
						bumperCount++;
					} 
					else 
					{
						tmpCell3.X = Map_GetXPos();
						tmpCell3.Y = Map_GetYPos();
					}
					USPRINTF("%s %d Bumper count: %d!\n",  __FUNCTION__, __LINE__, bumperCount);
#endif
#endif

					Wall_Straight_Distance = MFW_Setting[follow_type].right_bumper_val; //150;
					Wall_Distance+=300;
					if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
				} 
				else 
				{
					Wall_Distance-=100;
					if(Wall_Distance<Wall_Low_Limit)Wall_Distance=Wall_Low_Limit;			
					
					//WFM_wall_move_back();
					WFM_move_back(350);
					//WFM_update();
					WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
					if (Is_Bumper_Jamed()) 
					{
						Reset_Touch();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						//USPRINTF("%s %d: Check: Bumper 3! break\n", __FUNCTION__, __LINE__);
						break;
					}

					//STOP_BRIFLY;
					Stop_Brifly();
					if (Jam < 3) 
					{
						if(Wall_Distance<200)
						{
							if(Get_LeftOBS()>(Get_LeftOBST_Value()-200))
							{
								Wall_Distance=Wall_High_Limit;
								Turn_Right(Turn_Speed-5, 300);
							}
							else
							{
								Turn_Right(Turn_Speed-5, 200);
							}
						}
						else
						{
							Turn_Right(Turn_Speed-5, 300);
						}
#if 0
#ifdef ZONE_WALLFOLLOW
						//Detect bumper hitting in a short distance
						if ( TwoPointsDistance(tmpCell3.X, tmpCell3.Y, Map_GetXPos(), Map_GetYPos()) < 3 )
						{
							bumperCount++;
						}
						else 
						{
							tmpCell3.X = Map_GetXPos();
							tmpCell3.Y = Map_GetYPos();
						}
						USPRINTF("%s %d Bumper count: %d!\n",  __FUNCTION__, __LINE__, bumperCount);
#endif
#endif

					}
					else 
					{
//						if(Wall_Distance<200)
//						{
							Turn_Right(Turn_Speed-5, 200);
//						}
//						else
//						{
//							Turn_Right(Turn_Speed - 10, 150);
//						}
//
#if 0
#ifdef ZONE_WALLFOLLOW
						//Detect bumper hitting in a short distance
						if ( TwoPointsDistance(tmpCell3.X, tmpCell3.Y, Map_GetXPos(), Map_GetYPos()) < 3 )
						{
							bumperCount++;
						}
						else 
						{
							tmpCell3.X = Map_GetXPos();
							tmpCell3.Y = Map_GetYPos();
						}
						USPRINTF("%s %d Bumper count: %d!\n",  __FUNCTION__, __LINE__, bumperCount);
#endif
#endif
					}
					Wall_Straight_Distance = MFW_Setting[follow_type].left_bumper_val; //250;
				}

				if (Get_WallAccelerate() < 2000)
				{
					Jam++;
				}
				else 
				{
					Jam = 0;
				}

				Reset_WallAccelerate();
				Wall_Straight_Distance = 200;
				//STOP_BRIFLY;
				Stop_Brifly();
				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
				
				Move_Forward(10, 10);

				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
				{
					Left_Wall_Buffer[Temp_Counter] = 0;
				}
				Reset_Wheel_Step();
			}

			if (Jam > 80) 
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				Set_Error_Code(Error_Code_Encoder);
				//CM_TouringCancel();
				//USPRINTF("%s %d: Check: Bumper 4! break\n", __FUNCTION__, __LINE__);
				break;
			}
//			if ( L_B_Counter > 10 ) {
//				if (!Is_MoveWithRemote()) {
//					if (!(follow_type == Map_Wall_Follow_Left_Zone || follow_type == Map_Wall_Follow_Left_Target || follow_type == Map_Wall_Follow_Find_Wall)) {
//						Set_Clean_Mode(Clean_Mode_RandomMode);
//						USPRINTF("%s %d: Check: Bumper 5! break\n", __FUNCTION__, __LINE__);
//						break;
//					}
//				}
//			}
//		}
#if 0
				/*------------------------------------------------------Short Distance Move-----------------------*/
				/*		if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance) {
						if (Get_LeftWheel_Step() < 500) {
						if (Get_WallAccelerate() < 100) {
						Move_Forward(15, 15);
						} else {
						Move_Forward(20, 20);
						}
						} else {
						Move_Forward(25, 25);
						}
						} else */ {
								/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
								if (Get_FrontOBS() < Get_FrontOBST_Value()) {

										Proportion = robot::instance()->robot_get_left_wall();

										Proportion = Proportion * 100 / Wall_Distance;

										Proportion -= 100;

										Delta = Proportion - Previous;

										if (Wall_Distance > 300)//over left
										{
												Left_Wall_Speed = 25 + Proportion / 12 + Delta / 5;
												Right_Wall_Speed = 25 - Proportion / 10 - Delta / 5;
												if (Right_Wall_Speed > 33) {
														Left_Wall_Speed = 9;
														Right_Wall_Speed = 33;
												}
										} else if (Wall_Distance > 150){//over left
												Left_Wall_Speed = 22 + Proportion / 15 + Delta / 7;
												Right_Wall_Speed = 22 - Proportion / 12 - Delta / 7;

												if (Right_Wall_Speed > 27) {
														Left_Wall_Speed = 8;
														Right_Wall_Speed = 30;
												}
										}
										else{
												Left_Wall_Speed = 15 + Proportion / 22 + Delta / 10;
												Right_Wall_Speed = 15 - Proportion / 18 - Delta / 10;

												if (Right_Wall_Speed > 18) {
														Left_Wall_Speed = 5;
														Right_Wall_Speed = 18;
												}

												if (Left_Wall_Speed > 20) {
														Left_Wall_Speed = 20;
												}
												if (Left_Wall_Speed < 4) {
														Left_Wall_Speed = 4;
												}
												if (Right_Wall_Speed < 4) {
														Right_Wall_Speed = 4;
												}
												if((Left_Wall_Speed - Right_Wall_Speed) > 5) {
														Left_Wall_Speed = Right_Wall_Speed + 5;
												}
										}
										/*slow move if left obs near a wall*/
										if (Get_LeftOBS() > Get_LeftOBST_Value()){
												if (Wall_Distance < Wall_High_Limit) {
														Wall_Distance++;
												}
										}
										if (Is_WallOBS_Near()){
												Left_Wall_Speed = Left_Wall_Speed / 2;
												Right_Wall_Speed = Right_Wall_Speed / 2;
										}

										Previous = Proportion;

										if (Left_Wall_Speed < 0) {
												Left_Wall_Speed = 0;
										}
										if (Left_Wall_Speed > 40) {
												Left_Wall_Speed = 40;
										}
										if (Right_Wall_Speed < 0) {
												Right_Wall_Speed = 0;
										}

										//printf("%s %d: %d %d\n", __FUNCTION__, __LINE__, Left_Wall_Speed, Right_Wall_Speed);
										Move_Forward(Left_Wall_Speed, Right_Wall_Speed);

										//If turing around at the same point
										if (R > 7500) {
												printf("Isolated Wall Follow!\n");

												printf("%s %d: Check: Isolated Wall Follow! break\n", __FUNCTION__, __LINE__);
												break;
										}

								} else {
										printf("%s %d: \n", __FUNCTION__, __LINE__);
										Stop_Brifly();
										Turn_Right(Turn_Speed, 750);
										printf("%s %d: \n", __FUNCTION__, __LINE__);
										Stop_Brifly();
										Move_Forward(15, 15);

										Wall_Distance = Wall_High_Limit;

								}
						}
#endif
		/*------------------------------------------------------Short Distance Move-----------------------*/
		if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance) 
		{
				//WFM_update();
				WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
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
				WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
		} 
		else 
		{
				/*------------------------------------------------------Wheel Speed adjustment-----------------------*/


#ifdef OBS_DYNAMIC
				if (Get_FrontOBS() < Get_FrontOBST_Value()) 
				{
#else
						if (Get_FrontOBS() < MFW_Setting[follow_type].front_obs_val)
						{
#endif

								Wheel_Speed_Base = 15 + Get_WallAccelerate() / 150;
								if(Wheel_Speed_Base>28)Wheel_Speed_Base = 28;

								Proportion = robot::instance()->robot_get_left_wall();

								Proportion = Proportion*100/Wall_Distance;

								Proportion-=100;

								Delta = Proportion - Previous;

								if(Wall_Distance>200)//over left
								{
										Left_Wall_Speed = Wheel_Speed_Base + Proportion/8 + Delta/3; //12
										Right_Wall_Speed = Wheel_Speed_Base - Proportion/9 - Delta/3; //10

										if(Wheel_Speed_Base<26)
										{
												if(Right_Wall_Speed > Wheel_Speed_Base+6)
												{
														Right_Wall_Speed = 34;
														Left_Wall_Speed = 4;
												}
												else if(Left_Wall_Speed > Wheel_Speed_Base+10)
												{
														Right_Wall_Speed = 5;		
														Left_Wall_Speed = 30;

												}
										}
										else
										{
												if(Right_Wall_Speed > 35)
												{
														Right_Wall_Speed = 35;
														Left_Wall_Speed = 4;
												}
										}				
								}
								else 
								{
										Left_Wall_Speed = Wheel_Speed_Base + Proportion/10 + Delta/3;  //16
										Right_Wall_Speed = Wheel_Speed_Base - Proportion/10 - Delta/4; //11

										if(Wheel_Speed_Base<26)
										{
												if(Right_Wall_Speed > Wheel_Speed_Base+4)
												{
														Right_Wall_Speed = 34;
														Left_Wall_Speed = 4;
												}

										}
										else
										{
												if(Right_Wall_Speed > 32)
												{
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
								/*
								//If wall follow a long distance, then save offset
								if ( TwoPointsDistance(tmpCell2.X, tmpCell2.Y, Map_GetXPos(), Map_GetYPos()) > 5 ) 
								{
										tmpCell2.X = Map_GetXPos();
										tmpCell2.Y = Map_GetYPos();
										rightWheelOffset = Get_RightWall_Step();
										leftWheelOffset = Get_LeftWall_Step();
								}*/
								
								/*
								//Rotating in a short distance area
								if (Get_RightWall_Step() - rightWheelOffset > Get_LeftWall_Step() - leftWheelOffset)
								{
										R = Get_RightWall_Step() - rightWheelOffset - ( Get_LeftWall_Step() - leftWheelOffset );
								}
								//USPRINTF("%s %d: R: %d\n", __FUNCTION__, __LINE__, R);

								//If turing around at the same point
								if (R > 7500)
								{
										USPRINTF("Isolated Wall Follow!\n");
										isIsolatedWallFollow = 1;
										USPRINTF_ZZ("%s %d: Check: Isolated Wall Follow! break\n", __FUNCTION__, __LINE__);
										break;
								}*/
#if 0
								if ((Get_RightWall_Step() > Map_Wall_Follow_Distance) ||(Get_LeftWall_Step() > Map_Wall_Follow_Distance)
												/*||
												  (TwoPointsDistance(startCell.X, startCell.Y, Map_GetXPos(), Map_GetYPos()) > 9)*/)
								{	//about 5 Meter, 6000 = 1 m
										Reset_Wall_Step();
										//					if (!Is_MoveWithRemote()) {
										//						Set_Clean_Mode(Clean_Mode_RandomMode);
										//						USPRINTF("%s %d: Check: Short Distance 3! break\n", __FUNCTION__, __LINE__);
										//						break;
										//					}
										//				}
						}
#endif
						/*if (Get_WallAccelerate() > 750) 
						{
								Set_Left_Brush(ENABLE);
								Set_Right_Brush(ENABLE);
						}*/
				} 
				else 
				{		
						Stop_Brifly();
						if (Get_WallAccelerate() < 2000)
						{
								Jam++;
						}
						Turn_Right(Turn_Speed - 5, 920);
						Stop_Brifly();
						//WFM_update();
						WF_Check_Loop_Closed(Gyro_GetAngle(0), Gyro_GetAngle(1));
						Move_Forward(15, 15);
						Reset_Wheel_Step();

						Wall_Distance = Wall_High_Limit;

				}
		}
		usleep(10000);
		}

		if (escape_thread_running == true) {
				escape_thread_running = false;
		}

		//pthread_join(escape_thread_id, NULL);

		ret = 0;
		if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME) {
				printf("%s %d: escape timeout %d(%d, %d), state 2\n", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME, (int)time(NULL), escape_trapped_timer);
				ret = 2;
		} else if (escape_state == Map_Escape_Trapped_Escaped) {
				printf("%s %d: escaped, state 0\n", __FUNCTION__, __LINE__);
				ret = 0;;
		} else {
				printf("%s %d: escaped, state 1\n", __FUNCTION__, __LINE__);
				ret = 1;
		}

		Stop_Brifly();
		Move_Forward(0, 0);
		return ret;
}

void Wall_Follow_Init_Slam(void){
		extern int8_t enable_slam_offset;
		extern void start_slam(void);
		robot::instance()->Subscriber();
		robot::instance()->start_lidar();
		//std::async(std::launch::async, start_slam);
		start_slam();
		/*while (robot::instance()->map_ready() == false || ros::ok()){
		  usleep(100);
		  ROS_WARN("waiting for map");
		  }*/
		sleep(5);
		enable_slam_offset = 2;
}

void Wall_Follow_Stop_Slam(void){
		extern int8_t enable_slam_offset;
		extern void start_slam(void);
		robot::instance()->UnSubscriber();
		Disable_Motors();
		robot::instance()->stop_lidar();
		//std::async(std::launch::async, start_slam);
		robot::instance()->stop_slam();
		/*while (robot::instance()->map_ready() == false || ros::ok()){
		  usleep(100);
		  ROS_WARN("waiting for map");
		  }*/
		enable_slam_offset = 0;
}
uint8_t WF_End_Wall_Follow(void){
		int16_t i;
		int8_t state;
		// X, Y in Target_Point are all counts.
		Point32_t   Next_Point, Target_Point;
		Point16_t   tmpPnt, pnt16ArTmp[3];
		MapTouringType  mt_state = MT_None;
		int16_t home_angle = robot::instance()->robot_get_home_angle();

		//CM_MoveToCell(0, 0, 2, 0, 1);
		/*
		   tmpPnt.X = countToCell(Home_Point.front().X);
		   tmpPnt.Y = countToCell(Home_Point.front().Y);
		   state = CM_MoveToCell(tmpPnt.X, tmpPnt.Y, 2, 0, 1);

		   ROS_INFO("In Start Pose, finish wall follow.");
		// Beep for the finish signal.
		for (i = 10; i > 0; i--) {
		Beep(i, 6, 0, 1);
		usleep(100000);
		}*/
		/**************************Go Home***************************/
		tmpPnt.X = countToCell(Home_Point.front().X);
		tmpPnt.Y = countToCell(Home_Point.front().Y);
		pnt16ArTmp[0] = tmpPnt;
		path_escape_set_trapped_cell(pnt16ArTmp, 1);

		if (remote_go_home == 1) {
				Set_LED(100, 100);
				SetHomeRemote();
		}


		CM_create_home_boundary();

		// Try all the saved home point until it reach the charger stub. (There will be at least one home point (0, 0).)
		tmpPnt.X = countToCell(Home_Point.front().X);
		tmpPnt.Y = countToCell(Home_Point.front().Y);
		// Delete the first home point, it works like a stack.
		Home_Point.pop_front();
		while (ros::ok())
		{
				ROS_INFO("Go home Target: (%d, %d), %u targets left.", tmpPnt.X, tmpPnt.Y, Home_Point.size());
				// Try go to exactly this home point.
				state = CM_MoveToCell( tmpPnt.X, tmpPnt.Y, 2, 0, 1 );
				ROS_INFO("CM_MoveToCell return %d.", state);

				if ( state == -2 && Home_Point.empty()) {
						// If it is the last saved home point, stop the robot.
						Disable_Motors();
						// Beep for the finish signal.
						for (i = 10; i > 0; i--) {
								Beep(i, 6, 0, 1);
								usleep(100000);
						}

						if (from_station >= 1) {
								Set_Clean_Mode(Clean_Mode_GoHome);
						} else {
								Set_Clean_Mode(Clean_Mode_Userinterface);
						}

						printf("%s %d: Finish cleanning but not stop near home, cleaning time: %d(s)\n", __FUNCTION__, __LINE__, Get_Work_Time());
						break;

				} else if (state == -3 && Home_Point.empty()) {
						// If it is the last saved home point, stop the robot.
						Disable_Motors();
						mt_state = MT_Battery;
						break;
				} else if (state == -5 && Home_Point.empty()) {
						// If it is the last saved home point, stop the robot.
						Disable_Motors();
						// Beep for the finish signal.
						for (i = 10; i > 0; i--) {
								Beep(i, 6, 0, 1);
								usleep(100000);
						}
						Set_Clean_Mode(Clean_Mode_Userinterface);
						printf("%s %d: Finish cleanning, cleaning time: %d(s)\n", __FUNCTION__, __LINE__, Get_Work_Time());
						break;
				} else if (state == 1 || state == -7) {
						// Call GoHome() function to try to go to charger stub.
						GoHome();

						// Check the clean mode to find out whether it has reach the charger.
						if (Get_Clean_Mode() == Clean_Mode_Charging)
						{
								printf("%s %d: Finish cleanning, cleaning time: %d(s)\n", __FUNCTION__, __LINE__, Get_Work_Time());
								break;
						}
						else if (Home_Point.empty())
						{
								// If it is the last point, it means it it now at (0, 0).
								if (from_station == 0) {
										CM_HeadToCourse(ROTATE_TOP_SPEED, home_angle);

										if (Touch_Detect()) {
												break;
										}
								}

								Disable_Motors();
								// Beep for the finish signal.
								for (i = 10; i > 0; i--) {
										Beep(i, 6, 0, 1);
										usleep(100000);
								}

								if (from_station >= 1) {
										Set_Clean_Mode(Clean_Mode_GoHome);
								} else {
										Set_Clean_Mode(Clean_Mode_Userinterface);
								}

								printf("%s %d: Finish cleanning, cleaning time: %d(s)\n", __FUNCTION__, __LINE__, Get_Work_Time());
								break;
						}

				}

				if (!Home_Point.empty())
				{
						// Get next home point 
						tmpPnt.X = countToCell(Home_Point.front().X);
						tmpPnt.Y = countToCell(Home_Point.front().Y);
						Home_Point.pop_front();
				}
				else
				{
						ROS_INFO("No home targets left and last target return %d, this message should not be printed, please check.", state);
						break;
				}
		}

		/***************************2.2-1 Go Home End***************************/





		/*****************************************Release Memory************************************/
		Wall_Follow_Stop_Slam();
		std::vector<Point32_t>(WF_Point).swap(WF_Point);
		debug_WF_map(MAP, 0, 0);
		debug_sm_map(SPMAP, 0, 0);
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return 0;
}
void WF_update_position(uint16_t heading_0, int16_t heading_1) {
		float   pos_x, pos_y;
		int8_t	c, d, e;
		int16_t	x, y;
		uint16_t	path_heading;
		int32_t	i, j;

		if (heading_0 > heading_1 && heading_0 - heading_1 > 1800) {
				path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
		} else if (heading_1 > heading_0 && heading_1 - heading_0 > 1800) {
				path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
		} else {
				path_heading = (uint16_t)(heading_0 + heading_1) >> 1;
		}

		x = Map_GetXPos();
		y = Map_GetYPos();

		//Map_MoveTo(dd * cos(deg2rad(path_heading, 10)), dd * sin(deg2rad(path_heading, 10)));
		pos_x = robot::instance()->robot_get_position_x() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		pos_y = robot::instance()->robot_get_position_y() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		Map_SetPosition(pos_x, pos_y);
#if 0
#if (ROBOT_SIZE == 5 || ROBOT_SIZE == 3)

		if (x != Map_GetXPos() || y != Map_GetYPos()) {
				for (c = 1; c >= -1; --c) {
						for (d = 1; d >= -1; --d) {
								i = Map_GetRelativeX(path_heading, CELL_SIZE * c, CELL_SIZE * d);
								j = Map_GetRelativeY(path_heading, CELL_SIZE * c, CELL_SIZE * d);
								e = Map_GetCell(MAP, countToCell(i), countToCell(j));

								if (e == BLOCKED_OBS || e == BLOCKED_BUMPER || e == BLOCKED_BOUNDARY ) {
										Map_SetCell(MAP, i, j, CLEANED);
								}
						}
				}
		}

		//Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE, CELL_SIZE), Map_GetRelativeY(heading_0, -CELL_SIZE, CELL_SIZE), CLEANED);
		//Map_SetCell(MAP, Map_GetRelativeX(heading_0, 0, CELL_SIZE), Map_GetRelativeY(heading_0, 0, CELL_SIZE), CLEANED);
		i = Map_GetRelativeX(heading_0, 0, 0);
		j = Map_GetRelativeY(heading_0, 0, 0);
		if(WF_Push_Point(countToCell(i),countToCell(j)) == 0){//mark after letf the same cell
				Map_SetCell(MAP, i, j, CLEANED);
		}
		i = Map_GetRelativeX(heading_0, CELL_SIZE_3, 0);
		j = Map_GetRelativeY(heading_0, CELL_SIZE_3, 0);
		if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, i, j, BLOCKED_OBS);
		}

#else

		i = Map_GetRelativeX(path_heading, 0, 0);
		j = Map_GetRelativeY(path_heading, 0, 0);
		Map_SetCell(MAP, Map_GetRelativeX(heading_0, 0, CELL_SIZE), Map_GetRelativeY(heading_0, 0, CELL_SIZE), CLEANED);

		Map_SetCell(MAP, Map_GetRelativeX(heading_0, CELL_SIZE, 0), Map_GetRelativeY(heading_0, CELL_SIZE, 0), BLOCKED_OBS);

#endif
#endif
}
/**************************************************************
Function:WF_Check_Loop_Closed
Description:
 *1.push a point
 *2.check last cell if cleaned, cleaned->break, uncleaned->continue
 *3.check if in same cell, same->loop until leave this cell, not same-> marked last cell as cleaned
 ***************************************************************/
void WF_Check_Loop_Closed(uint16_t heading_0, int16_t heading_1) {
		float	pos_x, pos_y;
		int8_t	c, d, e;
		int16_t	x, y;
		uint16_t	path_heading;
		int32_t	i, j;
		int8_t	push_state;
		bool	reach_state;
		if (heading_0 > heading_1 && heading_0 - heading_1 > 1800) {
				path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
		} else if (heading_1 > heading_0 && heading_1 - heading_0 > 1800) {
				path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
		} else {
				path_heading = (uint16_t)(heading_0 + heading_1) >> 1;
		}

		x = Map_GetXPos();
		y = Map_GetYPos();

		//Map_MoveTo(dd * cos(deg2rad(path_heading, 10)), dd * sin(deg2rad(path_heading, 10)));
		pos_x = robot::instance()->robot_get_position_x() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		pos_y = robot::instance()->robot_get_position_y() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		Map_SetPosition(pos_x, pos_y);

#if (ROBOT_SIZE == 5 || ROBOT_SIZE == 3)

		if (x != Map_GetXPos() || y != Map_GetYPos()) {
				for (c = 1; c >= -1; --c) {
						for (d = 1; d >= -1; --d) {
								i = Map_GetRelativeX(path_heading, CELL_SIZE * c, CELL_SIZE * d);
								j = Map_GetRelativeY(path_heading, CELL_SIZE * c, CELL_SIZE * d);
								e = Map_GetCell(MAP, countToCell(i), countToCell(j));

								if (e == BLOCKED_OBS || e == BLOCKED_BUMPER || e == BLOCKED_BOUNDARY ) {
										Map_SetCell(MAP, i, j, CLEANED);
								}
						}
				}
		}

		//Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE, CELL_SIZE), Map_GetRelativeY(heading_0, -CELL_SIZE, CELL_SIZE), CLEANED);
		//Map_SetCell(MAP, Map_GetRelativeX(heading_0, 0, CELL_SIZE), Map_GetRelativeY(heading_0, 0, CELL_SIZE), CLEANED);
		i = Map_GetRelativeX(heading_0, 0, 0);
		j = Map_GetRelativeY(heading_0, 0, 0);
		push_state = WF_Push_Point(countToCell(i),countToCell(j));//push a cell
		if(push_state == 1){
				reach_state = WF_Is_Reach_Cleaned();//check this cell if reached
				if(reach_state == true){//add reach_count
						if (reach_count == 0){
								reach_continuous_state = true;
						}
						if(reach_continuous_state = true){
								reach_count++;
								ROS_INFO("reach_count = %d", reach_count);
						}
				}
				else{
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
		//Map_SetCell(MAP, Map_GetRelativeX(heading_0, CELL_SIZE, CELL_SIZE), Map_GetRelativeY(heading_0, CELL_SIZE, CELL_SIZE), CLEANED);

		//if (should_mark == 1) {
		//if (rounding_type == ROUNDING_LEFT) {
		i = Map_GetRelativeX(heading_0, CELL_SIZE_3, 0);
		j = Map_GetRelativeY(heading_0, CELL_SIZE_3, 0);
		if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, i, j, BLOCKED_OBS);
		}
		/*} else if (rounding_type == ROUNDING_RIGHT) {
		  i = Map_GetRelativeX(heading_0, -CELL_SIZE_3, 0);
		  j = Map_GetRelativeY(heading_0, -CELL_SIZE_3, 0);
		  if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
		  Map_SetCell(MAP, i, j, BLOCKED_OBS);
		  }
		 */
		//}
		//}

#else

		i = Map_GetRelativeX(path_heading, 0, 0);
		j = Map_GetRelativeY(path_heading, 0, 0);
		Map_SetCell(MAP, Map_GetRelativeX(heading_0, 0, CELL_SIZE), Map_GetRelativeY(heading_0, 0, CELL_SIZE), CLEANED);

		//if (should_mark == 1) {
		//if (rounding_type == ROUNDING_LEFT && Get_Wall_ADC(0) > 200) {
		//if (rounding_type == ROUNDING_LEFT) {
		Map_SetCell(MAP, Map_GetRelativeX(heading_0, CELL_SIZE, 0), Map_GetRelativeY(heading_0, CELL_SIZE, 0), BLOCKED_OBS);
		//Map_SetCell(MAP, Map_GetRelativeX(heading_0, CELL_SIZE, CELL_SIZE), Map_GetRelativeY(heading_0, CELL_SIZE, CELL_SIZE), BLOCKED_OBS);
		/*} else if (rounding_type == ROUNDING_RIGHT) {
		  Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE, 0), Map_GetRelativeY(heading_0, -CELL_SIZE, 0), BLOCKED_OBS);
		//Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE, -CELL_SIZE), Map_GetRelativeY(heading_0, -CELL_SIZE, -CELL_SIZE), BLOCKED_OBS);
		}*/
		//}
#endif
}

bool WF_Is_Reach_Cleaned(void){
		int32_t x,y;
		//x = Map_GetXPos();
		//y = Map_GetYPos();

		//CM_count_normalize(Gyro_GetAngle(0), 1 * CELL_SIZE_3, CELL_SIZE_3, &x, &y);
		CM_count_normalize(Gyro_GetAngle(0), 0,  0, &x, &y);
		//Map_SetCell(MAP, x, y, 6);
		//if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == CLEANED) {
		try{
				if(WF_Point.empty() == false){
						//if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == CLEANED) {
						if((WF_Point.size() - 1) >= 0){
								if (Map_GetCell(MAP, (WF_Point.at(WF_Point.size() - 1)).X, (WF_Point.at(WF_Point.size() - 1)).Y) == CLEANED) {//size() - 2 means last two
										ROS_INFO("Reach X = %d, Reach Y = %d",  countToCell(x), countToCell(y));
										Beep(3, 25, 25, 1);
										return true;
								}
								else{
										return false;
								}
						}
						else{
								return false;
						}
				}
				else{
						return false;
				}
		}
		catch(const std::out_of_range& oor){
				std::cerr << "Out of range error:" << oor.what() << '\n';
		}
		return false;
}
int8_t WF_Push_Point(int32_t x, int32_t y){
		if (WF_Point.empty() == false){
				if(WF_Point.back().X != x || WF_Point.back().Y != y){
						New_WF_Point.X = x;
						New_WF_Point.Y = y;
						WF_Point.push_back(New_WF_Point);
						ROS_INFO("WF_Point.X = %d, WF_Point.y = %d, size = %d", WF_Point.back().X, WF_Point.back().Y, WF_Point.size());
						return 1;
				}
				else{
						return 0;//it means still in the same cell
				}
		}
		else{
				New_WF_Point.X = x;
				New_WF_Point.Y = y;
				WF_Point.push_back(New_WF_Point);
				ROS_INFO("WF_Point.X = %d, WF_Point.y = %d, size = %d", WF_Point.back().X, WF_Point.back().Y, WF_Point.size());
				return 1;
		}
}
