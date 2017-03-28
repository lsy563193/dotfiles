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
//Turn speed
#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed	18
#endif

volatile int32_t Map_Wall_Follow_Distance = 0;

bool	escape_thread_running = false;
//Timer
uint32_t escape_trapped_timer;

//MFW setting
static const MapWallFollowSetting MFW_Setting[6]= {{1200, 250, 150 },
							{1200, 250, 150},
							{1200, 250, 150},
							{1200, 250, 70},
							{1200, 250, 150},
							{1200, 250, 150},};

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

	pos_x = robot::instance()->robot_get_position_x();
	pos_y = robot::instance()->robot_get_position_y();
	while (ros::ok()) {
		distance = sqrtf(powf(pos_x - robot::instance()->robot_get_position_x(), 2) + powf(pos_y - robot::instance()->robot_get_position_y(), 2));
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

#ifdef OBS_DYNAMIC
		//if (Get_Bumper_Status() || Is_FrontOBS_Trig()) {
		if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value()) | Get_Cliff_Trig()) {
		
#else
		if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value()) | Get_Cliff_Trig()) {
#endif
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

			if (robot::instance()->robot_get_wall() > (Wall_Low_Limit)) {
				Wall_Distance = robot::instance()->robot_get_wall()  / 3;
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
			Left_Wall_Buffer[0] = robot::instance()->robot_get_wall();
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
#ifdef OBS_DYNAMIC
			if (Get_FrontOBS() < Get_FrontOBST_Value()) {
#else
			if (Get_FrontOBS() < MFW_Setting[follow_type].front_obs_val) {
#endif

				Proportion = robot::instance()->robot_get_wall();

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

	uint8_t		Temp_Counter = 0;
	uint16_t	i = 0;

	int			ret;
	int16_t		Left_Wall_Buffer[3] = {0};
	int32_t		Proportion = 0, Delta = 0, Previous = 0, R = 0;

	volatile int32_t		Wall_Straight_Distance = 100, Left_Wall_Speed = 0, Right_Wall_Speed = 0;
	static volatile int32_t	Wall_Distance = Wall_High_Limit;
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

	
	while (1) {
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

#ifdef OBS_DYNAMIC
		//if (Get_Bumper_Status() || Is_FrontOBS_Trig()) {
		if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value()) | Get_Cliff_Trig()) {
		
#else
		if (Get_Bumper_Status()||(Get_FrontOBS() > Get_FrontOBST_Value()) | Get_Cliff_Trig()) {
#endif
			printf("%s %d: Check: Get_Bumper_Status! Break!\n", __FUNCTION__, __LINE__);
			break;
		}

		usleep(10000);
	}

	//CM_HeadToCourse(Rotate_TopSpeed, Gyro_GetAngle(0) + 900);

	/* Set escape trapped timer when it is in Map_Wall_Follow_Escape_Trapped mode. */
	escape_trapped_timer = time(NULL);
	while (ros::ok()) {
		/*
		if ((time(NULL) - escape_trapped_timer) > ESCAPE_TRAPPED_TIME || escape_thread_running == false) {
			printf("%s %d: quit due to %s\n", __FUNCTION__, __LINE__, escape_thread_running == false ? "thread exit" : "timeout");
			break;
		}
		*/

#ifdef OBS_DYNAMIC

		OBS_Dynamic_Base(100);

#endif
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
                                if (Remote_Key(Remote_Random)) {
                                        Set_Clean_Mode(Clean_Mode_RandomMode);
                                        break;
                                }
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
				Display_Battery_Status(Display_Low);//max_distant_angle low
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

			if (robot::instance()->robot_get_wall() > (Wall_Low_Limit)) {
				Wall_Distance = robot::instance()->robot_get_wall()  / 3;
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
			Left_Wall_Buffer[0] = robot::instance()->robot_get_wall();
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
#ifdef OBS_DYNAMIC
			if (Get_FrontOBS() < Get_FrontOBST_Value()) {
#else
			if (Get_FrontOBS() < MFW_Setting[follow_type].front_obs_val) {
#endif

				Proportion = robot::instance()->robot_get_wall();

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
