/**
******************************************************************************
* @file    AI Cleaning Robot
* @author  ILife Team Dxsong
* @version V1.0
* @date    17-Nov-2011
* @brief   Random Path Cleaning Function
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "movement.h"
#include "spot.h"
#include "path_planning.h"
#include "map.h"
#include "gyro.h"
#include "wav.h"
#include <ros/ros.h>

#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed	18
#endif

#define SPOT_MAX_SPEED	(20)

#define SPOT_MODE "spot mode"

/* --------------------------------------------------Random Runnincg mode----------------------*/
void Spot_Mode(void)
{
	uint8_t Motor_OC_Counter = 0;
	uint16_t Radius = 0;
	uint8_t Move_Style = 1;
	uint8_t Spot_Flag = 0;
	uint8_t OBS_Counter = 0;
	uint8_t Stunk = 0;
	uint16_t Counter_Watcher = 0;

	Move_Style = Spiral_Right_Out;

	Reset_Touch();


	Set_LED(100,0);
	Set_SideBrush_PWM(60, 60);
	Set_MainBrush_PWM(90);
	Set_BLDC_Speed(90);

#ifdef BLDC_INSTALL
	usleep(10000);
	Set_VacMode(Vac_Max);
	Set_Vac_Speed();
#endif
	usleep(10000);
	Set_Dir_Right();
	Set_Wheel_Speed(25,5);
	Counter_Watcher=0;
	Reset_Rcon_Remote();
	Motor_OC_Counter=0;
	Reset_Wheel_Step();
	Reset_Work_Time();
	uint32_t step;
//	while (Get_LeftWheel_Step() < 6900 && ros::ok()) {
	while(ros::ok()){
		step =Get_LeftWheel_Step();
		if(step >=6900)break;
		usleep(40000);
		Counter_Watcher++;
		if (Counter_Watcher > 1000) {//about 200 seconds
			break;
		}
		Set_Dir_Right();
		Set_Wheel_Speed(25, 10);

		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if (Touch_Detect()) {
			Reset_Touch();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
		if (Check_Motor_Current()) {
			Motor_OC_Counter++;
			if (Motor_OC_Counter > 100) {
				Motor_OC_Counter = 0;
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return;
			}
		} else {
			Motor_OC_Counter = 0;
		}

		if (Remote_Key(Remote_All)) {
			if(Is_MoveWithRemote())
			{
				if (Remote_Key(Remote_Wall_Follow)) {
					Set_Clean_Mode(Clean_Mode_WallFollow);
					Reset_Rcon_Remote();
					//Move_Forward(10, 10);
					return;
				}
				/*
				if (Remote_Key(Remote_Random)) {
					Set_Clean_Mode(Clean_Mode_RandomMode);
					Reset_Rcon_Remote();
					//Move_Forward(10, 10);
					return;
				}*/
			}
			if (Remote_Key(Remote_Home)) {
				Set_MoveWithRemote();
				Set_Clean_Mode(Clean_Mode_GoHome);
				Move_Forward(10, 10);
				SetHomeRemote();
				Reset_Rcon_Remote();
				return;
			}
			Reset_Rcon_Remote();
		}
		if (Get_OBS_Status() || Get_Cliff_Trig()) {
			Move_Back();
			Stop_Brifly();
			Turn_Left(Turn_Speed, 2500);
			Move_Style = Spiral_Left_Out;
			break;
		}
	}

	Move_Forward(5, 5);
	Reset_Wheel_Step();
	Reset_Wall_Step();
	Set_MainBrush_PWM(90);
	Set_SideBrush_PWM(60, 60);
	Set_BLDC_Speed(90);
	Set_LED(100,0);
	Motor_OC_Counter = 0;

	while (ros::ok()) {
		usleep(1000);
		/*------------------------------------------------------Check Battery-----------------------*/
		if (Check_Bat_SetMotors(135000, 80000, 100000)) {	//Low Battery Event
			Display_Battery_Status(Display_Low);//min_distant_segment low
			usleep(30000);
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		//Set_MainBrush_PWM(80);
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if (Touch_Detect()) {
			Reset_Touch();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}

		if (Check_Motor_Current()) {
			Motor_OC_Counter++;
			if (Motor_OC_Counter > 100) {
				Motor_OC_Counter = 0;
				//Set_Clean_Mode(Clean_Mode_Userinterface);
				//return;
			}
		} else {
			Motor_OC_Counter = 0;
		}

		if (Remote_Key(Remote_All)) {
			if(Is_MoveWithRemote())
			{
				/*if (Remote_Key(Remote_Random)) {
					Set_Clean_Mode(Clean_Mode_RandomMode);
					Reset_Rcon_Remote();
					return;
				}*/
			}
			if (Remote_Key(Remote_Home)) {
				Set_MoveWithRemote();
				Set_Clean_Mode(Clean_Mode_GoHome);
				Move_Forward(10, 10);
				SetHomeRemote();
				Reset_Rcon_Remote();
				return;
			}
			Reset_Rcon_Remote();
		}
		/*------------------------------------------------------Runing Path-----------------------*/
		Set_Dir_Forward();
		switch (Move_Style) {
			case Spiral_Right_Out:
				step = Get_LeftWheel_Step();
				if (step > (Radius * 3)) {
					Reset_LeftWheel_Step();
					if (Radius < 100) {
						Radius += 1;
					} else {
						Radius += 3;
					}
					if (Radius > 250) {
						Move_Style = Spiral_Right_In;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_RightWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Left(Turn_Speed, 2500);
					Move_Style = Spiral_Left_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));
				break;

			case Spiral_Right_In:
				step = Get_LeftWheel_Step();
				if (step > (Radius * 3)) {
					Reset_LeftWheel_Step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 3;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {
					if (Get_RightWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Left(Turn_Speed, 2500);
					Move_Style = Spiral_Left_In;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case Spiral_Left_Out:
				step = Get_RightWheel_Step();
				if (step > (Radius * 3)) {
					Reset_RightWheel_Step();
					if (Radius < 100) {
						Radius += 1;
					} else {
						Radius += 3;
					}
					if (Radius > 250) {
						Move_Style = Spiral_Left_In;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case Spiral_Left_In:
				step = Get_RightWheel_Step();
				if (step > (Radius * 2)) {
					Reset_RightWheel_Step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 3;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_In;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			default:
				break;
		}
		if ((OBS_Counter > 15) || (Stunk > 3)) {
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if (Spot_Flag) {
			Spot_Flag = 0;
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
        if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right)){
            Disable_Motors();
            ROS_INFO("%s ,%d robot lift up\n",__FUNCTION__,__LINE__);
            wav_play(WAV_TAKEN_UP);
            Beep(1,5,25,1);
            usleep(20000);
        } 
	}
}

/*----------------------------------------------------------------Random Dirt Event---------------------------------*/
 
uint8_t Random_Dirt_Event(void)
{
	uint16_t Radius = 0;
	uint8_t Move_Style = 1;
	uint8_t Spot_Flag = 0;
	uint8_t OBS_Counter = 0;
	uint8_t Stunk = 0;
	uint8_t Flash_Counter = 0;
	uint16_t Watch_Counter = 0;
	uint8_t Flash_Flag = 0;
	uint8_t Motor_OC_Counter = 0;


	Move_Style = First_Round;

	Reset_Touch();

	Check_Bat_SetMotors(135000, 100000, 100000);

#ifdef BLDC_INSTALL
	Set_VacMode(Vac_Max);
	Set_Vac_Speed();
#endif

	Move_Forward(0, 0);
	Reset_Wheel_Step();
	Reset_Wall_Step();
	usleep(10000);

	Reset_Rcon_Remote();

	Motor_OC_Counter = 0;
	while (ros::ok()) {
		usleep(10000);
		Flash_Counter++;
		if (Flash_Counter > 20) {
			Watch_Counter++;
			if (Watch_Counter > 1000) {
				Set_Touch();
				return 1;
			}
			Flash_Counter = 0;
			Flash_Flag = 1 - Flash_Flag;
			if (Flash_Flag) {
				/*do led flash */
			} else {
				/*do led flash*/
			}
		}

		if (Remote_Key(Remote_All)) {
			//Main_Brush_PWM = MainBrush_Power;
			Move_Forward(30, 30);
			Reset_Rcon_Remote();
			return 0;
		}

		/*------------------------------------------------------Virtual Wall-----------------------*/
#ifdef VIRTUAL_WALL



#endif

		/*------------------------------------------------------Check Battery-----------------------*/
		if (Check_Bat_SetMotors(135000, 100000, 120000)) {	//Low Battery Event
			Move_Forward(30, 30);
			return 0;
		}

		if (Check_Motor_Current()) {
			Motor_OC_Counter++;
			if (Motor_OC_Counter > 50) {
				Motor_OC_Counter = 0;
				//Main_Brush_PWM = MainBrush_Power;
				Move_Forward(30, 30);
				return 0;
			}
		} else {
			Motor_OC_Counter = 0;
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if (Touch_Detect()) {
			return 1;
		}
		/*------------------------------------------------------Runing Path-----------------------*/

		switch (Move_Style) {
			case First_Round:
				if (Get_LeftWheel_Step() > 6000) {
					Move_Forward(0, 0);
					Reset_LeftWheel_Step();
					Move_Style = Spiral_Right_Out;
				}
				Set_Dir_Right();
				Set_Wheel_Speed(25, 10);

				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Set_LED(100,0);
					Turn_Left(Turn_Speed, 2500);
					Move_Forward(10, 10);
					Move_Style = Spiral_Left_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				break;

			case Spiral_Right_Out:
				if (Get_LeftWheel_Step() > (Radius * 3)) {
					Reset_LeftWheel_Step();
					if (Radius < 100) {
						Radius += 2;
					} else {
						Radius += 6;
					}
					if (Radius > 140) {
						Move_Style = Spiral_Right_In;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Set_LED(100,0);
					Turn_Left(Turn_Speed, 2500);
					Move_Style = Spiral_Left_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_Dir_Forward();
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case Spiral_Right_In:
				if (Get_LeftWheel_Step() > (Radius * 3)) {
					Reset_LeftWheel_Step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 6;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Left(Turn_Speed, 2500);
					Move_Style = Spiral_Left_In;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_Dir_Forward();
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case Spiral_Left_Out:
				if (Get_RightWheel_Step() > (Radius * 3)) {
					Reset_RightWheel_Step();
					if (Radius < 100) {
						Radius += 2;
					} else {
						Radius += 6;
					}
					if (Radius > 140) {
						Move_Style = Spiral_Left_In;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Set_LED(100,0);
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_Dir_Forward();
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case Spiral_Left_In:
				if (Get_RightWheel_Step() > (Radius * 2)) {
					Reset_RightWheel_Step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 6;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Set_LED(100,0);
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_In;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_Dir_Forward();
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			default:
				break;

		}
		if ((OBS_Counter > 5) || (Stunk > 2)) {
			//Main_Brush_PWM = MainBrush_Power;
			Move_Forward(30, 30);
			return 0;
		}
		if (Spot_Flag) {
			Spot_Flag = 0;
			//Main_Brush_PWM = MainBrush_Power;
			Move_Forward(30, 30);
			return 2;
		}
	}
	//return 2;
}

