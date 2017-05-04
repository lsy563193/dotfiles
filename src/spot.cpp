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

/* --------------------------------------------------Random Runnincg mode----------------------*/
void Spot_Mode(SpotType ST)
{
	uint8_t Motor_OC_Counter = 0;
	uint16_t Radius = 0;
	uint8_t Move_Style = 1;
	uint8_t Spot_Flag = 0;
	uint8_t OBS_Counter = 0;
	uint8_t Stunk = 0;
	uint16_t Counter_Watcher = 0;
	//these param for adjust spiral motion
	uint8_t sn=2;
    uint16_t rp=230;
	uint16_t ran = 250;
	if(ST == NormalSpot){
		sn = 3;
		rp = 230;
		ran = 400;
	}
	else if(ST == CleanSpot || ST == WallSpot){
		sn = 3;
		rp = 230; 
		ran = 200;
	}
	Move_Style = Spiral_Right_Out;

	Reset_Stop_Event_Status();

	if (!Is_Gyro_On())
	{
		// Restart the gyro.
		Set_Gyro_Off();
		// Wait for 30ms to make sure the off command has been effectived.
		usleep(30000);
		// Set gyro on before wav_play can save the time for opening the gyro.
		Set_Gyro_On();
		wav_play(WAV_CLEANING_SPOT);
		if (!Wait_For_Gyro_On())
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
	}
	else
	{
		wav_play(WAV_CLEANING_SPOT);
	}
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

		/*------------------------------------------------------stop event-----------------------*/
		if (Stop_Event()) {
//			Beep(5, 20, 0, 1);
			Stop_Brifly();
			if(ST == NormalSpot){
				Set_Clean_Mode(Clean_Mode_Userinterface);	
				wav_play(WAV_CLEANING_FINISHED);
			}
			// Key release detection, if user has not release the key, don't do anything.
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			Reset_Stop_Event_Status();
			return;
		}
		if (Check_Motor_Current()) {
			Motor_OC_Counter++;
			if (Motor_OC_Counter > 10) {
				Motor_OC_Counter = 0;
				if(ST == NormalSpot){
					Set_Clean_Mode(Clean_Mode_Userinterface);
					wav_play(WAV_CLEANING_FINISHED);
				}
				return;
			}
		} else {
			Motor_OC_Counter = 0;
		}
		if (Get_OBS_Status() || Get_Cliff_Trig()) {
			Move_Back();
			Stop_Brifly();
			Turn_Left(Turn_Speed, 2500);
			Move_Style = Spiral_Left_Out;
			break;
		}
		if(Get_Rcon_Remote())
		{
			if(ST == NormalSpot){
				if(Remote_Key(Remote_All)){
					Reset_Rcon_Remote();	
					Set_Clean_Mode(Clean_Mode_Userinterface);
					Disable_Motors();
					Stop_Brifly();
					wav_play(WAV_CLEANING_FINISHED);
					return;
				}
			}
			else if(ST == CleanSpot || ST == WallSpot){
					if (Remote_Key(Remote_Left | Remote_Right | Remote_Forward)){
						Reset_Rcon_Remote();
						return;
					}
					else if(Remote_Key(Remote_Home)){
						Set_MoveWithRemote();
						SetHomeRemote();
						Reset_Rcon_Remote();
						return ;
					}
			}
			Reset_Rcon_Remote();
		}
	}

	//Move_Forward(5, 5);
	Reset_Wheel_Step();
	Reset_Wall_Step();
	Set_MainBrush_PWM(90);
	Set_SideBrush_PWM(60, 60);
	Set_BLDC_Speed(90);
	Motor_OC_Counter = 0;

	while (ros::ok()) {
		usleep(10000);
		/*------------------------------------------------------Check Battery-----------------------*/
		if (Check_Bat_SetMotors(135000, 80000, 100000)) {	//Low Battery Event
			ROS_WARN("%s %d: Battery too low (< LOW_BATTERY_STOP_VOLTAGE)", __FUNCTION__, __LINE__);
			usleep(30000);
			if(ST == NormalSpot){
				Set_Clean_Mode(Clean_Mode_Userinterface);
				//wav_play(WAV_CLEANING_FINISHED);
			}
			break;
		}
		//Set_MainBrush_PWM(80);
		/*------------------------------------------------stop event-----------------------*/
		if (Stop_Event()) {
			Stop_Brifly();
			Disable_Motors();
			if(ST == NormalSpot){
				//wav_play(WAV_CLEANING_FINISHED);
				Set_Clean_Mode(Clean_Mode_Userinterface);
			}
			// Key release detection, if user has not release the key, don't do anything.
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			Reset_Stop_Event_Status();
			break;
		}
		uint8_t octype = Check_Motor_Current();
		if (octype) {
			if(Self_Check(octype) && (ST == NormalSpot)){
				Disable_Motors();
				Stop_Brifly();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
		}
		if (Get_Rcon_Remote()) {
			if(ST == NormalSpot){
				if(Remote_Key(Remote_All)){
					if(Get_Rcon_Remote() == Remote_Home){
						Set_MoveWithRemote();
						SetHomeRemote();
					}
					Disable_Motors();
					Stop_Brifly();
					Reset_Rcon_Remote();
					wav_play(WAV_CLEANING_FINISHED);
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return;
				}
			}
			else if(ST == CleanSpot || ST == WallSpot){
				if(Remote_Key(Remote_Home)){
					Reset_Rcon_Remote();
					Set_MoveWithRemote();
					SetHomeRemote();
					return;
				}
				else if(Remote_Key(Remote_Left | Remote_Right | Remote_Forward)){
					Reset_Rcon_Remote();	
					return;
				}
			}
			Reset_Rcon_Remote();
		}
		/*--------------------Runing Path-----------------------*/
		Set_Dir_Forward();
		switch (Move_Style) {
			case Spiral_Right_Out:
				step = Get_LeftWheel_Step();
				if (step > (Radius * sn)) {
					Reset_LeftWheel_Step();
					if (Radius < 100) {
						Radius += 1;
					} else {
						Radius += 3;
					}
					if (Radius > ran) {
						Move_Style = Spiral_Right_In;
						ROS_INFO("%s ,%d ,SPIRAL RIGHT IN",__FUNCTION__,__LINE__);
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
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + rp));
				break;

			case Spiral_Right_In:
				step = Get_LeftWheel_Step();
				if (step > (Radius * sn)) {
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
					ROS_INFO("%s ,%d ,SPIRAL LEFT IN",__FUNCTION__,__LINE__);
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + rp));

				break;

			case Spiral_Left_Out:
				step = Get_RightWheel_Step();
				if (step > (Radius * sn)) {
					Reset_RightWheel_Step();
					if (Radius < 100) {
						Radius += 1;
					} else {
						Radius += 3;
					}
					if (Radius > ran) {
						Move_Style = Spiral_Left_In;
						ROS_INFO("%s ,%d ,SPIRAL LEFT IN",__FUNCTION__,__LINE__);
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
					ROS_INFO("%s ,%d ,SPIRAL RIGHT OUT",__FUNCTION__,__LINE__);
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + rp));

				break;

			case Spiral_Left_In:
				step = Get_RightWheel_Step();
				if (step > (Radius * sn)) {
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
					}
					if(Get_Cliff_Trig()){
						Move_Back();
					}
					Stop_Brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_In;
					ROS_INFO("%s ,%d ,SPIRAL RIGHT IN",__FUNCTION__,__LINE__);
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + rp));

				break;

			default:
				break;
		}
		if ((OBS_Counter > 15) || (Stunk > 3)) {
			if(ST == NormalSpot)
				Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if (Spot_Flag) {
			Spot_Flag = 0;
			if(ST == NormalSpot)
				Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}

		if (Get_Cliff_Trig() == (Status_Cliff_All)) {
			Quick_Back(20,20);
			Stop_Brifly();
			if(Get_Cliff_Trig() == (Status_Cliff_All)){
				Quick_Back(20,20);
				Stop_Brifly();
			}
			if(Get_Cliff_Trig() == Status_Cliff_All){
				Quick_Back(20,20);
				Stop_Brifly();
				ROS_INFO("Cliff trigger three times ,robot lift up ");
				if(ST == NormalSpot)
					Set_Clean_Mode(Clean_Mode_Userinterface);
				Disable_Motors();
				wav_play(WAV_ERROR_LIFT_UP);
				break;
			}
			break;
		}
	}
	if(ST == NormalSpot)
		wav_play(WAV_CLEANING_FINISHED);
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

	Reset_Stop_Event_Status();

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
		/*------------------------------------------------------stop event-----------------------*/
		if (Stop_Event()) {
			Stop_Brifly();
			ROS_INFO("%s %d: Stop event!", __FUNCTION__, __LINE__);
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

