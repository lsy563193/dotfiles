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
#include "random_runing.h"
#include "movement.h"
#include "gyro.h"
#include "spot.h"
#include "config.h"
#include "wall_follow_short.h"
#include "spiral.h"
#include <ros/ros.h>

volatile uint8_t Half_Turn_Flag=0;

uint8_t Out_Trap_Right(void);
uint8_t Out_Trap_Left(void);
uint8_t Left_Bumper_Avoiding(void);
uint8_t Right_Bumper_Avoiding(void);
void Half_Turn_Left(uint16_t speed,uint16_t angle);
void Half_Turn_Right(uint16_t speed,uint16_t angle);

void Set_HalfTurn_Flag(void)
{
	Half_Turn_Flag=1;
}
void Reset_HalfTurn_Flag(void)
{
	Half_Turn_Flag=0;
}
uint8_t Is_HalfTurn_Flag(void)
{
	return Half_Turn_Flag;
}


/* --------------------------------------------------Random Runnincg mode----------------------*/
void Random_Running_Mode(void)
{
	volatile uint8_t Stunk = 0, Motor_Check_Code = 0, Temp_Cliff_Status = 0, Temp_OBS_Status = 0, Random_Factor = 0;
	volatile uint8_t Bumper_Counter=0,Wall_Bumper_Counter=0;
	volatile uint32_t Left_Wheel_Step_Buffer=0;
	uint32_t Moving_Speed=20;
	uint32_t Temp_Mobility_Distance=0;
	uint8_t Disacc=0;
	uint8_t Temp_Dirt_Status=0;
	uint8_t Low_Power_Counter=0;
	uint8_t Base_Wall_On=1;
	uint32_t Temp_Rcon_Status=0;
	uint8_t Temp_Bumper_Status=0;
	uint8_t Wall_Small_Counter = 0;
	uint8_t Wall_Mid_Counter = 0;
	uint8_t Wall_Bumper_Factor = 0;
	uint32_t OBS_Distance_Counter=0;
	uint8_t OBS_Cycle=0;
	uint8_t On_TrapOut_Flag=0;
	static uint8_t Vac_Mode_Buffer=0;
	static uint8_t N_H_T=0;
	static uint8_t Avoid_Flag=0;
//	static int32_t Wall_Sum_Value=0;
//	static int32_t Wall_Everage_Value=0;
//  static int32_t Wall_E_Counter=0;
//	static int32_t Temp_Wall_Buffer=0;
	volatile uint32_t OBS_Delay=0;

#ifdef VIRTUAL_WALL
	uint8_t Virtual_Wall_C = 0, Virtual_Wall_NG = 0;
#endif

	// Restart the gyro.
	Set_Gyro_Off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	Set_Gyro_On();
	if (!Wait_For_Gyro_On())
	{
		Set_Clean_Mode(Clean_Mode_Userinterface);
		return;
	}

	reset_start_work_time();
	Wall_Bumper_Factor = Get_Random_Factor()/15;
	Reset_MoveWithRemote();
//	Reset_Bumper_Error();
		Set_LED(100,0);
	Reset_Stop_Event_Status();

//  if(!Is_Dustbin_Install())
//  {
//    set_error_code(Error_Code_Dustbin);
//    Set_Clean_Mode(Clean_Mode_Userinterface);
//    Disable_Motors();
//    return;
//  }

	ROS_DEBUG_NAMED("random mode","-------in random running mode-----");
	if(is_on_charger_stub())
	{
		Set_Clean_Mode(Clean_Mode_Userinterface);
		Beep(2,25,25,1);
		usleep(400000);
		Reset_Rcon_Remote();
		Beep(2,25,25,1);
		Set_SideBrush_PWM(30,30);
		Set_MainBrush_PWM(0);
		Set_BLDC_Speed(30);
		Stop_Brifly();
		quick_back(30,750);
		if(Stop_Event()||is_charge_on())
		{
			Stop_Brifly();
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
		Beep(2,25,25,1);
		quick_back(30,750);
		if(Stop_Event())
		{
			Stop_Brifly();
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
		Beep(2,25,25,1);
		quick_back(30,750);
		if(Stop_Event())
		{
			Stop_Brifly();
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
		Beep(2,25,25,1);
		Turn_Right(Turn_Speed,1120+Get_Random_Factor()*10);
		if(Stop_Event())
		{
			Stop_Brifly();
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
		Stop_Brifly();
		//Initialize_Motor();
		Base_Wall_On=0;
	}

	Set_Clean_Mode(Clean_Mode_RandomMode);

	 work_motor_configure();
	Reset_Move_Distance();
	 reset_wheel_step();
	Reset_Stop_Event_Status();
	Wall_Bumper_Counter=0;
	Reset_Rcon_Remote();
	Set_Direction_Flag(Direction_Flag_Right);
	Stunk=0;
	Low_Power_Counter=0;
	 reset_rcon_status();

	Set_Vac_Speed();
	while(ros::ok())
	{
		usleep(10000);
		wall_dynamic_base(400);

#ifdef OBS_DYNAMIC
		OBS_Dynamic_Base(300);
#endif

		/*if(Get_Room_Mode())//small room
		{
			if(WorkFinish_ByRoom(Get_Room_Mode()))
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
		}*/

		/*-------------------------------------Mobility----------------------------------------------*/
		if(get_left_wheel_step()<500)
		{
			Temp_Mobility_Distance = Get_Move_Distance();
		}
		else
		{
			if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
			{
				Temp_Mobility_Distance = Get_Move_Distance();
				Check_Mobility();
				Reset_Mobility_Step();
			}
		}
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code= check_motor_current();
		if(Motor_Check_Code)
		{
			if(self_check(Motor_Check_Code))
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
			//Initialize_Motor();
		}
		/*------------------------------------------------------Check Battery-----------------------*/
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
		{
			Low_Power_Counter++;
			if(Low_Power_Counter>10)
			{
				ROS_WARN("%s %d: Battery too low (< LOW_BATTERY_STOP_VOLTAGE)", __FUNCTION__, __LINE__);
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
		}
		else
		{
			Low_Power_Counter=0;
		}
		/*------------------------------------------------------Stop event-----------------------*/
		if(Stop_Event())
		{
			Stop_Brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			Reset_Stop_Event_Status();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if(Get_Rcon_Remote() > 0)
		{
			#ifdef STANDARD_REMOTE
			if(Remote_Key(Remote_Left))
			{
				Stop_Brifly();
				Set_Dir_Left();
				set_wheel_speed(30, 30);
				//Set_SideBrush_PWM(60,60);
				usleep(100000);
				while(Remote_Key(Remote_Left))
				{
					Reset_Rcon_Remote();
					usleep(100000);
					if (Stop_Event())
					{
						Stop_Brifly();
						// Key release detection, if user has not release the key, don't do anything.
						while (Get_Key_Press() & KEY_CLEAN)
						{
							ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
							usleep(20000);
						}
						break;
					}
				}
				if (Stop_Event())
				{
					continue;
				}
				Stop_Brifly();
				Reset_Rcon_Remote();
				reset_wheel_step();
				Move_Forward(10,10);
			}
			if(Remote_Key(Remote_Right))
			{
				Stop_Brifly();
				Set_Dir_Right();
				set_wheel_speed(30, 30);
				//Set_SideBrush_PWM(60,60);
				usleep(100000);
				while(Remote_Key(Remote_Right))
				{
					Reset_Rcon_Remote();
					usleep(100000);
					if(Stop_Event())
					{
						Stop_Brifly();
						// Key release detection, if user has not release the key, don't do anything.
						while (Get_Key_Press() & KEY_CLEAN)
						{
							ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
							usleep(20000);
						}
						// The touch status should be handled by the main while loop of Random_Running_Mode.
						break;
					}
				}
				Stop_Brifly();
				Reset_Rcon_Remote();
				reset_wheel_step();
				Move_Forward(10,10);
			}
			#endif
			#ifdef SCREEN_REMOTE
			if(Remote_Key(Remote_Left))
			{
				Stop_Brifly();
				Turn_Left(Turn_Speed,500);
				Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(10,10);
			}
			if(Remote_Key(Remote_Right))
			{
				Stop_Brifly();
				Turn_Right(Turn_Speed,400);
				Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(10,10);
			}
			#ifdef BLDC_INSTALL
			if(Remote_Key(Remote_Max))
			{
				Switch_VacMode();
//			Stop_Brifly();
//			Turn_Right(Turn_Speed,1800);
//			Stop_Brifly();
				Reset_Rcon_Remote();
//			Reset_Wheel_Step();
//			Move_Forward(10,10);
			}
			#else
			if(Remote_Key(Remote_Max))
			{
				Stop_Brifly();
				Turn_Right(Turn_Speed,1800);
				Stop_Brifly();
				Reset_Rcon_Remote();
				reset_wheel_step();
				Move_Forward(10,10);
			}
			#endif
			#endif
			if(Remote_Key(Remote_Home)) //                                    Check Key Home
			{
				usleep(50000);
				Set_Clean_Mode(Clean_Mode_GoHome);
				set_home_remote();
				Reset_Rcon_Remote();
				return;
			}
//		if(Remote_Key(Remote_Random)) //                                    Check Key Home
//		{
//			Set_Clean_Mode(Clean_Mode_WallFollow);
//				Move_Forward(10,10);
//				Reset_Rcon_Remote();
//				break;
//		}
			if(Remote_Key(Remote_Spot))
			{
				Reset_Rcon_Remote();
				Vac_Mode_Buffer = Get_VacMode();
				Temp_Dirt_Status=Random_Dirt_Event();
				Set_VacMode(Vac_Mode_Buffer,false);
				Set_Vac_Speed();
				if(Temp_Dirt_Status==1)
				{
					// Stop_Event triggered in Random_Dirt_Event
					// Key release detection, if user has not release the key, don't do anything.
					while (Get_Key_Press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event status should be cleared.
					Set_Clean_Mode(Clean_Mode_Userinterface);
					break;
				}
				reset_wheel_step();
			}
			Reset_Rcon_Remote();
		}
		/*------------------------------------------------------Virtual wall Event-----------------------*/
#ifdef VIRTUAL_WALL
		if(Is_WorkFinish(Get_Room_Mode()))
		{
			if(Is_NearStation())
			{
				ROS_DEBUG("jump to the home mode in random running");
				Set_Clean_Mode(Clean_Mode_GoHome);
				break;
			}
		}
		else if(Base_Wall_On)
		{
			Temp_Rcon_Status = Get_Rcon_Status();
			if(Temp_Rcon_Status & 0x0f00)
			{
				Stop_Brifly();
				Stop_Brifly();
				if(get_left_wheel_step()<1000)
				{
					if(Is_Direction_Right())
					{
						Turn_Right(Turn_Speed,1200);
						Set_Direction_Flag(Direction_Flag_Right);
					}
					else
					{
						turn_left(Turn_Speed, 1200);
						Set_Direction_Flag(Direction_Flag_Left);
					}
				}
				else
				{
					if(Temp_Rcon_Status & RconL_HomeT)
					{
						Turn_Right(Turn_Speed,800);
						Set_Direction_Flag(Direction_Flag_Right);
					}
					else if(Temp_Rcon_Status & RconFL_HomeT)
					{
						Random_Back();
						Turn_Right(Turn_Speed,1120);
						Set_Direction_Flag(Direction_Flag_Right);
					}
					else if(Temp_Rcon_Status & RconFR_HomeT)
					{
						Random_Back();
						turn_left(Turn_Speed, 1120);
						Set_Direction_Flag(Direction_Flag_Left);
					}
					else if(Temp_Rcon_Status & RconR_HomeT)
					{
						turn_left(Turn_Speed, 1000);
						Set_Direction_Flag(Direction_Flag_Left);
					}
				}
				Move_Forward(2,2);
				reset_rcon_status();
				Base_Wall_On=0;
				reset_wheel_step();
				Wall_Small_Counter++;
				Wall_Mid_Counter++;
			}
		}
#endif
		/*------------------------------------------------------Virtual Wall--------------------*/
#ifdef VIRTUAL_WALL
		//Temp_Rcon_Status = Get_Rcon_Status();
		if (Is_VirtualWall()) {
			Reset_VirtualWall();
			Virtual_Wall_C++;
			if (Virtual_Wall_C > 1) {
				Stop_Brifly();
				//WalkAlongVirtualWall(Temp_Rcon_Status);
				Temp_Rcon_Status = 0;
			}
		} else {
			Virtual_Wall_NG++;
			if (Virtual_Wall_NG > 100) {
				Virtual_Wall_NG = 0;
				Virtual_Wall_C = 0;
			}
		}
#endif

		/*------------------------------------------------------Cliff Event-----------------------*/
		Temp_Cliff_Status= get_cliff_trig();
		if(Temp_Cliff_Status)
		{
			ROS_DEBUG("random running , cliff event!");
			set_wheel_speed(0, 0);
			set_dir_backward();
			usleep(30000);
			if(get_cliff_trig()||(get_left_wheel_step()<200))
			{
				Cliff_Move_Back();
				if(get_cliff_trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
					break;
				}
				if(get_cliff_trig())
				{
					if(cliff_escape())
					{
						Set_Clean_Mode(Clean_Mode_Userinterface);
						break;
					}
				}
				if(cliff_event(Temp_Cliff_Status))
				{
					Set_Direction_Flag(Direction_Flag_Left);
				}
				else
				{
					Set_Direction_Flag(Direction_Flag_Right);
				}
				reset_wheel_step();
				reset_temp_pwm();
				Stunk++;
				Bumper_Counter++;
				Wall_Bumper_Counter+=2;
				Wall_Small_Counter++;
				Wall_Mid_Counter++;
				Move_Forward(5,5);
				usleep(10000);
			}
			Reset_LeftWheel_Step();
			set_dir_forward();
		}
		/*------------------------------------------------------Bumper Event-----------------------*/
		/*-----------------left bumper ------------------------------------*/
		if(get_bumper_status()&LeftBumperTrig)
		{
			ROS_DEBUG("random running , left bumpe event!");
			Avoid_Flag=0;
			Left_Wheel_Step_Buffer= get_left_wheel_step();
			Add_Average(get_left_wheel_step());
			if(get_left_wheel_step()>14000)
			{
				Wall_Bumper_Counter+=2;
			}
			else
			{
				Wall_Bumper_Counter+=3;
			}
			set_wheel_speed(0, 0);
			usleep(10000);
			Temp_Bumper_Status = get_bumper_status();
			Random_Back();
			if(Is_Bumper_Jamed())break;

			Stunk++;
			if(Stunk>7)
			{
				Turn_Right(Turn_Speed,240);
				Move_Forward(10,10);
				if(Out_Trap_Left())
				{
					// Out_Trap_Left() return 1 may be caused by Stop_Event.
					// Key release detection, if user has not release the key, don't do anything.
					while (Get_Key_Press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event should be cleared.
					Set_Clean_Mode(Clean_Mode_Userinterface);
					Reset_Stop_Event_Status();
					break;
				}
				Stunk=0;
				Set_Direction_Flag(Direction_Flag_Right);
				Turn_Right(Turn_Speed,80);
			}
			else
			{
				if((Wall_Small_Counter>30)&&(!Is_Move_Finished(300000)))
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,400);
					Stop_Brifly();
					if(Wall_Follow_Short(3000))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Small_Counter=0;
					Wall_Mid_Counter=0;
					Reset_Move_Distance();
				}
				else if((Wall_Mid_Counter>40)||(Is_Move_Finished(460000)))
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,400);
					Stop_Brifly();
					if(Wall_Follow_Short(1000))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
					Reset_Move_Distance();
				}
				else if((Wall_Bumper_Counter>(Short_Wall_Trig+Wall_Bumper_Factor))&&(Get_Random_Factor()<25))
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,400);
					Stop_Brifly();
					if(Wall_Follow_Short(Get_Average_Move()))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
				}
				else
				{
					if(Left_Wheel_Step_Buffer<1000)
					{
						if(Is_Direction_Right())
						{
							Turn_Right(Turn_Speed,660);
							Set_Direction_Flag(Direction_Flag_Right);
						}
						else
						{
							turn_left(Turn_Speed, 660);
							Set_Direction_Flag(Direction_Flag_Left);
						}
					}
					else
					{
						if(Temp_Bumper_Status == AllBumperTrig)
						{
							//Half_Turn_Right(Turn_Speed,800+Get_Random_Factor()*7);
							Turn_Right(Turn_Speed,800+Get_Random_Factor()*7);
							Avoid_Flag=1;
						}
						else
						{
							if(Get_Random_Factor()<60)
							{
								Turn_Right(Turn_Speed,800);
								if(Left_Bumper_Avoiding())Avoid_Flag=1;
								if (Stop_Event())
								{
									// Continue to let the main while loop to process the stop event status.
									continue;
								}
							}
							else
							{
								//Half_Turn_Right(Turn_Speed,700+Get_Random_Factor()*9);
								Turn_Right(Turn_Speed,700+Get_Random_Factor()*9);
								Avoid_Flag=1;
							}
						}
						Set_Direction_Flag(Direction_Flag_Right);
					}
				}
			}
			if(!Avoid_Flag)
			{
				reset_temp_pwm();
				reset_wheel_step();
			}
			else
			{
				if(get_left_wheel_step()> get_right_wheel_step())
				{
					Set_RightWheel_Step(get_left_wheel_step());
				}
				Set_RightWheel_Step(get_right_wheel_step()/2);
			}
			Set_Mobility_Step(0);
			Bumper_Counter++;
			Wall_Small_Counter++;
			Wall_Mid_Counter++;
		}

/*---------------------------------------------------------Right Bumper ----------------------------------*/
		if(get_bumper_status()&RightBumperTrig)
		{
			ROS_DEBUG("random running ,right bumper event ");
			Avoid_Flag=0;
			Left_Wheel_Step_Buffer= get_left_wheel_step();
			Add_Average(get_left_wheel_step());
			if(get_left_wheel_step()>14000)
			{
				Wall_Bumper_Counter+=1;
			}
			else
			{
				Wall_Bumper_Counter+=2;
			}
			set_wheel_speed(0, 0);
			usleep(10000);
			Temp_Bumper_Status = get_bumper_status();
			Random_Back();
			if(Is_Bumper_Jamed())break;
			Stunk++;
			if(Stunk>7)
			{
				turn_left(Turn_Speed, 240);
				Move_Forward(10,10);
				if(Out_Trap_Right())
				{
					// Out_Trap_Right() return 1 may be caused by Stop_Event.
					// Key release detection, if user has not release the key, don't do anything.
					while (Get_Key_Press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event status should be cleared.
					Set_Clean_Mode(Clean_Mode_Userinterface);
					Reset_Stop_Event_Status();
					break;
				}
				Stunk=0;
				Set_Direction_Flag(Direction_Flag_Left);
				turn_left(Turn_Speed, 80);
			}
			else
			{
				if((Wall_Small_Counter>30)&&(!Is_Move_Finished(300000)))
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,900);
					Stop_Brifly();
					if(Wall_Follow_Short(4000))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Reset_Move_Distance();
					Wall_Small_Counter=0;
				}
				else if((Wall_Mid_Counter>40)||(Is_Move_Finished(460000)))
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,900);
					Stop_Brifly();
					if(Wall_Follow_Short(1000))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
					Reset_Move_Distance();
				}
				else if((Wall_Bumper_Counter>(Short_Wall_Trig+Wall_Bumper_Factor))&&(Get_Random_Factor()<25))
				{
					Stop_Brifly();
					Turn_Right(Turn_Speed,900);
					Stop_Brifly();
					if(Wall_Follow_Short(Get_Average_Move()))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
				}
				else
				{
					if(Left_Wheel_Step_Buffer<1000)
					{
						if(Is_Direction_Left())
						{
							turn_left(Turn_Speed, 660);
							Set_Direction_Flag(Direction_Flag_Left);
						}
						else
						{
							Turn_Right(Turn_Speed,660);
							Set_Direction_Flag(Direction_Flag_Right);
						}
					}
					else
					{
						if(Temp_Bumper_Status == AllBumperTrig)
						{
							//Half_Turn_Left(Turn_Speed,800+Get_Random_Factor()*6);
							turn_left(Turn_Speed, 800 + Get_Random_Factor() * 6);
							Avoid_Flag=1;
						}
						else
						{
							if(Get_Random_Factor()<60)
							{
								turn_left(Turn_Speed, 800);
								if(Right_Bumper_Avoiding())Avoid_Flag=1;
								if (Stop_Event())
								{
									// Continue to let the main while loop to process the stop event status.
									continue;
								}
							}
							else
							{
								//Half_Turn_Left(Turn_Speed,700+Get_Random_Factor()*8);
								turn_left(Turn_Speed, 700 + Get_Random_Factor() * 8);
								Avoid_Flag=1;
							}
						}
						Set_Direction_Flag(Direction_Flag_Left);
					}
				}
			}
			if(!Avoid_Flag)
			{
				reset_temp_pwm();
				reset_wheel_step();
			}
			else
			{
				if(get_left_wheel_step()> get_right_wheel_step())
				{
					Set_RightWheel_Step(get_left_wheel_step());
				}
				Set_RightWheel_Step(get_right_wheel_step()/2);
			}
			Set_Mobility_Step(0);
			Bumper_Counter++;
			Wall_Small_Counter++;
			Wall_Mid_Counter++;
		}

	/*------------------------------------------------------OBS_Status-----------------------*/
		if(Temp_OBS_Status)
		{
			ROS_DEBUG("random running ,obs event ");
			Temp_OBS_Status=Get_OBS_Status();
			Left_Wheel_Step_Buffer= get_left_wheel_step();
			Add_Average(get_left_wheel_step());
			//Random_Back();
			set_wheel_speed(0, 0);
			reset_temp_pwm();
			usleep(10000);
			//Stop_Brifly();
			N_H_T=0;
			Reset_HalfTurn_Flag();
			if(Is_LeftWheel_Reach(30))
			{
				if(Left_Wheel_Step_Buffer>14000)
				{
					Wall_Bumper_Counter+=1;
				}
				else
				{
					Wall_Bumper_Counter+=2;
				}
				Stop_Brifly();
				Bumper_Counter++;
			}

			if((Wall_Small_Counter>30)&&(!Is_Move_Finished(300000)))
			{
				if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
				{
					Turn_Right(Turn_Speed,900);
					Stop_Brifly();
				}
				Move_Forward(5,5);
				if(Wall_Follow_Short(4000))return;
				Stunk=0;
				Wall_Bumper_Counter=0;
				Wall_Small_Counter=0;
				Reset_Move_Distance();
				Wall_Mid_Counter=0;
			}
			else if((Wall_Mid_Counter>40)||(Is_Move_Finished(460000)))
			{
				if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
				{
					Turn_Right(Turn_Speed,900);
					Stop_Brifly();
				}
				Move_Forward(5,5);
				if(Wall_Follow_Short(1000))return;
				Stunk=0;
				Wall_Bumper_Counter=0;
				Wall_Mid_Counter=0;
				Wall_Small_Counter=0;
				Reset_Move_Distance();
			}
			else if((Wall_Bumper_Counter> (Short_Wall_Trig+Wall_Bumper_Factor))&&(Get_Random_Factor()<25))
			{
				if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
				{
					Turn_Right(Turn_Speed,900);
					Stop_Brifly();
				}
				Move_Forward(5,5);
				if(Wall_Follow_Short(Get_Average_Move()))return;
				Stunk=0;
				Wall_Bumper_Counter=0;
				Wall_Mid_Counter=0;
				Wall_Small_Counter=0;
			}
			else
			{
				Random_Back();
				//Stop_Brifly();
				Stunk++;
				if(get_bumper_status())
				{
					Random_Back();
					if(Is_Bumper_Jamed())break;
					Stop_Brifly();
				}
				if(Left_Wheel_Step_Buffer<1000)
				{
					if(Is_Direction_Left())
					{
						if(Stunk>10)
						{
							if(Out_Trap_Right())
							{
								// Out_Trap_Right() return 1 may be caused by Stop_Event.
								// Key release detection, if user has not release the key, don't do anything.
								while (Get_Key_Press() & KEY_CLEAN)
								{
									ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
									usleep(20000);
								}
								// Key relaesed, then the touch status and stop event status should be cleared.
								Set_Clean_Mode(Clean_Mode_Userinterface);
								Reset_Stop_Event_Status();
								break;
							}
							Stunk=0;
							turn_left(Turn_Speed, 240);
						}
						else
						{
							if(Left_Wheel_Step_Buffer<300)
							{
								turn_left(Turn_Speed - 10, 400);
							}
							else
							{
								turn_left(Turn_Speed, 400);
							}
						}
						N_H_T=1;
						Set_Direction_Flag(Direction_Flag_Left);
					}
					else
					{
						if(Stunk>10)
						{
							if(Out_Trap_Left())
							{
								// Out_Trap_Left() return 1 may be caused by Stop_Event.
								// Key release detection, if user has not release the key, don't do anything.
								while (Get_Key_Press() & KEY_CLEAN)
								{
									ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
									usleep(20000);
								}
								// Key relaesed, then the touch status and stop event status should be cleared.
								Set_Clean_Mode(Clean_Mode_Userinterface);
								Reset_Stop_Event_Status();
								break;
							}
							Stunk=0;
							Turn_Right(Turn_Speed,240);
						}
						else
						{
							if(Left_Wheel_Step_Buffer<300)
							{
								Turn_Right(Turn_Speed-10,400);
							}
							else
							{
								Turn_Right(Turn_Speed,400);
							}
						}
						N_H_T=1;
						Set_Direction_Flag(Direction_Flag_Right);
					}
				}
				else
				{
					Random_Factor=Left_Wheel_Step_Buffer%2;
					if(Temp_OBS_Status==0xA2)// LFR
					{
						if(Random_Factor)
						{
							//Half_Turn_Left(Turn_Speed,750+Get_Random_Factor()*8);
							turn_left(Turn_Speed, 750 + Get_Random_Factor() * 8);
							Set_Direction_Flag(Direction_Flag_Left);
						}
						else
						{
							//Half_Turn_Right(Turn_Speed,800+Get_Random_Factor()*8);
							Turn_Right(Turn_Speed,800+Get_Random_Factor()*8);

							Set_Direction_Flag(Direction_Flag_Right);
						}
					}
					else if(Temp_OBS_Status==0x72)//LF
					{
						if(Random_Factor)
						{
							//Half_Turn_Right(Turn_Speed,1200);
							Turn_Right(Turn_Speed,1200);
							Set_Direction_Flag(Direction_Flag_Right);
						}
						else
						{
							//Half_Turn_Left(Turn_Speed,1200);
							turn_left(Turn_Speed, 750 + Get_Random_Factor() * 8);
							Set_Direction_Flag(Direction_Flag_Left);
						}
					}
					else if(Temp_OBS_Status&0x02)//L
					{
						if(Stunk>10)
						{
							if(Out_Trap_Left())
							{
								// Out_Trap_Left() return 1 may be caused by Stop_Event.
								// Key release detection, if user has not release the key, don't do anything.
								while (Get_Key_Press() & KEY_CLEAN)
								{
									ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
									usleep(20000);
								}
								// Key relaesed, then the touch status and stop event status should be cleared.
								Set_Clean_Mode(Clean_Mode_Userinterface);
								Reset_Stop_Event_Status();
								break;
							}
							Stunk=0;
							//Half_Turn_Right(Turn_Speed,240);
							Turn_Right(Turn_Speed,240);


						}
						else
						{
							if((Bumper_Counter%3)==0)
								//Half_Turn_Right(Turn_Speed,800+Get_Random_Factor()*7);
								Turn_Right(Turn_Speed,800+Get_Random_Factor()*7);
							else
								//Half_Turn_Right(Turn_Speed,750+Get_Random_Factor()*8);
								Turn_Right(Turn_Speed,750+Get_Random_Factor()*8);

						}
						Set_Direction_Flag(Direction_Flag_Right);
					}
					else
					{
						if(Stunk>10)
						{
							if(Out_Trap_Right())
							{
								// Out_Trap_Right() return 1 may be caused by Stop_Event.
								// Key release detection, if user has not release the key, don't do anything.
								while (Get_Key_Press() & KEY_CLEAN)
								{
									ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
									usleep(20000);
								}
								// Key relaesed, then the touch status and stop event status should be cleared.
								Set_Clean_Mode(Clean_Mode_Userinterface);
								Reset_Stop_Event_Status();
								break;
							}
							Stunk=0;
							turn_left(Turn_Speed, 240);
						}
						else
						{
							Temp_OBS_Status>>=4;
							if(Temp_OBS_Status==0x08)
							{
								if(Random_Factor)
								{
									//Half_Turn_Left(Turn_Speed,750+Get_Random_Factor()*7);
									turn_left(Turn_Speed, 750 + Get_Random_Factor() * 7);
									Set_Direction_Flag(Direction_Flag_Left);
								}
								else
								{
									//Half_Turn_Right(Turn_Speed,750+Get_Random_Factor()*7);
									Turn_Right(Turn_Speed,750+Get_Random_Factor()*7);
									Set_Direction_Flag(Direction_Flag_Right);
								}
							}
							else if(Temp_OBS_Status>8)
							{
								//Half_Turn_Left(Turn_Speed,800+Get_Random_Factor()*7);
								turn_left(Turn_Speed, 800 + Get_Random_Factor() * 7);
								Set_Direction_Flag(Direction_Flag_Left);
							}
							else
							{
								if((Bumper_Counter%3)==0)
									//Half_Turn_Left(Turn_Speed,850+Get_Random_Factor()*7);
									turn_left(Turn_Speed, 850 + Get_Random_Factor() * 7);
								else
									//Half_Turn_Left(Turn_Speed,800+Get_Random_Factor()*7);
									turn_left(Turn_Speed, 800 + Get_Random_Factor() * 7);
								Set_Direction_Flag(Direction_Flag_Left);
							}
						}
					}
				}
			}
			OBS_Cycle = 0;
			On_TrapOut_Flag=0;
			if(Get_Rcon_Remote() <= 0)
			{
				if(Get_OBS_Status())
				{
					if(!N_H_T)
					{
						Stop_Brifly();
						Random_Back();
						Stop_Brifly();
					}

					do
					{
						OBS_Cycle++;
						Stunk++;
						if(OBS_Cycle>3)
						{
							Adjust_OBST_Value();
							Stunk=Stunk-OBS_Cycle;
							break;
						}
						if(Is_Direction_Left())
						{
							if((Stunk>10)||(OBS_Cycle>7))
							{
								Stunk = 0;
								On_TrapOut_Flag=1;
								break;
							}
							else
							{
								OBS_Turn_Left(Turn_Speed-5,400);
							}
							Set_Direction_Flag(Direction_Flag_Left);
						}
						else
						{
							if((Stunk>10)||(OBS_Cycle>7))
							{
								Stunk = 0;
								On_TrapOut_Flag=2;
								break;
							}
							else
							{
								OBS_Turn_Right(Turn_Speed-5,400);
							}
							Set_Direction_Flag(Direction_Flag_Right);
						}
						usleep(10000);
					}while(Get_OBS_Status());
					Reset_HalfTurn_Flag();
				}
			}
			if(On_TrapOut_Flag==1)
			{
				if(Out_Trap_Right())
				{
					// Out_Trap_Right() return 1 may be caused by Stop_Event.
					// Key release detection, if user has not release the key, don't do anything.
					while (Get_Key_Press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event status should be cleared.
					Set_Clean_Mode(Clean_Mode_Userinterface);
					Reset_Stop_Event_Status();
					break;
				}
				Turn_Right(Turn_Speed-5,240);
				Move_Forward(10,10);
				reset_temp_pwm();
			}
			else if(On_TrapOut_Flag==2)
			{
				if(Out_Trap_Left())
				{
					// Out_Trap_Left() return 1 may be caused by Stop_Event.
					// Key release detection, if user has not release the key, don't do anything.
					while (Get_Key_Press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event status should be cleared.
					Set_Clean_Mode(Clean_Mode_Userinterface);
					Reset_Stop_Event_Status();
					break;
				}
				Turn_Right(Turn_Speed-5,240);
				Move_Forward(10,10);
				reset_temp_pwm();
			}
			if(!Is_HalfTurn_Flag())
			{
				//Stop_Brifly();
				Move_Forward(0,0);
				reset_temp_pwm();
				//usleep(50000);
				reset_wheel_step();
				usleep(10000);
			}
			else
			{
				if(get_left_wheel_step()> get_right_wheel_step())
				{
					Set_RightWheel_Step(get_left_wheel_step());
				}
				Set_RightWheel_Step(get_right_wheel_step()/2);
			}
			Set_Mobility_Step(0);
			Wall_Small_Counter++;
			Wall_Mid_Counter++;
			Temp_OBS_Status=0;
		}
		/* -----------------------------Speed up ----------------------------------*/
		//if(Wall_Small_Counter>40)Wall_Small_Counter=0;
		if(Wall_Mid_Counter>70)
		{
			Wall_Mid_Counter=0;
			Reset_Move_Distance();
		}
		if(Is_LeftWheel_Reach(29000))
		{
			if(Spiral()){
				if (Stop_Event())
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
					Reset_Stop_Event_Status();
				}
				break;
			}
			reset_wheel_step();
			Set_HalfTurn_Flag();
		}
		if(Is_LeftWheel_Reach(24000))
		{
			Wall_Small_Counter=0;
		}
		if(Is_LeftWheel_Reach(2500))
		{
			Stunk=0;
//			Reset_Bumper_Error();
			if(Get_LeftBrush_Stall())set_left_brush_stall(0);
			if(Get_RightBrush_Stall())set_right_brush_stall(0);
		}
		else if(Is_LeftWheel_Reach(750))
		{
			Base_Wall_On=1;
			//Set_Left_Brush(ENABLE);
			//Set_Right_Brush(ENABLE);
		}

	/*-------------------------------------------------------------------------------------------------------------------*/
		if((get_cliff_trig()==0)&&(get_bumper_status()==0))
		{
			if(is_obs_near())
			{
				if(Moving_Speed>30)
				{
					Moving_Speed--;
				}
				Set_RightWheel_Step(400);
			}


			if(Get_OBS_Status())
			{
				if(Moving_Speed>10)
				{
					Disacc++;
					if(Disacc>3)
					{
						Disacc=0;
						Moving_Speed--;
					}
				}
				OBS_Distance_Counter++;
				OBS_Delay = Moving_Speed*OBS_Distance_Counter;
				if(OBS_Delay>1000)
				{
					OBS_Distance_Counter=0;
					Temp_OBS_Status = Get_OBS_Status();
				}
				Set_RightWheel_Step(200);
			}
			else
			{
				OBS_Distance_Counter=0;
				Temp_OBS_Status=0;
				Moving_Speed=(get_right_wheel_step()/80)+20;
				if(Is_HalfTurn_Flag())
				{
					Reset_HalfTurn_Flag();
					if(Moving_Speed<35)Moving_Speed=35;
				}
				else
				{
					if(Moving_Speed<25)Moving_Speed=25;
				}
				if(Moving_Speed>Max_Speed)Moving_Speed=Max_Speed;
			}
			Move_Forward(Moving_Speed,Moving_Speed);
		}
	}
}

/*------------------------------------------------------Out Trap Right--------------------------------------------------------*/
uint8_t Out_Trap_Right(void)
{
	int32_t R=0;
	uint8_t Motor_Check_Code=0;
	uint32_t Bump_Counter=0;
	reset_wheel_step();
	//Reset_Move_Distance();
	reset_rcon_status();
	reset_wall_step();
	while(ros::ok())
	{
		usleep(10000);
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code= check_motor_current();
		if(Motor_Check_Code)
		{
			if(self_check(Motor_Check_Code))
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 1;
			}
			//Initialize_Motor();
		}
		/*-------------------------------------------------------Wheel ---------------------------------------*/
		if(get_left_wall_step() - get_right_wall_step())
		{
			R= get_left_wall_step() - get_right_wall_step();
			if(R>7500)//turn over 3600 degree
			{
				return 0;
			}
		}
		/*------------------------------------------------------Stop event-----------------------*/
		if(Stop_Event())
		{
			Stop_Brifly();
			return 1;
		}

		if(Remote_Key(Remote_Left))
		{
			Turn_Right(Turn_Speed,240);
			Move_Forward(30,30);
			Reset_Rcon_Remote();
			return 0;
		}
		if(Get_Rcon_Status()&0x0f00)return 0;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			return 0;
		}
#endif

		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
		}
		if(get_bumper_status()&LeftBumperTrig)
		{
			Stop_Brifly();
			Wall_Move_Back();
			if(Is_Bumper_Jamed())return 1;
			turn_left(Turn_Speed - 5, 800);
			Stop_Brifly();
			Reset_LeftWheel_Step();
			Bump_Counter++;
		}

		if(get_bumper_status()&RightBumperTrig)
		{
			Stop_Brifly();
			Wall_Move_Back();
			if(Is_Bumper_Jamed())return 1;
			turn_left(Turn_Speed - 8, 150);
			Stop_Brifly();
			Reset_LeftWheel_Step();
			Bump_Counter++;
		}
		if(Bump_Counter>15)return 0;
		if(Is_Front_Close())
		{
			Stop_Brifly();
			turn_left(Turn_Speed - 8, 640);
			Stop_Brifly();
			Reset_LeftWheel_Step();
		}
		if(Is_LeftWheel_Reach(5000))return 0;
		if(get_right_wall_step()>12000)return 0;

		if(get_cliff_trig())
		{
			return 0;
		}
		if(get_left_wheel_step()<130)
		{
			Move_Forward(15,15);
		}
		else
		{
			Move_Forward(38,6);
		}
	}
}

/*------------------------------------------------------Out Trap Left--------------------------------------------------------*/
uint8_t Out_Trap_Left(void)
{
	int32_t R=0;
	uint8_t Motor_Check_Code=0;
	uint32_t Bump_Counter=0;
	reset_wheel_step();
	//Reset_Move_Distance();
	reset_rcon_status();
	reset_wall_step();
	while(ros::ok())
	{
		usleep(10000);
	/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code= check_motor_current();
		if(Motor_Check_Code)
		{
			if(self_check(Motor_Check_Code))
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return 1;
			}
		//Initialize_Motor();
		}
	/*-------------------------------------------------------Wheel ---------------------------------------*/
		if(get_right_wall_step()> get_left_wall_step())
		{
			R= get_right_wall_step()- get_left_wall_step();
			if(R>7500)//turn over 3600 degree
			{
				return 0;
			}
		}
		/*------------------------------------------------------Stop event-----------------------*/
		if(Stop_Event())
		{
			Stop_Brifly();
			return 1;
		}

		if(Remote_Key(Remote_Right))
		{
			Turn_Right(Turn_Speed,300);
			Move_Forward(30,30);
			Reset_Rcon_Remote();
			return 0;
		}
		if(Get_Rcon_Status()&0x0f00)return 0;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			return 0;
		}
#endif

		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
		}
		if(get_bumper_status()&RightBumperTrig)
		{
			Stop_Brifly();
			Wall_Move_Back();
			if(Is_Bumper_Jamed())return 1;
			Turn_Right(Turn_Speed-5,1025);
			Stop_Brifly();
			Reset_RightWheel_Step();
			Bump_Counter++;
		}

		if(get_bumper_status()&LeftBumperTrig)
		{
			Stop_Brifly();
			Wall_Move_Back();
			if(Is_Bumper_Jamed())return 1;
			Turn_Right(Turn_Speed-8,150);
			Stop_Brifly();
			Reset_RightWheel_Step();
			Bump_Counter++;
		}
		if(Bump_Counter>15)return 0;
		if(Is_Front_Close())
		{
			Stop_Brifly();
			Turn_Right(Turn_Speed-8,800);
			Stop_Brifly();
			Reset_RightWheel_Step();
		}
		if(Is_RightWheel_Reach(5000))return 0;
		if(get_left_wall_step()>12000)return 0;

		if(get_cliff_trig())
		{
			return 0;
		}
		if(get_right_wheel_step()<130)
		{
			Move_Forward(15,15);
		}
		else
		{
			Move_Forward(6,38);
		}
	}
}


uint8_t Left_Bumper_Avoiding(void)
{
	uint16_t Counter_Watcher=0;
	uint32_t Temp_A_Speed=0;
	//Stop_Brifly();
	reset_wheel_step();
	Move_Forward(5,20);
	while(get_right_wheel_step()<2000&&ros::ok())
	{
		usleep(100);
		Temp_A_Speed = get_right_wheel_step()/8 + 20;
		if(Temp_A_Speed<20)Temp_A_Speed=20;
		if(Temp_A_Speed>42)Temp_A_Speed=42;
		Move_Forward(Temp_A_Speed/4,Temp_A_Speed);

		Counter_Watcher++;
		if(Counter_Watcher>50000)
		{
			if(is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
				return 0;
			}
			return 0;
		}
		if(Stop_Event())
		{
			Stop_Brifly();
			return 0;
		}
		if(Get_Rcon_Remote() > 0)
		{
			Reset_Rcon_Remote();
			return 0;
		}
		if(get_bumper_status())break;
		if(Get_OBS_Status())break;
		if(get_cliff_trig())break;
		if((check_motor_current()==Check_Left_Wheel)||(check_motor_current()==Check_Right_Wheel))return 0;
		if(Get_Rcon_Status()&0x0f00)return 0;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			break;
		}
#endif

	}
	if(get_right_wheel_step()>=2000)return 1;
	Stop_Brifly();
	Stop_Brifly();
	return 0;
}

uint8_t Right_Bumper_Avoiding(void)
{
	uint16_t Counter_Watcher=0;
	uint32_t Temp_A_Speed=0;
	//Stop_Brifly();
	reset_wheel_step();
	Move_Forward(20,5);
	while(get_left_wheel_step()<2000)
	{
		usleep(100);
		Temp_A_Speed = get_left_wheel_step()/8 + 20;
		if(Temp_A_Speed<20)Temp_A_Speed=20;
		if(Temp_A_Speed>42)Temp_A_Speed=42;
		Move_Forward(Temp_A_Speed,Temp_A_Speed/4);
		Counter_Watcher++;
		if(Counter_Watcher>50000)
		{
			if(is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
				return 0;
			}
			return 0;
		}
		if(Stop_Event())
		{
			Stop_Brifly();
			return 0;
		}
		if(Get_Rcon_Remote() > 0)
		{
			Reset_Rcon_Remote();
			return 0;
		}
		if(get_bumper_status())break;
		if(Get_OBS_Status())break;
		if(get_cliff_trig())break;
		if((check_motor_current()==Check_Left_Wheel)||(check_motor_current()==Check_Right_Wheel))return 0;
		if(Get_Rcon_Status()&0x0f00)return 0;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			break;
		}
#endif

	}
	if(get_left_wheel_step()>=2000)return 1;
	Stop_Brifly();
	Stop_Brifly();
	return 0;
}

/*-------- Turn Left ------------------------*/
void Half_Turn_Left(uint16_t speed,uint16_t angle)
{
	uint8_t H_S=0;
	uint16_t Counter_Watcher=0;
	uint8_t Temp_H_Flag=0;
	turn_left(speed, angle / 2);
	if(Get_Rcon_Remote() > 0)
	{
		Reset_Rcon_Remote();
		return;
	}
	//set_dir_forward();
	//Set_LeftTPWM(0);

	reset_wheel_step();
	reset_temp_pwm();
	usleep(10000);
	Move_Forward(0,speed);
	Counter_Watcher=0;
	Reset_HalfTurn_Flag();
	ROS_DEBUG("half turn left angle :%d",angle);
	while(get_right_wheel_step()<angle && ros::ok())
	{
		usleep(100);
		Counter_Watcher++;
		if(Counter_Watcher>40000)
		{
			if(is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
			}
			return;
		}
		if(Is_Turn_Remote())Temp_H_Flag=1;
		if(get_bumper_status())Temp_H_Flag=1;
		if(Is_Front_Close())
		{
			set_wheel_speed(0, 20);
		}
		else
		{
			H_S = get_right_wheel_step()/8 + speed;
			if(H_S>42)H_S=42;
			set_wheel_speed(0, H_S);
		}
		if(get_cliff_trig())Temp_H_Flag=1;
		if(Stop_Event())
		{
			Temp_H_Flag=1;
		}
		if((check_motor_current()==Check_Left_Wheel)||(check_motor_current()==Check_Right_Wheel))Temp_H_Flag=1;
		if(Temp_H_Flag)break;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			break;
		}
#endif

	}
	if(Temp_H_Flag)
	{
		set_wheel_speed(0, 0);
		return;
	}
	Set_HalfTurn_Flag();
}

/*-------- Tur_Right ------------------------*/
void Half_Turn_Right(uint16_t speed,uint16_t angle)
{
	uint16_t H_S=0;
	uint16_t Counter_Watcher=0;
	uint8_t Temp_H_Flag=0;
	Turn_Right(speed,angle/2);
	if(Get_Rcon_Remote() > 0)
	{
		Reset_Rcon_Remote();
		return;
	}
//	set_dir_forward();
//	Set_TempPWM(20,0);
	//Set_RightTPWM(0);
	 reset_wheel_step();
	 reset_temp_pwm();
	usleep(10000);
	Move_Forward(speed,0);
  //set_wheel_speed(42,0);
	Counter_Watcher=0;
	Reset_HalfTurn_Flag();
	ROS_DEBUG("half turn right angle :%d",angle);
	while(get_left_wheel_step()<angle && ros::ok())
	{
		usleep(100);
		Counter_Watcher++;
		if(Counter_Watcher>40000)
		{
			if(is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
			}
			return;
		}
		if(Is_Turn_Remote())Temp_H_Flag=1;
		if(get_bumper_status())Temp_H_Flag=1;
		if(Is_Front_Close())
		{
			set_wheel_speed(20, 0);
		}
		else
		{
			H_S = get_left_wheel_step()/8 + speed;
			if(H_S>42)H_S=42;
			set_wheel_speed(H_S, 0);
		}
		if(get_cliff_trig())Temp_H_Flag=1;
		if(Stop_Event())
		{
			Temp_H_Flag=1;
		}
		if((check_motor_current()==Check_Left_Wheel)||(check_motor_current()==Check_Right_Wheel))Temp_H_Flag=1;
		if(Temp_H_Flag)break;

#ifdef VIRTUAL_WALL
		if (Is_VirtualWall()) {
			break;
		}
#endif

	}
	if(Temp_H_Flag)
	{
		set_wheel_speed(0, 0);
		return;
	}
	Set_HalfTurn_Flag();
}

