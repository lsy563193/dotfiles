/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   UserInterface Fuction
	           Display Button lights and waiting for user to select cleaning mode
						 Plan setting , set hours and minutes
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "movement.h"
#include "user_interface.h"
#include <ros/ros.h>
#include "config.h"
#include "wav.h"
#include "robot.hpp"
//extern volatile ADC_Value_Struct ADC_Value;


#define USER_INTERFACE "user interface"
/*------------------------------------------------------------User Interface ----------------------------------*/
void User_Interface(void)
{
	static volatile uint8_t Press_time=0;
	static volatile uint8_t Temp_Mode=0;
	static volatile uint16_t Error_Show_Counter=400;
	static volatile uint16_t TimeOutCounter=0;
	
	uint16_t Temp_Battery_Voltage=1600;
	uint8_t BTA_Power_Dis=0;

#ifdef STANDARD_DISPLAY
	uint8_t LED_Add=0;
	uint8_t LED_Brightness=0;
	uint8_t Display_Counter=0;
#endif

#ifdef ONE_KEY_DISPLAY
	uint8_t ONE_Display_Counter=20;
#endif

	Press_time=0;
	Temp_Mode=0;
	Error_Show_Counter=400;
	TimeOutCounter=0;

	Disable_Motors();
//	Beep(3,25,25,1);
	usleep(600000);

//	Reset_Encoder_Error();

	Reset_Rcon_Remote();

	Reset_Touch();
	//Enable_PPower();
	//Disable_Motors();
	Reset_Rcon_Status();
//	Clear_Clcok_Receive();
	// Reset touch to avoid previous touch leads to directly go to navigation mode.
	Reset_Touch();
//	ResetHomeRemote();
	Set_VacMode(Vac_Normal);

#if CONTINUE_CLEANING_AFTER_CHARGE
	if (!robot::instance()->Is_Cleaning_Paused())
	{
		Set_Gyro_Off();
	}
#endif
	while(ros::ok())
	{
		usleep(2000);	
		if(Remote_Key(Remote_Forward | Remote_Right | Remote_Left | Remote_Backward))
		{
			Set_Clean_Mode(Clean_Mode_Remote);
			Reset_Rcon_Remote();
			return;
	  	}
		/*
		if (!Is_Gyro_On()){
			Set_Gyro_On();
			Set_Gyro_Status();
			usleep(2000000);
			//printf("Gyro_Status%d\n", Is_Gyro_On());
		}*/
		/*
		if(remote_cmd == Remote_Right|| remote_cmd ==Remote_Left||remote_cmd == Remote_Forward)
	  	{
			Set_Clean_Mode(Clean_Mode_Remote);
			return;
	  	}*/


		#ifdef SCREEN_REMOTE
//		if(Remote_Clock_Received())
//			Set_Remote_Schedule();
		#endif
		/*--------------------------------------------------------Check if on the station--------------*/
		if(Is_ChargerOn())
		{
			Set_Clean_Mode(Clean_Mode_Charging);
			break;
		}
		else
		{
			/* -----------------------------Check if spot event ----------------------------------*/
			if(Remote_Key(Remote_Spot))//                                       Check Remote Key Spin
			{
				//Transmite_BAT();
			 	Set_MoveWithRemote();
				Reset_Rcon_Remote();
				Temp_Mode=Clean_Mode_Spot;
			}

			/* -----------------------------Check if Home event ----------------------------------*/
			if(Remote_Key(Remote_Home)) //                                    Check Key Home
			{
				Set_LED(100,100);
			//	Press_time=Get_Key_Time(KEY_HOME);
				Temp_Mode=Clean_Mode_GoHome;
			//	Reset_MoveWithRemote();
				Set_MoveWithRemote();
				SetHomeRemote();
				Reset_Rcon_Remote();
			}
			/* -----------------------------Check if Random event ----------------------------------*/
			/*
			if(Remote_Key(Remote_Random))//                                  Check Remote Key Random
			{
			  	Set_MoveWithRemote();
				Reset_Rcon_Remote();
				Temp_Mode=Clean_Mode_RandomMode;
			}*/
			/* -----------------------------Check if wall follow event ----------------------------------*/
			if(Remote_Key(Remote_Wall_Follow))//                                  Check Remote Key Wallfollow
			{
			  	Set_MoveWithRemote();
				Reset_Rcon_Remote();
				Temp_Mode=Clean_Mode_WallFollow;
			}
		}
		/* -----------------------------Check if Clean event ----------------------------------*/
//		if(Is_Alarm())
//		{
//			Reset_Alarm();
//			if(Get_AlarmSet_Minutes()==Get_Time_Minutes())
//			{
//				Temp_Mode=Clean_Mode_Navigation;
//				Reset_MoveWithRemote();
//			}
//		}
		if(Remote_Key(Remote_Clean))//                                       Check Remote Key Clean
		{
			Reset_Rcon_Remote();
			Temp_Mode=Clean_Mode_Navigation;
			Reset_MoveWithRemote();
		}
		if(Get_Key_Press() & KEY_CLEAN)//                                    Check Key Clean
		{
			Set_LED(100,0);
			//Beep(2,25,25,2);
			//TX_D();
			Press_time=Get_Key_Time(KEY_CLEAN);
			// Long press on the clean button means let the robot go to sleep mode.
			if(Press_time>20)
			{
				ROS_INFO("%s %d: Long press and go to sleep mode.", __FUNCTION__, __LINE__);
				Beep(6,25,25,2);
				// Wait for user to release the key.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_INFO("User still holds the key.");
					usleep(100000);
				}
				// Key relaesed, then the touch status should be cleared.
				Reset_Touch();
				Set_LED(0,0);
			  	Temp_Mode=Clean_Mode_Sleep;
			}
			else
			{
				Beep(4,25,25,2);
				Temp_Mode=Clean_Mode_Navigation;
				Reset_Work_Time();
			}
			Reset_MoveWithRemote();
		//	Reset_Error_Code();
		}

		/* ----------------------------- ----------------------------------*/
		if(Temp_Mode)
		{
			//Reset_Error_Code();
			if(Is_ChargerOn())
			{
			  	if(!Is_AtHomeBase())
				{
					Temp_Mode=0;
				}
			}
			if((Temp_Mode==Clean_Mode_GoHome)||(Temp_Mode==Clean_Mode_Sleep))
			{
//				Reset_Bumper_Error();
//				Reset_Error_Code();
				Set_Clean_Mode(Temp_Mode);
//				Set_CleanKeyDelay(0);
				return;
			}
			if((Temp_Mode==Clean_Mode_WallFollow)||(Temp_Mode==Clean_Mode_Spot)||(Temp_Mode==Clean_Mode_RandomMode)||(Temp_Mode==Clean_Mode_Navigation))
			{
				//ROS_INFO("[user_interface.cpp] GetBatteryVoltage = %d.", GetBatteryVoltage());
				if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
				{
//					Set_Error_Code(Error_Code_Cliff);
//					Error_Show_Counter=400;
					ROS_INFO("%s %d: Cliff triggered, can't change mode.", __FUNCTION__, __LINE__);
					Temp_Mode=0;
				}
				//else if(GetBatteryVoltage() < 1520)
				else if(GetBatteryVoltage() < 1320)
				{
					ROS_DEBUG_NAMED(USER_INTERFACE,"BATTERY VOLTAGE LOW!");
					ROS_INFO("BATTERY VOLTAGE LOW!");
					Set_LED(0,0);
					Display_Battery_Status(Display_Low);
					Beep(6,25,25,-1);
					Temp_Mode=0;
				}
				else
				{
//					Reset_Error_Code();
//					Set_LED(100,0);
//					if(Get_Room_Mode()==Room_Mode_Auto)
//					{
//						Set_LED(100,0);
//						Beep(1,25,25,2);
//						Set_LED(0,0);
//						Beep(3,25,25,2);
//						Set_LED(100,0);
//						Beep(5,25,25,2);
//					}
//					else
//					{
//	          			Set_LED(100,0);
//						Beep(5,25,25,2);
//						Beep(3,25,25,2);
//					}
					if(!Is_ChargerOn()&&(Temp_Mode!=Clean_Mode_Navigation)){
						Initialize_Motor();
						ROS_DEBUG_NAMED(USER_INTERFACE," init motors.");
					}
					Set_Clean_Mode(Temp_Mode);
//					Set_CleanKeyDelay(0);
					Reset_Rcon_Remote();
//					Reset_Bumper_Error();
					return;
				}
			}
			Disable_Motors();
			Temp_Mode=0;
		}

		if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right)) {
			Disable_Motors();
			ROS_INFO("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
			wav_play(WAV_ERROR_LIFT_UP);
//			Beep(1, 10, 25, 1);
			usleep(20000);
		}

		Error_Show_Counter++;
	  	if(Error_Show_Counter>500)
	  	{
//			Test_Mode_Flag=0;
			Error_Show_Counter=0;
//			Sound_Out_Error(Get_Error_Code());
		}

#ifdef ONE_KEY_DISPLAY
		usleep(8000);

		//ROS_INFO("One key min_distant_segment logic. odc = %d", ONE_Display_Counter);
		ONE_Display_Counter++;
		if(ONE_Display_Counter>99)
		{
			ONE_Display_Counter=0;
			TimeOutCounter++;
			if(TimeOutCounter>15)
			{
				TimeOutCounter=0;
				Set_Clean_Mode(Clean_Mode_Sleep);
				break;
			}
			if(TimeOutCounter>0)//on base but miss charging , adjust position to charge
			{
				//ROS_INFO("TimeOutCounter = %d.", TimeOutCounter);
				if(Is_AtHomeBase())
				{
					ROS_DEBUG_NAMED(USER_INTERFACE,"At home base.");
					if(Get_Cliff_Trig()==0)
					{
						ROS_DEBUG_NAMED(USER_INTERFACE,"Cliff not triggered.");
						if(Turn_Connect())
						{
							ROS_DEBUG_NAMED(USER_INTERFACE,"Turn_connect pass.");
							Set_Clean_Mode(Clean_Mode_Charging);
							break;
						}
						Disable_Motors();
					}
				}
			}
		}
		
		Temp_Battery_Voltage = GetBatteryVoltage();
		
		if(Temp_Battery_Voltage<1350)
		{
			BTA_Power_Dis=1;
		}
		if(Temp_Battery_Voltage>1380)
		{
			BTA_Power_Dis=0;
		}
		
//		if(Get_Error_Code())//min_distant_segment Error = red led full
//		{
//			Set_LED(0,100);
//		}
		if(BTA_Power_Dis)//min_distant_segment low battery = red & green
		{
			Set_LED(ONE_Display_Counter,ONE_Display_Counter);
		}
		else
		{
			Set_LED(ONE_Display_Counter,0);//min_distant_segment normal green
		}

#endif

#ifdef STANDARD_DISPLAY
		{
			Display_Counter++;
			if(Display_Counter>0x000a)
		  	{
				if(Temp_Charge_CleangKey_Delay<1)Temp_Charge_CleangKey_Delay=1;
			 	Temp_Charge_CleangKey_Delay--;
				//Set_CleanKeyDelay(Temp_Charge_CleangKey_Delay);
			 	TimeOutCounter++;
				if(TimeOutCounter>160)
				{
			  		TimeOutCounter=0;
				  	Set_Clean_Mode(Clean_Mode_Sleep);
				  	break;
				}
				Display_Counter=0;
			 	if(flag)
				{
					LED_Add++;
					if(LED_Add>1)
					{
						LED_Add=0;
					  	LED_Brightness++;
						if(LED_Brightness>=7)flag=0;
					}
				}
				else
				{
					if(LED_Brightness>0)
					{
					  LED_Brightness--;
					}
					if(LED_Brightness==0)
					{
						 LED_Add++;
						if(LED_Add>1)
						{
						  	LED_Add=0;
					    	flag=1;
						}
					}
				}
//				if(Get_Error_Code())
//				{
//				  if((LED_Brightness==0)&&(LED_Add==0))Display_Content(LED_Exclamation,LED_Error,100,0,LED_Brightness);
//				  else Display_Content(LED_Spot|LED_Plan|LED_Clean|LED_Home|LED_Clock|LED_Exclamation,LED_Error,Get_Error_Code(),0,LED_Brightness);
//				}
				else
				{
          			if(GetBatteryVoltage() < 1350)
          			{
            			Display_Low_BTA(LED_Brightness);
          			}
          			else
          			{
  				  		if((LED_Brightness==0)&&(LED_Add==0)){
							//Display_Content(0,100,100,0,LED_Brightness);
							Set_LED(LED_Brightness,LED_Brightness);
						}
  						else{
							//Display_Content(LED_Spot|LED_Plan|LED_Clean|LED_Home|LED_Clock,100,100,0,LED_Brightness);
							Set_LED(LED_Brightness,LED_Brightness);
						}
          			}
				}
			}
		}
#endif

	}
//	Set_CleanKeyDelay(0);
}




