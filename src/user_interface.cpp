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
	Beep(3,25,25,1);
	ROS_DEBUG_NAMED(USER_INTERFACE,"in user interface mode");
	usleep(600000);

//	Reset_Encoder_Error();

	Reset_Rcon_Remote();

	//Enable_PPower();
	//Disable_Motors();
	Reset_Rcon_Status();
//	Clear_Clcok_Receive();
//	Set_Room_Mode(Room_Mode_Large);
//	ResetHomeRemote();
	Set_VacMode(Vac_Normal);
	while(ros::ok())
	{
		uint32_t remote_cmd = Get_Rcon_Remote();
		if(remote_cmd == Remote_Forward|| remote_cmd == Remote_Right
		    || remote_cmd ==Remote_Left||remote_cmd == Remote_Max)
	  	{
			Set_Clean_Mode(Clean_Mode_Remote);
			return;
	  	}


		#ifdef SCREEN_REMOTE
//		if(Remote_Clock_Received())
//			Set_Remote_Schedule();
		#endif
		/*--------------------------------------------------------Check if on the station--------------*/
		if(Is_ChargerOn())
		{
//		  	if(Get_CleanKeyDelay()<1)
//	  		{
//				Set_Clean_Mode(Clean_Mode_Charging);
//				break;
//			}
		}
		else
		{
			/* -----------------------------Check if spot event ----------------------------------*/
			if(Remote_Key(Remote_Spot))//                                       Check Remote Key Spin
			{
				//Transmite_BAT();
			 	Set_MoveWithRemote();
				Temp_Mode=Clean_Mode_Spot;
			}

			/* -----------------------------Check if Home event ----------------------------------*/
			if(Remote_Key(Remote_Home)) //                                    Check Key Home
			{
				Set_LED(100,100);
			//	Press_time=Get_Key_Time(KEY_HOME);
				Temp_Mode=Clean_Mode_GoHome;
				Reset_MoveWithRemote();
				SetHomeRemote();
			}
			/* -----------------------------Check if wall follow event ----------------------------------*/
			if(Remote_Key(Remote_Random))//                                  Check Remote Key Wallfollow
			{
			  	Set_MoveWithRemote();
				//Temp_Mode=Clean_Mode_WallFollow;
				Temp_Mode=Clean_Mode_RandomMode;
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
//			Set_Room_Mode(Room_Mode_Large);
			Press_time=10;
			while(Press_time--)
			{
//				if(Remote_Key(Remote_Clean))
//				{
//					Set_Room_Mode(Room_Mode_Auto);
//					break;
//				}
				usleep(50000);
			}
     		Temp_Mode=Clean_Mode_Navigation;
			Reset_Rcon_Remote();
			Reset_MoveWithRemote();
		}
		if(Get_Key_Press()==KEY_CLEAN)//                                    Check Key Clean
		{
			Set_LED(100,0);
			Beep(2,25,25,2);
			//TX_D();
		  	Press_time=Get_Key_Time(KEY_CLEAN);
			if(Press_time>100)
			{
				Beep(3,25,25,2);
				Set_LED(0,0);
			  	Temp_Mode=Clean_Mode_Sleep;
			}
			else
			{
				//Set_Room_Mode(Room_Mode_Large);
				Press_time=5;
				while(Press_time--)
				{
					if(Get_Key_Press()==KEY_CLEAN)
					{
						Beep(2,25,25,2);
//						Set_Room_Mode(Room_Mode_Auto);
						break;
					}
					usleep(50000);
				}
			  	Temp_Mode=Clean_Mode_Navigation;
        		Reset_Work_Timer_Start();
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
				if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
				{
//					Set_Error_Code(Error_Code_Cliff);
					Error_Show_Counter=400;
					Temp_Mode=0;
				}
				else if(GetBatteryVoltage() < 1300)
				{

					ROS_DEBUG_NAMED(USER_INTERFACE,"BATTERY VOLTAGE LOW!");
					Set_LED(0,0);
					Display_Battery_Status(Display_Low);
					Beep(6,25,25,2);
					Beep(6,25,25,2);
					usleep(100000);
					Set_LED(0,0);
					usleep(300000);
					Display_Battery_Status(Display_Low);
					Beep(6,25,25,2);
					Beep(6,25,25,2);
					Temp_Mode=0;
				}
				else
				{
//					Reset_Error_Code();
					Set_LED(100,0);
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
  //          			Set_LED(100,0);
	//					Beep(5,25,25,2);
	//					Beep(3,25,25,2);
	//				}
//					if(!Is_ChargerOn()&&(Temp_Mode!=Clean_Mode_Navigation))Initialize_Motor();
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

		Error_Show_Counter++;
	  	if(Error_Show_Counter>500)
	  	{
//			Test_Mode_Flag=0;
			Error_Show_Counter=0;
//			Sound_Out_Error(Get_Error_Code());
		}

		#ifdef ONE_KEY_DISPLAY
		usleep(8000);

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
//				if(Is_Base_C())
//				{
//					Reset_Base_C();
//					if(Get_Rcon_Status()&0x000000ff)
//					{
//						Reset_Rcon_Status();
//						if(Get_Cliff_Trig()==0)
//						{
//							if(Turn_Connect())
//							{
//								Set_Clean_Mode(Clean_Mode_Charging);
//								break;
//							}
//							Disable_Motors();
//						}
//
//					}
//				}
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
		
//		if(Get_Error_Code())//display Error = red led full
//		{
//			Set_LED(0,100);
//		}
		if(BTA_Power_Dis)//display low battery = red & green
		{
			Set_LED(ONE_Display_Counter,ONE_Display_Counter);
		}
		else
		{
			Set_LED(ONE_Display_Counter,0);//display normal green
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




