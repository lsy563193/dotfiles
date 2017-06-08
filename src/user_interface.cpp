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

extern uint32_t g_cur_wtime;
/*------------------------------------------------------------User Interface ----------------------------------*/
void User_Interface(void)
{
	static volatile uint8_t Press_time=0;
	static volatile uint8_t Temp_Mode=0;
	static volatile uint16_t Error_Show_Counter=400;
	static volatile uint32_t TimeOutCounter=0;

#ifdef ONE_KEY_DISPLAY
	uint16_t LedBreathCount=100;
	uint8_t breath =1;
#endif
	bool Battery_Ready_to_clean = true;
	bool battery_too_low_ = false;

	uint16_t charge_signal_count_ = 0;
	uint16_t no_charge_signal_count_ = 0;
	bool detect_charger_signal_ = false;

	Press_time=0;
	Temp_Mode=0;
	Error_Show_Counter=400;
	TimeOutCounter=0;

	Disable_Motors();
//	Beep(3,25,25,1);
	//usleep(600000);
//	Beep(3,25,25,1);
	usleep(600000);

//	Reset_Encoder_Error();

	Reset_Rcon_Remote();

	Set_Plan_Status(0);

	Reset_Stop_Event_Status();
	//Enable_PPower();
	//Disable_Motors();
	Reset_Rcon_Status();
//	Clear_Clcok_Receive();
	// Reset touch to avoid previous touch leads to directly go to navigation mode.
//	ResetHomeRemote();
	Set_VacMode(Vac_Save);

	// Count for error alarm.
	uint8_t Error_Alarm_Counter = 2;

	ROS_INFO("%s,%d ,BatteryVoltage = %d v.",__FUNCTION__,__LINE__, GetBatteryVoltage());
	// Check the battery to warn the user.
	if(!Check_Bat_Ready_To_Clean() && !robot::instance()->isManualPaused())
	{
		ROS_WARN("%s %d: Battery level low %4dV(limit in %4dV).", __FUNCTION__, __LINE__,GetBatteryVoltage(),(int)BATTERY_READY_TO_CLEAN_VOLTAGE);
		Battery_Ready_to_clean = false;
		wav_play(WAV_BATTERY_LOW);
	}

	while(ros::ok())
	{
		usleep(10000);
		// Check the battery to warn the user.
		if (Check_Bat_Stop())
		{
			battery_too_low_ = true;
		}
		if(!Check_Bat_Ready_To_Clean() && !robot::instance()->isManualPaused())
		{
			Battery_Ready_to_clean = false;
		}

#ifdef ONE_KEY_DISPLAY

		//ROS_INFO("One key min_distant_segment logic. odc = %d", LedBreathCount);
		if(breath){
			LedBreathCount--;
			if(LedBreathCount<=0)
				breath = 0;
		}
		else{
			LedBreathCount++;
			if(LedBreathCount >=100)
				breath = 1;
		}
		// TimeOutCounter is for counting that robot will go to sleep mode if there is not any control signal within 600s(10min).
		TimeOutCounter++;
		if(TimeOutCounter > USER_INTERFACE_TIMEOUT * 100)
		{
			TimeOutCounter=0;
			ROS_WARN("Userinterface mode didn't receive any command in 10mins, go to sleep mode.");
			Set_Clean_Mode(Clean_Mode_Sleep);
			break;
		}

		if(Get_Error_Code())//min_distant_segment Error = red led full
		{
			Set_LED(0,100);
		}
		else if(!Battery_Ready_to_clean)//min_distant_segment low battery = red & green
		{
			Set_LED(LedBreathCount,LedBreathCount);
		}
		else
		{
			Set_LED(LedBreathCount,0);//min_distant_segment normal green
		}

#endif

		// Alarm for error.
		// The TimeOutCounter will be atleast 1 here.
		if (Error_Alarm_Counter != 0 && Get_Error_Code() && (TimeOutCounter % 1000) == 0)
		{
			Error_Alarm_Counter--;
			Alarm_Error();
		}

		/*-------------------------------If has error, only clean key or remote key clean will reset it--------------*/
		if (Get_Error_Code() != Error_Code_None)
		{
			if (Remote_Key(Remote_All & ~Remote_Clean))
			{
				Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
				Reset_Rcon_Remote();
				Error_Alarm_Counter = 0;
				Alarm_Error();
			}
			else if (Remote_Key(Remote_Clean) || Get_Key_Press() & KEY_CLEAN)
			{
				Beep(2, 2, 0, 1);//Beep for useless remote command
				// Wait for user to release the key.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_INFO("User still holds the key.");
					usleep(100000);
				}
				// Key relaesed, then the touch status and stop event status should be cleared.
				Reset_Stop_Event_Status();
				wav_play(WAV_CLEAR_ERROR);
				Set_Error_Code(Error_Code_None);
				Reset_Rcon_Remote();
			}

			if (Get_Plan_Status() == 3)
			{
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				Alarm_Error();
				wav_play(WAV_CANCEL_APPOINTMENT);
				Set_Plan_Status(0);
			}

			continue;
		}

		/*--------------------------------------------------------If manual pause cleaning, check cliff--------------*/
		if (robot::instance()->isManualPaused())
		{
			if (Get_Cliff_Trig() & (Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
			{
				ROS_WARN("Robot lifted up during manual pause, reset manual pause status.");
				wav_play(WAV_ERROR_LIFT_UP);
				Clear_Manual_Pause();
			}
		}
		else{
			Reset_Work_Time();
			g_cur_wtime = 0;
		}

		/*--------------------------------------------------------Check if on the charger stub--------------*/
		if(Is_AtHomeBase() && (Get_Cliff_Trig() == 0))//on base but miss charging , adjust position to charge
		{
			if(Turn_Connect())
			{
				Temp_Mode = Clean_Mode_Charging;
			}
			Disable_Motors();
		}

		/*--------------------------------------------------------Check if remote move event--------------*/
		if(Remote_Key(Remote_Forward | Remote_Right | Remote_Left | Remote_Backward))
		{
			Temp_Mode = Clean_Mode_Remote;
			Reset_Rcon_Remote();
		}

		/* -----------------------------Check if spot event ----------------------------------*/
		if(Remote_Key(Remote_Spot))//                                       Check Remote Key Spin
		{
			//Transmite_BAT();
			Set_MoveWithRemote();
			Reset_Rcon_Remote();
			Temp_Mode=Clean_Mode_Spot;
		}

		/* -----------------------------Check if detects home signal -------------------------*/
		if (Is_Station())
		{
			if (!robot::instance()->isManualPaused())
			{
				detect_charger_signal_ = true;
				no_charge_signal_count_ = 0;
			}
			//else
			//	ROS_DEBUG("Manual pause and skip the charger signal.");
			Reset_Rcon_Status();
		}
		else
		{
			if (detect_charger_signal_)
				no_charge_signal_count_++;
		}

		if (no_charge_signal_count_ > 20)
		{
			charge_signal_count_ = 0;
			detect_charger_signal_ = false;
		}

		if (detect_charger_signal_)
		{
			charge_signal_count_++;
			//ROS_WARN("%s %d: User_Interface detects charger signal count %d.", __FUNCTION__, __LINE__, charge_signal_count_);
		}

		if (charge_signal_count_ > 18000) // 18000 x 10ms = 3min
		{
			Temp_Mode = Clean_Mode_GoHome;
		}

		/* -----------------------------Check if Home event ----------------------------------*/
		if(Remote_Key(Remote_Home)) //                                    Check Key Home
		{
			Temp_Mode=Clean_Mode_GoHome;
		//	Reset_MoveWithRemote();
			Set_MoveWithRemote();
			SetHomeRemote();
			Reset_Rcon_Remote();
		}

		/* -----------------------------Check if wall follow event ----------------------------------*/
		if(Remote_Key(Remote_Wall_Follow))//                                  Check Remote Key Wallfollow
		{
			Set_MoveWithRemote();
			Reset_Rcon_Remote();
			Temp_Mode=Clean_Mode_WallFollow;
		}

		/* -----------------------------Check if plan event ----------------------------------*/
		switch (Get_Plan_Status())
		{
			case 1:
			{
				ROS_INFO("%s %d: Appointment received.", __FUNCTION__, __LINE__);
				Beep(2, 2, 0, 1);
				Set_Plan_Status(0);
				break;
			}
			case 2:
			{
				ROS_INFO("%s %d: Appointment canceled.", __FUNCTION__, __LINE__);
				wav_play(WAV_CANCEL_APPOINTMENT);
				Set_Plan_Status(0);
				break;
			}
			case 3:
			{
				ROS_INFO("%s %d: Appointment activated.", __FUNCTION__, __LINE__);
				// Sleep for 50ms cause the status 3 will be sent for 3 times.
				usleep(50000);
				Set_Plan_Status(0);
				Temp_Mode=Clean_Mode_Navigation;
				break;
			}
			case 4:
			{
				ROS_INFO("%s %d: Appointment succeeded.", __FUNCTION__, __LINE__);
				wav_play(WAV_APPOINTMENT_DONE);
				Set_Plan_Status(0);
				break;
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

		/* -----------------------------Check if remote clean event ----------------------------*/
		if(Remote_Key(Remote_Clean))//                                       Check Remote Key Clean
		{
			Reset_Rcon_Remote();
			Temp_Mode=Clean_Mode_Navigation;
			Reset_MoveWithRemote();
		}

		/* -----------------------------Check if key clean event ----------------------------*/
		if(Get_Key_Press() & KEY_CLEAN)//                                    Check Key Clean
		{
//			Beep(2, 15, 0, 1);
			Press_time=Get_Key_Time(KEY_CLEAN);
			// Long press on the clean button means let the robot go to sleep mode.
			if(Press_time>20)
			{
				ROS_INFO("%s %d: Long press and go to sleep mode.", __FUNCTION__, __LINE__);
				//Beep(6,25,25,1);
				Beep(1, 4, 0, 1);
				usleep(100000);
				Beep(2,4,0,1);
				usleep(100000);
				Beep(3,4,0,1);
				usleep(100000);
				Beep(5,4,4,1);
				// Wait for user to release the key.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_INFO("User still holds the key.");
					usleep(100000);
				}
				// Key relaesed, then the touch status and stop event status should be cleared.
				Reset_Stop_Event_Status();
				Temp_Mode=Clean_Mode_Sleep;
			}
			else
			{
				Temp_Mode=Clean_Mode_Navigation;
				Reset_Work_Time();
			}
			Reset_MoveWithRemote();
		//	Reset_Error_Code();
		}

		if(Temp_Mode)
		{
			if((Temp_Mode==Clean_Mode_Sleep)||(Temp_Mode==Clean_Mode_Charging))
			{
//				Reset_Bumper_Error();
//				Reset_Error_Code();
				Set_Clean_Mode(Temp_Mode);
//				Set_CleanKeyDelay(0);
				break;
			}
			if((Temp_Mode==Clean_Mode_GoHome)||(Temp_Mode==Clean_Mode_WallFollow)||(Temp_Mode==Clean_Mode_Spot)||(Temp_Mode==Clean_Mode_RandomMode)||(Temp_Mode==Clean_Mode_Navigation)||(Temp_Mode==Clean_Mode_Remote))
			{
				ROS_INFO("[user_interface.cpp] GetBatteryVoltage = %d.", GetBatteryVoltage());
				if(Get_Cliff_Trig() & (Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
				{
//					Set_Error_Code(Error_Code_Cliff);
//					Error_Show_Counter=400;
					ROS_WARN("%s %d: Robot lift up.", __FUNCTION__, __LINE__);
					wav_play(WAV_ERROR_LIFT_UP);
					Temp_Mode=0;
				}
				else if (battery_too_low_)
				{
					ROS_WARN("%s %d: Battery level low %4dV(limit in %4d V)", __FUNCTION__, __LINE__,GetBatteryVoltage(),(int)LOW_BATTERY_STOP_VOLTAGE);
					wav_play(WAV_BATTERY_LOW);
					Temp_Mode=0;
				}
				else if((Temp_Mode != Clean_Mode_GoHome && Temp_Mode != Clean_Mode_Remote) && !Battery_Ready_to_clean)
				{
					ROS_WARN("%s %d: Battery level low %4dV(limit in %4d V)", __FUNCTION__, __LINE__,GetBatteryVoltage(),(int)BATTERY_READY_TO_CLEAN_VOLTAGE);
					wav_play(WAV_BATTERY_LOW);
					Temp_Mode=0;
				}
				else
				{
//					Reset_Error_Code();
					Set_Clean_Mode(Temp_Mode);
//					Set_CleanKeyDelay(0);
					Reset_Rcon_Remote();
//					Reset_Bumper_Error();
					break;
				}
			}
			Temp_Mode=0;
		}

		//Error_Show_Counter++;
		//if(Error_Show_Counter>500)
		//{
		//	Test_Mode_Flag=0;
		//	Error_Show_Counter=0;
		//	Sound_Out_Error(Get_Error_Code());
		//}
	}

	if (Get_Clean_Mode() != Clean_Mode_Sleep)
	{
		// Any manual operation will reset the error status.
		ROS_INFO("Reset the error code,");
		Set_Error_Code(Error_Code_None);
	}
}
