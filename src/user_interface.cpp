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
#include "event_manager.h"

uint8_t Temp_Mode=0;
time_t charger_signal_start_time;
uint16_t charger_signal_delay = 20;
boost::mutex charger_signal_delay_mutex;
time_t battery_low_start_time;
uint16_t battery_low_delay = 10;
boost::mutex battery_low_delay_mutex;
/*------------------------------------------------------------User Interface ----------------------------------*/
void User_Interface(void)
{
	static volatile uint8_t Press_time=0;
	static volatile uint16_t Error_Show_Counter=400;
	time_t start_time;

#ifdef ONE_KEY_DISPLAY
	uint16_t LedBreathCount=100;
	uint8_t breath =1;
#endif
	bool Battery_Ready_to_clean = true;
	bool battery_too_low_ = false;

	// Count for error alarm.
	uint8_t Error_Alarm_Counter = 2;

	charger_signal_delay = 0;
	start_time = time(NULL);
	Press_time=0;
	Temp_Mode=0;
	Error_Show_Counter=400;

	user_interface_register_events();
	Disable_Motors();
	Reset_Rcon_Remote();
	Set_Plan_Status(0);
	Reset_Stop_Event_Status();
	Reset_Rcon_Status();
	Set_VacMode(Vac_Save);

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

		charger_signal_delay_mutex.lock();
		if (charger_signal_delay > 0)
			charger_signal_delay--;
		charger_signal_delay_mutex.unlock();

		battery_low_delay_mutex.lock();
		if (battery_low_delay > 0)
			battery_low_delay--;
		battery_low_delay_mutex.unlock();
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
		if(time(NULL) - start_time > USER_INTERFACE_TIMEOUT)
		{
			ROS_WARN("Userinterface mode didn't receive any command in 10mins, go to sleep mode.");
			Set_Clean_Mode(Clean_Mode_Sleep);
			break;
		}

		if(get_error_code())//min_distant_segment Error = red led full
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
		if (get_error_code())
			if ((Error_Alarm_Counter == 2 && (time(NULL) - start_time) > 10) || (Error_Alarm_Counter == 1 && (time(NULL) - start_time) > 20))
			{
				Error_Alarm_Counter--;
				alarm_error();
			}

		/*-------------------------------If has error, only clean key or remote key clean will reset it--------------*/
		if (get_error_code() != Error_Code_None)
		{
			if (Remote_Key(Remote_All & ~Remote_Clean))
			{
				beep_for_command(false);
				Reset_Rcon_Remote();
				Error_Alarm_Counter = 0;
				alarm_error();
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
				set_error_code(Error_Code_None);
				Reset_Rcon_Remote();
			}

			if (Get_Plan_Status() == 3)
			{
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				alarm_error();
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

		/*--------------------------------------------------------Check if on the charger stub--------------*/
		if(is_on_charger_stub() || is_direct_charge())//on base but miss charging , adjust position to charge
		{
			ROS_WARN("%s %d: Detect charging.", __FUNCTION__, __LINE__);
			if (is_direct_charge())
				Temp_Mode = Clean_Mode_Charging;
			else if(Turn_Connect())
				Temp_Mode = Clean_Mode_Charging;
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
				if (!robot::instance()->isManualPaused())
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
			if(Press_time>151)
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
				// Wait for beep finish.
				usleep(200000);
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
				Temp_Mode=Clean_Mode_Navigation;
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
//					set_error_code(Error_Code_Cliff);
//					Error_Show_Counter=400;
					ROS_WARN("%s %d: Robot lift up.", __FUNCTION__, __LINE__);
					wav_play(WAV_ERROR_LIFT_UP);
					Temp_Mode=0;
					charger_signal_delay = 0;
				}
				else if (battery_too_low_)
				{
					ROS_WARN("%s %d: Battery level low %4dV(limit in %4d V)", __FUNCTION__, __LINE__,GetBatteryVoltage(),(int)LOW_BATTERY_STOP_VOLTAGE);
					wav_play(WAV_BATTERY_LOW);
					Temp_Mode=0;
					charger_signal_delay = 0;
				}
				else if((Temp_Mode != Clean_Mode_GoHome && Temp_Mode != Clean_Mode_Remote) && !Battery_Ready_to_clean)
				{
					ROS_WARN("%s %d: Battery level low %4dV(limit in %4d V)", __FUNCTION__, __LINE__,GetBatteryVoltage(),(int)BATTERY_READY_TO_CLEAN_VOLTAGE);
					wav_play(WAV_BATTERY_LOW);
					Temp_Mode=0;
					charger_signal_delay = 0;
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

	}

	if (Get_Clean_Mode() != Clean_Mode_Sleep)
	{
		// Any manual operation will reset the error status.
		ROS_INFO("Reset the error code,");
		set_error_code(Error_Code_None);
	}

	user_interface_unregister_events();
}

void user_interface_register_events(void)
{
	ROS_WARN("Register rcon");
	event_manager_set_current_mode(EVT_MODE_USER_INTERFACE);

#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &user_interface_handle_ ##name); \
	event_manager_enable_handler(y, enabled);

	/* Rcon */
	event_manager_register_and_enable_x(rcon, EVT_RCON, true);
	/* Battery */
	event_manager_register_and_enable_x(battery_low, EVT_BATTERY_LOW, true);
}

void user_interface_unregister_events(void)
{
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	/* Rcon */
	event_manager_register_and_disable_x(EVT_RCON);
	/* Battery */
	event_manager_register_and_disable_x(EVT_BATTERY_LOW);
}

void user_interface_handle_rcon(bool state_now, bool state_last)
{
	if (robot::instance()->isManualPaused())
	{
		Reset_Rcon_Status();
		ROS_DEBUG("%s %d: User_Interface detects charger signal, but ignore for manual pause.", __FUNCTION__, __LINE__);
		return;
	}

	ROS_DEBUG("%s %d: User_Interface detects charger signal for %ds.", __FUNCTION__, __LINE__, (int)(time(NULL) - charger_signal_start_time));
	boost::mutex::scoped_lock(charger_signal_delay_mutex);
	if (charger_signal_delay == 0)
		charger_signal_start_time = time(NULL);

	if (time(NULL) - charger_signal_start_time > 180)// 3 mins
	{
		Temp_Mode = Clean_Mode_GoHome;
		Reset_Rcon_Status();
		return;
	}

	charger_signal_delay = 20;
	Reset_Rcon_Status();

}

void user_interface_handle_battery_low(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: User_Interface detects battery low %dmv for %ds.", __FUNCTION__, __LINE__, robot::instance()->getBatteryVoltage(), (int)(time(NULL) - battery_low_start_time));
	boost::mutex::scoped_lock(battery_low_delay_mutex);
	if (battery_low_delay == 0)
		battery_low_start_time = time(NULL);

	if (time(NULL) - battery_low_start_time > 5)// 5 seconds
	{
		Temp_Mode = Clean_Mode_Sleep;
		return;
	}

	battery_low_delay = 10;
}
