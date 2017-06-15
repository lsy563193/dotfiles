#include <stdint.h>
#include <unistd.h>
#include <ros/ros.h>

#include "sleep.h"
#include "movement.h"
#include "wav.h"
/*----------------------------------------------------------------Sleep mode---------------------------*/
void sleep_mode(void)
{
	uint16_t sleep_time_counter_ = 0;
	
	Reset_Stop_Event_Status();
	Reset_Rcon_Status();
	Set_LED(0,0);
	
	Disable_Motors();
	Set_Main_PwrByte(POWER_DOWN);
	ROS_INFO("%s %d,power status %u ",__FUNCTION__,__LINE__,Get_Main_PwrByte());
	while(ros::ok())
	{
		usleep(20000);

		sleep_time_counter_++;
		if (sleep_time_counter_ > 1500)
		{
			// Check the battery for every 30s. If battery below 12.5v, power of core board will be cut off.
			ResetSleepModeFlag();
			ROS_WARN("Wake up robotbase to check if battery too low(<12.5v).");
			sleep_time_counter_ = 0;
		}
		else if(!GetSleepModeFlag())
			SetSleepModeFlag();

		// This is necessary because once the rcon has signal, it means base stm32 board has receive the rcon signal for 3 mins.
		// If not reset here, it may cause directly go home once it sleeps.
		Reset_Rcon_Status();

		if (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s,%d, get key press ",__FUNCTION__,__LINE__);
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Set_Main_PwrByte(POWER_ACTIVE);
			ResetSleepModeFlag();
			Set_LED(100, 0);
			Beep(4, 4, 0, 1);
			usleep(100000);
			Beep(3,4,0,1);
			usleep(100000);
			Beep(2,4,0,1);
			usleep(100000);
			Beep(1,4,4,1);
			// Wait for user to release the key.
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_WARN("User still holds the key.");
				usleep(20000);
			}
			Reset_Stop_Event_Status();
			break;
		}

		// Check if plan activated.
		if (Get_Plan_Status() == 3)
		{
			if (Get_Error_Code() == Error_Code_None)
			{
				Set_Main_PwrByte(Clean_Mode_Navigation);
				ResetSleepModeFlag();
				Beep(4, 4, 0, 1);
				usleep(100000);
				Beep(3,4,0,1);
				usleep(100000);
				Beep(2,4,0,1);
				usleep(100000);
				Beep(1,4,4,1);
				Reset_Stop_Event_Status();
				Set_Clean_Mode(Clean_Mode_Navigation);
				break;
			}
			else
			{
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				Alarm_Error();
				wav_play(WAV_CANCEL_APPOINTMENT);
				Set_Plan_Status(0);
			}
		}

		if(Remote_Key(Remote_Clean))
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Set_Main_PwrByte(POWER_ACTIVE);
			ResetSleepModeFlag();
			Reset_Rcon_Remote();
			Beep(4, 4, 0, 1);
			usleep(100000);
			Beep(3,4,0,1);
			usleep(100000);
			Beep(2,4,0,1);
			usleep(100000);
			Beep(1,4,4,1);
			break;
		}
		Reset_Rcon_Remote();
		if(is_on_charger_stub() || is_direct_charge())//on base but miss charging , adjust position to charge
		{
			Set_Main_PwrByte(POWER_ACTIVE);
			ResetSleepModeFlag();
			Reset_Rcon_Remote();
			Beep(4, 4, 0, 1);
			usleep(100000);
			Beep(3,4,0,1);
			usleep(100000);
			Beep(2,4,0,1);
			usleep(100000);
			Beep(1,4,4,1);
			if (is_direct_charge() || Turn_Connect())
			{
				Set_Clean_Mode(Clean_Mode_Charging);
				break;
			}
		}

		/*-----------------Check if near the charging base-----------------------------*/
		if(Is_Station() && (!Get_Error_Code()))
		{
			ROS_INFO("%s,%d,Rcon_status = %x",__FUNCTION__,__LINE__,Get_Rcon_Status());
			Reset_Rcon_Status();
			Set_Clean_Mode(Clean_Mode_GoHome);
			SetHomeRemote();
			Set_Main_PwrByte(POWER_ACTIVE);
			ResetSleepModeFlag();
			break;
		}
		if(Is_ChargerOn())
		{
			Set_Clean_Mode(Clean_Mode_Charging);
			Set_Main_PwrByte(POWER_ACTIVE);
			ResetSleepModeFlag();
			break;
		}
	}
	// Alarm for error.
	if (Get_Clean_Mode() == Clean_Mode_Userinterface && Get_Error_Code())
	{
		Set_LED(0, 100);
		Alarm_Error();
	}

	// Wait 1.5s to avoid gyro can't open if switch to navigation mode too soon after waking up.
	usleep(1500000);
}
