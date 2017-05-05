#include <stdint.h>
#include <unistd.h>
#include <ros/ros.h>
#include "sleep.h"
#include "movement.h"
#include "wav.h"
#include <ros/ros.h>
/*----------------------------------------------------------------Sleep mode---------------------------*/
void Sleep_Mode(void)
{
  	uint8_t time=0;
	static uint32_t Ch_WP_Counter=0;
	
	Reset_Stop_Event_Status();
	Reset_Rcon_Status();
	Set_LED(0,0);
	
	Disable_Motors();
	Set_Main_PwrByte(POWER_DOWN);
	ROS_INFO("%s %d,power status %u ",__FUNCTION__,__LINE__,Get_Main_PwrByte());
	while(ros::ok())
	{
		usleep(200000);
		SetSleepModeFlag();
		// Time for key pressing
		time=0;
		if (Get_Key_Press() & KEY_CLEAN)
		{
			time = Get_Key_Time(KEY_CLEAN);
			if (time > 20)
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				Set_Main_PwrByte(POWER_ACTIVE);
				ResetSleepModeFlag();
				Beep(3, 50, 0, 1);
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_INFO("%s %d: Clean key pressed", __FUNCTION__, __LINE__);
					usleep(100000);
				}
				Reset_Stop_Event_Status();
				
				break;
			}
		}
		if(Get_Plan_Status())
		{
			Set_Plan_Status(false);
	//		wav_play(WAV_APPOINTMENT_DONE);
			Beep(Beep_Error_Sounds, 2, 0, 1);
		}
		if(Remote_Key(Remote_Clean))
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Set_Main_PwrByte(POWER_ACTIVE);
			ResetSleepModeFlag();
			Reset_Rcon_Remote();
			break;
		}
		Reset_Rcon_Remote();
		if(Is_AtHomeBase() && (Get_Cliff_Trig() == 0))//on base but miss charging , adjust position to charge
		{
			Set_Main_PwrByte(POWER_ACTIVE);
			if(Turn_Connect())
			{
				Set_Clean_Mode(Clean_Mode_Charging);
				break;
			}
		}
		/*-----------------Check if near the charging base-----------------------------*/
		if(Get_Rcon_Status()&0x777777 && !Get_Error_Code())
		{
			Ch_WP_Counter++;
			Reset_Rcon_Status();
			ROS_INFO("%d,%d,Rcon_status %d",__FUNCTION__,__LINE__,Get_Rcon_Status());
			if(Ch_WP_Counter>50)
			{
				Ch_WP_Counter=0;
				Set_Clean_Mode(Clean_Mode_GoHome);
				SetHomeRemote();
				Set_Main_PwrByte(POWER_ACTIVE);
				ResetSleepModeFlag();
				break;
			}
		}
		if(Is_ChargerOn())
		{
			Ch_WP_Counter=0;
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
}
