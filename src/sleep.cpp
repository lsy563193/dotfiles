#include <stdint.h>
#include <unistd.h>
#include <ros/ros.h>
#include "sleep.h"
#include "movement.h"
#include <ros/ros.h>
/*----------------------------------------------------------------Sleep mode---------------------------*/
void Sleep_Mode(void)
{
  	uint8_t time=0;
	static uint32_t Ch_WP_Counter=0;

	ROS_DEBUG_NAMED("sleep","-----in sleep mode-----");	
	
	/*---------------------------------Wake Up-------------------------------*/
	Reset_Touch();
	Set_LED(0,0);
	
	/*--------------------------------ENTER LOW POWER--------------------------*/
	
	Disable_Motors();
	
	while(ros::ok())
	{
		usleep(200000);
		/*---------------------------------Wake Up-------------------------------*/
		Set_LED(0,0);

		// Time for key pressing
		time=0;
		//judge which wakeup signal
		if (Get_Key_Press() & KEY_CLEAN)
		{
			time = Get_Key_Time(KEY_CLEAN);
			if (time > 20)
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				Beep(3, 50, 0, 1);
				// Wait for user to release the key.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_INFO("%s %d: User still holds the key.", __FUNCTION__, __LINE__);
					usleep(100000);
				}
				Reset_Touch();
				return;
			}
		}

		if(Remote_Key(Remote_Clean))
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Reset_Rcon_Remote();
			return;
		}
		if(Is_AtHomeBase() && (Get_Cliff_Trig() == 0))//on base but miss charging , adjust position to charge
		{
			if(Turn_Connect())
			{
				Set_Clean_Mode(Clean_Mode_Charging);
				return;
			}
		}
		/*-----------------Check if near the charging base-----------------------------*/
		if(Get_Rcon_Status()&0x777777)
		{
			Ch_WP_Counter++;
			if(Ch_WP_Counter>50)
			{
				Ch_WP_Counter=0;
				Set_Clean_Mode(Clean_Mode_GoHome);
				//Enable_PPower();
				SetHomeRemote();
				Set_LED(100,100);
				//Wake_Up_Adjust();
	  			return;
			}
		}
		if(Is_ChargerOn())
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Charging);
			return;
		}
		/*-----------------Check if near the charging base-----------------------------*/
		if(Get_Rcon_Status()&0x777777)
		{
			Ch_WP_Counter++;
			if(Ch_WP_Counter>50)
			{
				Ch_WP_Counter=0;
				Set_Clean_Mode(Clean_Mode_GoHome);
				SetHomeRemote();
				Set_LED(100,100);
			  return;
			}
		}
	}
}
