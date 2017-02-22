#include <stdint.h>
#include <unistd.h>
#include "sleep.h"
#include "movement.h"
#include <ros/ros.h>
/*----------------------------------------------------------------Sleep mode---------------------------*/
void Sleep_Mode(void)
{
  	uint8_t time=0;
	static uint32_t Ch_WP_Counter=0;

	ROS_DEBUG_NAMED("sleep","-----in sleep mode-----");	
	Disable_Motors();
	
	/*---------------------------------Wake Up-------------------------------*/
	Reset_Touch();
	Set_LED(0,0);
	
	while(ros::ok()){
		usleep(100000);
		while(Get_Key_Press() == KEY_CLEAN)
		{
			usleep(100000);// Wait for 100ms.
			time++;
			if(time>15)
			{
				break;
			}
		}
		if(time>15) // Over 1500ms.
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Beep(3, 50, 0, 1);
			Set_LED(100, 0);
			// Wait for 2s before it wakes. So that it won't directly go to navigation mode.
			usleep(2000000);
			return;
		}
		if(Remote_Key(Remote_Clean))
		{
			Ch_WP_Counter=0;
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
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
				//Enable_PPower();
				SetHomeRemote();
				Set_LED(100,100);
				//Wake_Up_Adjust();
	  			return;
			}
		}
	}	

}
