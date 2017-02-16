#include <stdint.h>
#include <unistd.h>
#include "sleep.h"
#include "movement.h"

/*----------------------------------------------------------------Sleep mode---------------------------*/
void Sleep_Mode(void)
{
  uint8_t time=0;
	static uint32_t Ch_WP_Counter=0;

 // Disable_Motors();


//	delay(5000);

//	Reset_GPIO();
	
	time=0;

//  USART_DMA_String(11,"\n\r Sleep! ");
	
//	delay(500);
	
	
	/*--------------------------------ENTER LOW POWER--------------------------*/
	
	Disable_Motors();
	
	/*---------------------------------Wake Up-------------------------------*/
	Reset_Touch();
	Set_LED(0,0);
	
	//Enable_PPower();
	//judge which wakeup signal 
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
	//usleep(1000000);
//	if(Is_Alarm())
//	{
//		Ch_WP_Counter=0;
//		Set_Clean_Mode(Clean_Mode_Userinterface);
//		return;
//	}
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
//			Enable_PPower();
			SetHomeRemote();
			Set_LED(100,100);
//			Wake_Up_Adjust();
		  return;
		}
	}
	
	
	Set_Clean_Mode(Clean_Mode_Sleep);

}
