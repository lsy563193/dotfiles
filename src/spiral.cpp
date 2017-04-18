 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Spiral Path cleaning mode
	           move in a mode of a circle path by increasing the Radius
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "spiral.h"
#include "config.h"
#include "movement.h"
#include "spot.h"
#include <ros/ros.h>
#include <stdint.h>
#include <unistd.h>

///*----------------------------------------------------------- Spining Event -----------------------*/
uint8_t Spiral(void)
{
	uint32_t Motor_Check_Code=0;
	uint8_t Temp_Dirt_Status=0;
	uint32_t Radius=0;
	uint32_t Max_Radius=0;
	int32_t Right_Wheel_Speed=0;
	uint8_t First_Round_Flag=1;
	uint8_t Vac_Mode_Buffer=0;
	ROS_DEBUG("In Spiral ");	
	Max_Radius = 180 + Get_Random_Factor();

	Reset_Wheel_Step();
	Move_Forward(38,0);
	while(ros::ok())
	{

		Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
			return 0;	  
		}
		/*-------------------------------------------------------Check Battery ------------------*/
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
		{
			return 0;
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect())
		{
			Stop_Brifly();
			// Set touch status to make sure this event can be detected by main process while loop.
			Set_Touch();
			return 1;
		}
		if(Remote_Key(Remote_All))
		{
			if(Remote_Key(Remote_Spot))
			{
        		Vac_Mode_Buffer = Get_VacMode();				
				Temp_Dirt_Status=Random_Dirt_Event();
				Set_VacMode(Vac_Mode_Buffer);
				Set_Vac_Speed();
				Reset_Rcon_Remote();
				if(Temp_Dirt_Status==1)
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return 1;
				}
				return 0;
			}
			if(Remote_Key(Remote_Left))
			{
				Turn_Left(Turn_Speed,700);
				Move_Forward(30,30);
				Set_Clean_Mode(Clean_Mode_RandomMode);
				Reset_Rcon_Remote();
				return 1;
			}
			if(Remote_Key(Remote_Right))
			{
				Turn_Right(Turn_Speed,560);
				Move_Forward(30,30);
				Set_Clean_Mode(Clean_Mode_RandomMode);
				Reset_Rcon_Remote();
				return 1;
			}
			if(Remote_Key(Remote_Home)) //                                    Check Key Home
			{
				Set_LED(100,100);
				Set_Clean_Mode(Clean_Mode_GoHome);
				Reset_Rcon_Remote();
				SetHomeRemote();
				return 1;
			}
			/*
			if(Remote_Key(Remote_Random)) //                                    Check Key Home
			{
				Turn_Right(Turn_Speed,560);
				Move_Forward(30,30);
				Set_Clean_Mode(Clean_Mode_RandomMode);
				Reset_Rcon_Remote();
				return 1;
			}*/
		}

		Right_Wheel_Speed = (42*Radius)/(Radius+100);
		if(Right_Wheel_Speed>42)Right_Wheel_Speed=42;
		if(Right_Wheel_Speed<0)Right_Wheel_Speed=0;
		
		Set_Wheel_Speed(42,Right_Wheel_Speed);
//		Move_Forward(42,Right_Wheel_Speed);
		if(First_Round_Flag)
		{
			if(Get_LeftWheel_Step()>3000)First_Round_Flag=0;
		}
		else
		{
			if(Get_LeftWheel_Step()>(Radius))
			{
				Reset_Wheel_Step();
				if(Radius<100)
				{
					Radius+=1;
				}
				else
				{
					Radius+=2;
				}
			}
		}

		if(Radius>Max_Radius)
		{
			return 0;
		}
		
		if(Get_Bumper_Status()||Get_Cliff_Trig()||Get_OBS_Status())
		{ 
			return 0;
		}
	}
	//return 0;
}



