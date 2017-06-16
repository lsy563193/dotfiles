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

	reset_wheel_step();
	Move_Forward(38,0);
	while(ros::ok())
	{

		Motor_Check_Code= check_motor_current();
		if(Motor_Check_Code)
		{
			return 0;	  
		}
		/*-------------------------------------------------------Check Battery ------------------*/
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
		{
			return 0;
		}
		/*------------------------------------------------------Stop event-----------------------*/
		if(Stop_Event())
		{
			Stop_Brifly();
			return 1;
		}
		if(Get_Rcon_Remote() > 0)
		{
			if(Remote_Key(Remote_Spot))
			{
        		Vac_Mode_Buffer = Get_VacMode();				
				Temp_Dirt_Status=Random_Dirt_Event();
				Set_VacMode(Vac_Mode_Buffer,false);
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
				turn_left(Turn_Speed, 700);
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
				Set_Clean_Mode(Clean_Mode_GoHome);
				Reset_Rcon_Remote();
				set_home_remote();
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

		set_wheel_speed(42, Right_Wheel_Speed);
//		Move_Forward(42,g_right_wheel_speed);
		if(First_Round_Flag)
		{
			if(get_left_wheel_step()>3000)First_Round_Flag=0;
		}
		else
		{
			if(get_left_wheel_step()>(Radius))
			{
				reset_wheel_step();
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
		
		if(get_bumper_status()|| get_cliff_trig()||Get_OBS_Status())
		{ 
			return 0;
		}
	}
	//return 0;
}



