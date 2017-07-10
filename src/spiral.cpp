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
uint8_t spiral(void)
{
	uint32_t Motor_Check_Code=0;
	uint8_t Temp_Dirt_Status=0;
	uint32_t Radius=0;
	uint32_t Max_Radius=0;
	int32_t Right_Wheel_Speed=0;
	uint8_t First_Round_Flag=1;
	uint8_t Vac_Mode_Buffer=0;
	ROS_DEBUG("In spiral ");
	Max_Radius = 180 + get_random_factor();

	reset_wheel_step();
	move_forward(38, 0);
	while(ros::ok())
	{

		Motor_Check_Code= check_motor_current();
		if(Motor_Check_Code)
		{
			return 0;	  
		}
		/*-------------------------------------------------------Check Battery ------------------*/
		if(check_bat_set_motors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power))//Low Battery Event
		{
			return 0;
		}
		/*------------------------------------------------------Stop event-----------------------*/
		if(stop_event())
		{
			stop_brifly();
			return 1;
		}
		if(get_rcon_remote() > 0)
		{
			if(remote_key(Remote_Spot))
			{
        		Vac_Mode_Buffer = get_vac_mode();
				Temp_Dirt_Status=Random_Dirt_Event();
				set_vacmode(Vac_Mode_Buffer, false);
				set_vac_speed();
				reset_rcon_remote();
				if(Temp_Dirt_Status==1)
				{
					set_clean_mode(Clean_Mode_Userinterface);
					return 1;
				}
				return 0;
			}
			if(remote_key(Remote_Left))
			{
				turn_left(Turn_Speed, 700);
				move_forward(30, 30);
				set_clean_mode(Clean_Mode_RandomMode);
				reset_rcon_remote();
				return 1;
			}
			if(remote_key(Remote_Right))
			{
				turn_right(Turn_Speed, 560);
				move_forward(30, 30);
				set_clean_mode(Clean_Mode_RandomMode);
				reset_rcon_remote();
				return 1;
			}
			if(remote_key(Remote_Home)) //                                    Check Key Home
			{
				set_clean_mode(Clean_Mode_GoHome);
				reset_rcon_remote();
				set_home_remote();
				return 1;
			}
			/*
			if(remote_key(Remote_Random)) //                                    Check Key Home
			{
				turn_right(Turn_Speed,560);
				move_forward(30,30);
				set_clean_mode(Clean_Mode_RandomMode);
				reset_rcon_remote();
				return 1;
			}*/
		}

		Right_Wheel_Speed = (42*Radius)/(Radius+100);
		if(Right_Wheel_Speed>42)Right_Wheel_Speed=42;
		if(Right_Wheel_Speed<0)Right_Wheel_Speed=0;

		set_wheel_speed(42, Right_Wheel_Speed);
//		move_forward(42,g_right_wheel_speed);
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
		
		if(get_bumper_status()|| get_cliff_trig()|| get_obs_status())
		{ 
			return 0;
		}
	}
	//return 0;
}



