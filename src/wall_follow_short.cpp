 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Move near the wall on the left in a certain distance
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "wall_follow_short.h"
#include "movement.h"
#include "config.h"
#include "spot.h"
#include <ros/ros.h>
#include <stdint.h>
#include <unistd.h>


const uint32_t Wall_Distance_Array[10]={1000,1500,1800,3000,3500,2500,5500,6000,4000,2200};

const uint32_t Wall_Room_Array[5]={20000,10000,6000,4500,3000};


uint8_t Wall_Follow_Short(uint32_t dis)
{
	static volatile uint8_t Motor_Check_Code=0;
  static volatile int32_t Proportion=0;
	static volatile int32_t Delta=0;
	static volatile int32_t Previous=0;
  static volatile int32_t Wall_Distance=Wall_High_Limit;
	static volatile int32_t Wall_Straight_Distance=100;
	static volatile int32_t Left_Wall_Speed=0;
	static volatile int32_t Right_Wall_Speed=0;
  static uint8_t Temp_Random_Factor=0;
  static uint32_t Temp_Rcon_Status = 0;
	static uint8_t Temp_Counter=0;
	static uint8_t Jam=0;
	static int32_t R=0;
  static uint8_t Bumper_Limit = 0;
	static int16_t Left_Wall_Buffer[3]={0};
  static uint32_t Temp_Mobility_Distance=0;
  static uint8_t Mobility_Temp_Error=0;
	static int32_t Follow_Distance = 0;
  static uint32_t SWall_B_Counter=0;

	Motor_Check_Code=0;
	Proportion=0;
	Delta=0;
	Previous=0;
	Wall_Distance=Wall_High_Limit;
	Wall_Straight_Distance=100;
	Left_Wall_Speed=0;
	Right_Wall_Speed=0;
	Temp_Random_Factor=0;
	Temp_Rcon_Status = 0;
	Temp_Counter=0;
	Jam=0;
	R=0;
//	Bumper_Limit = 0;
	Left_Wall_Buffer[0]=0;
	Left_Wall_Buffer[1]=0;
	Left_Wall_Buffer[2]=0;
	Temp_Mobility_Distance=0;
	Mobility_Temp_Error=0;
	Follow_Distance = 0;
	ROS_DEBUG("In Wall Follow short");	
	if(get_work_time()>9000)
	{
		Follow_Distance=30000;
		Bumper_Limit=13;
	}
	else
	{
		Temp_Random_Factor = get_random_factor()/10;
		
		if(Temp_Random_Factor>9)Temp_Random_Factor=9;
		
		
		dis/=2000;
		if(dis>4)dis=4;
		
		Follow_Distance = Wall_Room_Array[dis];
		
		Follow_Distance += (Wall_Distance_Array[Temp_Random_Factor]);

		Bumper_Limit = get_random_factor()/25;
		
		Bumper_Limit +=6;
	}

//	reset_bumper_error();

	work_motor_configure();
	set_vac_speed();
	set_right_wheel_speed(15);
	reset_wall_step();
	move_forward(7, 7);
	reset_rcon_status();
	reset_wheel_step();
	set_mobility_step(1000);
	reset_average_counter();
	reset_wall_accelerate();
	Wall_Straight_Distance=300;
	set_led(100, 0);
	SWall_B_Counter=0;

  while(ros::ok())
  {
	usleep(1000);
    if(get_left_wheel_step()<500)
    {
      Mobility_Temp_Error=0;
      Temp_Mobility_Distance = get_move_distance();
    }
    else
    {
      if((get_move_distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = get_move_distance();
        if(get_mobility_step()<1)
        {
          Mobility_Temp_Error++;
          if(Mobility_Temp_Error>5)
          {
            break;
          }
        }
        else
        {
          Mobility_Temp_Error=0;
        }
				reset_mobility_step();
      }
    }
	  /*------------------------------------------------------Check Current-----------------------*/
	   Motor_Check_Code= check_motor_current();
		if(Motor_Check_Code)
		{
		  if(self_check(Motor_Check_Code))
			{
				set_clean_mode(Clean_Mode_Userinterface);
			  return 1;
			}
      break;
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(stop_event())
		{
			set_clean_mode(Clean_Mode_Userinterface);
			beep(5, 20, 0, 1);
			stop_brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status should be cleared.
			reset_stop_event_status();
		  return 1;
		}
		#ifdef BLDC_INSTALL
		if(Remote_Key(Remote_Max))
		{
			Switch_VacMode();
			Reset_Rcon_Remote();
		}
		else if(Get_Rcon_Remote() > 0)
		{
			Reset_Rcon_Remote();
			break;
		}
		#else
		if(get_rcon_remote() > 0)
		{
			reset_rcon_remote();
			break;
		}
		#endif
		/*------------------------------------------------------Check Battery-----------------------*/
		
		if(check_bat_set_motors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power))//Low Battery Event
    	{
			break;
 		}	
		/*------------------------------------------------------Virtual Wall Event-----------------------*/
		Temp_Rcon_Status = get_rcon_status();
    	if(Temp_Rcon_Status)
    	{
				reset_rcon_status();
      		if(Temp_Rcon_Status&0X0FFF)
      		{
        		if(is_work_finish(get_room_mode()))
        		{
          			break;
        		}
      		}
			if(Temp_Rcon_Status&0X0F00)
      		{
						stop_brifly();
				if(Temp_Rcon_Status & RconFR_HomeT)
				{
					Turn_Right(Turn_Speed,1300);
				}
				else if(Temp_Rcon_Status & RconFL_HomeT)
				{
					Turn_Right(Turn_Speed,1200);
				}
				else if(Temp_Rcon_Status & RconL_HomeT)
				{
					Turn_Right(Turn_Speed,900);
				}
				else if(Temp_Rcon_Status & RconR_HomeT)
				{
					Turn_Right(Turn_Speed,1500);
				}
				//stop_brifly();
						move_forward(10, 10);
						reset_rcon_status();
				Wall_Straight_Distance=80;
						reset_wall_accelerate();
			}
    	} 
		/*---------------------------------------------------Virtual Wall-----------------------*/
#ifdef VIRTUAL_WALL

		if (virtual_wall_turn_right()) {
			Wall_Straight_Distance = 150;
			reset_wall_accelerate();
		}

#endif

		/*---------------------------------------------------Bumper Event-----------------------*/
    if(get_bumper_status()&RightBumperTrig)
    {
			stop_brifly();
			wall_move_back();
		if(get_wall_accelerate()>80)
		{
			if(is_bumper_jamed())break;
		}
		if(get_wall_accelerate()<2000)
		{
			Jam++;
		}
		else
		{
		  Jam=0;
		}

	    Turn_Right(Turn_Speed-5,720);
			move_forward(15, 15);
			reset_wall_accelerate();
		Wall_Straight_Distance=375;

		for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
		{
			Left_Wall_Buffer[Temp_Counter]=0;
		}
			stop_brifly();
			reset_wheel_step();
		SWall_B_Counter+=1;
    }
	if(get_bumper_status()&LeftBumperTrig)
    {
			set_wheel_speed(0, 0);
      	usleep(30000);
			
//      if(get_wall_adc(0)>200)
//			{
//				Wall_Distance-=100;
//			}
//			else
//			{
		Wall_Distance-=100;
			
		if(Wall_Distance<Wall_Low_Limit)Wall_Distance=Wall_Low_Limit;	
			
		if(get_bumper_status()&RightBumperTrig)
		{
			wall_move_back();
			if(is_bumper_jamed())break;;
			Turn_Right(Turn_Speed-5,600);
			Wall_Straight_Distance=150;
		}
		else
		{
			wall_move_back();
			if(is_bumper_jamed())break;
			if(Jam<3)
			{
				Turn_Right(Turn_Speed-10,300);
			}
			else
			{
				Turn_Right(Turn_Speed-10,150);
			}
			Wall_Straight_Distance=250;
		}

		if(get_wall_accelerate()<2000)
		{
			Jam++;
		}
		else
		{
			Jam=0;
		}

		reset_wall_accelerate();
		move_forward(10, 10);
		for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
		{
			Left_Wall_Buffer[Temp_Counter]=0;
		}	
      	//stop_brifly();
		reset_wheel_step();
		SWall_B_Counter+=1;
    }
    

		/*------------------------------------------------------Cliff Event-----------------------*/
    if(get_cliff_trig())
    {
			set_wheel_speed(0, 0);
			set_dir_backward();
	    usleep(15000);
//			if(get_cliff_trig())
//			{
			cliff_move_back();
				if(get_cliff_trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
				{
					set_clean_mode(Clean_Mode_Userinterface);
				  break;
				}
				if(get_cliff_trig())
			  {
				  if(cliff_escape())
					{
						set_clean_mode(Clean_Mode_Userinterface);
            return 1;
					}
				}
			
				Turn_Right(Turn_Speed-10,900);
			stop_brifly();
			reset_wall_accelerate();
			reset_wheel_step();
//				break;
//			}
    }
		/*---------------------------------------------------Jam---------------------------------------------*/
		if(Jam>12)
		{
			break;
		}
		if(SWall_B_Counter>Bumper_Limit)break;
    /*---------------------------------------------------Jam---------------------------------------------*/
    if(Wall_Distance>=200)
		{
			Left_Wall_Buffer[2]=Left_Wall_Buffer[1];
			Left_Wall_Buffer[1]=Left_Wall_Buffer[0];
			Left_Wall_Buffer[0]= get_wall_adc(0);
			if(Left_Wall_Buffer[0]<100)
			{
			  if((Left_Wall_Buffer[1]-Left_Wall_Buffer[0])>(Wall_Distance/25))
				{
				  if((Left_Wall_Buffer[2]-Left_Wall_Buffer[1])>(Wall_Distance/25))
					{
					  if(get_wall_accelerate()>300)
						{
						  if((get_right_wheel_speed()- get_left_wheel_speed())>=-3)
							{
								move_forward(18, 16);
								usleep(100000);
								reset_wall_accelerate();
								Wall_Straight_Distance=300;
							}
						}
					}
				}
			}
		}
		
		/*------------------------------------------------------Short Distance Move-----------------------*/
		if (get_wall_accelerate() < (uint32_t) Wall_Straight_Distance)
		{
      		if(get_left_wheel_step()<500)
      		{
        		if(get_wall_accelerate()<100)
  				{
						move_forward(15, 15);
  				}
  				else
  				{
						move_forward(20, 20);
  				}
      		}
      		else
      		{
						move_forward(25, 25);
      		}
		}
		else
		{
			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
	    	if(get_front_obs()< get_front_obs_value())
	    	{
					Proportion = get_wall_adc(0);
					
					Proportion = Proportion*100/Wall_Distance;
					
					Proportion-=100;
				
				  Delta = Proportion - Previous;
					
					if(Wall_Distance>400)//over left
					{
						Left_Wall_Speed = 25 + Proportion/12 + Delta/5;
						Right_Wall_Speed = 25 - Proportion/10 - Delta/5;
						if(Right_Wall_Speed>33)
						{
							Left_Wall_Speed=7;
							Right_Wall_Speed=40;
						}
					}
					else 
					{
						Left_Wall_Speed = 22 + Proportion/16 + Delta/5;
						Right_Wall_Speed = 22 - Proportion/11 - Delta/5;
						if(Right_Wall_Speed>28)
						{
							Left_Wall_Speed=6;
							Right_Wall_Speed=30;
						}
					}

					Previous = Proportion;
					
					if(Left_Wall_Speed<0)Left_Wall_Speed=0;
					if(Left_Wall_Speed>40)Left_Wall_Speed=40;
					if(Right_Wall_Speed<0)Right_Wall_Speed=0;

					move_forward(Left_Wall_Speed, Right_Wall_Speed);

				if(get_right_wall_step()> get_left_wall_step())
				{
					R= get_right_wall_step()- get_left_wall_step();
					if(R>7500)//turn over 3600 degree
					{
						break;
					}
				}
	      
	      		if((get_right_wall_step()>Follow_Distance)||(get_left_wall_step()>Follow_Distance))//about 5 Meter
	      		{
          			break; 
	      		}
				if(get_wall_accelerate()>750)
				{
					//Set_Left_Brush(ENABLE);
			   		//Set_Right_Brush(ENABLE);
				}
	   		}
	    	else
	    	{
					stop_brifly();
				if(get_left_wheel_step()<12500)
				{
					if(get_front_obs()> get_front_obs_value())
					{
						if(get_wall_accelerate()<2000)
						{
							Jam++;
						}
					Turn_Right(Turn_Speed-5,800);
						move_forward(15, 15);
					}
					else
					{
						Turn_Right(Turn_Speed-5,500);
						move_forward(15, 15);
					}
				}
				else
				{
					Turn_Right(Turn_Speed-5,900);
					move_forward(15, 15);
				}
					reset_wheel_step();
				Wall_Distance+=200;
				if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
			}
		}
	}
	set_direction_flag(Direction_Flag_Right);
	//stop_brifly();
	//Wheel_Stop();
	set_wheel_speed(0, 0);
	usleep(20000);
	return 0;
}



