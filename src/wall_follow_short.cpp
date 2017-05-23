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
	if(Get_Work_Time()>9000)
	{
		Follow_Distance=30000;
		Bumper_Limit=13;
	}
	else
	{
		Temp_Random_Factor = Get_Random_Factor()/10;
		
		if(Temp_Random_Factor>9)Temp_Random_Factor=9;
		
		
		dis/=2000;
		if(dis>4)dis=4;
		
		Follow_Distance = Wall_Room_Array[dis];
		
		Follow_Distance += (Wall_Distance_Array[Temp_Random_Factor]);

		Bumper_Limit = Get_Random_Factor()/25;
		
		Bumper_Limit +=6;
	}

//	Reset_Bumper_Error();

	Work_Motor_Configure();
	Set_Vac_Speed();
	Set_RightWheel_Speed(15);
	Reset_Wall_Step();
	Move_Forward(7,7);
  	Reset_Rcon_Status();
  	Reset_Wheel_Step();
  	Set_Mobility_Step(1000);
	Reset_Average_Counter();
	Reset_WallAccelerate();
	Wall_Straight_Distance=300;
	Set_LED(100,0);
	SWall_B_Counter=0;

  while(ros::ok())
  {
	usleep(1000);
    if(Get_LeftWheel_Step()<500)
    {
      Mobility_Temp_Error=0;
      Temp_Mobility_Distance = Get_Move_Distance();
    }
    else
    {
      if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
      {
        Temp_Mobility_Distance = Get_Move_Distance();
        if(Get_Mobility_Step()<1)
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
        Reset_Mobility_Step();
      }
    }
	  /*------------------------------------------------------Check Current-----------------------*/
	   Motor_Check_Code=Check_Motor_Current();
		if(Motor_Check_Code)
		{
		  if(Self_Check(Motor_Check_Code))
			{
        Set_Clean_Mode(Clean_Mode_Userinterface);
			  return 1;
			}
      Reset_TempPWM();
      break;
		}
		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Stop_Event())
		{
		  Set_Clean_Mode(Clean_Mode_Userinterface);
			Beep(5, 20, 0, 1);
			Stop_Brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status should be cleared.
			Reset_Stop_Event_Status();
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
		if(Get_Rcon_Remote() > 0)
		{
			Reset_Rcon_Remote();
			break;
		}
		#endif
		/*------------------------------------------------------Check Battery-----------------------*/
		
		if(Check_Bat_SetMotors(Clean_Vac_Power,Clean_SideBrush_Power,Clean_MainBrush_Power))//Low Battery Event
    	{
			break;
 		}	
		/*------------------------------------------------------Virtual Wall Event-----------------------*/
		Temp_Rcon_Status = Get_Rcon_Status();
    	if(Temp_Rcon_Status)
    	{
      		Reset_Rcon_Status();
      		if(Temp_Rcon_Status&0X0FFF)
      		{
        		if(Is_WorkFinish(Get_Room_Mode()))
        		{
          			break;
        		}
      		}
			if(Temp_Rcon_Status&0X0F00)
      		{
				Stop_Brifly();
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
				//Stop_Brifly();
				Move_Forward(10,10);
				Reset_Rcon_Status();
				Wall_Straight_Distance=80;
				Reset_WallAccelerate();
			}
    	} 
		/*---------------------------------------------------Virtual Wall-----------------------*/
#ifdef VIRTUAL_WALL

		if (VirtualWall_TurnRight()) {
			Wall_Straight_Distance = 150;
			Reset_WallAccelerate();
		}

#endif

		/*---------------------------------------------------Bumper Event-----------------------*/
    if(Get_Bumper_Status()&RightBumperTrig)
    {
		Stop_Brifly();
		Wall_Move_Back();
		if(Get_WallAccelerate()>80)
		{
			if(Is_Bumper_Jamed())break;
		}
		if(Get_WallAccelerate()<2000)
		{
			Jam++;
		}
		else
		{
		  Jam=0;
		}

	    Turn_Right(Turn_Speed-5,720);
	    Move_Forward(15,15);
		Reset_WallAccelerate();
		Wall_Straight_Distance=375;

		for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
		{
			Left_Wall_Buffer[Temp_Counter]=0;
		}	
      	Stop_Brifly();
      	Reset_Wheel_Step();
		SWall_B_Counter+=1;
    }
	if(Get_Bumper_Status()&LeftBumperTrig)
    {	
    	Set_Wheel_Speed(0,0);
     	Reset_TempPWM();
      	usleep(30000);
			
//      if(Get_Wall_ADC(0)>200)
//			{
//				Wall_Distance-=100;
//			}
//			else
//			{
		Wall_Distance-=100;
			
		if(Wall_Distance<Wall_Low_Limit)Wall_Distance=Wall_Low_Limit;	
			
		if(Get_Bumper_Status()&RightBumperTrig)
		{
			Wall_Move_Back();
			if(Is_Bumper_Jamed())break;;
			Turn_Right(Turn_Speed-5,600);
			Wall_Straight_Distance=150;
		}
		else
		{
			Wall_Move_Back();
			if(Is_Bumper_Jamed())break;
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

		if(Get_WallAccelerate()<2000)
		{
			Jam++;
		}
		else
		{
			Jam=0;
		}
			
		Reset_WallAccelerate();
      	Move_Forward(10,10);
		for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++)
		{
			Left_Wall_Buffer[Temp_Counter]=0;
		}	
      	//Stop_Brifly();
      	Reset_Wheel_Step();
		SWall_B_Counter+=1;
    }
    

		/*------------------------------------------------------Cliff Event-----------------------*/
    if(Get_Cliff_Trig())
    {
		  Set_Wheel_Speed(0,0);
      Set_Dir_Backward();
	    usleep(15000);
//			if(Get_Cliff_Trig())
//			{
			  Cliff_Move_Back();
				if(Get_Cliff_Trig()==(Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
				  break;
				}
				if(Get_Cliff_Trig())
			  {
				  if(Cliff_Escape())
					{
					  Set_Clean_Mode(Clean_Mode_Userinterface);
            return 1;
					}
				}
			
				Turn_Right(Turn_Speed-10,900);
				Stop_Brifly();
				Reset_WallAccelerate();
				Reset_Wheel_Step();
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
			Left_Wall_Buffer[0]=Get_Wall_ADC(0);
			if(Left_Wall_Buffer[0]<100)
			{
			  if((Left_Wall_Buffer[1]-Left_Wall_Buffer[0])>(Wall_Distance/25))
				{
				  if((Left_Wall_Buffer[2]-Left_Wall_Buffer[1])>(Wall_Distance/25))
					{
					  if(Get_WallAccelerate()>300)
						{
						  if((Get_RightWheel_Speed()-Get_LeftWheel_Speed())>=-3)
							{
								Move_Forward(18,16);
								usleep(100000);
								Reset_WallAccelerate();
								Wall_Straight_Distance=300;
							}
						}
					}
				}
			}
		}
		
		/*------------------------------------------------------Short Distance Move-----------------------*/
		if (Get_WallAccelerate() < (uint32_t) Wall_Straight_Distance)
		{
      		if(Get_LeftWheel_Step()<500)
      		{
        		if(Get_WallAccelerate()<100)
  				{
  			  		Move_Forward(15,15);
  				}
  				else
  				{
  			  		Move_Forward(20,20);
  				}
      		}
      		else
      		{
       			Move_Forward(25,25);
      		}
		}
		else
		{
			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
	    	if(Get_FrontOBS()<Get_FrontOBST_Value())
	    	{
					Proportion = Get_Wall_ADC(0);
					
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
					
					Move_Forward(Left_Wall_Speed,Right_Wall_Speed);

				if(Get_RightWall_Step()>Get_LeftWall_Step())
				{
					R=Get_RightWall_Step()-Get_LeftWall_Step();
					if(R>7500)//turn over 3600 degree
					{
						break;
					}
				}
	      
	      		if((Get_RightWall_Step()>Follow_Distance)||(Get_LeftWall_Step()>Follow_Distance))//about 5 Meter
	      		{
          			break; 
	      		}
				if(Get_WallAccelerate()>750)
				{
					//Set_Left_Brush(ENABLE);
			   		//Set_Right_Brush(ENABLE);
				}
	   		}
	    	else
	    	{
				Stop_Brifly();
				if(Get_LeftWheel_Step()<12500)
				{
					if(Get_FrontOBS()>Get_FrontOBST_Value())
					{
						if(Get_WallAccelerate()<2000)
						{
							Jam++;
						}
					Turn_Right(Turn_Speed-5,800);
					Move_Forward(15,15);
					}
					else
					{
						Turn_Right(Turn_Speed-5,500);
						Move_Forward(15,15);
					}
				}
				else
				{
					Turn_Right(Turn_Speed-5,900);
					Move_Forward(15,15);
				}
				Reset_Wheel_Step();
				Wall_Distance+=200;
				if(Wall_Distance>Wall_High_Limit)Wall_Distance=Wall_High_Limit;
			}
		}
	}
	Set_Direction_Flag(Direction_Flag_Right);
	//Stop_Brifly();
	//Wheel_Stop();
	Set_Wheel_Speed(0,0);
	usleep(20000);
	return 0;
}



