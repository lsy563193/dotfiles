 /**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V1.0
  * @date    17-Nov-2011
  * @brief   Random Path Cleaning Function
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "random_runing.h"
#include "movement.h"
#include "gyro.h"
#include "spot.h"
#include "config.h"
#include "wall_follow_short.h"
#include "spiral.h"
#include <ros/ros.h>

volatile uint8_t Half_Turn_Flag=0;

uint8_t Out_Trap_Right(void);
uint8_t Out_Trap_Left(void);
uint8_t Left_Bumper_Avoiding(void);
uint8_t Right_Bumper_Avoiding(void);
void Half_Turn_Left(uint16_t speed,uint16_t angle);
void Half_Turn_Right(uint16_t speed,uint16_t angle);

void Set_HalfTurn_Flag(void)
{
	Half_Turn_Flag=1;
}
void Reset_HalfTurn_Flag(void)
{
	Half_Turn_Flag=0;
}
uint8_t Is_HalfTurn_Flag(void)
{
	return Half_Turn_Flag;
}


/* --------------------------------------------------Random Runnincg mode----------------------*/
void Random_Running_Mode(void)
{
	volatile uint8_t Stunk = 0, Motor_Check_Code = 0, Temp_Cliff_Status = 0, Temp_OBS_Status = 0, Random_Factor = 0;
	volatile uint8_t Bumper_Counter=0,Wall_Bumper_Counter=0;
	volatile uint32_t Left_Wheel_Step_Buffer=0;
	uint32_t Moving_Speed=20;
	uint32_t Temp_Mobility_Distance=0;
	uint8_t Disacc=0;
	uint8_t Temp_Dirt_Status=0;
	uint8_t Low_Power_Counter=0;
	uint8_t Base_Wall_On=1;
	uint32_t Temp_Rcon_Status=0;
	uint8_t Temp_Bumper_Status=0;
	uint8_t Wall_Small_Counter = 0;
	uint8_t Wall_Mid_Counter = 0;
	uint8_t Wall_Bumper_Factor = 0;
	uint32_t OBS_Distance_Counter=0;
	uint8_t OBS_Cycle=0;
	uint8_t On_TrapOut_Flag=0;
	static uint8_t Vac_Mode_Buffer=0;
	static uint8_t N_H_T=0;
	static uint8_t Avoid_Flag=0;
//	static int32_t Wall_Sum_Value=0;
//	static int32_t Wall_Everage_Value=0;
//  static int32_t Wall_E_Counter=0;
//	static int32_t Temp_Wall_Buffer=0;
	volatile uint32_t OBS_Delay=0;

#ifdef VIRTUAL_WALL
	uint8_t Virtual_Wall_C = 0, Virtual_Wall_NG = 0;
#endif

	// Restart the gyro.
	Set_Gyro_Off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	Set_Gyro_On();
	if (!Wait_For_Gyro_On())
	{
		set_clean_mode(Clean_Mode_Userinterface);
		return;
	}

	reset_start_work_time();
	Wall_Bumper_Factor = get_random_factor()/15;
	 reset_move_with_remote();
//	reset_bumper_error();
	 set_led(100, 0);
	 reset_stop_event_status();

//  if(!Is_Dustbin_Install())
//  {
//    set_error_code(Error_Code_Dustbin);
//    set_clean_mode(Clean_Mode_Userinterface);
//    disable_motors();
//    return;
//  }

	ROS_DEBUG_NAMED("random mode","-------in random running mode-----");
	if(is_on_charger_stub())
	{
		set_clean_mode(Clean_Mode_Userinterface);
		beep(2, 25, 25, 1);
		usleep(400000);
		reset_rcon_remote();
		beep(2, 25, 25, 1);
		set_side_brush_pwm(30, 30);
		set_main_brush_pwm(0);
		set_bldc_speed(30);
		stop_brifly();
		quick_back(30,750);
		if(stop_event()||is_charge_on())
		{
			stop_brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			reset_stop_event_status();
			return;
		}
		beep(2, 25, 25, 1);
		quick_back(30,750);
		if(stop_event())
		{
			stop_brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			reset_stop_event_status();
			return;
		}
		beep(2, 25, 25, 1);
		quick_back(30,750);
		if(stop_event())
		{
			stop_brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			reset_stop_event_status();
			return;
		}
		beep(2, 25, 25, 1);
		Turn_Right(Turn_Speed,1120+ get_random_factor()*10);
		if(stop_event())
		{
			stop_brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			reset_stop_event_status();
			return;
		}
		stop_brifly();
		//initialize_motor();
		Base_Wall_On=0;
	}

	 set_clean_mode(Clean_Mode_RandomMode);

	 work_motor_configure();
	 reset_move_distance();
	 reset_wheel_step();
	 reset_stop_event_status();
	Wall_Bumper_Counter=0;
	 reset_rcon_remote();
	 set_direction_flag(Direction_Flag_Right);
	Stunk=0;
	Low_Power_Counter=0;
	 reset_rcon_status();

	 set_vac_speed();
	while(ros::ok())
	{
		usleep(10000);
		wall_dynamic_base(400);

#ifdef OBS_DYNAMIC
		obs_dynamic_base(300);
#endif

		/*if(Get_Room_Mode())//small room
		{
			if(WorkFinish_ByRoom(get_room_mode()))
			{
				set_clean_mode(Clean_Mode_Userinterface);
				break;
			}
		}*/

		/*-------------------------------------Mobility----------------------------------------------*/
		if(get_left_wheel_step()<500)
		{
			Temp_Mobility_Distance = get_move_distance();
		}
		else
		{
			if((get_move_distance()-Temp_Mobility_Distance)>500)
			{
				Temp_Mobility_Distance = get_move_distance();
				check_mobility();
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
				break;
			}
			//initialize_motor();
		}
		/*------------------------------------------------------Check Battery-----------------------*/
		if(check_bat_set_motors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power))//Low Battery Event
		{
			Low_Power_Counter++;
			if(Low_Power_Counter>10)
			{
				ROS_WARN("%s %d: Battery too low (< LOW_BATTERY_STOP_VOLTAGE)", __FUNCTION__, __LINE__);
				set_clean_mode(Clean_Mode_Userinterface);
				break;
			}
		}
		else
		{
			Low_Power_Counter=0;
		}
		/*------------------------------------------------------Stop event-----------------------*/
		if(stop_event())
		{
			stop_brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			reset_stop_event_status();
			set_clean_mode(Clean_Mode_Userinterface);
			break;
		}
		if(get_rcon_remote() > 0)
		{
			#ifdef STANDARD_REMOTE
			if(remote_key(Remote_Left))
			{
				stop_brifly();
				set_dir_left();
				set_wheel_speed(30, 30);
				//set_side_brush_pwm(60,60);
				usleep(100000);
				while(remote_key(Remote_Left))
				{
					reset_rcon_remote();
					usleep(100000);
					if (stop_event())
					{
						stop_brifly();
						// Key release detection, if user has not release the key, don't do anything.
						while (get_key_press() & KEY_CLEAN)
						{
							ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
							usleep(20000);
						}
						break;
					}
				}
				if (stop_event())
				{
					continue;
				}
				stop_brifly();
				reset_rcon_remote();
				reset_wheel_step();
				move_forward(10, 10);
			}
			if(remote_key(Remote_Right))
			{
				stop_brifly();
				set_dir_right();
				set_wheel_speed(30, 30);
				//set_side_brush_pwm(60,60);
				usleep(100000);
				while(remote_key(Remote_Right))
				{
					reset_rcon_remote();
					usleep(100000);
					if(stop_event())
					{
						stop_brifly();
						// Key release detection, if user has not release the key, don't do anything.
						while (get_key_press() & KEY_CLEAN)
						{
							ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
							usleep(20000);
						}
						// The touch status should be handled by the main while loop of Random_Running_Mode.
						break;
					}
				}
				stop_brifly();
				reset_rcon_remote();
				reset_wheel_step();
				move_forward(10, 10);
			}
			#endif
			#ifdef SCREEN_REMOTE
			if(Remote_Key(Remote_Left))
			{
				Stop_Brifly();
				Turn_Left(Turn_Speed,500);
				Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(10,10);
			}
			if(Remote_Key(Remote_Right))
			{
				Stop_Brifly();
				Turn_Right(Turn_Speed,400);
				Stop_Brifly();
				Reset_Rcon_Remote();
				Reset_Wheel_Step();
				Move_Forward(10,10);
			}
			#ifdef BLDC_INSTALL
			if(Remote_Key(Remote_Max))
			{
				Switch_VacMode();
//			Stop_Brifly();
//			Turn_Right(Turn_Speed,1800);
//			Stop_Brifly();
				Reset_Rcon_Remote();
//			Reset_Wheel_Step();
//			Move_Forward(10,10);
			}
			#else
			if(remote_key(Remote_Max))
			{
				Stop_Brifly();
				Turn_Right(Turn_Speed,1800);
				stop_brifly();
				reset_rcon_remote();
				reset_wheel_step();
				move_forward(10,10);
			}
			#endif
			#endif
			if(remote_key(Remote_Home)) //                                    Check Key Home
			{
				usleep(50000);
				set_clean_mode(Clean_Mode_GoHome);
				set_home_remote();
				reset_rcon_remote();
				return;
			}
//		if(remote_key(Remote_Random)) //                                    Check Key Home
//		{
//			set_clean_mode(Clean_Mode_WallFollow);
//				move_forward(10,10);
//				reset_rcon_remote();
//				break;
//		}
			if(remote_key(Remote_Spot))
			{
				reset_rcon_remote();
				Vac_Mode_Buffer = get_vac_mode();
				Temp_Dirt_Status=Random_Dirt_Event();
				set_vacmode(Vac_Mode_Buffer, false);
				set_vac_speed();
				if(Temp_Dirt_Status==1)
				{
					// stop_event triggered in Random_Dirt_Event
					// Key release detection, if user has not release the key, don't do anything.
					while (get_key_press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event status should be cleared.
					set_clean_mode(Clean_Mode_Userinterface);
					break;
				}
				reset_wheel_step();
			}
			reset_rcon_remote();
		}
		/*------------------------------------------------------Virtual wall Event-----------------------*/
#ifdef VIRTUAL_WALL
		if(is_work_finish(get_room_mode()))
		{
			if(is_near_station())
			{
				ROS_DEBUG("jump to the home mode in random running");
				set_clean_mode(Clean_Mode_GoHome);
				break;
			}
		}
		else if(Base_Wall_On)
		{
			Temp_Rcon_Status = get_rcon_status();
			if(Temp_Rcon_Status & 0x0f00)
			{
				stop_brifly();
				stop_brifly();
				if(get_left_wheel_step()<1000)
				{
					if(is_direction_right())
					{
						Turn_Right(Turn_Speed,1200);
						set_direction_flag(Direction_Flag_Right);
					}
					else
					{
						turn_left(Turn_Speed, 1200);
						set_direction_flag(Direction_Flag_Left);
					}
				}
				else
				{
					if(Temp_Rcon_Status & RconL_HomeT)
					{
						Turn_Right(Turn_Speed,800);
						set_direction_flag(Direction_Flag_Right);
					}
					else if(Temp_Rcon_Status & RconFL_HomeT)
					{
						random_back();
						Turn_Right(Turn_Speed,1120);
						set_direction_flag(Direction_Flag_Right);
					}
					else if(Temp_Rcon_Status & RconFR_HomeT)
					{
						random_back();
						turn_left(Turn_Speed, 1120);
						set_direction_flag(Direction_Flag_Left);
					}
					else if(Temp_Rcon_Status & RconR_HomeT)
					{
						turn_left(Turn_Speed, 1000);
						set_direction_flag(Direction_Flag_Left);
					}
				}
				move_forward(2, 2);
				reset_rcon_status();
				Base_Wall_On=0;
				reset_wheel_step();
				Wall_Small_Counter++;
				Wall_Mid_Counter++;
			}
		}
#endif
		/*------------------------------------------------------Virtual Wall--------------------*/
#ifdef VIRTUAL_WALL
		//Temp_Rcon_Status = get_rcon_status();
		if (is_virtual_wall_()) {
			reset_virtual_wall();
			Virtual_Wall_C++;
			if (Virtual_Wall_C > 1) {
				stop_brifly();
				//WalkAlongVirtualWall(Temp_Rcon_Status);
				Temp_Rcon_Status = 0;
			}
		} else {
			Virtual_Wall_NG++;
			if (Virtual_Wall_NG > 100) {
				Virtual_Wall_NG = 0;
				Virtual_Wall_C = 0;
			}
		}
#endif

		/*------------------------------------------------------Cliff Event-----------------------*/
		Temp_Cliff_Status= get_cliff_trig();
		if(Temp_Cliff_Status)
		{
			ROS_DEBUG("random running , cliff event!");
			set_wheel_speed(0, 0);
			set_dir_backward();
			usleep(30000);
			if(get_cliff_trig()||(get_left_wheel_step()<200))
			{
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
						break;
					}
				}
				if(cliff_event(Temp_Cliff_Status))
				{
					set_direction_flag(Direction_Flag_Left);
				}
				else
				{
					set_direction_flag(Direction_Flag_Right);
				}
				reset_wheel_step();
				reset_temp_pwm();
				Stunk++;
				Bumper_Counter++;
				Wall_Bumper_Counter+=2;
				Wall_Small_Counter++;
				Wall_Mid_Counter++;
				move_forward(5, 5);
				usleep(10000);
			}
			reset_left_wheel_step();
			set_dir_forward();
		}
		/*------------------------------------------------------Bumper Event-----------------------*/
		/*-----------------left bumper ------------------------------------*/
		if(get_bumper_status()&LeftBumperTrig)
		{
			ROS_DEBUG("random running , left bumpe event!");
			Avoid_Flag=0;
			Left_Wheel_Step_Buffer= get_left_wheel_step();
			add_average(get_left_wheel_step());
			if(get_left_wheel_step()>14000)
			{
				Wall_Bumper_Counter+=2;
			}
			else
			{
				Wall_Bumper_Counter+=3;
			}
			set_wheel_speed(0, 0);
			usleep(10000);
			Temp_Bumper_Status = get_bumper_status();
			random_back();
			if(is_bumper_jamed())break;

			Stunk++;
			if(Stunk>7)
			{
				Turn_Right(Turn_Speed,240);
				move_forward(10, 10);
				if(Out_Trap_Left())
				{
					// Out_Trap_Left() return 1 may be caused by stop_event.
					// Key release detection, if user has not release the key, don't do anything.
					while (get_key_press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event should be cleared.
					set_clean_mode(Clean_Mode_Userinterface);
					reset_stop_event_status();
					break;
				}
				Stunk=0;
				set_direction_flag(Direction_Flag_Right);
				Turn_Right(Turn_Speed,80);
			}
			else
			{
				if((Wall_Small_Counter>30)&&(!is_move_finished(300000)))
				{
					stop_brifly();
					Turn_Right(Turn_Speed,400);
					stop_brifly();
					if(Wall_Follow_Short(3000))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Small_Counter=0;
					Wall_Mid_Counter=0;
					reset_move_distance();
				}
				else if((Wall_Mid_Counter>40)||(is_move_finished(460000)))
				{
					stop_brifly();
					Turn_Right(Turn_Speed,400);
					stop_brifly();
					if(Wall_Follow_Short(1000))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
					reset_move_distance();
				}
				else if((Wall_Bumper_Counter>(Short_Wall_Trig+Wall_Bumper_Factor))&&(get_random_factor()<25))
				{
					stop_brifly();
					Turn_Right(Turn_Speed,400);
					stop_brifly();
					if(Wall_Follow_Short(get_average_move()))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
				}
				else
				{
					if(Left_Wheel_Step_Buffer<1000)
					{
						if(is_direction_right())
						{
							Turn_Right(Turn_Speed,660);
							set_direction_flag(Direction_Flag_Right);
						}
						else
						{
							turn_left(Turn_Speed, 660);
							set_direction_flag(Direction_Flag_Left);
						}
					}
					else
					{
						if(Temp_Bumper_Status == AllBumperTrig)
						{
							//Half_Turn_Right(Turn_Speed,800+get_random_factor()*7);
							Turn_Right(Turn_Speed,800+ get_random_factor()*7);
							Avoid_Flag=1;
						}
						else
						{
							if(get_random_factor()<60)
							{
								Turn_Right(Turn_Speed,800);
								if(Left_Bumper_Avoiding())Avoid_Flag=1;
								if (stop_event())
								{
									// Continue to let the main while loop to process the stop event status.
									continue;
								}
							}
							else
							{
								//Half_Turn_Right(Turn_Speed,700+get_random_factor()*9);
								Turn_Right(Turn_Speed,700+ get_random_factor()*9);
								Avoid_Flag=1;
							}
						}
						set_direction_flag(Direction_Flag_Right);
					}
				}
			}
			if(!Avoid_Flag)
			{
				reset_temp_pwm();
				reset_wheel_step();
			}
			else
			{
				if(get_left_wheel_step()> get_right_wheel_step())
				{
					set_right_wheel_step(get_left_wheel_step());
				}
				set_right_wheel_step(get_right_wheel_step() / 2);
			}
			set_mobility_step(0);
			Bumper_Counter++;
			Wall_Small_Counter++;
			Wall_Mid_Counter++;
		}

/*---------------------------------------------------------Right Bumper ----------------------------------*/
		if(get_bumper_status()&RightBumperTrig)
		{
			ROS_DEBUG("random running ,right bumper event ");
			Avoid_Flag=0;
			Left_Wheel_Step_Buffer= get_left_wheel_step();
			add_average(get_left_wheel_step());
			if(get_left_wheel_step()>14000)
			{
				Wall_Bumper_Counter+=1;
			}
			else
			{
				Wall_Bumper_Counter+=2;
			}
			set_wheel_speed(0, 0);
			usleep(10000);
			Temp_Bumper_Status = get_bumper_status();
			random_back();
			if(is_bumper_jamed())break;
			Stunk++;
			if(Stunk>7)
			{
				turn_left(Turn_Speed, 240);
				move_forward(10, 10);
				if(Out_Trap_Right())
				{
					// Out_Trap_Right() return 1 may be caused by stop_event.
					// Key release detection, if user has not release the key, don't do anything.
					while (get_key_press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event status should be cleared.
					set_clean_mode(Clean_Mode_Userinterface);
					reset_stop_event_status();
					break;
				}
				Stunk=0;
				set_direction_flag(Direction_Flag_Left);
				turn_left(Turn_Speed, 80);
			}
			else
			{
				if((Wall_Small_Counter>30)&&(!is_move_finished(300000)))
				{
					stop_brifly();
					Turn_Right(Turn_Speed,900);
					stop_brifly();
					if(Wall_Follow_Short(4000))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					reset_move_distance();
					Wall_Small_Counter=0;
				}
				else if((Wall_Mid_Counter>40)||(is_move_finished(460000)))
				{
					stop_brifly();
					Turn_Right(Turn_Speed,900);
					stop_brifly();
					if(Wall_Follow_Short(1000))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
					Wall_Small_Counter=0;
					reset_move_distance();
				}
				else if((Wall_Bumper_Counter>(Short_Wall_Trig+Wall_Bumper_Factor))&&(get_random_factor()<25))
				{
					stop_brifly();
					Turn_Right(Turn_Speed,900);
					stop_brifly();
					if(Wall_Follow_Short(get_average_move()))return;
					Stunk=0;
					Wall_Bumper_Counter=0;
					Wall_Mid_Counter=0;
				}
				else
				{
					if(Left_Wheel_Step_Buffer<1000)
					{
						if(is_direction_left())
						{
							turn_left(Turn_Speed, 660);
							set_direction_flag(Direction_Flag_Left);
						}
						else
						{
							Turn_Right(Turn_Speed,660);
							set_direction_flag(Direction_Flag_Right);
						}
					}
					else
					{
						if(Temp_Bumper_Status == AllBumperTrig)
						{
							//Half_Turn_Left(Turn_Speed,800+get_random_factor()*6);
							turn_left(Turn_Speed, 800 + get_random_factor() * 6);
							Avoid_Flag=1;
						}
						else
						{
							if(get_random_factor()<60)
							{
								turn_left(Turn_Speed, 800);
								if(Right_Bumper_Avoiding())Avoid_Flag=1;
								if (stop_event())
								{
									// Continue to let the main while loop to process the stop event status.
									continue;
								}
							}
							else
							{
								//Half_Turn_Left(Turn_Speed,700+get_random_factor()*8);
								turn_left(Turn_Speed, 700 + get_random_factor() * 8);
								Avoid_Flag=1;
							}
						}
						set_direction_flag(Direction_Flag_Left);
					}
				}
			}
			if(!Avoid_Flag)
			{
				reset_temp_pwm();
				reset_wheel_step();
			}
			else
			{
				if(get_left_wheel_step()> get_right_wheel_step())
				{
					set_right_wheel_step(get_left_wheel_step());
				}
				set_right_wheel_step(get_right_wheel_step() / 2);
			}
			set_mobility_step(0);
			Bumper_Counter++;
			Wall_Small_Counter++;
			Wall_Mid_Counter++;
		}

	/*------------------------------------------------------OBS_Status-----------------------*/
		if(Temp_OBS_Status)
		{
			ROS_DEBUG("random running ,obs event ");
			Temp_OBS_Status= get_obs_status();
			Left_Wheel_Step_Buffer= get_left_wheel_step();
			add_average(get_left_wheel_step());
			//random_back();
			set_wheel_speed(0, 0);
			reset_temp_pwm();
			usleep(10000);
			//stop_brifly();
			N_H_T=0;
			Reset_HalfTurn_Flag();
			if(is_left_wheel_reach(30))
			{
				if(Left_Wheel_Step_Buffer>14000)
				{
					Wall_Bumper_Counter+=1;
				}
				else
				{
					Wall_Bumper_Counter+=2;
				}
				stop_brifly();
				Bumper_Counter++;
			}

			if((Wall_Small_Counter>30)&&(!is_move_finished(300000)))
			{
				if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
				{
					Turn_Right(Turn_Speed,900);
					stop_brifly();
				}
				move_forward(5, 5);
				if(Wall_Follow_Short(4000))return;
				Stunk=0;
				Wall_Bumper_Counter=0;
				Wall_Small_Counter=0;
				reset_move_distance();
				Wall_Mid_Counter=0;
			}
			else if((Wall_Mid_Counter>40)||(is_move_finished(460000)))
			{
				if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
				{
					Turn_Right(Turn_Speed,900);
					stop_brifly();
				}
				move_forward(5, 5);
				if(Wall_Follow_Short(1000))return;
				Stunk=0;
				Wall_Bumper_Counter=0;
				Wall_Mid_Counter=0;
				Wall_Small_Counter=0;
				reset_move_distance();
			}
			else if((Wall_Bumper_Counter> (Short_Wall_Trig+Wall_Bumper_Factor))&&(get_random_factor()<25))
			{
				if(Temp_OBS_Status & (Status_Right_OBS|Status_Front_OBS))
				{
					Turn_Right(Turn_Speed,900);
					stop_brifly();
				}
				move_forward(5, 5);
				if(Wall_Follow_Short(get_average_move()))return;
				Stunk=0;
				Wall_Bumper_Counter=0;
				Wall_Mid_Counter=0;
				Wall_Small_Counter=0;
			}
			else
			{
				random_back();
				//stop_brifly();
				Stunk++;
				if(get_bumper_status())
				{
					random_back();
					if(is_bumper_jamed())break;
					stop_brifly();
				}
				if(Left_Wheel_Step_Buffer<1000)
				{
					if(is_direction_left())
					{
						if(Stunk>10)
						{
							if(Out_Trap_Right())
							{
								// Out_Trap_Right() return 1 may be caused by stop_event.
								// Key release detection, if user has not release the key, don't do anything.
								while (get_key_press() & KEY_CLEAN)
								{
									ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
									usleep(20000);
								}
								// Key relaesed, then the touch status and stop event status should be cleared.
								set_clean_mode(Clean_Mode_Userinterface);
								reset_stop_event_status();
								break;
							}
							Stunk=0;
							turn_left(Turn_Speed, 240);
						}
						else
						{
							if(Left_Wheel_Step_Buffer<300)
							{
								turn_left(Turn_Speed - 10, 400);
							}
							else
							{
								turn_left(Turn_Speed, 400);
							}
						}
						N_H_T=1;
						set_direction_flag(Direction_Flag_Left);
					}
					else
					{
						if(Stunk>10)
						{
							if(Out_Trap_Left())
							{
								// Out_Trap_Left() return 1 may be caused by stop_event.
								// Key release detection, if user has not release the key, don't do anything.
								while (get_key_press() & KEY_CLEAN)
								{
									ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
									usleep(20000);
								}
								// Key relaesed, then the touch status and stop event status should be cleared.
								set_clean_mode(Clean_Mode_Userinterface);
								reset_stop_event_status();
								break;
							}
							Stunk=0;
							Turn_Right(Turn_Speed,240);
						}
						else
						{
							if(Left_Wheel_Step_Buffer<300)
							{
								Turn_Right(Turn_Speed-10,400);
							}
							else
							{
								Turn_Right(Turn_Speed,400);
							}
						}
						N_H_T=1;
						set_direction_flag(Direction_Flag_Right);
					}
				}
				else
				{
					Random_Factor=Left_Wheel_Step_Buffer%2;
					if(Temp_OBS_Status==0xA2)// LFR
					{
						if(Random_Factor)
						{
							//Half_Turn_Left(Turn_Speed,750+get_random_factor()*8);
							turn_left(Turn_Speed, 750 + get_random_factor() * 8);
							set_direction_flag(Direction_Flag_Left);
						}
						else
						{
							//Half_Turn_Right(Turn_Speed,800+get_random_factor()*8);
							Turn_Right(Turn_Speed,800+ get_random_factor()*8);

							set_direction_flag(Direction_Flag_Right);
						}
					}
					else if(Temp_OBS_Status==0x72)//LF
					{
						if(Random_Factor)
						{
							//Half_Turn_Right(Turn_Speed,1200);
							Turn_Right(Turn_Speed,1200);
							set_direction_flag(Direction_Flag_Right);
						}
						else
						{
							//Half_Turn_Left(Turn_Speed,1200);
							turn_left(Turn_Speed, 750 + get_random_factor() * 8);
							set_direction_flag(Direction_Flag_Left);
						}
					}
					else if(Temp_OBS_Status&0x02)//L
					{
						if(Stunk>10)
						{
							if(Out_Trap_Left())
							{
								// Out_Trap_Left() return 1 may be caused by stop_event.
								// Key release detection, if user has not release the key, don't do anything.
								while (get_key_press() & KEY_CLEAN)
								{
									ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
									usleep(20000);
								}
								// Key relaesed, then the touch status and stop event status should be cleared.
								set_clean_mode(Clean_Mode_Userinterface);
								reset_stop_event_status();
								break;
							}
							Stunk=0;
							//Half_Turn_Right(Turn_Speed,240);
							Turn_Right(Turn_Speed,240);


						}
						else
						{
							if((Bumper_Counter%3)==0)
								//Half_Turn_Right(Turn_Speed,800+get_random_factor()*7);
								Turn_Right(Turn_Speed,800+ get_random_factor()*7);
							else
								//Half_Turn_Right(Turn_Speed,750+get_random_factor()*8);
								Turn_Right(Turn_Speed,750+ get_random_factor()*8);

						}
						set_direction_flag(Direction_Flag_Right);
					}
					else
					{
						if(Stunk>10)
						{
							if(Out_Trap_Right())
							{
								// Out_Trap_Right() return 1 may be caused by stop_event.
								// Key release detection, if user has not release the key, don't do anything.
								while (get_key_press() & KEY_CLEAN)
								{
									ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
									usleep(20000);
								}
								// Key relaesed, then the touch status and stop event status should be cleared.
								set_clean_mode(Clean_Mode_Userinterface);
								reset_stop_event_status();
								break;
							}
							Stunk=0;
							turn_left(Turn_Speed, 240);
						}
						else
						{
							Temp_OBS_Status>>=4;
							if(Temp_OBS_Status==0x08)
							{
								if(Random_Factor)
								{
									//Half_Turn_Left(Turn_Speed,750+get_random_factor()*7);
									turn_left(Turn_Speed, 750 + get_random_factor() * 7);
									set_direction_flag(Direction_Flag_Left);
								}
								else
								{
									//Half_Turn_Right(Turn_Speed,750+get_random_factor()*7);
									Turn_Right(Turn_Speed,750+ get_random_factor()*7);
									set_direction_flag(Direction_Flag_Right);
								}
							}
							else if(Temp_OBS_Status>8)
							{
								//Half_Turn_Left(Turn_Speed,800+get_random_factor()*7);
								turn_left(Turn_Speed, 800 + get_random_factor() * 7);
								set_direction_flag(Direction_Flag_Left);
							}
							else
							{
								if((Bumper_Counter%3)==0)
									//Half_Turn_Left(Turn_Speed,850+get_random_factor()*7);
									turn_left(Turn_Speed, 850 + get_random_factor() * 7);
								else
									//Half_Turn_Left(Turn_Speed,800+get_random_factor()*7);
									turn_left(Turn_Speed, 800 + get_random_factor() * 7);
								set_direction_flag(Direction_Flag_Left);
							}
						}
					}
				}
			}
			OBS_Cycle = 0;
			On_TrapOut_Flag=0;
			if(get_rcon_remote() <= 0)
			{
				if(get_obs_status())
				{
					if(!N_H_T)
					{
						stop_brifly();
						random_back();
						stop_brifly();
					}

					do
					{
						OBS_Cycle++;
						Stunk++;
						if(OBS_Cycle>3)
						{
							adjust_obs_value();
							Stunk=Stunk-OBS_Cycle;
							break;
						}
						if(is_direction_left())
						{
							if((Stunk>10)||(OBS_Cycle>7))
							{
								Stunk = 0;
								On_TrapOut_Flag=1;
								break;
							}
							else
							{
								obs_turn_left(Turn_Speed - 5, 400);
							}
							set_direction_flag(Direction_Flag_Left);
						}
						else
						{
							if((Stunk>10)||(OBS_Cycle>7))
							{
								Stunk = 0;
								On_TrapOut_Flag=2;
								break;
							}
							else
							{
								obs_turn_right(Turn_Speed - 5, 400);
							}
							set_direction_flag(Direction_Flag_Right);
						}
						usleep(10000);
					}while(get_obs_status());
					Reset_HalfTurn_Flag();
				}
			}
			if(On_TrapOut_Flag==1)
			{
				if(Out_Trap_Right())
				{
					// Out_Trap_Right() return 1 may be caused by stop_event.
					// Key release detection, if user has not release the key, don't do anything.
					while (get_key_press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event status should be cleared.
					set_clean_mode(Clean_Mode_Userinterface);
					reset_stop_event_status();
					break;
				}
				Turn_Right(Turn_Speed-5,240);
				move_forward(10, 10);
				reset_temp_pwm();
			}
			else if(On_TrapOut_Flag==2)
			{
				if(Out_Trap_Left())
				{
					// Out_Trap_Left() return 1 may be caused by stop_event.
					// Key release detection, if user has not release the key, don't do anything.
					while (get_key_press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					// Key relaesed, then the touch status and stop event status should be cleared.
					set_clean_mode(Clean_Mode_Userinterface);
					reset_stop_event_status();
					break;
				}
				Turn_Right(Turn_Speed-5,240);
				move_forward(10, 10);
				reset_temp_pwm();
			}
			if(!Is_HalfTurn_Flag())
			{
				//stop_brifly();
				move_forward(0, 0);
				reset_temp_pwm();
				//usleep(50000);
				reset_wheel_step();
				usleep(10000);
			}
			else
			{
				if(get_left_wheel_step()> get_right_wheel_step())
				{
					set_right_wheel_step(get_left_wheel_step());
				}
				set_right_wheel_step(get_right_wheel_step() / 2);
			}
			set_mobility_step(0);
			Wall_Small_Counter++;
			Wall_Mid_Counter++;
			Temp_OBS_Status=0;
		}
		/* -----------------------------Speed up ----------------------------------*/
		//if(Wall_Small_Counter>40)Wall_Small_Counter=0;
		if(Wall_Mid_Counter>70)
		{
			Wall_Mid_Counter=0;
			reset_move_distance();
		}
		if(is_left_wheel_reach(29000))
		{
			if(Spiral()){
				if (stop_event())
				{
					set_clean_mode(Clean_Mode_Userinterface);
					reset_stop_event_status();
				}
				break;
			}
			reset_wheel_step();
			Set_HalfTurn_Flag();
		}
		if(is_left_wheel_reach(24000))
		{
			Wall_Small_Counter=0;
		}
		if(is_left_wheel_reach(2500))
		{
			Stunk=0;
//			reset_bumper_error();
			if(get_left_brush_stall())set_left_brush_stall(0);
			if(get_right_brush_stall())set_right_brush_stall(0);
		}
		else if(is_left_wheel_reach(750))
		{
			Base_Wall_On=1;
			//Set_Left_Brush(ENABLE);
			//Set_Right_Brush(ENABLE);
		}

	/*-------------------------------------------------------------------------------------------------------------------*/
		if((get_cliff_trig()==0)&&(get_bumper_status()==0))
		{
			if(is_obs_near())
			{
				if(Moving_Speed>30)
				{
					Moving_Speed--;
				}
				set_right_wheel_step(400);
			}


			if(get_obs_status())
			{
				if(Moving_Speed>10)
				{
					Disacc++;
					if(Disacc>3)
					{
						Disacc=0;
						Moving_Speed--;
					}
				}
				OBS_Distance_Counter++;
				OBS_Delay = Moving_Speed*OBS_Distance_Counter;
				if(OBS_Delay>1000)
				{
					OBS_Distance_Counter=0;
					Temp_OBS_Status = get_obs_status();
				}
				set_right_wheel_step(200);
			}
			else
			{
				OBS_Distance_Counter=0;
				Temp_OBS_Status=0;
				Moving_Speed=(get_right_wheel_step()/80)+20;
				if(Is_HalfTurn_Flag())
				{
					Reset_HalfTurn_Flag();
					if(Moving_Speed<35)Moving_Speed=35;
				}
				else
				{
					if(Moving_Speed<25)Moving_Speed=25;
				}
				if(Moving_Speed>Max_Speed)Moving_Speed=Max_Speed;
			}
			move_forward(Moving_Speed, Moving_Speed);
		}
	}
}

/*------------------------------------------------------Out Trap Right--------------------------------------------------------*/
uint8_t Out_Trap_Right(void)
{
	int32_t R=0;
	uint8_t Motor_Check_Code=0;
	uint32_t Bump_Counter=0;
	reset_wheel_step();
	//reset_move_distance();
	reset_rcon_status();
	reset_wall_step();
	while(ros::ok())
	{
		usleep(10000);
		/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code= check_motor_current();
		if(Motor_Check_Code)
		{
			if(self_check(Motor_Check_Code))
			{
				set_clean_mode(Clean_Mode_Userinterface);
				return 1;
			}
			//initialize_motor();
		}
		/*-------------------------------------------------------Wheel ---------------------------------------*/
		if(get_left_wall_step() - get_right_wall_step())
		{
			R= get_left_wall_step() - get_right_wall_step();
			if(R>7500)//turn over 3600 degree
			{
				return 0;
			}
		}
		/*------------------------------------------------------Stop event-----------------------*/
		if(stop_event())
		{
			stop_brifly();
			return 1;
		}

		if(remote_key(Remote_Left))
		{
			Turn_Right(Turn_Speed,240);
			move_forward(30, 30);
			reset_rcon_remote();
			return 0;
		}
		if(get_rcon_status()&0x0f00)return 0;

#ifdef VIRTUAL_WALL
		if (is_virtual_wall_()) {
			return 0;
		}
#endif

		if(check_bat_set_motors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power))//Low Battery Event
		{
			set_clean_mode(Clean_Mode_Userinterface);
			return 1;
		}
		if(get_bumper_status()&LeftBumperTrig)
		{
			stop_brifly();
			wall_move_back();
			if(is_bumper_jamed())return 1;
			turn_left(Turn_Speed - 5, 800);
			stop_brifly();
			reset_left_wheel_step();
			Bump_Counter++;
		}

		if(get_bumper_status()&RightBumperTrig)
		{
			stop_brifly();
			wall_move_back();
			if(is_bumper_jamed())return 1;
			turn_left(Turn_Speed - 8, 150);
			stop_brifly();
			reset_left_wheel_step();
			Bump_Counter++;
		}
		if(Bump_Counter>15)return 0;
		if(is_front_close())
		{
			stop_brifly();
			turn_left(Turn_Speed - 8, 640);
			stop_brifly();
			reset_left_wheel_step();
		}
		if(is_left_wheel_reach(5000))return 0;
		if(get_right_wall_step()>12000)return 0;

		if(get_cliff_trig())
		{
			return 0;
		}
		if(get_left_wheel_step()<130)
		{
			move_forward(15, 15);
		}
		else
		{
			move_forward(38, 6);
		}
	}
}

/*------------------------------------------------------Out Trap Left--------------------------------------------------------*/
uint8_t Out_Trap_Left(void)
{
	int32_t R=0;
	uint8_t Motor_Check_Code=0;
	uint32_t Bump_Counter=0;
	reset_wheel_step();
	//reset_move_distance();
	reset_rcon_status();
	reset_wall_step();
	while(ros::ok())
	{
		usleep(10000);
	/*------------------------------------------------------Check Current-----------------------*/
		Motor_Check_Code= check_motor_current();
		if(Motor_Check_Code)
		{
			if(self_check(Motor_Check_Code))
			{
				set_clean_mode(Clean_Mode_Userinterface);
				return 1;
			}
		//initialize_motor();
		}
	/*-------------------------------------------------------Wheel ---------------------------------------*/
		if(get_right_wall_step()> get_left_wall_step())
		{
			R= get_right_wall_step()- get_left_wall_step();
			if(R>7500)//turn over 3600 degree
			{
				return 0;
			}
		}
		/*------------------------------------------------------Stop event-----------------------*/
		if(stop_event())
		{
			stop_brifly();
			return 1;
		}

		if(remote_key(Remote_Right))
		{
			Turn_Right(Turn_Speed,300);
			move_forward(30, 30);
			reset_rcon_remote();
			return 0;
		}
		if(get_rcon_status()&0x0f00)return 0;

#ifdef VIRTUAL_WALL
		if (is_virtual_wall_()) {
			return 0;
		}
#endif

		if(check_bat_set_motors(Clean_Vac_Power, Clean_SideBrush_Power, Clean_MainBrush_Power))//Low Battery Event
		{
			set_clean_mode(Clean_Mode_Userinterface);
			return 1;
		}
		if(get_bumper_status()&RightBumperTrig)
		{
			stop_brifly();
			wall_move_back();
			if(is_bumper_jamed())return 1;
			Turn_Right(Turn_Speed-5,1025);
			stop_brifly();
			reset_right_wheel_step();
			Bump_Counter++;
		}

		if(get_bumper_status()&LeftBumperTrig)
		{
			stop_brifly();
			wall_move_back();
			if(is_bumper_jamed())return 1;
			Turn_Right(Turn_Speed-8,150);
			stop_brifly();
			reset_right_wheel_step();
			Bump_Counter++;
		}
		if(Bump_Counter>15)return 0;
		if(is_front_close())
		{
			stop_brifly();
			Turn_Right(Turn_Speed-8,800);
			stop_brifly();
			reset_right_wheel_step();
		}
		if(is_right_wheel_reach(5000))return 0;
		if(get_left_wall_step()>12000)return 0;

		if(get_cliff_trig())
		{
			return 0;
		}
		if(get_right_wheel_step()<130)
		{
			move_forward(15, 15);
		}
		else
		{
			move_forward(6, 38);
		}
	}
}


uint8_t Left_Bumper_Avoiding(void)
{
	uint16_t Counter_Watcher=0;
	uint32_t Temp_A_Speed=0;
	//stop_brifly();
	reset_wheel_step();
	move_forward(5, 20);
	while(get_right_wheel_step()<2000&&ros::ok())
	{
		usleep(100);
		Temp_A_Speed = get_right_wheel_step()/8 + 20;
		if(Temp_A_Speed<20)Temp_A_Speed=20;
		if(Temp_A_Speed>42)Temp_A_Speed=42;
		move_forward(Temp_A_Speed / 4, Temp_A_Speed);

		Counter_Watcher++;
		if(Counter_Watcher>50000)
		{
			if(is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
				return 0;
			}
			return 0;
		}
		if(stop_event())
		{
			stop_brifly();
			return 0;
		}
		if(get_rcon_remote() > 0)
		{
			reset_rcon_remote();
			return 0;
		}
		if(get_bumper_status())break;
		if(get_obs_status())break;
		if(get_cliff_trig())break;
		if((check_motor_current()==Check_Left_Wheel)||(check_motor_current()==Check_Right_Wheel))return 0;
		if(get_rcon_status()&0x0f00)return 0;

#ifdef VIRTUAL_WALL
		if (is_virtual_wall_()) {
			break;
		}
#endif

	}
	if(get_right_wheel_step()>=2000)return 1;
	stop_brifly();
	stop_brifly();
	return 0;
}

uint8_t Right_Bumper_Avoiding(void)
{
	uint16_t Counter_Watcher=0;
	uint32_t Temp_A_Speed=0;
	//stop_brifly();
	reset_wheel_step();
	move_forward(20, 5);
	while(get_left_wheel_step()<2000)
	{
		usleep(100);
		Temp_A_Speed = get_left_wheel_step()/8 + 20;
		if(Temp_A_Speed<20)Temp_A_Speed=20;
		if(Temp_A_Speed>42)Temp_A_Speed=42;
		move_forward(Temp_A_Speed, Temp_A_Speed / 4);
		Counter_Watcher++;
		if(Counter_Watcher>50000)
		{
			if(is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
				return 0;
			}
			return 0;
		}
		if(stop_event())
		{
			stop_brifly();
			return 0;
		}
		if(get_rcon_remote() > 0)
		{
			reset_rcon_remote();
			return 0;
		}
		if(get_bumper_status())break;
		if(get_obs_status())break;
		if(get_cliff_trig())break;
		if((check_motor_current()==Check_Left_Wheel)||(check_motor_current()==Check_Right_Wheel))return 0;
		if(get_rcon_status()&0x0f00)return 0;

#ifdef VIRTUAL_WALL
		if (is_virtual_wall_()) {
			break;
		}
#endif

	}
	if(get_left_wheel_step()>=2000)return 1;
	stop_brifly();
	stop_brifly();
	return 0;
}

/*-------- Turn Left ------------------------*/
void Half_Turn_Left(uint16_t speed,uint16_t angle)
{
	uint8_t H_S=0;
	uint16_t Counter_Watcher=0;
	uint8_t Temp_H_Flag=0;
	turn_left(speed, angle / 2);
	if(get_rcon_remote() > 0)
	{
		reset_rcon_remote();
		return;
	}
	//set_dir_forward();
	//Set_LeftTPWM(0);

	reset_wheel_step();
	reset_temp_pwm();
	usleep(10000);
	move_forward(0, speed);
	Counter_Watcher=0;
	Reset_HalfTurn_Flag();
	ROS_DEBUG("half turn left angle :%d",angle);
	while(get_right_wheel_step()<angle && ros::ok())
	{
		usleep(100);
		Counter_Watcher++;
		if(Counter_Watcher>40000)
		{
			if(is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
			}
			return;
		}
		if(is_turn_remote())Temp_H_Flag=1;
		if(get_bumper_status())Temp_H_Flag=1;
		if(is_front_close())
		{
			set_wheel_speed(0, 20);
		}
		else
		{
			H_S = get_right_wheel_step()/8 + speed;
			if(H_S>42)H_S=42;
			set_wheel_speed(0, H_S);
		}
		if(get_cliff_trig())Temp_H_Flag=1;
		if(stop_event())
		{
			Temp_H_Flag=1;
		}
		if((check_motor_current()==Check_Left_Wheel)||(check_motor_current()==Check_Right_Wheel))Temp_H_Flag=1;
		if(Temp_H_Flag)break;

#ifdef VIRTUAL_WALL
		if (is_virtual_wall_()) {
			break;
		}
#endif

	}
	if(Temp_H_Flag)
	{
		set_wheel_speed(0, 0);
		return;
	}
	Set_HalfTurn_Flag();
}

/*-------- Tur_Right ------------------------*/
void Half_Turn_Right(uint16_t speed,uint16_t angle)
{
	uint16_t H_S=0;
	uint16_t Counter_Watcher=0;
	uint8_t Temp_H_Flag=0;
	Turn_Right(speed,angle/2);
	if(get_rcon_remote() > 0)
	{
		reset_rcon_remote();
		return;
	}
//	set_dir_forward();
//	Set_TempPWM(20,0);
	//Set_RightTPWM(0);
	 reset_wheel_step();
	 reset_temp_pwm();
	usleep(10000);
	 move_forward(speed, 0);
  //set_wheel_speed(42,0);
	Counter_Watcher=0;
	Reset_HalfTurn_Flag();
	ROS_DEBUG("half turn right angle :%d",angle);
	while(get_left_wheel_step()<angle && ros::ok())
	{
		usleep(100);
		Counter_Watcher++;
		if(Counter_Watcher>40000)
		{
			if(is_encoder_fail())
			{
				set_error_code(Error_Code_Encoder);
			}
			return;
		}
		if(is_turn_remote())Temp_H_Flag=1;
		if(get_bumper_status())Temp_H_Flag=1;
		if(is_front_close())
		{
			set_wheel_speed(20, 0);
		}
		else
		{
			H_S = get_left_wheel_step()/8 + speed;
			if(H_S>42)H_S=42;
			set_wheel_speed(H_S, 0);
		}
		if(get_cliff_trig())Temp_H_Flag=1;
		if(stop_event())
		{
			Temp_H_Flag=1;
		}
		if((check_motor_current()==Check_Left_Wheel)||(check_motor_current()==Check_Right_Wheel))Temp_H_Flag=1;
		if(Temp_H_Flag)break;

#ifdef VIRTUAL_WALL
		if (is_virtual_wall_()) {
			break;
		}
#endif

	}
	if(Temp_H_Flag)
	{
		set_wheel_speed(0, 0);
		return;
	}
	Set_HalfTurn_Flag();
}

