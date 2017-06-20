#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <wav.h>

#include "movement.h"
#include "charger.hpp"
#include "robot.hpp"
#include "gyro.h"
#include "random_runing.h"
#include "core_move.h"
#include "event_manager.h"

#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed 18
#endif


/*---------------------------------------------------------------- Charge Function ------------------------*/
uint8_t g_stop_charge_counter = 0;
void Charge_Function(void)
{

	volatile uint8_t Display_Switch=1;

	bool Battery_Full = false;

	#ifdef ONE_KEY_DISPLAY

	uint8_t One_Display_Counter=0;

	#endif

	// This counter is for debug message.
	uint8_t Show_Batv_Counter=0;

	// This counter is for checking if battery enough to continue cleaning.
	uint16_t Bat_Enough_To_Continue_Cleaning_Counter = 0;

	// This counter is for avoiding occasionly is_charge_on return 0 when robot is charging, cause it will stop charger mode.
	uint8_t Stop_Charge_Counter = 0;

	bool eh_status_now=false, eh_status_last=false;
	set_led(100, 100);
	set_start_charge();
	wav_play(WAV_BATTERY_CHARGE);
	set_plan_status(0);
	uint16_t bat_v;
	ROS_INFO("[gotocharger.cpp] Start charger mode.");
	charge_register_event();
	while(ros::ok())
	{
		usleep(20000);
		bat_v = get_battery_voltage();

		if (robot::instance()->isLowBatPaused())
		{
			if (bat_v >= CONTINUE_CLEANING_VOLTAGE)
			{
				Bat_Enough_To_Continue_Cleaning_Counter++;
				//ROS_INFO("Bat_Enough_To_Continue_Cleaning_Counter = %d.", Bat_Enough_To_Continue_Cleaning_Counter);
			}
			else
			{
				Bat_Enough_To_Continue_Cleaning_Counter = 0;
			}

			if (Bat_Enough_To_Continue_Cleaning_Counter > 500)// About 10 seconds.
			{
				ROS_INFO("Robot finish charging, continue cleaning.");
				set_clean_mode(Clean_Mode_Navigation);
				break;
			}
		}
		if(Show_Batv_Counter > 250)
		{
			ROS_INFO(" In charge mode looping , battery voltage %5.2f V.",bat_v/100.0);
			Show_Batv_Counter = 0;
		}
		else
		{
			Show_Batv_Counter++;
		}
//		#ifdef SCREEN_REMOTE
//		if(Remote_Clock_Received())
//		{
//			Set_Remote_Schedule();
//		}
//		#endif

		if(event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			continue;
		}
		if(g_stop_charge_counter > 0)g_stop_charge_counter--;
		if(g_stop_charge_counter == 0)	//disconnect to charger for 0.5s, exit charge mode
		{
			if(robot::instance()->isLowBatPaused())
			{
				ROS_INFO("[gotocharger.cpp] Exit charger mode and continue cleaning.");
				set_clean_mode(Clean_Mode_Navigation);
				break;
			}

			ROS_INFO("[gotocharger.cpp] Exit charger mode and go to userinterface mode.");
			set_clean_mode(Clean_Mode_Userinterface);
			break;
		}
		if (get_clean_mode() == Clean_Mode_Navigation)
			break;

		/*-----------------------------------------------------Schedul Timer Up-----------------*/
//		if(Is_Alarm())
//		{
//			Reset_Alarm();
//			if(is_on_charger_stub())
//			{
//				set_vacmode(Vac_Normal);
//				set_room_mode(Room_Mode_Large);
//				set_clean_mode(Clean_Mode_Navigation);
//				break;
//			}
//		}

		#ifdef ONE_KEY_DISPLAY
		if (check_bat_full() && !Battery_Full)
		{
			Battery_Full = true;
			set_led(0, 0);
			wav_play(WAV_BATTERY_CHARGE_DONE);
		}

		if (!Battery_Full)
		{
			// For displaying breathing LED
			if(Display_Switch)
			{
				One_Display_Counter+=2;
				if(One_Display_Counter>98)Display_Switch=0;
			}
			else
			{
				One_Display_Counter-=2;
				if(One_Display_Counter<2)Display_Switch=1;
			}

			set_led(One_Display_Counter, One_Display_Counter);
		}
		#endif

	}
	charge_unregister_event();
	set_stop_charge();
	// Wait for 20ms to make sure stop charging command has been sent.
	usleep(20000);
}

/*----------------------------------------------------------------GO Home  ----------------*/
void GoHome(void)
{

	uint32_t Receive_Code = 0;
//	move_forward(9,9);
//	set_side_brush_pwm(30,30);
//	set_main_brush_pwm(30);
	reset_rcon_status();
	//delay(1500);
//	wav_play(WAV_BACK_TO_CHARGER);
	// This is for calculating the robot turning.
	float Current_Angle;
	float Last_Angle;
	float Angle_Offset;
	// This step is for counting angle change when the robot turns.
	float Gyro_Step = 0;

	set_led(100, 100);
	set_side_brush_pwm(30, 30);
	set_main_brush_pwm(30);

	stop_brifly();
	reset_rcon_status();
	// Save the start angle.
	Last_Angle = robot::instance()->getAngle();
	// Enable the charge function
	set_start_charge();

	while(Gyro_Step < 360)
	{
		// For GoHome(), if reach the charger stub during turning, should stop immediately.
		if (is_charge_on())
		{
			ROS_DEBUG("%s %d: Reach charger at first turn.", __FUNCTION__, __LINE__);
			disable_motors();
			usleep(100000);
			if (is_charge_on())
			{
				set_clean_mode(Clean_Mode_Charging);
				break;
			}
		}
		if (stop_event())
		{
			ROS_WARN("%s %d: stop_event in turning 360 degrees to find charger signal.", __FUNCTION__, __LINE__);
			set_clean_mode(Clean_Mode_Userinterface);
			disable_motors();
			break;
		}

		//prompt for useless remote command
		if (get_rcon_remote() > 0) {
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (get_rcon_remote() & (Remote_Clean)) {
			} else {
				beep_for_command(false);
				reset_rcon_remote();
			}
		}

		if(get_bumper_status())
		{
			random_back();
			if(is_bumper_jamed())
			{
				set_clean_mode(Clean_Mode_Userinterface);
				break;
			}
		}
		Receive_Code = get_rcon_status();
		reset_rcon_status();
		if(Receive_Code&RconFL_HomeR)//FL H_R
		{
			ROS_INFO("Start with FL-R.");
			turn_left(Turn_Speed, 900);
			stop_brifly();
			Around_ChargerStation(0);
			break;
		}
		if(Receive_Code&RconFR_HomeL)//FR H_L
		{
			ROS_INFO("Start with FR-L.");
			Turn_Right(Turn_Speed,900);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}

		if(Receive_Code&RconFL_HomeL)//FL H_L
		{
			ROS_INFO("Start with FL-L.");
			Turn_Right(Turn_Speed,900);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconFR_HomeR)//FR H_R
		{
			ROS_INFO("Start with FR-R.");
			turn_left(Turn_Speed, 900);
			stop_brifly();
			Around_ChargerStation(0);
			break;
		}
		if(Receive_Code&RconFL2_HomeR)//FL2 H_R
		{
			ROS_INFO("Start with FL2-R.");
			turn_left(Turn_Speed, 850);
			stop_brifly();
			Around_ChargerStation(0);
			break;
		}
		if(Receive_Code&RconFR2_HomeL)//FR2 H_L
		{
			ROS_INFO("Start with FR2-L.");
			Turn_Right(Turn_Speed,850);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}

		if(Receive_Code&RconFL2_HomeL)//FL2 H_L
		{
			ROS_INFO("Start with FL2-L.");
			Turn_Right(Turn_Speed,600);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconFR2_HomeR)//FR2 H_R
		{
			ROS_INFO("Start with FR2-R.");
			turn_left(Turn_Speed, 600);
			stop_brifly();
			Around_ChargerStation(0);
			break;
		}

		if(Receive_Code&RconL_HomeL)// L  H_L
		{
			ROS_INFO("Start with L-L.");
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconR_HomeR)// R  H_R
		{
			ROS_INFO("Start with R-R.");
			Around_ChargerStation(0);
			break;
		}

		if(Receive_Code&RconL_HomeR)// L  H_R
		{
			ROS_INFO("Start with L-R.");
			turn_left(Turn_Speed, 1500);
			Around_ChargerStation(0);
			break;
		}
		if(Receive_Code&RconR_HomeL)// R  H_L
		{
			ROS_INFO("Start with R-L.");
			Turn_Right(Turn_Speed,1500);
			Around_ChargerStation(1);
			break;
		}
/*--------------------------HomeT-----------------*/
		if(Receive_Code&RconFL_HomeT)//FL H_T
		{
			ROS_INFO("Start with FL-T.");
			Turn_Right(Turn_Speed,600);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconFR_HomeT)//FR H_T
		{
			ROS_INFO("Start with FR-T.");
			Turn_Right(Turn_Speed,800);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}

		if(Receive_Code&RconFL2_HomeT)//FL2 H_T
		{
			ROS_INFO("Start with FL2-T.");
			Turn_Right(Turn_Speed,600);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconFR2_HomeT)//FR2 H_T
		{
			ROS_INFO("Start with FR2-T.");
			Turn_Right(Turn_Speed,800);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}

		if(Receive_Code&RconL_HomeT)// L  H_T
		{
			ROS_INFO("Start with L-T.");
			Turn_Right(Turn_Speed,1200);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconR_HomeT)// R  H_T
		{
			ROS_INFO("Start with R-T.");
			Turn_Right(Turn_Speed,1200);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}

/*--------------BL BR---------------------*/
		if((Receive_Code&RconBL_HomeL))//BL H_L    //OK
		{
			ROS_INFO("Start with BL-L.");
			turn_left(30, 800);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}
		if((Receive_Code&RconBR_HomeR))//BR H_L R  //OK
		{
			ROS_INFO("Start with BR-R.");
			Turn_Right(30,800);
			stop_brifly();
			Around_ChargerStation(0);
			break;
		}

		if((Receive_Code&RconBL_HomeR))//BL H_R
		{
			ROS_INFO("Start with BL-R.");
			turn_left(30, 800);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}
		if((Receive_Code&RconBR_HomeL))//BL H_L R
		{
			ROS_INFO("Start with BR-L.");
			Turn_Right(30,800);
			stop_brifly();
			Around_ChargerStation(0);
			break;
		}

		if((Receive_Code&RconBL_HomeT))//BL H_T
		{
			ROS_INFO("Start with BL-T.");
			turn_left(30, 300);
			stop_brifly();
			Around_ChargerStation(1);
			break;
		}
		if((Receive_Code&RconBR_HomeT))//BR H_T
		{
			ROS_INFO("Start with BR-T.");
			Turn_Right(30,300);
			stop_brifly();
			Around_ChargerStation(0);
			break;
		}
		usleep(50000);
		Current_Angle = robot::instance()->getAngle();
		Angle_Offset = Current_Angle - Last_Angle;
		ROS_DEBUG("Current_Angle = %f, Last_Angle = %f, Angle_Offset = %f, Gyro_Step = %f.", Current_Angle, Last_Angle, Angle_Offset, Gyro_Step);
		if (Angle_Offset > 0)
		{
			// For passing the boundary of angle range. e.g.(179 - (-178))
			if (Angle_Offset >= 180)
				Angle_Offset -= 360;
			else
				// For sudden change of angle, normally it shouldn't turn back for a few degrees, however if something hit robot to opposit degree, we can skip that angle change.
				Angle_Offset = 0;
		}
		Gyro_Step += (-Angle_Offset);
		Last_Angle = Current_Angle;

		set_dir_right();
		set_wheel_speed(10, 10);
	}

	if (Gyro_Step >= 360)
		set_clean_mode(Clean_Mode_Userinterface);

	// If robot didn't reach the charger, go back to userinterface mode.
	if(get_clean_mode() != Clean_Mode_Charging && get_clean_mode() != Clean_Mode_GoHome)
	{
		extern std::list <Point32_t> g_home_point;
		if (!stop_event() && g_home_point.empty())
		{
			set_led(100, 0);
			stop_brifly();
			wav_play(WAV_BACK_TO_CHARGER_FAILED);
		}
	}

}

void Around_ChargerStation(uint8_t Dir)
{
//	uint32_t Temp_Steps=0;
	uint8_t Temp_Position=0;
//	uint8_t Mobility_Temp_Error=0;
//	uint32_t Temp_Mobility_Distance=0;
	uint32_t Temp_Rcon_Status=0;
	uint8_t Signal_Counter=0;
	uint32_t No_Signal_Counter=0;
	uint32_t N_Around_LRSignal=0;
	uint8_t Bumper_Counter=0;
	uint8_t Cliff_Counter = 0;
	move_forward(9, 9);
	set_side_brush_pwm(30, 30);
	set_main_brush_pwm(30);
	set_bldc_speed(Vac_Speed_NormalL);
	//delay(500);
	reset_rcon_status();
	reset_wheel_step();
	reset_move_distance();
	ROS_INFO("%s, %d: Call Around_ChargerStation with dir = %d.", __FUNCTION__, __LINE__, Dir);
	while(1)
	{
//		if(get_left_wheel_step()<500)
//		{
//			Mobility_Temp_Error=0;
//			Temp_Mobility_Distance = get_move_distance();
//		}
//		else
//		{
//			if((get_move_distance()-Temp_Mobility_Distance)>500)
//			{
//				Temp_Mobility_Distance = get_move_distance();
//				if(get_mobility_step()<1)
//				{
//					Mobility_Temp_Error++;
//					if(Mobility_Temp_Error>3)
//					{
//						set_clean_mode(Clean_Mode_GoHome);
//						return;
//					}
//				}
//				else
//				{
//					Mobility_Temp_Error=0;
//				}
//				reset_mobility_step();
//			}
//		}

		if(get_cliff_trig())
		{
			if (get_cliff_trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
			{
				disable_motors();
				ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
				wav_play(WAV_ERROR_LIFT_UP);
				set_clean_mode(Clean_Mode_Userinterface);
				return;
			}

			while (get_cliff_trig() && Cliff_Counter < 3)
			{
				// Move back until escape cliff triggered.
				move_back();
				Cliff_Counter++;
				usleep(40000);
				if (get_cliff_trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
				{
					disable_motors();
					ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
					wav_play(WAV_ERROR_LIFT_UP);
					set_clean_mode(Clean_Mode_Userinterface);
					return;
				}
			}
			if (Cliff_Counter == 3)
			{
				set_clean_mode(Clean_Mode_Userinterface);
			}
			else
			{
				turn_left(Turn_Speed, 1750);
				move_forward(9, 9);
				set_clean_mode(Clean_Mode_GoHome);
			}
			return;
		}

//		/*------------------------------------------------------Stop event-----------------------*/
		if(stop_event())
		{
			stop_brifly();
			if (stop_event())
			{
				//beep(5, 20, 0, 1);
				// Key release detection, if user has not release the key, don't do anything.
				while (get_key_press() & KEY_CLEAN)
				{
					ROS_WARN("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
					usleep(20000);
				}
				// Do not reset g_stop_event_status is for when robot is going home in navigation mode,
				// when stop event status is on,
				// it will know and won't go to next home point.
				if (! robot::instance()->isLowBatPaused())
					if (! robot::instance()->isManualPaused())
						reset_stop_event_status();
			}
			// If key pressed, go back to user interface mode.
			set_clean_mode(Clean_Mode_Userinterface);
			return;
		}

		//prompt for useless remote command
		if (get_rcon_remote() > 0) {
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (get_rcon_remote() & (Remote_Clean)) {
			} else {
				beep_for_command(false);
				reset_rcon_remote();
			}
		}

		if(check_bat_stop())
		{
			ROS_WARN("%s %d: Battery too low (< LOW_BATTERY_STOP_VOLTAGE)", __FUNCTION__, __LINE__);
			//delay(10000);
			usleep(1000000);
			set_clean_mode(Clean_Mode_Sleep);
			return;
		}

//		if(Home_Check_Current())return;

		if(get_bumper_status())
		{
			Bumper_Counter++;
			random_back();
			if(is_bumper_jamed())
			{
				return;
			}
			if(Dir)
			{
				// Robot at left side of charger stub.
				turn_left(Turn_Speed, 1800);
			}
			else
			{
				// Robot at right side of charger stub.
				Turn_Right(Turn_Speed,1800);
			}
			reset_rcon_status();
			move_forward(10, 10);
			//???
			Dir = 1-Dir;
			if(Bumper_Counter>1)
			{
				set_clean_mode(Clean_Mode_GoHome);
				return ;
			}
			//reset_wheel_step();
			No_Signal_Counter=0;
		}

		if(is_charge_on())
		{
			ROS_DEBUG("%s %d: is_charge_on!!", __FUNCTION__, __LINE__);
			disable_motors();
			stop_brifly();
//			delay(2000);
			usleep(200000);
			if(is_charge_on())
			{
				//delay(5000);
				usleep(200000);
				if(is_charge_on())
				{
//					Reset_Error_Code();
					set_clean_mode(Clean_Mode_Charging);
//					beep(2, 25, 0, 1);
//					reset_rcon_remote();
					return;
				}
			}
			else if(turn_connect())
			{
				set_clean_mode(Clean_Mode_Charging);
//				reset_rcon_remote();
				return;
			}
			else
			{
				set_side_brush_pwm(30, 30);
				set_main_brush_pwm(0);
				////Back(30,800);
				//Back(30,300);
				quick_back(30,300);
				set_main_brush_pwm(30);
				stop_brifly();
			}
			if (stop_event())
			{
				ROS_WARN("%s %d: stop_event in turn_connect.", __FUNCTION__, __LINE__);
				disable_motors();
				return;
			}
		}

		Temp_Rcon_Status = get_rcon_status();
		if(Temp_Rcon_Status)
		{
			No_Signal_Counter=0;
			reset_rcon_status();
		}
		else
		{
			No_Signal_Counter++;
			if(No_Signal_Counter>80)
			{
				//beep(1, 25, 75, 3);
				ROS_WARN("No charger signal received.");
				set_clean_mode(Clean_Mode_GoHome);
				return ;
			}
		}
		/*
		if(Temp_Rcon_Status&0x4000)
		{
			Turn_Right(30,2200);
			move_forward(10,10);
			Dir=1-Dir;
		}*/
		ROS_DEBUG("%s %d Check DIR: %d, and do something", __FUNCTION__, __LINE__, Dir);
		if(Dir == 1)//10.30
		{
//			if(get_right_wheel_step()>20000)
//			{
//				stop_brifly();
//				Turn_Right(Turn_Speed,2200);
//				set_clean_mode(Clean_Mode_GoHome);
//				return ;
//			}

			if(Temp_Rcon_Status&RconL_HomeT)  //L_T
			{
//				if((++LTSignal_Count)>=3)
//				{
//					LTSignal_Count = 0;
//					LLSignal_Count = 0;
					ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
				move_forward(19, 5);
//					Uniform_Forward(32,22);	//OK(19,5)
//					Delay_Arounding(50);
					usleep(100000);
//				}
			}
			else if(Temp_Rcon_Status&RconL_HomeL)  //L_L  9 18
			{
//				if((++LLSignal_Count)>=3)
//				{
//					LLSignal_Count = 0;
					ROS_DEBUG("%s, %d: Detect L-L.", __FUNCTION__, __LINE__);
				move_forward(19, 5);
//					Uniform_Forward(28,22);
//					Delay_Arounding(50);
					usleep(100000);
//				}
			}
			else if(Temp_Rcon_Status&RconL_HomeR)  //L_R  9 18
			{
				ROS_DEBUG("%s, %d: Detect L-R.", __FUNCTION__, __LINE__);
				move_forward(17, 9);
//				Uniform_Forward(25,10);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFL2_HomeT)  //FL2_T
			{
				ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
				move_forward(16, 19);
//				Uniform_Forward(25,10);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFL2_HomeL)  //FL_HL
			{
				ROS_DEBUG("%s, %d: Detect FL2-L.", __FUNCTION__, __LINE__);
				move_forward(15, 11);
//				Uniform_Forward(25,10);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else  if(Temp_Rcon_Status&RconFL2_HomeR)//FL2_HR
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				move_forward(9, 15);
//				Uniform_Forward(25,10);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFL_HomeL)	//FR_HL
			{
				ROS_DEBUG("%s, %d: Detect FL-L.", __FUNCTION__, __LINE__);
//				stop_brifly();
				Turn_Right(Turn_Speed,500);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconFL_HomeR)	 //FR_HL
			{
				ROS_DEBUG("%s, %d: Detect FL-R.", __FUNCTION__, __LINE__);
//				stop_brifly();
				turn_left(Turn_Speed, 600);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconFL_HomeT)	 //FR_HT
			{
				ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
//				stop_brifly();
				Turn_Right(Turn_Speed,500);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconFR_HomeT)	 //R_HT
			{
				ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
//				stop_brifly();
				Turn_Right(Turn_Speed,800);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconFR2_HomeT) //FR2_T //OK
			{
				ROS_DEBUG("%s, %d: Detect FR2-T.", __FUNCTION__, __LINE__);
//				stop_brifly();
				Turn_Right(Turn_Speed,900);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconR_HomeT)  //OK
			{
				ROS_DEBUG("%s, %d: Detect R-T.", __FUNCTION__, __LINE__);
//				stop_brifly();
				Turn_Right(Turn_Speed,1100);
				move_forward(5, 5);
				Dir = 0;
			}
			else
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				move_forward(16, 34);  //0K (16,35)	  1100
//				Uniform_Forward(14,31);
//				Delay_Arounding(110);
				usleep(100000);
			}

			if(Temp_Rcon_Status&(RconFR_HomeR))
			{
				ROS_DEBUG("%s, %d: Detect FR-R, call By_Path().", __FUNCTION__, __LINE__);
//				stop_brifly();
				By_Path();
				return;
			}
			if(Temp_Rcon_Status&(RconFL_HomeR))
			{
				ROS_DEBUG("%s, %d: Detect FL-R, call By_Path().", __FUNCTION__, __LINE__);
//				stop_brifly();
				By_Path();
				return;
			}
			if(Temp_Rcon_Status&(RconL_HomeR))
			{
				ROS_DEBUG("%s, %d: Detect L-R, call By_Path().", __FUNCTION__, __LINE__);
				Signal_Counter++;
				N_Around_LRSignal=0;
				if(Signal_Counter>0)
				{
					ROS_DEBUG("%s %d Signal_Counter>0, check position.", __FUNCTION__, __LINE__);
					Signal_Counter=0;
					stop_brifly();
					Temp_Position = Check_Position(Round_Left);
					ROS_DEBUG("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Position);
					stop_brifly();
					if(Temp_Position==1)
					{
//						Reset_Error_Code();
//						SetDisplayError(Error_Code_None);
//						reset_stop_event_status();
						ROS_INFO("%s %d return to Clean_Mode_Userinterface", __FUNCTION__, __LINE__);
						set_clean_mode(Clean_Mode_Userinterface);
						return;
					}
					if(Temp_Position==2)
					{
						ROS_DEBUG("%s %d call By_Path()", __FUNCTION__, __LINE__);
						//move_forward(1,1);
						By_Path();
						return;
					}
				}
			}
			else
			{
				N_Around_LRSignal++;
				if(N_Around_LRSignal>4)
				{
					if(Signal_Counter>0)Signal_Counter--;
				}
			}
		}
		else//30.10
		{
//			if(get_left_wheel_step()>20000)
//			{
//				stop_brifly();
//				turn_left(Turn_Speed,2200);
//				set_clean_mode(Clean_Mode_GoHome);
//				return ;
//			}
			if(Temp_Rcon_Status&RconR_HomeT)   // OK ,(10,26)
			{
				ROS_DEBUG("%s %d Detect R-T.", __FUNCTION__, __LINE__);
				move_forward(5, 19);
				usleep(100000);
//				if((++RTSignal_Count)>=4)
//				{
//					RTSignal_Count = 0;
//					RRSignal_Count = 0;
//					Uniform_Forward(22,32);  //13 18
//					Delay_Arounding(60);
//				}
			}
			else if(Temp_Rcon_Status&RconR_HomeR)  //ok 18 13
			{
				ROS_DEBUG("%s %d Detect R-R.", __FUNCTION__, __LINE__);
				move_forward(5, 19);
				usleep(100000);
//				if((++RRSignal_Count)>=4)
//				{
//					RRSignal_Count = 0;
//					Uniform_Forward(22,28);
//					Delay_Arounding(60);
//				}
			}
			else if(Temp_Rcon_Status&RconR_HomeL)  //ok 18 13
			{
				ROS_DEBUG("%s %d Detect R-L.", __FUNCTION__, __LINE__);
				move_forward(9, 17);
				usleep(100000);
//				Delay_Arounding(100);
			}
			else if(Temp_Rcon_Status&RconFR2_HomeT)   //turn left
			{
				ROS_DEBUG("%s %d Detect FR2-T.", __FUNCTION__, __LINE__);
				move_forward(19, 17);
//				Uniform_Forward(10,25);
				usleep(100000);
//				Delay_Arounding(60);
			}
			else if(Temp_Rcon_Status&RconFR2_HomeR)  //OK
			{
				ROS_DEBUG("%s %d Detect FR2-R.", __FUNCTION__, __LINE__);
				move_forward(11, 15);
//				Uniform_Forward(10,25);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFR2_HomeL)  //
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				move_forward(15, 9);
				usleep(100000);
//				Uniform_Forward(10,25);
//				Delay_Arounding(100);
			}
			else if(Temp_Rcon_Status&RconFR_HomeR)	//OK
			{
				ROS_DEBUG("%s, %d: Detect FR-R.", __FUNCTION__, __LINE__);
//				stop_brifly();
				turn_left(Turn_Speed, 500);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconFR_HomeL)	//OK
			{
				ROS_DEBUG("%s, %d: Detect FR-L.", __FUNCTION__, __LINE__);
//				stop_brifly();
				Turn_Right(Turn_Speed,600);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconFR_HomeT)	//ok
			{
				ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
//				stop_brifly();
				turn_left(Turn_Speed, 500);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconFL_HomeT)	//OK
			{
				ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
//				stop_brifly();
				turn_left(Turn_Speed, 800);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconFL2_HomeT)  //OK
			{
				ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
//				stop_brifly();
				turn_left(Turn_Speed, 900);
				move_forward(5, 5);
			}
			else if(Temp_Rcon_Status&RconL_HomeT)  //OK
			{
				ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
//				stop_brifly();
				turn_left(Turn_Speed, 1100);
				move_forward(5, 5);
				Dir = 1;
			}
			else
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				move_forward(34, 16);
//				move_forward(31,14);  //0K (35,13)
//				Delay_Arounding(110);
				usleep(100000);
			}

			if(Temp_Rcon_Status&(RconFL_HomeL))
			{
				ROS_DEBUG("%s, %d: Detect FL-L, call By_Path().", __FUNCTION__, __LINE__);
//				stop_brifly();
//				Turn_Right(30,500);
				By_Path();
				return;
			}
			if(Temp_Rcon_Status&(RconFR_HomeL))
			{
				ROS_DEBUG("%s, %d: Detect FR-L, call By_Path().", __FUNCTION__, __LINE__);
//				stop_brifly();
//				Turn_Right(Turn_Speed,300);
				By_Path();
				return;
			}
			if((Temp_Rcon_Status&(RconR_HomeL)))
			{
				ROS_DEBUG("%s, %d: Detect R-L, call By_Path().", __FUNCTION__, __LINE__);
				N_Around_LRSignal=0;
				Signal_Counter++;
				if(Signal_Counter>0)
				{
					ROS_DEBUG("%s %d Signal_Counter>0, check position.", __FUNCTION__, __LINE__);
					Signal_Counter=0;
					stop_brifly();
					Temp_Position = Check_Position(Round_Right);
					ROS_DEBUG("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Position);
					stop_brifly();
					if(Temp_Position==1)
					{
//						Reset_Error_Code();
//						SetDisplayError(Error_Code_None);
//						reset_stop_event_status();
						ROS_INFO("%s %d return to Clean_Mode_Userinterface", __FUNCTION__, __LINE__);
						set_clean_mode(Clean_Mode_Userinterface);
						return;
					}
					if(Temp_Position==2)
					{
						ROS_DEBUG("%s %d call By_Path()", __FUNCTION__, __LINE__);
						//move_forward(1,1);
						By_Path();
						return;
					}
				}
			}
			else
			{
				N_Around_LRSignal++;
				if(N_Around_LRSignal>4)
				{
					if(Signal_Counter>0)Signal_Counter--;
				}
			}
		}

		usleep(50000);
	}
}

/*------------------------------------------------*/
uint8_t Check_Position(uint8_t Dir)
{
//	uint32_t Counter_Watcher=0;
	uint32_t Receive_Code = 0;
	// This angle is for counting angle change when the robot turns.
	float Current_Angle;
	float Last_Angle;
	float Angle_Offset;
	// This step is for counting angle change when the robot turns.
	float Gyro_Step = 0;

	if(Dir == Round_Left)
	{
		ROS_DEBUG("Check position Dir = left");
		set_dir_left();
	}
	else if(Dir == Round_Right)
	{
		ROS_DEBUG("Check position Dir = right");
		set_dir_right();
	}
	set_wheel_speed(10, 10);

	Last_Angle = robot::instance()->getAngle();
	ROS_DEBUG("Last_Angle = %f.", Last_Angle);

//	while(get_left_wheel_step()<3600)
	while(Gyro_Step < 360)
	{
//		delay(1);
		usleep(50000);
		Current_Angle = robot::instance()->getAngle();
		Angle_Offset = Current_Angle - Last_Angle;
		ROS_DEBUG("Current_Angle = %f, Last_Angle = %f, Angle_Offset = %f, Gyro_Step = %f.", Current_Angle, Last_Angle, Angle_Offset, Gyro_Step);
		if (Dir == Round_Left)
		{
			if (Angle_Offset < 0)
				Angle_Offset += 360;
			Gyro_Step += Angle_Offset;
		}
		if (Dir == Round_Right)
		{
			if (Angle_Offset > 0)
				Angle_Offset -= 360;
			Gyro_Step += (-Angle_Offset);
		}
		Last_Angle = Current_Angle;
		//Counter_Watcher++;
		//if(Counter_Watcher>150000)
		//{
		//	if(is_encoder_fail())
		//	{
		//		set_error_code(Error_Code_Encoder);
		//	}
		//	return 1;
		//}
		Receive_Code = (get_rcon_status()&(RconL_HomeL|RconL_HomeR|RconFL_HomeL|RconFL_HomeR|RconR_HomeL|RconR_HomeR|RconFR_HomeL|RconFR_HomeR));
		ROS_DEBUG("Check_Position get_rcon_status() == %x, R... == %x, receive code: %x.", get_rcon_status(), (RconL_HomeL|RconL_HomeR|RconFL_HomeL|RconFL_HomeR|RconR_HomeL|RconR_HomeR|RconFR_HomeL|RconFR_HomeR), Receive_Code);
		if(Receive_Code)
		{
			reset_rcon_status();
			if (Receive_Code & RconL_HomeL)ROS_DEBUG("Check_Position get L-L");
			if (Receive_Code & RconL_HomeR)ROS_DEBUG("Check_Position get L-R");
			if (Receive_Code & RconFL_HomeL)ROS_DEBUG("Check_Position get FL-L");
			if (Receive_Code & RconFL_HomeR)ROS_DEBUG("Check_Position get FL-R");
			if (Receive_Code & RconR_HomeL)ROS_DEBUG("Check_Position get R-L");
			if (Receive_Code & RconR_HomeR)ROS_DEBUG("Check_Position get R-R");
			if (Receive_Code & RconFR_HomeL)ROS_DEBUG("Check_Position get FR-L");
			if (Receive_Code & RconFR_HomeR)ROS_DEBUG("Check_Position get FR-R");
		}

		if(Dir == Round_Left)
		{
			if(Receive_Code & (RconFR_HomeL|RconFR_HomeR))
			{
				ROS_DEBUG("Check position left and return 2.");
				return 2;
			}
		}
		if(Dir == Round_Right)
		{
			if(Receive_Code & (RconFL_HomeL|RconFL_HomeR))
			{
				ROS_DEBUG("Check position right and return 2.");
				return 2;
			}
		}
		if(stop_event())
		{
			//beep(5, 20, 0, 1);
			stop_brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (get_key_press() & KEY_CLEAN)
			{
				ROS_WARN("%s %d: User hasn't release key.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Do not reset g_stop_event_status is for when robot is going home in navigation mode,
			// when stop event status is on, it will know and won't go to next home point.
			if (!robot::instance()->isLowBatPaused())
				if (!robot::instance()->isManualPaused())
					reset_stop_event_status();
			return 1;
		}

		//prompt for useless remote command
		if (get_rcon_remote() > 0) {
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (get_rcon_remote() & (Remote_Clean)) {
			} else {
				beep_for_command(false);
				reset_rcon_remote();
			}
		}

		if(is_charge_on())
		{
			ROS_DEBUG("%s %d: is_charge_on!!", __FUNCTION__, __LINE__);
			disable_motors();
			stop_brifly();
//			delay(2000);
			usleep(200000);
			if(is_charge_on())
			{
				//delay(5000);
				usleep(200000);
				if(is_charge_on())
				{
//					Reset_Error_Code();
					set_clean_mode(Clean_Mode_Charging);
//					beep(2, 25, 0, 1);
//					reset_rcon_remote();
					return 2;
				}
			}
			else if(turn_connect())
			{
				set_clean_mode(Clean_Mode_Charging);
//				reset_rcon_remote();
				return 2;
			}
			else
			{
				set_side_brush_pwm(30, 30);
				set_main_brush_pwm(0);
				////Back(30,800);
				//Back(30,300);
				quick_back(30,300);
				set_main_brush_pwm(30);
				stop_brifly();
			}
			if (stop_event())
			{
				ROS_WARN("%s %d: stop_event in turn_connect.", __FUNCTION__, __LINE__);
				disable_motors();
				return 1;
			}
		}
		uint8_t octype = check_motor_current();
		if(octype){
			if(self_check(octype)){
				ROS_INFO("%s ,%d motor over current ",__FUNCTION__,__LINE__);
				return 1;
			}
		}
	}
//	reset_temp_pwm();
	return 0;
}

void By_Path(void)
{
	uint8_t Cycle=0;
//	uint8_t Mobility_Temp_Error=0;
//	uint32_t Temp_Mobility_Distance=0;
	uint32_t Receive_Code=0;
	uint32_t Temp_Code =0 ;
	uint8_t Position_Far=1;
	uint16_t NoSignal_Counter=0;
	uint8_t Temp_Check_Position=0;
	uint8_t Near_Counter=0;
	uint8_t Bumper_Counter=0;
	uint8_t Side_Counter=0;

//	reset_wheel_step();

	reset_stop_event_status();
//	Display_Content(LED_Home,100,100,0,7);

//	Enable the charge function
	set_start_charge();

	move_forward(9, 9);
	set_side_brush_pwm(30, 30);
	set_main_brush_pwm(30);
	set_bldc_speed(Vac_Speed_NormalL);
//	set_home_remote();

//	beep(1);

	while(1)
	{
		Receive_Code = 0;
//		if(get_left_wheel_step()<500)
//		{
//			Mobility_Temp_Error=0;
//			Temp_Mobility_Distance = get_move_distance();
//		}
//		else
//		{
//			if((get_move_distance()-Temp_Mobility_Distance)>500)
//			{
//				Temp_Mobility_Distance = get_move_distance();
//				if(get_mobility_step()<1)
//				{
//					Mobility_Temp_Error++;
//					if(Mobility_Temp_Error>3)
//					{
//						set_clean_mode(Clean_Mode_GoHome);
//						return;
//					}
//				}
//				else
//				{
//					Mobility_Temp_Error=0;
//				}
//				reset_mobility_step();
//			}
//		}

		Cycle=10;
		ROS_DEBUG("%s, %d: Refresh cycle.", __FUNCTION__, __LINE__);
		while(Cycle--)
		{
			//ROS_DEBUG("new round, Bumper_Counter = %d.", Bumper_Counter);
			if(is_charge_on())
			{
				ROS_DEBUG("%s %d: is_charge_on!!", __FUNCTION__, __LINE__);
				disable_motors();
				stop_brifly();
//				delay(2000);
				usleep(200000);
				if(is_charge_on())
				{
					//delay(5000);
					usleep(200000);
					if(is_charge_on())
					{
//						Reset_Error_Code();
						set_clean_mode(Clean_Mode_Charging);
//						beep(2, 25, 0, 1);
//						reset_rcon_remote();
						return;
					}
				}
				else if(turn_connect())
				{
					set_clean_mode(Clean_Mode_Charging);
//					reset_rcon_remote();
					return;
				}
				else
				{
					set_side_brush_pwm(30, 30);
					set_main_brush_pwm(0);
					////Back(30,800);
					//Back(30,300);
					quick_back(30,300);
					set_main_brush_pwm(30);
					stop_brifly();
				}
				if (stop_event())
				{
					ROS_WARN("%s %d: stop_event in turn_connect.", __FUNCTION__, __LINE__);
					disable_motors();
					return;
				}

				//prompt for useless remote command
				if (get_rcon_remote() > 0) {
					ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
					if (get_rcon_remote() & (Remote_Clean)) {
					} else {
						beep_for_command(false);
						reset_rcon_remote();
					}
				}
			}
			/*----------------------------------------------OBS------------------Event---------------*/
			//ROS_DEBUG("get_Left_bumper_Status");
			if(get_bumper_status()&LeftBumperTrig)
			{
//				random_back();
				reset_rcon_status();
				if(!Position_Far)
				{
					stop_brifly();
					if(turn_connect())
					{
						set_clean_mode(Clean_Mode_Charging);
						ROS_INFO("Set Clean_Mode_Charging and return");
						return;
					}
					if (stop_event())
					{
						ROS_WARN("%s %d: stop_event in turn_connect.", __FUNCTION__, __LINE__);
						disable_motors();
						return;
					}

					//prompt for useless remote command
					if (get_rcon_remote() > 0) {
						ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
						if (get_rcon_remote() & (Remote_Clean)) {
						} else {
							beep_for_command(false);
							reset_rcon_remote();
						}
					}
					set_side_brush_pwm(30, 30);
					set_main_brush_pwm(0);
//					Back(30,2500);//waiting
					quick_back(30,300);//waiting
					ROS_DEBUG("%d: quick_back in !position_far", __LINE__);
					set_main_brush_pwm(30);
					stop_brifly();
					if(Bumper_Counter>0)
					{
						move_forward(0, 0);
						set_clean_mode(Clean_Mode_GoHome);
						ROS_DEBUG("%d, Return from LeftBumperTrig.", __LINE__);
						return;
					}
				}
				else if((get_rcon_status()&(RconFL2_HomeL|RconFL2_HomeR|RconFR2_HomeL|RconFR2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR))==0)
				{
					random_back();
					Turn_Right(Turn_Speed,1100);
					move_forward(8, 8);
					set_clean_mode(Clean_Mode_GoHome);
					ROS_DEBUG("%d, Return from LeftBumperTrig.", __LINE__);
					return;
				}
				else
				{
					random_back();
					Turn_Right(Turn_Speed,1100);
					set_side_brush_pwm(30, 30);
					set_main_brush_pwm(30);
					move_forward(8, 8);
				}
				if(is_bumper_jamed())
				{
					return;
				}
				Bumper_Counter++;
				ROS_DEBUG("%d, Left bumper count =%d.", __LINE__, Bumper_Counter);
			}
			//ROS_DEBUG("Get_Right_Bumper_Status");
			if(get_bumper_status()&RightBumperTrig)
			{
//				random_back();
				reset_rcon_status();
				if(!Position_Far)
				{
					stop_brifly();
					if(turn_connect())
					{
						set_clean_mode(Clean_Mode_Charging);
						return;
					}
					if (stop_event())
					{
						ROS_WARN("%s %d: stop_event in turn_connect.", __FUNCTION__, __LINE__);
						disable_motors();
						return;
					}
	
					//prompt for useless remote command
					if (get_rcon_remote() > 0) {
						ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
						if (get_rcon_remote() & (Remote_Clean)) {
						} else {
							beep_for_command(false);
							reset_rcon_remote();
						}
					}
					set_side_brush_pwm(30, 30);
					set_main_brush_pwm(0);
					quick_back(30,300);
					set_main_brush_pwm(30);
					stop_brifly();
					if(Bumper_Counter>0)
					{
						move_forward(0, 0);
						set_clean_mode(Clean_Mode_GoHome);
						ROS_DEBUG("%d, Return from RightBumperTrig.", __LINE__);
						return;
					}
				}
				else if((get_rcon_status()&(RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFL2_HomeL|RconFL2_HomeR|RconFR2_HomeL|RconFR2_HomeR))==0)
				{
					random_back();
					turn_left(Turn_Speed, 1100);
					move_forward(8, 8);
					set_clean_mode(Clean_Mode_GoHome);
					ROS_DEBUG("%d, Return from RightBumperTrig.", __LINE__);
					return;
				}
				else
				{
					random_back();
					turn_left(Turn_Speed, 1100);
					set_side_brush_pwm(30, 30);
					set_main_brush_pwm(30);
					move_forward(8, 8);
				}
				if(is_bumper_jamed())return;
				Bumper_Counter++;
				ROS_DEBUG("%d, Right bumper count =%d.", __LINE__, Bumper_Counter);
			}

			if(Position_Far)
			{
				if(get_cliff_trig())
				{
					if (get_cliff_trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
					{
						disable_motors();
						ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
						wav_play(WAV_ERROR_LIFT_UP);
						set_clean_mode(Clean_Mode_Userinterface);
						return;
					}
					move_back();
					move_back();
					turn_left(Turn_Speed, 1750);
					move_forward(9, 9);
					set_clean_mode(Clean_Mode_GoHome);
					return;
				}
			}
			else
			{
				if(get_cliff_trig())
				{
					if (get_cliff_trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
					{
						disable_motors();
						ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
						wav_play(WAV_ERROR_LIFT_UP);
						set_clean_mode(Clean_Mode_Userinterface);
						return;
					}
					set_wheel_speed(0, 0);
					set_dir_backward();
//					delay(300);
					usleep(30000);
					if(get_cliff_trig())
					{
						move_back();
						move_back();
						turn_left(Turn_Speed, 1750);
						move_forward(9, 9);
						set_clean_mode(Clean_Mode_GoHome);
						return;
					}
					set_dir_forward();
					break;
				}
			}

			/*------------------------------------------------------stop event-----------------------*/
			if(stop_event())
			{
				//beep(5, 20, 0, 1);
				stop_brifly();
				// Key release detection, if user has not release the key, don't do anything.
				while (get_key_press() & KEY_CLEAN)
				{
					ROS_WARN("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
					usleep(20000);
				}
				// Do not reset g_stop_event_status is for when robot is going home in navigation mode,
				// when stop event status is on,
				// it will know and won't go to next home point.
				if (!robot::instance()->isLowBatPaused())
					if (!robot::instance()->isManualPaused())
						reset_stop_event_status();

				set_clean_mode(Clean_Mode_Userinterface);
				return;
			}
			//prompt for useless remote command
			if (get_rcon_remote() > 0) {
				ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
				if (get_rcon_remote() & (Remote_Clean)) {
				} else {
					beep_for_command(false);
					reset_rcon_remote();
				}
			}
			if(check_bat_stop())
			{
//				delay(10000);
				usleep(1000000);
				set_clean_mode(Clean_Mode_Userinterface);
				return;
			}
			if(Home_Check_Current())return;
//			delay(100);
			usleep(10000);
		}


		Receive_Code = get_rcon_status();
		Temp_Code = Receive_Code;
//		Temp_Code &= 0x003f0fff;
		Temp_Code &= (	RconL_HomeL|RconL_HomeT|RconL_HomeR| \
						RconFL2_HomeL|RconFL2_HomeT|RconFL2_HomeR| \
						RconFL_HomeL|RconFL_HomeT|RconFL_HomeR| \
						RconFR_HomeL|RconFR_HomeT|RconFR_HomeR| \
						RconFR2_HomeL|RconFR2_HomeT|RconFR2_HomeR| \
						RconR_HomeL|RconR_HomeT|RconR_HomeR \
					 );
//		reset_rcon_status();
		if(Receive_Code)
		{
//			if((Receive_Code&0x00060000) == 0x00060000)Position_Far = 0;
			if((Receive_Code&(RconFR_HomeT|RconFL_HomeT)) == (RconFR_HomeT|RconFL_HomeT))
			{
				Position_Far = 0;
				ROS_INFO("%s, %d: Robot face HomeT, Position_Far = 0.", __FUNCTION__, __LINE__);
			}
//			if(Receive_Code&0x00390000)Position_Far = 0;
			if(Receive_Code&(RconFR2_HomeT|RconFL2_HomeT|RconR_HomeT|RconL_HomeT))
			{
				Position_Far = 0;
				ROS_DEBUG("%s, %d: Robot side face HomeT, Position_Far = 0.", __FUNCTION__, __LINE__);
			}
//			if(Receive_Code&0x003f0000)
			if(Receive_Code&(RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT|RconFL2_HomeT|RconR_HomeT|RconL_HomeT))
			{
				Near_Counter++;
				if(Near_Counter > 1)
				{
					Position_Far = 0;
					ROS_DEBUG("%s, %d: Robot near HomeT counter > 1, Position_Far = 0.", __FUNCTION__, __LINE__);
				}
//				if((Receive_Code&0x000000ff) == 0)
				if((Receive_Code&(RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR)) == 0)
				{
					Side_Counter++;
					if(Side_Counter > 5)
					{
						move_forward(9, 9);
						ROS_INFO("%s, %d: Robot goes far, back to gohome mode.", __FUNCTION__, __LINE__);
						set_clean_mode(Clean_Mode_GoHome);
						return;
					}
				}
				else
				{
					Side_Counter = 0;
				}
			}

//			if((Receive_Code&0x00000024) == 0x00000024)Position_Far = 0;
			if((Receive_Code&(RconFL_HomeL|RconFR_HomeR))==(RconFL_HomeL|RconFR_HomeR))
			{
				ROS_DEBUG("%s, %d: Robot sees HomeL or HomeR, Position_Far = 0.", __FUNCTION__, __LINE__);
				Position_Far=0;
			}
			reset_rcon_status();
			NoSignal_Counter = 0;
		}
		else
		{
			Near_Counter = 0;
			NoSignal_Counter++;
			if(NoSignal_Counter>50)
			{
				NoSignal_Counter = 0;
				stop_brifly();
				Temp_Check_Position = Check_Position(Round_Left);
				ROS_DEBUG("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Check_Position);
				if(Temp_Check_Position == 1)
				{
					set_clean_mode(Clean_Mode_Userinterface);
					return;
				}
				else if(Temp_Check_Position == 0)
				{
					ROS_INFO("%s, %d: Robot can't see charger, return to gohome mode.", __FUNCTION__, __LINE__);
					stop_brifly();
					Turn_Right(Turn_Speed,1000);
					stop_brifly();
					move_forward(10, 10);
					set_clean_mode(Clean_Mode_GoHome);
					return;
				}
			}
		}

//		Temp_Code &= 0x00000fff;
		Temp_Code &= (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconR_HomeL| \
					RconL_HomeR|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR|RconR_HomeR);
		if(Position_Far)
		{
			switch(Temp_Code)
			{
//				case 0x024:   move_forward(12,12);break;			//FL_L/FR_R
				case (RconFL_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 12);
					break;

//				case 0x03c:   move_forward(12,12);break;		  //FL_L/FL_R/FR_L/FR_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 12);
					break;

//				case 0xbd:	  move_forward(12,12);break;			//FL2_L/FL_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 12);
					break;

//				case 0xA5:	  move_forward(12,12);break;			//FL2_L/FL_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 12);
					break;

//				case 0X0AD:   move_forward(13,11);break;			//FL2_L/FL_L/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
					move_forward(13, 11);
					break;

//				case 0x08D:   move_forward(13,9);break;				//FL2_L/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
					move_forward(13, 9);
					break;

//				case 0x089:   move_forward(12,9);break;				//FL2_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 9);
					break;

//				case 0x02C:   move_forward(11,9);break;				//FL_L/FR_L/FR_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(11, 9);
					break;

//				case 0x00D:   move_forward(12,8);break;				//FR_L/FR_R/FR2_R
				case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 8);
					break;

//				case 0x00C:   move_forward(12,8);break;				//FR_L/FR_R
				case (RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 8);
					break;

//				case 0x008:   move_forward(11,8);break;				//FR_L
				case (RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FR_L.", __FUNCTION__, __LINE__);
					move_forward(11, 8);
					break;

//				case 0x00F:   move_forward(11,7);break;				//FR_L/FR_R/FR2_L/FR2_R
				case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 7);
					break;

//				case 0x009:   move_forward(13,7);break;				//FR_L/FR2_R
				case (RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(13, 7);
					break;

//				case 0x00B:   move_forward(12,7);break;				//FR_L/FR2_L/FR2_R
				case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 7);
					break;

//				case 0x001:   move_forward(12,6);break;				//FR2_R
				case (RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 6);
					break;

//				case 0x002:   Turn_Right(20,250);				   //FRR_L
				case (RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x003:   Turn_Right(20,350);				   //FR2_L/FR2_R
				case (RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,350);
					stop_brifly();
					move_forward(10, 3);
					break;

//				case 0x28:	  move_forward(11,10);break;			 //FL_L/FR_L
				case (RconFL_HomeL|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L.", __FUNCTION__, __LINE__);
					move_forward(11, 10);
					break;

//				case 0x2B:	  move_forward(11,9);break;				 //FL_L/FR_L/FR2_L/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 9);
					break;

//				case 0x29:	  move_forward(12,10);break;			 //FL_L/FR_L/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 10);
					break;

//				case 0X0A:	  move_forward(12,7);break;				 //FR_L/FR2_L
				case (RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
					move_forward(12, 7);
					break;

//				case 0X54:	  move_forward(12,11);break;			 //FL_L/FR_L/FR2_L
				case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0X2D:	  move_forward(13,10);break;			 //FL_L/FR_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(13, 10);
					break;

//				case 0X05:	  move_forward(13,8);break;				 //FR_R/FR2_R
				case (RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(13, 8);
					break;

//				case 0x04:	  move_forward(12,11);break;
				case (RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0x45:	  move_forward(12,10);break;			 //FL2_R/FR_R/FR2_R
				case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 10);
					break;

//				case 0xc5:	  move_forward(12,10);break;			 //FL2_L/FL2_R/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 10);
					break;

//				case 0xCD:	  move_forward(13,7);break;				   //FL2_L/FL2_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(13, 7);
					break;

//				case 0xcf:	  move_forward(13,7);break;				 //FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(13, 7);
					break;

//				case 0xA9:	  move_forward(12,11);break;			 //FL2_L/FL_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0x85:	  move_forward(13,9);break;
				case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(13, 9);
					break;

//				case 0x8f:	  move_forward(13,8);break;				 //FL2_L/FR_L/FR_R/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(13, 8);
					break;

//				case 0x84:	  move_forward(13,9);break;
				case (RconFL2_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(13, 9);
					break;

//				case 0x31:	  move_forward(12,11);break;			 //FL_L/FL_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0x9D:	  move_forward(12,11);break;			 //FL2_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0xAC:	  move_forward(12,11);break;			 //FL2_L/FL_L/FR_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0xbc:	  move_forward(12,11);break;			 //FL2_L/FL_L/FL_R/FR_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0x25:	  move_forward(12,11);break;			 //FL_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0xb8:	  move_forward(13,12);break;			 //FL2_L/FL_L/FL_R/FR_L
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
					move_forward(13, 12);
					break;

//				case 0xA8:	  move_forward(13,12);break;			 //FL2_L/FL_L/FR_l
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
					move_forward(13, 12);
					break;

//				case 0x803:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: Position_Far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						Turn_Right(20,250);
					stop_brifly();
					move_forward(9, 3);
						break;

//				case 0x802:   Turn_Right(20,400);
				case (RconR_HomeR|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,400);
					stop_brifly();
					move_forward(8, 3);
					break;

//				case 0x80b:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					stop_brifly();
					move_forward(10, 4);
					break;

//				case 0xC02:	  Turn_Right(20,450);
				case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,450);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0xC00 :  Turn_Right(20,500);
				case (RconR_HomeR|RconR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/R_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					stop_brifly();
					move_forward(9, 6);
					break;

//				case 0x400 :  Turn_Right(20,550);
				case (RconR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,550);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x80A:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x402:   Turn_Right(20,500);
				case (RconR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x805:
				case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 2);
					break;

//				case 0x801:   move_forward(9,0);
				case (RconR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 0);
					break;

//				case 0x101:   move_forward(9,1);
				case (RconL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 1);
					break;

//				case 0x185:   move_forward(9,6);
				case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 6);
					break;

//				case 0x800:   move_forward(10,0);
				case (RconR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R.", __FUNCTION__, __LINE__);
					move_forward(10, 0);
					break;

//				case 0x8B:	  move_forward(13,12);break;						//FL2_L/FR_L/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(13, 12);
					break;

//				case 0x22:	  move_forward(13,12);break;			//FL2_R/FR_R
				case (RconFL_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
					move_forward(13, 12);
					break;

//				case 0x81:	  move_forward(14,4);break;
				case (RconFL2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(14, 4);
					break;

//				case 0x23:	  move_forward(12,11);break;
				case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(12, 11);
					break;

//				case 0Xb5:	  move_forward(10,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(10, 12);
					break;

//				case 0xb1:	  move_forward(8,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 12);
					break;

//				case 0x91:	  move_forward(11,13);break;
				case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 13);
					break;

//				case 0x34:	  move_forward(10,12);break;
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(10, 12);
					break;

//				case 0xb0:	  move_forward(9,13);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
					move_forward(9, 13);
					break;

//				case 0x30:	  move_forward(10,12);break;
				case (RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R.", __FUNCTION__, __LINE__);
					move_forward(10, 12);
					break;

//				case 0x10:	  move_forward(7,11);break;
				case (RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_R.", __FUNCTION__, __LINE__);
					move_forward(7, 11);
					break;

//				case 0xf0:	  move_forward(7,12);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
					move_forward(7, 12);
					break;

//				case 0x90:	  move_forward(9,13);break;
				case (RconFL2_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
					move_forward(9, 13);
					break;

//				case 0xD0:	  move_forward(8,13);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					move_forward(8, 13);
					break;

//				case 0x80:	  move_forward(8,14);break;
				case (RconFL2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L.", __FUNCTION__, __LINE__);
					move_forward(8, 14);
					break;

//				case 0x40:	  turn_left(20,250);
				case (RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 250);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0xC0:	  turn_left(20,350);
				case (RconFL2_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 350);
					stop_brifly();
					move_forward(3, 10);
					break;

//				case 0x14:	  move_forward(11,12);break;
				case (RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0xD4:	  move_forward(10,12);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(10, 12);
					break;

//				case 0x94:	  move_forward(10,12);break;
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(10, 12);
					break;

//				case 0X50:	  move_forward(7,12);break;
				case (RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
					move_forward(7, 12);
					break;

//				case 0X2A:	  move_forward(11,12);break;
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0XB4:	  move_forward(9,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(9, 12);
					break;

//				case 0XA0:	  move_forward(7,12);break;
				case (RconFL2_HomeL|RconFL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
					move_forward(7, 12);
					break;

//				case 0x20:	  move_forward(11,12);break;
				case (RconFL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_L.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0xA2:	  move_forward(10,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
					move_forward(10, 12);
					break;

//				case 0xA3:	  move_forward(10,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(10, 12);
					break;

//				case 0xB3:	  move_forward(7,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 12);
					break;

//				case 0xF3:	  move_forward(7,12);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 12);
					break;

//				case 0x95:	  move_forward(11,12);break;			//FL2_L/FL_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0xA1:	  move_forward(7,12);break;				//FL2_L/FL_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 12);
					break;

//				case 0xF1:	  move_forward(7,12);break;				//FL2_L/FL2_R/FL_L/FL_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 12);
					break;

//				case 0x21:	  move_forward(10,12);break;			//FL2_L/FL_L/FR2_R //FL2_L/FR_R
				case (RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(10, 12);
					break;

//				case 0x8c:	  move_forward(11,12);break;			//FL2_L/FL_L/FR2_R //FL2_L/FR_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0xb9:	  move_forward(11,12);break;			//FL2_L/FL_R/FL_R/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0x35:	  move_forward(11,12);break;			//FL_L/FL_R/FR_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0x3d:	  move_forward(11,12);break;			//FL_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0xa4:	  move_forward(11,12);break;			//FL2_L/FL_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0x1d:	  move_forward(11,12);break;			//FL_R/FR_L/FR_R/FR2_R
				case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0x15:	  move_forward(9,12);break;				//FL_R/FR_R/FR2_R
				case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 12);
					break;

//				case 0x1C0:   turn_left(20,250);
				case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 250);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x140:   turn_left(20,400);
				case (RconL_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 400);
					stop_brifly();
					move_forward(3, 8);
					break;

//				case 0x1d0:   turn_left(20,250);
				case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					turn_left(20, 250);
					stop_brifly();
					move_forward(4, 10);
					break;

//				case 0x340:   turn_left(20,450);
				case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 450);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x300 :  turn_left(20,500);
				case (RconL_HomeR|RconL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, L_R/L_L.", __FUNCTION__, __LINE__);
					turn_left(20, 500);
					stop_brifly();
					move_forward(6, 9);
					break;

//				case 0x200:   turn_left(20,550);
				case (RconL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_R.", __FUNCTION__, __LINE__);
					turn_left(20, 550);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x150:   turn_left(20,250);
				case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					turn_left(20, 250);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x240:   turn_left(20,500);
				case (RconL_HomeR|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_R/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 500);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x1A0:
				case (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L/FL_L.", __FUNCTION__, __LINE__);
					move_forward(2, 9);
					break;

//				case 0x180:   move_forward(0,9);
				case (RconL_HomeL|RconFL2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L.", __FUNCTION__, __LINE__);
					move_forward(0, 9);
					break;

//				case 0x880:   move_forward(1,9);
				case (RconR_HomeR|RconFL2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FL2_L.", __FUNCTION__, __LINE__);
					move_forward(1, 9);
					break;

//				case 0x8A1:   move_forward(6,9);
				case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(6, 9);
					break;

//				case 0x100:   move_forward(0,10);break;
				case (RconL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, L_L.", __FUNCTION__, __LINE__);
					move_forward(0, 10);
					break;

//				case 0xD1:	  move_forward(11,12);break;		  //FL_R/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0x44:	  move_forward(11,12);break;		  //FL_L/FR2_L
				case (RconFL2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

//				case 0x18:		move_forward(7,12);break;
				case (RconFL_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_R/FR_L.", __FUNCTION__, __LINE__);
					move_forward(7, 12);
					break;

//				case 0xC4:		move_forward(11,12);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(11, 12);
					break;

				default:
					ROS_DEBUG("%s, %d: Position_Far, else:%x.", __FUNCTION__, __LINE__, Temp_Code);
					move_forward(10, 11);
//					USPRINTF("**************default angle is ,default code	is 0x%x\n",Temp_Code);
					break;
			}
		}
		else
		{
			switch(Temp_Code)
			{
//				case 0x024:		move_forward(8,8);	break;			//FL_L/FR_R
				case (RconFL_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL-L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 8);
					break;

//				case 0x3c:	 move_forward(9,9);break;			  //FL_L/FL_R/FR_L/FR_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(9, 9);
					break;
//				case 0xbd:	  move_forward(8,8);break;			  //FL2_L/FL_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 8);
					break;

//				case 0xA5:	  move_forward(9,9);break;			  //FL2_L/FL_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(9, 9);
					break;

//				case 0X0AD:   move_forward(9,8);break;			   //FL2_L/FL_L/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
					move_forward(9, 8);
					break;

//				case 0x08D:   move_forward(10,8);break;			   //FL2_L/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
					move_forward(10, 8);
					break;

//				case 0x089:   move_forward(9,7);break;			   //FL2_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 7);
					break;

//				case 0x02C:   move_forward(8,6);break;			   //FL_L/FR_L/FR_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 6);
					break;

//				case 0x00D:   move_forward(9,6);break;			   //FR_L/FR_R/FR2_R
				case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 6);
					break;

//				case 0x00C:   move_forward(8,6);break;			   //FR_L/FR_R
				case (RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 6);
					break;

//				case 0x008:   move_forward(7,4);break;			   //FR_L
				case (RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L.", __FUNCTION__, __LINE__);
					move_forward(7, 4);
					break;

//				case 0x00F:   move_forward(7,3);break;			   //FR_L/FR_R/FR2_L/FR2_R
				case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 3);
					break;

//				case 0x009:   move_forward(9,5);break;			   //FR_L/FR2_R
				case (RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 5);
					break;

//				case 0x00B:   move_forward(8,3);break;			   //FR_L/FR2_L/FR2_R
				case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 3);
					break;

//				case 0x001:   move_forward(9,3);break;			   //FR2_R
				case (RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 3);
					break;

//				case 0x002:
				case (RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x003:   Turn_Right(20,350);				  //FR2_L/FR2_R
				case (RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,350);
					stop_brifly();
					move_forward(10, 3);
					break;

//				case 0x28:	  move_forward(8,7);break;				//FL_L/FR_L
				case (RconFL_HomeL|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0x2B:	  move_forward(8,6);break;				//FL_L/FR_L/FR2_L/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 6);
					break;

//				case 0x29:	  move_forward(9,7);break;				//FL_L/FR_L/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 7);
					break;

//				case 0X0A:	  move_forward(9,4);break;				//FR_L/FR2_L
				case (RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
					move_forward(9, 4);
					break;

//				case 0X54:	  move_forward(8,7);break;				//FL_L/FR_L/FR2_L
				case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0X2D:	  move_forward(8,6);break;				//FL_L/FR_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 6);
					break;

//				case 0X05:	  move_forward(9,7);break;				//FR_R/FR2_R
				case (RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 7);
					break;

//				case 0x04:	  move_forward(8,7);break;				//
				case (RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0x45:	  move_forward(8,7);break;				//FL2_R/FR_R/FR2_R
				case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0xc5:	  move_forward(8,6);break;				//FL2_L/FL2_R/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 6);
					break;

//				case 0xCD:	  move_forward(8,3);break;				  //FL2_L/FL2_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 3);
					break;

//				case 0xcf:	  move_forward(8,3);break;				//FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 3);
					break;

//				case 0xA9:	  move_forward(8,7);break;			   //FL2_L/FL_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0x85:	  move_forward(9,8);break;			   //
				case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 8);
					break;

//				case 0x8f:	  move_forward(8,7);break;			   //FL2_L/FL_L/FR2_R//FL2_L/FR_L/FR_R/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0x84:	  move_forward(9,8);break;			   //
				case (RconFL2_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(9, 8);
					break;

//				case 0x31:	  move_forward(9,8);break;			   //FL_L/FL_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 8);
					break;

//				case 0x9D:	  move_forward(8,7);break;			   //FL2_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0xAC:	  move_forward(8,7);break;			   //FL2_L/FL_L/FR_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0xbc:	  move_forward(8,7);break;			   //FL2_L/FL_L/FL_R/FR_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0x25:	  move_forward(8,7);break;			   //FL_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0xb8:	  move_forward(9,8);break;			   //FL2_L/FL_L/FL_R/FR_L
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
					move_forward(9, 8);
					break;

//				case 0xA8:	  move_forward(9,6);break;			   //FL2_L/FL_L/FR_l
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
					move_forward(9, 6);
					break;

//				case 0x803:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x802:   Turn_Right(20,400);
				case (RconR_HomeR|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,400);
					stop_brifly();
					move_forward(8, 3);
					break;

//				case 0x80b:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					stop_brifly();
					move_forward(10, 4);
					break;

//				case 0xC02:	  Turn_Right(20,450);
				case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,450);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0xC00 :  Turn_Right(20,500);
				case (RconR_HomeR|RconR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/R_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					stop_brifly();
					move_forward(9, 6);
					break;

//				case 0x400 :  Turn_Right(20,550);
				case (RconR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,550);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x80A:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x402:   Turn_Right(20,500);
				case (RconR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					stop_brifly();
					move_forward(9, 3);
					break;

//				case 0x805:
				case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 2);
					break;

//				case 0x801:   move_forward(9,0);
				case (RconR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 0);
					break;

//				case 0x101:   move_forward(9,1);
				case (RconL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 1);
					break;

//				case 0x185:   move_forward(9,6);
				case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 6);
					break;

//				case 0x800:   move_forward(10,0);
				case (RconR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R.", __FUNCTION__, __LINE__);
					move_forward(10, 0);
					break;

//				case 0x8B:	  move_forward(8,7);break;			  //FL2_L/FR_L/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 7);
					break;

//				case 0x22:	  move_forward(9,8);break;			  //FL2_R/FR_R
				case (RconFL_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
					move_forward(9, 8);
					break;

//				case 0x81:	  move_forward(9,2);break;
				case (RconFL2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 2);
					break;

//				case 0x23:	  move_forward(9,8);break;
				case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(9, 8);
					break;

//				case 0Xb5:	  move_forward(8,9);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 9);
					break;

//				case 0xb1:	  move_forward(8,10);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 10);
					break;

//				case 0x91:	  move_forward(7,9);break;
				case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 9);
					break;

//				case 0x34:	  move_forward(6,8);break;
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(6, 8);
					break;

//				case 0xb0:	  move_forward(6,9);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
					move_forward(6, 9);
					break;

//				case 0x30:	  move_forward(6,8);break;
				case (RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R.", __FUNCTION__, __LINE__);
					move_forward(6, 8);
					break;

//				case 0x10:	  move_forward(4,7);break;
				case (RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R.", __FUNCTION__, __LINE__);
					move_forward(4, 7);
					break;

//				case 0xf0:	  move_forward(3,7);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
					move_forward(3, 7);
					break;

//				case 0x90:	  move_forward(5,9);break;
				case (RconFL2_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
					move_forward(5, 9);
					break;

//				case 0xD0:	  move_forward(3,8);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					move_forward(3, 8);
					break;

//				case 0x80:	  move_forward(3,9);break;
				case (RconFL2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L.", __FUNCTION__, __LINE__);
					move_forward(3, 9);
					break;

//				case 0x40:	  turn_left(20,250);
				case (RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 250);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0xC0:	  turn_left(20,350);
				case (RconFL2_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 350);
					stop_brifly();
					move_forward(3, 10);
					break;

//				case 0x14:	  move_forward(7,8);break;
				case (RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0xD4:	  move_forward(6,8);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(6, 8);
					break;

//				case 0x94:	  move_forward(7,9);break;
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(7, 9);
					break;

//				case 0X50:	  move_forward(4,9);break;
				case (RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
					move_forward(4, 9);
					break;

//				case 0X2A:	  move_forward(7,8);break;
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0XB4:	  move_forward(6,8);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(6, 8);
					break;

//				case 0XA0:	  move_forward(7,9);break;
				case (RconFL2_HomeL|RconFL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
					move_forward(7, 9);
					break;

//				case 0x20:	  move_forward(7,8);break;
				case (RconFL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0xA2:	  move_forward(7,8);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0xA3:	  move_forward(6,8);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(6, 8);
					break;

//				case 0xB3:	  move_forward(3,8);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(3, 8);
					break;

//				case 0xF3:	  move_forward(3,8);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(3, 8);
					break;

//				case 0x95:	  move_forward(7,8);break;//FL2_L/FL_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0xA1:	  move_forward(8,9);break;			  //FL2_L/FL_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 9);
					break;

//				case 0xF1:	  move_forward(7,8);break;			  //FL2_L/FL2_R/FL_L/FL_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0x21:	  move_forward(8,9);break;			  //FL2_L/FL_L/FR2_R //FL2_L/FR_R
				case (RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 9);
					break;

//				case 0x8c:	  move_forward(8,9);break;			  //FL2_L/FL_L/FR2_R //FL2_L/FR_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 9);
					break;

//				case 0xb9:	  move_forward(7,8);break;			  //FL2_L/FL_R/FL_R/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0x35:	  move_forward(7,8);break;			  //FL_L/FL_R/FR_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0x3d:	  move_forward(7,8);break;			  //FL_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0xa4:	  move_forward(7,8);break;			  //FL2_L/FL_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0x1d:	  move_forward(8,9);break;			  //FL_R/FR_L/FR_R/FR2_R
				case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(8, 9);
					break;

				case 0x15:
					move_forward(6, 9);break;			  //FL_R/FR_R/FR2_R
				case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(6, 9);
					break;

//				case 0x1C0:   turn_left(20,250);
				case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 250);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x140:   turn_left(20,400);
				case (RconL_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 400);
					stop_brifly();
					move_forward(3, 8);
					break;

//				case 0x1d0:   turn_left(20,250);
				case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					turn_left(20, 250);
					stop_brifly();
					move_forward(4, 10);
					break;

//				case 0x340:   turn_left(20,450);
				case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 450);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x300 :  turn_left(20,500);
				case (RconL_HomeR|RconL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, L_R/L_L.", __FUNCTION__, __LINE__);
					turn_left(20, 500);
					stop_brifly();
					move_forward(6, 9);
					break;

//				case 0x200:   turn_left(20,550);
				case (RconL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_R.", __FUNCTION__, __LINE__);
					turn_left(20, 550);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x150:   turn_left(20,250);
				case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					turn_left(20, 250);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x240:   turn_left(20,500);
				case (RconL_HomeR|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_R/FL2_R.", __FUNCTION__, __LINE__);
					turn_left(20, 500);
					stop_brifly();
					move_forward(3, 9);
					break;

//				case 0x1A0:
				case (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L/FL_L.", __FUNCTION__, __LINE__);
					move_forward(2, 9);
					break;

//				case 0x180:   move_forward(0,9);
				case (RconL_HomeL|RconFL2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L.", __FUNCTION__, __LINE__);
					move_forward(0, 9);
					break;

//				case 0x880:   move_forward(1,9);
				case (RconR_HomeR|RconFL2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FL2_L.", __FUNCTION__, __LINE__);
					move_forward(1, 9);
					break;

//				case 0x8A1:   move_forward(6,9);
				case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(6, 9);
					break;

//				case 0x100:   move_forward(0,10);break;
				case (RconL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, L_L.", __FUNCTION__, __LINE__);
					move_forward(0, 10);
					break;

//				case 0xD1:	  move_forward(7,8);break;			  //FL_R/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					move_forward(7, 8);
					break;

//				case 0x44:	  move_forward(8,9);break;			  //FL_L/FR2_L
				case (RconFL2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 9);
					break;

//				case 0x18:	  move_forward(2,9);break;
				case (RconFL_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R/FR_L.", __FUNCTION__, __LINE__);
					move_forward(2, 9);
					break;

//				case 0xC4:	  move_forward(8,9);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
					move_forward(8, 9);
					break;

				default:
//					USPRINTF("**************default angle is %d, default code  is 0x%x\n",angle,Temp_Code);
					ROS_DEBUG("%s, %d: !Position_Far, else:%x.", __FUNCTION__, __LINE__, Temp_Code);
					move_forward(7, 8);
					break;
			}
		}
//		USART_DMA_String(11,"\n\n\r HOME   ");//left cliff
//		USART_DMA_Numbers(Temp_Code);
//		Display_TM1618(Temp_Code,1);
//		Display_Content(0,Temp_Code/100,Temp_Code%100,0,1);
//		delay(500);
		usleep(50000);
	}
}

/*------------------------------------------------*/
uint8_t Home_Check_Current(void)
{
	uint8_t Motor_Check_Code= check_motor_current();
	if(Motor_Check_Code)
	{
		if(self_check(Motor_Check_Code))
		{
			set_clean_mode(Clean_Mode_Userinterface);
			return 1;
		}
		else
		{
			Home_Motor_Set();
			set_clean_mode(Clean_Mode_GoHome);
			return 1;
		}
	}
	return 0;
}

/*-------------------Turn OFF the Vaccum-----------------------------*/
void Home_Motor_Set(void)
{
	set_bldc_speed(0);
	set_main_brush_pwm(20);
	set_side_brush_pwm(20, 20);
	move_forward(20, 20);
	//Reset_WheelSLow();
	//reset_bumper_error();

}

void charge_register_event(void)
{
	ROS_WARN("%s, %d: Register events.", __FUNCTION__, __LINE__);
	event_manager_set_current_mode(EVT_MODE_CHARGE);
#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &charge_handle_ ##name); \
	event_manager_enable_handler(y, enabled)

	/* Charge Status */
	event_manager_register_and_enable_x(charge_detect, EVT_CHARGE_DETECT, true);
	/* Plan */
	event_manager_register_and_enable_x(remote_plan, EVT_REMOTE_PLAN, true);
	/* key */
	event_manager_register_and_enable_x(key, EVT_KEY_CLEAN, true);
	/* Remote */
	event_manager_register_and_enable_x(remote_cleaning, EVT_REMOTE_CLEAN, true);
}

void charge_unregister_event(void)
{
	ROS_WARN("%s, %d: Unregister events.", __FUNCTION__, __LINE__);
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	/* Charge Status */
	event_manager_register_and_disable_x(EVT_CHARGE_DETECT);
	/* Plan */
	event_manager_register_and_disable_x(EVT_REMOTE_PLAN);
	/* Key */
	event_manager_register_and_disable_x(EVT_KEY_CLEAN);
	/* Remote */
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
}

void charge_handle_charge_detect(bool state_now, bool state_last)
{
	g_stop_charge_counter = 20;
}
void charge_handle_remote_plan(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Remote key plan has been pressed.", __FUNCTION__, __LINE__);

	switch(get_plan_status())
	{
		case 1:
		{
			ROS_WARN("%s %d: Remote key plan has been pressed. Plan received.", __FUNCTION__, __LINE__);
			beep_for_command(true);
			break;
		}
		case 2:
		{
			ROS_WARN("%s %d: Plan canceled.", __FUNCTION__, __LINE__);
			wav_play(WAV_CANCEL_APPOINTMENT);
			break;
		}
		case 3:
		{
			ROS_WARN("%s %d: Plan activated.", __FUNCTION__, __LINE__);
			if (get_error_code() != Error_Code_None)
			{
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
				alarm_error();
				wav_play(WAV_CANCEL_APPOINTMENT);
				set_plan_status(0);
				break;
			}
			else if(get_cliff_trig() & (Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right))
			{
				ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
				wav_play(WAV_ERROR_LIFT_UP);
				wav_play(WAV_CANCEL_APPOINTMENT);
				set_plan_status(0);
				break;
			}
			else
			{
				// Sleep for 50ms cause the status 3 will be sent for 3 times.
				usleep(50000);
				if (!robot::instance()->isManualPaused())
					set_clean_mode(Clean_Mode_Navigation);
				break;
			}
		}
		case 4:
		{
			ROS_WARN("%s %d: Plan confirmed.", __FUNCTION__, __LINE__);
			wav_play(WAV_APPOINTMENT_DONE);
			break;
		}
	}
	set_plan_status (0);
}
void charge_handle_key(bool state_now, bool state_last)
{
	if (is_direct_charge())
	{
		ROS_WARN("Can not go to navigation mode during direct charging.");
		beep_for_command(false);
		// Key release detection, if user has not release the key, don't do anything.
		while (get_key_press() & KEY_CLEAN)
		{
			ROS_WARN("%s %d: User hasn't release key.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
	}
	else if (!check_bat_ready_to_clean())
	{
		ROS_WARN("Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(1400) + 60, can't go to navigation mode.");
		wav_play(WAV_BATTERY_LOW);
	}
	else if (is_on_charger_stub())
	{
		ROS_WARN("[gotocharger.cpp] Exit charger mode and go to navigation mode.");
		// Key release detection, if user has not release the key, don't do anything.
		while (get_key_press() & KEY_CLEAN)
		{
			ROS_WARN("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
		set_clean_mode(Clean_Mode_Navigation);
	}
}
void charge_handle_remote_cleaning(bool stat_now, bool state_last)
{
	if (remote_key(Remote_Clean)) {
		reset_rcon_remote();
		if (is_direct_charge())
		{
			ROS_WARN("Can not go to navigation mode during direct charging.");
			beep_for_command(false);
		}
		else if (!check_bat_ready_to_clean())
		{
			ROS_WARN("Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(1400) + 60, can't go to navigation mode.");
			wav_play(WAV_BATTERY_LOW);
		}
		else if (is_on_charger_stub())
		{
			set_clean_mode(Clean_Mode_Navigation);
		}
	}
	else{
		beep_for_command(false);
		reset_rcon_remote();
	}
}

