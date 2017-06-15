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

#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed 18
#endif


/*---------------------------------------------------------------- Charge Function ------------------------*/
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

	Set_LED(100,100);
	set_start_charge();
	wav_play(WAV_BATTERY_CHARGE);
	Set_Plan_Status(0);
	uint16_t bat_v;
	ROS_INFO("[gotocharger.cpp] Start charger mode.");
	while(ros::ok())
	{
		usleep(20000);
		bat_v = GetBatteryVoltage();

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
				Set_Clean_Mode(Clean_Mode_Navigation);
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

		if(!is_charge_on())//check if charger unplug
		{
			ROS_DEBUG("Leave charger");
			if (Stop_Charge_Counter > 25)
			{
				// Stop_Charge_Counter > 25 means robot has left charger stub for 0.5s.
				if (robot::instance()->isLowBatPaused())
				{
					ROS_INFO("[gotocharger.cpp] Exit charger mode and continue cleaning.");
					Set_Clean_Mode(Clean_Mode_Navigation);
					break;
				}

				ROS_INFO("[gotocharger.cpp] Exit charger mode and go to userinterface mode.");
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
			else
			{
				Stop_Charge_Counter++;
				if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
				{
					ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
					//wav_play(WAV_ERROR_LIFT_UP);
					robot::instance()->resetLowBatPause();
					Set_Clean_Mode(Clean_Mode_Userinterface);
					break;
				}
			}
		}
		else
		{
			Stop_Charge_Counter = 0;
		}
		/*----------------------------------------------------Check Key---------------------*/
		if(Get_Key_Press() & KEY_CLEAN)//							Check Key Clean
		{
			//Beep(5, 20, 0, 1);
//			Reset_Error_Code();
			if (is_direct_charge())
			{
				ROS_WARN("Can not go to navigation mode during direct charging.");
				Beep(Beep_Error_Sounds, 2, 0, 1);// Beep for invalid key.
				// Key release detection, if user has not release the key, don't do anything.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_WARN("%s %d: User hasn't release key.", __FUNCTION__, __LINE__);
					usleep(20000);
				}
			}
			else if (!Check_Bat_Ready_To_Clean())
			{
				ROS_WARN("Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(1400) + 60, can't go to navigation mode.");
				wav_play(WAV_BATTERY_LOW);
			}
			else if (is_on_charger_stub())
			{
				ROS_WARN("[gotocharger.cpp] Exit charger mode and go to navigation mode.");
				// Key release detection, if user has not release the key, don't do anything.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_WARN("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
					usleep(20000);
				}
				Set_Clean_Mode(Clean_Mode_Navigation);
				break;
			}
		}
		/*if(Remote_Key(Remote_Random))//							Check Remote Key Clean
		{
			set_stop_charge();
			Reset_Rcon_Remote();
			if(is_on_charger_stub())
			{
				Set_VacMode(Vac_Normal);
//				Set_Room_Mode(Room_Mode_Large);
//				Press_time=10;
//				while(Press_time--)
//				{
//					if(Remote_Key(Remote_Wallfollow))
//					{
//						Set_Room_Mode(Room_Mode_Auto);
//						Reset_Rcon_Remote();
//						break;
//					}
//					delay(500);
//				}
				Set_Clean_Mode(Clean_Mode_RandomMode);
				break;
			}
		}*/
		if(Get_Rcon_Remote()){
			if (Remote_Key(Remote_Clean)) {
				Reset_Rcon_Remote();
				if (is_direct_charge())
				{
					ROS_WARN("Can not go to navigation mode during direct charging.");
					Beep(Beep_Error_Sounds, 2, 0, 1);// Beep for invalid key.
				}
				else if (!Check_Bat_Ready_To_Clean())
				{
					ROS_WARN("Battery below BATTERY_READY_TO_CLEAN_VOLTAGE(1400) + 60, can't go to navigation mode.");
					wav_play(WAV_BATTERY_LOW);
				}
				else if (is_on_charger_stub())
				{
					Set_Clean_Mode(Clean_Mode_Navigation);
					break;
				}
			}
			else{
				Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
				Reset_Rcon_Remote();
			}
		}
		/* check plan setting*/
		switch (Get_Plan_Status())
		{
			case 1:
			{
				ROS_INFO("%s %d: Appointment received.", __FUNCTION__, __LINE__);
				Beep(2, 2, 0, 1);
				Set_Plan_Status(0);
				break;
			}
			case 2:
			{
				ROS_INFO("%s %d: Appointment canceled.", __FUNCTION__, __LINE__);
				wav_play(WAV_CANCEL_APPOINTMENT);
				Set_Plan_Status(0);
				break;
			}
			case 3:
			{
				ROS_INFO("%s %d: Appointment activated.", __FUNCTION__, __LINE__);
				if (Get_Error_Code() == Error_Code_None)
				{
					// Sleep for 50ms cause the status 3 will be sent for 3 times.
					usleep(50000);
					Set_Clean_Mode(Clean_Mode_Navigation);
				}
				else
				{
					ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
					Alarm_Error();
					wav_play(WAV_CANCEL_APPOINTMENT);
				}
				Set_Plan_Status(0);
				break;
			}
			case 4:
			{
				ROS_INFO("%s %d: Appointment succeeded.", __FUNCTION__, __LINE__);
				wav_play(WAV_APPOINTMENT_DONE);
				Set_Plan_Status(0);
				break;
			}

		}
		if (Get_Clean_Mode() == Clean_Mode_Navigation)
			break;

		/*-----------------------------------------------------Schedul Timer Up-----------------*/
//		if(Is_Alarm())
//		{
//			Reset_Alarm();
//			if(is_on_charger_stub())
//			{
//				Set_VacMode(Vac_Normal);
//				Set_Room_Mode(Room_Mode_Large);
//				Set_Clean_Mode(Clean_Mode_Navigation);
//				break;
//			}
//		}

		#ifdef ONE_KEY_DISPLAY
		if (Check_Bat_Full() && !Battery_Full)
		{
			Battery_Full = true;
			Set_LED(0,0);
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

			Set_LED(One_Display_Counter,One_Display_Counter);
		}
		#endif

	}
	set_stop_charge();
	// Wait for 20ms to make sure stop charging command has been sent.
	usleep(20000);
}

/*----------------------------------------------------------------GO Home  ----------------*/
void GoHome(void)
{

	uint32_t Receive_Code = 0;
//	Move_Forward(9,9);
//	Set_SideBrush_PWM(30,30);
//	Set_MainBrush_PWM(30);
	Reset_Rcon_Status();
	//delay(1500);
//	wav_play(WAV_BACK_TO_CHARGER);
	// This is for calculating the robot turning.
	float Current_Angle;
	float Last_Angle;
	float Angle_Offset;
	// This step is for counting angle change when the robot turns.
	float Gyro_Step = 0;

	Set_LED(100,100);
	Set_SideBrush_PWM(30,30);
	Set_MainBrush_PWM(30);

	Stop_Brifly();
	Reset_Rcon_Status();
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
			Disable_Motors();
			usleep(100000);
			if (is_charge_on())
			{
				Set_Clean_Mode(Clean_Mode_Charging);
				break;
			}
		}
		if (Stop_Event())
		{
			ROS_WARN("%s %d: Stop_Event in turning 360 degrees to find charger signal.", __FUNCTION__, __LINE__);
			Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			break;
		}

		//prompt for useless remote command
		if (Get_Rcon_Remote() > 0) {
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (Get_Rcon_Remote() & (Remote_Clean)) {
			} else {
				Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
				Reset_Rcon_Remote();
			}
		}

		if(Get_Bumper_Status())
		{
			Random_Back();
			if(Is_Bumper_Jamed())
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
		}
		Receive_Code = Get_Rcon_Status();
		Reset_Rcon_Status();
		if(Receive_Code&RconFL_HomeR)//FL H_R
		{
			ROS_INFO("Start with FL-R.");
			Turn_Left(Turn_Speed,900);
			Stop_Brifly();
			Around_ChargerStation(0);
			break;
		}
		if(Receive_Code&RconFR_HomeL)//FR H_L
		{
			ROS_INFO("Start with FR-L.");
			Turn_Right(Turn_Speed,900);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}

		if(Receive_Code&RconFL_HomeL)//FL H_L
		{
			ROS_INFO("Start with FL-L.");
			Turn_Right(Turn_Speed,900);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconFR_HomeR)//FR H_R
		{
			ROS_INFO("Start with FR-R.");
			Turn_Left(Turn_Speed,900);
			Stop_Brifly();
			Around_ChargerStation(0);
			break;
		}
		if(Receive_Code&RconFL2_HomeR)//FL2 H_R
		{
			ROS_INFO("Start with FL2-R.");
			Turn_Left(Turn_Speed,850);
			Stop_Brifly();
			Around_ChargerStation(0);
			break;
		}
		if(Receive_Code&RconFR2_HomeL)//FR2 H_L
		{
			ROS_INFO("Start with FR2-L.");
			Turn_Right(Turn_Speed,850);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}

		if(Receive_Code&RconFL2_HomeL)//FL2 H_L
		{
			ROS_INFO("Start with FL2-L.");
			Turn_Right(Turn_Speed,600);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconFR2_HomeR)//FR2 H_R
		{
			ROS_INFO("Start with FR2-R.");
			Turn_Left(Turn_Speed,600);
			Stop_Brifly();
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
			Turn_Left(Turn_Speed,1500);
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
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconFR_HomeT)//FR H_T
		{
			ROS_INFO("Start with FR-T.");
			Turn_Right(Turn_Speed,800);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}

		if(Receive_Code&RconFL2_HomeT)//FL2 H_T
		{
			ROS_INFO("Start with FL2-T.");
			Turn_Right(Turn_Speed,600);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconFR2_HomeT)//FR2 H_T
		{
			ROS_INFO("Start with FR2-T.");
			Turn_Right(Turn_Speed,800);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}

		if(Receive_Code&RconL_HomeT)// L  H_T
		{
			ROS_INFO("Start with L-T.");
			Turn_Right(Turn_Speed,1200);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}
		if(Receive_Code&RconR_HomeT)// R  H_T
		{
			ROS_INFO("Start with R-T.");
			Turn_Right(Turn_Speed,1200);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}

/*--------------BL BR---------------------*/
		if((Receive_Code&RconBL_HomeL))//BL H_L    //OK
		{
			ROS_INFO("Start with BL-L.");
			Turn_Left(30,800);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}
		if((Receive_Code&RconBR_HomeR))//BR H_L R  //OK
		{
			ROS_INFO("Start with BR-R.");
			Turn_Right(30,800);
			Stop_Brifly();
			Around_ChargerStation(0);
			break;
		}

		if((Receive_Code&RconBL_HomeR))//BL H_R
		{
			ROS_INFO("Start with BL-R.");
			Turn_Left(30,800);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}
		if((Receive_Code&RconBR_HomeL))//BL H_L R
		{
			ROS_INFO("Start with BR-L.");
			Turn_Right(30,800);
			Stop_Brifly();
			Around_ChargerStation(0);
			break;
		}

		if((Receive_Code&RconBL_HomeT))//BL H_T
		{
			ROS_INFO("Start with BL-T.");
			Turn_Left(30,300);
			Stop_Brifly();
			Around_ChargerStation(1);
			break;
		}
		if((Receive_Code&RconBR_HomeT))//BR H_T
		{
			ROS_INFO("Start with BR-T.");
			Turn_Right(30,300);
			Stop_Brifly();
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

		Set_Dir_Right();
		Set_Wheel_Speed(10, 10);
	}

	if (Gyro_Step >= 360)
		Set_Clean_Mode(Clean_Mode_Userinterface);

	// If robot didn't reach the charger, go back to userinterface mode.
	if(Get_Clean_Mode() != Clean_Mode_Charging && Get_Clean_Mode() != Clean_Mode_GoHome)
	{
		extern std::list <Point32_t> g_home_point;
		if (!Stop_Event() && g_home_point.empty())
		{
			Set_LED(100, 0);
			Stop_Brifly();
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
	Move_Forward(9,9);
	Set_SideBrush_PWM(30,30);
	Set_MainBrush_PWM(30);
	Set_BLDC_Speed(Vac_Speed_NormalL);
	//delay(500);
	Reset_Rcon_Status();
	Reset_Wheel_Step();
	Reset_Move_Distance();
	ROS_INFO("%s, %d: Call Around_ChargerStation with dir = %d.", __FUNCTION__, __LINE__, Dir);
	while(1)
	{
//		if(Get_LeftWheel_Step()<500)
//		{
//			Mobility_Temp_Error=0;
//			Temp_Mobility_Distance = Get_Move_Distance();
//		}
//		else
//		{
//			if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
//			{
//				Temp_Mobility_Distance = Get_Move_Distance();
//				if(Get_Mobility_Step()<1)
//				{
//					Mobility_Temp_Error++;
//					if(Mobility_Temp_Error>3)
//					{
//						Set_Clean_Mode(Clean_Mode_GoHome);
//						return;
//					}
//				}
//				else
//				{
//					Mobility_Temp_Error=0;
//				}
//				Reset_Mobility_Step();
//			}
//		}

		if(Get_Cliff_Trig())
		{
			if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
			{
				Disable_Motors();
				ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
				wav_play(WAV_ERROR_LIFT_UP);
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return;
			}

			while (Get_Cliff_Trig() && Cliff_Counter < 3)
			{
				// Move back until escape cliff triggered.
				Move_Back();
				Cliff_Counter++;
				usleep(40000);
				if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
				{
					Disable_Motors();
					ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
					wav_play(WAV_ERROR_LIFT_UP);
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return;
				}
			}
			if (Cliff_Counter == 3)
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
			}
			else
			{
				Turn_Left(Turn_Speed,1750);
				Move_Forward(9,9);
				Set_Clean_Mode(Clean_Mode_GoHome);
			}
			return;
		}

//		/*------------------------------------------------------Stop event-----------------------*/
		if(Stop_Event())
		{
			Stop_Brifly();
			if (Stop_Event())
			{
				//Beep(5, 20, 0, 1);
				// Key release detection, if user has not release the key, don't do anything.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_WARN("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
					usleep(20000);
				}
				// Do not reset Stop_Event_Status is for when robot is going home in navigation mode,
				// when stop event status is on,
				// it will know and won't go to next home point.
				if (! robot::instance()->isLowBatPaused())
					if (! robot::instance()->isManualPaused())
						Reset_Stop_Event_Status();
			}
			// If key pressed, go back to user interface mode.
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}

		//prompt for useless remote command
		if (Get_Rcon_Remote() > 0) {
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (Get_Rcon_Remote() & (Remote_Clean)) {
			} else {
				Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
				Reset_Rcon_Remote();
			}
		}

		if(Check_Bat_Stop())
		{
			ROS_WARN("%s %d: Battery too low (< LOW_BATTERY_STOP_VOLTAGE)", __FUNCTION__, __LINE__);
			//delay(10000);
			usleep(1000000);
			Set_Clean_Mode(Clean_Mode_Sleep);
			return;
		}

//		if(Home_Check_Current())return;

		if(Get_Bumper_Status())
		{
			Bumper_Counter++;
			Random_Back();
			if(Is_Bumper_Jamed())
			{
				return;
			}
			if(Dir)
			{
				// Robot at left side of charger stub.
				Turn_Left(Turn_Speed,1800);
			}
			else
			{
				// Robot at right side of charger stub.
				Turn_Right(Turn_Speed,1800);
			}
			Reset_Rcon_Status();
			Move_Forward(10,10);
			//???
			Dir = 1-Dir;
			if(Bumper_Counter>1)
			{
				Set_Clean_Mode(Clean_Mode_GoHome);
				return ;
			}
			//Reset_Wheel_Step();
			No_Signal_Counter=0;
		}

		if(is_charge_on())
		{
			ROS_DEBUG("%s %d: is_charge_on!!", __FUNCTION__, __LINE__);
			Disable_Motors();
			Stop_Brifly();
//			delay(2000);
			usleep(200000);
			if(is_charge_on())
			{
				//delay(5000);
				usleep(200000);
				if(is_charge_on())
				{
//					Reset_Error_Code();
					Set_Clean_Mode(Clean_Mode_Charging);
//					Beep(2, 25, 0, 1);
//					Reset_Rcon_Remote();
					return;
				}
			}
			else if(Turn_Connect())
			{
				Set_Clean_Mode(Clean_Mode_Charging);
//				Reset_Rcon_Remote();
				return;
			}
			else
			{
				Set_SideBrush_PWM(30,30);
				Set_MainBrush_PWM(0);
				////Back(30,800);
				//Back(30,300);
				quick_back(30,300);
				Set_MainBrush_PWM(30);
				Stop_Brifly();
			}
			if (Stop_Event())
			{
				ROS_WARN("%s %d: Stop_Event in Turn_Connect.", __FUNCTION__, __LINE__);
				Disable_Motors();
				return;
			}
		}

		Temp_Rcon_Status = Get_Rcon_Status();
		if(Temp_Rcon_Status)
		{
			No_Signal_Counter=0;
			Reset_Rcon_Status();
		}
		else
		{
			No_Signal_Counter++;
			if(No_Signal_Counter>80)
			{
				//Beep(1, 25, 75, 3);
				ROS_WARN("No charger signal received.");
				Set_Clean_Mode(Clean_Mode_GoHome);
				return ;
			}
		}
		/*
		if(Temp_Rcon_Status&0x4000)
		{
			Turn_Right(30,2200);
			Move_Forward(10,10);
			Dir=1-Dir;
		}*/
		ROS_DEBUG("%s %d Check DIR: %d, and do something", __FUNCTION__, __LINE__, Dir);
		if(Dir == 1)//10.30
		{
//			if(Get_RightWheel_Step()>20000)
//			{
//				Stop_Brifly();
//				Turn_Right(Turn_Speed,2200);
//				Set_Clean_Mode(Clean_Mode_GoHome);
//				return ;
//			}

			if(Temp_Rcon_Status&RconL_HomeT)  //L_T
			{
//				if((++LTSignal_Count)>=3)
//				{
//					LTSignal_Count = 0;
//					LLSignal_Count = 0;
					ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
					Move_Forward(19,5);
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
					Move_Forward(19,5);
//					Uniform_Forward(28,22);
//					Delay_Arounding(50);
					usleep(100000);
//				}
			}
			else if(Temp_Rcon_Status&RconL_HomeR)  //L_R  9 18
			{
				ROS_DEBUG("%s, %d: Detect L-R.", __FUNCTION__, __LINE__);
				Move_Forward(17,9);
//				Uniform_Forward(25,10);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFL2_HomeT)  //FL2_T
			{
				ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
				Move_Forward(16,19);
//				Uniform_Forward(25,10);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFL2_HomeL)  //FL_HL
			{
				ROS_DEBUG("%s, %d: Detect FL2-L.", __FUNCTION__, __LINE__);
				Move_Forward(15,11);
//				Uniform_Forward(25,10);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else  if(Temp_Rcon_Status&RconFL2_HomeR)//FL2_HR
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				Move_Forward(9,15);
//				Uniform_Forward(25,10);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFL_HomeL)	//FR_HL
			{
				ROS_DEBUG("%s, %d: Detect FL-L.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Right(Turn_Speed,500);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFL_HomeR)	 //FR_HL
			{
				ROS_DEBUG("%s, %d: Detect FL-R.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Left(Turn_Speed,600);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFL_HomeT)	 //FR_HT
			{
				ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Right(Turn_Speed,500);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFR_HomeT)	 //R_HT
			{
				ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Right(Turn_Speed,800);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFR2_HomeT) //FR2_T //OK
			{
				ROS_DEBUG("%s, %d: Detect FR2-T.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Right(Turn_Speed,900);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconR_HomeT)  //OK
			{
				ROS_DEBUG("%s, %d: Detect R-T.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Right(Turn_Speed,1100);
				Move_Forward(5,5);
				Dir = 0;
			}
			else
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				Move_Forward(16,34);  //0K (16,35)	  1100
//				Uniform_Forward(14,31);
//				Delay_Arounding(110);
				usleep(100000);
			}

			if(Temp_Rcon_Status&(RconFR_HomeR))
			{
				ROS_DEBUG("%s, %d: Detect FR-R, call By_Path().", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				By_Path();
				return;
			}
			if(Temp_Rcon_Status&(RconFL_HomeR))
			{
				ROS_DEBUG("%s, %d: Detect FL-R, call By_Path().", __FUNCTION__, __LINE__);
//				Stop_Brifly();
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
					Stop_Brifly();
					Temp_Position = Check_Position(Round_Left);
					ROS_DEBUG("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Position);
					Stop_Brifly();
					if(Temp_Position==1)
					{
//						Reset_Error_Code();
//						SetDisplayError(Error_Code_None);
//						Reset_Stop_Event_Status();
						ROS_INFO("%s %d return to Clean_Mode_Userinterface", __FUNCTION__, __LINE__);
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return;
					}
					if(Temp_Position==2)
					{
						ROS_DEBUG("%s %d call By_Path()", __FUNCTION__, __LINE__);
						//Move_Forward(1,1);
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
//			if(Get_LeftWheel_Step()>20000)
//			{
//				Stop_Brifly();
//				Turn_Left(Turn_Speed,2200);
//				Set_Clean_Mode(Clean_Mode_GoHome);
//				return ;
//			}
			if(Temp_Rcon_Status&RconR_HomeT)   // OK ,(10,26)
			{
				ROS_DEBUG("%s %d Detect R-T.", __FUNCTION__, __LINE__);
				Move_Forward(5,19);
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
				Move_Forward(5,19);
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
				Move_Forward(9,17);
				usleep(100000);
//				Delay_Arounding(100);
			}
			else if(Temp_Rcon_Status&RconFR2_HomeT)   //turn left
			{
				ROS_DEBUG("%s %d Detect FR2-T.", __FUNCTION__, __LINE__);
				Move_Forward(19,17);
//				Uniform_Forward(10,25);
				usleep(100000);
//				Delay_Arounding(60);
			}
			else if(Temp_Rcon_Status&RconFR2_HomeR)  //OK
			{
				ROS_DEBUG("%s %d Detect FR2-R.", __FUNCTION__, __LINE__);
				Move_Forward(11,15);
//				Uniform_Forward(10,25);
//				Delay_Arounding(100);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFR2_HomeL)  //
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				Move_Forward(15,9);
				usleep(100000);
//				Uniform_Forward(10,25);
//				Delay_Arounding(100);
			}
			else if(Temp_Rcon_Status&RconFR_HomeR)	//OK
			{
				ROS_DEBUG("%s, %d: Detect FR-R.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Left(Turn_Speed,500);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFR_HomeL)	//OK
			{
				ROS_DEBUG("%s, %d: Detect FR-L.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Right(Turn_Speed,600);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFR_HomeT)	//ok
			{
				ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Left(Turn_Speed,500);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFL_HomeT)	//OK
			{
				ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Left(Turn_Speed,800);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFL2_HomeT)  //OK
			{
				ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Left(Turn_Speed,900);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconL_HomeT)  //OK
			{
				ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
//				Stop_Brifly();
				Turn_Left(Turn_Speed,1100);
				Move_Forward(5,5);
				Dir = 1;
			}
			else
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				Move_Forward(34,16);
//				Move_Forward(31,14);  //0K (35,13)
//				Delay_Arounding(110);
				usleep(100000);
			}

			if(Temp_Rcon_Status&(RconFL_HomeL))
			{
				ROS_DEBUG("%s, %d: Detect FL-L, call By_Path().", __FUNCTION__, __LINE__);
//				Stop_Brifly();
//				Turn_Right(30,500);
				By_Path();
				return;
			}
			if(Temp_Rcon_Status&(RconFR_HomeL))
			{
				ROS_DEBUG("%s, %d: Detect FR-L, call By_Path().", __FUNCTION__, __LINE__);
//				Stop_Brifly();
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
					Stop_Brifly();
					Temp_Position = Check_Position(Round_Right);
					ROS_DEBUG("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Position);
					Stop_Brifly();
					if(Temp_Position==1)
					{
//						Reset_Error_Code();
//						SetDisplayError(Error_Code_None);
//						Reset_Stop_Event_Status();
						ROS_INFO("%s %d return to Clean_Mode_Userinterface", __FUNCTION__, __LINE__);
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return;
					}
					if(Temp_Position==2)
					{
						ROS_DEBUG("%s %d call By_Path()", __FUNCTION__, __LINE__);
						//Move_Forward(1,1);
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
		Set_Dir_Left();
	}
	else if(Dir == Round_Right)
	{
		ROS_DEBUG("Check position Dir = right");
		Set_Dir_Right();
	}
	Set_Wheel_Speed(10,10);

	Last_Angle = robot::instance()->getAngle();
	ROS_DEBUG("Last_Angle = %f.", Last_Angle);

//	while(Get_LeftWheel_Step()<3600)
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
		//	if(Is_Encoder_Fail())
		//	{
		//		Set_Error_Code(Error_Code_Encoder);
		//	}
		//	return 1;
		//}
		Receive_Code = (Get_Rcon_Status()&(RconL_HomeL|RconL_HomeR|RconFL_HomeL|RconFL_HomeR|RconR_HomeL|RconR_HomeR|RconFR_HomeL|RconFR_HomeR));
		ROS_DEBUG("Check_Position Get_Rcon_Status() == %x, R... == %x, receive code: %x.", Get_Rcon_Status(), (RconL_HomeL|RconL_HomeR|RconFL_HomeL|RconFL_HomeR|RconR_HomeL|RconR_HomeR|RconFR_HomeL|RconFR_HomeR), Receive_Code);
		if(Receive_Code)
		{
			Reset_Rcon_Status();
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
		if(Stop_Event())
		{
			//Beep(5, 20, 0, 1);
			Stop_Brifly();
			// Key release detection, if user has not release the key, don't do anything.
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_WARN("%s %d: User hasn't release key.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Do not reset Stop_Event_Status is for when robot is going home in navigation mode,
			// when stop event status is on, it will know and won't go to next home point.
			if (!robot::instance()->isLowBatPaused())
				if (!robot::instance()->isManualPaused())
					Reset_Stop_Event_Status();
			return 1;
		}

		//prompt for useless remote command
		if (Get_Rcon_Remote() > 0) {
			ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
			if (Get_Rcon_Remote() & (Remote_Clean)) {
			} else {
				Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
				Reset_Rcon_Remote();
			}
		}

		if(is_charge_on())
		{
			ROS_DEBUG("%s %d: is_charge_on!!", __FUNCTION__, __LINE__);
			Disable_Motors();
			Stop_Brifly();
//			delay(2000);
			usleep(200000);
			if(is_charge_on())
			{
				//delay(5000);
				usleep(200000);
				if(is_charge_on())
				{
//					Reset_Error_Code();
					Set_Clean_Mode(Clean_Mode_Charging);
//					Beep(2, 25, 0, 1);
//					Reset_Rcon_Remote();
					return 2;
				}
			}
			else if(Turn_Connect())
			{
				Set_Clean_Mode(Clean_Mode_Charging);
//				Reset_Rcon_Remote();
				return 2;
			}
			else
			{
				Set_SideBrush_PWM(30,30);
				Set_MainBrush_PWM(0);
				////Back(30,800);
				//Back(30,300);
				quick_back(30,300);
				Set_MainBrush_PWM(30);
				Stop_Brifly();
			}
			if (Stop_Event())
			{
				ROS_WARN("%s %d: Stop_Event in Turn_Connect.", __FUNCTION__, __LINE__);
				Disable_Motors();
				return 1;
			}
		}
		uint8_t octype = Check_Motor_Current();
		if(octype){
			if(Self_Check(octype)){
				ROS_INFO("%s ,%d motor over current ",__FUNCTION__,__LINE__);
				return 1;
			}
		}
	}
//	Reset_TempPWM();
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

//	Reset_Wheel_Step();

	Reset_Stop_Event_Status();
//	Display_Content(LED_Home,100,100,0,7);

//	Enable the charge function
	set_start_charge();

	Move_Forward(9,9);
	Set_SideBrush_PWM(30,30);
	Set_MainBrush_PWM(30);
	Set_BLDC_Speed(Vac_Speed_NormalL);
//	SetHomeRemote();

//	Beep(1);

	while(1)
	{
		Receive_Code = 0;
//		if(Get_LeftWheel_Step()<500)
//		{
//			Mobility_Temp_Error=0;
//			Temp_Mobility_Distance = Get_Move_Distance();
//		}
//		else
//		{
//			if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
//			{
//				Temp_Mobility_Distance = Get_Move_Distance();
//				if(Get_Mobility_Step()<1)
//				{
//					Mobility_Temp_Error++;
//					if(Mobility_Temp_Error>3)
//					{
//						Set_Clean_Mode(Clean_Mode_GoHome);
//						return;
//					}
//				}
//				else
//				{
//					Mobility_Temp_Error=0;
//				}
//				Reset_Mobility_Step();
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
				Disable_Motors();
				Stop_Brifly();
//				delay(2000);
				usleep(200000);
				if(is_charge_on())
				{
					//delay(5000);
					usleep(200000);
					if(is_charge_on())
					{
//						Reset_Error_Code();
						Set_Clean_Mode(Clean_Mode_Charging);
//						Beep(2, 25, 0, 1);
//						Reset_Rcon_Remote();
						return;
					}
				}
				else if(Turn_Connect())
				{
					Set_Clean_Mode(Clean_Mode_Charging);
//					Reset_Rcon_Remote();
					return;
				}
				else
				{
					Set_SideBrush_PWM(30,30);
					Set_MainBrush_PWM(0);
					////Back(30,800);
					//Back(30,300);
					quick_back(30,300);
					Set_MainBrush_PWM(30);
					Stop_Brifly();
				}
				if (Stop_Event())
				{
					ROS_WARN("%s %d: Stop_Event in Turn_Connect.", __FUNCTION__, __LINE__);
					Disable_Motors();
					return;
				}

				//prompt for useless remote command
				if (Get_Rcon_Remote() > 0) {
					ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
					if (Get_Rcon_Remote() & (Remote_Clean)) {
					} else {
						Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
						Reset_Rcon_Remote();
					}
				}
			}
			/*----------------------------------------------OBS------------------Event---------------*/
			//ROS_DEBUG("get_Left_bumper_Status");
			if(Get_Bumper_Status()&LeftBumperTrig)
			{
//				Random_Back();
				Reset_Rcon_Status();
				if(!Position_Far)
				{
					Stop_Brifly();
					if(Turn_Connect())
					{
						Set_Clean_Mode(Clean_Mode_Charging);
						ROS_INFO("Set Clean_Mode_Charging and return");
						return;
					}
					if (Stop_Event())
					{
						ROS_WARN("%s %d: Stop_Event in Turn_Connect.", __FUNCTION__, __LINE__);
						Disable_Motors();
						return;
					}

					//prompt for useless remote command
					if (Get_Rcon_Remote() > 0) {
						ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
						if (Get_Rcon_Remote() & (Remote_Clean)) {
						} else {
							Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
							Reset_Rcon_Remote();
						}
					}
					Set_SideBrush_PWM(30,30);
					Set_MainBrush_PWM(0);
//					Back(30,2500);//waiting
					quick_back(30,300);//waiting
					ROS_DEBUG("%d: quick_back in !position_far", __LINE__);
					Set_MainBrush_PWM(30);
					Stop_Brifly();
					if(Bumper_Counter>0)
					{
						Move_Forward(0,0);
						Set_Clean_Mode(Clean_Mode_GoHome);
						ROS_DEBUG("%d, Return from LeftBumperTrig.", __LINE__);
						return;
					}
				}
				else if((Get_Rcon_Status()&(RconFL2_HomeL|RconFL2_HomeR|RconFR2_HomeL|RconFR2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR))==0)
				{
					Random_Back();
					Turn_Right(Turn_Speed,1100);
					Move_Forward(8,8);
					Set_Clean_Mode(Clean_Mode_GoHome);
					ROS_DEBUG("%d, Return from LeftBumperTrig.", __LINE__);
					return;
				}
				else
				{
					Random_Back();
					Turn_Right(Turn_Speed,1100);
					Set_SideBrush_PWM(30,30);
					Set_MainBrush_PWM(30);
					Move_Forward(8,8);
				}
				if(Is_Bumper_Jamed())
				{
					return;
				}
				Bumper_Counter++;
				ROS_DEBUG("%d, Left bumper count =%d.", __LINE__, Bumper_Counter);
			}
			//ROS_DEBUG("Get_Right_Bumper_Status");
			if(Get_Bumper_Status()&RightBumperTrig)
			{
//				Random_Back();
				Reset_Rcon_Status();
				if(!Position_Far)
				{
					Stop_Brifly();
					if(Turn_Connect())
					{
						Set_Clean_Mode(Clean_Mode_Charging);
						return;
					}
					if (Stop_Event())
					{
						ROS_WARN("%s %d: Stop_Event in Turn_Connect.", __FUNCTION__, __LINE__);
						Disable_Motors();
						return;
					}
	
					//prompt for useless remote command
					if (Get_Rcon_Remote() > 0) {
						ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
						if (Get_Rcon_Remote() & (Remote_Clean)) {
						} else {
							Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
							Reset_Rcon_Remote();
						}
					}
					Set_SideBrush_PWM(30,30);
					Set_MainBrush_PWM(0);
					quick_back(30,300);
					Set_MainBrush_PWM(30);
					Stop_Brifly();
					if(Bumper_Counter>0)
					{
						Move_Forward(0,0);
						Set_Clean_Mode(Clean_Mode_GoHome);
						ROS_DEBUG("%d, Return from RightBumperTrig.", __LINE__);
						return;
					}
				}
				else if((Get_Rcon_Status()&(RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFL2_HomeL|RconFL2_HomeR|RconFR2_HomeL|RconFR2_HomeR))==0)
				{
					Random_Back();
					Turn_Left(Turn_Speed,1100);
					Move_Forward(8,8);
					Set_Clean_Mode(Clean_Mode_GoHome);
					ROS_DEBUG("%d, Return from RightBumperTrig.", __LINE__);
					return;
				}
				else
				{
					Random_Back();
					Turn_Left(Turn_Speed,1100);
					Set_SideBrush_PWM(30,30);
					Set_MainBrush_PWM(30);
					Move_Forward(8,8);
				}
				if(Is_Bumper_Jamed())return;
				Bumper_Counter++;
				ROS_DEBUG("%d, Right bumper count =%d.", __LINE__, Bumper_Counter);
			}

			if(Position_Far)
			{
				if(Get_Cliff_Trig())
				{
					if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
					{
						Disable_Motors();
						ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
						wav_play(WAV_ERROR_LIFT_UP);
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return;
					}
					Move_Back();
					Move_Back();
					Turn_Left(Turn_Speed,1750);
					Move_Forward(9,9);
					Set_Clean_Mode(Clean_Mode_GoHome);
					return;
				}
			}
			else
			{
				if(Get_Cliff_Trig())
				{
					if (Get_Cliff_Trig() == (Status_Cliff_Left | Status_Cliff_Front | Status_Cliff_Right))
					{
						Disable_Motors();
						ROS_WARN("%s, %d robot lift up\n", __FUNCTION__, __LINE__);
						wav_play(WAV_ERROR_LIFT_UP);
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return;
					}
					Set_Wheel_Speed(0,0);
					Set_Dir_Backward();
//					delay(300);
					usleep(30000);
					if(Get_Cliff_Trig())
					{
						Move_Back();
						Move_Back();
						Turn_Left(Turn_Speed,1750);
						Move_Forward(9,9);
						Set_Clean_Mode(Clean_Mode_GoHome);
						return;
					}
					Set_Dir_Forward();
					break;
				}
			}

			/*------------------------------------------------------stop event-----------------------*/
			if(Stop_Event())
			{
				//Beep(5, 20, 0, 1);
				Stop_Brifly();
				// Key release detection, if user has not release the key, don't do anything.
				while (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_WARN("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
					usleep(20000);
				}
				// Do not reset Stop_Event_Status is for when robot is going home in navigation mode,
				// when stop event status is on,
				// it will know and won't go to next home point.
				if (!robot::instance()->isLowBatPaused())
					if (!robot::instance()->isManualPaused())
						Reset_Stop_Event_Status();

				Set_Clean_Mode(Clean_Mode_Userinterface);
				return;
			}
			//prompt for useless remote command
			if (Get_Rcon_Remote() > 0) {
				ROS_INFO("%s %d: Rcon", __FUNCTION__, __LINE__);
				if (Get_Rcon_Remote() & (Remote_Clean)) {
				} else {
					Beep(Beep_Error_Sounds, 2, 0, 1);//Beep for useless remote command
					Reset_Rcon_Remote();
				}
			}
			if(Check_Bat_Stop())
			{
//				delay(10000);
				usleep(1000000);
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return;
			}
			if(Home_Check_Current())return;
//			delay(100);
			usleep(10000);
		}


		Receive_Code = Get_Rcon_Status();
		Temp_Code = Receive_Code;
//		Temp_Code &= 0x003f0fff;
		Temp_Code &= (	RconL_HomeL|RconL_HomeT|RconL_HomeR| \
						RconFL2_HomeL|RconFL2_HomeT|RconFL2_HomeR| \
						RconFL_HomeL|RconFL_HomeT|RconFL_HomeR| \
						RconFR_HomeL|RconFR_HomeT|RconFR_HomeR| \
						RconFR2_HomeL|RconFR2_HomeT|RconFR2_HomeR| \
						RconR_HomeL|RconR_HomeT|RconR_HomeR \
					 );
//		Reset_Rcon_Status();
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
						Move_Forward(9,9);
						ROS_INFO("%s, %d: Robot goes far, back to gohome mode.", __FUNCTION__, __LINE__);
						Set_Clean_Mode(Clean_Mode_GoHome);
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
			Reset_Rcon_Status();
			NoSignal_Counter = 0;
		}
		else
		{
			Near_Counter = 0;
			NoSignal_Counter++;
			if(NoSignal_Counter>50)
			{
				NoSignal_Counter = 0;
				Stop_Brifly();
				Temp_Check_Position = Check_Position(Round_Left);
				ROS_DEBUG("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Check_Position);
				if(Temp_Check_Position == 1)
				{
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return;
				}
				else if(Temp_Check_Position == 0)
				{
					ROS_INFO("%s, %d: Robot can't see charger, return to gohome mode.", __FUNCTION__, __LINE__);
					Stop_Brifly();
					Turn_Right(Turn_Speed,1000);
					Stop_Brifly();
					Move_Forward(10,10);
					Set_Clean_Mode(Clean_Mode_GoHome);
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
//				case 0x024:   Move_Forward(12,12);break;			//FL_L/FR_R
				case (RconFL_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,12);
					break;

//				case 0x03c:   Move_Forward(12,12);break;		  //FL_L/FL_R/FR_L/FR_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,12);
					break;

//				case 0xbd:	  Move_Forward(12,12);break;			//FL2_L/FL_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,12);
					break;

//				case 0xA5:	  Move_Forward(12,12);break;			//FL2_L/FL_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,12);
					break;

//				case 0X0AD:   Move_Forward(13,11);break;			//FL2_L/FL_L/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(13,11);
					break;

//				case 0x08D:   Move_Forward(13,9);break;				//FL2_L/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(13,9);
					break;

//				case 0x089:   Move_Forward(12,9);break;				//FL2_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,9);
					break;

//				case 0x02C:   Move_Forward(11,9);break;				//FL_L/FR_L/FR_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,9);
					break;

//				case 0x00D:   Move_Forward(12,8);break;				//FR_L/FR_R/FR2_R
				case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,8);
					break;

//				case 0x00C:   Move_Forward(12,8);break;				//FR_L/FR_R
				case (RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,8);
					break;

//				case 0x008:   Move_Forward(11,8);break;				//FR_L
				case (RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(11,8);
					break;

//				case 0x00F:   Move_Forward(11,7);break;				//FR_L/FR_R/FR2_L/FR2_R
				case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,7);
					break;

//				case 0x009:   Move_Forward(13,7);break;				//FR_L/FR2_R
				case (RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,7);
					break;

//				case 0x00B:   Move_Forward(12,7);break;				//FR_L/FR2_L/FR2_R
				case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,7);
					break;

//				case 0x001:   Move_Forward(12,6);break;				//FR2_R
				case (RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,6);
					break;

//				case 0x002:   Turn_Right(20,250);				   //FRR_L
				case (RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x003:   Turn_Right(20,350);				   //FR2_L/FR2_R
				case (RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,350);
					Stop_Brifly();
					Move_Forward(10,3);
					break;

//				case 0x28:	  Move_Forward(11,10);break;			 //FL_L/FR_L
				case (RconFL_HomeL|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(11,10);
					break;

//				case 0x2B:	  Move_Forward(11,9);break;				 //FL_L/FR_L/FR2_L/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,9);
					break;

//				case 0x29:	  Move_Forward(12,10);break;			 //FL_L/FR_L/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,10);
					break;

//				case 0X0A:	  Move_Forward(12,7);break;				 //FR_L/FR2_L
				case (RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
					Move_Forward(12,7);
					break;

//				case 0X54:	  Move_Forward(12,11);break;			 //FL_L/FR_L/FR2_L
				case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0X2D:	  Move_Forward(13,10);break;			 //FL_L/FR_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,10);
					break;

//				case 0X05:	  Move_Forward(13,8);break;				 //FR_R/FR2_R
				case (RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,8);
					break;

//				case 0x04:	  Move_Forward(12,11);break;
				case (RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0x45:	  Move_Forward(12,10);break;			 //FL2_R/FR_R/FR2_R
				case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,10);
					break;

//				case 0xc5:	  Move_Forward(12,10);break;			 //FL2_L/FL2_R/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,10);
					break;

//				case 0xCD:	  Move_Forward(13,7);break;				   //FL2_L/FL2_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,7);
					break;

//				case 0xcf:	  Move_Forward(13,7);break;				 //FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,7);
					break;

//				case 0xA9:	  Move_Forward(12,11);break;			 //FL2_L/FL_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0x85:	  Move_Forward(13,9);break;
				case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,9);
					break;

//				case 0x8f:	  Move_Forward(13,8);break;				 //FL2_L/FR_L/FR_R/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,8);
					break;

//				case 0x84:	  Move_Forward(13,9);break;
				case (RconFL2_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,9);
					break;

//				case 0x31:	  Move_Forward(12,11);break;			 //FL_L/FL_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0x9D:	  Move_Forward(12,11);break;			 //FL2_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0xAC:	  Move_Forward(12,11);break;			 //FL2_L/FL_L/FR_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0xbc:	  Move_Forward(12,11);break;			 //FL2_L/FL_L/FL_R/FR_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0x25:	  Move_Forward(12,11);break;			 //FL_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0xb8:	  Move_Forward(13,12);break;			 //FL2_L/FL_L/FL_R/FR_L
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(13,12);
					break;

//				case 0xA8:	  Move_Forward(13,12);break;			 //FL2_L/FL_L/FR_l
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(13,12);
					break;

//				case 0x803:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: Position_Far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						Turn_Right(20,250);
						Stop_Brifly();
						Move_Forward(9,3);
						break;

//				case 0x802:   Turn_Right(20,400);
				case (RconR_HomeR|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,400);
					Stop_Brifly();
					Move_Forward(8,3);
					break;

//				case 0x80b:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					Stop_Brifly();
					Move_Forward(10,4);
					break;

//				case 0xC02:	  Turn_Right(20,450);
				case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,450);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0xC00 :  Turn_Right(20,500);
				case (RconR_HomeR|RconR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/R_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					Stop_Brifly();
					Move_Forward(9,6);
					break;

//				case 0x400 :  Turn_Right(20,550);
				case (RconR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,550);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x80A:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x402:   Turn_Right(20,500);
				case (RconR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x805:
				case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,2);
					break;

//				case 0x801:   Move_Forward(9,0);
				case (RconR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,0);
					break;

//				case 0x101:   Move_Forward(9,1);
				case (RconL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,1);
					break;

//				case 0x185:   Move_Forward(9,6);
				case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,6);
					break;

//				case 0x800:   Move_Forward(10,0);
				case (RconR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,0);
					break;

//				case 0x8B:	  Move_Forward(13,12);break;						//FL2_L/FR_L/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,12);
					break;

//				case 0x22:	  Move_Forward(13,12);break;			//FL2_R/FR_R
				case (RconFL_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
					Move_Forward(13,12);
					break;

//				case 0x81:	  Move_Forward(14,4);break;
				case (RconFL2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(14,4);
					break;

//				case 0x23:	  Move_Forward(12,11);break;
				case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(12,11);
					break;

//				case 0Xb5:	  Move_Forward(10,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,12);
					break;

//				case 0xb1:	  Move_Forward(8,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,12);
					break;

//				case 0x91:	  Move_Forward(11,13);break;
				case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,13);
					break;

//				case 0x34:	  Move_Forward(10,12);break;
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,12);
					break;

//				case 0xb0:	  Move_Forward(9,13);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,13);
					break;

//				case 0x30:	  Move_Forward(10,12);break;
				case (RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,12);
					break;

//				case 0x10:	  Move_Forward(7,11);break;
				case (RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,11);
					break;

//				case 0xf0:	  Move_Forward(7,12);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break;

//				case 0x90:	  Move_Forward(9,13);break;
				case (RconFL2_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,13);
					break;

//				case 0xD0:	  Move_Forward(8,13);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,13);
					break;

//				case 0x80:	  Move_Forward(8,14);break;
				case (RconFL2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L.", __FUNCTION__, __LINE__);
					Move_Forward(8,14);
					break;

//				case 0x40:	  Turn_Left(20,250);
				case (RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,250);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0xC0:	  Turn_Left(20,350);
				case (RconFL2_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,350);
					Stop_Brifly();
					Move_Forward(3,10);
					break;

//				case 0x14:	  Move_Forward(11,12);break;
				case (RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0xD4:	  Move_Forward(10,12);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,12);
					break;

//				case 0x94:	  Move_Forward(10,12);break;
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,12);
					break;

//				case 0X50:	  Move_Forward(7,12);break;
				case (RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break;

//				case 0X2A:	  Move_Forward(11,12);break;
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0XB4:	  Move_Forward(9,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,12);
					break;

//				case 0XA0:	  Move_Forward(7,12);break;
				case (RconFL2_HomeL|RconFL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break;

//				case 0x20:	  Move_Forward(11,12);break;
				case (RconFL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_L.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0xA2:	  Move_Forward(10,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
					Move_Forward(10,12);
					break;

//				case 0xA3:	  Move_Forward(10,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,12);
					break;

//				case 0xB3:	  Move_Forward(7,12);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break;

//				case 0xF3:	  Move_Forward(7,12);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break;

//				case 0x95:	  Move_Forward(11,12);break;			//FL2_L/FL_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0xA1:	  Move_Forward(7,12);break;				//FL2_L/FL_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break;

//				case 0xF1:	  Move_Forward(7,12);break;				//FL2_L/FL2_R/FL_L/FL_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break;

//				case 0x21:	  Move_Forward(10,12);break;			//FL2_L/FL_L/FR2_R //FL2_L/FR_R
				case (RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,12);
					break;

//				case 0x8c:	  Move_Forward(11,12);break;			//FL2_L/FL_L/FR2_R //FL2_L/FR_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0xb9:	  Move_Forward(11,12);break;			//FL2_L/FL_R/FL_R/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0x35:	  Move_Forward(11,12);break;			//FL_L/FL_R/FR_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0x3d:	  Move_Forward(11,12);break;			//FL_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0xa4:	  Move_Forward(11,12);break;			//FL2_L/FL_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0x1d:	  Move_Forward(11,12);break;			//FL_R/FR_L/FR_R/FR2_R
				case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0x15:	  Move_Forward(9,12);break;				//FL_R/FR_R/FR2_R
				case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,12);
					break;

//				case 0x1C0:   Turn_Left(20,250);
				case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,250);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x140:   Turn_Left(20,400);
				case (RconL_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,400);
					Stop_Brifly();
					Move_Forward(3,8);
					break;

//				case 0x1d0:   Turn_Left(20,250);
				case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,250);
					Stop_Brifly();
					Move_Forward(4,10);
					break;

//				case 0x340:   Turn_Left(20,450);
				case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,450);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x300 :  Turn_Left(20,500);
				case (RconL_HomeR|RconL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, L_R/L_L.", __FUNCTION__, __LINE__);
					Turn_Left(20,500);
					Stop_Brifly();
					Move_Forward(6,9);
					break;

//				case 0x200:   Turn_Left(20,550);
				case (RconL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,550);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x150:   Turn_Left(20,250);
				case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,250);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x240:   Turn_Left(20,500);
				case (RconL_HomeR|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, L_R/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,500);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x1A0:
				case (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L/FL_L.", __FUNCTION__, __LINE__);
					Move_Forward(2,9);
					break;

//				case 0x180:   Move_Forward(0,9);
				case (RconL_HomeL|RconFL2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, L_L/FL2_L.", __FUNCTION__, __LINE__);
					Move_Forward(0,9);
					break;

//				case 0x880:   Move_Forward(1,9);
				case (RconR_HomeR|RconFL2_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FL2_L.", __FUNCTION__, __LINE__);
					Move_Forward(1,9);
					break;

//				case 0x8A1:   Move_Forward(6,9);
				case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,9);
					break;

//				case 0x100:   Move_Forward(0,10);break;
				case (RconL_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, L_L.", __FUNCTION__, __LINE__);
					Move_Forward(0,10);
					break;

//				case 0xD1:	  Move_Forward(11,12);break;		  //FL_R/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0x44:	  Move_Forward(11,12);break;		  //FL_L/FR2_L
				case (RconFL2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

//				case 0x18:		Move_Forward(7,12);break;
				case (RconFL_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: Position_Far, FL_R/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break;

//				case 0xC4:		Move_Forward(11,12);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: Position_Far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(11,12);
					break;

				default:
					ROS_DEBUG("%s, %d: Position_Far, else:%x.", __FUNCTION__, __LINE__, Temp_Code);
					Move_Forward(10,11);
//					USPRINTF("**************default angle is ,default code	is 0x%x\n",Temp_Code);
					break;
			}
		}
		else
		{
			switch(Temp_Code)
			{
//				case 0x024:		Move_Forward(8,8);	break;			//FL_L/FR_R
				case (RconFL_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL-L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,8);
					break;

//				case 0x3c:	 Move_Forward(9,9);break;			  //FL_L/FL_R/FR_L/FR_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,9);
					break;
//				case 0xbd:	  Move_Forward(8,8);break;			  //FL2_L/FL_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,8);
					break;

//				case 0xA5:	  Move_Forward(9,9);break;			  //FL2_L/FL_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,9);
					break;

//				case 0X0AD:   Move_Forward(9,8);break;			   //FL2_L/FL_L/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(9,8);
					break;

//				case 0x08D:   Move_Forward(10,8);break;			   //FL2_L/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(10,8);
					break;

//				case 0x089:   Move_Forward(9,7);break;			   //FL2_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,7);
					break;

//				case 0x02C:   Move_Forward(8,6);break;			   //FL_L/FR_L/FR_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break;

//				case 0x00D:   Move_Forward(9,6);break;			   //FR_L/FR_R/FR2_R
				case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,6);
					break;

//				case 0x00C:   Move_Forward(8,6);break;			   //FR_L/FR_R
				case (RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break;

//				case 0x008:   Move_Forward(7,4);break;			   //FR_L
				case (RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(7,4);
					break;

//				case 0x00F:   Move_Forward(7,3);break;			   //FR_L/FR_R/FR2_L/FR2_R
				case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,3);
					break;

//				case 0x009:   Move_Forward(9,5);break;			   //FR_L/FR2_R
				case (RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,5);
					break;

//				case 0x00B:   Move_Forward(8,3);break;			   //FR_L/FR2_L/FR2_R
				case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,3);
					break;

//				case 0x001:   Move_Forward(9,3);break;			   //FR2_R
				case (RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,3);
					break;

//				case 0x002:
				case (RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x003:   Turn_Right(20,350);				  //FR2_L/FR2_R
				case (RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,350);
					Stop_Brifly();
					Move_Forward(10,3);
					break;

//				case 0x28:	  Move_Forward(8,7);break;				//FL_L/FR_L
				case (RconFL_HomeL|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0x2B:	  Move_Forward(8,6);break;				//FL_L/FR_L/FR2_L/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break;

//				case 0x29:	  Move_Forward(9,7);break;				//FL_L/FR_L/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,7);
					break;

//				case 0X0A:	  Move_Forward(9,4);break;				//FR_L/FR2_L
				case (RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
					Move_Forward(9,4);
					break;

//				case 0X54:	  Move_Forward(8,7);break;				//FL_L/FR_L/FR2_L
				case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0X2D:	  Move_Forward(8,6);break;				//FL_L/FR_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break;

//				case 0X05:	  Move_Forward(9,7);break;				//FR_R/FR2_R
				case (RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,7);
					break;

//				case 0x04:	  Move_Forward(8,7);break;				//
				case (RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0x45:	  Move_Forward(8,7);break;				//FL2_R/FR_R/FR2_R
				case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0xc5:	  Move_Forward(8,6);break;				//FL2_L/FL2_R/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break;

//				case 0xCD:	  Move_Forward(8,3);break;				  //FL2_L/FL2_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,3);
					break;

//				case 0xcf:	  Move_Forward(8,3);break;				//FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,3);
					break;

//				case 0xA9:	  Move_Forward(8,7);break;			   //FL2_L/FL_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0x85:	  Move_Forward(9,8);break;			   //
				case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,8);
					break;

//				case 0x8f:	  Move_Forward(8,7);break;			   //FL2_L/FL_L/FR2_R//FL2_L/FR_L/FR_R/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0x84:	  Move_Forward(9,8);break;			   //
				case (RconFL2_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,8);
					break;

//				case 0x31:	  Move_Forward(9,8);break;			   //FL_L/FL_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,8);
					break;

//				case 0x9D:	  Move_Forward(8,7);break;			   //FL2_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0xAC:	  Move_Forward(8,7);break;			   //FL2_L/FL_L/FR_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0xbc:	  Move_Forward(8,7);break;			   //FL2_L/FL_L/FL_R/FR_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0x25:	  Move_Forward(8,7);break;			   //FL_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0xb8:	  Move_Forward(9,8);break;			   //FL2_L/FL_L/FL_R/FR_L
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(9,8);
					break;

//				case 0xA8:	  Move_Forward(9,6);break;			   //FL2_L/FL_L/FR_l
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(9,6);
					break;

//				case 0x803:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x802:   Turn_Right(20,400);
				case (RconR_HomeR|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,400);
					Stop_Brifly();
					Move_Forward(8,3);
					break;

//				case 0x80b:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					Stop_Brifly();
					Move_Forward(10,4);
					break;

//				case 0xC02:	  Turn_Right(20,450);
				case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,450);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0xC00 :  Turn_Right(20,500);
				case (RconR_HomeR|RconR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/R_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					Stop_Brifly();
					Move_Forward(9,6);
					break;

//				case 0x400 :  Turn_Right(20,550);
				case (RconR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,550);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x80A:   Turn_Right(20,250);
				case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,250);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x402:   Turn_Right(20,500);
				case (RconR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_L/FR2_L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					Stop_Brifly();
					Move_Forward(9,3);
					break;

//				case 0x805:
				case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,2);
					break;

//				case 0x801:   Move_Forward(9,0);
				case (RconR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,0);
					break;

//				case 0x101:   Move_Forward(9,1);
				case (RconL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,1);
					break;

//				case 0x185:   Move_Forward(9,6);
				case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,6);
					break;

//				case 0x800:   Move_Forward(10,0);
				case (RconR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R.", __FUNCTION__, __LINE__);
					Move_Forward(10,0);
					break;

//				case 0x8B:	  Move_Forward(8,7);break;			  //FL2_L/FR_L/FR2_L/FR2_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,7);
					break;

//				case 0x22:	  Move_Forward(9,8);break;			  //FL2_R/FR_R
				case (RconFL_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
					Move_Forward(9,8);
					break;

//				case 0x81:	  Move_Forward(9,2);break;
				case (RconFL2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,2);
					break;

//				case 0x23:	  Move_Forward(9,8);break;
				case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(9,8);
					break;

//				case 0Xb5:	  Move_Forward(8,9);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,9);
					break;

//				case 0xb1:	  Move_Forward(8,10);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,10);
					break;

//				case 0x91:	  Move_Forward(7,9);break;
				case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,9);
					break;

//				case 0x34:	  Move_Forward(6,8);break;
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break;

//				case 0xb0:	  Move_Forward(6,9);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,9);
					break;

//				case 0x30:	  Move_Forward(6,8);break;
				case (RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break;

//				case 0x10:	  Move_Forward(4,7);break;
				case (RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(4,7);
					break;

//				case 0xf0:	  Move_Forward(3,7);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(3,7);
					break;

//				case 0x90:	  Move_Forward(5,9);break;
				case (RconFL2_HomeL|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(5,9);
					break;

//				case 0xD0:	  Move_Forward(3,8);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(3,8);
					break;

//				case 0x80:	  Move_Forward(3,9);break;
				case (RconFL2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L.", __FUNCTION__, __LINE__);
					Move_Forward(3,9);
					break;

//				case 0x40:	  Turn_Left(20,250);
				case (RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,250);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0xC0:	  Turn_Left(20,350);
				case (RconFL2_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,350);
					Stop_Brifly();
					Move_Forward(3,10);
					break;

//				case 0x14:	  Move_Forward(7,8);break;
				case (RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0xD4:	  Move_Forward(6,8);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break;

//				case 0x94:	  Move_Forward(7,9);break;
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,9);
					break;

//				case 0X50:	  Move_Forward(4,9);break;
				case (RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
					Move_Forward(4,9);
					break;

//				case 0X2A:	  Move_Forward(7,8);break;
				case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0XB4:	  Move_Forward(6,8);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break;

//				case 0XA0:	  Move_Forward(7,9);break;
				case (RconFL2_HomeL|RconFL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
					Move_Forward(7,9);
					break;

//				case 0x20:	  Move_Forward(7,8);break;
				case (RconFL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0xA2:	  Move_Forward(7,8);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0xA3:	  Move_Forward(6,8);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break;

//				case 0xB3:	  Move_Forward(3,8);break;
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(3,8);
					break;

//				case 0xF3:	  Move_Forward(3,8);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(3,8);
					break;

//				case 0x95:	  Move_Forward(7,8);break;//FL2_L/FL_L/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0xA1:	  Move_Forward(8,9);break;			  //FL2_L/FL_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,9);
					break;

//				case 0xF1:	  Move_Forward(7,8);break;			  //FL2_L/FL2_R/FL_L/FL_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0x21:	  Move_Forward(8,9);break;			  //FL2_L/FL_L/FR2_R //FL2_L/FR_R
				case (RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,9);
					break;

//				case 0x8c:	  Move_Forward(8,9);break;			  //FL2_L/FL_L/FR2_R //FL2_L/FR_R
				case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,9);
					break;

//				case 0xb9:	  Move_Forward(7,8);break;			  //FL2_L/FL_R/FL_R/FR_L/FR2_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0x35:	  Move_Forward(7,8);break;			  //FL_L/FL_R/FR_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0x3d:	  Move_Forward(7,8);break;			  //FL_L/FL_R/FR_L/FR_R/FR2_R
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0xa4:	  Move_Forward(7,8);break;			  //FL2_L/FL_L/FR_R
				case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0x1d:	  Move_Forward(8,9);break;			  //FL_R/FR_L/FR_R/FR2_R
				case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,9);
					break;

				case 0x15:	  Move_Forward(6,9);break;			  //FL_R/FR_R/FR2_R
				case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,9);
					break;

//				case 0x1C0:   Turn_Left(20,250);
				case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,250);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x140:   Turn_Left(20,400);
				case (RconL_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,400);
					Stop_Brifly();
					Move_Forward(3,8);
					break;

//				case 0x1d0:   Turn_Left(20,250);
				case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,250);
					Stop_Brifly();
					Move_Forward(4,10);
					break;

//				case 0x340:   Turn_Left(20,450);
				case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,450);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x300 :  Turn_Left(20,500);
				case (RconL_HomeR|RconL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, L_R/L_L.", __FUNCTION__, __LINE__);
					Turn_Left(20,500);
					Stop_Brifly();
					Move_Forward(6,9);
					break;

//				case 0x200:   Turn_Left(20,550);
				case (RconL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,550);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x150:   Turn_Left(20,250);
				case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,250);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x240:   Turn_Left(20,500);
				case (RconL_HomeR|RconFL2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, L_R/FL2_R.", __FUNCTION__, __LINE__);
					Turn_Left(20,500);
					Stop_Brifly();
					Move_Forward(3,9);
					break;

//				case 0x1A0:
				case (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L/FL_L.", __FUNCTION__, __LINE__);
					Move_Forward(2,9);
					break;

//				case 0x180:   Move_Forward(0,9);
				case (RconL_HomeL|RconFL2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, L_L/FL2_L.", __FUNCTION__, __LINE__);
					Move_Forward(0,9);
					break;

//				case 0x880:   Move_Forward(1,9);
				case (RconR_HomeR|RconFL2_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FL2_L.", __FUNCTION__, __LINE__);
					Move_Forward(1,9);
					break;

//				case 0x8A1:   Move_Forward(6,9);
				case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(6,9);
					break;

//				case 0x100:   Move_Forward(0,10);break;
				case (RconL_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, L_L.", __FUNCTION__, __LINE__);
					Move_Forward(0,10);
					break;

//				case 0xD1:	  Move_Forward(7,8);break;			  //FL_R/FR_R/FR2_R
				case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
					Move_Forward(7,8);
					break;

//				case 0x44:	  Move_Forward(8,9);break;			  //FL_L/FR2_L
				case (RconFL2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,9);
					break;

//				case 0x18:	  Move_Forward(2,9);break;
				case (RconFL_HomeR|RconFR_HomeL):
					ROS_DEBUG("%s, %d: !Position_Far, FL_R/FR_L.", __FUNCTION__, __LINE__);
					Move_Forward(2,9);
					break;

//				case 0xC4:	  Move_Forward(8,9);break;
				case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
					ROS_DEBUG("%s, %d: !Position_Far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(8,9);
					break;

				default:
//					USPRINTF("**************default angle is %d, default code  is 0x%x\n",angle,Temp_Code);
					ROS_DEBUG("%s, %d: !Position_Far, else:%x.", __FUNCTION__, __LINE__, Temp_Code);
					Move_Forward(7,8);
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
	uint8_t Motor_Check_Code=Check_Motor_Current();
	if(Motor_Check_Code)
	{
		if(Self_Check(Motor_Check_Code))
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return 1;
		}
		else
		{
			Home_Motor_Set();
			Set_Clean_Mode(Clean_Mode_GoHome);
			return 1;
		}
	}
	return 0;
}

/*-------------------Turn OFF the Vaccum-----------------------------*/
void Home_Motor_Set(void)
{
	Set_BLDC_Speed(0);
	Set_MainBrush_PWM(20);
	Set_SideBrush_PWM(20,20);
	Move_Forward(20,20);
	//Reset_WheelSLow();
	//Reset_Bumper_Error();

}

