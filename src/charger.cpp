#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>

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

	uint8_t Display_Full_Switch=0;

	#ifdef ONE_KEY_DISPLAY

	uint8_t One_Display_Counter=0;

	#endif

	// This counter is for debug message.
	uint8_t Show_Batv_Counter=0;

#if CONTINUE_CLEANING_AFTER_CHARGE
	// This counter is for checking if battery enough to continue cleaning.
	uint16_t Bat_Enough_To_Continue_Cleaning_Counter = 0;
#endif

	// Reset the lowBattery flag in core_move.cpp and stop beeping.
	Beep(0, 0, 0, 1);
	lowBattery = 0;

	set_start_charge();
	uint16_t bat_v;
	ROS_INFO("[gotocharger.cpp] Start charger mode.");
	while(ros::ok())
	{
		usleep(20000);
		bat_v = robot::instance()->robot_get_battery_voltage();

#if CONTINUE_CLEANING_AFTER_CHARGE
		if (robot::instance()->Is_Cleaning_Paused())
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
#endif
		ROS_DEBUG_NAMED("charger"," Loop for charger mode,voltage %f.",bat_v/100.0);
		if(Show_Batv_Counter > 250)
		{
			ROS_INFO(" Loop for charger mode,voltage %f.",bat_v/100.0);
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

		if(!Is_ChargerOn())//check if charger unplug
		{
#if CONTINUE_CLEANING_AFTER_CHARGE
			if (robot::instance()->Is_Cleaning_Paused())
			{
				ROS_INFO("[gotocharger.cpp] Exit charger mode and continue cleaning.");
				Set_Clean_Mode(Clean_Mode_Navigation);
				break;
			}
#endif
			ROS_INFO("[gotocharger.cpp] Exit charger mode and go to userinterface mode.");
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		/*----------------------------------------------------Check Key---------------------*/
		if(Get_Key_Press() == KEY_CLEAN)//                                    Check Key Clean
		{
			Get_Key_Time(KEY_CLEAN);
//			Reset_Error_Code();
			if(Is_AtHomeBase()) {
				ROS_INFO("[gotocharger.cpp] Exit charger mode and go to navigation mode.");
//				Set_Room_Mode(Room_Mode_Large);
				Set_Clean_Mode(Clean_Mode_Navigation);
				break;
			}
		}
		/*if(Remote_Key(Remote_Random))//                                       Check Remote Key Clean
		{
			set_stop_charge();
			Reset_Rcon_Remote();
			if(Is_AtHomeBase())
			{
				Set_VacMode(Vac_Normal);
//				Set_Room_Mode(Room_Mode_Large);
//				Press_time=10;
//				while(Press_time--)
//				{
//					if(Get_Rcon_Remote()==Remote_Wallfollow)
//					{
//						Set_LED(100,0);
//						Beep(1);
//						Set_LED(0,0);
//						Beep(3);
//						Set_LED(100,0);
//						Beep(5);
//						Set_Room_Mode(Room_Mode_Auto);
//						break;
//					}
//					delay(500);
//				}
				Set_Clean_Mode(Clean_Mode_RandomMode);
				break;
			}
		}*/
		if (Remote_Key(Remote_Clean)) {
			set_stop_charge();
			Reset_Rcon_Remote();
			if(Is_AtHomeBase()) {
//				Set_VacMode(Vac_Normal);
//				Set_Room_Mode(Room_Mode_Large);
				Set_Clean_Mode(Clean_Mode_Navigation);
				break;
			}
		}
		/*-----------------------------------------------------Schedul Timer Up-----------------*/
//		if(Is_Alarm())
//		{
//			Reset_Alarm();
//      		if(Is_AtHomeBase())
//			{
//				Set_VacMode(Vac_Normal);
//				Set_Room_Mode(Room_Mode_Large);
//				Set_Clean_Mode(Clean_Mode_Navigation);
//				break;
//			}
//		}

		#ifdef ONE_KEY_DISPLAY
		if(robot::instance()->robot_get_battery_voltage())
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

			if(Display_Full_Switch)
			{
				Set_LED(100,0);
			}
			else
			{
				Set_LED(One_Display_Counter,One_Display_Counter);
			}
		}
		#endif

  }
}

/*----------------------------------------------------------------GO Home  ----------------*/
void GoHome(void)
{
	uint32_t Receive_Code=0;
//	Move_Forward(9,9);
//  Set_SideBrush_PWM(30,30);
//  Set_MainBrush_PWM(30);
//	Display_Home_LED();
	Reset_Rcon_Status();
	//delay(1500);

	// This is for calculating the robot turning.
	uint16_t Current_Angle;
	uint16_t Last_Angle;
	int Angle_Offset;
	// This step is for counting angle change when the robot turns.
	long Gyro_Step = 0;

	// Stop all the motors to keep the robot at peace, so that it can successfully open the gyrp.
	Disable_Motors();
	set_gyro(1, 0);
	ROS_INFO("GoHome function opening gyro.");
	while(robot::instance()->robot_get_angle_v() == 0)
	{
		usleep(100000);
	}
	ROS_INFO("GoHome function open gyro succeeded.");
	Set_SideBrush_PWM(30,30);
	Set_MainBrush_PWM(30);

	Stop_Brifly();

	// Save the start angle.
	Last_Angle = Gyro_GetAngle(0);

	while(Gyro_Step < 3600)
	{
		Receive_Code = Get_Rcon_Status();
		Reset_Rcon_Status();
		if(Receive_Code&RconFR_HomeT)//fr ht
		{
			ROS_INFO("Start with FR-T.");
			//Turn_Left(Turn_Speed,1000);
			Turn_Left(Turn_Speed,300);
			Stop_Brifly();
			Around_ChargerStation(0);
			return;
		}
		if(Receive_Code&RconFL_HomeT)//fl ht
		{
			ROS_INFO("Start with FL-T.");
			//Turn_Right(Turn_Speed,1000);
			Turn_Right(Turn_Speed,300);
			Stop_Brifly();
			Around_ChargerStation(1);
			return;
		}
		if(Receive_Code&RconL_HomeT)//l t
		{
			ROS_INFO("Start with L-T.");
			Turn_Right(Turn_Speed,1200);
			Stop_Brifly();
			Around_ChargerStation(1);
			return;
		}
		if(Receive_Code&RconR_HomeT)//r t
		{
			ROS_INFO("Start with R-T.");
			Turn_Left(Turn_Speed,1200);
			Stop_Brifly();
			Around_ChargerStation(0);
			return;
		}
		if(Receive_Code&RconR_HomeR)//r t
		{
			ROS_INFO("Start with R-R.");
			//Turn_Right(Turn_Speed,1200);
			//Stop_Brifly();
			Around_ChargerStation(0);
			return;
		}
		if(Receive_Code&RconL_HomeL)//r t
		{
			ROS_INFO("Start with L-L.");
			//Turn_Right(Turn_Speed,1200);
			//Stop_Brifly();
			Around_ChargerStation(1);
			return;
		}
		/*-----------------------------------------*/
		if(Receive_Code&RconFR_HomeL)//r t
		{
			ROS_INFO("Start with FR-L.");
			Turn_Right(Turn_Speed,900);
			Stop_Brifly();
			Around_ChargerStation(1);
			return;
		}
		if(Receive_Code&RconFL_HomeR)//r t
		{
			ROS_INFO("Start with FL-R.");
			Turn_Left(Turn_Speed,900);
			Stop_Brifly();
			Around_ChargerStation(0);
			return;
		}
		/*-----------------------------------------*/
		if(Receive_Code&RconFL_HomeL)//r t
		{
			ROS_INFO("Start with FL-L.");
			Turn_Right(Turn_Speed,900);
			Stop_Brifly();
			Around_ChargerStation(1);
			return;
		}
		if(Receive_Code&RconFR_HomeR)//r t
		{
			ROS_INFO("Start with FR-R.");
			Turn_Left(Turn_Speed,900);
			Stop_Brifly();
			Around_ChargerStation(0);
			return;
		}

		//if((Receive_Code&0x000000ff))// infront of : straight
		if((Receive_Code&(RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR)))// infront of : straight
		{
			//Receive_Code&=0x000000ff;
			Receive_Code&=(RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR);
			ROS_INFO("Start switch.");
			switch(Receive_Code)
			{
				//case 0x00000008:Turn_Right(20,900);break;//		if(Receive_Code==RconFR_HomeL)
				case RconFR_HomeL:
					Turn_Right(20,900);
					break;//		if(Receive_Code==RconFR_HomeL)
			    //case 0x00000010:Turn_Left(20,900);break;//		if(Receive_Code==RconFL_HomeR)
			    case RconFL_HomeR:
					Turn_Left(20,900);
					break;//		if(Receive_Code==RconFL_HomeR)
				//case 0x0000000c:Turn_Right(20,400);break;//		if(Receive_Code==(RconFR_HomeR|RconFR_HomeL))
				case (RconFR_HomeL|RconFR_HomeR):
					Turn_Right(20,400);
					break;//		if(Receive_Code==(RconFR_HomeR|RconFR_HomeL))
				//case 0x00000030:Turn_Left(20,400);break;//		if(Receive_Code==(RconFL_HomeR|RconFL_HomeL))
				case (RconFL_HomeL|RconFL_HomeR):
					Turn_Left(20,400);
					break;//		if(Receive_Code==(RconFL_HomeR|RconFL_HomeL))
				//case 0x00000014:Turn_Left(20,700);break;//		if(Receive_Code==(RconFL_HomeR|RconFR_HomeR))
				case (RconFL_HomeR|RconFR_HomeR):
					Turn_Left(20,700);
					break;//		if(Receive_Code==(RconFL_HomeR|RconFR_HomeR))
				//case 0x00000028:Turn_Right(20,700);break;//		if(Receive_Code==(RconFL_HomeL|RconFR_HomeL))
				case (RconFL_HomeL|RconFR_HomeL):
					Turn_Right(20,700);
					break;//		if(Receive_Code==(RconFL_HomeL|RconFR_HomeL))
				//case 0x00000020:Turn_Right(20,700);break;//		if(Receive_Code==RconFL_HomeL)
				case RconFL_HomeL:
					Turn_Right(20,700);
					break;//		if(Receive_Code==RconFL_HomeL)
				//case 0x00000004:Turn_Left(20,700);break;//		if(Receive_Code==RconFR_HomeR)
				case RconFR_HomeR:
					Turn_Left(20,700);
					break;//		if(Receive_Code==RconFR_HomeR)
				//case 0x0000002c:Turn_Right(20,700);break;//		if(Receive_Code==(RconFR_HomeL|RconFL_HomeL|RconFR_HomeR))
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					Turn_Right(20,700);
					break;//		if(Receive_Code==(RconFR_HomeL|RconFL_HomeL|RconFR_HomeR))
				//case 0x00000034:Turn_Left(20,700);break;//		if(Receive_Code==(RconFR_HomeR|RconFL_HomeR|RconFL_HomeL))
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					Turn_Left(20,700);
					break;//		if(Receive_Code==(RconFR_HomeR|RconFL_HomeR|RconFL_HomeL))
				//case 0x00000001:Turn_Left(20,700);break;//		if(Receive_Code==(RconFR_HomeR|RconFL_HomeR|RconFL_HomeL))
				//case 0x00000002:Turn_Right(20,1300);break;//		if(Receive_Code==(RconR_HomeL))
				case RconR_HomeL:
					Turn_Right(20,1300);
					break;//		if(Receive_Code==(RconR_HomeL))
				//case 0x00000040:Turn_Left(20,1300);break;//		if(Receive_Code==(RconL_HomeR))
				case RconL_HomeR:
					Turn_Left(20,1300);
					break;//		if(Receive_Code==(RconL_HomeR))
				//case 0x00000080:Turn_Left(20,700);break;//		if(Receive_Code==(RconFR_HomeR|RconFL_HomeR|RconFL_HomeL))
				default:
					Turn_Right(20,900);
					break;
			}
			Stop_Brifly();
			//Move_Forward(10,10);
			By_Path();
			return;
		}
		//if((Receive_Code&0x3000))//br hlr
		if(Receive_Code&(RconBR_HomeL|RconBR_HomeR))//br hlr
		{
			ROS_INFO("Start with BR-LR.");
			Turn_Right(30,900);
			Stop_Brifly();
			By_Path();
			return;
		}
		//if((Receive_Code&0x30000))//bl hlr
		if(Receive_Code&(RconBL_HomeL|RconBL_HomeR))//bl hlr
		{
			ROS_INFO("Start with BL-LR.");
			Turn_Left(30,900);
			Stop_Brifly();
			By_Path();
			return;
		}
		if((Receive_Code&RconBL_HomeT))//bl hlr
		{
			ROS_INFO("Start with BL-T.");
			Turn_Left(30,1300);
			Stop_Brifly();
			Around_ChargerStation(0);
			//By_Path();
			return;
		}
		if((Receive_Code&RconBR_HomeT))//bl hlr
		{
			ROS_INFO("Start with BR-T.");
			Turn_Right(30,1300);
			Stop_Brifly();
			Around_ChargerStation(1);
			//By_Path();
			return;
		}
		usleep(50000);
		Current_Angle = Gyro_GetAngle(0);
		Angle_Offset = Current_Angle - Last_Angle;
		//ROS_INFO("Current_Angle = %d, Last_Angle = %d, Angle_Offset = %d, Gyro_Step = %ld.", Current_Angle, Last_Angle, Angle_Offset, Gyro_Step);
		if (Angle_Offset > 0)
		{
			Angle_Offset -= 3600;
		}
		Gyro_Step += abs(Angle_Offset);
		Last_Angle = Current_Angle;

		Set_Dir_Right();
		Set_Wheel_Speed(10, 10);
	}
	// If robot didn't reach the charger, go back to userinterface mode.
	if(Get_Clean_Mode() != Clean_Mode_Charging)
	{
		Set_Clean_Mode(Clean_Mode_Userinterface);
	}
}

void Around_ChargerStation(uint8_t Dir)
{
// Dir == 0 means robot is at the right side of the charger stub.
// Dir == 1 means robot is at the left side of the charger stub.
//  uint32_t Temp_Steps=0;
	uint8_t Temp_Position=0;
	//uint8_t Mobility_Temp_Error=0;
	//uint32_t Temp_Mobility_Distance=0;
	uint32_t Temp_Rcon_Status=0;
	uint8_t Signal_Counter=0;
	uint32_t No_Signal_Counter=0;
	uint32_t N_Around_LRSignal=0;
	uint8_t Bumper_Counter=0;
	Move_Forward(9,9);
	Set_SideBrush_PWM(30,30);
	Set_MainBrush_PWM(30);
	Set_BLDC_Speed(Vac_Speed_NormalL);
	Reset_Rcon_Status();
//	Display_Home_LED();
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
//			  Temp_Mobility_Distance = Get_Move_Distance();
//			  if(Get_Mobility_Step()<1)
//			  {
//			    Mobility_Temp_Error++;
//			    if(Mobility_Temp_Error>3)
//			    {
//			      Set_Clean_Mode(Clean_Mode_GoHome);
//			      return;
//			    }
//			  }
//			  else
//			  {
//			    Mobility_Temp_Error=0;
//			  }
//			  Reset_Mobility_Step();
//			}
//		}

		if(Get_Cliff_Trig())
		{
			while (Get_Cliff_Trig())
			{
				// Move back until escape cliff triggered.
				Move_Back();
			}
			Turn_Left(Turn_Speed,1750);
			Move_Forward(9,9);
			Set_Clean_Mode(Clean_Mode_GoHome);
			return;
		}

		/*------------------------------------------------------Touch and Remote event-----------------------*/
		if(Touch_Detect() || Remote_Key(Remote_Clean))
		{
			//Reset_Touch();
			// If key pressed, go back to user interface mode.
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
		/*------------------------------------------------------Low battery event-----------------------*/
		if(GetBatteryVoltage()<Low_Battery_Limit)
		{
			Display_Battery_Status(Display_Low);
			//delay(10000);
			usleep(1000000);
			Set_Clean_Mode(Clean_Mode_Sleep);
			return;
		}

//		if(Home_Check_Current())return;

		/*------------------------------------------------------Bumper event-----------------------*/
		if(Get_Bumper_Status())
		{
			Bumper_Counter++;
			Random_Back();
			if(Get_Bumper_Status())
			{
				Random_Back();
				if(Get_Bumper_Status())
				{
					// Bumper jamed.
					while (Get_Bumper_Status()){
						// Sleep for 2s and detect again, and beep to alarm in the first 0.5s
						Beep(3, 25, 0, 1);
						usleep(2000000);
					}
					return;
				}
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
				Beep(1, 25, 75, 3);
				ROS_INFO("No charger signal received.");
				Set_Clean_Mode(Clean_Mode_Userinterface);
				//Set_Clean_Mode(Clean_Mode_GoHome);
				return;
			}
			//Display_TM1618(No_Signal_Counter,0);
		}
		/*
		if(Temp_Rcon_Status&0x4000)
		{
		  Turn_Right(30,2200);
		  Move_Forward(10,10);
		  Dir=1-Dir;
		}*/

		ROS_INFO("%s %d Check DIR: %d, and do something", __FUNCTION__, __LINE__, Dir);
		if(Dir == 1)//10.30
		{
			//if(Get_RightWheel_Step()>20000)
			//{
			//	Stop_Brifly();
			//	Turn_Right(Turn_Speed,2200);
			//	Set_Clean_Mode(Clean_Mode_GoHome);
			//	return ;
			//}

			// If still detects HomeT or HomeL, keep rounding to adjust the pose.
			if(Temp_Rcon_Status&RconL_HomeT)
			{
				ROS_INFO("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
				Move_Forward(30,4);
				//delay(1000);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconL_HomeL)
			{
				ROS_INFO("%s, %d: Detect L-L.", __FUNCTION__, __LINE__);
				Move_Forward(16,12);
				//delay(1000);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFL_HomeL)
			{
				ROS_INFO("%s, %d: Detect FL-L.", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				Turn_Right(Turn_Speed,500);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFL_HomeT)
			{
				ROS_INFO("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				Turn_Right(Turn_Speed,500);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFR_HomeT)
			{
				ROS_INFO("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				Turn_Right(Turn_Speed,800);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconR_HomeT)
			{
				ROS_INFO("%s, %d: Detect R-T.", __FUNCTION__, __LINE__);
				//Stop_Brifly();
				Turn_Right(Turn_Speed,1000);
				Move_Forward(5,5);
			}
			else
			{
				ROS_INFO("%s, %d: Else.", __FUNCTION__, __LINE__);
				Move_Forward(6,30);
			}

			// Once detects HomeR, robot should be at the right pose infront of the charge stub.
			if(Temp_Rcon_Status&(RconFR_HomeR))
			{
				ROS_INFO("%s, %d: Detect FR-R, call By_Path().", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				//Turn_Left(Turn_Speed,300);
				By_Path();
				return;
			}
			if(Temp_Rcon_Status&(RconFL_HomeR))
			{
				ROS_INFO("%s %d Detect FL-R, call By_Path()", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				Turn_Left(Turn_Speed,300);
				By_Path();
				return;
			}
			//if(Temp_Rcon_Status&(RconL_HomeR))
			if(Temp_Rcon_Status&(RconBL_HomeR))
			{
				// Maybe over go a little bit, turn around and check if at the front of charge stub.
				//ROS_INFO("%s %d Detect L-R.", __FUNCTION__, __LINE__);
				ROS_INFO("%s %d Detect BL-R.", __FUNCTION__, __LINE__);
				Signal_Counter++;
				N_Around_LRSignal=0;
				if(Signal_Counter>0)
				{
					ROS_INFO("%s %d Signal_Counter>0, check position.", __FUNCTION__, __LINE__);
					Signal_Counter=0;
					Stop_Brifly();
					Temp_Position = Check_Position(Round_Left);
					ROS_INFO("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Position);
					if(Temp_Position==1)
					{
						//Reset_Error_Code();
						//Reset_Touch();
						ROS_INFO("%s %d return to Clean_Mode_Userinterface", __FUNCTION__, __LINE__);
						Set_Clean_Mode(Clean_Mode_Userinterface);
						return;
					}
					if(Temp_Position==2)
					{
						ROS_INFO("%s %d call By_Path()", __FUNCTION__, __LINE__);
						Move_Forward(1,1);
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
			//if(Get_LeftWheel_Step()>20000)
			//{
			//	Stop_Brifly();
			//	Turn_Left(Turn_Speed,2200);
			//	Set_Clean_Mode(Clean_Mode_GoHome);
			//	ROS_INFO("%s %d return to Clean_Mode_GoHome", __FUNCTION__, __LINE__);
			//	return ;
			//}
			if(Temp_Rcon_Status&RconR_HomeT)
			{
				ROS_INFO("%s %d Detect R-T.", __FUNCTION__, __LINE__);
				Move_Forward(4,30);
				//delay(1000);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconR_HomeR)
			{
				ROS_INFO("%s %d Detect R-R.", __FUNCTION__, __LINE__);
				Move_Forward(12,16);
				//delay(1000);
				usleep(100000);
			}
			else if(Temp_Rcon_Status&RconFR_HomeR)
			{
				ROS_INFO("%s %d Detect FR-R.", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				Turn_Left(Turn_Speed,500);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFR_HomeT)
			{
				ROS_INFO("%s %d Detect FR-T.", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				Turn_Left(Turn_Speed,500);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconFL_HomeT)
			{
				ROS_INFO("%s %d Detect FL-T.", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				Turn_Left(Turn_Speed,800);
				Move_Forward(5,5);
			}
			else if(Temp_Rcon_Status&RconL_HomeT)
			{
				ROS_INFO("%s %d Detect L-T.", __FUNCTION__, __LINE__);
				//Stop_Brifly();
				Turn_Left(Turn_Speed,1000);
				Move_Forward(5,5);
				Dir=1;
			}
			else
			{
				ROS_INFO("%s %d Else.", __FUNCTION__, __LINE__);
				Move_Forward(30,6);
			}

			if(Temp_Rcon_Status&(RconFL_HomeL))
			{
				ROS_INFO("%s %d Detect FL-L, call By_Path()", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				//Turn_Right(30,500);
				By_Path();
				return;
			}
			if(Temp_Rcon_Status&(RconFR_HomeL))
			{
				ROS_INFO("%s %d Detect FR-L, call By_Path()", __FUNCTION__, __LINE__);
				////Stop_Brifly();
				Turn_Right(Turn_Speed,300);
				By_Path();
				return;
			}
			//if((Temp_Rcon_Status&(RconR_HomeL)))
			if((Temp_Rcon_Status&(RconBR_HomeL)))
			{
				ROS_INFO("%s %d Detect BR-L.", __FUNCTION__, __LINE__);
				Signal_Counter++;
				N_Around_LRSignal=0;
				if(Signal_Counter>0)
				{
					ROS_INFO("%s %d Signal_Counter>0, check position.", __FUNCTION__, __LINE__);
					Signal_Counter=0;
					Stop_Brifly();
					Temp_Position = Check_Position(Round_Right);
					ROS_INFO("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Position);
					if(Temp_Position==1)
					{
						//Reset_Error_Code();
						Reset_Touch();
						Set_Clean_Mode(Clean_Mode_Userinterface);
						ROS_INFO("%s %d return to Clean_Mode_Userinterface", __FUNCTION__, __LINE__);
						return;
					}
					if(Temp_Position==2)
					{
						Move_Forward(1,1);
						ROS_INFO("%s %d call By_Path()", __FUNCTION__, __LINE__);
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

		//delay(500);
		usleep(50000);
	}
}

/*------------------------------------------------*/
uint8_t Check_Position(uint8_t Dir)
{
	//uint32_t Counter_Watcher=0;
	uint32_t Receive_Code = 0;
	// This angle is for counting angle change when the robot turns.
	uint16_t Current_Angle;
	uint16_t Last_Angle;
	int Angle_Offset;
	// This step is for counting angle change when the robot turns.
	long Gyro_Step = 0;

	if(Dir == Round_Left)
	{
		ROS_INFO("Dir = left");
		Set_Dir_Left();
	}
	else if(Dir == Round_Right)
	{
		ROS_INFO("Dir = right");
		Set_Dir_Right();
	}
	//Set_Wheel_Speed(17,17);
	Set_Wheel_Speed(10,10);
	//Reset_LeftWheel_Step();
	//Reset_TempPWM();
	//Counter_Watcher=0;
	//Reset_Touch();
	// Save the start angle
	Last_Angle = Gyro_GetAngle(0);
	ROS_INFO("Last_Angle = %d.", Last_Angle);

	//while(Get_LeftWheel_Step()<3600)
	while(Gyro_Step < 3600)
	//while(Gyro_Step < 36000)
	{
		//delay(1);
		usleep(50000);
		Current_Angle = Gyro_GetAngle(0);
		Angle_Offset = Current_Angle - Last_Angle;
		ROS_INFO("Current_Angle = %d, Last_Angle = %d, Angle_Offset = %d, Gyro_Step = %ld.", Current_Angle, Last_Angle, Angle_Offset, Gyro_Step);
		if (Angle_Offset < 0 && Dir == Round_Left)
		{
			Angle_Offset += 3600;
		}
		if (Angle_Offset > 0 && Dir == Round_Right)
		{
			Angle_Offset -= 3600;
		}
		Gyro_Step += abs(Angle_Offset);
		Last_Angle = Current_Angle;
		//Counter_Watcher++;
		//if(Counter_Watcher>150000)
		//{
		//	if(Is_Encoder_Fail())
		//	{
		//	  Set_Error_Code(Error_Code_Encoder);
		//	  Set_Touch();
		//	}
		//	return 1;
		//}
		Receive_Code = (Get_Rcon_Status()&(RconL_HomeL|RconL_HomeR|RconFL_HomeL|RconFL_HomeR|RconR_HomeL|RconR_HomeR|RconFR_HomeL|RconFR_HomeR));
		ROS_INFO("Check_Position Get_Rcon_Status() == %x, R... == %x, receive code: %x.", Get_Rcon_Status(), (RconL_HomeL|RconL_HomeR|RconFL_HomeL|RconFL_HomeR|RconR_HomeL|RconR_HomeR|RconFR_HomeL|RconFR_HomeR), Receive_Code);
		if(Receive_Code)
		{
			Reset_Rcon_Status();
			if (Receive_Code & RconL_HomeL)ROS_INFO("Check_Position get L-L");
			if (Receive_Code & RconL_HomeR)ROS_INFO("Check_Position get L-R");
			if (Receive_Code & RconFL_HomeL)ROS_INFO("Check_Position get FL-L");
			if (Receive_Code & RconFL_HomeR)ROS_INFO("Check_Position get FL-R");
			if (Receive_Code & RconR_HomeL)ROS_INFO("Check_Position get R-L");
			if (Receive_Code & RconR_HomeR)ROS_INFO("Check_Position get R-R");
			if (Receive_Code & RconFR_HomeL)ROS_INFO("Check_Position get FR-L");
			if (Receive_Code & RconFR_HomeR)ROS_INFO("Check_Position get FR-R");
		}
		if(Dir == Round_Left)
		{
			if(Receive_Code & (RconFR_HomeL|RconFR_HomeR))
			{
				// Robot has turned back heading towards charge stub.
				ROS_INFO("Check position left and return 2.");
				return 2;
			}
		}
		if(Dir == Round_Right)
		{
			if(Receive_Code & (RconFL_HomeL|RconFL_HomeR))
			{
				// Robot has turned back heading towards charge stub.
				ROS_INFO("Check position right and return 2.");
				return 2;
			}
		}
		// if(Is_Remote())return 1;
		if(Touch_Detect())
		{
			return 1;
		}
		//if((Check_Motor_Current()==Check_Left_Wheel)||(Check_Motor_Current()==Check_Right_Wheel))return 1;
	}
	//Reset_TempPWM();
	return 0;
}

void By_Path(void)
{
	uint8_t Cycle=0;
	//uint8_t Mobility_Temp_Error=0;
	//uint32_t Temp_Mobility_Distance=0;
	uint32_t Receive_Code=0;
	uint32_t Temp_Code =0 ;
	uint8_t Position_Far=1;
	uint16_t NoSignal_Counter=0;
	uint8_t Temp_Check_Position=0;
	uint8_t Near_Counter=0;
	volatile uint8_t Bumper_Counter=0;
	uint8_t Side_Counter=0;
	//Reset_Wheel_Step();

	Reset_Touch();
	//Display_Content(LED_Home,100,100,0,7);

	// Enable the charge function
	set_start_charge();

	Move_Forward(9,9);
	Set_SideBrush_PWM(30,30);
	Set_MainBrush_PWM(30);
	Set_BLDC_Speed(Vac_Speed_NormalL);
//  Set_LED(100,100);
//	SetHomeRemote();
//	Display_Home_LED();
	while(1)
	{
		Receive_Code=0;
//		if(Get_LeftWheel_Step()<500)
//		{
//			Mobility_Temp_Error=0;
//			Temp_Mobility_Distance = Get_Move_Distance();
//		}
//		else
//		{
//		  if((Get_Move_Distance()-Temp_Mobility_Distance)>500)
//		  {
//		    Temp_Mobility_Distance = Get_Move_Distance();
//		    if(Get_Mobility_Step()<1)
//		    {
//		      Mobility_Temp_Error++;
//		      if(Mobility_Temp_Error>3)
//		      {
//		        Set_Clean_Mode(Clean_Mode_GoHome);
//		        return;
//		      }
//		    }
//		    else
//		    {
//		      Mobility_Temp_Error=0;
//		    }
//		    Reset_Mobility_Step();
//		  }
//		}

		Cycle=10;
		ROS_INFO("%s, %d: Refresh cycle.", __FUNCTION__, __LINE__);
		while(Cycle--)
		{
			ROS_INFO("new round, Bumper_Counter = %d.", Bumper_Counter);
			if(Is_ChargerOn())
			{
				ROS_INFO("Is_ChargerOn!!");
				Stop_Brifly();
				//delay(2000);
				usleep(200000);
				if(Is_ChargerOn())
				{
					//delay(5000);
					usleep(200000);
					if(Is_ChargerOn())
					{
						//Reset_Error_Code();
						Set_Clean_Mode(Clean_Mode_Charging);
						Beep(2, 15, 0, 1);
						//Reset_Rcon_Remote();
						return;
					}
				}
				else if(Turn_Connect())
				{
					Set_Clean_Mode(Clean_Mode_Charging);
					//Reset_Rcon_Remote();
					return;
				}
				else
				{
					Set_SideBrush_PWM(30,30);
					Set_MainBrush_PWM(0);
					////Back(30,800);
					//Back(30,300);
					Quick_Back(30,300);
					Set_MainBrush_PWM(30);
					Stop_Brifly();
				}
			}
			/*----------------------------------------------OBS------------------Event---------------*/
			ROS_INFO("get_Left_bumper_Status");
			if(Get_Bumper_Status()&&LeftBumperTrig)//waiting for modify
			{
				//Random_Back();
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
					Set_SideBrush_PWM(30,30);
					Set_MainBrush_PWM(0);
					//Back(30,2500);//waiting
					Quick_Back(30,300);//waiting
					ROS_INFO("%d: Quick_Back in !position_far", __LINE__);
					Set_MainBrush_PWM(30);
					Stop_Brifly();
					if(Bumper_Counter>0)
					{
						Move_Forward(0,0);
						Set_Clean_Mode(Clean_Mode_GoHome);
						ROS_INFO("%d, Return from LeftBumperTrig.", __LINE__);
						return;
					}
				}
				else if((Get_Rcon_Status()&(RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR))==0)
				{
					Random_Back();
					Turn_Right(Turn_Speed,1100);
					Move_Forward(8,8);
					Set_Clean_Mode(Clean_Mode_GoHome);
					ROS_INFO("%d, Return from LeftBumperTrig.", __LINE__);
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
				ROS_INFO("%d, Left bumper count =%d.", __LINE__, Bumper_Counter);
			}
			ROS_INFO("Get_Right_Bumper_Status");
			if(Get_Bumper_Status()&RightBumperTrig)
			{
				//Random_Back();
				Reset_Rcon_Status();
				if(!Position_Far)
				{
					Stop_Brifly();
					if(Turn_Connect())
					{
						Set_Clean_Mode(Clean_Mode_Charging);
						return;
					}
					Set_SideBrush_PWM(30,30);
					Set_MainBrush_PWM(0);
					Quick_Back(30,300);
					Set_MainBrush_PWM(30);
					Stop_Brifly();
					if(Bumper_Counter>0)
					{
						Move_Forward(0,0);
						Set_Clean_Mode(Clean_Mode_GoHome);
						ROS_INFO("%d, Return from RightBumperTrig.", __LINE__);
						return;
					}
				}
				//else if((Get_Rcon_Status()&0X00FF)==0)
				else if((Get_Rcon_Status()&(RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR))==0)
				{
					Random_Back();
					Turn_Left(Turn_Speed,1100);
					Move_Forward(8,8);
					Set_Clean_Mode(Clean_Mode_GoHome);
					ROS_INFO("%d, Return from RightBumperTrig.", __LINE__);
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
				ROS_INFO("%d, Right bumper count =%d.", __LINE__, Bumper_Counter);
			}

			if(Position_Far)
			{
				if(Get_Cliff_Trig())
				{
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
					Set_Wheel_Speed(0,0);
					Set_Dir_Backward();
					//delay(300);
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

			/*------------------------------------------------------Touch and Remote event-----------------------*/
			if(Touch_Detect())
			{
				//Reset_Touch();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				return;
			}
			if(GetBatteryVoltage()<Low_Battery_Limit)
			{
				Display_Battery_Status(Display_Low);
				//delay(10000);
				usleep(1000000);
				Set_Clean_Mode(Clean_Mode_Sleep);
				return;
			}

			if(Home_Check_Current())return;
			//delay(100);
			usleep(10000);

		}

		//delay(500);

		Receive_Code = Get_Rcon_Status();
		Temp_Code = Receive_Code;
		//Temp_Code &= 0x000330ff;
		Temp_Code &= (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR|RconBL_HomeL|RconBL_HomeR|RconBR_HomeL|RconBR_HomeR);
		if(Receive_Code)
		{
			//if((Receive_Code&0x00000600)==0x00000600)Position_Far=0;
			if((Receive_Code&(RconFL_HomeT|RconFR_HomeT))==(RconFL_HomeT|RconFR_HomeT))
			{
				Position_Far=0;
				ROS_INFO("%s, %d: Robot face HomeT, Position_Far = 0.", __FUNCTION__, __LINE__);
			}
			//if(Receive_Code&0x00000900)Position_Far=0;
			if(Receive_Code&(RconL_HomeT|RconR_HomeT))
			{
				Position_Far=0;
				ROS_INFO("%s, %d: Robot side face HomeT, Position_Far = 0.", __FUNCTION__, __LINE__);
			}
			//if(Receive_Code&0x00000f00)
			if(Receive_Code&(RconL_HomeT|RconR_HomeT|RconFL_HomeT|RconFR_HomeT))
			{
				// If robot sees the HomeT signal.
				Near_Counter++;
				if(Near_Counter>1)
				{
					Position_Far=0;
					ROS_INFO("%s, %d: Robot near HomeT counter > 1, Position_Far = 0.", __FUNCTION__, __LINE__);
				}
				//if((Receive_Code&0x000000ff)==0)
				if((Receive_Code&(RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR))==0)
				{
					Side_Counter++;
					if(Side_Counter>5)
					{
						Move_Forward(9,9);
						ROS_INFO("%s, %d: Robot goes far, back to gohome mode.", __FUNCTION__, __LINE__);
						Set_Clean_Mode(Clean_Mode_GoHome);
						return;
					}
				}
				else
				{
					Side_Counter=0;
				}
			}

			//if((Receive_Code&0x00000024)==0x00000024)Position_Far=0;
			if((Receive_Code&(RconFL_HomeL|RconFR_HomeR))==(RconFL_HomeL|RconFR_HomeR))
			{
				ROS_INFO("%s, %d: Robot sees HomeL or HomeR, Position_Far = 0.", __FUNCTION__, __LINE__);
				Position_Far=0;
			}
			Reset_Rcon_Status();
			NoSignal_Counter=0;
		}
		else
		{
			Near_Counter=0;
			NoSignal_Counter++;
			if(NoSignal_Counter>50)
			{
				NoSignal_Counter=0;
				Stop_Brifly();
				Temp_Check_Position = Check_Position(Round_Left);
				ROS_INFO("%s %d Check position return %d.", __FUNCTION__, __LINE__, Temp_Check_Position);
				if(Temp_Check_Position==1)
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

		if(Position_Far)
		{
			switch(Temp_Code)
			{
				//case 0x0000003c:Move_Forward(14,14);break;//00 11 11 00
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: Position_Far, F-LR.", __FUNCTION__, __LINE__);
					Move_Forward(14,14);
					break;//00 11 11 00
				//case 0x00000024:Move_Forward(13,13);break;//00 10 01 00
				case (RconFL_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-L, FR_R.", __FUNCTION__, __LINE__);
					Move_Forward(13,13);
					break;//00 10 01 00

				//case 0x000000ff:Move_Forward(12,12);break;//11 11 11 11*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/L/R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(12,12);
					break;//11 11 11 11*

				//case 0x000000fc:Move_Forward(8,15);break;//11 11 11 00*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(8,15);
					break;//11 11 11 00*
				//case 0x0000003f:Move_Forward(15,8);break;//00 11 11 11*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(15,8);
					break;//00 11 11 11*

				//case 0x000000fd:Move_Forward(8,15);break;//11 11 11 01*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/L-LR, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(8,15);
					break;//11 11 11 01*
				//case 0x000000bf:Move_Forward(15,8);break;//10 11 11 11*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/R-LR, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(15,8);
					break;//10 11 11 11*

				//case 0x000000f0:Move_Forward(8,15);break;//11 11 00 00*
				case (RconFL_HomeL|RconFL_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(8,15);
					break;//11 11 00 00*
				//case 0x0000000f:Move_Forward(15,8);break;//00 00 11 11*
				case (RconFR_HomeL|RconFR_HomeR|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FR/R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(15,8);
					break;//00 00 11 11*

				//case 0x00000034:Move_Forward(10,13);break;//00 11 01 00
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-LR, FR-R.", __FUNCTION__, __LINE__);
					Move_Forward(10,13);
					break;//00 11 01 00
				//case 0x00000014:Move_Forward(10,15);break;//00 01 01 00
				case (RconFL_HomeR|RconFR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR-R.", __FUNCTION__, __LINE__);
					Move_Forward(10,15);
					break;//00 01 01 00
				//case 0x00000010:Move_Forward(5,15);break; //00 01 00 00
				case (RconFL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-R.", __FUNCTION__, __LINE__);
					Move_Forward(5,15);
					break; //00 01 00 00

				//case 0x00000074:Move_Forward(11,15);break;//01 11 01 00*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-LR, FR/L-R.", __FUNCTION__, __LINE__);
					Move_Forward(11,15);
					break;//01 11 01 00*
				//case 0x000000f4:Move_Forward(11,15);break;//11 11 01 00*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/L-LR, FR-R.", __FUNCTION__, __LINE__);
					Move_Forward(11,15);
					break;//11 11 01 00*
				//case 0x000000d4:Move_Forward(10,15);break;//11 01 01 00*
				case (RconFL_HomeR|RconFR_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR-R, L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(10,15);
					break;//11 01 01 00*
				//case 0x000000d0:Move_Forward(3,8);break; //11 01 00 00*
				case (RconFL_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-R, L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(3,8);
					break; //11 01 00 00*

				//case 0x0000002c:Move_Forward(13,10);break;//00 10 11 00
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-L, FR-LR.", __FUNCTION__, __LINE__);
					Move_Forward(13,10);
					break;//00 10 11 00
				//case 0x00000028:Move_Forward(15,10);break;//00 10 10 00
				case (RconFL_HomeL|RconFR_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL/FR-L.", __FUNCTION__, __LINE__);
					Move_Forward(15,10);
					break;//00 10 10 00
				//case 0x00000008:Move_Forward(15,5);break; //00 00 10 00
				case (RconFR_HomeL):
					ROS_INFO("%s, %d: Position_Far, FR-L.", __FUNCTION__, __LINE__);
					Move_Forward(15,5);
					break; //00 00 10 00

				//case 0x0000002e:Move_Forward(15,11);break;//00 10 11 10*
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconR_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL/R-L, FR-LR.", __FUNCTION__, __LINE__);
					Move_Forward(15,11);
					break;//00 10 11 10*
				//case 0x0000002f:Move_Forward(15,11);break;//00 10 11 11*
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-L, FR/R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(15,11);
					break;//00 10 11 11*
				//case 0x0000002b:Move_Forward(15,10);break;//00 10 10 11*
				case (RconFL_HomeL|RconFR_HomeL|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/R-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(15,10);
					break;//00 10 10 11*
				//case 0x0000000b:Move_Forward(8,3);break; //00 00 10 11*
				case (RconFR_HomeL|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FR/R-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(8,3);
					break; //00 00 10 11*
				//right
				//case 0x00000001:Move_Forward(15,0);break;      //00 00 00 01#
				case (RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/R-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(15,0);
					break;      //00 00 00 01#
//  		    case 0x0001:Turn_Right(20,300);       //00 00 00 01
//  		                Stop_Brifly();
//  		                Move_Forward(30,0);
//  		                break;                    //00 00 00 10
				//case 0x00000002:Turn_Right(20,500);
				case (RconR_HomeL):
					ROS_INFO("%s, %d: Position_Far, R-L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);
					Stop_Brifly();
					Move_Forward(30,0);
					break;
				//case 0x00000003:Move_Forward(15,0);break;       //00 00 00 11
				case (RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(15,0);
					break;       //00 00 00 11

				//case 0x00000080:Move_Forward(0,15);break;        //10 00 00 00
				case (RconL_HomeL):
					ROS_INFO("%s, %d: Position_Far, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(0,15);
					break;        //10 00 00 00
//    	    case 0x0080:Turn_Left(20,300);        //10 00 00 00
//    	                Stop_Brifly();
//    	                Move_Forward(0,30);
//    	                break;
				//case 0x00000040:Turn_Left(20,500);        //01 00 00 00
				case (RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, L-R.", __FUNCTION__, __LINE__);
					Turn_Left(20,500);        //01 00 00 00
					Stop_Brifly();
					Move_Forward(0,30);
					break;
				//case 0x000000c0:Move_Forward(0,15);break;        // 11 00 00 00
				case (RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(0,15);
					break;        // 11 00 00 00

				/*----------------------*/
				//case 0x00000009:Move_Forward(15,8);break; //00 00 10 01
				case (RconFR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FR-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(15,8);
					break; //00 00 10 01
				//case 0x00000090:Move_Forward(8,15);break; //10 01 00 00
				case (RconFL_HomeR|RconL_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL-R, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(8,15);
					break; //10 01 00 00

				//case 0x00000094:Move_Forward(8,13);break; //10 01 01 00
				case (RconFL_HomeR|RconFR_HomeR|RconL_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL/FR-R, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(8,13);
					break; //10 01 01 00
				//case 0x00000029:Move_Forward(13,8);break; //00 01 10 01
				case (RconFL_HomeL|RconFR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(13,8);
					break; //00 01 10 01

				//case 0x0000000d:Move_Forward(14,10);break; //00 00 11 01
				case (RconFR_HomeL|RconFR_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FR-LR, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(14,10);
					break; //00 00 11 01
				//case 0x000000b0:Move_Forward(10,14);break; //10 11 00 00
				case (RconFL_HomeL|RconFL_HomeR|RconL_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL-LR, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(10,14);
					break; //10 11 00 00

				//case 0x0000000A:Move_Forward(15,8);break; //00 00 10 10
				case (RconFR_HomeL|RconR_HomeL):
					ROS_INFO("%s, %d: Position_Far, FR-L, R-L.", __FUNCTION__, __LINE__);
					Move_Forward(15,8);
					break; //00 00 10 10
				//case 0x00000050:Move_Forward(8,15);break; //01 01 00 00
				case (RconFL_HomeR|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-L, L-R.", __FUNCTION__, __LINE__);
					Move_Forward(8,15);
					break; //01 01 00 00

				//case 0x0000002A:Move_Forward(15,8);break; //00 10 10 10
				case (RconFL_HomeL|RconFR_HomeL|RconR_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL/FR-L, R-L.", __FUNCTION__, __LINE__);
					Move_Forward(15,8);
					break; //00 10 10 10
				//case 0x00000054:Move_Forward(8,15);break; //01 01 01 00
				case (RconFL_HomeR|RconFR_HomeR|RconL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR-R, L-R.", __FUNCTION__, __LINE__);
					Move_Forward(8,15);
					break; //01 01 01 00

				//case 0x0000002D:Move_Forward(12,9);break; //00 10 11 01
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-L, FR-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(12,9);
					break; //00 10 11 01
				//case 0x000000B4:Move_Forward(9,12);break; //10 11 01 00
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconL_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL-LR, FR-R, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(9,12);
					break; //10 11 01 00

				//case 0x00000041:Move_Forward(15,9);break; //01 00 00 01
				case (RconL_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, LR-R.", __FUNCTION__, __LINE__);
					Move_Forward(15,9);
					break; //01 00 00 01
				//case 0x00000082:Move_Forward(9,15);break; //10 00 00 10
				case (RconL_HomeL|RconR_HomeL):
					ROS_INFO("%s, %d: Position_Far, LR-L.", __FUNCTION__, __LINE__);
					Move_Forward(9,15);
					break; //10 00 00 10

				//case 0x0000000c:Move_Forward(12,8);break; //00 00 11 00
				case (RconFR_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: Position_Far, LR-R.", __FUNCTION__, __LINE__);
					Move_Forward(12,8);
					break; //00 00 11 00
				//case 0x00000030:Move_Forward(8,12);break; //00 11 00 00
				case (RconFL_HomeL|RconFL_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-LR.", __FUNCTION__, __LINE__);
					Move_Forward(8,12);
					break; //00 11 00 00

				//case 0x00000005:Move_Forward(15,4);break; //00 00 01 01
				case (RconFR_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FR/R-R.", __FUNCTION__, __LINE__);
					Move_Forward(15,4);
					break; //00 00 01 01
				//case 0x000000a0:Move_Forward(4,15);break; //10 10 00 00
				case (RconFL_HomeL|RconL_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL/L-L.", __FUNCTION__, __LINE__);
					Move_Forward(4,15);
					break; //10 10 00 00

				//case 0x00000004:Move_Forward(8,15);break; //00 00 01 00*
				case (RconFR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FR-R.", __FUNCTION__, __LINE__);
					Move_Forward(8,15);
					break; //00 00 01 00*
				//case 0x00000020:Move_Forward(15,8);break; //00 10 00 00*
				case (RconFL_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL-L.", __FUNCTION__, __LINE__);
					Move_Forward(15,8);
					break; //00 10 00 00*

				//case 0x00000095:Move_Forward(11,13);break;//10 01 01 01
				case (RconFL_HomeR|RconFR_HomeR|RconL_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/R-R, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(11,13);
					break;//10 01 01 01
				//case 0x000000A9:Move_Forward(13,11);break;//10 10 10 01
				case (RconFL_HomeL|RconFR_HomeL|RconL_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL/FR/L-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(13,11);
					break;//10 10 10 01

				//case 0x000000B5:Move_Forward(12,14);break;//10 11 01 01
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconL_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-LR, FR-R, L-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(12,14);
					break;//10 11 01 01
				//case 0x000000AD:Move_Forward(14,12);break;//10 10 11 01
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-L, FR-LR, L-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(14,12);
					break;//10 10 11 01

				//case 0x0000001c:Move_Forward(10,8);break; //00 01 11 00
				case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: Position_Far, FL-R, FR-LR.", __FUNCTION__, __LINE__);
					Move_Forward(10,8);
					break; //00 01 11 00
				//case 0x00000038:Move_Forward(8,10);break; //00 11 10 00
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
					ROS_INFO("%s, %d: Position_Far, FL-LR, FR-L.", __FUNCTION__, __LINE__);
					Move_Forward(8,10);
					break; //00 11 10 00

				//case 0x00001000: Turn_Right(20,1100);         //00 00 00 00 00 10
				case (RconBR_HomeL):
					ROS_INFO("%s, %d: Position_Far, BR-L.", __FUNCTION__, __LINE__);
					Turn_Right(20,1100);         //00 00 00 00 00 10
					Stop_Brifly();
					Move_Forward(12,12);
					break;
				//case 0x00020000:Turn_Left(20,1100);         //01 00 00 00 00 00
				case (RconBL_HomeR):
					ROS_INFO("%s, %d: Position_Far, BL-R.", __FUNCTION__, __LINE__);
					Turn_Left(20,1100);         //01 00 00 00 00 00
					Stop_Brifly();
					Move_Forward(12,12);
					break;

				//case 0x00002000:Move_Forward(10,0);break; //00 00 00 00 00 01
				case (RconBR_HomeR):
					ROS_INFO("%s, %d: Position_Far, BL-R.", __FUNCTION__, __LINE__);
					Move_Forward(10,0);
					break; //00 00 00 00 00 01
				//case 0x00010000:Move_Forward(0,10);break; //10 00 00 00 00 00
				case (RconBL_HomeL):
					ROS_INFO("%s, %d: Position_Far, BL-L.", __FUNCTION__, __LINE__);
					Move_Forward(0,10);
					break; //10 00 00 00 00 00

				default:
					ROS_INFO("%s, %d: Position_Far, else:%x.", __FUNCTION__, __LINE__, Temp_Code);
					Move_Forward(12,15);
					break;
			}
		}
		else
		{
			//Temp_Code&=0x000000ff;
			Temp_Code&=(RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR);
			switch(Temp_Code)
			{
				//case 0x0000003c:Move_Forward(10,10);break;// 00 11 11 00
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR-LR.", __FUNCTION__, __LINE__);
					Move_Forward(10,10);
					break;// 00 11 11 00
				//case 0x00000024:Move_Forward(9,9);break;// 00 10 01 00
				case (RconFL_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-L, FR-R.", __FUNCTION__, __LINE__);
					Move_Forward(9,9);
					break;// 00 10 01 00

				//case 0x000000ff:Move_Forward(8,8);break;//11 11 11 11*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR/L/R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(8,8);
					break;//11 11 11 11*

				//case 0x000000fc:Move_Forward(3,7);break;//11 11 11 00*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR/L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(3,7);
					break;//11 11 11 00*
				//case 0x0000003f:Move_Forward(7,3);break;//00 11 11 11*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR/R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(7,3);
					break;//00 11 11 11*

				//case 0x000000fd:Move_Forward(5,7);break;//11 11 11 01*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconL_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR/L-LR, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(5,7);
					break;//11 11 11 01*
				//case 0x000000bf:Move_Forward(7,5);break;//10 11 11 11*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR/R-LR, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(7,5);
					break;//10 11 11 11*

				//case 0x000000f0:Move_Forward(3,7);break;//11 11 00 00*
				case (RconFL_HomeL|RconFL_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(3,7);
					break;//11 11 00 00*
				//case 0x0000000f:Move_Forward(7,3);break;//00 00 11 11*
				case (RconFR_HomeL|RconFR_HomeR|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FR/R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(7,3);
					break;//00 00 11 11*

				//case 0x00000034:Move_Forward(4,7);break;// 00 11 01 00
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-LR, FR-R.", __FUNCTION__, __LINE__);
					Move_Forward(3,8);
					break;// 00 11 01 00
				//case 0x00000014:Move_Forward(2,6);break;// 00 01 01 00
				case (RconFL_HomeR|RconFR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR-R.", __FUNCTION__, __LINE__);
					Move_Forward(1,6);
					break;// 00 01 01 00
				//case 0x00000010:Move_Forward(2,6);break;// 00 01 00 00
				case (RconFL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-R.", __FUNCTION__, __LINE__);
					Move_Forward(1,6);
					break;// 00 01 00 00

				//case 0x00000074:Move_Forward(4,7);break;//01 11 01 00*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-LR, FR/L-R.", __FUNCTION__, __LINE__);
					Move_Forward(3,8);
					break;//01 11 01 00*
				//case 0x000000f4:Move_Forward(4,7);break;//11 11 01 00*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-LR, FR-R, L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(3,8);
					break;//11 11 01 00*
				//case 0x000000d4:Move_Forward(6,8);break;//11 01 01 00*
				case (RconFL_HomeR|RconFR_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR-R, L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break;//11 01 01 00*
				//case 0x000000d0:Move_Forward(3,8);break; //11 01 00 00*
				case (RconFL_HomeR|RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-R, L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(3,8);
					break; //11 01 00 00*

				//case 0x0000002c:Move_Forward(7,5);break; // 00 10 11 00
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR-R, L-LR.", __FUNCTION__, __LINE__);
					Move_Forward(7,5);
					break; // 00 10 11 00
				//case 0x00000028:Move_Forward(6,2);break; // 00 10 10 00
				case (RconFL_HomeL|RconFR_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL/FR-L.", __FUNCTION__, __LINE__);
					Move_Forward(6,1);
					break; // 00 10 10 00
				//case 0x00000008:Move_Forward(6,2);break;// 00 00 10 00
				case (RconFR_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FR-L.", __FUNCTION__, __LINE__);
					Move_Forward(6,1);
					break;// 00 00 10 00

				//case 0x0000002e:Move_Forward(7,5);break;//00 10 11 10*
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconR_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL-L, FR-LR, R-L.", __FUNCTION__, __LINE__);
					Move_Forward(7,5);
					break;//00 10 11 10*
				//case 0x0000002f:Move_Forward(7,4);break;//00 10 11 11*
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-L, FR-LR, R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(7,4);
					break;//00 10 11 11*
				//case 0x0000002b:Move_Forward(8,6);break;//00 10 10 11*
				case (RconFL_HomeL|RconFR_HomeL|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR-L, R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break;//00 10 10 11*
				//case 0x0000000b:Move_Forward(8,3);break; //00 00 10 11*
				case (RconFR_HomeL|RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FR-L, R-LR.", __FUNCTION__, __LINE__);
					Move_Forward(8,3);
					break; //00 00 10 11*
				//right
				//case 0x00000001:Move_Forward(12,0);break;      //00 00 00 01#
				case (RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(12,0);
					break; // 00 00 00 01#
//         		case 0x0001:Turn_Right(20,300);      // 00 00 00 01
//         		       Stop_Brifly();
//         		       Move_Forward(14,0);
//         		       break;
				//case 0x00000002:Turn_Right(20,500);      // 00 00 00 10
				case (RconR_HomeL):
					ROS_INFO("%s, %d: !Position_Far, R-L.", __FUNCTION__, __LINE__);
					Turn_Right(20,500);      // 00 00 00 10
					Stop_Brifly();
					Move_Forward(14,0);
					break;
				//case 0x00000003:Turn_Right(20,400);      // 00 00 00 11
				case (RconR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, R-LR.", __FUNCTION__, __LINE__);
					Turn_Right(20,400);      // 00 00 00 11
					Stop_Brifly();
					Move_Forward(14,0);
					break;
				//case 0x00000080:Move_Forward(0,12);break;   // 10 00 00 00#
				case (RconL_HomeL):
					ROS_INFO("%s, %d: !Position_Far, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(0,12);
					break;   // 10 00 00 00#
//         0x0	080:Turn_Left(20,300);        // 10 00 00 00
//            	    Stop_Brifly();
//            	    Move_Forward(0,14);
//            	    break;
				//case 0x00000040:Turn_Left(20,500);        // 01 00 00 00
				case (RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, L-R.", __FUNCTION__, __LINE__);
					Turn_Left(20,500);        // 01 00 00 00
					Stop_Brifly();
					Move_Forward(0,14);
					break;
				//case 0x000000c0:Turn_Left(20,400);        // 11 00 00 00
				case (RconL_HomeL|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, L-LR.", __FUNCTION__, __LINE__);
					Turn_Left(20,400);        // 11 00 00 00
					Stop_Brifly();
					Move_Forward(0,14);
					break;
				/*----------------------*/
				//case 0x00000009:Move_Forward(10,3);break;// 00 00 10 01
				case (RconFR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FR-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(10,3);
					break;// 00 00 10 01
				//case 0x00000090:Move_Forward(3,10);break;// 10 01 00 00
				case (RconFL_HomeR|RconL_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL-R, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(3,10);
					break;// 10 01 00 00

				//case 0x00000094:Move_Forward(4,8);break;// 10 01 01 00
				case (RconFL_HomeR|RconFR_HomeR|RconL_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL-R, FR-R, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(4,8);
					break;// 10 01 01 00
				case (RconFL_HomeL|RconFR_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-L, FR-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(8,4);
					break;// 00 10 10 01

				//case 0x0000000d:Move_Forward(12,7);break; // 00 00 11 01
				case (RconFR_HomeL|RconFR_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FR-LR, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(12,7);
					break; // 00 00 11 01
				//case 0x000000b0:Move_Forward(7,12);break; // 10 11 00 00
				case (RconFL_HomeL|RconFL_HomeR|RconL_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL-LR, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(7,12);
					break; // 10 11 00 00

				//case 0x0000000A:Move_Forward(12,5);break; // 00 00 10 10
				case (RconFR_HomeL|RconR_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FR/R-L.", __FUNCTION__, __LINE__);
					Move_Forward(12,5);
					break; // 00 00 10 10
				//case 0x00000050:Move_Forward(5,12);break; // 01 01 00 00
				case (RconFL_HomeR|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/L-R.", __FUNCTION__, __LINE__);
					Move_Forward(5,12);
					break; // 01 01 00 00

				//case 0x0000002A:Move_Forward(8,7);break; // 00 10 10 10
				case (RconFL_HomeL|RconFR_HomeL|RconR_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL/FR/R-L.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break; // 00 10 10 10
				//case 0x00000054:Move_Forward(7,8);break; // 01 01 01 00
				case (RconFL_HomeR|RconFR_HomeR|RconL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR/L-R.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break; // 01 01 01 00

				//case 0x0000002D:Move_Forward(9,5);break;  // 00 10 11 01
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-L, FR-LR, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(9,5);
					break;  // 00 10 11 01
				//case 0x000000B4:Move_Forward(5,9);break;  // 10 11 01 00
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconL_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL-LR, FR-R, L-L.", __FUNCTION__, __LINE__);
					Move_Forward(5,9);
					break;  // 10 11 01 00

				//case 0x00000041:Move_Forward(9,5);break; // 01 00 00 01
				case (RconL_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, R/R-R.", __FUNCTION__, __LINE__);
					Move_Forward(9,5);
					break; // 01 00 00 01
				//case 0x00000082:Move_Forward(5,9);break; // 10 00 00 10
				case (RconL_HomeL|RconR_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FR/R-L.", __FUNCTION__, __LINE__);
					Move_Forward(5,9);
					break; // 10 00 00 10

				//case 0x0000000c:Move_Forward(9,7);break; // 00 00 11 00
				case (RconFR_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FR-LR.", __FUNCTION__, __LINE__);
					Move_Forward(9,6);
					break; // 00 00 11 00
				//case 0x00000030:Move_Forward(6,8);break;// 00 11 00 00
				case (RconFL_HomeL|RconFL_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-LR.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break;// 00 11 00 00

				//case 0x00000005:Move_Forward(11,6);break;// 00 00 01 01
				case (RconFR_HomeR|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FR/R-R.", __FUNCTION__, __LINE__);
					Move_Forward(11,6);
					break;// 00 00 01 01
				//case 0x000000a0:Move_Forward(6,11);break; // 10 10 00 00
				case (RconFL_HomeL|RconL_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL/L-L.", __FUNCTION__, __LINE__);
					Move_Forward(6,11);
					break; // 10 10 00 00

				//case 0x00000004:Move_Forward(7,9);break; // 00 00 01 00*
				case (RconFR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FR-R.", __FUNCTION__, __LINE__);
					//Move_Forward(7,9);
					Move_Forward(6,8);
					break; // 00 00 01 00*
				//case 0x00000020:Move_Forward(8,6);break; // 00 10 00 00*
				case (RconFL_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL-L.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break; // 00 10 00 00*

				//case 0x00000095:Move_Forward(7,8);break; // 10 01 01 01
				case (RconFL_HomeR|RconFR_HomeR|RconL_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR-R, L-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break; // 10 01 01 01
				//case 0x000000A9:Move_Forward(8,7);break; // 10 10 10 01
				case (RconFL_HomeL|RconFR_HomeL|RconL_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL/FR/L-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break; // 10 10 10 01

				//case 0x000000B5:Move_Forward(7,8);break; // 10 11 01 01
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconL_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-LR, FR-R, L-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break; // 10 11 01 01
				//case 0x000000AD:Move_Forward(8,7);break; // 10 10 11 01
				case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconL_HomeL|RconR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-L, FR-LR, L-L, R-R.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break; // 10 10 11 01

				//case 0x0000001c:Move_Forward(7,8);break;// 00 01 11 00*
				case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
					ROS_INFO("%s, %d: !Position_Far, FL-R, FR-LR.", __FUNCTION__, __LINE__);
					Move_Forward(6,8);
					break;// 00 01 11 00*
				//case 0x00000038:Move_Forward(8,7);break;// 00 11 10 00*
				case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
					ROS_INFO("%s, %d: !Position_Far, FL-LR, FR-L.", __FUNCTION__, __LINE__);
					Move_Forward(8,6);
					break;// 00 11 10 00*

				default:
					ROS_INFO("%s, %d: !Position_Far, else:%x.", __FUNCTION__, __LINE__, Temp_Code);
					//Move_Forward(7,8);
					Move_Forward(7,7);
					break;
			}
		}
//
//		USART_DMA_String(11,"\n\n\r HOME   ");//left cliff
//USART_DMA_Numbers(Temp_Code);
		//Display_TM1618(Temp_Code,1);
//Display_Content(0,Temp_Code/100,Temp_Code%100,0,1);
				//delay(500);
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

