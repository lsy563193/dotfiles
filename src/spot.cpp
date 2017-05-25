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

#include "movement.h"
#include "spot.h"
#include "path_planning.h"
#include "robot.hpp"
#include "robotbase.h"
#include "core_move.h"
#include "map.h"
#include "gyro.h"
#include "wav.h"
#include "motion_manage.h"
#include "slam.h"
#include <ros/ros.h>
#include <time.h>
#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed	18
#endif

#define SPOT_MAX_SPEED	(20)
extern uint8_t g_should_follow_wall;//used to decide obstacle 
/* --------------------------------------------------Random Runnincg mode----------------------*/
void Spot_Mode(SpotType ST)
{
	uint8_t Motor_OC_Counter = 0;
	uint16_t Radius = 0;
	uint8_t Move_Style = 1;
	uint8_t Spot_Flag = 0;
	uint8_t OBS_Counter = 0;
	uint8_t Stunk = 0;
	uint16_t Counter_Watcher = 0;
	//these param for adjust spiral motion
	uint8_t sn=2;
    uint16_t rp=230;
	uint16_t ran = 250;
	if(ST == NormalSpot){
		sn = 3;
		rp = 230;
		ran = 400;
	}
	else if(ST == CleanSpot || ST == WallSpot){
		sn = 3;
		rp = 230; 
		ran = 200;
	}
	Move_Style = Spiral_Right_Out;

	Reset_Stop_Event_Status();

	if (!Is_Gyro_On())
	{
		// Restart the gyro.
		Set_Gyro_Off();
		// Wait for 30ms to make sure the off command has been effectived.
		usleep(30000);
		// Set gyro on before wav_play can save the time for opening the gyro.
		Set_Gyro_On();
		wav_play(WAV_CLEANING_SPOT);
		if (!Wait_For_Gyro_On())
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			return;
		}
	}
	else
	{
		wav_play(WAV_CLEANING_SPOT);
	}
	Set_LED(100,0);
	Set_SideBrush_PWM(60, 60);
	Set_MainBrush_PWM(90);
	Set_BLDC_Speed(90);

#ifdef BLDC_INSTALL
	usleep(10000);
	Set_VacMode(Vac_Max);
	Set_Vac_Speed();
#endif

	usleep(10000);
	Set_Dir_Right();
	Set_Wheel_Speed(25,5);
	Counter_Watcher=0;
	Reset_Rcon_Remote();
	Motor_OC_Counter=0;
	Reset_Wheel_Step();
	Reset_Work_Time();
	uint32_t step;
//	while (Get_LeftWheel_Step() < 6900 && ros::ok()) {
	while(ros::ok()){
		#ifdef OBS_DYNAMIC_MOVETOTARGET
		OBS_Dynamic_Base(20);
		#endif
		step =Get_LeftWheel_Step();
		if(step >=6900)break;
		usleep(40000);
		Counter_Watcher++;
		if (Counter_Watcher > 1000) {//about 200 seconds
			break;
		}
		Set_Dir_Right();
		Set_Wheel_Speed(25, 10);

		/*------------------------------------------------------stop event-----------------------*/
		if (Stop_Event()) {
//			Beep(5, 20, 0, 1);
			Stop_Brifly();
			if(ST == NormalSpot){
				Set_Clean_Mode(Clean_Mode_Userinterface);	
				wav_play(WAV_CLEANING_FINISHED);
			}
			// Key release detection, if user has not release the key, don't do anything.
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			Reset_Stop_Event_Status();
			return;
		}
		if (Check_Motor_Current()) {
			Motor_OC_Counter++;
			if (Motor_OC_Counter > 10) {
				Motor_OC_Counter = 0;
				if(ST == NormalSpot){
					Set_Clean_Mode(Clean_Mode_Userinterface);
					wav_play(WAV_CLEANING_FINISHED);
				}
				return;
			}
		} else {
			Motor_OC_Counter = 0;
		}
		if (Get_OBS_Status() || Get_Cliff_Trig()) {
			Move_Back();
			Stop_Brifly();
			Turn_Left(Turn_Speed, 2500);
			Move_Style = Spiral_Left_Out;
			break;
		}
		if(Get_Rcon_Remote())
		{
			if(ST == NormalSpot){
				if(Remote_Key(Remote_All)){
					Reset_Rcon_Remote();	
					Set_Clean_Mode(Clean_Mode_Userinterface);
					Disable_Motors();
					Stop_Brifly();
					wav_play(WAV_CLEANING_FINISHED);
					return;
				}
			}
			else if(ST == CleanSpot || ST == WallSpot){
					if (Remote_Key(Remote_Left | Remote_Right | Remote_Forward)){
						Reset_Rcon_Remote();
						return;
					}
					else if(Remote_Key(Remote_Home)){
						Set_MoveWithRemote();
						SetHomeRemote();
						Reset_Rcon_Remote();
						return ;
					}
			}
			Reset_Rcon_Remote();
		}
	}

	//Move_Forward(5, 5);
	Reset_Wheel_Step();
	Reset_Wall_Step();
	Set_MainBrush_PWM(90);
	Set_SideBrush_PWM(60, 60);
	Set_BLDC_Speed(90);
	Motor_OC_Counter = 0;

	while (ros::ok()) {
		usleep(10000);
		/*------------------------------------------------------Check Battery-----------------------*/
		if (Check_Bat_SetMotors(135000, 80000, 100000)) {	//Low Battery Event
			ROS_WARN("%s %d: Battery too low (< LOW_BATTERY_STOP_VOLTAGE)", __FUNCTION__, __LINE__);
			usleep(30000);
			if(ST == NormalSpot){
				Set_Clean_Mode(Clean_Mode_Userinterface);
				wav_play(WAV_CLEANING_FINISHED);
			}
			break;
		}
		//Set_MainBrush_PWM(80);
		/*------------------------------------------------stop event-----------------------*/
		if (Stop_Event()) {
			Stop_Brifly();
			Disable_Motors();
			if(ST == NormalSpot){
				Set_Clean_Mode(Clean_Mode_Userinterface);
				//wav_play(WAV_CLEANING_FINISHED);
			}
			// Key release detection, if user has not release the key, don't do anything.
			while (Get_Key_Press() & KEY_CLEAN)
			{
				ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
				usleep(20000);
			}
			// Key relaesed, then the touch status and stop event status should be cleared.
			Reset_Stop_Event_Status();
			break;
		}
		uint8_t octype = Check_Motor_Current();
		if (octype) {
			if(Self_Check(octype) && (ST == NormalSpot)){
				Disable_Motors();
				Stop_Brifly();
				Set_Clean_Mode(Clean_Mode_Userinterface);
				break;
			}
		}
		if (Get_Rcon_Remote()) {
			if(ST == NormalSpot){
				if(Remote_Key(Remote_All)){
					if(Get_Rcon_Remote() == Remote_Home){
						Set_MoveWithRemote();
						SetHomeRemote();
					}
					Disable_Motors();
					Stop_Brifly();
					Reset_Rcon_Remote();
					wav_play(WAV_CLEANING_FINISHED);
					Set_Clean_Mode(Clean_Mode_Userinterface);
					return;
				}
			}
			else if(ST == CleanSpot || ST == WallSpot){
				if(Remote_Key(Remote_Home)){
					Reset_Rcon_Remote();
					Set_MoveWithRemote();
					SetHomeRemote();
					return;
				}
				else if(Remote_Key(Remote_Left | Remote_Right | Remote_Forward)){
					Reset_Rcon_Remote();	
					return;
				}
			}
		}
		/*--------------------Runing Path-----------------------*/
		Set_Dir_Forward();
		switch (Move_Style) {
			case Spiral_Right_Out:
				step = Get_LeftWheel_Step();
				if (step > (Radius * sn)) {
					Reset_LeftWheel_Step();
					if (Radius < 100) {
						Radius += 1;
					} else {
						Radius += 3;
					}
					if (Radius > ran) {
						Move_Style = Spiral_Right_In;
						ROS_INFO("%s ,%d ,SPIRAL RIGHT IN",__FUNCTION__,__LINE__);
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_RightWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
						if(Is_Bumper_Jamed())
						{
							return ;
						}
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Left(Turn_Speed, 2500);
					Move_Style = Spiral_Left_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + rp));
				break;

			case Spiral_Right_In:
				step = Get_LeftWheel_Step();
				if (step > (Radius * sn)) {
					Reset_LeftWheel_Step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 3;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {
					if (Get_RightWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
						if(Is_Bumper_Jamed())
						{
							return ;
						}
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Left(Turn_Speed, 2500);
					Move_Style = Spiral_Left_In;
					ROS_INFO("%s ,%d ,SPIRAL LEFT IN",__FUNCTION__,__LINE__);
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + rp));

				break;

			case Spiral_Left_Out:
				step = Get_RightWheel_Step();
				if (step > (Radius * sn)) {
					Reset_RightWheel_Step();
					if (Radius < 100) {
						Radius += 1;
					} else {
						Radius += 3;
					}
					if (Radius > ran) {
						Move_Style = Spiral_Left_In;
						ROS_INFO("%s ,%d ,SPIRAL LEFT IN",__FUNCTION__,__LINE__);
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
						if(Is_Bumper_Jamed())
						{
							return ;
						}
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_Out;
					ROS_INFO("%s ,%d ,SPIRAL RIGHT OUT",__FUNCTION__,__LINE__);
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + rp));

				break;

			case Spiral_Left_In:
				step = Get_RightWheel_Step();
				if (step > (Radius * sn)) {
					Reset_RightWheel_Step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 3;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
						if(Is_Bumper_Jamed())
						{
							return ;
						}
					}
					if(Get_Cliff_Trig()){
						Move_Back();
					}
					Stop_Brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_In;
					ROS_INFO("%s ,%d ,SPIRAL RIGHT IN",__FUNCTION__,__LINE__);
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + rp));

				break;

			default:
				break;
		}
		if ((OBS_Counter > 15) || (Stunk > 3)) {
			if(ST == NormalSpot)
				Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if (Spot_Flag) {
			Spot_Flag = 0;
			if(ST == NormalSpot)
				Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}

		if (Get_Cliff_Trig() == (Status_Cliff_All)) {
			Quick_Back(20,20);
			Stop_Brifly();
			if(Get_Cliff_Trig() == (Status_Cliff_All)){
				Quick_Back(20,20);
				Stop_Brifly();
			}
			if(Get_Cliff_Trig() == Status_Cliff_All){
				Quick_Back(20,20);
				Stop_Brifly();
				ROS_INFO("Cliff trigger three times ,robot lift up ");
				if(ST == NormalSpot)
					Set_Clean_Mode(Clean_Mode_Userinterface);
				Disable_Motors();
				wav_play(WAV_ERROR_LIFT_UP);
				break;
			}
			break;
		}
	}
	if(ST == NormalSpot)
		wav_play(WAV_CLEANING_FINISHED);
}
/*
* @author: mengshige1988@qq.com
* @brief: find the first nearest point
* @param1: reference point 
* @param2: pointer  to the point list
* @param3: pointing to the near_point 
* @return: void
* */
void Find_NearestPoint(Point32_t ref_point,std::list<Point32_t> *point_list,Point32_t *near_point)
{
	std::list<Point32_t>::const_iterator tp;
	Point32_t t_p;
	float dist=0.0;
	float mindist = 1.0;
	float t_dist = FLT_MAX;
	t_p = point_list->front();
	for(tp = point_list->begin() ; tp != point_list->end(); tp++){
		dist = sqrt( pow( ref_point.X - tp->X,2 ) + pow( ref_point.Y - tp->Y,2 ) );
		if(absolute(dist- mindist) <= 0.1){
			near_point->X = tp->X;
			near_point->Y = tp->Y;
			//ROS_WARN("%s,%d,find near point (%d,%d)",__FUNCTION__,__LINE__,tp->X,tp->Y);
			return;
		}
		if(dist<t_dist){
			t_dist = dist;
			t_p.X = tp->X;
			t_p.Y = tp->Y;
		}
	}

	//ROS_WARN("%s,%d,find near point (%d,%d)",__FUNCTION__,__LINE__,t_p.X,t_p.Y);
	near_point->X = t_p.X;
	near_point->Y = t_p.Y;
}
/*
* @author: mengshige1988@qq.com
* @brief: spot mode ,control robot rolling in rounding movement ,according to cell map .
* @param: SpotType.
*	SpotType: clean_spot  ,remote_spot  ,wall_spot. 
* @return : None
* */
void Spot_WithCell(SpotType st,float spot_radius){
	Reset_Stop_Event_Status();
	Reset_Rcon_Status();
    MapTouringType mt_state;
	std::list<Point32_t> target;
	Point32_t nextPoint;
	uint8_t spot_stuck = 0;

	/*--------initialize gyro & map & plan & slam --------*/
	if(st == NormalSpot){

		MotionManage motion;//start slam
		if(! motion.initSucceeded()){
			ROS_WARN("%s %d: Init MotionManage failed!", __FUNCTION__, __LINE__);
			Disable_Motors();
			return;
		}
		std::list<Point32_t>::const_iterator tp;
		uint8_t spiral_type;
		if((clock()/CLOCKS_PER_SEC) %2 == 0){
			spiral_type = Spiral_Left_Out;	
			ROS_INFO("%s %d ,spiral left out",__FUNCTION__,__LINE__);
		}
		else{
			spiral_type = Spiral_Right_Out;
			ROS_INFO("%s ,%d spiral right out",__FUNCTION__,__LINE__);
		}
		Point32_t StopPoint = {0,0};
		Point32_t nearPoint = {0,0};
		uint8_t Is_Dict_Change = 0;
		uint32_t tmp_coor;
		uint8_t od_spiral_out = 0,od_spiral_in = 0;
		while(ros::ok()){
			#ifdef OBS_DYNAMIC_MOVETOTARGET
				OBS_Dynamic_Base(20);
			#endif
			/*-------get target list ---------*/
			Spot_GetTarget(spiral_type, spot_radius, &target, 0, 0);
			if(Is_Dict_Change){
				Find_NearestPoint(StopPoint,&target,&nearPoint);
				StopPoint.X = 0;
				StopPoint.Y = 0;
			}
			/*--------iterator target ----------*/
			for( tp = target.begin(); tp != target.end(); tp++){
				usleep(20000);
				if(Spot_HandleException(st)){
					return;	
				}
				//ROS_WARN("%s ,%d target point (%d,%d),nearPoint.X = %d,nearPoint.Y = %d",__FUNCTION__,__LINE__,tp->X,tp->Y,nearPoint.X,nearPoint.Y);
				if(Is_Dict_Change){
					if((nearPoint.X == tp->X) && (nearPoint.Y == tp->Y)){
						Is_Dict_Change = 0;
						nearPoint.X = 0;
						nearPoint.Y = 0;
					}
					else{
						continue;
					}
				}
				/*----ready to spot movement ------*/

				nextPoint.X = cellToCount(tp->X);
				nextPoint.Y = cellToCount(tp->Y);
				if(! CM_LinearMoveToPoint(nextPoint,SPOT_MAX_SPEED,false,true))
					return;

				//if detect obs or bumper cliff trigger ,than change diraction
				if(g_should_follow_wall){
					g_should_follow_wall = 0;
					Is_Dict_Change = 1;
					//ROS_WARN("%s,%d,OBS or Bumper cliff detect",__FUNCTION__,__LINE__);
					if(spiral_type == Spiral_Right_Out){
						od_spiral_out += 1;
						if(od_spiral_out >= 3){
							od_spiral_out = 0;
							//ROS_WARN("%s ,%d ,set spiral type to left in",__FUNCTION__,__LINE__);
							spiral_type = Spiral_Left_In;
						}
						else{
							//ROS_WARN("%s ,%d ,set spiral type to left out",__FUNCTION__,__LINE__);
							spiral_type = Spiral_Left_Out;
						}
						if(tp == target.begin()){
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						else{
							tp--;
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,StopPoint.X,StopPoint.Y);
					}
					else if(spiral_type == Spiral_Left_Out){
						od_spiral_out += 1;
						if(od_spiral_out >= 3){
							od_spiral_out = 0;
							//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
							spiral_type = Spiral_Right_In;
						}
						else{
							spiral_type = Spiral_Right_Out;
							//ROS_WARN("%s ,%d ,set spiral type to right out",__FUNCTION__,__LINE__);
						}
						if(tp == target.begin()){
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						else{
							tp--;
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,StopPoint.X,StopPoint.Y);
					}
					else if(spiral_type == Spiral_Right_In){
						//ROS_WARN("%s ,%d ,set spiral type to left in",__FUNCTION__,__LINE__);
						spiral_type = Spiral_Left_In;
						od_spiral_in++;
						if(od_spiral_in > 3){
							Is_Dict_Change = 0;
							od_spiral_in =0;
							spot_stuck = 1;
							break;
						}
						if((tp->X == 0) && (tp->Y == 0)){
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						else{
							tp--;
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,StopPoint.X,StopPoint.Y);
					}
					else if(spiral_type == Spiral_Left_In){
						//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
						spiral_type = Spiral_Right_In;
						od_spiral_in++;
						if(od_spiral_in > 3){
							Is_Dict_Change = 0;
							od_spiral_in =0;
							spot_stuck = 1;
							break;
						}
						if((tp->X == 0) && (tp->Y == 0)){
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						else{
							tp--;
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,StopPoint.X,StopPoint.Y);
					}
					break;
				}//ending if(g_should_follow_wall)
			}//ending for(tp = target.begin();...)
			if(Is_Dict_Change){
				continue;
			} 
			if((spiral_type == Spiral_Right_In) || (spiral_type == Spiral_Left_In)){//spot done
				ROS_INFO("%s, %d, spot mode clean finishing",__FUNCTION__,__LINE__);
				if(spot_stuck){
					CM_LinearMoveToPoint(StopPoint,SPOT_MAX_SPEED,false,true);
					spot_stuck = 0;
				} 
				break;
			}
			else if(spiral_type == Spiral_Right_Out){
				//ROS_INFO("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
				spiral_type  = Spiral_Right_In;
			}
			else if(spiral_type == Spiral_Left_Out){
				//ROS_INFO("%s,%d,set spiral type to left in ",__FUNCTION__,__LINE__);
				spiral_type = Spiral_Left_In;
			}
		}//ending while(ros::ok)
	}//ending if(st == NormalSpot)
	else if(st == CleanSpot || st == WallSpot){
		
		Set_LED(100,0);
		Switch_VacMode(false);
		Set_MainBrush_PWM(80);
		Set_SideBrush_PWM(60,60);
		wav_play(WAV_CLEANING_SPOT);
		std::list<Point32_t>::const_iterator tp;
		uint8_t spiral_type;
		if((clock()/CLOCKS_PER_SEC) %2 == 0){
			spiral_type = Spiral_Left_Out;	
			ROS_INFO("%s %d ,spiral left out",__FUNCTION__,__LINE__);
		}
		else{
			spiral_type = Spiral_Right_Out;
			ROS_INFO("%s ,%d spiral right out",__FUNCTION__,__LINE__);
		}
		Point32_t StopPoint = {0,0};
		Point32_t nearPoint = {0,0};
		uint8_t Is_Dict_Change = 0;
		uint32_t tmp_coor;
		uint8_t od_spiral_out = 0,od_spiral_in = 0;
		int32_t x_offset = (int32_t)Map_GetXPos();
		int32_t y_offset = (int32_t)Map_GetYPos();
		while(ros::ok()){
			/*-------get target list ---------*/
			Spot_GetTarget(spiral_type,spot_radius,&target,x_offset,y_offset);
			if(Is_Dict_Change){
				Find_NearestPoint(StopPoint,&target,&nearPoint);
				StopPoint.X = 0;
				StopPoint.Y = 0;
			}
			/*--------iterator target ----------*/
			for(tp = target.begin(); tp!=target.end(); tp++){
				usleep(20000);
				if(Spot_HandleException(st)){
					return;	
				}
				//ROS_WARN("%s ,%d target point (%d,%d),StopPoint.X = %d,StopPoint.Y = %d",__FUNCTION__,__LINE__,tp->X,tp->Y,StopPoint.X,StopPoint.Y);
				if(Is_Dict_Change){
					if((nearPoint.X == tp->X) && (nearPoint.Y == tp->Y)){
						Is_Dict_Change = 0;
						nearPoint.X = 0;
						nearPoint.Y = 0;
					}
					else{
						continue;
					}
				}
				/*----ready to spot movement ------*/

				nextPoint.X = cellToCount(tp->X);
				nextPoint.Y = cellToCount(tp->Y);
				if(! CM_LinearMoveToPoint(nextPoint,SPOT_MAX_SPEED,false,true)) {
					Set_Clean_Mode(Clean_Mode_Userinterface);
					Disable_Motors();
					return;
				}
				//if detect obs or bumper cliff trigger ,than change diraction
				if(g_should_follow_wall){
					g_should_follow_wall = 0;
					Is_Dict_Change = 1;
					//ROS_WARN("%s,%d,OBS or Bumper or Cliff detect",__FUNCTION__,__LINE__);
					if(spiral_type == Spiral_Right_Out){
						od_spiral_out += 1;
						if(od_spiral_out >= 3){
							od_spiral_out = 0;
							//ROS_WARN("%s ,%d ,set spiral type to left in",__FUNCTION__,__LINE__);
							spiral_type = Spiral_Left_In;
						}
						else{
							//ROS_WARN("%s ,%d ,set spiral type to left out",__FUNCTION__,__LINE__);
							spiral_type = Spiral_Left_Out;
						}
						if(tp == target.begin()){
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						else{
							tp--;
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,StopPoint.X,StopPoint.Y);
					}
					else if(spiral_type == Spiral_Left_Out){
						od_spiral_out += 1;
						if(od_spiral_out >= 3){
							od_spiral_out = 0;
							//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
							spiral_type = Spiral_Right_In;
						}
						else{
							spiral_type = Spiral_Right_Out;
							//ROS_WARN("%s ,%d ,set spiral type to right out",__FUNCTION__,__LINE__);
						}
						if(tp == target.begin()){
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						else{
							tp--;
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,StopPoint.X,StopPoint.Y);
					}
					else if(spiral_type == Spiral_Right_In){
						//ROS_WARN("%s ,%d ,set spiral type to left in",__FUNCTION__,__LINE__);
						spiral_type = Spiral_Left_In;
						od_spiral_in++;
						if(od_spiral_in > 3){
							Is_Dict_Change = 0;
							od_spiral_in =0;
							spot_stuck = 1;
							break;
						}
						if((tp->X == 0) && (tp->Y == 0)){
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						else{
							tp--;
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,StopPoint.X,StopPoint.Y);
					}
					else if(spiral_type == Spiral_Left_In){
						//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
						spiral_type = Spiral_Right_In;
						od_spiral_in++;
						if(od_spiral_in > 3){
							Is_Dict_Change = 0;
							od_spiral_in =0;
							spot_stuck = 1;
							break;
						}
						if((tp->X == 0) && (tp->Y == 0)){
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						else{
							tp--;
							StopPoint.X = tp->X;
							StopPoint.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,StopPoint.X,StopPoint.Y);
					}
					break;
				}//ending if(g_should_follow_wall)	
			}//ending for(tp = target.begin();...)
			if(Is_Dict_Change){
				continue;
			} 
			if((spiral_type == Spiral_Right_In) || (spiral_type == Spiral_Left_In)){//spot done
				ROS_INFO("%s, %d, spot mode clean finishing",__FUNCTION__,__LINE__);
				StopPoint.X = x_offset;
				StopPoint.Y = y_offset;
				if(spot_stuck){
					CM_LinearMoveToPoint(StopPoint,SPOT_MAX_SPEED,false,true);
					spot_stuck = 0;
				} 
				break;
			}
			else if(spiral_type == Spiral_Right_Out){
				//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
				spiral_type  = Spiral_Right_In;
			}
			else if(spiral_type == Spiral_Left_Out){
				//ROS_WARN("%s,%d,set spiral type to left in ",__FUNCTION__,__LINE__);
				spiral_type = Spiral_Left_In;
			}
		}//ending while(ros::ok)
	}//ending if(st == Cleanspot...)
}
/*
 * author: mengshige1988@qq.com
 * @brief: calculate target points for spot move
 * @param1: sp_type
 *		sp_type;Spiral_Right_Out,Spiral_Left_Out,Spiral_Right_In,Spiral_Left_In.
 * @param2: radius
 *			spiral radius in meters
 * @param3: *target
 *			target list pointer
 * @return: None
 */
void Spot_GetTarget(uint8_t sp_type,float radiu,std::list<Point32_t> *target,int32_t x_off,int32_t y_off){
	uint8_t spt = sp_type;
	Point32_t CurPoint;
	int32_t x,x_l,y,y_l;
	x = x_l = x_off;
	y = y_l = y_off;
	target->clear();
	uint16_t st_c = 1;//number of spiral count
	uint16_t st_n = (uint16_t)(radiu*1000/CELL_SIZE);//number of spiral
	//ROS_INFO("%s,%d,number of spiral %d",__FUNCTION__,__LINE__,st_n);
	int16_t i;
	switch(spt){
		case Spiral_Left_Out:
			while(1){
				if(st_c>st_n){
					break;
				}
				if(st_c == 1){
					CurPoint.X = x;
					CurPoint.Y = y;
					target->push_back(CurPoint);
				}
				else if((st_c%2)==0){//even number
					x = x_l + 1;
					x_l = x;
					for(i = 0;i<st_c;i++){
						y = y_l + i;
						CurPoint.X = x;
						CurPoint.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(CurPoint);
					}
					for(i=0;i<(st_c - 1);i++){
						x = x_l - i - 1;
						CurPoint.X = x;
						CurPoint.Y = y;
						//target->push_back(CurPoint);
					}
				}
				else{//odd number
					x = x_l - 1;
					x_l = x;
					for(i=0;i<st_c;i++){
						y = y_l - i;
						CurPoint.X = x;
						CurPoint.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(CurPoint);
					}
					for(i=0;i<(st_c-1);i++){
						x = x_l + i + 1;
						CurPoint.X = x;
						CurPoint.Y = y;
						//target->push_back(CurPoint);
					}
				}
				x_l = x;
				y_l = y;
				st_c+=1;
			}
			target->push_back(CurPoint);
			break;
		case Spiral_Right_Out:
			while(1){
				if(st_c>st_n){
					break;
				}
				if(st_c == 1){
					CurPoint.X = x;
					CurPoint.Y = y;
					target->push_back(CurPoint);
				}
				else if((st_c%2)==0){//even number
					x = x_l + 1;
					x_l = x;
					for(i = 0;i<st_c;i++){
						y = y_l - i;
						CurPoint.X = x;
						CurPoint.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(CurPoint);
					}
					for(i=0;i<(st_c - 1);i++){
						x = x_l - i - 1;
						CurPoint.X = x;
						CurPoint.Y = y;
						//target->push_back(CurPoint);
					}
				}
				else{//odd number
					x = x_l - 1;
					x_l = x;
					for(i=0;i < st_c;i++){
						y = y_l + i;
						CurPoint.X = x;
						CurPoint.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(CurPoint);
					}
					for(i=0;i<(st_c-1);i++){
						x = x_l + i + 1;
						CurPoint.X = x;
						CurPoint.Y = y;
						//target->push_back(CurPoint);
					}
				}
				x_l = x;
				y_l = y;
				st_c+=1;
			}
			target->push_back(CurPoint);
			break;
		case Spiral_Right_In:
			while(1){
				if(st_c>st_n){
					break;
				}
				if(st_c == 1){
					CurPoint.X = x;
					CurPoint.Y = y;
					target->push_back(CurPoint);
				}
				else if((st_c%2)==0){//even number
					y = y_l + 1;
					y_l = y;
					for(i = 0;i < st_c;i++){
						x = x_l - i;
						CurPoint.X = x;
						CurPoint.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(CurPoint);
					}
					for(i = 0;i < (st_c -1);i++){
						y = y_l - i -1;
						CurPoint.X = x;
						CurPoint.Y = y;
						//target->push_back(CurPoint);
					}
				}
				else{ //odd number
					y = y_l -1;
					y_l = y;
					for(i = 0;i<st_c;i++){
						x = x_l + i;
						CurPoint.X = x;
						CurPoint.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(CurPoint);
					}
					for(i = 0;i<(st_c -1 );i++){
						y = y_l + i + 1;
						CurPoint.X = x;
						CurPoint.Y = y;
						//target->push_back(CurPoint);
					}
				}
				x_l = x;
				y_l = y;
				st_c += 1;
			}
			target->push_back(CurPoint);
			target->reverse();
			break;
		case Spiral_Left_In:
			while(1){
				if(st_c>st_n){
					break;
				}
				if(st_c == 1){
					CurPoint.X = x;
					CurPoint.Y = y;
					target->push_back(CurPoint);
				}
				else if((st_c%2)==0){//even number
					y = y_l - 1;
					y_l = y;
					for(i = 0;i < st_c;i++){
						x = x_l - i;
						CurPoint.X = x;
						CurPoint.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(CurPoint);
					}
					for(i=0;i<(st_c - 1);i++){
						y = y_l + i + 1;
						CurPoint.X = x;
						CurPoint.Y = y;
						//target->push_back(CurPoint);
					}
				}
				else{//odd number
					y = y_l + 1;
					y_l = y;
					for(i=0;i<st_c;i++){
						x = x_l + i;
						CurPoint.X = x;
						CurPoint.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(CurPoint);
					}
					for(i=0;i<(st_c-1);i++){
						y = y_l -i - 1;
						CurPoint.X = x;
						CurPoint.Y = y;
						//target->push_back(CurPoint);
					}
				}
				x_l = x;
				y_l = y;
				st_c+=1;
			}
			target->push_back(CurPoint);
			target->reverse();
			break;
		default:
			break;

    }
}
/*
* @author: mengshige1988@qq.com
* @brief: spot handle exception
* @param: spot type (NormalSpot,CleanSpot,WallSpot)
* @return: 1 for exception occur , 0 for not occur
*/
int8_t Spot_HandleException(SpotType st)
{
	/*-----detect stop event (remote_clean ,key press) ------*/
	if (Stop_Event()) {
		Stop_Brifly();
		Disable_Motors();
		while (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: key pressing.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
		if(st == NormalSpot){
			Set_Clean_Mode(Clean_Mode_Userinterface);
		}
		Reset_Stop_Event_Status();
		return 1;
	}
    uint8_t octype = Check_Motor_Current();
	if (octype) {
		if(Self_Check(octype) && (st == NormalSpot)){
			return 1;
		}
		return 1;
	}
	/*--------- cliff detect event--------------*/
	if (Get_Cliff_Trig() == (Status_Cliff_All)) {
		Quick_Back(20,20);//1 time
		Stop_Brifly();
		if(Get_Cliff_Trig() == (Status_Cliff_All)){
			Quick_Back(20,20);//2 times
			Stop_Brifly();
		}
		if(Get_Cliff_Trig() == Status_Cliff_All){
			Quick_Back(20,20);//3 times
			Stop_Brifly();
			ROS_INFO("%s %d, Cliff trigger three times ,robot lift up ",__FUNCTION__,__LINE__);
			if(st == NormalSpot)
				Set_Clean_Mode(Clean_Mode_Userinterface);
			Disable_Motors();
			//wav_play(WAV_ERROR_LIFT_UP);
			return 1;
		}
	}
	/*--------get remote event ---------------*/
	if (Get_Rcon_Remote()) {
		if(st == NormalSpot){
			if(Remote_Key(Remote_All)){
				if(Get_Rcon_Remote() == Remote_Home){
					Set_MoveWithRemote();
					SetHomeRemote();
				}
				Reset_Rcon_Remote();
				return 1;
			}
		}
		else if(st == CleanSpot || st == WallSpot){
			if(Remote_Key(Remote_Home)){
				Reset_Rcon_Remote();
				Set_MoveWithRemote();
				SetHomeRemote();
				return 1;
			}
			else if(Remote_Key(Remote_Left | Remote_Right | Remote_Forward)){
					Reset_Rcon_Remote();
				return 1;
			}
		}
	}
}
/*----------------------------------------------------------------Random Dirt Event---------------------------------*/
 
uint8_t Random_Dirt_Event(void)
{
	uint16_t Radius = 0;
	uint8_t Move_Style = 1;
	uint8_t Spot_Flag = 0;
	uint8_t OBS_Counter = 0;
	uint8_t Stunk = 0;
	uint8_t Flash_Counter = 0;
	uint16_t Watch_Counter = 0;
	uint8_t Flash_Flag = 0;
	uint8_t Motor_OC_Counter = 0;


	Move_Style = First_Round;

	Reset_Stop_Event_Status();

	Check_Bat_SetMotors(135000, 100000, 100000);

#ifdef BLDC_INSTALL
	Set_VacMode(Vac_Max);
	Set_Vac_Speed();
#endif

	Move_Forward(0, 0);
	Reset_Wheel_Step();
	Reset_Wall_Step();
	usleep(10000);

	Reset_Rcon_Remote();

	Motor_OC_Counter = 0;
	while (ros::ok()) {
		usleep(10000);
		Flash_Counter++;
		if (Flash_Counter > 20) {
			Watch_Counter++;
			if (Watch_Counter > 1000) {
				Set_Touch();
				return 1;
			}
			Flash_Counter = 0;
			Flash_Flag = 1 - Flash_Flag;
			if (Flash_Flag) {
				/*do led flash */
			} else {
				/*do led flash*/
			}
		}

		if (Remote_Key(Remote_All)) {
			//Main_Brush_PWM = MainBrush_Power;
			Move_Forward(30, 30);
			Reset_Rcon_Remote();
			return 0;
		}

		/*------------------------------------------------------Virtual Wall-----------------------*/
#ifdef VIRTUAL_WALL



#endif

		/*------------------------------------------------------Check Battery-----------------------*/
		if (Check_Bat_SetMotors(135000, 100000, 120000)) {	//Low Battery Event
			Move_Forward(30, 30);
			return 0;
		}

		if (Check_Motor_Current()) {
			Motor_OC_Counter++;
			if (Motor_OC_Counter > 50) {
				Motor_OC_Counter = 0;
				//Main_Brush_PWM = MainBrush_Power;
				Move_Forward(30, 30);
				return 0;
			}
		} else {
			Motor_OC_Counter = 0;
		}
		/*------------------------------------------------------stop event-----------------------*/
		if (Stop_Event()) {
			Stop_Brifly();
			ROS_INFO("%s %d: Stop event!", __FUNCTION__, __LINE__);
			return 1;
		}
		/*------------------------------------------------------Runing Path-----------------------*/

		switch (Move_Style) {
			case First_Round:
				if (Get_LeftWheel_Step() > 6000) {
					Move_Forward(0, 0);
					Reset_LeftWheel_Step();
					Move_Style = Spiral_Right_Out;
				}
				Set_Dir_Right();
				Set_Wheel_Speed(25, 10);

				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Left(Turn_Speed, 2500);
					Move_Forward(10, 10);
					Move_Style = Spiral_Left_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				break;

			case Spiral_Right_Out:
				if (Get_LeftWheel_Step() > (Radius * 3)) {
					Reset_LeftWheel_Step();
					if (Radius < 100) {
						Radius += 2;
					} else {
						Radius += 6;
					}
					if (Radius > 140) {
						Move_Style = Spiral_Right_In;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Left(Turn_Speed, 2500);
					Move_Style = Spiral_Left_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_Dir_Forward();
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case Spiral_Right_In:
				if (Get_LeftWheel_Step() > (Radius * 3)) {
					Reset_LeftWheel_Step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 6;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Left(Turn_Speed, 2500);
					Move_Style = Spiral_Left_In;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_Dir_Forward();
				Set_LeftWheel_Speed(SPOT_MAX_SPEED);
				Set_RightWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case Spiral_Left_Out:
				if (Get_RightWheel_Step() > (Radius * 3)) {
					Reset_RightWheel_Step();
					if (Radius < 100) {
						Radius += 2;
					} else {
						Radius += 6;
					}
					if (Radius > 140) {
						Move_Style = Spiral_Left_In;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_Out;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_Dir_Forward();
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case Spiral_Left_In:
				if (Get_RightWheel_Step() > (Radius * 2)) {
					Reset_RightWheel_Step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 6;
					}
				}
				if (Get_Bumper_Status() || Get_Cliff_Trig() || Spot_OBS_Status()) {

					if (Get_LeftWall_Step() < 3000) {
						Stunk++;
					}
					if (Get_Bumper_Status()) {
						Random_Back();
					} else if (Get_Cliff_Trig()) {
						Move_Back();
					}
					Stop_Brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = Spiral_Right_In;
					Reset_Wheel_Step();
					Reset_Wall_Step();
					OBS_Counter++;
				}
				Set_Dir_Forward();
				Set_RightWheel_Speed(SPOT_MAX_SPEED);
				Set_LeftWheel_Speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			default:
				break;

		}
		if ((OBS_Counter > 5) || (Stunk > 2)) {
			//Main_Brush_PWM = MainBrush_Power;
			Move_Forward(30, 30);
			return 0;
		}
		if (Spot_Flag) {
			Spot_Flag = 0;
			//Main_Brush_PWM = MainBrush_Power;
			Move_Forward(30, 30);
			return 2;
		}
	}
	//return 2;
}

