#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>

#include "movement.h"
#include "gotocharger.hpp"
#include "robot.hpp"

#define GOTO_CHARGER "goto charger"
#define GOTO_CHARGER_TIMEOUT 180 //3 minutes

void goto_charger(){
	uint8_t fl,fr,l,r,bl,br;
	int16_t fobs;
	uint32_t rcon;
	uint8_t stub_l=0x04;
	uint8_t stub_r=0x01;
	uint8_t stub_t=0x02;
	bool set_charge_state = false;
	uint8_t in_charge_stub_move = 1;
	bool left_bumper,right_bumper;
	uint8_t brush_pwr;
	bool on_charger_stub= false;
	Reset_Wheel_Step();
	Set_Wheel_Speed(0,0);
	ros::Time startTime;
	startTime = ros::Time::now();
	while(ros::ok()){
		usleep(10000);
		if((ros::Time::now()-startTime).toSec()>GOTO_CHARGER_TIMEOUT){
			control_set(CTL_CHARGER,0x00);
			Set_CleanTool_Power(0,0,0,0);
			ROS_DEBUG_NAMED(GOTO_CHARGER,"goto charger timeout!!!");
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if(robot::instance()->robot_get_charge_status()){
			movement_go(0);
			on_charger_stub= true;	
			ROS_DEBUG_NAMED(GOTO_CHARGER,"on charger stub");
			Set_CleanTool_Power(0,0,0,0);
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if(Get_Clean_Mode() != Clean_Mode_Charging){
			control_set(CTL_CHARGER,0x00);
			Set_CleanTool_Power(0,0,0,0);
			ROS_DEBUG_NAMED(GOTO_CHARGER,"jump out go_home_mode");
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}
		if(!set_charge_state){
			set_charge_state = true;
			ROS_DEBUG_NAMED(GOTO_CHARGER,"goto charger");
			Set_Clean_Mode(Clean_Mode_Userinterface);
			control_set(CTL_CHARGER, 0x01);
		}
		
		if(Get_Rcon_Remote())
		{
			Set_Clean_Mode(Clean_Mode_Userinterface);
			break;
		}	
		fobs = robot::instance()->robot_get_obs_front();
		rcon = robot::instance()->robot_get_rcon();
		if(fobs>=100)
			brush_pwr=0;
		else
			brush_pwr=20;
		if(rcon>0){
			Set_CleanTool_Power(brush_pwr,brush_pwr,brush_pwr,brush_pwr);
			fl = ((rcon&0x00f00000)>>20);
			fr = ((rcon&0x000f0000)>>16);
			l  = ((rcon&0x0000f000)>>12);
			r  = ((rcon&0x00000f00)>>8);
			bl = ((rcon&0x000000f0)>>4);
			br = ( rcon&0x0000000f);

			left_bumper = robot::instance()->robot_get_bumper_left();
			right_bumper = robot::instance()->robot_get_bumper_right();
			if(left_bumper|| right_bumper){
				if(left_bumper&&(fl&stub_l||fr&stub_r)){
					movement_go(-150);
					usleep(500000);
					movement_turn(-220,-80);
					usleep(700000);
				}
				else if(right_bumper&&(fl&stub_l||fr&stub_r)){
					movement_go(-150);
					usleep(500000);
					movement_turn(-80,-220);
					usleep(700000);		
				}
				else if(left_bumper){
					movement_go(-150);
					usleep(500000);
					movement_turn(-200,-10);
					usleep(1300000);
				}	
				else if(right_bumper){
					movement_go(-150);
					usleep(500000);
					movement_turn(-10,-200);
					usleep(1300000);
				}
				else if(right_bumper&&left_bumper){
					movement_go(-100);
					usleep(700000);
				}
			}
			else if((fl|fr|l|r|bl|br)&0x05){
				if(fl & stub_l || fr & stub_r){
					if((fl & stub_r) && (fr & stub_r))
						if(fobs>=100)
							movement_go(40);
						else
							movement_go(100);
					else if((fl & stub_l) && !(fr & stub_r))
						if(	fobs>=100)
							movement_turn(-30,80);
						else
							movement_turn(80,180);
					else if(!(fl & stub_l) && (fr &stub_r))
						if(fobs>=100)
							movement_turn(80,-30);	
						else
							movement_turn(180,80);
				}
				else if(l&stub_l || l&stub_r || bl&stub_l || bl&stub_r)
					movement_rot_left(80);
				else if(r&stub_l || r&stub_r || br&stub_r || br&stub_l)
					movement_rot_right(80);
			}else {	
				if(fl&stub_t&&fr&stub_t)
					if(fobs>=100)
						movement_go(50);
					else
						movement_go(100);
				else if(l&stub_t)
					movement_rot_left(50);
				else if(r&stub_t)
					movement_rot_right(50);	
				else if(bl& stub_t) 
					movement_rot_left(50);
				else if(br&stub_t)
					movement_rot_right(50);

			}
		}
	}
	
}
