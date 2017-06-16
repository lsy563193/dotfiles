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
#include "event_manager.h"
#include "slam.h"
#include <ros/ros.h>
#include <time.h>
#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed	18
#endif

#define SPOT_MAX_SPEED	(20)
extern uint8_t g_should_follow_wall;//used to decide obstacle 
extern bool g_remote_spot;//spot event handler variable
extern bool g_remote_home;
extern bool g_fatal_quit_event;
extern bool g_key_clean_pressed;
/*
* @author mengshige1988@qq.com
* @brief find the first nearest point.
* @param1 reference point.
* @param2 pointer  to the point list.
* @param3 pointing to the near_point.
* @return None.
* */
static uint8_t find_nearest_point(Point32_t ref_point,std::list<Point32_t> *point_list,Point32_t *near_point)
{
	std::list<Point32_t>::reverse_iterator tp;
	Point32_t t_p;
	float dist=0.0,dist_last;
	float mindist = 1.0;
	float t_dist = FLT_MAX;
	t_p = point_list->front();
	for(tp = point_list->rbegin(); tp != point_list->rend(); tp++){
		dist = sqrt( pow( ref_point.X - tp->X,2 ) + pow( ref_point.Y - tp->Y,2 ) );
		if(absolute(dist- mindist) <= 0.1){
			near_point->X = tp->X;
			near_point->Y = tp->Y;
			//ROS_WARN("%s,%d,find near point (%d,%d)",__FUNCTION__,__LINE__,tp->X,tp->Y);
			return 1;
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
    return 1;
}
/*
* @author mengshige1988@qq.com
* @brief spot mode ,control robot rolling in rounding movement ,according to cell map .
* @param1 SpotType: clean_spot  ,remote_spot  ,wall_spot. 
* @param2 spot_diameter ,the spiral diameter in meters
* @return None
* */
void spot_with_cell(SpotType st,float spot_diameter)
{
	reset_stop_event_status();
	reset_rcon_status();
	std::list<Point32_t> target;
	uint8_t spot_stuck = 0;
	g_remote_spot = false;
	/*--------initialize gyro & map & plan & slam --------*/
	if(st == NORMAL_SPOT){
		g_remote_spot = g_remote_home = g_fatal_quit_event = g_key_clean_pressed = false;
		MotionManage motion;//start slam
		if(! motion.initSucceeded()){
			ROS_WARN("%s %d: Init MotionManage failed!", __FUNCTION__, __LINE__);
			disable_motors();
			return;
		}
		cm_register_events();
		std::list<Point32_t>::const_iterator tp;
		uint8_t spiral_type;
		if((clock()/CLOCKS_PER_SEC) %2 == 0){
			spiral_type = SPIRAL_LEFT_OUT;	
			ROS_INFO("%s %d ,spiral left out",__FUNCTION__,__LINE__);
		}
		else{
			spiral_type = SPIRAL_RIGHT_OUT;
			ROS_INFO("%s ,%d spiral right out",__FUNCTION__,__LINE__);
		}
		Point32_t stop_point = {0,0};
		Point32_t near_point = {0,0};
		uint8_t is_dirct_change = 0;
		uint32_t tmp_coor;
		uint8_t od_spiral_out = 0,od_spiral_in = 0;
		while(ros::ok()){
			/*-------get target list ---------*/
			gen_spot_target(spiral_type, spot_diameter, &target, 0, 0);
			if(is_dirct_change){
				find_nearest_point(stop_point,&target,&near_point);
				stop_point.X = 0;
				stop_point.Y = 0;
			}
			/*--------iterator target ----------*/
			for( tp = target.begin(); tp != target.end(); tp++){
				usleep(20000);
				//if(Spot_HandleException(st)){
				//	return;	
				//}
				//ROS_WARN("%s ,%d target point (%d,%d),near_point.X = %d,near_point.Y = %d",__FUNCTION__,__LINE__,tp->X,tp->Y,near_point.X,near_point.Y);
				if(is_dirct_change){
					if((near_point.X == tp->X) && (near_point.Y == tp->Y)){
						is_dirct_change = 0;
						near_point.X = 0;
						near_point.Y = 0;
					}
					else{
						continue;
					}
				}

				/*----ready to spot movement ------*/

				cm_linear_move_to_point({cell_to_count(tp->X),cell_to_count(tp->Y)},SPOT_MAX_SPEED,false,true);
 
	            if(g_fatal_quit_event ||  g_key_clean_pressed ||  g_remote_spot || g_remote_home){
		            robot::instance()->resetManualPause();
                    g_fatal_quit_event = g_key_clean_pressed = g_remote_home = g_remote_spot = false;
                    return;
                }

				//if detect obs or bumper or  cliff trigger ,than change diraction
				if(g_should_follow_wall){
					g_should_follow_wall = 0;
					is_dirct_change = 1;
					//ROS_WARN("%s,%d,OBS or Bumper cliff detect",__FUNCTION__,__LINE__);
					if(spiral_type == SPIRAL_RIGHT_OUT){
						od_spiral_out += 1;
						if(od_spiral_out >= 3){
							od_spiral_out = 0;
							//ROS_WARN("%s ,%d ,set spiral type to left in",__FUNCTION__,__LINE__);
							spiral_type = SPIRAL_LEFT_IN;
						}
						else{
							//ROS_WARN("%s ,%d ,set spiral type to left out",__FUNCTION__,__LINE__);
							spiral_type = SPIRAL_LEFT_OUT;
						}
						if(tp == target.begin()){
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						else{
							tp--;
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,stop_point.X,stop_point.Y);
					}
					else if(spiral_type == SPIRAL_LEFT_OUT){
						od_spiral_out += 1;
						if(od_spiral_out >= 3){
							od_spiral_out = 0;
							//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
							spiral_type = SPIRAL_RIGHT_IN;
						}
						else{
							spiral_type = SPIRAL_RIGHT_OUT;
							//ROS_WARN("%s ,%d ,set spiral type to right out",__FUNCTION__,__LINE__);
						}
						if(tp == target.begin()){
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						else{
							tp--;
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,stop_point.X,stop_point.Y);
					}
					else if(spiral_type == SPIRAL_RIGHT_IN){
						//ROS_WARN("%s ,%d ,set spiral type to left in",__FUNCTION__,__LINE__);
						spiral_type = SPIRAL_LEFT_IN;
						od_spiral_in++;
						if(od_spiral_in > 3){
							is_dirct_change = 0;
							od_spiral_in =0;
							spot_stuck = 1;
							break;
						}
						if(tp == target.begin()){
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						else{
							tp--;
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,stop_point.X,stop_point.Y);
					}
					else if(spiral_type == SPIRAL_LEFT_IN){
						//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
						spiral_type = SPIRAL_RIGHT_IN;
						od_spiral_in++;
						if(od_spiral_in > 3){
							is_dirct_change = 0;
							od_spiral_in =0;
							spot_stuck = 1;
							break;
						}
						if(tp == target.begin()){
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						else{
							tp--;
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,stop_point.X,stop_point.Y);
					}
                    break;//break for loop
				}//ending if(g_should_follow_wall)
			}//ending for(tp = target.begin();...)
			if(is_dirct_change){
				continue;
			} 
			if((spiral_type == SPIRAL_RIGHT_IN) || (spiral_type == SPIRAL_LEFT_IN)){//spot done
				ROS_INFO("%s, %d, spot mode clean finishing",__FUNCTION__,__LINE__);
				if(spot_stuck){
					//go back to start point
					cm_linear_move_to_point(stop_point, SPOT_MAX_SPEED, false, true);
					spot_stuck = 0;
				} 
				break;
			}
			else if(spiral_type == SPIRAL_RIGHT_OUT){
				//ROS_INFO("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
				spiral_type  = SPIRAL_RIGHT_IN;
			}
			else if(spiral_type == SPIRAL_LEFT_OUT){
				//ROS_INFO("%s,%d,set spiral type to left in ",__FUNCTION__,__LINE__);
				spiral_type = SPIRAL_LEFT_IN;
			}
		}//ending while(ros::ok)
		cm_unregister_events();
	}//ending if(st == NORMAL_SPOT)
	else if(st == CLEAN_SPOT || st == WALL_SPOT){
		event_manager_enable_handler(EVT_REMOTE_MODE_SPOT,false);
		set_led(100, 0);
		switch_vac_mode(false);
		set_main_brush_pwm(80);
		set_side_brush_pwm(60, 60);
		wav_play(WAV_CLEANING_SPOT);
		std::list<Point32_t>::const_iterator tp;
		uint8_t spiral_type;
		if((clock()/CLOCKS_PER_SEC) %2 == 0){
			spiral_type = SPIRAL_LEFT_OUT;	
			ROS_INFO("%s %d ,spiral left out",__FUNCTION__,__LINE__);
		}
		else{
			spiral_type = SPIRAL_RIGHT_OUT;
			ROS_INFO("%s ,%d spiral right out",__FUNCTION__,__LINE__);
		}
		Point32_t stop_point = {0,0};
		Point32_t near_point = {0,0};
		uint8_t is_dirct_change = 0;
		uint32_t tmp_coor;
		uint8_t od_spiral_out = 0,od_spiral_in = 0;
		int32_t x_offset = (int32_t) map_get_x_cell();
		int32_t y_offset = (int32_t) map_get_y_cell();
		while(ros::ok()){
			/*-------get target list ---------*/
			gen_spot_target(spiral_type,spot_diameter,&target,x_offset,y_offset);
			if(is_dirct_change){
				find_nearest_point(stop_point,&target,&near_point);
				stop_point.X = 0;
				stop_point.Y = 0;
			}
			/*--------iterator target ----------*/
			for(tp = target.begin(); tp!=target.end(); tp++){
				usleep(20000);
				//if(Spot_HandleException(st)){
				//	return;	
				//}
				//ROS_WARN("%s ,%d target point (%d,%d),stop_point.X = %d,stop_point.Y = %d",__FUNCTION__,__LINE__,tp->X,tp->Y,stop_point.X,stop_point.Y);
				if(is_dirct_change){
					if((near_point.X == tp->X) && (near_point.Y == tp->Y)){
						is_dirct_change = 0;
						near_point.X = 0;
						near_point.Y = 0;
					}
					else{
						continue;
					}
				}
				/*----ready to spot movement ------*/
				cm_linear_move_to_point({cell_to_count(tp->X),cell_to_count(tp->Y)},SPOT_MAX_SPEED,false,true);
                if(g_fatal_quit_event ||  g_key_clean_pressed || g_remote_home){
									disable_motors();
                    return;
                }

				//if detect obs or bumper cliff trigger ,than change diraction
				if(g_should_follow_wall){
					g_should_follow_wall = 0;
					is_dirct_change = 1;
					//ROS_WARN("%s,%d,OBS or Bumper or Cliff detect",__FUNCTION__,__LINE__);
					if(spiral_type == SPIRAL_RIGHT_OUT){
						od_spiral_out += 1;
						if(od_spiral_out >= 3){
							od_spiral_out = 0;
							//ROS_WARN("%s ,%d ,set spiral type to left in",__FUNCTION__,__LINE__);
							spiral_type = SPIRAL_LEFT_IN;
						}
						else{
							//ROS_WARN("%s ,%d ,set spiral type to left out",__FUNCTION__,__LINE__);
							spiral_type = SPIRAL_LEFT_OUT;
						}
						if(tp == target.begin()){
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						else{
							tp--;
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,stop_point.X,stop_point.Y);
					}
					else if(spiral_type == SPIRAL_LEFT_OUT){
						od_spiral_out += 1;
						if(od_spiral_out >= 3){
							od_spiral_out = 0;
							//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
							spiral_type = SPIRAL_RIGHT_IN;
						}
						else{
							spiral_type = SPIRAL_RIGHT_OUT;
							//ROS_WARN("%s ,%d ,set spiral type to right out",__FUNCTION__,__LINE__);
						}
						if(tp == target.begin()){
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						else{
							tp--;
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,stop_point.X,stop_point.Y);
					}
					else if(spiral_type == SPIRAL_RIGHT_IN){
						//ROS_WARN("%s ,%d ,set spiral type to left in",__FUNCTION__,__LINE__);
						spiral_type = SPIRAL_LEFT_IN;
						od_spiral_in++;
						if(od_spiral_in > 3){
							is_dirct_change = 0;
							od_spiral_in =0;
							spot_stuck = 1;
							break;
						}
						if(tp ==  target.begin()){
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						else{
							tp--;
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,stop_point.X,stop_point.Y);
					}
					else if(spiral_type == SPIRAL_LEFT_IN){
						//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
						spiral_type = SPIRAL_RIGHT_IN;
						od_spiral_in++;
						if(od_spiral_in > 3){
							is_dirct_change = 0;
							od_spiral_in =0;
							spot_stuck = 1;
							break;
						}
						if(tp == target.begin()){
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						else{
							tp--;
							stop_point.X = tp->X;
							stop_point.Y = tp->Y;
						}
						//ROS_WARN("%s,%d,stop point (%d,%d)",__FUNCTION__,__LINE__,stop_point.X,stop_point.Y);
					}
					break;
				}//ending if(g_should_follow_wall)	
			}//ending for(tp = target.begin();...)
			if(is_dirct_change){
				continue;
			} 
			if((spiral_type == SPIRAL_RIGHT_IN) || (spiral_type == SPIRAL_LEFT_IN)){//spot done
				ROS_INFO("%s, %d, spot mode clean finishing",__FUNCTION__,__LINE__);
				stop_point.X = x_offset;
				stop_point.Y = y_offset;
				if(spot_stuck){
                    //go back to start point
					cm_linear_move_to_point(stop_point, SPOT_MAX_SPEED, false, true);
					spot_stuck = 0;
				} 
				break;
			}
			else if(spiral_type == SPIRAL_RIGHT_OUT){
				//ROS_WARN("%s ,%d ,set spiral type to right in",__FUNCTION__,__LINE__);
				spiral_type  = SPIRAL_RIGHT_IN;
			}
			else if(spiral_type == SPIRAL_LEFT_OUT){
				//ROS_WARN("%s,%d,set spiral type to left in ",__FUNCTION__,__LINE__);
				spiral_type = SPIRAL_LEFT_IN;
			}
		}//ending while(ros::ok)
	event_manager_enable_handler(EVT_REMOTE_MODE_SPOT,true);
	}//ending if(st == CLEAN_SPOT...)
}
/*
 * @author mengshige1988@qq.com
 * @brief calculate target points for spot move
 * @param1 sp_type
 *		sp_type;SPIRAL_RIGHT_OUT,SPIRAL_LEFT_OUT,SPIRAL_RIGHT_OUT,SPIRAL_LEFT_IN.
 * @param2 diameters
 *			spiral diameters in meters
 * @param3 *target
 *			target list pointer
 * @return None
 */
void gen_spot_target(uint8_t sp_type,float diameter,std::list<Point32_t> *target,int32_t x_off,int32_t y_off)
{
	uint8_t spt = sp_type;
	Point32_t cur_point;
	int32_t x,x_l,y,y_l;
	x = x_l = x_off;
	y = y_l = y_off;
	target->clear();
	uint16_t st_c = 1;//number of spiral count
	uint16_t st_n = (uint16_t)(diameter*1000/CELL_SIZE);//number of spiral
	//ROS_INFO("%s,%d,number of spiral %d",__FUNCTION__,__LINE__,st_n);
	int16_t i;
	switch(spt){
		case SPIRAL_LEFT_OUT:
			while(1){
				if(st_c>st_n){
					break;
				}
				if(st_c == 1){
					cur_point.X = x;
					cur_point.Y = y;
					target->push_back(cur_point);
				}
				else if((st_c%2)==0){//even number
					x = x_l + 1;
					x_l = x;
					for(i = 0;i<st_c;i++){
						y = y_l + i;
						cur_point.X = x;
						cur_point.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(cur_point);
					}
					for(i=0;i<(st_c - 1);i++){
						x = x_l - i - 1;
						cur_point.X = x;
						cur_point.Y = y;
						//target->push_back(cur_point);
					}
				}
				else{//odd number
					x = x_l - 1;
					x_l = x;
					for(i=0;i<st_c;i++){
						y = y_l - i;
						cur_point.X = x;
						cur_point.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(cur_point);
					}
					for(i=0;i<(st_c-1);i++){
						x = x_l + i + 1;
						cur_point.X = x;
						cur_point.Y = y;
						//target->push_back(cur_point);
					}
				}
				x_l = x;
				y_l = y;
				st_c+=1;
			}
			target->push_back(cur_point);
			break;
		case SPIRAL_RIGHT_OUT:
			while(1){
				if(st_c>st_n){
					break;
				}
				if(st_c == 1){
					cur_point.X = x;
					cur_point.Y = y;
					target->push_back(cur_point);
				}
				else if((st_c%2)==0){//even number
					x = x_l + 1;
					x_l = x;
					for(i = 0;i<st_c;i++){
						y = y_l - i;
						cur_point.X = x;
						cur_point.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(cur_point);
					}
					for(i=0;i<(st_c - 1);i++){
						x = x_l - i - 1;
						cur_point.X = x;
						cur_point.Y = y;
						//target->push_back(cur_point);
					}
				}
				else{//odd number
					x = x_l - 1;
					x_l = x;
					for(i=0;i < st_c;i++){
						y = y_l + i;
						cur_point.X = x;
						cur_point.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(cur_point);
					}
					for(i=0;i<(st_c-1);i++){
						x = x_l + i + 1;
						cur_point.X = x;
						cur_point.Y = y;
						//target->push_back(cur_point);
					}
				}
				x_l = x;
				y_l = y;
				st_c+=1;
			}
			target->push_back(cur_point);
			break;
        case SPIRAL_RIGHT_IN:
			while(1){
				if(st_c>st_n){
					break;
				}
				if(st_c == 1){
					cur_point.X = x;
					cur_point.Y = y;
					target->push_back(cur_point);
				}
				else if((st_c%2)==0){//even number
					y = y_l + 1;
					y_l = y;
					for(i = 0;i < st_c;i++){
						x = x_l - i;
						cur_point.X = x;
						cur_point.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(cur_point);
					}
					for(i = 0;i < (st_c -1);i++){
						y = y_l - i -1;
						cur_point.X = x;
						cur_point.Y = y;
						//target->push_back(cur_point);
					}
				}
				else{ //odd number
					y = y_l -1;
					y_l = y;
					for(i = 0;i<st_c;i++){
						x = x_l + i;
						cur_point.X = x;
						cur_point.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(cur_point);
					}
					for(i = 0;i<(st_c -1 );i++){
						y = y_l + i + 1;
						cur_point.X = x;
						cur_point.Y = y;
						//target->push_back(cur_point);
					}
				}
				x_l = x;
				y_l = y;
				st_c += 1;
			}
			target->push_back(cur_point);
			target->reverse();
			break;
		case SPIRAL_LEFT_IN:
			while(1){
				if(st_c>st_n){
					break;
				}
				if(st_c == 1){
					cur_point.X = x;
					cur_point.Y = y;
					target->push_back(cur_point);
				}
				else if((st_c%2)==0){//even number
					y = y_l - 1;
					y_l = y;
					for(i = 0;i < st_c;i++){
						x = x_l - i;
						cur_point.X = x;
						cur_point.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(cur_point);
					}
					for(i=0;i<(st_c - 1);i++){
						y = y_l + i + 1;
						cur_point.X = x;
						cur_point.Y = y;
						//target->push_back(cur_point);
					}
				}
				else{//odd number
					y = y_l + 1;
					y_l = y;
					for(i=0;i<st_c;i++){
						x = x_l + i;
						cur_point.X = x;
						cur_point.Y = y;
						if(i == 0 || i  == (st_c -1))
							target->push_back(cur_point);
					}
					for(i=0;i<(st_c-1);i++){
						y = y_l -i - 1;
						cur_point.X = x;
						cur_point.Y = y;
						//target->push_back(cur_point);
					}
				}
				x_l = x;
				y_l = y;
				st_c+=1;
			}
			target->push_back(cur_point);
			target->reverse();
			break;
		default:
			break;

    }
}
/*
* @author mengshige1988@qq.com
* @brief spot handle exception
* @param spot type (NORMAL_SPOT,CLEAN_SPOT,WALL_SPOT)
* @return ,1 for exception occur , 0 for not occur
*/
int8_t spot_handle_exception(SpotType st)
{
	/*-----detect stop event (remote_clean ,key press) ------*/
	if (stop_event()) {
		stop_brifly();
		disable_motors();
		while (get_key_press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: key pressing.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
		if(st == NORMAL_SPOT){
			set_clean_mode(Clean_Mode_Userinterface);
		}
		reset_stop_event_status();
		return 1;
	}
    uint8_t octype = check_motor_current();
	if (octype) {
		if(self_check(octype) && (st == NORMAL_SPOT)){
			return 1;
		}
		return 1;
	}
	/*--------- cliff detect event--------------*/
	if (get_cliff_trig() == (Status_Cliff_All)) {
		quick_back(20,20);//1 time
		stop_brifly();
		if(get_cliff_trig() == (Status_Cliff_All)){
			quick_back(20,20);//2 times
			stop_brifly();
		}
		if(get_cliff_trig() == Status_Cliff_All){
			quick_back(20,20);//3 times
			stop_brifly();
			ROS_INFO("%s %d, Cliff trigger three times ,robot lift up ",__FUNCTION__,__LINE__);
			if(st == NORMAL_SPOT)
				set_clean_mode(Clean_Mode_Userinterface);
			disable_motors();
			//wav_play(WAV_ERROR_LIFT_UP);
			return 1;
		}
	}
	/*--------get remote event ---------------*/
	if (get_rcon_remote()) {
		if(st == NORMAL_SPOT){
			if(remote_key(Remote_All)){
				if(get_rcon_remote() == Remote_Home){
					set_move_with_remote();
					set_home_remote();
				}
				reset_rcon_remote();
				return 1;
			}
		}
		else if(st == CLEAN_SPOT || st == WALL_SPOT){
			if(remote_key(Remote_Home)){
				reset_rcon_remote();
				set_move_with_remote();
				set_home_remote();
				return 1;
			}
			else if(remote_key(Remote_Left | Remote_Right | Remote_Forward)){
				reset_rcon_remote();
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

	reset_stop_event_status();

	check_bat_set_motors(135000, 100000, 100000);

#ifdef BLDC_INSTALL
	Set_VacMode(Vac_Max);
	Set_Vac_Speed();
#endif

	move_forward(0, 0);
	reset_wheel_step();
	reset_wall_step();
	usleep(10000);

	reset_rcon_remote();

	Motor_OC_Counter = 0;
	while (ros::ok()) {
		usleep(10000);
		Flash_Counter++;
		if (Flash_Counter > 20) {
			Watch_Counter++;
			if (Watch_Counter > 1000) {
				set_touch();
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

		if (remote_key(Remote_All)) {
			//Main_Brush_PWM = MainBrush_Power;
			move_forward(30, 30);
			reset_rcon_remote();
			return 0;
		}

		/*------------------------------------------------------Virtual Wall-----------------------*/
#ifdef VIRTUAL_WALL



#endif

		/*------------------------------------------------------Check Battery-----------------------*/
		if (check_bat_set_motors(135000, 100000, 120000)) {	//Low Battery Event
			move_forward(30, 30);
			return 0;
		}

		if (check_motor_current()) {
			Motor_OC_Counter++;
			if (Motor_OC_Counter > 50) {
				Motor_OC_Counter = 0;
				//Main_Brush_PWM = MainBrush_Power;
				move_forward(30, 30);
				return 0;
			}
		} else {
			Motor_OC_Counter = 0;
		}
		/*------------------------------------------------------stop event-----------------------*/
		if (stop_event()) {
			stop_brifly();
			ROS_INFO("%s %d: Stop event!", __FUNCTION__, __LINE__);
			return 1;
		}
		/*------------------------------------------------------Runing Path-----------------------*/

		switch (Move_Style) {
			case First_Round:
				if (get_left_wheel_step() > 6000) {
					move_forward(0, 0);
					reset_left_wheel_step();
					Move_Style = SPIRAL_RIGHT_OUT;
				}
				set_dir_right();
				set_wheel_speed(25, 10);

				if (get_bumper_status() || get_cliff_trig() || spot_obs_status()) {

					if (get_bumper_status()) {
						random_back();
					} else if (get_cliff_trig()) {
						move_back();
					}
					stop_brifly();
					turn_left(Turn_Speed, 2500);
					move_forward(10, 10);
					Move_Style = SPIRAL_LEFT_OUT;
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				break;

			case SPIRAL_RIGHT_OUT:
				if (get_left_wheel_step() > (Radius * 3)) {
					reset_left_wheel_step();
					if (Radius < 100) {
						Radius += 2;
					} else {
						Radius += 6;
					}
					if (Radius > 140) {
						Move_Style = SPIRAL_RIGHT_IN;
					}
				}
				if (get_bumper_status() || get_cliff_trig() || spot_obs_status()) {

					if (get_left_wall_step() < 3000) {
						Stunk++;
					}
					if (get_bumper_status()) {
						random_back();
					} else if (get_cliff_trig()) {
						move_back();
					}
					stop_brifly();
					Move_Style = SPIRAL_LEFT_OUT;
					turn_left(Turn_Speed, 2500);
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				set_dir_forward();
				set_left_wheel_speed(SPOT_MAX_SPEED);
				set_right_wheel_speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case SPIRAL_RIGHT_IN:
				if (get_left_wheel_step() > (Radius * 3)) {
					reset_left_wheel_step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 6;
					}
				}
				if (get_bumper_status() || get_cliff_trig() || spot_obs_status()) {

					if (get_left_wall_step() < 3000) {
						Stunk++;
					}
					if (get_bumper_status()) {
						random_back();
					} else if (get_cliff_trig()) {
						move_back();
					}
					stop_brifly();
					Move_Style = SPIRAL_LEFT_IN;
					turn_left(Turn_Speed, 2500);
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				set_dir_forward();
				set_left_wheel_speed(SPOT_MAX_SPEED);
				set_right_wheel_speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case SPIRAL_LEFT_OUT:
				if (get_right_wheel_step() > (Radius * 3)) {
					reset_right_wheel_step();
					if (Radius < 100) {
						Radius += 2;
					} else {
						Radius += 6;
					}
					if (Radius > 140) {
						Move_Style = SPIRAL_LEFT_IN;
					}
				}
				if (get_bumper_status() || get_cliff_trig() || spot_obs_status()) {

					if (get_left_wall_step() < 3000) {
						Stunk++;
					}
					if (get_bumper_status()) {
						random_back();
					} else if (get_cliff_trig()) {
						move_back();
					}
					stop_brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = SPIRAL_RIGHT_OUT;
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				set_dir_forward();
				set_right_wheel_speed(SPOT_MAX_SPEED);
				set_left_wheel_speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case SPIRAL_LEFT_IN:
				if (get_right_wheel_step() > (Radius * 2)) {
					reset_right_wheel_step();
					if (Radius < 3) {
						Spot_Flag = 1;
					}
					if (Radius < 100) {
						Radius -= 1;
					} else {
						Radius -= 6;
					}
				}
				if (get_bumper_status() || get_cliff_trig() || spot_obs_status()) {

					if (get_left_wall_step() < 3000) {
						Stunk++;
					}
					if (get_bumper_status()) {
						random_back();
					} else if (get_cliff_trig()) {
						move_back();
					}
					stop_brifly();
					Turn_Right(Turn_Speed, 2000);
					Move_Style = SPIRAL_RIGHT_IN;
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				set_dir_forward();
				set_right_wheel_speed(SPOT_MAX_SPEED);
				set_left_wheel_speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			default:
				break;

		}
		if ((OBS_Counter > 5) || (Stunk > 2)) {
			//Main_Brush_PWM = MainBrush_Power;
			move_forward(30, 30);
			return 0;
		}
		if (Spot_Flag) {
			Spot_Flag = 0;
			//Main_Brush_PWM = MainBrush_Power;
			move_forward(30, 30);
			return 2;
		}
	}
	//return 2;
}
