//
// Created by root on 11/17/17.
//

#include "pp.h"
#include "rcon.h"
#include "core_move.h"
#include "mathematics.h"
#include "lidar.hpp"

Rcon c_rcon;

Rcon::Rcon()
{
	init();
}
void Rcon::init()
{
	rcon_status_ = 0;
	found_charger_ = false;
	in_rcon_signal_range_ = false;
	charger_pos_.X = 0;
	charger_pos_.Y = 0;
	should_mark_charger_ = false;
	should_mark_temp_charger_ = false;
}
Rcon::~Rcon()
{
	init();
}

void Rcon::setRconPos(float cd,float dist)
{
	int16_t x = 0;
	int16_t y = 0;
	float yaw = robot::instance()->getPoseAngle()/10.0;
	float wpx = cosf( (float)ranged_angle((yaw+cd)*10)/10.0 * PI/180.0 )*dist+robot::instance()->getPoseX();
	float wpy = sinf( (float)ranged_angle((yaw+cd)*10)/10.0 * PI/180.0 )*dist+robot::instance()->getPoseY();
	x = (int16_t)(wpx*1000/CELL_SIZE);
	y = (int16_t)(wpy*1000/CELL_SIZE);
	charger_pos_.X = x;
	charger_pos_.Y = y;
	ROS_INFO("%s,%d:rcon value \033[32m0x%x\033[0m,charger direction \033[32m%f\033[0m,cureent direction \033[32m%f\033[0m,distance \033[32m%f\033[0m,world pos(\033[32m%f,%f\033[0m), cell pos(\033[32m%hd,%hd\033[0m)",__FUNCTION__,__LINE__,rcon_status_&RconAll_Home_T,cd,yaw,dist,wpx,wpy,x,y);

}

bool Rcon::estimateChargerPos(uint32_t rcon_value)
{
	if(!(rcon_value & RconAll_Home_T)){
		return false;
	}
	enum {flfr,frfr2,flfl2,fl2l,fr2r,bll,brr,fl2,fr2,l,r,bl,br};
	static int8_t cnt[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
	float cd = 0.0;//charger direction corrding to robot,in degrees
	float dist = 0.0;
	float len = 0.0;
	const int MAX_CNT = 2;
	const float DETECT_RANGE_MAX = 0.63;
	const float DETECT_RANGE_MIN = 0.33;

	/*-- here we only detect top signal from charge stub --*/
	//ROS_INFO("%s,%d,rcon_value 0x%x",__FUNCTION__,__LINE__,rcon_value & RconAll_Home_T);
	if( (rcon_value & RconFR_HomeT) || (rcon_value & RconFL_HomeT) ){ //fl & fr sensor
		if((cnt[flfr]++) >= MAX_CNT){
			cnt[flfr] = 0;
			cd = 0;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconFL_HomeT ) && (rcon_value & RconFL_HomeT)  && !(rcon_value & RconAll_R_HomeT)){//fl & fl2 sensor
		if((cnt[flfl2]++) >= MAX_CNT){
			cnt[flfl2] = 0;
			cd = 20;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconFR_HomeT ) && (rcon_value & RconFR2_HomeT) && !(rcon_value & RconAll_L_HomeT)){//fr & fr2 sensor
		if((cnt[frfr2]++) >= MAX_CNT){
			cnt[frfr2] = 0;
			cd = -20;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconFL2_HomeT) && !(rcon_value & RconAll_R_HomeT) //fl2 sensor
				&& !(rcon_value & RconBL_HomeT) )//to avoid charger signal reflection from other flat
	{
		if((cnt[fl2]++) >= MAX_CNT){
			cnt[fl2] = 0;
			cd = 43.0;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconFR2_HomeT) && !(rcon_value & RconAll_L_HomeT) //fr2 sensor
				&& !(rcon_value & RconBR_HomeT) ){//to avoid charger signal reflection from other flat
		if((cnt[fr2]++) >= MAX_CNT){
			cnt[fr2] = 0;
			cd = -43.0;
		}
		else{
			return false;
		}
	}

	else if( (rcon_value & RconL_HomeT) && (rcon_value & RconFL2_HomeT) && !(rcon_value & RconAll_R_HomeT) ){//fl2 & l sensor
		if((cnt[fl2l]++) >= MAX_CNT){
			cnt[fl2l] = 0;
			cd = 65;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconR_HomeT) && (rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT) ){//fr2 & r sensor
		if((cnt[fr2r]++) >= MAX_CNT){
			cnt[fr2r]=0;
			cd = -65;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconL_HomeT) && !(rcon_value & RconAll_R_HomeT) ){//l sensor
		if((cnt[l]++) >= MAX_CNT){
			cnt[l] = 0;
			cd = 85;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT) ){//r sensor
		if((cnt[r]++) >= MAX_CNT){
			cnt[r]=0;
			cd = -85;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconBL_HomeT) && (rcon_value & RconL_HomeT) && !(rcon_value & RconAll_R_HomeT)){//l & bl sensor
		if((cnt[bll]++) >= MAX_CNT){
			cnt[bll] = 0;
			cd = 110;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconBR_HomeT) && (rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT)){//r & br sensor
		if((cnt[brr]++) >= MAX_CNT){
			cnt[brr] = 0;
			cd = -110;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconBL_HomeT) && !(rcon_value & RconAll_R_HomeT) ){//bl sensor
		if((cnt[bl]++) >= MAX_CNT){
			cnt[bl] = 0;
			cd = 133;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconBR_HomeT) && !(rcon_value & RconAll_L_HomeT) ){//br sensor
		if((cnt[br]++) >= MAX_CNT){
			cnt[br] = 0;
			cd = -133;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
	memset(cnt,0,sizeof(int8_t)*13);
	if(robot::instance()->isScanAllow() && lidar.lidarCheckFresh(0.210,1)){
		int count = 0;
		double sum = 0.0;
		for(int i = cd -1;i<=cd +1;i++){//for calculate avarage distance
			dist = lidar.getLidarDistance(180+i);
			if(dist <= DETECT_RANGE_MAX && dist >= DETECT_RANGE_MIN){
				sum += dist;
				count++;
			}
		}
		if(count != 0){
			dist = sum /(count*1.0);
			if(fabs(cd)  == 43.0){//it's easy to make mistake in this angle ,which robot was almost paralled to the charger station ,so we add 0.1 to avoid mistake.
				if( dist >= (DETECT_RANGE_MIN + 0.1) ){
					setRconPos(cd,dist);
					ROS_INFO("\033[42;37m%s,%d,cd %f, rcon_state = 0x%x,dist = %f\033[0m",__FUNCTION__,__LINE__,cd,rcon_value,dist);
					should_mark_temp_charger_ = true;
					return false;
				}
			}
			else{
				if( dist > DETECT_RANGE_MAX || dist < DETECT_RANGE_MIN){
					ROS_INFO("\033[42;37m%s,%d,cd %f, rcon_state = 0x%x, distrance too far or too near\033[0m",__FUNCTION__,__LINE__,cd,rcon_value);
					return false;
				}
			}
		}
		else
			return false;
	} else{
		ROS_INFO("\033[42;37m%s,%d,cd %f, rcon_state = 0x%x\033[0m",__FUNCTION__,__LINE__,cd,rcon_value);
		return false;
	}
	setRconPos(cd,dist);
	found_charger_ = true;
	should_mark_charger_ = true;
	return true;
}

uint32_t Rcon::getStatus()
{
	const int CHARGE_SIGNAL_RANGES = 20;//IN CELL SIZE
	uint32_t rcon_value = 0;
	if(found_charger_){
		int16_t dist = (int16_t)two_points_distance(charger_pos_.X,charger_pos_.Y,CostMap::get_x_cell(),CostMap::get_y_cell());
		if(dist <= CHARGE_SIGNAL_RANGES){
			in_rcon_signal_range_ = true;
		}
		else{
			found_charger_ = false;
			in_rcon_signal_range_ = false;
		}
	}
	if (!cs.is_going_home()
				&& g_from_charger
				&& !in_rcon_signal_range_
				&& g_motion_init_succeeded
				&& !mt.is_go_to_charger()
				&& !mt.is_follow_wall()){
		rcon_value = 0;
	}
	else{
		rcon_value = rcon_status_;
	}
	return rcon_value;
}

static int get_trig_(uint32_t rcon_value)
{
	enum {
		left, fl1, fl2, fr2, fr1, right
	};
	static int8_t cnt[6] = {0, 0, 0, 0, 0, 0};
	const int MAX_CNT = 1;
	if ( rcon_value& RconL_HomeT)
		cnt[left]++;
	if ( rcon_value& RconFL_HomeT)
		cnt[fl1]++;
	if ( rcon_value& RconFL2_HomeT)
		cnt[fl2]++;
	if ( rcon_value& RconFR2_HomeT)
		cnt[fr2]++;
	if ( rcon_value& RconFR_HomeT)
		cnt[fr1]++;
	if ( rcon_value& RconR_HomeT)
		cnt[right]++;
	auto ret = 0;
	for (int i = 0; i < 6; i++)
		if (cnt[i] > MAX_CNT) {
			cnt[left] = cnt[fl1] = cnt[fl2] = cnt[fr2] = cnt[fr1] = cnt[right] = 0;
			ret = i + 1;
			break;
		}
	return ret;
}

int Rcon::getTrig(void)
{
	uint32_t rcon_value = getStatus();
	if (mt.is_follow_wall()) {
		if ((rcon_value &
					(RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT))) {
			rcon_status_ = 0;
			return get_trig_(rcon_value);
		}
		else{
			rcon_status_ = 0;
			return 0;
		}
	}
	else if (mt.is_linear()) {
		if (cm_is_exploration()) {
			auto status = rcon_value & RconAll_Home_T;
			rcon_status_ = 0;
			return status;
		}
		if(!found_charger_ && !cs.is_going_home() && estimateChargerPos(rcon_value)){
			rcon_status_ = 0;
			return get_trig_(rcon_value);
		}
		else if (rcon_value & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)) {
			rcon_status_ = 0;
			return get_trig_(rcon_value);
		}
		else{
			rcon_status_ = 0;
			return 0;
		}
	}
	else if (mt.is_go_to_charger()) {
		auto status = rcon_value;
		rcon_status_ = 0;
		return status;
	}

	return 0;
}
