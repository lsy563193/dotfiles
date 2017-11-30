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
#include "shortest_path.h"
#include "robot.hpp"
#include "robotbase.h"
#include "core_move.h"
#include "map.h"
#include "gyro.h"
#include "speaker.h"
#include "motion_manage.h"
#include "event_manager.h"
#include "slam.h"

#include <algorithm>
#include <ros/ros.h>
#include <time.h>
#include <vacuum.h>
#include <brush.h>
#include "clean_mode.h"

#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed  18
#endif

#define SPOT_MAX_SPEED  (20)
static SpotMovement *spot_obj = NULL;

static void spot_motor_configure()
{
	vacuum.setMode(Vac_Max);
	brush.setMainPwm(80);
	brush.setSidePwm(60, 60);
}

SpotMovement::SpotMovement(float diameter = 1.0)
{
	if (spot_obj == NULL)
		spot_obj = this;
	spot_diameter_ = diameter;
	near_cell_ = {0, 0};
	begin_cell_ = {0, 0};
	is_obs_trigger_ = 0;
	is_stuck_ = 0;
	sout_od_cnt_ = 0;
	sin_od_cnt_ = 0;
	targets_=NULL;
	st_ = NO_SPOT;
	spot_init_ = 0;
	spot_bumper_cnt_ = 0;
}

SpotMovement::~SpotMovement()
{
	spotDeinit();
}

SpotMovement *SpotMovement::instance()
{
	if (spot_obj == NULL)
		ROS_ERROR("%s,%d,spot obj should be init", __FUNCTION__, __LINE__);
	return spot_obj;
}


void SpotMovement::spotInit(float diameter, Cell_t cur_cell)
{

	if ((clock() / CLOCKS_PER_SEC) % 2 == 0)//random set spiral_type
	{
		spiral_type_ = ANTI_CLOCKWISE;
		ROS_INFO("\033[36m" "%s %d ,anti clockwise" "\033[0m", __FUNCTION__, __LINE__);
	} else
	{
		spiral_type_ = CLOCKWISE;
		ROS_INFO("\033[36m" "%s ,%d clockwise " "\033[0m", __FUNCTION__, __LINE__);
	}
	spot_diameter_ = diameter;
	near_cell_ = {0, 0};
	begin_cell_ = {cur_cell.X, cur_cell.Y};
	//bp_ = targets_->end();
	//tp_ = targets_->end();
	is_obs_trigger_ = 0;
	is_stuck_ = 0;
	sout_od_cnt_ = 0;
	sin_od_cnt_ = 0;
	targets_ = NULL;
	spot_init_ = 1;
	spot_bumper_cnt_ = (uint16_t)(diameter*1000/CELL_SIZE);
	spot_motor_configure();
	go_last_point_ = 0;
}

void SpotMovement::spotDeinit()
{
	near_cell_ = {0, 0};
	begin_cell_ = {0, 0};
	//bp_ = targets_->end();
	//tp_ = targets_->end();
	is_obs_trigger_ = 0;
	is_stuck_ = 0;
	sout_od_cnt_ = 0;
	sin_od_cnt_ = 0;
	targets_ = NULL;
	spot_diameter_ = 0;
	spot_init_ = 0;
	spot_bumper_cnt_ = 0;
	if(getSpotType() == CLEAN_SPOT){
		cs_work_motor();
	}
	resetSpotType();
}

#if 0
void SpotMovement::getNeighbourCell(std::vector<Cell_t>::iterator &bp,Cell_t *np)
{
	if (bp == targets_->begin())
	{
		if (spiral_type_ == CLOCKWISE_OUT ){
			*np = {bp->X, (int16_t)(bp->Y - 1)};
		}
		else if( spiral_type_ == ANTI_CLOCKWISE_OUT){
			*np = {bp->X, (int16_t)(bp->Y + 1)};
		}
	}
	else
	{
		if ( ( bp-1 )->X == bp->X )
		{
			if (spiral_type_ == CLOCKWISE_OUT || spiral_type_ == ANTI_CLOCKWISE_OUT)
				*np = { (int16_t)( ( bp->X > 0 )? bp->X + 1: bp->X - 1), bp->Y };
			else if (spiral_type_ == CLOCKWISE_IN || spiral_type_ == ANTI_CLOCKWISE_IN)
				*np = { (int16_t)( ( bp->X > 0 )? bp->X - 1: bp->X + 1), bp->Y };
		}
		else if ( ( bp-1 )->Y == bp->Y)
		{
			if (spiral_type_ == CLOCKWISE_OUT || spiral_type_ == ANTI_CLOCKWISE_OUT)
				*np = { bp->X, (int16_t)( ( bp->Y > 0 )? bp->Y + 1: bp->Y - 1 ) };
			else if (spiral_type_ == CLOCKWISE_IN || spiral_type_ == ANTI_CLOCKWISE_IN)
				*np = { bp->X, (int16_t)( ( bp->Y > 0 )? bp->Y - 1: bp->Y + 1 ) };
		}
	}
	ROS_INFO("\033[35m" "%s,%d, neighbour cell (%d,%d)" "\033[0m", __FUNCTION__, __LINE__, np->X, np->Y);
}
#endif
#if 0
uint8_t SpotMovement::setNearCell(const Cell_t &cur_cell,Cell_t *nc)
{
	if (spiral_type_ == CLOCKWISE_OUT || spiral_type_ == ANTI_CLOCKWISE_OUT)
	{
		targets_ = (spiral_type_ == CLOCKWISE_OUT) ? &targets_acw_out_ : &targets_cw_out_;
		tp_ = targets_->begin();
		spiral_type_ = (spiral_type_ == CLOCKWISE_OUT) ? ANTI_CLOCKWISE_OUT : CLOCKWISE_OUT;
	}
	else if (spiral_type_ == CLOCKWISE_IN || spiral_type_ == ANTI_CLOCKWISE_IN)
	{
		targets_ = (spiral_type_ == CLOCKWISE_IN) ? &targets_acw_in_ : &targets_cw_in_;
		tp_ = targets_->begin();
		spiral_type_ = (spiral_type_ == CLOCKWISE_IN) ? ANTI_CLOCKWISE_IN : CLOCKWISE_IN;
	}
	ROS_INFO("\033[35m" "%s,%d,spiral changed %s" "\033[0m",__FUNCTION__, __LINE__, (spiral_type_ == 1 || spiral_type_ == 4)? ((spiral_type_ == 1)?"clokwise out":"anti clockwise out"):(spiral_type_ == 2?"clockwise in":"anti clockwise in"));
	if(bp_->X == cur_cell.X && bp_->Y == cur_cell.Y){
		tp_++;
	}
	for(;tp_!=targets_->end();++tp_)//search  interator in current cell
	{
		if( cur_cell.X == tp_->X && cur_cell.Y == tp_->Y )
		{
			bp_ = tp_;
		}
	}
	ROS_INFO("\033[35m" "%s,%d,current cell (%d,%d),bumper cell (%d,%d)" "\033[0m",__FUNCTION__,__LINE__,cur_cell.X,cur_cell.Y,bp_->X,bp_->Y);

	int ret = 0;
	std::vector<Cell_t>::iterator ttp;
	int cnt = 0;
	while(ros::ok()){
		getNeighbourCell(bp_,nc);
		for (ttp = targets_->begin(); ttp != targets_->end(); ++ttp)//search near interator in current target list
		{
			if(nc->X == ttp->X && nc->Y == ttp->Y){
				ret = 1;
				tp_ = ttp;
				break;
			}
		}
		ROS_INFO("\033[35m" "%s,%d,%s near cell (%d,%d)" "\033[0m",__FUNCTION__,__LINE__,ret?"find":"not find",nc->X,nc->Y);
		if(ret){
			return ret;
		}
		if(++cnt >= 2){
			return 0;
		}
		if(ret == 0 && (spiral_type_ != CLOCKWISE_IN || spiral_type_ != ANTI_CLOCKWISE_IN)){
			spiral_type_ = (spiral_type_ == CLOCKWISE_OUT) ? ANTI_CLOCKWISE_IN : CLOCKWISE_IN;
			targets_ = (spiral_type_ == CLOCKWISE_IN) ? &targets_cw_in_ : &targets_acw_in_;
			ROS_INFO("\033[36m" "%s,%d,not find neighbour cell change to %s" "\033[0m",__FUNCTION__,__LINE__,(spiral_type_ == CLOCKWISE_OUT) ? "ANTI_CLOCKWISE_IN" : "CLOCKWISE_IN");
		}
		else{
			return ret;
		}
	}

}
#endif

void SpotMovement::genTargets(uint8_t sp_type,float diameter,std::vector<Cell_t> *target,const Cell_t begincell)
{
	uint8_t spt = sp_type;
	int16_t x, x_l, y, y_l;
	x = x_l = begincell.X;
	y = y_l = begincell.Y;
	target->clear();
	uint16_t st_c = 1;//number of spiral count
	uint16_t c_c = 1;//cell counter
	uint16_t st_n = (uint16_t) ceil(diameter * 1000 / (CELL_SIZE*2));//number of spiral
	ROS_INFO( "%s,%d,number of cells" "\033[36m" " %d" "\033[0m",__FUNCTION__,__LINE__,st_n);
	int16_t i;
	int mid_it;// for store the last pos in clockwise/anti clockwise out

	if(st_c == 1){
		target->push_back({x,y});
		st_c +=1;
		c_c +=2;
	}

	if (spt == CLOCKWISE)
	{
		std::string msg("clockwise out: (");
		msg +=std::to_string(x)+","+std::to_string(x)+")";
		while (ros::ok()) //clockwise out
		{
			if (st_c > st_n)
			{
				if(st_n %2 == 0)
				{
					x = x+ 2 ;//ANTI_CLOCKWISE_OUT ? x -2 : x +2;
					target->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				break;
			}
			else if ((st_c % 2) == 0)
			{
				x =  x_l -2;
				x_l = x;
				for (i = 0; i < c_c; i=i+2)
				{
					y = y_l + i;
					target->push_back({x, y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 2; i < c_c; i=i+2)
				{
					x =  x_l +i;
					target->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}

			}
			else
			{
				x = x_l + 2;
				x_l = x;
				for (i = 0; i < c_c; i=i+2)
				{
					y = y_l - i;
					target->push_back({x, y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 2; i < c_c ; i=i+2)
				{
					x =  x_l -i ;
					target->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			x_l = x;
			y_l = y;
			st_c += 1;
			c_c += 2;
		}
		mid_it = target->size();
		x = x_l = begincell.X;
		y = y_l = begincell.Y;
		st_c = 1;
		c_c = 1;	
		if(st_c == 1){
			target->push_back({x,y});
			st_c +=1;
			c_c +=2;
		}
		msg+="\n clockwise in: ("+std::to_string(x)+","+std::to_string(y)+")";
		while (ros::ok())//clockwise in
		{
			if (st_c > st_n){
				if(st_n %2 == 0){
					y = y + 2;
					target->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				break;
			}
			else if ((st_c % 2) == 0)
			{
				y = y_l - 2;
				y_l = y;
				for (i = 0; i < c_c; i=i+2)
				{
					x = x_l + i;
					target->push_back({x, y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 2; i < c_c; i=i+2)
				{
					y = y_l + i;
					target->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			else
			{
				y = y_l + 2;
				y_l = y;
				for (i = 0; i < c_c; i=i+2)
				{
					x = x_l - i;
					target->push_back({x, y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 2; i < c_c; i=i+2)
				{
					y = y_l - i;
					target->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			x_l = x;
			y_l = y;
			st_c += 1;
			c_c += 2;
		}
		ROS_INFO("\033[36m""%s""\033[0m",msg.c_str());	
		std::reverse(target->begin()+mid_it, target->end());
	}
	else if (spt == ANTI_CLOCKWISE)
	{
		std::string msg("anti clockwise out: (");
		msg+=std::to_string(x)+","+std::to_string(y)+")";
		while (ros::ok()) //anti clockwise out
		{
			if (st_c > st_n)
			{
				if(st_n %2 == 0)
				{
					x = x+ 2 ;
					target->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				break;
			}
			else if ((st_c % 2) == 0)
			{
				x =  x_l + 2;
				x_l = x;
				for (i = 0; i < c_c; i=i+2)
				{
					y = y_l + i;
					target->push_back({x, y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 2; i < c_c; i=i+2)
				{
					x =  x_l - i;
					target->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}

			}
			else
			{
				x = x_l - 2;
				x_l = x;
				for (i = 0; i < c_c; i=i+2)
				{
					y = y_l - i;
					target->push_back({x, y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 2; i < c_c ; i=i+2)
				{
					x =  x_l + i ;
					target->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			x_l = x;
			y_l = y;
			st_c += 1;
			c_c += 2;
		}
		mid_it = target->size();
		x = x_l = begincell.X;
		y = y_l = begincell.Y;
		st_c = 1;c_c = 1;
		if(st_c == 1){
			target->push_back({x,y});
			st_c +=1;
			c_c +=2;
		}
		msg+="\n anti clockwise in: ("+std::to_string(x)+","+std::to_string(y)+")";
		while (ros::ok())//anti clockwise in
		{
			if (st_c > st_n){
				if(st_n %2 == 0){
					y = y + 2;
					target->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				break;
			}
			else if ((st_c % 2) == 0)
			{
				y = y_l - 2;
				y_l = y;
				for (i = 0; i < c_c; i=i+2)
				{
					x = x_l - i;
					target->push_back({x, y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 2; i < c_c; i=i+2)
				{
					y = y_l + i;
					target->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			else
			{
				y = y_l + 2;
				y_l = y;
				for (i = 0; i < c_c; i=i+2)
				{
					x = x_l + i;
					target->push_back({x, y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 2; i < c_c; i=i+2)
				{
					y = y_l - i;
					target->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			x_l = x;
			y_l = y;
			st_c += 1;
			c_c += 2;
		}
		ROS_INFO("\033[36m""%s""\033[0m",msg.c_str());	
		std::reverse(target->begin()+mid_it, target->end());
	}
}

void SpotMovement::stright2End(uint8_t spt, std::vector<Cell_t>::iterator &tp)
{
	uint8_t in_row=0,in_col=0;
	while((tp+1) != targets_->end() && ros::ok())//stright to the end of column or row
	{
		if( tp->X == (tp+1)->X && !in_row)
		{
			tp++;
			in_col = 1;
		}
		else if(in_col)
		{
			break;
		}
		if( tp->Y == (tp+1)->Y && !in_col)
		{
			tp++;
			in_row = 1;
		}
		else if(in_row)
		{
			break;
		}
	}
}

uint8_t SpotMovement::spotNextTarget(const Cell_t& cur_cell,PPTargetType *target_path)
{
	auto last = target_path->front();
	uint8_t ret = 0;
	SpotType spt = getSpotType();
	if (!isSpotInit() && spt != NO_SPOT)//for the first time
	{
		/*---init spot move and set start cell---*/
		if (spt == CLEAN_SPOT)
		{
			speaker.play(SPEAKER_CLEANING_SPOT);
			spotInit(1.0, cur_cell);//start from current cell
		}
		else if( spt == NORMAL_SPOT){
			spotInit(1.0, {0, 0});//start from 0,0
		}
		/*---generate targets ---*/
		genTargets( CLOCKWISE,      spot_diameter_, &targets_cw_,     begin_cell_);
		genTargets( ANTI_CLOCKWISE, spot_diameter_, &targets_acw_,    begin_cell_);
		/*---spiral out first-----*/
		targets_ = (spiral_type_ == CLOCKWISE) ? &targets_cw_: &targets_acw_;
		tp_ = targets_->begin();
		/*----put all targets in target_path----*/
		pushAllTargets(target_path);
		target_last_.clear();
		ret = 1;
	}
	else if (spot_init_ == 1 && isOBSTrigger())// bumper/obs trigger
	{
		ROS_INFO("\033[36m""spot.cpp bumper/obs trigger""\033[0m");
		resetOBSTrigger();
		cost_map.print(MAP, target_path->back().X, target_path->back().Y);
		Cell_t current_cell = cost_map.getCurrCell();
		ROS_INFO("\033[36m""current cell(%d,%d)""\033[0m",current_cell.X,current_cell.Y);
		int pnb_ret = 0;
		if(target_last_.empty()){
			target_last_ = *target_path;
		}
		else{
			last = target_last_.front();
		}
		target_path->clear();
		ROS_INFO("\033[36m""last(%d,%d)""\033[0m",last.X,last.Y);
		Cell_t next_cell;
		/*---search cells---*/
		uint32_t size = target_last_.size();
		for(int i = 0;i<size;i++){
			next_cell = target_last_.front();
			if(last == next_cell){ 
				target_last_.pop_front();
				pnb_ret = path_next_shortest(current_cell, next_cell, *target_path);
				if(pnb_ret == 1){
					ret = 1;
					ROS_INFO("\033[36m" "%s,%d , bumper/obs trigger, get last cell (%d %d) " "\033[0m", __FUNCTION__, __LINE__, next_cell.X,next_cell.Y);
					break;
				}
				last = target_last_.front();
			}
			else{
				target_last_.pop_front();
			}
			ROS_INFO("\033[36m""drop target(%d,%d)""\033[0m",next_cell.X,next_cell.Y);
		}
		if(pnb_ret <= 0){
			ROS_INFO("\033[36m" "%s,%d, no target find maybe stuck!" "\033[0m",__FUNCTION__,__LINE__);
			ret = endSpot(target_path);
			return ret;
		}
	}
	else if(spot_init_ == 1 && !target_last_.empty())
	{
		ROS_INFO("\033[36m" "spot.cpp continue clean...""\033[0m");
		(*target_path).clear();
		*target_path = target_last_;
		std::string msg("rest cells:");
		/*-------print the rest of cells -----*/
		for(auto it = (*target_path).begin();it!=(*target_path).end();it++){
			msg+="("+std::to_string(it->X)+","+std::to_string(it->Y)+")"+"->";
		}
		ROS_INFO("\033[36m""%s""\033[0m",msg.c_str());	
		target_last_.clear();
		ret = 1;
	}
	else if(spot_init_ == 1 && tp_ == targets_->end())
	{
		ret = endSpot(target_path);
	}
	return ret;
}

void SpotMovement::pushAllTargets(PPTargetType *target_path)
{
	(*target_path).clear();
	while(tp_ != targets_->end() && ros::ok()){
		stright2End(spiral_type_, tp_ );
		if ((tp_+1) != targets_->end())
		{
			ROS_INFO("\033[36m" "%s,%d , get next cell (%d %d) " "\033[0m", __FUNCTION__, __LINE__, tp_->X, tp_->Y);
			(*target_path).push_back({tp_->X,tp_->Y});
		}
		else
		{
			(*target_path).push_back({tp_->X,tp_->Y});
			tp_++;
			break;
		}
	}
}

int8_t SpotMovement::endSpot(PPTargetType *target_path)
{
	int8_t ret;	
	if(go_last_point_ == 0){
		ROS_INFO("\033[36m""go back to begin cell""\033[0m");
		(*target_path).clear();
		(*target_path).push_back({begin_cell_.X, begin_cell_.Y});// go back to begin cell
		ret = 1;
		go_last_point_ = 1;
	}
	else{
		ret = (getSpotType() == CLEAN_SPOT)?1:0;
		spotDeinit();//clear all spot variable
		ROS_INFO("\033[36m" "%s,%d , spot ending, ending cell (%d %d) " "\033[0m", __FUNCTION__, __LINE__, begin_cell_.X, begin_cell_.Y);
		go_last_point_ = 0;
	}
	return ret;
}

