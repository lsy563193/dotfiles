#include <ros/ros.h>
#include "pp.h"
#include "path_algorithm.h"
#include "arch.hpp"

#define CLOCKWISE 1
#define ANTI_CLOCKWISE 2

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm()
{
	initVariables(1.0,nav_map.getCurrCell());
	//random set spot direction
//	if ((clock() / CLOCKS_PER_SEC) % 2 == 0){
//		genTargets( CLOCKWISE,  spot_diameter_, &targets_,     begin_cell_);
//	}
//	else{
		genTargets( ANTI_CLOCKWISE, spot_diameter_, &targets_,     begin_cell_);
//	}

}

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm(float diameter,Cell_t cur_cell)
{
	initVariables(diameter,cur_cell);
	//random set spot direction
	if ((clock() / CLOCKS_PER_SEC) % 2 == 0){
		genTargets( CLOCKWISE,  spot_diameter_, &targets_,     begin_cell_);
	}
	else{
		genTargets( ANTI_CLOCKWISE, spot_diameter_, &targets_,     begin_cell_);
	}

}

SpotCleanPathAlgorithm::~SpotCleanPathAlgorithm()
{
	initVariables(0,{0,0,0});
}

void SpotCleanPathAlgorithm::initVariables(float diameter,Cell_t cur_cell)
{
	PP_INFO();
	spot_diameter_ = diameter;
	spot_running_ = false;
	begin_cell_ = cur_cell;
	targets_last_.clear();
}

void SpotCleanPathAlgorithm::genTargets(uint8_t sp_type,float diameter,Path_t *targets,const Cell_t begincell)
{
	uint8_t spt = sp_type;
	int16_t x, x_l, y, y_l;
	x = x_l = begincell.X;
	y = y_l = begincell.Y;
	targets->clear();
	uint16_t spiral_count = 1;//number of spiral count
	uint16_t cell_number = 1;//cell counter
	uint16_t spiral_number = (uint16_t) ceil(diameter * 1000 / (CELL_SIZE));//number of spiral
	ROS_INFO( "%s,%d,number of cells" "\033[36m" " %d" "\033[0m",__FUNCTION__,__LINE__,spiral_number);
	int16_t i;
	int mid_it;// for store the last pos in clockwise/anti clockwise out

	if(spiral_count == 1){
		targets->push_back({x,y,0});
		spiral_count +=1;
		cell_number +=1;
	}

	if (spt == CLOCKWISE)
	{
		std::string msg("clockwise out: (");
		msg +=std::to_string(x)+","+std::to_string(x)+")";
		while (ros::ok()) //clockwise out
		{
			if (spiral_count > spiral_number)
			{
				if(spiral_number %2 == 0)
				{
					x = x + 1 ;
					targets->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				break;
			}
			else if ((spiral_count % 2) == 0)
			{
				x =  x_l -1;
				x_l = x;
				for (i = 0; i < cell_number; i=i+1)
				{
					y = y_l + i;
					targets->push_back({x, y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 1; i < cell_number; i=i+1)
				{
					x =  x_l +i;
					targets->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}

			}
			else
			{
				x = x_l + 1;
				x_l = x;
				for (i = 0; i < cell_number; i=i+1)
				{
					y = y_l - i;
					targets->push_back({x, y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 1; i < cell_number ; i=i+1)
				{
					x =  x_l -i ;
					targets->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			x_l = x;
			y_l = y;
			spiral_count += 1;
			cell_number += 1;
		}
		mid_it = targets->size();
		//reset some variables
		x = x_l = begincell.X;
		y = y_l = begincell.Y;
		spiral_count = 1;
		cell_number = 1;
		if(spiral_count == 1){
			targets->push_back({x,y});
			spiral_count +=1;
			cell_number +=1;
		}
		msg+="\n clockwise in: ("+std::to_string(x)+","+std::to_string(y)+")";
		while (ros::ok())//clockwise in
		{
			if (spiral_count > spiral_number){
				if(spiral_number %2 == 0){
					y = y + 2;
					targets->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				break;
			}
			else if ((spiral_count % 2) == 0)
			{
				y = y_l - 1;
				y_l = y;
				for (i = 0; i < cell_number; i=i+1)
				{
					x = x_l + i;
					targets->push_back({x, y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 1; i < cell_number; i=i+1)
				{
					y = y_l + i;
					targets->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			else
			{
				y = y_l + 1;
				y_l = y;
				for (i = 0; i < cell_number; i=i+1)
				{
					x = x_l - i;
					targets->push_back({x, y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 1; i < cell_number; i=i+1)
				{
					y = y_l - i;
					targets->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			x_l = x;
			y_l = y;
			spiral_count += 1;
			cell_number += 1;
		}
		ROS_INFO("\033[36m""%s""\033[0m",msg.c_str());
		std::reverse(targets->begin()+mid_it, targets->end());
	}
	else if (spt == ANTI_CLOCKWISE)
	{
		std::string msg("anti clockwise out: (");
		msg+=std::to_string(x)+","+std::to_string(y)+")";
		while (ros::ok()) //anti clockwise out
		{
			if (spiral_count > spiral_number)
			{
				if(spiral_number %2 == 0)
				{
					x = x+ 1 ;
					targets->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				break;
			}
			else if ((spiral_count % 2) == 0)
			{
				x =  x_l + 1;
				x_l = x;
				for (i = 0; i < cell_number; i=i+1)
				{
					y = y_l + i;
					targets->push_back({x, y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 1; i < cell_number; i=i+1)
				{
					x =  x_l - i;
					targets->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}

			}
			else
			{
				x = x_l - 1;
				x_l = x;
				for (i = 0; i < cell_number; i=i+1)
				{
					y = y_l - i;
					targets->push_back({x, y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 1; i < cell_number ; i=i+1)
				{
					x =  x_l + i ;
					targets->push_back({x,y});
					msg += "->("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			x_l = x;
			y_l = y;
			spiral_count += 1;
			cell_number += 1;
		}
		mid_it = targets->size();
		//reset some variable
		x = x_l = begincell.X;
		y = y_l = begincell.Y;
		spiral_count = 1;cell_number = 1;
		if(spiral_count == 1){
			targets->push_back({x,y});
			spiral_count +=1;
			cell_number +=1;
		}
		msg+="\n anti clockwise in: ("+std::to_string(x)+","+std::to_string(y)+")";
		while (ros::ok())//anti clockwise in
		{
			if (spiral_count > spiral_number){
				if(spiral_number %2 == 0){
					y = y + 1;
					targets->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				break;
			}
			else if ((spiral_count % 2) == 0)
			{
				y = y_l - 1;
				y_l = y;
				for (i = 0; i < cell_number; i=i+1)
				{
					x = x_l - i;
					targets->push_back({x, y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 1; i < cell_number; i=i+1)
				{
					y = y_l + i;
					targets->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			else
			{
				y = y_l + 1;
				y_l = y;
				for (i = 0; i < cell_number; i=i+1)
				{
					x = x_l + i;
					targets->push_back({x, y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
				for (i = 1; i < cell_number; i=i+1)
				{
					y = y_l - i;
					targets->push_back({x,y});
					msg += "<-("+std::to_string(x)+","+std::to_string(y)+")";
				}
			}
			x_l = x;
			y_l = y;
			spiral_count += 1;
			cell_number += 1;
		}
		ROS_INFO("\033[36m""%s""\033[0m",msg.c_str());
		std::reverse(targets->begin()+mid_it, targets->end());
	}
}

bool SpotCleanPathAlgorithm::generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir, Path_t &plan_path)
{
	if(!ev.bumper_triggered && !ev.cliff_triggered && !ev.rcon_triggered){
		if(!spot_running_){
			spot_running_ = true;
			plan_path = targets_;
			PP_INFO();
			ROS_INFO("targets size %d",targets_.size());
		}
		else{
			PP_INFO();
			if(plan_path.size() >= 2)
				if(!targets_last_.empty())
					plan_path = targets_last_;
				else
					plan_path = plan_path;
			else{
				plan_path.clear();
				return false;
			}
		}
		ROS_INFO("plan_path size %d",plan_path.size());
		fillPathWithDirection(plan_path);
		return true;
	}
	else if(ev.bumper_triggered || ev.cliff_triggered || ev.rcon_triggered)
	{
		bool ret = false;
		Path_t shortest_path;
		if(targets_last_.empty())
		{
			while(ros::ok())
			{
				if(plan_path.size() > 2){
					plan_path.pop_front();
					auto next_cell = plan_path.front();
					shortest_path = findShortestPath(map,curr_cell,next_cell,last_dir,false);
					if(shortest_path.empty()){
						continue;
					}
					else{
						ret = true;
						targets_last_ = shortest_path;
						break;
					}
				}
				else{
					break;
				}
			}
		}
		else
		{
			while(ros::ok())
			{
				if(targets_last_.size() > 2)
				{
					targets_last_.pop_front();
					auto next_cell = targets_last_.front();
					shortest_path = findShortestPath(map,curr_cell,next_cell,last_dir,false);
					if(shortest_path.empty()){
						continue;
					}
					else{
						ret = true;
						break;
					}
				}
				else
					break;
			}
		}
		plan_path.clear();
		plan_path = shortest_path;
		fillPathWithDirection(plan_path);
		return ret;
	}
}

bool SpotCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	checkTrappedUsingDijkstra(map, curr_cell);
}
#if 0
void SpotCleanPathAlgorithm::filterTarget()
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

void SpotCleanPathAlgorithm::pushAllTargets(Path_t *targets)
{
	(*targets).clear();
	while(tp_ != targets_->end() && ros::ok()){
		filterTarget();
		if ((tp_+1) != targets_->end())
		{
			ROS_INFO("\033[36m" "%s,%d , get next cell (%d %d) " "\033[0m", __FUNCTION__, __LINE__, tp_->X, tp_->Y);
			(*targets).push_back({tp_->X,tp_->Y});
		}
		else
		{
			(*targets).push_back({tp_->X,tp_->Y});
			tp_++;
			break;
		}
	}
}

bool SpotCleanPathAlgorithm::endSpot(Path_t *targets)
{
	bool ret;
	if(go_last_cell_== false){
		ROS_INFO("\033[36m""go back to begin cell""\033[0m");
		(*targets).clear();
		(*targets).push_back({begin_cell_.X, begin_cell_.Y});// go back to begin cell
		ret = true;
		go_last_cell_= true;
	}
	else{
		ret = (getSpotType() == CLEAN_SPOT)?true:false;
		spotDeinit();//clear all spot variable
		ROS_INFO("\033[36m" "%s,%d , spot ending, ending cell (%d %d) " "\033[0m", __FUNCTION__, __LINE__, begin_cell_.X, begin_cell_.Y);
		go_last_cell_= false;
	}
	return ret;
}

bool SpotCleanPathAlgorithm::spotNextTarget(const Cell_t& cur_cell,Path_t *targets)
{
	auto last = targets->front();
	bool ret = false;
	if (isSpotInit() && 0/*isOBSTrigger()*/)// bumper/obs trigger
	{
		ROS_INFO("\033[36m""spot.cpp bumper/obs trigger""\033[0m");
		resetOBSTrigger();
		Cell_t current_cell = map_get_curr_cell();
		ROS_INFO("\033[36m""current cell(%d,%d)""\033[0m",current_cell.X,current_cell.Y);
		int pnb_ret = 0;
		if(target_old_.empty()){
			target_old_ = *targets;
		}
		else{
			last = target_old_.front();
		}
		targets->clear();
		ROS_INFO("\033[36m""last(%d,%d)""\033[0m",last.X,last.Y);
		Cell_t next_cell;
		/*---search cells---*/
		uint32_t size = target_old_.size();
		for(uint32_t i = 0; i<target_old_size(); i++){
			next_cell = target_old_.front();
			if(last == next_cell){ 
				target_old_.pop_front();
				pnb_ret = findShortestPath(map,current_cell, next_cell, *targets);
				if(pnb_ret == 1){
					ret = true;
					ROS_INFO("\033[36m" "%s,%d , bumper/obs trigger, get last cell (%d %d) " "\033[0m", __FUNCTION__, __LINE__, next_cell.X,next_cell.Y);
					break;
				}
				last = target_old_.front();
			}
			else{
				target_old_.pop_front();
			}
			ROS_INFO("\033[36m""drop targets(%d,%d)""\033[0m",next_cell.X,next_cell.Y);
		}
		if(pnb_ret <= 0){
			ROS_INFO("\033[36m" "%s,%d, no targets find maybe stuck!" "\033[0m",__FUNCTION__,__LINE__);
			ret = endSpot(targets);
			return ret;
		}
	}
	else if(isSpotInit() && !target_old_.empty())
	{
		ROS_INFO("\033[36m" "spot.cpp continue clean...""\033[0m");
		(*targets).clear();
		*targets = target_old_;
		std::string msg("reset cells:");
		/*-------print the rest of cells for debug-----*/
		for(auto it = (*targets).begin();it!=(*targets).end();it++){
			msg+="("+std::to_string(it->X)+","+std::to_string(it->Y)+")"+"->";
		}
		ROS_INFO("\033[36m""%s""\033[0m",msg.c_str());
		target_old_.clear();
		ret = true;
	}
	else if(isSpotInit()&& tp_ == targets_->end())
	{
		ret = endSpot(targets_);
	}
	return ret;
}
#endif


