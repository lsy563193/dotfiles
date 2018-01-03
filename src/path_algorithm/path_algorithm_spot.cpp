#include <ros/ros.h>
#include "pp.h"
#include "path_algorithm.h"
#include "arch.hpp"

#define CLOCKWISE 1
#define ANTI_CLOCKWISE 2

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm()
{
	initVariables(0.6,updatePosition().toCell());
	genTargets( ANTI_CLOCKWISE, spot_diameter_, &targets_cells_,     begin_cell_);
}

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm(float diameter,Cell_t cur_cell)
{
	initVariables(diameter,cur_cell);
	genTargets( ANTI_CLOCKWISE, spot_diameter_, &targets_cells_,     begin_cell_);
}

SpotCleanPathAlgorithm::~SpotCleanPathAlgorithm()
{
	initVariables(0,{0,0});
}

bool SpotCleanPathAlgorithm::generatePath(GridMap &map, const Point32_t &curr, const MapDirection &last_dir, Points &plan_path)
{
	if(!ev.bumper_triggered && !ev.cliff_triggered && !ev.rcon_triggered){
		if(!spot_running_){
			spot_running_ = true;
			plan_path = cells_generate_points(targets_cells_);
			ROS_INFO("targets size %d",plan_path.size());
			return true;
		}
		else{
			plan_path.clear();
			ROS_INFO("spot end...");
			return false;
		}
	}
	else if(ev.bumper_triggered || ev.cliff_triggered || ev.rcon_triggered)
	{
		Points new_plan_path;
		Point32_t cur = curr;
		if(plan_path.size() >= 2)
		{
			Point32_t next_point;
			do{
				plan_path.pop_front();
				next_point = plan_path.front();
				auto shortest_path_cells = findShortestPath(map,cur.toCell(),next_point.toCell(),last_dir,true);
				auto shortest_path = cells_generate_points(shortest_path_cells);
				if(shortest_path.empty()){
					ROS_INFO("not find shortest_path %d continue",__LINE__);
					continue;
				}
				else{
					new_plan_path.clear();
					for(Point32_t point:shortest_path){
						ROS_INFO("\033[32m first find short path:(%d,%d)\033[0m",point.toCell().X,point.toCell().Y);
						new_plan_path.push_back(point);
					}
					ROS_INFO("target (%d,%d)",next_point.toCell().X,next_point.toCell().Y);
					break;
				}
			}while(ros::ok() && plan_path.size() >=1 );

			ROS_INFO("\033[32m new_plan_path size %d,the remained points size %d\033[0m",new_plan_path.size(),plan_path.size());	
			while(ros::ok() && plan_path.size() >=2)
			{
				plan_path.pop_front();
				cur = plan_path.front();
				if(map.getCell(COST_MAP,cur.toCell().X,cur.toCell().Y) < COST_HIGH ){
					ROS_INFO("\033[32m push back to new plan_path :(%d,%d)\033[0m",cur.toCell().X,cur.toCell().Y);
					new_plan_path.push_back(cur);
				}
				else{
					do{
						plan_path.pop_front();
						cur = plan_path.front();
						if(map.getCell(COST_MAP,cur.toCell().X,cur.toCell().Y) < COST_HIGH)
							break;
					}while(plan_path.size() >= 1);

					if(plan_path.size()<1)
						break;
					next_point = cur;
					cur = new_plan_path.back();
					auto next_dir = (MapDirection)cur.TH;
					auto shortest_path_cells = findShortestPath(map,cur.toCell(),next_point.toCell(),next_dir,true);
					auto shortest_path = cells_generate_points(shortest_path_cells);
					if(shortest_path.empty()){
						ROS_INFO("not find shortest_path %d continue",__LINE__);
					}
					else{
						ROS_INFO("target (%d,%d)",next_point.toCell().X,next_point.toCell().Y);
						for(Point32_t point:shortest_path){
							ROS_INFO("\033[32m After first find, short path:(%d,%d)\033[0m",point.toCell().X,point.toCell().Y);
							new_plan_path.push_back(point);
						}
					}
				}
			}
			plan_path.clear();
			ROS_INFO("\033[32m new_plan_path size %d\033[0m",new_plan_path.size());	
			plan_path = new_plan_path;
			if(new_plan_path.size()<1)
				return false;
			return true;
		}
		else{
			plan_path.clear();
			ROS_INFO("spot end...");
			return false;
		}
	}

}

void SpotCleanPathAlgorithm::initVariables(float diameter,Cell_t cur_cell)
{
	PP_INFO();
	spot_diameter_ = diameter;
	spot_running_ = false;
	begin_cell_ = cur_cell;
	targets_cells_.clear();
}

void SpotCleanPathAlgorithm::genTargets(uint8_t sp_type,float diameter,Cells *targets,const Cell_t begincell)
{
	targets->clear();
	int16_t i;
	uint32_t mid_it;// for store the last pos in clockwise/anti clockwise outint16_t x, x_last, y, y_last;
	int16_t x,y;
	int16_t x_last,y_last;
	uint16_t spiral_count = 1;//number of spiral count
	uint16_t cell_number = 1;//cell counter
	uint16_t spiral_number = (uint16_t) ceil(diameter * 1000 / (CELL_SIZE));//number of spiral
	ROS_INFO( "%s,%d,number of cells" "\033[36m" " %d" "\033[0m",__FUNCTION__,__LINE__,spiral_number);
	if (sp_type == CLOCKWISE)
	{
		for(int OUT = 1;OUT>=0;OUT--)
		{
			mid_it = targets->size();
			//reset some variables
			x = x_last = begincell.x;
			y = y_last = begincell.y;
			spiral_count = 1;
			cell_number = 1;

			if(spiral_count == 1){
				targets->push_back({x,y});
				spiral_count +=1;
				cell_number +=1;
			}

			while (ros::ok()) //clockwise out
			{
				if (spiral_count > spiral_number)
				{
					if(spiral_number %2 == 0)
					{
						if(OUT){
							x = x + 1 ;
							targets->push_back({x,y});
						}
					}
					else if(!OUT)
						targets->pop_back();

					break;
				}
				else if ((spiral_count % 2) == 0)
				{
					if(OUT){
						x =  x_last -1;
						x_last = x;
					}
					else{
						y = y_last - 1;
						y_last = y;
					}
					for (i = 0; i < cell_number; i=i+1)
					{
						if(OUT)
							y = y_last + i;
						else
							x = x_last + i;
						targets->push_back({x,y});
					}
					for (i = 1; i < cell_number; i=i+1)
					{
						if(OUT)
							x = x_last + i;
						else
							y = y_last + i;
						targets->push_back({x,y});
					}
				}
				else
				{
					if(OUT){
						x = x_last + 1;
						x_last = x;
					}
					else{
						y = y_last + 1;
						y_last = y;
					}
					for (i = 0; i < cell_number; i=i+1)
					{
						if(OUT)
							y = y_last - i;
						else
							x = x_last - i;
						targets->push_back({0,0});
					}
					for (i = 1; i < cell_number ; i=i+1)
					{
						if(OUT)
							x =  x_last -i ;
						else
							y = y_last - i;
						targets->push_back({0,0});
					}
				}
				x_last = x;
				y_last = y;
				spiral_count += 1;
				cell_number += 1;
			}
		}
		std::reverse(targets->begin()+mid_it, targets->end());
	}
	else if (sp_type == ANTI_CLOCKWISE)
	{
		for(int OUT = 1;OUT>=0;OUT--)
		{
			mid_it = targets->size();
			//reset some variable
			x = x_last = begincell.x;
			y = y_last = begincell.y;
			spiral_count = 1;
			cell_number = 1;
			targets->push_back({x,y});
			spiral_count +=1;
			cell_number +=1;
			while (ros::ok()) //anti clockwise out
			{
				if (spiral_count > spiral_number)
				{
					if(spiral_number %2 == 0)
					{
						if(OUT){
							x = x - 1;
							targets->push_back({x,y});
						}
					}
					else if(!OUT)
						targets->pop_back();
					break;
				}
				else if ((spiral_count % 2) == 0)
				{
					if(OUT){
						x =  x_last + 1;
						x_last = x;
					}
					else{
						y = y_last - 1;
						y_last = y;
					}
					for (i = 0; i < cell_number; i=i+1)
					{
						if(OUT)
							y = y_last + i;
						else
							x = x_last - i;
						targets->push_back({x,y});
					}
					for (i = 1; i < cell_number; i=i+1)
					{
						if(OUT)
							x = x_last - i;
						else
							y = y_last + i;
						targets->push_back({x,y});
					}
				}
				else
				{
					if(OUT){
						x = x_last - 1;
						x_last = x;
					}
					else{
						y = y_last + 1;
						y_last = y;
					}
					for (i = 0; i < cell_number; i=i+1)
					{
						if(OUT)
							y = y_last - i;
						else
							x = x_last + i;
						targets->push_back({x,y});
					}
					for (i = 1; i < cell_number ; i=i+1)
					{
						if(OUT)
							x =  x_last + i ;
						else
							y = y_last - i;
						targets->push_back({x,y});
					}
				}
				x_last = x;
				y_last = y;
				spiral_count += 1;
				cell_number += 1;
			}
		}
		std::reverse(targets->begin()+mid_it, targets->end());
	}
	displayCellPath(*targets);
}

bool SpotCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	checkTrappedUsingDijkstra(map, curr_cell);
}

