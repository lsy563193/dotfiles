#include <ros/ros.h>
#include <event_manager.h>
#include "robot.hpp"
#include "path_algorithm.h"
#include <initializer_list>

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm()
{
	Cell_t  cur = getPosition().toCell();
	float radius = 0.5;
	initVariables(radius,cur);
	genTargets( ANTI_CLOCKWISE, radius, &targets_cells_,cur);
}

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm(float radius,Cell_t cur_cell)
{
	initVariables(radius,cur_cell);
	genTargets( ANTI_CLOCKWISE,radius, &targets_cells_, cur_cell);
}

SpotCleanPathAlgorithm::~SpotCleanPathAlgorithm()
{
	initVariables(0,{0,0});
}

/*
bool SpotCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path)
{
	if(!ev.bumper_triggered && !ev.cliff_triggered && !ev.rcon_status){
		if(!spot_running_){
			spot_running_ = true;
			plan_path = *cells_generate_points(make_unique<Cells>(targets_cells_));
			ROS_INFO("targets size %lu",plan_path.size());
			return true;
		}
		else if(plan_path.size() >= 2){
			ROS_INFO("maybe run over the target");
			plan_path = plan_path;
			return true;
		}
		else{
			plan_path.clear();
			ROS_INFO("spot end...");
			return false;
		}
	}
	else if(ev.bumper_triggered || ev.cliff_triggered || ev.rcon_status)
	{
		Points new_plan_path;
		Point_t cur = curr;
		if(plan_path.size() >= 2)
		{
			//---first find shortest path---
			Point_t next_point;
			do{
				plan_path.pop_front();
				next_point = plan_path.front();
				Points shortest_path;
				if( isTargetReachable(map,next_point.toCell()) ){

					auto shortest_path_cells = findShortestPath(map,cur.toCell(),next_point.toCell(),last_dir,true,true,min_corner_,max_corner_);
					if(!shortest_path_cells.empty())
						plan_path = *cells_generate_points(make_unique<Cells>(shortest_path_cells));
					else
						continue;
				}
				else{
					ROS_INFO("not find shortest_path %d continue",__LINE__);
					continue;
				}

				new_plan_path.clear();
				for(Point_t point:shortest_path){
					ROS_INFO("\033[32m first find short path:(%d,%d)\033[0m",point.toCell().x,point.toCell().y);
					new_plan_path.push_back(point);
				}

				ROS_INFO("target (%d,%d)",next_point.toCell().x,next_point.toCell().y);
				break;
			}while(ros::ok() && plan_path.size() >=1 );

			//-----second put the remaind targets into new_plan_path -----
			//-----if remaind targets in COST_HIGH find shortest path again-----
			ROS_INFO("\033[32m new_plan_path size %lu,the remained points size %lu\033[0m",new_plan_path.size(),plan_path.size());	
			while(ros::ok() && plan_path.size() >=2)
			{
				plan_path.pop_front();
				cur = plan_path.front();
				if(map.getCell(COST_MAP,cur.toCell().x,cur.toCell().y) < COST_HIGH ){
					ROS_INFO("\033[32m push back to new_plan_path :(%d,%d)\033[0m",cur.toCell().x,cur.toCell().y);
					new_plan_path.push_back(cur);
				}
				else{
					do{
						plan_path.pop_front();
						cur = plan_path.front();
						if(map.getCell(COST_MAP,cur.toCell().x,cur.toCell().y) < COST_HIGH)
							break;
					}while(plan_path.size() >= 1);

					if(plan_path.size()<1)
						break;
					next_point = cur;
					cur = new_plan_path.back();
					auto next_dir = cur.dir;
					Points shortest_path;
					if( isTargetReachable(map,next_point.toCell()) ){
						auto shortest_path_cells = findShortestPath(map, cur.toCell(), next_point.toCell(), next_dir, true, true, min_corner_, max_corner_);

						if(!shortest_path_cells.empty())
							shortest_path = *cells_generate_points(make_unique<Cells>(shortest_path_cells));
						else
							continue;
					}
					else{
						ROS_INFO("not find shortest_path %d continue",__LINE__);
						continue;
					}
					ROS_INFO("target (%d,%d)",next_point.toCell().x,next_point.toCell().y);
					for(Point_t point:shortest_path){
						ROS_INFO("\033[32m After first find, short path:(%d,%d)\033[0m",point.toCell().x,point.toCell().y);
						new_plan_path.push_back(point);
					}
				}
			}
			plan_path.clear();
			ROS_INFO("\033[32m new_plan_path size %lu\033[0m",new_plan_path.size());	
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
*/

bool SpotCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path)
{
	//curr = curr;
	//last_dir = last_dir;
	//map = map;
	if(!ev.bumper_triggered && !ev.cliff_triggered && !ev.rcon_status && !ev.lidar_triggered){
		if(!spot_running_){
			spot_running_ = true;
			plan_path = *cells_generate_points(make_unique<Cells>(targets_cells_));
			ROS_INFO("%s,%d ,targets size %lu",__FUNCTION__,__LINE__,plan_path.size());
			return true;
		}
		else if(event_detect_)
		{
			event_detect_ = false;
			if(plan_path_last_.size() > 1 )
			{
				plan_path_last_.pop_front();
				plan_path = plan_path_last_;
				return true;
			}
			else
			{
				INFO_RED("spot end...");
				return false;
			}
		}
		else if(plan_path.size() > 1)
		{
			return true;
		}
		else{
			plan_path.clear();
			INFO_RED("spot end...");
			return false;
		}
	}
	else if(ev.bumper_triggered || ev.cliff_triggered || ev.rcon_status || ev.lidar_triggered)
	{
		if(event_detect_ && plan_path_last_.size()>1 )
		{
			plan_path_last_.pop_front();
			plan_path.clear();
			plan_path.push_back(getPosition());
			plan_path.push_back(plan_path_last_.front());
			return true;
		}
		if(plan_path.size() > 2)
		{
			event_detect_  = true;
			plan_path_last_.clear();
			plan_path_last_ = plan_path ;
			plan_path.clear();
			plan_path.push_back(getPosition());
			plan_path.push_back(plan_path_last_.front());
			plan_path_last_.pop_front();
			return true;
		}
		else
		{
			plan_path.clear();
			INFO_RED("spot end...");
			return false;
		}
	}

}

void SpotCleanPathAlgorithm::initVariables(float radius,Cell_t cur_cell)
{
	spot_running_ = false;
	event_detect_ = false;
	plan_path_last_.clear();
	const int abit = 3;
	int16_t half_cell_num = (int16_t)ceil(radius/CELL_SIZE);
	min_corner_.x = cur_cell.x - half_cell_num - abit;
	min_corner_.y = cur_cell.y - half_cell_num - abit;
	max_corner_.x = cur_cell.x + half_cell_num + abit;
	max_corner_.y = cur_cell.y + half_cell_num + abit;
	targets_cells_.clear();
}

/*
void SpotCleanPathAlgorithm::genTargets(uint8_t sp_type,float radius,Cells *targets,const Cell_t begincell)
{
	targets->clear();
	int16_t i;
	uint32_t mid_it; 
	int16_t x,y;
	int16_t x_last,y_last;
	uint16_t spiral_count = 1;//number of spiral count
	uint16_t cell_number = 1;//cell counter
	uint16_t step = 1;//cell step
	uint16_t radius_cell_n = (uint16_t) ceil(radius * 2  / (CELL_SIZE))/step;//number of spiral
	ROS_INFO( "%s,%d,number of spiral" "\033[36m" " %d" "\033[0m",__FUNCTION__,__LINE__,radius_cell_n);

	if (sp_type == CLOCKWISE)
	{
		for(int OUT = 1;OUT>=0;OUT--)
		{
			// -- push first cell
			mid_it = targets->size();
			x = x_last = begincell.x;
			y = y_last = begincell.y;
			spiral_count = 1;
			cell_number = 1;
			targets->push_back({x,y});
			spiral_count +=step;
			cell_number +=step;
			//-- clockwise out
			while (ros::ok())
			{
				// -- push last cell
				if (spiral_count > radius_cell_n)
				{
					if(radius_cell_n %2 == 0)
					{
						if(OUT){
							x = x + step ;
							targets->push_back({x,y});
						}
					}
					else if(!OUT)
						targets->pop_back();

					break;
				}
				//-- push even spiral cell
				else if ((spiral_count % 2) == 0)
				{
					if(OUT){
						x = x_last - step;
						x_last = x;
					}
					else{
						y = y_last - step;
						y_last = y;
					}
					for (i = 0; i < cell_number; i=i+step)
					{
						if(OUT)
							y = y_last + i;
						else
							x = x_last + i;
						targets->push_back({x,y});
					}
					for (i = step; i < cell_number; i=i+step)
					{
						if(OUT)
							x = x_last + i;
						else
							y = y_last + i;
						targets->push_back({x,y});
					}
				}
				//-- push odd spiral cell
				else
				{
					if(OUT){
						x = x_last + step;
						x_last = x;
					}
					else{
						y = y_last + step;
						y_last = y;
					}
					for (i = 0; i < cell_number; i=i+step)
					{
						if(OUT)
							y = y_last - i;
						else
							x = x_last - i;
						targets->push_back({0,0});
					}
					for (i = step; i < cell_number ; i=i+step)
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
				spiral_count += step;
				cell_number += step;
			}
		}
		std::reverse(targets->begin()+mid_it, targets->end());
	}
	else if (sp_type == ANTI_CLOCKWISE)
	{
		for(int OUT = 1;OUT>=0;OUT--)
		{
			// -- push first cell
			mid_it = targets->size();
			//reset some variable
			x = x_last = begincell.x;
			y = y_last = begincell.y;
			spiral_count = 1;
			cell_number = 1;
			targets->push_back({x,y});
			spiral_count +=1;
			cell_number +=step;
			//anti clockwise out
			while (ros::ok())
			{
				//-- push last cell
				if (spiral_count > radius_cell_n)
				{
					if(radius_cell_n %2 == 0)
					{
						if(OUT){
							x = x - step;
							targets->push_back({x,y});
						}
					}
					else if(!OUT)
						targets->pop_back();
					break;
				}
				// -- push even spiral cell
				else if ((spiral_count % 2) == 0)
				{
					if(OUT){
						x =  x_last + step;
						x_last = x;
					}
					else{
						y = y_last - step;
						y_last = y;
					}
					for (i = 0; i < cell_number; i=i+step)
					{
						if(OUT)
							y = y_last + i;
						else
							x = x_last - i;
						targets->push_back({x,y});
					}
					for (i = step; i < cell_number; i=i+step)
					{
						if(OUT)
							x = x_last - i;
						else
							y = y_last + i;
						targets->push_back({x,y});
					}
				}
				// --push odd spiral cell
				else
				{
					if(OUT){
						x = x_last - step;
						x_last = x;
					}
					else{
						y = y_last + step;
						y_last = y;
					}
					for (i = 0; i < cell_number; i=i+step)
					{
						if(OUT)
							y = y_last - i;
						else
							x = x_last + i;
						targets->push_back({x,y});
					}
					for (i = step; i < cell_number ; i=i+step)
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
				cell_number += step;
			}
		}
		std::reverse(targets->begin()+mid_it, targets->end());
	}
	displayCellPath(*targets);
}
*/

void SpotCleanPathAlgorithm::genTargets(uint8_t sp_type,float radius,Cells *targets,const Cell_t begincell)
{
	sp_type = sp_type;

	//std::initializer_list<Cell_t>  p_path{
		//			{0, 0},{1,0},{2, 0},{2,1},{2, 2},{1,2},{0, 2},
		//			{-1,2},{-2, 2},{-2,1},{-2, 0},{-2,-1},{-2, -2},{-1,-2},{0, -2},
		//			{1,-2},{2, -2},{3,-2},{4, -2},{4,-1},{4, 0},{4,1},{4, 2},{4,3},
			//		{4, 4},{3,4},{2, 4},{1,4},{0, 4},{-1,4},{-2, 4},{-3,4},
				//	{-4, 4},{-4,3},{-4, 2},{-4,1},{-4, 0},{-4,-1},{-4, -2},
					//{-4,-3},{-4, -4},{-3,-4},{-2, -4},{-1,-4},{0, -4},{1,-4},{2, -4},{2,-3},
				//	{2, -2},{2,-1},{2, 0},{2,1},{2, 2},{1,2},{0, 2},{-1,2},
					//{-2, 2},{-2,1},{-2, 0},{-2,-1},{-2, -2},{-1,-2},{0, -2},{1,-2},{2,-2},{2,-1},{2,0},{1,0},{0, 0}
					//};
	std::initializer_list<Cell_t> p_path{
	{0, 0},{1, 0},
	{2, 0},{2, 1},{2, 2},{1, 2},{0, 2},{-1, 2},{-2, 2},{-2, 1},{-2, 0},{-2, -1},{-2, -2},{-1, -2},{0, -2},{1, -2},{2, -2},
	{3, -2},{3, -1},{3, 0},{3, 1},{3, 2},{3, 3},{2, 3},{1, 3},{0, 3},{-1, 3},{-2, 3},{-3, 3},{-3, 2},{-3, 1},{-3, 0},{-3, -1},{-3, -2},{-3, -3},{-2, -3},{-1, -3},{0, -3},{1, -3},{2, -3},{3, -3},
	{4, -3},{4, -2},{4, -1},{4, 0},{4, 1},{4, 2},{4, 3},{4, 4},{3, 4},{2, 4},{1, 4},{0, 4},{-1, 4},{-2, 4},{-3, 4},{-4, 4},{-4, 3},{-4, 2},{-4, 1},{-4, 0},{-4, -1},{-4, -2},{-4, -3},{-4, -4},{-3, -4},{-2, -4},{-1, -4},{0, -4},{1, -4},{2, -4},{3, -4},{4, -4},{4, -3},{4, -2},{4, -1},{4, 0},{4, 1},{4, 2},{4, 3},
	{3, 3},{2, 3},{1, 3},{0, 3},{-1, 3},{-2, 3},{-3, 3},{-3, 2},{-3, 1},{-3, 0},{-3, -1},{-3, -2},{-3, -3},{-2, -3},{-1, -3},{0, -3},{1, -3},{2, -3},{3, -3},{3, -2},{3, -1},{3, 0},{3, 1},{3, 2},
	{2, 2},{1, 2},{0, 2},{-1, 2},{-2, 2},{-2, 1},{-2, 0},{-2, -1},{-2, -2},{-1, -2},{0, -2},{1, -2},{2, -2},{2, -1},{2, 0},{2, 1},
	{1, 1},{0, 1},{-1, 1},{-1, 0},{-1, -1},{0, -1},{1, -1},{1, 0},
	{0, 0}
	};
	std::initializer_list<Cell_t>::iterator p_it = p_path.begin();
	for(int i=0;i <p_path.size();i++){
		targets->push_back(*(p_it+i) + begincell);
		//ROS_INFO("%s,%d,path list (%d,%d)",__FUNCTION__,__LINE__,targets->back().GetX(),targets->back().GetY());
	}

}

bool SpotCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	return checkTrappedUsingDijkstra(map, curr_cell);
}

