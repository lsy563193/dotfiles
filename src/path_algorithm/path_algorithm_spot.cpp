#include <ros/ros.h>
#include "pp.h"
#include "path_algorithm.h"
#include "arch.hpp"

#define CLOCKWISE 1
#define ANTI_CLOCKWISE 2

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm()
{
	initVariables(0.8,getPosition().toCell());
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
	auto curr_cell = curr.toCell();
	if(!ev.bumper_triggered && !ev.cliff_triggered && !ev.rcon_triggered){
		if(!spot_running_){
			spot_running_ = true;
			plan_path = cells_generate_points(targets_cells_);
		}
		else{
			if(targets_last_points_.size() >= 1 && block_event_){
				block_event_ = false;
				refactorTargets(map, &targets_last_points_);
				plan_path = targets_last_points_;
			}
			else{
				plan_path.clear();
				ROS_INFO("spot end...");
				return false;
			}
		}
		ROS_INFO("targets size %d",plan_path.size());
		return true;
	}
	else if(ev.bumper_triggered || ev.cliff_triggered || ev.rcon_triggered)
	{
		PP_INFO();
		block_event_ = true;
		bool ret = false;
		Cells shortest_path;
		if(targets_last_points_.empty())
		{
			while(ros::ok())
			{
				if(plan_path.size() >= 1){
					plan_path.pop_front();
					auto next_cell = plan_path.front().toCell();
					shortest_path = findShortestPath(map,curr_cell,next_cell,last_dir,false);
					if(shortest_path.empty()){
						continue;
					}
					else{
						ret = true;
						targets_last_points_ = plan_path;
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
				if(targets_last_points_.size() >= 1)
				{
					targets_last_points_.pop_front();
					auto next_cell = targets_last_points_.front().toCell();
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
		plan_path = cells_generate_points(shortest_path);
		return ret;
	}
}

void SpotCleanPathAlgorithm::initVariables(float diameter,Cell_t cur_cell)
{
	PP_INFO();
	spot_diameter_ = diameter;
	spot_running_ = false;
	block_event_ = false;
	begin_cell_ = cur_cell;
	targets_last_points_.clear();
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

void SpotCleanPathAlgorithm::giveMeCleanPoint(GridMap map,Point32_t &point)
{
	Cell_t cell = point.toCell();
	if(map.getCell(COST_MAP,cell.x,cell.y) >= COST_HIGH){
		if(isXAxis((MapDirection)point.th)){
			if(isPos((MapDirection)point.th))
				point.y = cellToCount(cell.y+1);
			else
				point.y = cellToCount(cell.y-1);

		}
		else if(isYAxis((MapDirection)point.th)){
			if(isPos((MapDirection)point.th))
				point.x = cellToCount(cell.x-1);
			else
				point.x = cellToCount(cell.x+1);
		}
	}
}

void SpotCleanPathAlgorithm::refactorTargets(GridMap map,Points *targets)
{

	ROS_INFO("%s,%d",__FUNCTION__,__LINE__);
	// Get the map range.
	int16_t x_min, x_max, y_min, y_max;
	map.getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);
	// Reset the COST_MAP.
	map.reset(COST_MAP);
	// Mark obstacles in COST_MAP
	for (int16_t i = x_min - 1; i <= x_max + 1; ++i) {
		for (int16_t j = y_min - 1; j <= y_max + 1; ++j) {
			CellState cs = map.getCell(CLEAN_MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				//for (m = ROBOT_RIGHT_OFFSET + 1; m <= ROBOT_LEFT_OFFSET - 1; m++)
				for (int16_t m = ROBOT_RIGHT_OFFSET; m <= ROBOT_LEFT_OFFSET; m++) {
					for (int16_t n = ROBOT_RIGHT_OFFSET; n <= ROBOT_LEFT_OFFSET; n++) {
						map.setCell(COST_MAP, (i + m), (j + n), COST_HIGH);
					}
				}
			}
		}
	}
	map.print(COST_MAP,getPosition().toCell().x,getPosition().toCell().y);

	Points::iterator it;
	for( it= targets->begin();it!=targets->end();it++){
		giveMeCleanPoint(map, *it);
	}
	displayPointPath(*targets);
}
