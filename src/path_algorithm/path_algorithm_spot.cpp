#include <ros/ros.h>
#include "pp.h"
#include "path_algorithm.h"
#include "arch.hpp"

#define CLOCKWISE 1
#define ANTI_CLOCKWISE 2

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm()
{
	initVariables(1.0,getPosition().toCell());
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

bool SpotCleanPathAlgorithm::generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &plan_path)
{
	if(!ev.bumper_triggered && !ev.cliff_triggered && !ev.rcon_triggered){
		if(!spot_running_){
			spot_running_ = true;
			plan_path = cells_generate_points(targets_cells_);
			ROS_INFO("targets size %d",plan_path.size());
			return true;
		}
		else if(plan_path.size() >= 2){
			INFO_CYAN("maybe over the tmp target");
			plan_path = plan_path;
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
			/*---first find shortest path---*/
			Point32_t next_point;
			do{
				plan_path.pop_front();
				next_point = plan_path.front();
				auto shortest_path_cells = findShortestPath(map,cur.toCell(),next_point.toCell(),last_dir,true,true,min_corner_,max_corner_);
				auto shortest_path = cells_generate_points(shortest_path_cells);
				if(shortest_path.empty()){
					ROS_INFO("not find shortest_path %d continue",__LINE__);
					continue;
				}
				else{
					new_plan_path.clear();
					for(Point32_t point:shortest_path){
						ROS_INFO("\033[32m first find short path:(%d,%d)\033[0m",point.toCell().x,point.toCell().y);
						new_plan_path.push_back(point);
					}
					ROS_INFO("target (%d,%d)",next_point.toCell().x,next_point.toCell().y);
					break;
				}
			}while(ros::ok() && plan_path.size() >=1 );

			/*-----second put the remaind targets into new_plan_path -----*/
			/*-----if remaind targets in COST_HIGH find shortest path again-----*/
			ROS_INFO("\033[32m new_plan_path size %d,the remained points size %d\033[0m",new_plan_path.size(),plan_path.size());	
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
					auto next_dir = cur.th;
					auto shortest_path_cells = findShortestPath(map,cur.toCell(),next_point.toCell(),next_dir,true,true,min_corner_,max_corner_);
					auto shortest_path = cells_generate_points(shortest_path_cells);
					if(shortest_path.empty()){
						ROS_INFO("not find shortest_path %d continue",__LINE__);
					}
					else{
						ROS_INFO("target (%d,%d)",next_point.toCell().x,next_point.toCell().y);
						for(Point32_t point:shortest_path){
							ROS_INFO("\033[32m After first find, short path:(%d,%d)\033[0m",point.toCell().x,point.toCell().y);
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
	int16_t half_cell_num = (int16_t)ceil(spot_diameter_*1000/CELL_SIZE)/2;
	min_corner_.x = begin_cell_.x - half_cell_num - 3;
	min_corner_.y = begin_cell_.y - half_cell_num - 3;
	max_corner_.x = begin_cell_.x + half_cell_num + 3;
	max_corner_.y = begin_cell_.y + half_cell_num + 3;
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
	uint16_t step = 1;
	uint16_t spiral_number = (uint16_t) ceil(diameter * 1000 / (CELL_SIZE))/step;//number of spiral
	ROS_INFO( "%s,%d,number of spiral" "\033[36m" " %d" "\033[0m",__FUNCTION__,__LINE__,spiral_number);
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
			targets->push_back({x,y});
			spiral_count +=step;
			cell_number +=step;
			while (ros::ok()) //clockwise out
			{
				if (spiral_count > spiral_number)
				{
					if(spiral_number %2 == 0)
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
			mid_it = targets->size();
			//reset some variable
			x = x_last = begincell.x;
			y = y_last = begincell.y;
			spiral_count = 1;
			cell_number = 1;
			targets->push_back({x,y});
			spiral_count +=1;
			cell_number +=step;
			while (ros::ok()) //anti clockwise out
			{
				if (spiral_count > spiral_number)
				{
					if(spiral_number %2 == 0)
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

bool SpotCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	checkTrappedUsingDijkstra(map, curr_cell);
}

/*
 * @param1 map
 * @param2 current cell
 * @param3 target cell
 * @param4 old direction
 * boundary,Limit to the search range between cur(x, y) and target(x, y)
 * @param6 used unkown cell(witch not clean or occupies) while create COST_MAP
 */
Cells SpotCleanPathAlgorithm::spotFindShortestPath(GridMap& map, Cell_t cur, Cell_t target,int old_dir, bool used_unknow)
{
	int16_t i, j, m, n;
	int16_t tracex, tracey, tracex_tmp, tracey_tmp, passValue, nextPassValue, passSet, offset;

	Cells shortest_path{};

	int16_t end_x = target.x;
	int16_t end_y = target.y;
	int16_t curr_x = cur.x;
	int16_t curr_y = cur.y;

	int16_t half_cell_num = (int16_t)ceil(spot_diameter_*1000/CELL_SIZE)/2;
	int16_t x_min = begin_cell_.x - half_cell_num - 3;
	int16_t y_min = begin_cell_.y - half_cell_num - 3;
	int16_t x_max = begin_cell_.x + half_cell_num + 3;
	int16_t y_max = begin_cell_.y + half_cell_num + 3;

	ROS_INFO("%s,%d,spot_begin_cell(%d,%d),target(%d,%d),x_min,y_min(%d,%d),x_max,y_max(%d,%d)",__FUNCTION__,__LINE__,begin_cell_.x,begin_cell_.y,target.x,target.y,x_min,y_min,x_max,y_max);

	INFO_GREEN("create COST_MAP");
	/* Reset the cells in the shorest path costmap. */
	for (i = x_min - 1; i <= x_max + 1; ++i) {
		for (j = y_min - 1; j <= y_max + 1; ++j) {
			map.setCell(COST_MAP, (int32_t) i, (int32_t) j, COST_NO);
		}
	}

	/* Marked the obstcals to the shorest path costmap. */
	for (i = x_min - 1; i <= x_max + 1; ++i) {
		for (j = y_min - 1; j <= y_max + 1; ++j) {
			CellState cs = map.getCell(CLEAN_MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				//for (m = ROBOT_RIGHT_OFFSET + 1; m <= ROBOT_LEFT_OFFSET - 1; m++) {
				for (m = ROBOT_RIGHT_OFFSET; m <= ROBOT_LEFT_OFFSET; m++) {
					for (n = ROBOT_RIGHT_OFFSET; n <= ROBOT_LEFT_OFFSET; n++) {
						map.setCell(COST_MAP, (int32_t) (i + m), (int32_t) (j + n), COST_HIGH);
					}
				}
			}
			else if(cs == UNCLEAN && used_unknow)
				map.setCell(COST_MAP, (int32_t) (i), (int32_t) (j), COST_HIGH);
		}
	}

	// Set for target cell. For reverse algorithm, we will generate a-star map from target cell.
	map.setCell(COST_MAP, target.x, target.y, COST_1);

	// For protection, the start cell must be reachable.
	if (map.getCell(COST_MAP, cur.x, cur.y) == COST_HIGH)
	{
		ROS_ERROR("%s %d: Start cell has high cost(%d)! It may cause bug, please check.",
							__FUNCTION__, __LINE__, map.getCell(COST_MAP, cur.x, cur.y));
		map.print(COST_MAP, target.x, target.y);
		map.setCell(COST_MAP, cur.x, cur.y, COST_NO);
	}

	offset = 0;
	passSet = 1;
	passValue = 1;
	nextPassValue = 2;
	
	INFO_GREEN("optimise COST_MAP");
	while (map.getCell(COST_MAP, curr_x, curr_y) == COST_NO && passSet == 1)
	{
		offset++;
		passSet = 0;

		/*
		 * The following 2 for loops is for optimise the computational time.
		 * Since there is not need to go through the whole costmap for seaching the
		 * cell that have the next pass value.
		 *
		 * It can use the offset to limit the range of searching, since in each loop
		 * the cell (x -/+ offset, y -/+ offset) would be set only. The cells far away
		 * to the robot position won't be set.
		 */
		for (i = end_x - offset; i <= end_x + offset; i++) {
			if (i < x_min || i > x_max)
				continue;

			for (j = end_y - offset; j <= end_y + offset; j++) {
				if (j < y_min || j > y_max)
					continue;

				/* Found a cell that has a pass value equal to the current pass value. */
				if(map.getCell(COST_MAP, i, j) == passValue) {
					/* Set the lower cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i - 1, j) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) (i - 1), (int32_t) j, (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the upper cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i + 1, j) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) (i + 1), (int32_t) j, (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the cell on the right hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i, j - 1) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) i, (int32_t) (j - 1), (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the cell on the left hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i, j + 1) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) i, (int32_t) (j + 1), (CellState) nextPassValue);
						passSet = 1;
					}
				}
			}
		}

		/* Update the pass value. */
		passValue = nextPassValue;
		nextPassValue++;

		/* Reset the pass value, pass value can only between 1 to 5. */
		if(nextPassValue == COST_PATH)
			nextPassValue = 1;
	}

	/* The target position still have a cost of 0, which mean it is not reachable. */
	if (map.getCell(COST_MAP, end_x, end_y) == COST_NO || map.getCell(COST_MAP, end_x, end_y) == COST_HIGH) {
		ROS_WARN("%s, %d: target point (%d, %d) is not reachable(%d), return -2.", __FUNCTION__, __LINE__, end_x, end_y,
				 map.getCell(COST_MAP, end_x, end_y));
#if	DEBUG_COST_MAP
		map.print(COST_MAP, end_x, end_y);
#endif
		return shortest_path;
	}
	else{
#if	DEBUG_COST_MAP
		map.print(COST_MAP, end_x, end_y);
#endif
	}
	/*
	 * Start from the target position, trace back the path by the cost level.
	 * Value of cells on the path is set to 6. Stops when reach the current
	 * robot position.
	 *
	 * The last robot direction is use, this is to avoid using the path that
	 * have the same direction as previous action.
	 */
	Cell_t t;
	t.x = tracex = tracex_tmp = curr_x;
	t.y = tracey = tracey_tmp = curr_y;
	shortest_path.push_back(t);

	uint16_t next = 0;
	int16_t	totalCost=0;
	int16_t costAtCell, targetCost; 
	int16_t dest_dir = (old_dir == MAP_POS_Y || old_dir == MAP_NEG_Y) ? 1: 0;
	ROS_INFO("%s %d: dest dir: %d", __FUNCTION__, __LINE__, dest_dir);

	while (tracex != end_x || tracey != end_y)
	{
		costAtCell = map.getCell(COST_MAP, tracex, tracey);
		targetCost = costAtCell - 1;

		/* Reset target cost to 5, since cost only set from 1 to 5 in the shorest path costmap. */
		if (targetCost == 0)
			targetCost = COST_5;

		/* Set the cell value to 6 if the cells is on the path. */
		map.setCell(COST_MAP, (int32_t) tracex, (int32_t) tracey, COST_PATH);

#define COST_SOUTH	{											\
				if (next == 0 && (map.getCell(COST_MAP, tracex - 1, tracey) == targetCost)) {	\
					tracex--;								\
					next = 1;								\
					dest_dir = 1;								\
				}										\
			}

#define COST_WEST	{											\
				if (next == 0 && (map.getCell(COST_MAP, tracex, tracey - 1) == targetCost)) {	\
					tracey--;								\
					next = 1;								\
					dest_dir = 0;								\
				}										\
			}

#define COST_EAST	{											\
				if (next == 0 && (map.getCell(COST_MAP, tracex, tracey + 1) == targetCost)) {	\
					tracey++;								\
					next = 1;								\
					dest_dir = 0;								\
				}										\
			}

#define COST_NORTH	{											\
				if (next == 0 && map.getCell(COST_MAP, tracex + 1, tracey) == targetCost) {	\
					tracex++;								\
					next = 1;								\
					dest_dir = 1;								\
				}										\
			}

		next = 0;
		if (dest_dir == 0) {
			COST_WEST
			COST_EAST
			COST_SOUTH
			COST_NORTH
		} else {
			COST_SOUTH
			COST_NORTH
			COST_WEST
			COST_EAST
		}

#undef COST_EAST
#undef COST_SOUTH
#undef COST_WEST
#undef COST_NORTH

		totalCost++;
		if (shortest_path.back().x != tracex && shortest_path.back().y != tracey) {
			t.x = tracex_tmp;
			t.y = tracey_tmp;
			shortest_path.push_back(t);
		}
		tracex_tmp = tracex;
		tracey_tmp = tracey;
	}

	map.setCell(COST_MAP, (int32_t) tracex, (int32_t) tracey, COST_PATH);

	t.x = tracex_tmp;
	t.y = tracey_tmp;
	shortest_path.push_back(t);

	displayCellPath(shortest_path);
	return shortest_path;
}
