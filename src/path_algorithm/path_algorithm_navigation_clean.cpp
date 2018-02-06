//
// Created by lsy563193 on 12/13/17.
//

#include <dev.h>
#include "ros/ros.h"
#include "robot.hpp"
#include "path_algorithm.h"

extern int g_follow_last_follow_wall_dir;
bool NavCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path)
{

	plan_path.clear();
	auto curr_cell = curr.toCell();
	//Step 1: Find possible plan_path in same lane.
	auto plan_path_cell = findTargetInSameLane(map, curr_cell);
	if (!plan_path_cell.empty())
	{
		plan_path = cells_generate_points(plan_path_cell);
		// Congratulation!! plan_path is generated successfully!!
		map.print(CLEAN_MAP, plan_path_cell.back().x, plan_path_cell.back().y);
		return true;
	}

	//Step 2: Find all possible plan_path at the edge of cleaned area and filter plan_path in same lane.

	// Copy map to a BoundingBox2 type b_map.
	BoundingBox2 b_map;
	auto b_map_temp = map.generateBound();

	for (const auto &cell : b_map_temp) {
		if (map.getCell(CLEAN_MAP, cell.x, cell.y) != UNCLEAN)
			b_map.Add(cell);
	}

	Cells targets = filterAllPossibleTargets(map, curr_cell, b_map);

	//Step 3: Generate the COST_MAP for map and filter plan_path that are unreachable.
	map.generateSPMAP(curr_cell, targets);

	targets.erase(std::remove_if(targets.begin(), targets.end(),[&map](Cell_t it){
		auto cost = map.getCell(COST_MAP, it.x, it.y);
		return cost == COST_NO || cost == COST_HIGH;
	}),targets.end());
	ROS_INFO("%s %d: After generating COST_MAP, Get %lu reachable plan_path.", __FUNCTION__, __LINE__, targets.size());
	if (targets.size() != 0)
		displayTargetList(targets);
	else
		// Now plan_path is empty.
		return false;

	//Step 4: Trace back the path of these plan_path in COST_MAP.
	PathList paths_for_reachable_targets = tracePathsToTargets(map, targets, curr_cell);

	//Step 5: Filter paths to get the best target.
	Cell_t best_target;
	if (!filterPathsToSelectTarget(map, paths_for_reachable_targets, curr_cell, best_target))
		// Now plan_path is empty.
		return false;

	//Step 6: Find shortest path for this best target.
	Cell_t corner1,corner2;
	Cells shortest_path = findShortestPath(map, curr_cell, best_target, last_dir, true,false,corner1,corner2);
	if (shortest_path.empty())
		// Now plan_path is empty.
		return false;

	//Step 7: Optimize path for adjusting it away from obstacles..
	optimizePath(map, shortest_path);
	map.print(CLEAN_MAP, shortest_path.back().x, shortest_path.back().y);

	//Step 8: Fill path with direction.
	plan_path = cells_generate_points(shortest_path);

	// Congratulation!! plan_path is generated successfully!!
//	plan_path_cell = shortest_path;

	return true;
}

Cells NavCleanPathAlgorithm::findTargetInSameLane(GridMap &map, const Cell_t &curr_cell)
{
	int8_t is_found = 0;
	Cell_t it[2]; // it[0] means the furthest cell of x positive direction, it[1] means the furthest cell of x negative direction.

//	map.print(CLEAN_MAP, 0, 0);
	for (auto i = 0; i < 2; i++) {
		it[i] = curr_cell;
		auto unclean_cells = 0;

		for (Cell_t neighbor = it[i] + cell_direction_index_[i];
				 !map.cellIsOutOfRange(neighbor + cell_direction_index_[i]) && !map.isBlocksAtY(neighbor.x, neighbor.y);
				 neighbor += cell_direction_index_[i])
		{
			unclean_cells += map.isUncleanAtY(neighbor.x, neighbor.y);
			if (unclean_cells >= 3) {
				it[i] = neighbor;
				unclean_cells = 0;
//				ROS_INFO("%s %d: it[%d](%d,%d)", __FUNCTION__, __LINE__, i, it[i].x, it[i].y);
			}
//			ROS_WARN("%s %d: it[%d](%d,%d)", __FUNCTION__, __LINE__, i, it[i].x, it[i].y);
//			ROS_WARN("%s %d: neighbor(%d,%d)", __FUNCTION__, __LINE__, neighbor.x, neighbor.y);
		}
	}

	Cell_t target;
	if (it[0].x != curr_cell.x)
	{
		target = it[0];
		is_found++;
	}
	if (it[1].x != curr_cell.x)
	{
		target = it[1];
		is_found++;
	}
//	ROS_WARN("%s %d: curr(%d,%d) is_found(%d),it(%d,%d)", __FUNCTION__, __LINE__, curr_cell.x, curr_cell.y,is_found,it[0],it[1]);
	if (is_found == 2)
	{
		// Select the nearest side.
		if (std::abs(curr_cell.x - it[0].x) < std::abs(curr_cell.x - it[0].x))
			target = it[0];

		//todo
//			ROS_ERROR("%s %d: 1 g_follow_last_follow_wall_dir(%d)", __FUNCTION__, __LINE__, g_follow_last_follow_wall_dir);
		if(g_follow_last_follow_wall_dir!=0)
		{
			beeper.beepForCommand(VALID);
			ROS_ERROR("%s %d: g_follow_last_follow_wall_dir(%d)", __FUNCTION__, __LINE__, g_follow_last_follow_wall_dir);
			if(g_follow_last_follow_wall_dir == 1)
				target = it[1];
			else//(g_follow_last_follow_wall_dir == 2)
				target = it[0];
		}
	}
	g_follow_last_follow_wall_dir = 0;
	Cells path{};
	if (is_found)
	{
		path.push_front(target);
		path.push_front(getPosition().toCell());
//		ROS_INFO("%s %d:curr(%d,%d) x pos:(%d,%d), x neg:(%d,%d), target:(%d,%d),is_found(%d)", __FUNCTION__, __LINE__,curr_cell.x,curr_cell.y, it[0].x, it[0].y, it[1].x, it[1].y, target.x, target.y,is_found);
//		map.print(CLEAN_MAP, target.x, target.y);
	}
	else
		ROS_INFO("%s %d: x pos:(%d,%d), x neg:(%d,%d), target not found.", __FUNCTION__, __LINE__, it[0].x, it[0].y, it[1].x, it[1].y);

	return path;
}
class FilterTarget{
public:
	FilterTarget(const Cell_t& curr){
		curr_ = curr;
	}
	void operator()(const Cell_t &it)
	{
		if(!tmps_.empty()) {
			if (it.y != tmps_.back().y) {
				is_continue_ = false;
			}
			else if (it.x - tmps_.back().x == 1) {
				if (is_continue_ && curr_ != it)
					tmps_.pop_back();
				is_continue_ = curr_ != it;
			}
		}
		tmps_.push_back(it);
	}
	operator Cells()
	{
		return tmps_;
	}

private:
	bool is_continue_{};
	Cells tmps_{};
	Cell_t curr_{};
};

Cells NavCleanPathAlgorithm::filterAllPossibleTargets(GridMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map)
{
	ROS_INFO("%s %d: Find all possible targets.", __FUNCTION__, __LINE__);
	Cells targets{};

	auto b_map_copy = b_map;
	b_map_copy.pos_ = b_map.min;

	// Check all boundarys between cleaned cells and unclean cells.
	for (const auto &cell : b_map_copy) {
		if (map.getCell(CLEAN_MAP, cell.x, cell.y) != CLEANED /*|| std::abs(cell.y % 2) == 1*/)
			continue;

		Cell_t neighbor;
		for (auto i = 0; i < 4; i++) {
			neighbor = cell + cell_direction_index_[i];
			if (map.getCell(CLEAN_MAP, neighbor.x, neighbor.y) == UNCLEAN && map.isBlockAccessible(neighbor.x, neighbor.y))
				targets.push_back(neighbor);
		}
	}

	std::sort(targets.begin(),targets.end(),[](Cell_t l,Cell_t r){
		return (l.y < r.y || (l.y == r.y && l.x < r.x));
	});

	displayTargetList(targets);

	targets = std::for_each(targets.begin(), targets.end(),FilterTarget(curr_cell));

	displayTargetList(targets);

	return targets;
}

PathList NavCleanPathAlgorithm::tracePathsToTargets(GridMap &map, const Cells &target_list, const Cell_t& start)
{
	PathList paths{};
	int16_t cost, x_min, x_max, y_min, y_max;
	map.getMapRange(COST_MAP, &x_min, &x_max, &y_min, &y_max);
	for (auto& it : target_list) {
		auto trace = it;
		Cells path{};
		//Trace the path for this target 'it'.
		while (trace != start) {
			cost = map.getCell(COST_MAP, trace.x, trace.y) - 1;
			if (cost == 0)
				cost = COST_5;

			path.push_front(trace);
			for(auto i =0; i<4 ; i++)
			{
				auto neighbor = trace - cell_direction_index_[i];
				if(!((neighbor).x >= x_min && neighbor.x <= x_max && (neighbor).y >= y_min && neighbor.y <= y_max))
					continue;

				if (map.getCell(COST_MAP, neighbor.x, neighbor.y) == cost) {
					trace = neighbor;
					break;
				}
			}
		}
		path.push_front(trace);
		paths.push_back(path);
	}

	return paths;
}


class IsIncrease {
public:
	IsIncrease(bool is_reverse=false):is_reverse_(is_reverse){};

	int operator()(const Cell_t &a, const Cell_t &b) {
//		printf("b(%d,%d),a(%d,%d)\n",b.x, b.y,b.x, a.y);
		return (is_reverse_) ? a.y > b.y : a.y < b.y ;
//		std::cout<<"return true"<<std::endl;
	};
private:
	bool is_reverse_=false;
};

class BestTargetFilter {
public:
	BestTargetFilter(int16_t min_y,int16_t max_y,int turn_count=0,bool is_reverse=false):min_y_(min_y),max_y_(max_y),turn_count_(turn_count),is_reverse_(is_reverse) {};

	int operator()(const Cells &path) {
		if (path.back().y < min_y_ || path.back().y > max_y_)
			return false;
		if(turn_count_ ==0) {
			return std::is_sorted(path.begin(), path.end(), IsIncrease(is_reverse_));
		}
		else if(turn_count_ == 1){
			auto point = std::is_sorted_until(path.begin(), path.end(), IsIncrease(!is_reverse_));
			return point != path.end() && std::is_sorted(point, path.end(), IsIncrease(is_reverse_));
		}
		else/* if(turn_count_ ==-1)*/{//any turn
			return true;
		}
	};
//private:
//	bool is_reverse_=false;
	int16_t min_y_;
	int16_t max_y_;
	int turn_count_;
	bool is_reverse_{};
};

class MinYAndShortestPath {
public:
	MinYAndShortestPath(int16_t curr_y, bool is_reverse,int turn_count):is_reverse_(is_reverse),curr_y_(curr_y),turn_count_(turn_count){ };
	bool operator()(const Cells &path_a, const Cells &path_b) {
		if(turn_count_ == 0 || turn_count_ == 1000)
		{
			if(path_a.back().y == path_b.back().y)
				return path_a.size() < path_b.size();
			return (std::abs(path_a.back().y - curr_y_) < std::abs(path_b.back().y - curr_y_));
		}
		else if(turn_count_ == 1){
			auto top_a = std::min_element(path_a.begin(), path_a.end(),IsIncrease(is_reverse_));
			auto top_b = std::min_element(path_b.begin(), path_b.end(),IsIncrease(is_reverse_));
//			printf("top a(%d,%d)\n", top_a->x, top_a->y);
//			printf("top b(%d,%d)\n", top_b->x, top_b->y);
			if(top_a->y == top_b->y) {
				if (path_a.back().y == path_b.back().y)
					return path_a.size() < path_b.size();
				return (std::abs(path_a.back().y - top_a->y) < std::abs(path_b.back().y - top_b->y));
			}else
			return (std::abs(top_a->y - curr_y_) < std::abs(top_b->y - curr_y_));
		}else /*if(turn_count_ == 1000)*/{
			return path_a.size() < path_b.size();
		}
	};
	bool is_reverse_{};
	int16_t curr_y_{};
	int turn_count_;
};

bool NavCleanPathAlgorithm::filterPathsToSelectTarget(GridMap &map, PathList &paths, const Cell_t &cell_curr, Cell_t &best_target) {
	std::deque<BestTargetFilter> filters{};
	Cell_t min_cell,max_cell;
	map.getMapRange(CLEAN_MAP,&min_cell.x, &max_cell.x,&min_cell.y,&max_cell.y);
	BestTargetFilter filters_case1{static_cast<int16_t>(cell_curr.y + 2), max_cell.y, 0, false};
	BestTargetFilter filters_case1_1{cell_curr.y, static_cast<int16_t>(cell_curr.y + 1), 0, false};
	BestTargetFilter filters_case2{min_cell.y, max_cell.y,1,false};
	BestTargetFilter filters_case3{min_cell.y, static_cast<int16_t>(cell_curr.y - 2), 0, true};
	BestTargetFilter filters_case3_1{static_cast<int16_t>(min_cell.y - 1), cell_curr.y, 0, true};
	BestTargetFilter filters_case4{min_cell.y, max_cell.y,1,true};
	BestTargetFilter filters_case5{cell_curr.y, max_cell.y, 1000,false};
	BestTargetFilter filters_case6{min_cell.y, max_cell.y, 1000,true};
	filters.push_back(filters_case1);
	filters.push_back(filters_case1_1);

	if(!map.getCell(CLEAN_MAP,cell_curr.x,cell_curr.y-2) == UNCLEAN)
		filters.push_back(filters_case2);
	filters.push_back(filters_case3);
	filters.push_back(filters_case3_1);
	filters.push_back(filters_case4);
	filters.push_back(filters_case5);
	filters.push_back(filters_case6);

	for (auto &&filter : filters) {
		printf("is towards Y+(%d),y_range(%d,%d),allow turn count(%d)\n",!filter.is_reverse_,filter.min_y_,filter.max_y_,filter.turn_count_);
		PathList filtered_paths{};
		std::copy_if(paths.begin(), paths.end(), std::back_inserter(filtered_paths), BestTargetFilter(filter));
		if (!filtered_paths.empty()) {
//			std::sort(filtered_paths.begin(), filtered_paths.end(), );
			auto best_path = std::min_element(filtered_paths.begin(), filtered_paths.end(), MinYAndShortestPath(cell_curr.y, filter.is_reverse_,filter.turn_count_));
			printf("cell: ");
			printf("best_path: ");
			for (auto &&cell : *best_path) {
				printf("{%d,%d},",cell.x,cell.y);
			}
			printf("\n");
			best_target =  best_path->back();
			printf("best_target(%d,%d)\n", best_target.x, best_target.y);
			return true;
		}
	}
	return false;
}

bool NavCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	return checkTrappedUsingDijkstra(map, curr_cell);
}
