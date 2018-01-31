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

	Cells filtered_targets = filterAllPossibleTargets(map, curr_cell, b_map);

	//Step 3: Generate the COST_MAP for map and filter plan_path that are unreachable.
	Cells reachable_targets = getReachableTargets(map, curr_cell, filtered_targets);
	ROS_INFO("%s %d: After generating COST_MAP, Get %lu reachable plan_path.", __FUNCTION__, __LINE__, reachable_targets.size());
	if (reachable_targets.size() != 0)
		displayTargetList(reachable_targets);
	else
		// Now plan_path is empty.
		return false;

	//Step 4: Trace back the path of these plan_path in COST_MAP.
	PathList paths_for_reachable_targets = tracePathsToTargets(map, reachable_targets, curr_cell);

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
			beeper.play_for_command(VALID);
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

Cells NavCleanPathAlgorithm::filterAllPossibleTargets(GridMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map)
{
	ROS_INFO("%s %d: Find all possible targets.", __FUNCTION__, __LINE__);
	Cells possible_target_list{};

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
				possible_target_list.push_back(neighbor);
		}
	}

	std::sort(possible_target_list.begin(),possible_target_list.end(),[](Cell_t l,Cell_t r){
		return (l.y < r.y || (l.y == r.y && l.x < r.x));
	});

	displayTargetList(possible_target_list);

	ROS_INFO("%s %d: Filter targets in the same line.", __FUNCTION__, __LINE__);
	Cells filtered_targets{};
	/* Filter the targets. */
	for(;!possible_target_list.empty();) {
		auto y = possible_target_list.front().y;
		Cells tmp_list{};
		std::remove_if(possible_target_list.begin(), possible_target_list.end(), [&y, &tmp_list](Cell_t &it) {
			if (it.y == y && (tmp_list.empty() || (it.x - tmp_list.back().x == 1))) {
				tmp_list.push_back(it);
				return true;
			}
			return false;
		});
		possible_target_list.resize(possible_target_list.size() - tmp_list.size());
		if (tmp_list.size() > 2) {
			tmp_list.erase(std::remove_if(tmp_list.begin() + 1, tmp_list.end() - 1, [&curr_cell](Cell_t &it) {
				return it.x != curr_cell.x;
			}), tmp_list.end() - 1);
		}
		for(const auto& it:tmp_list){
			filtered_targets.push_back(it);
		};
//		ROS_WARN("~%s %d: Filter targets in the same line.", __FUNCTION__, __LINE__);
	}

	displayTargetList(filtered_targets);

	return filtered_targets;
}

Cells NavCleanPathAlgorithm::getReachableTargets(GridMap &map, const Cell_t &curr_cell, Cells &possible_targets)
{
	map.generateSPMAP(curr_cell, possible_targets);
	Cells reachable_targets{};
	for (auto it = possible_targets.begin(); it != possible_targets.end();)
	{
		CellState it_cost = map.getCell(COST_MAP, it->x, it->y);
		if (it_cost == COST_NO || it_cost == COST_HIGH)
		{
			it = possible_targets.erase(it);
			continue;
		}
		else
		{
			reachable_targets.push_back(*it);
			it++;
		}
	}
	return reachable_targets;
}

PathList NavCleanPathAlgorithm::tracePathsToTargets(GridMap &map, const Cells &target_list, const Cell_t& start)
{
	PathList paths{};
	int16_t trace_cost, x_min, x_max, y_min, y_max;
	map.getMapRange(COST_MAP, &x_min, &x_max, &y_min, &y_max);
	for (auto& it : target_list) {
		auto trace = it;
		Cells path{};
		//Trace the path for this target 'it'.
		while (trace != start) {
			trace_cost = map.getCell(COST_MAP, trace.x, trace.y) - 1;

			if (trace_cost == 0) {
				trace_cost = COST_5;
			}

			path.push_front(trace);

			if ((trace.x - 1 >= x_min) && (map.getCell(COST_MAP, trace.x - 1, trace.y) == trace_cost)) {
				trace.x--;
				continue;
			}

			if ((trace.x + 1 <= x_max) && (map.getCell(COST_MAP, trace.x + 1, trace.y) == trace_cost)) {
				trace.x++;
				continue;
			}

			if ((trace.y - 1 >= y_min) && (map.getCell(COST_MAP, trace.x, trace.y - 1) == trace_cost)) {
				trace.y--;
				continue;
			}

			if ((trace.y + 1 <= y_max) && (map.getCell(COST_MAP, trace.x, trace.y + 1) == trace_cost)) {
				trace.y++;
				continue;
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
		return (!is_reverse_) ? a.y < b.y : a.y > b.y ;
//		std::cout<<"return true"<<std::endl;
	};
private:
	bool is_reverse_=false;
};

class BestTargetFilter {
public:
	BestTargetFilter(int16_t min_y,int16_t max_y,int turn_count=0,bool is_revease=false):min_y_(min_y),max_y_(max_y),turn_count_(turn_count),is_revease_(is_revease) {};

	int operator()(const Cells &path) {
		if (path.back().y < min_y_ || path.back().y > max_y_)
			return false;
		if(turn_count_ ==0) {
			return std::is_sorted(path.begin(), path.end(), IsIncrease(is_revease_));
		}
		else if(turn_count_ == 1){
			auto point = std::is_sorted_until(path.begin(), path.end(), IsIncrease(!is_revease_));
			auto is_sorted = std::is_sorted(point, path.end(), IsIncrease());
			return point != path.end() && std::is_sorted(point, path.end(), IsIncrease(is_revease_));
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
	bool is_revease_{};
};

class MinYAndShortestPath {
public:
	bool operator()(const Cells &path_a, const Cells &path_b) {
		if(path_a.back().y > path_b.back().y)
			return false;
		if(path_a.back().y < path_b.back().y)
			return true;
		if(path_a.back().y == path_b.back().y)
			return path_a.size() < path_b.size();
	};
};

bool NavCleanPathAlgorithm::filterPathsToSelectTarget(GridMap &map, PathList &paths, const Cell_t &cell_curr, Cell_t &best_target) {
	PathList filtered_paths{};

	for (auto &&path : paths) { for (auto &&cell : path) { printf("(%d,%d) ",cell.x,cell.y); } printf("\n"); }
	std::deque<BestTargetFilter> filters{{(int16_t)(cell_curr.y+2), 200, 0, false},
																			 {cell_curr.y,(int16_t)(cell_curr.y+1) ,0,false},
																			 {cell_curr.y, 200,1,false},
																			 {-200, cell_curr.y,0,true},
																			 {-200, cell_curr.y,1,true},
																			 {cell_curr.y, 200, -1,false},
																			 {-200, 200, -1,false},
	};
	for (auto &&filter : filters) {
		ROS_ERROR("is towards Y+(%d),y_range(%d,%d),allow turn count(%d)",!filter.is_revease_,filter.min_y_,filter.max_y_,filter.turn_count_);
		std::copy_if(paths.begin(), paths.end(), std::back_inserter(filtered_paths), BestTargetFilter(filter));
		if (!filtered_paths.empty()) {
			best_target = std::min_element(filtered_paths.begin(), filtered_paths.end(), MinYAndShortestPath())->back();
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
