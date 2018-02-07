//
// Created by lsy563193 on 12/13/17.
//

#include <dev.h>
#include "ros/ros.h"
#include "robot.hpp"
#include "path_algorithm.h"

extern int g_follow_last_follow_wall_dir;

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
				if (is_continue_)
					tmps_.pop_back();
				is_continue_ = curr_.x != it.x;
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

bool NavCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path)
{

	map.print(CLEAN_MAP, 0, 0);
	plan_path.clear();
	auto curr_cell = curr.toCell();
	ROS_INFO("Step 1: Find possible plan_path in same lane.");
	auto plan_path_cell = findTargetInSameLane(map, curr_cell);
	if (!plan_path_cell.empty())
	{
		plan_path = cells_generate_points(plan_path_cell);
		// Congratulation!! plan_path is generated successfully!!
		map.print(CLEAN_MAP, plan_path_cell.back().x, plan_path_cell.back().y);
		return true;
	}

	ROS_INFO("Step 2: Find all possible plan_path at the edge of cleaned area and filter plan_path in same lane.");
	Cells targets{};
	map.generateSPMAP(curr_cell,targets);

	std::sort(targets.begin(),targets.end(),[](Cell_t l,Cell_t r){
		return (l.y < r.y || (l.y == r.y && l.x < r.x));
	});

	targets = std::for_each(targets.begin(), targets.end(),FilterTarget(curr_cell));

	displayTargetList(targets);

//	targets.erase(std::remove_if(targets.begin(), targets.end(),[&map](Cell_t it){
//		auto cost = map.getCell(COST_MAP, it.x, it.y);
//		return cost == COST_NO || cost == COST_1;
//	}),targets.end());

	displayTargetList(targets);

	if (targets.empty())
		return false;

	ROS_INFO("Step 3: Trace back the path of these plan_path in COST_MAP.");
	PathList paths{};
	for (auto& target : targets) {
		Cells path{};
		tracePathsToTarget(map, target, curr_cell, path,0);
		if(!path.empty())
			paths.push_back(path);
	}

	ROS_INFO("Step 4: Filter paths to get the best target.");

	Cells path;
	if (!filterPathsToSelectBestPath(map, paths, curr_cell, path,last_dir))
		return false;

	map.print(COST_MAP, path.back().x,path.back().y);
	ROS_INFO("Step 5: Optimize path for adjusting it away from obstacles..");
	optimizePath(map, path);

	ROS_INFO("Step 6: Fill path with direction.");
	plan_path = cells_generate_points(path);

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

		for (Cell_t neighbor = it[i] + cell_direction_[i];
				 !map.cellIsOutOfRange(neighbor + cell_direction_[i]) && !map.isBlocksAtY(neighbor.x, neighbor.y);
				 neighbor += cell_direction_[i])
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

void NavCleanPathAlgorithm::tracePathsToTarget(GridMap &map, const Cell_t &target, const Cell_t &start, Cells &path,int last_i) {
	auto cost = map.getCell(COST_MAP, target.x, target.y);
	for (auto iterator = target; iterator != start;) {
		cost -= 1;
		if(cost <= 2)
			break;
		for (auto i = 0; i < 4; i++) {
			auto neighbor = iterator + cell_direction_[(last_i + i) % 4];
			if (map.isOutOfMap(neighbor))
				continue;

			if (map.getCell(COST_MAP, neighbor.x, neighbor.y) == cost) {
				if (i != 0) {
					last_i = (last_i + i) % 4;
					path.push_front(iterator);
				}
				iterator = neighbor;
				break;
			}
		}
	}
	if(cost == 3)
	{
		if (path.back() != target)
			path.push_back(target);
		path.push_front(start);
	}
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

bool NavCleanPathAlgorithm::filterPathsToSelectBestPath(GridMap &map, PathList &paths, const Cell_t &cell_curr,
																												Cells &best_path,Dir_t last_dir) {
	std::deque<BestTargetFilter> filters{};
	Cell_t min_cell,max_cell;
	map.getMapRange(CLEAN_MAP,&min_cell.x, &max_cell.x,&min_cell.y,&max_cell.y);
//	BestTargetFilter filters_curr_y{static_cast<int16_t>(cell_curr.y - 2), cell_curr.y, 1, false};
	BestTargetFilter filters_pos_2_puls{static_cast<int16_t>(cell_curr.y + 2), max_cell.y, 0, false};
	BestTargetFilter filters_pos_0_to_1{cell_curr.y, static_cast<int16_t>(cell_curr.y + 1), 0, false};
	BestTargetFilter filters_pos_1_turn{min_cell.y, max_cell.y,1,false};
	BestTargetFilter filters_nag_2_puls{min_cell.y, static_cast<int16_t>(cell_curr.y - 2), 0, true};
	BestTargetFilter filters_nag_0_to_1{static_cast<int16_t>(min_cell.y - 1), cell_curr.y, 0, true};
	BestTargetFilter filters_nag_1_turn{min_cell.y, max_cell.y,1,true};
	BestTargetFilter filters_pos_n_turn{cell_curr.y, max_cell.y, 1000,false};
	BestTargetFilter filters_nag_n_turn{min_cell.y, max_cell.y, 1000,true};
//	filters.push_back(filters_curr_y);
	filters.push_back(filters_pos_2_puls);
	filters.push_back(filters_pos_0_to_1);

	if(!map.getCell(CLEAN_MAP,cell_curr.x,cell_curr.y-2) == UNCLEAN)
		filters.push_back(filters_pos_1_turn);
	filters.push_back(filters_nag_2_puls);
	filters.push_back(filters_nag_0_to_1);
	filters.push_back(filters_nag_1_turn);
	filters.push_back(filters_pos_n_turn);
	filters.push_back(filters_nag_n_turn);

	for (auto &&filter : filters) {
		printf("is towards Y+(%d),y_range(%d,%d),allow turn count(%d)\n",!filter.is_reverse_,filter.min_y_,filter.max_y_,filter.turn_count_);
		PathList filtered_paths{};
		std::copy_if(paths.begin(), paths.end(), std::back_inserter(filtered_paths), BestTargetFilter(filter));
		if (!filtered_paths.empty()) {
//			std::sort(filtered_paths.begin(), filtered_paths.end(), );
			best_path = *std::min_element(filtered_paths.begin(), filtered_paths.end(), MinYAndShortestPath(cell_curr.y, filter.is_reverse_,filter.turn_count_));
			return true;
		}
	}
	return false;
}

bool NavCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	return checkTrappedUsingDijkstra(map, curr_cell);
}

void NavCleanPathAlgorithm::optimizePath(GridMap &map, Cells &path) {
	// Optimize only if the path have more than 3 cells.
	if (path.size() <= 3)
	{
		ROS_INFO("%s %d:Path too short(size: %ld), optimization terminated.", __FUNCTION__, __LINE__, path.size());
		return;
	}
	printf("\n");
	ROS_INFO("%s %d: Start optimizing Path", __FUNCTION__, __LINE__);

	ROS_INFO("%s %d: step *.1, check simply last 3 cell ", __FUNCTION__, __LINE__);
	PathList paths{};
	Cells targets{*(path.end()-4)};
	Cells path_simply{};
	tracePathsToTarget(map, path.back() ,*(path.end()-4),  path_simply, 2);
	if(path_simply.size() == 3)
	{
		path.erase(path.end()-3,path.end()-1);
		path.insert((path.end()-1),*(path_simply.end()-2));
		ROS_INFO("%s %d: step *.1 do success ", __FUNCTION__, __LINE__);
//		map.print(CLEAN_MAP, path.front(),path.back().x, path.back().y);
	}
	ROS_INFO("%s %d: step *.2, away from obstacles ", __FUNCTION__, __LINE__);
//	map.print(COST_MAP, path.front(), path.back().x,path.back().y);
	for(auto iterator = path.begin(); iterator != path.end()-3; ++iterator){
		auto p1 = iterator;
		auto p2 = iterator+1;
		auto p3 = iterator+2;
		int dir_p32 =0;
		auto p_32 = *p3 - *p2;
		dir_p32 = (p_32.x != 0)?(p_32.x > 0 ? 0 : 1):(p_32.y > 0 ? 2: 3);
		int dir_21 =0;
		auto p_21 = *p2 - *p1;
		dir_21 = (p_21.x != 0)?(p_21.x > 0 ? 0 : 1):(p_21.y > 0 ? 2: 3);
//		printf("p1(%d,%d),p2(%d,%d),p3(%d,%d),dir_p32(%d),\n",p1->x,p1->y, p2->x,p2->y,p3->x, p3->y,dir_p32);
		auto p_it = *p2;
		for(; p_it != *p3+cell_direction_[dir_p32]; p_it += cell_direction_[dir_p32])
		{
//			printf("{%d,%d},",p_it.x, p_it.y);
			auto p_side = p_it + cell_direction_[dir_21];
//			auto p_side2 = p_it + cell_index_[dir_21]*3;
//			printf("{%d,%d},",p_side.x, p_side.y);
//			printf("{%d,%d},",p_side2.x, p_side2.y);
//			printf("\n");
//			if(map.getCell(CLEAN_MAP, p_side.x,p_side.y) !=BLOCKED && )
			if(!map.isBlockAccessible(p_side.x, p_side.y))
				break;
		}
		if(p_it == *p3+cell_direction_[dir_p32])
		{
			*p2 += cell_direction_[dir_21];
			*p3 += cell_direction_[dir_21];
			ROS_INFO("%s %d: step *.2 do success ", __FUNCTION__, __LINE__);
			printf("\n");
		}
//		printf("\n");
//		printf("\n");
//		printf("\n");
	}
}
