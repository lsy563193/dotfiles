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
	auto path = findTargetInSameLane(map, curr_cell);
	if (!path.empty())
	{
		plan_path = cells_generate_points(path);
		// Congratulation!! plan_path is generated successfully!!
		map.print(CLEAN_MAP, path.back().x, path.back().y);
		return true;
	}

	ROS_INFO("Step 2: Find all possible plan_path at the edge of cleaned area and filter plan_path in same lane.");
	Cells targets{};
	map.generateSPMAP(curr_cell,targets);

	std::sort(targets.begin(),targets.end(),[](Cell_t l,Cell_t r){
		return (l.y < r.y || (l.y == r.y && l.x < r.x));
	});

	targets = std::for_each(targets.begin(), targets.end(),FilterTarget(curr_cell));

//	displayTargetList(targets);

	displayTargetList(targets);

	if (targets.empty())
		return false;

	ROS_INFO("Step 3: Trace back the path of these plan_path in COST_MAP.");
	PathList paths{};
	for (auto& target : targets) {
		Cells path{};
		findPath(map, curr_cell, target,  path, 0);
		if(!path.empty())
			paths.push_back(path);
	}

	for (auto &&path : paths) {
		displayCellPath(path);
	}
	ROS_INFO("Step 4: Filter paths to get the best target.");

	if (!filterPathsToSelectBestPath(map, paths, curr_cell, path))
		return false;
	displayCellPath(path);
	ROS_INFO("Step 5: Optimize path for adjusting it away from obstacles..");
	optimizePath(map, path);

	ROS_INFO("Step 6: Fill path with direction.");
	plan_path = cells_generate_points(path);

	// Congratulation!! plan_path is generated successfully!!
//	path = shortest_path;

	map.print(COST_MAP, path.back().x,path.back().y);
	map.print(CLEAN_MAP, path.back().x, path.back().y);
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

void NavCleanPathAlgorithm::findPath(GridMap &map, const Cell_t &start, const Cell_t &target,
																		 Cells &path,
																		 int last_i) {
	auto cost = map.getCell(COST_MAP, target.x, target.y);
	auto iterator = target;
	for (; iterator != start;) {
		if(map.getCell(COST_MAP, iterator.x, iterator.y) != cost)
		{
			printf("start(%d,%d) iterator(%d,%d),target(%d,%d)cost(%d)\n",start.x, start.y, iterator.x, iterator.y,cost, target.x, target.y);
			map.print(CLEAN_MAP, 0, 0);
			map.print(COST_MAP, 0, 0);
			ROS_ASSERT(map.getCell(COST_MAP, iterator.x, iterator.y) == cost);
		}
		cost -= 1;
		if(cost < 0 || cost > 5)
			break;
		if(cost == 0)
			cost = 5;
		for (auto i = 0; i < 4; i++) {
			auto neighbor = iterator + cell_direction_[(last_i + i) % 4];
			if (map.isOutOfTargetRange(neighbor))
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
	if(iterator == start)
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
		ROS_ERROR("is_reverse_(%d)", is_reverse_);
		return is_reverse_ ? a.y > b.y : a.y < b.y ;
	};
private:
	bool is_reverse_=false;
};

class BestTargetFilter {
public:
	BestTargetFilter(const Cell_t& min,const Cell_t& max,int turn_count=0,bool is_toward_pos=false):min_(min),max_(max),turn_count_(turn_count),is_toward_pos_(is_toward_pos) {};

	int operator()(const Cells &path) {
		if (path.back().y < min_.y || path.back().y > max_.y || path.back().x < min_.x || path.back().x >max_.x)
			return false;
		if(turn_count_ ==0) {
			return std::is_sorted(path.begin(), path.end(), IsIncrease(is_toward_pos_));
		}
		else if(turn_count_ == 1){
			auto point = std::is_sorted_until(path.begin(), path.end(), IsIncrease(!is_toward_pos_));
			return point != path.end() && std::is_sorted(point, path.end(), IsIncrease(is_toward_pos_));
		}
		else/* if(turn_count_ ==-1)*/{//any turn
			return true;
		}
	};
//private:
//	bool is_reverse_=false;
	Cell_t min_;
	Cell_t max_;
	int turn_count_;
	bool is_toward_pos_{};
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
			return path_a.size() < path_b.size();
		}else /*if(turn_count_ == 1000)*/{
			return path_a.size() < path_b.size();
		}
	};
	bool is_reverse_{};
	int16_t curr_y_{};
	int turn_count_;
};

bool NavCleanPathAlgorithm::filterPathsToSelectBestPath(GridMap &map, PathList &paths, const Cell_t &cell_curr, Cells &best_path) {
	std::deque<BestTargetFilter> filters{};
	Cell_t min_cell,max_cell;
	map.getMapRange(CLEAN_MAP,&min_cell.x, &max_cell.x,&min_cell.y,&max_cell.y);
	BestTargetFilter filters_case0{cell_curr + Cell_t{3,0} , Cell_t{max_cell.x,cell_curr.y}, 1, true};

	BestTargetFilter filters_case1{Cell_t{min_cell.x, cell_curr.y + 2}, max_cell, 0, true};
	BestTargetFilter filters_case1_1{Cell_t{min_cell.x, cell_curr.y}, Cell_t{min_cell.x, cell_curr.y+1}, 0, true};
	BestTargetFilter filters_case2{min_cell, max_cell,1,true};
	BestTargetFilter filters_case3{min_cell, Cell_t{min_cell.x, cell_curr.y-2}, 0, false};
	BestTargetFilter filters_case3_1{Cell_t{min_cell.x, cell_curr.y-2}, Cell_t{min_cell.x, cell_curr.y-1} , 0, false};
	BestTargetFilter filters_case4{min_cell, max_cell,1,false};
	BestTargetFilter filters_case5{Cell_t{min_cell.x, cell_curr.y}, max_cell, 1000,true};
	BestTargetFilter filters_case6{min_cell, max_cell, 1000,false};
	filters.push_back(filters_case0);
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
		printf("is towards Y+(%d),y_range min(%d,%d)max(%d,%d),allow turn count(%d)\n",filter.is_toward_pos_,filter.min_.x,filter.min_.y,filter.max_.x,filter.max_.y,filter.turn_count_);
		PathList filtered_paths{};
		std::copy_if(paths.begin(), paths.end(), std::back_inserter(filtered_paths), BestTargetFilter(filter));

		if (!filtered_paths.empty()) {
			for (auto &&path : filtered_paths) {
				displayCellPath(path);
			}
//			std::sort(filtered_paths.begin(), filtered_paths.end(), );
			best_path = *std::min_element(filtered_paths.begin(), filtered_paths.end(), MinYAndShortestPath(cell_curr.y, filter.is_toward_pos_,filter.turn_count_));
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
	Cells path2{};
	findPath(map, path.front(), path.back(), path2, 2);
	if(path2.size() < path.size())
	{
		path.swap(path2);
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
			auto p_side2 = p_it + cell_direction_[dir_21]*2;
			auto p_side3 = p_it + cell_direction_[dir_21]*3;
			if(map.isABlock(p_side2.x,p_side2.y) || map.isABlock(p_side3.x,p_side3.y))
				break;
		}
		if(p_it == *p3+cell_direction_[dir_p32] && map.isBlockAccessible(p_it.x, p_it.y))
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
