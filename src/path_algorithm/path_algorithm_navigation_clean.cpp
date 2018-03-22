//
// Created by lsy563193 on 12/13/17.
//

#include <dev.h>
#include "ros/ros.h"
#include "robot.hpp"
#include "path_algorithm.h"

extern int g_follow_last_follow_wall_dir;

int size_of_path(const Cells &path){
		int sum=0;
		for (auto iterator = path.begin(); iterator != path.end()-1; ++iterator) {
			sum += (iterator->x == (iterator+1)->x) ? std::abs(iterator->y - (iterator+1)->y) : std::abs(iterator->x - (iterator+1)->x);
		}
		return sum;
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
			else {
				if (it.x - tmps_.back().x == 1) {
					if (is_continue_)
						tmps_.pop_back();
					is_continue_ = curr_.x != it.x;
				}else
					is_continue_ = false;
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
	plan_path.clear();
	auto curr_cell = curr.toCell();
	ROS_INFO("Step 1: Find possible plan_path in same lane.");
	Cells path{};
	if(curr_cell.y % 2==0) {
		path = findTargetInSameLane(map, curr_cell);
		if (!path.empty()) {
			plan_path = cells_generate_points(path);
			// Congratulation!! plan_path is generated successfully!!
			map.print(CLEAN_MAP, path);
			curr_filter_ = nullptr;
			return true;
		}
	}

	ROS_INFO("Step 2: Find all possible plan_path at the edge of cleaned area and filter plan_path in same lane.");
	Cells targets{};
	map.generateSPMAP(curr_cell,targets);
//	map.find_if(curr_cell, targets,[&](const Cell_t &c_it){
//		return c_it.y%2 == 0 && map.getCell(CLEAN_MAP, c_it.x, c_it.y) == UNCLEAN  && map.isBlockAccessible(c_it.x, c_it.y);
//	});
//	map.print(COST_MAP, Cells{curr_cell});

	std::sort(targets.begin(),targets.end(),[](Cell_t l,Cell_t r){
		return (l.y < r.y || (l.y == r.y && l.x < r.x));
	});

	targets = std::for_each(targets.begin(), targets.end(),FilterTarget(curr_cell));

	displayTargetList(targets);
//	map.print(CLEAN_MAP, targets);
//	map.print(CLEAN_MAP, targets);

	if (targets.empty())
	{
		map.print(CLEAN_MAP, path);
		map.print(COST_MAP, path);
		Cell_t target;
		int dijkstra_cleaned_count;
		if(!findTargetUsingDijkstra(map,getPosition().toCell(),target,dijkstra_cleaned_count))
			return false;
		targets.push_back(target);
	}

	if (!filterPathsToSelectBestPath(map, targets, curr_cell, path,last_dir))
		return false;
	if(path.size() > 4 )
	{
		ROS_INFO("Step 5: size_of_path > 15 Optimize path for adjusting it away from obstacles..");
		optimizePath(map, path);
	}

	if(curr_filter_ == &filter_p0_1t_xn || curr_filter_ == &filter_p0_1t_xp)
		path.push_back(Cell_t{path.back().x, static_cast<int16_t>(path.front().y - 2)});//for setting follow wall target line
	else if(curr_filter_ == &filter_n0_1t_xn || curr_filter_ == &filter_n0_1t_xp)
		path.push_back(Cell_t{path.back().x, static_cast<int16_t>(path.front().y + 2)});//for setting follow wall target line

	ROS_INFO("Step 6: Fill path with direction.");
	plan_path = cells_generate_points(path);

	displayCellPath(path);
	map.print(CLEAN_MAP, path);
	return true;
}

Cells NavCleanPathAlgorithm::findTargetInSameLane(GridMap &map, const Cell_t &curr_cell)
{
	int8_t is_found = 0;
	Cell_t it[2]; // it[0] means the furthest cell of x positive direction, it[1] means the furthest cell of x negative direction.
	const auto OVER_CELL_SIZE = 2;
//	map.print(CLEAN_MAP, 0, 0);
	for (auto i = 0; i < 2; i++) {
		it[i] = curr_cell;
		auto neighbor = it[i] + cell_direction_[i];
		for (; !map.cellIsOutOfRange(neighbor) && !map.isBlocksAtY(neighbor.x, neighbor.y) ; neighbor += cell_direction_[i])
		{
			if (map.getCell(CLEAN_MAP, neighbor.x, neighbor.y) == UNCLEAN)
				it[i] = neighbor ;
		}
		//optimizePath
		if (!map.isBlockAtY(BLOCKED_SLIP, neighbor.x, neighbor.y) &&
							!map.isBlockAtY(BLOCKED_TILT, neighbor.x, neighbor.y) &&
							!map.isBlockAtY(BLOCKED_RCON,neighbor.x,neighbor.y) &&
							it[i] != curr_cell) {
			it[i] += cell_direction_[i] * OVER_CELL_SIZE;
		}
	}

	Cell_t target;
	if (it[0].x != curr_cell.x)
	{
		target = it[0];
		if(target.x >= MAP_SIZE )
			target.x = MAP_SIZE - 1;
		is_found++;
	}
	if (it[1].x != curr_cell.x)
	{
		target = it[1];
		if(target.x <= MAP_SIZE )
			target.x = -MAP_SIZE + 1;
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
//			beeper.beepForCommand(VALID);
			ROS_ERROR("%s %d: g_follow_last_follow_wall_dir(%d)", __FUNCTION__, __LINE__, g_follow_last_follow_wall_dir);
			if(g_follow_last_follow_wall_dir == 1)
				target = it[1];
			else//(g_follow_last_follow_wall_dir == 2)
				target = it[0];
		}
		auto dir = lidar.compLaneDistance();
		if(dir != -1)
		{
			target = it[0];
			if( dir == 1)
				target = it[1];
		}
	}
	g_follow_last_follow_wall_dir = 0;
	Cells path{};
	if (is_found)
	{
		path.push_front(target);
		path.push_front(getPosition().toCell());
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
//			map.print(CLEAN_MAP, 0, 0);
//			map.print(COST_MAP, 0, 0);
			ROS_ASSERT(map.getCell(COST_MAP, iterator.x, iterator.y) == cost);
		}
		cost -= 1;
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
	if (path.back() != target)
		path.push_back(target);
	path.push_front(start);
}


class MinYAndShortestPath {
public:
	MinYAndShortestPath(int16_t curr_y, bool is_reverse,int turn_count):is_reverse_(is_reverse),curr_y_(curr_y),turn_count_(turn_count){ };
	bool operator()(const Cells &path_a, const Cells &path_b) {
		if(turn_count_ == 0 || turn_count_ == 1000)
		{
			if(path_a.back().y == path_b.back().y)
				return size_of_path(path_a) < size_of_path(path_b);
			return (std::abs(path_a.back().y - curr_y_) < std::abs(path_b.back().y - curr_y_));
		}
		else {
			return size_of_path(path_a) < size_of_path(path_b);
		}
	};

	bool is_reverse_{};
	int16_t curr_y_{};
	int turn_count_;
};

bool NavCleanPathAlgorithm::filterPathsToSelectBestPath(GridMap &map, const Cells &targets, const Cell_t &cell_curr, Cells &best_path, const Dir_t &last_dir) {
	std::deque<BestTargetFilter*> filters{};
	Cell_t min_cell,max_cell;
	map.getMapRange(CLEAN_MAP,&min_cell.x, &max_cell.x,&min_cell.y,&max_cell.y);

	ROS_WARN("last_dir(%d)\n",last_dir);
	if(isXAxis(last_dir))
	{
		if(isPos(last_dir))
		{
			filters.push_back(&filter_p0_1t_xp);
			filters.push_back(&filter_n0_1t_xp);
		}
		else
		{
			filters.push_back(&filter_p0_1t_xn);
			filters.push_back(&filter_n0_1t_xn);
		}
	}

	if(cell_curr.y%2 ==0)
		filters.push_back(&filter_p2);
	else
		filters.push_back(&filter_p1);

	filters.push_back(&filter_p4p);

	if(cell_curr.y%2 ==0)
		filters.push_back(&filter_n2);
	else
		filters.push_back(&filter_n1);

	filters.push_back(&filter_p_1t);
	filters.push_back(&filter_n4n);
	filters.push_back(&filter_n_1t);
	filters.push_back(&filter_p_1000t);
	filters.push_back(&filter_n_1000t);

	for (auto &&filter : filters) {
		filter->update(const_cast<Cell_t &>(cell_curr), min_cell, max_cell);
		ROS_WARN("is towards Y+(%d),y_range min(%d,%d)max(%d,%d),allow turn count(%d)", filter->is_toward_pos_,
						 filter->min_.x, filter->min_.y, filter->max_.x, filter->max_.y, filter->turn_count_);

		Cells filtered_targets{};
		std::copy_if(targets.begin(), targets.end(), std::back_inserter(filtered_targets), [&filter](Cell_t target) {
			return !(target.y < filter->min_.y || target.y > filter->max_.y || target.x < filter->min_.x ||
							 target.x > filter->max_.x);
		});

		if (filtered_targets.empty())
			continue;

//		displayTargetList(filtered_targets);
		PathList paths{};
		for (auto &target : filtered_targets) {
			Cells path{};
			int dir = 2;
			if (!filter->towardPos())
				dir = 3;
			findPath(map, cell_curr, target, path, dir);
			paths.push_back(path);
		}

		PathList filtered_paths{};
		std::copy_if(paths.begin(), paths.end(), std::back_inserter(filtered_paths), BestTargetFilter(*filter));
		if (!filtered_paths.empty()) {
			best_path = *std::min_element(filtered_paths.begin(), filtered_paths.end(),
																		MinYAndShortestPath(cell_curr.y, filter->is_toward_pos_, filter->turn_count_));
			ROS_INFO("path.len(%d)",size_of_path(best_path));
			if (filter == &filter_p_1t) {
				if(size_of_path(best_path) > 25){
					ROS_WARN("path.len is too long to find other path(%d)",size_of_path(best_path));
					beeper.beepForCommand(VALID);
					continue ;
				}
			}
			curr_filter_ = filter;
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
	if(path.size() <= 3)
		return;
	// Optimize only if the path have more than 3 cells.
	ROS_INFO("%s %d: Start optimizing Path", __FUNCTION__, __LINE__);
	auto find_index = [&](Cell_t p1, Cell_t p2, Cell_t p3) {
			int dir_p23 = 0;
			auto p_23 = p3 - p2;
			dir_p23 = (p_23.x != 0) ? (p_23.x > 0 ? 0 : 1) : (p_23.y > 0 ? 2 : 3);
			int dir_p12 = 0;
			auto p_12 = p2 - p1;
			dir_p12 = (p_12.x != 0) ? (p_12.x > 0 ? 0 : 1) : (p_12.y > 0 ? 2 : 3);
//		printf("p1(%d,%d),p2(%d,%d),p3(%d,%d),dir_p23(%d),\n",p1->x,p1->y, p2->x,p2->y,p3->x, p3->y,dir_p23);
			auto p12_it = p2;
			for (;; p12_it += cell_direction_[dir_p12]) {
				auto p23_it = p12_it;
				for (; p23_it != p3 + cell_direction_[dir_p23]; p23_it += cell_direction_[dir_p23]) {
//					printf("{%d,%d},",p23_it.x, p23_it.y);
					if (!map.isNotBlockAndCleaned(p23_it.x, p23_it.y))
					{
						printf("\n1 break it(%d,%d)!!\n", p12_it.x, p12_it.y);
						return (p12_it - cell_direction_[dir_p12]- p2)/2;
					}
				}
//				printf("\n");
				if (!(p23_it == p3 + cell_direction_[dir_p23] && map.isBlockAccessible(p23_it.x, p23_it.y))) {
					{

						printf("\n1 break it(%d,%d)!!\n", p12_it.x, p12_it.y);
						return (p12_it - cell_direction_[dir_p12] - p2)/2;
					}
				}
				else{
//					p2 += cell_direction_[dir_p12];
					p3 += cell_direction_[dir_p12];
//					printf("opt success, +1 !\n");
				}
			}
		};
	for(auto iterator = path.begin(); iterator != path.end()-3; ++iterator) {
		auto p1 = iterator;
		auto p2 = iterator + 1;
		auto p3 = iterator + 2;

		auto shift_cell = find_index(*p1, *p2, *p3);
//		if(shift_cell != Cell_t{0,0})
//		{
			*p2 += shift_cell;
			*p3 += shift_cell;
			ROS_INFO("%s %d: step *.2 do success shift_cell(%d,%d)\n\n", __FUNCTION__, __LINE__,shift_cell.x, shift_cell.y);
//		}

	}

}
