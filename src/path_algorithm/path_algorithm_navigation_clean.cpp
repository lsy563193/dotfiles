//
// Created by lsy563193 on 12/13/17.
//

#include <dev.h>
#include "ros/ros.h"
#include "robot.hpp"
#include "path_algorithm.h"

extern int g_follow_last_follow_wall_dir;


std::unique_ptr<std::deque<BestTargetFilter*>> NavCleanPathAlgorithm::generateBounds(GridMap& map) {

	std::deque<BestTargetFilter*> filters;
	if(curr_filter_ == &filter_short_path && priority_dir != MAP_POS_Y)
		filters.push_back(&filter_pos_of_y_axis);

	filters.push_back(&filter_curr_line_pos);
	filters.push_back(&filter_curr_line_neg);

	if(isXAxis(priority_dir)) {
		int16_t dx = correct_curr_.x + static_cast<int16_t>(isPos(priority_dir) ? 4 : -4);
		int16_t dy = 2;
		ROS_INFO("filter_after_obstacle:dx,dy(%d,%d)", dx, dy);
		if ((map.getCell(CLEAN_MAP, dx, correct_curr_.y) == UNCLEAN && map.getCell(CLEAN_MAP, dx, correct_curr_.y-dy) == CLEANED))
			filters.push_back(&filter_after_obstacle_neg);
		if ((map.getCell(CLEAN_MAP, dx, correct_curr_.y) == UNCLEAN && map.getCell(CLEAN_MAP, dx, correct_curr_.y+dy) == CLEANED))
			filters.push_back(&filter_after_obstacle_pos);
	}

	filters.push_back(&filter_next_line_pos);

	if(!(curr_filter_ == &filter_short_path && priority_dir != MAP_POS_Y) )
		filters.push_back(&filter_pos_of_y_axis);

	filters.push_back(&filter_next_line_neg);


	if(isXAxis(priority_dir)) {
		int16_t dx = correct_curr_.x + static_cast<int16_t>(isPos(priority_dir) ? 4 : -4);
//		int16_t dy = correct_curr_.y + static_cast<int16_t>(trend_pos ? 2 : -2);
		int16_t dy = 2;
		ROS_WARN("dx,dy(%d,%d),trend_pos(%d)",dx,dy,trend_pos);
		if ((map.getCell(CLEAN_MAP, dx, correct_curr_.y) == CLEANED && map.getCell(CLEAN_MAP, dx, correct_curr_.y + dy) == UNCLEAN))
			filters.push_back(&filter_top_of_y_axis_pos);
		if ((map.getCell(CLEAN_MAP, dx, correct_curr_.y) == CLEANED && map.getCell(CLEAN_MAP, dx, correct_curr_.y - dy) == UNCLEAN))
			filters.push_back(&filter_top_of_y_axis_neg);
	}

//	filters.push_back(&filter_n3p);

	filters.push_back(&filter_short_path);
	return make_unique<std::deque<BestTargetFilter*>>(filters);
}

static BoundingBox2 getLine(const Cell_t& curr,GridMap& map)
{
	Cell_t c_it[2];
	for (auto i = 0; i < 2; i++) {
			c_it[i] = curr;
			for (;; c_it[i] += cell_direction_[i]) {
                auto tmp = c_it[i] +cell_direction_[i]*2;
				if (map.cellIsOutOfTargetRange(tmp)  || map.isBlocksAtY(tmp.x, tmp.y))
					break;
				if (map.getCell(CLEAN_MAP, c_it[i].x, c_it[i].y) == UNCLEAN)
					break;
			}
		}
		return {c_it[1], c_it[0]};
}

void NavCleanPathAlgorithm::adjustPosition(Points&  plan_path)
{
	int16_t tmp_curr_y=origen_curr_.y;
	if(origen_curr_.y%2 == 1 && curr_filter_ != nullptr)
	{
		curr_filter_->displayName();

		if(curr_filter_ == &filter_curr_line_pos || curr_filter_ == &filter_curr_line_neg)
		{
			tmp_curr_y = plan_path.back().toCell().y;
		}
		if(curr_filter_ == &filter_next_line_pos || curr_filter_ == &filter_next_line_neg)
		{
			tmp_curr_y = plan_path.front().toCell().y;
			if(g_follow_last_follow_wall_dir != 0)
				tmp_curr_y = plan_path.back().toCell().y;
		}
		ROS_WARN("in odd line: adjust Position(%d)->(%d)", origen_curr_.y, correct_curr_.y);
	}
	correct_curr_ = {origen_curr_.x, tmp_curr_y};
}
bool NavCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr_p, const Dir_t &last_dir, Points &plan_path)
{
	Cells path{};
	Cells targets{};
	if(origen_curr_ != curr_p.toCell())
	{
		origen_curr_ = curr_p.toCell();
		adjustPosition(plan_path);
	}
	else
	{
		ROS_WARN("origen_curr equal curr_p, that mean adjustPosition cast path plan to same target(%d,%d),(%d,%d)",origen_curr_.x, origen_curr_.y, correct_curr_.x, correct_curr_.y);
		correct_curr_ = origen_curr_;
	}

	plan_path.clear();
	map.markRobot(correct_curr_, CLEAN_MAP);
	map_bound = map.genTargetRange();
	curr_bound = getLine(correct_curr_, map);
	priority_dir = last_dir;
	auto filters = *generateBounds(map);
	ROS_WARN("priority_dir(%d),trend_pos(%d)\n",priority_dir,trend_pos);
	g_follow_last_follow_wall_dir = 0;
	for(auto&&filter : filters)
	{
		curr_filter_=filter;
		filter->updateTargetAndRangeBound();
		filter->displayName();
		ROS_INFO("target_bound(%d,%d,%d,%d)",filter->target_bound.min.x, filter->target_bound.min.y, filter->target_bound.max.x, filter->target_bound.max.y);
		ROS_INFO("range_bound(%d,%d,%d,%d)",filter->range_bound.min.x, filter->range_bound.min.y, filter->range_bound.max.x, filter->range_bound.max.y);
		func_compare_two_t expand_condition = nullptr;
		if (filter->is_forbit_turn_) {
			expand_condition = [&](const Cell_t &next, const Cell_t &neighbor) {
				return map.isBlockAccessible(neighbor.x, neighbor.y) && neighbor.y >= next.y;
			};
		}
		else{
			expand_condition = [&](const Cell_t &next, const Cell_t &neighbor) {
				return map.isBlockAccessible(neighbor.x, neighbor.y);
			};
		}

		if(dijkstra(map, correct_curr_, path, true, IsTarget(&map,filter->target_bound), isAccessable(&map,expand_condition, filter->range_bound)))
			break;
//		map.print(correct_curr_,COST_MAP,path);
	}

	if(path.empty())
	{
		curr_filter_ = nullptr;
		return false;
	}

	trend_pos = curr_filter_ != &filter_next_line_neg;
	optimizePath(map, path, priority_dir);

	plan_path = *cells_to_points(path);

	displayCellPath(path);
	map.print(correct_curr_,COST_MAP,path);
	map.print(curr_p.toCell(), CLEAN_MAP, path);
	return true;
}


bool NavCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	if(robot::instance()->p_mode->getNextMode() == Mode::cm_navigation) {
		return checkTrappedUsingDijkstra(map, curr_cell);
	}
	else if(robot::instance()->p_mode->getNextMode() == Mode::cm_exploration) {
		Cells cells{};
		auto p_cm = boost::dynamic_pointer_cast<CleanModeExploration>(robot::instance()->p_mode);
//		return !(dijkstra(p_cm->clean_map_, getPosition().toCell(), cells,CellEqual(Cell_t{0,0}),true,isAccessable(p_cm->clean_map_.genRange(), &p_cm->clean_map_)));

		auto expand_condition = [&](const Cell_t &cell, const Cell_t &neighbor_cell)
		{
			return p_cm->clean_map_.isBlockAccessible(neighbor_cell.x, neighbor_cell.y) &&
					p_cm->clean_map_.getCell(CLEAN_MAP, neighbor_cell.x, neighbor_cell.y) == CLEANED;
		};

		return !dijkstra(p_cm->clean_map_, getPosition().toCell(), cells, true, CellEqual(Cell_t{0,0}), isAccessable(&p_cm->clean_map_, expand_condition));
	}
}
//
//auto _check_limit = [&](Cell_t &shift_cell, const bool is_dir_x) {
//	if (is_dir_x && std::abs(shift_cell.x) > 3) {
//		shift_cell.x = static_cast<int16_t>(shift_cell.x > 0 ? 3 : -3);
//	} else if (!is_dir_x && shift_cell.y > 3) {
//		shift_cell.y = static_cast<int16_t>(shift_cell.y > 0 ? 3 : -3);
//	}
//};

void NavCleanPathAlgorithm::optimizePath(GridMap &map, Cells &path, Dir_t& priority_dir) {

	ROS_INFO("Step 5:optimizePath");
	if(curr_filter_ == &filter_curr_line_pos || curr_filter_ == &filter_curr_line_neg) {
		ROS_INFO("filter_curr_line_pos:curr line");
		const auto OVER_CELL_SIZE = 4;
		if (robot::instance()->p_mode->getNextMode() == Mode::cm_navigation) {
			auto i = path.back().x > path.front().x ? 0 : 1;
			auto &c_it = path.back();
			auto tmp = c_it;
			for (; ; tmp += cell_direction_[i]) {
//				ROS_INFO("tmp(%d,%d)",tmp.x, tmp.y);
				if (map.getCell(CLEAN_MAP, tmp.x, tmp.y) == CLEANED) {
					break;
				}
				if(map.isBlocksAtY(tmp.x, tmp.y)){
					if (!map.isBlockAtY(BLOCKED_SLIP, tmp.x, tmp.y) &&
						!map.isBlockAtY(BLOCKED_TILT, tmp.x, tmp.y) &&
						!map.isBlockAtY(BLOCKED_RCON, tmp.x, tmp.y) /*c_it != curr*/) {
						tmp += cell_direction_[i] * OVER_CELL_SIZE;
					} else{
						tmp -= cell_direction_[i] * 2;
					}
					break;
				}
				if(map.cellIsOutOfTargetRange(tmp))
					break;
			}
		c_it = tmp;
		map.cellPreventOutOfRange(c_it);
		}
	}
	else if (curr_filter_ == &filter_after_obstacle_pos)
		path.push_back( Cell_t{path.back().x, static_cast<int16_t>(path.front().y + 3)});//for setting follow wall target line
	else if (curr_filter_ == &filter_after_obstacle_neg)
		path.push_back( Cell_t{path.back().x, static_cast<int16_t>(path.front().y - 3)});//for setting follow wall target line
	else if (curr_filter_ == &filter_top_of_y_axis_pos)
		path.push_back( Cell_t{path.back().x, static_cast<int16_t>(path.front().y - 3)});//for setting follow wall target line
	else if (curr_filter_ == &filter_top_of_y_axis_neg)
		path.push_back( Cell_t{path.back().x, static_cast<int16_t>(path.front().y + 3)});//for setting follow wall target line
	else {
		APathAlgorithm::optimizePath(map, path, priority_dir);

	}
}
