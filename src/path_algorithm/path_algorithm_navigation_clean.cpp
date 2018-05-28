//
// Created by lsy563193 on 12/13/17.
//

#include <dev.h>
#include "ros/ros.h"
#include "robot.hpp"
#include "path_algorithm.h"

extern int g_follow_last_follow_wall_dir;

#if !USE_NEW_PATH_PLAN

void path_crop(std::unique_ptr<Cells> &cells)
{
	if(cells->size() >2)
	{
        for(auto c_it = cells->begin()+1; c_it != cells->end()-1;)
		{
			c_it = (get_dir(c_it , c_it-1) == get_dir(c_it+1 , c_it)) ? cells->erase(c_it) : ++c_it;
		}
	}
}
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

std::unique_ptr<Cells> NavCleanPathAlgorithm::findTargetInSameLane(GridMap &map, const Cell_t &curr) {
	int8_t is_found = 0;
	Cell_t c_it[2]; // c_it[0] means the furthest cell of x positive direction, c_it[1] means the furthest cell of x negative direction.
	for (auto i = 0; i < 2; i++) {
		auto tmp = c_it[i] = curr;
		for (;; tmp += cell_direction_[i]) {
			if (map.cellIsOutOfRange(tmp) || map.isBlocksAtY(tmp.x, tmp.y))
				break;
			if (map.getCell(CLEAN_MAP, tmp.x, tmp.y) == UNCLEAN)
			{
				c_it[i] = tmp;
				break;
			}
		}
	}

	Cell_t target;
	if (c_it[0].x != curr.x)
	{
		target = c_it[0];
		if(target.x >= MAP_SIZE )
			target.x = MAP_SIZE - 1;
		is_found++;
	}
	if (c_it[1].x != curr.x)
	{
		target = c_it[1];
		if(target.x <= -MAP_SIZE )
			target.x = -MAP_SIZE + 1;
		is_found++;
	}
//	ROS_WARN("%s %d: curr(%d,%d) is_found(%d), c_it[0](%d,%d), c_it[1](%d,%d)", __FUNCTION__, __LINE__, curr.x, curr.y,
//			 is_found, c_it[0].x, c_it[0].y, c_it[1].x, c_it[1].y);
	if (is_found == 2)
	{
		// Select the nearest side.
		if (std::abs(curr.x - c_it[0].x) < std::abs(curr.x - c_it[0].x))
			target = c_it[0];

		//todo
//			ROS_ERROR("%s %d: 1 g_follow_last_follow_wall_dir(%d)", __FUNCTION__, __LINE__, g_follow_last_follow_wall_dir);
		if(g_follow_last_follow_wall_dir!=0)
		{
//			beeper.beepForCommand(VALID);
			ROS_INFO("%s %d: g_follow_last_follow_wall_dir(%d)", __FUNCTION__, __LINE__, g_follow_last_follow_wall_dir);
			if(g_follow_last_follow_wall_dir == 1)
				target = c_it[1];
			else//(g_follow_last_follow_wall_dir == 2)
				target = c_it[0];
		}
		auto dir = lidar.compLaneDistance();
		if(dir != -1)
		{
			target = c_it[0];
			if( dir == 1)
				target = c_it[1];
		}
	}
	g_follow_last_follow_wall_dir = 0;
	auto  path = make_unique<Cells>();
	if (is_found)
	{
		path->push_front(target);
		path->push_front(curr);
	}
	else
		ROS_INFO("%s %d: x pos:(%d,%d), x neg:(%d,%d), target not found.", __FUNCTION__, __LINE__, c_it[0].x, c_it[0].y, c_it[1].x, c_it[1].y);

	return path;
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

//bool NavCleanPathAlgorithm::filterPathsToSelectBestPath(GridMap &map, const Cells &targets, const Cell_t &curr, Cells &best_path, const Dir_t &last_dir) {
//	Cell_t min_cell,max_cell;
//
//
////	filters.push_back(&filter_p_1t);
////	filters.push_back(&filter_n3n_0t);
////	filters.push_back(&filter_n_1t);
////	filters.push_back(&filter_p_1000t);
////	filters.push_back(&filter_all);
//
//	for (auto &&filter : filters) {
//		filter->updateTargetAndRangeBound();
//		Cells filtered_targets{};
//		std::copy_if(targets.begin(), targets.end(), std::back_inserter(filtered_targets), [&filter](Cell_t target) {
//			return filter->Contain(target);
//		});
//
//		if (filtered_targets.empty())
//			continue;
//
////		displayTargetList(filtered_targets);
//		PathList paths{};
//		for (auto &target : filtered_targets) {
//			Cells path{};
//			Dir_t dir = MAP_POS_Y;
//			if (!filter->towardPos())
//				dir = MAP_NEG_Y;
//			findPath(map, curr, target, path, dir);
//			paths.push_back(path);
//		}
//
//		PathList filtered_paths{};
//		std::copy_if(paths.begin(), paths.end(), std::back_inserter(filtered_paths), BestTargetFilter(*filter));
//		if (!filtered_paths.empty()) {
//			best_path = *std::min_element(filtered_paths.begin(), filtered_paths.end(),
//																		MinYAndShortestPath(curr.y, filter->is_toward_pos_, filter->turn_count_));
//			ROS_INFO("path.len(%d)",size_of_path(best_path));
//			if (filter == &filter_p_1t) {
//				if(size_of_path(best_path) > 25){
//					ROS_WARN("path.len is too long to find other path(%d)",size_of_path(best_path));
//					beeper.debugBeep(VALID);
//					continue ;
//				}
//			}
//			curr_filter_ = filter;
//			return true;
//		}
//	}
//	return false;
//}

class IsTarget
{
public:
	IsTarget(GridMap* map,BoundingBox2 bound):map_(map), bound_(bound) { };
    bool operator()(const Cell_t &c_it) {
		return c_it.y % 2 == 0 && map_->getCell(CLEAN_MAP, c_it.x, c_it.y) == UNCLEAN &&
			   map_->isBlockAccessible(c_it.x, c_it.y) && bound_.Contains(c_it) &&bound_.Contains(c_it);
//			   && std::any_of(std::begin(cell_direction_4),std::end(cell_direction_4),[&](const Cell_t& cell){ return map_.getCell(CLEAN_MAP, cell.x, cell.y) == CLEANED;});
	}

private:
	GridMap* map_;
	BoundingBox2 bound_;
};
std::unique_ptr<std::deque<BestTargetFilter*>> NavCleanPathAlgorithm::generateBounds(GridMap& map, const Cell_t& curr, int bound_i,Dir_t last_dir) {

	std::deque<BestTargetFilter*> filters;
	ROS_WARN("last_dir(%d)\n",last_dir);
	filters.push_back(&filter_p0_0_p);
	filters.push_back(&filter_p0_0_n);

	if(isXAxis(last_dir))
	{
		if (isPos(last_dir))
		{
			if((map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x + 4), static_cast<int16_t>(curr.y)) == UNCLEAN
				&& map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x + 4), static_cast<int16_t>(curr.y - 2)) == CLEANED))
			filters.push_back(&filter_p0_1t_p);
			if((map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x + 4), static_cast<int16_t>(curr.y)) == UNCLEAN
				&& map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x + 4), static_cast<int16_t>(curr.y + 2)) == CLEANED))
				filters.push_back(&filter_n0_1t_p);
		}
		else //if (!isPos(last_dir))
		{
			if(map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x - 4), static_cast<int16_t>(curr.y)) == UNCLEAN
					&& map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x - 4), static_cast<int16_t>(curr.y - 2)) == CLEANED)
			filters.push_back(&filter_p0_1t_n);
			if(map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x - 4), static_cast<int16_t>(curr.y)) == UNCLEAN
			   && map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x - 4), static_cast<int16_t>(curr.y + 2)) == CLEANED)
			filters.push_back(&filter_n0_1t_n);
		}
	}

	if(curr.y%2 ==0)
		filters.push_back(&filter_p2_0t);
	else
		filters.push_back(&filter_p1_0t);

	filters.push_back(&filter_p3p_0t);

	if(curr.y%2 ==0)
		filters.push_back(&filter_n2_0t);
	else
		filters.push_back(&filter_n1_0t);

	filters.push_back(&filter_short_path);
	priority_dir = last_dir;
	return make_unique<std::deque<BestTargetFilter*>>(filters);

}
BoundingBox2 NavCleanPathAlgorithm::getLine(const Cell_t& curr,GridMap& map)
{
	Cell_t c_it[2];
	for (auto i = 0; i < 2; i++) {
			c_it[i] = curr;
			for (;; c_it[i] += cell_direction_[i]) {
                auto tmp = c_it[i] +cell_direction_[i]*2;
				if (map.cellIsOutOfRange(tmp)  || map.isBlocksAtY(tmp.x, tmp.y))
					break;
				if (map.getCell(CLEAN_MAP, c_it[i].x, c_it[i].y) == UNCLEAN)
					break;
			}
		}
		return {c_it[1], c_it[0]};
}
bool NavCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr_p, const Dir_t &last_dir, Points &plan_path)
{
	plan_path.clear();
	Cells path{};
	Cells targets{};
	curr_ = curr_p.toCell();
	map.markRobot(curr_, CLEAN_MAP);
	map_bound = map.genRange();
	curr_bound = getLine(curr_, map);
	auto filters = *generateBounds(map, curr_, pt_,last_dir);
	for(auto&&filter : filters)
	{
		curr_filter_=filter;
		filter->updateTargetAndRangeBound();
		filter->displayName();
		ROS_INFO("target_bound(%d,%d,%d,%d)",filter->target_bound.min.x, filter->target_bound.min.y, filter->target_bound.max.x, filter->target_bound.max.y);
		ROS_INFO("range_bound(%d,%d,%d,%d)",filter->range_bound.min.x, filter->range_bound.min.y, filter->range_bound.max.x, filter->range_bound.max.y);
		if(dijstra(map, curr_, path, IsTarget(&map,filter->target_bound), true, isAccessable(filter->range_bound,&map)))
			break;
//		map.print(curr_,COST_MAP,path);
	}

	if(path.empty())
		return false;

	optimizePath(map, path,last_dir);

	plan_path = *cells_generate_points(make_unique<Cells>(path));

	displayCellPath(path);
	map.print(curr_,COST_MAP,path);
	map.print(curr_p.toCell(), CLEAN_MAP, path);
	return true;
}
#else

class PathPlan{
public:
	PathPlan(Dir_t priority_dir,Cell_t& curr):priority_dir_(priority_dir),curr_(curr) {

	};
    bool operator()(const Cell_t& l, const Cell_t& r) {
		if(l.y == curr_.y && r.y == curr_.y) {
			if (isXAxis(priority_dir_)) {
				if (isPos(priority_dir_))
					return l.x < r.x;
				else
					return l.x > r.x;
			}
		}
		if(l.y == curr_.y && r.y != curr_.y)
			return false;
		if(l.y != curr_.y && r.y == curr_.y)
			return true;
        else{
			if(l.y == r.y)
			{
				return std::abs(l.x-curr_.x) > std::abs(r.x-curr_.x);
			}
            else
			{
                if(r.y -curr_.y > 0 && l.y - curr_.y < 0)
					return true;
				if(r.y -curr_.y < 0 && l.y - curr_.y > 0)
					return false;
				else if(r.y - curr_.y > 0 && l.y - curr_.y > 0)
				{
					return std::abs(r.y-curr_.y) < std::abs(l.y-curr_.y);
				}
                else/* if(r.y - curr_.y < 0 && l.y - curr_.y < 0)*/
				{
					return std::abs(r.y-curr_.y) < std::abs(l.y-curr_.y);
				}
			}
		}
	}

private:
    Dir_t priority_dir_;
	BoundingBox2 bound_;
	Cell_t curr_;
//	std::deque<Line> lines;
};


//range_type_t path_classity(std::unique_ptr<Cells> &cells, const Cell_t curr)
//{
//	auto size = cells->size();
//	auto dy = cells->back().y - curr.y;
//	auto dx = cells->back().x - curr.x;
//    if(size == 2)
//	{
//        if(dy == 0)
//		{
//			if(dx > 0)
//            	return range_type_t::X_Y_LANE;
//			if(dx < 0)
//				return range_type_t::CURR_LANE;
//		} else {
//			if(std::abs(dy <= 2)) {
//				if (dy > 0)
//					return range_type_t::Y_AXIS_POS_NEXT;
//				else
//					return range_type_t::Y_AXIS_NEG_NEXT;
//			}
//		}
//	}else if (size == 3){
//        if(dy > 0)
//		{
//			if(std::abs(dy <= 2))
//				return range_type_t::Y_AXIS_POS_NEXT;
//
//		}
//        if(dy < 0)
//		{
//			if(std::abs(dy <= 2))
//				return range_type_t::Y_AXIS_NEG_NEXT;
//		}
//	}else
//	{
//			return range_type_t::XY_AXIS_ANY;
//	}
//}

void path_opt(std::unique_ptr<Cells> &cells, const Cell_t& curr, int pt)
{
	if(pt == X_Y_LANE)
	{
		if(cells->back().x > curr.x)
			cells->back().x += 5;
		else
			cells->back().x -= 5;
	} else
	if(pt == CURR_LANE)
	{
        cells->emplace_back(cells->back()+Cell_t{0,-2});//for follow wall target
	} else
	if(pt == CURR_LANE_NEG)
	{
		cells->emplace_back(cells->back()+Cell_t{0,2});//for follow wall target
	}
}

std::unique_ptr<std::tuple<BoundingBox2, BoundingBox2,Dir_t>> NavCleanPathAlgorithm::generateBounds(GridMap& map, const Cell_t& curr, int bound_i,Dir_t last_dir) {
	BoundingBox2 bound_target;
	BoundingBox2 bound_valid;
    Dir_t priority_dir = last_dir;
	static BoundingBox2 curr_bound{};

	Cell_t c_it[2];
	if (bound_i == 0) {//same lane
		for (auto i = 0; i < 2; i++) {
			c_it[i] = curr;
			for (;; c_it[i] += cell_direction_[i]) {
                auto tmp = c_it[i] +cell_direction_[i]*2;
				if (map.cellIsOutOfRange(tmp)  || map.isBlocksAtY(tmp.x, tmp.y))
					break;
				if (map.getCell(CLEAN_MAP, c_it[i].x, c_it[i].y) == UNCLEAN)
					break;
			}
//			if (!map.isBlockAtY(BLOCKED_SLIP, c_it.c_it, c_it.y) &&
//				!map.isBlockAtY(BLOCKED_TILT, c_it.c_it, c_it.y) &&
//				!map.isBlockAtY(BLOCKED_RCON, c_it.c_it, c_it.y) &&
//				c_it[i] != curr_cell) {
//				c_it[i] += cell_direction_[i] * OVER_CELL_SIZE;
//			}
		}
		curr_bound = {c_it[1], c_it[0]};
        bound_valid = bound_target = curr_bound;

		if(!isXAxis(last_dir)) {
			if (g_follow_last_follow_wall_dir != 0) {
				ROS_ERROR("%s %d: g_follow_last_follow_wall_dir(%d)", __FUNCTION__, __LINE__,
						  g_follow_last_follow_wall_dir);
				if (1 == g_follow_last_follow_wall_dir)
					priority_dir = MAP_NEG_X;
				else//(g_follow_last_follow_wall_dir == 2)
					priority_dir = MAP_POS_X;
			}
			auto dir = lidar.compLaneDistance();
			if (dir != -1) {
				priority_dir = MAP_POS_X;
				if (dir == 1)
					priority_dir = MAP_NEG_X;
			}
			g_follow_last_follow_wall_dir = 0;
		}

	}else if (bound_i == 1) {//same lane front
		bound_target = {curr,curr};
		bound_valid = bound_target;
        if(isXAxis(last_dir)) {
			if (isPos(last_dir)
				&& map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x + 4), static_cast<int16_t>(curr.y)) == UNCLEAN
				&& map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x + 4), static_cast<int16_t>(curr.y - 2)) == CLEANED) {
				bound_target = {curr + Cell_t{3, 0}, curr + Cell_t{5, 0}};
				bound_valid = {curr - Cell_t{0, 2}, bound_target.max};
			} else if (!isPos(last_dir)
					   && map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x - 4), static_cast<int16_t>(curr.y)) == UNCLEAN
					   && map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x - 4), static_cast<int16_t>(curr.y - 2)) == CLEANED) {
				bound_target = {curr - Cell_t{5, 0}, curr - Cell_t{3, 0}};
				bound_valid = {bound_target.min - Cell_t{0, 2}, curr};
			}
		}
	} else if (bound_i == 2) {//same lane neg
		bound_target = {curr,curr};
		bound_valid = bound_target;
		if(isXAxis(last_dir)) {
			if (isPos(last_dir)
				&& map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x + 4), static_cast<int16_t>(curr.y)) == UNCLEAN
				&& map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x + 4), static_cast<int16_t>(curr.y + 2)) == CLEANED) {
				bound_target = {curr + Cell_t{3, 0}, curr + Cell_t{5, 0}};
				bound_valid = {curr, bound_target.max + Cell_t{0, 2}};
			} else if (!isPos(last_dir)
					   && map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x - 4), static_cast<int16_t>(curr.y)) == UNCLEAN
					   && map.getCell(CLEAN_MAP, static_cast<int16_t>(curr.x - 4), static_cast<int16_t>(curr.y + 2)) == CLEANED) {
				bound_target = {curr - Cell_t{5, 0}, curr - Cell_t{3, 0}};
				bound_valid = {bound_target.min, curr + Cell_t{0, 2}};
			}
		}
	}
	else if(bound_i == 3){//1 pos
		bound_target = {curr_bound.min + Cell_t{0, 2}, curr_bound.max + Cell_t{0, 2}};
		bound_valid = {bound_target.min - Cell_t{0,2}, bound_target.max};
	}else if (bound_i == 4) {//n pos
		bound_target = {Cell_t{map.generateBound2().min.x ,curr.y}, map.generateBound2().max};
		bound_valid = bound_target;
	}else if (bound_i == 5) {//1 neg
		bound_target = {curr_bound.min + Cell_t{0, -2}, curr_bound.max + Cell_t{0, -2}};
		bound_valid = {bound_target.min, Cell_t{bound_target.max.x, curr.y}};
	} else if (bound_i == 6) {// n neg
		bound_target = map.generateBound2();
		bound_valid = bound_target;
	}
	return make_unique<std::tuple<BoundingBox2, BoundingBox2, Dir_t>>(bound_target, bound_valid, priority_dir);
}
bool NavCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr_p, const Dir_t &last_dir, Points &plan_path)
{
	plan_path.clear();
	auto curr = curr_p.toCell();
	map.markRobot(CLEAN_MAP);
	pt_ = X_Y_LANE;
    std::unique_ptr<Cell_t> target;
	std::unique_ptr<Cells> cells;

    for(; pt_ < RANGE_END; ++pt_) {
		BoundingBox2 b1,b2;
		Dir_t pri_dir;
		std::tie(b1, b2, pri_dir) = *generateBounds(map, curr, pt_,last_dir);
		printf("pt_(%d),target bounds(%d,%d, %d,%d) ",pt_, b1.min.x, b1.min.y, b1.max.x, b1.max.y);
		printf("valid bounds(%d,%d, %d,%d),pri_dir(%d)\n", b2.min.x, b2.min.y, b2.max.x, b2.max.y,pri_dir);
		target = find_target(curr, IsTarget(map,b2),
								  std::bind(&APathAlgorithm::isAccessible, this, std::placeholders::_1, b2, map),
								  PathPlan(last_dir, curr));

		if (target == nullptr)
			continue;

        printf("find target(%d,%d)!!\n",target->x, target->y);
		cells = shortestPath(curr, *target, std::bind(&APathAlgorithm::isAccessible, this, std::placeholders::_1, b2, map), pri_dir);
		break;
	}

    if(pt_ == RANGE_END)
		return false;
	path_crop(cells);
	printf("target(%d,%d, path_type(%d))\n",target->x, target->y,Y_AXIS_NEG_NEXT);
	path_opt(cells, curr, pt_);
    map.print(curr_p.toCell(), CLEAN_MAP, *cells);
	displayCellPath(*cells);
	plan_path = *cells_generate_points(cells);
	return true;
}

#endif


bool NavCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	if(robot::instance()->p_mode->getNextMode() == Mode::cm_navigation) {
		return checkTrappedUsingDijkstra(map, curr_cell);
	}
	else if(robot::instance()->p_mode->getNextMode() == Mode::cm_exploration) {
		Cells cells{};
		auto p_cm = boost::dynamic_pointer_cast<CleanModeExploration>(robot::instance()->p_mode);
		return !(dijstra(p_cm->clean_map_, getPosition().toCell(), cells,EqualTarget(Cell_t{0,0}),true,isAccessable(p_cm->clean_map_.genRange(), &p_cm->clean_map_)));
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


bool shift_path(GridMap &map, const Cell_t &p1, Cell_t &p2, Cell_t &p3, int num,bool is_first, bool is_reveave) {
	auto dir_p23 = get_dir(p3, p2);
	auto dir_p12 = is_reveave ? get_dir(p1, p2) : get_dir(p2, p1);
//	ROS_INFO("dir_p12(%d), dir_p23(%d)", dir_p12, dir_p23);
	auto is_break = false;
	auto p12_it = p2;
	auto i = 1;
	for (; i <= num * 2; i++) {
		p12_it += cell_direction_[dir_p12];
//		ROS_ERROR("p12_it,%d,%d", p12_it.x, p12_it.y);
		for (auto p23_it = p12_it; p23_it != p3 + cell_direction_[dir_p12] * i+cell_direction_[dir_p23]; p23_it += cell_direction_[dir_p23]) {
			ROS_WARN("p23_it,%d,%d", p23_it.x, p23_it.y);
			if (!map.isBlockAccessible(p23_it.x, p23_it.y)) {
				is_break = true;
				break;
			}
		}
		if(is_break)
			break;
	}
	if (i > 1) {
		auto shift = (p12_it - p2);
		if(is_first)
			shift /= 2;
		ROS_ERROR("(shift(%d,%d),", shift.x, shift.y);
		p2 += shift;
		p3 += shift;
		return shift != Cell_t{0,0};
	}
	return false;
}


void NavCleanPathAlgorithm::optimizePath(GridMap &map, Cells &path, Dir_t last_dir) {

	ROS_INFO("Step 5:optimizePath");
	if(curr_filter_ == &filter_p0_0_p || curr_filter_ == &filter_p0_0_n) {
		ROS_INFO("filter_p0_0_p:curr line");
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
					}
					break;
				}
				if(map.cellIsOutOfRange(tmp))
					break;
			}
		c_it = tmp;
		map.cellPreventOutOfRange(c_it);
		}
	}
	else if (curr_filter_ == &filter_p0_1t_n || curr_filter_ == &filter_p0_1t_p)
		path.push_back(
				Cell_t{path.back().x, static_cast<int16_t>(path.front().y - 3)});//for setting follow wall target line
	else if (curr_filter_ == &filter_n0_1t_n || curr_filter_ == &filter_n0_1t_p)
		path.push_back(
				Cell_t{path.back().x, static_cast<int16_t>(path.front().y + 3)});//for setting follow wall target line
	else {
		displayCellPath(path);
		if(path.size() > 2)
		{
			if(is_opposite_dir(get_dir(path.begin()+1, path.begin()), last_dir) ||
					(path.begin()->y%2 == 1 && isXAxis(last_dir) && get_dir(path.begin()+1, path.begin()) == (last_dir)))
			{
				ROS_WARN("opposite dir");
				ROS_INFO("dir(%d,%d)",get_dir(path.begin()+1, path.begin()), last_dir);
				beeper.debugBeep(INVALID);
				auto tmp = path.front();
				auto iterator = path.begin();
				if(shift_path(map, *(iterator + 2), *(iterator + 1), *(iterator + 0),1,true,true))
				{
					if(*(iterator + 1) == *(iterator + 2))
						path.erase(path.begin()+1);
					path.push_front(tmp);
				}
			}
		}
		if (path.size() > 3) {
			ROS_INFO(" size_of_path > 3 Optimize path for adjusting it away from obstacles..");
			displayCellPath(path);
			for (auto iterator = path.begin(); iterator != path.end() - 3; ++iterator) {
				ROS_INFO("dir(%d), y(%d)", get_dir(iterator + 1, iterator + 2), (iterator+1)->y);
				if(isXAxis(get_dir(iterator + 1, iterator + 2)) && (iterator+1)->y % 2 == 1) {
					ROS_WARN("in odd line ,try move to even line(%d)!", (iterator + 1)->x);
					shift_path(map, *iterator, *(iterator + 1), *(iterator + 2), 1, false,false);
				}else{
					ROS_INFO("in x dir, is in even line try mv to even");
					auto num = isXAxis(get_dir(iterator + 1, iterator + 2)) ? 2 : 1;
					shift_path(map, *iterator, *(iterator + 1), *(iterator + 2),num,true,false);
				}
			}
		}
	}
}
