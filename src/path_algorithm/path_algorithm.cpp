//
// Created by austin on 17-12-3.
//

#include <queue>
#include <log.h>
#include "ros/ros.h"
#include "robot.hpp"
#include "path_algorithm.h"
class PriorityQueueElement{
public:
	PriorityQueueElement() { };
	PriorityQueueElement(const Cell_t& elem, int score, bool removed):elem_(elem),score_(score),removed_(removed) { };

	bool operator<(const PriorityQueueElement& other) const{
		return score_ > other.score_;
	}

    Cell_t elem_{};
    int score_{};
    bool removed_{};

};

class PriorityQueue {
public:
	PriorityQueue(std::function<int(const Cell_t&)> key)
	:key_get_score_(std::move(key)){
	}
//	:key_get_score_(std::move(key)){
//	}
	bool contains(Cell_t& item) { return map_.find(item) != map_.end(); }

	void push(const Cell_t& item) {
		PriorityQueueElement e{item, key_get_score_(item),false};
		pq_.push(e);
        map_[item] = e;
	}

	void remove(Cell_t& elem){
		map_[elem].removed_ = true;
	}

	Cell_t pop() {
        while (true){
            auto e = pq_.top();
            pq_.pop();
            if (!e.removed_){
				map_.erase(e.elem_);
				return e.elem_;
			}
    	}
	}

	bool empty() const { return map_.empty(); }

private:
	std::function<int(const Cell_t&)> key_get_score_;
    std::priority_queue<PriorityQueueElement,std::vector<PriorityQueueElement>, std::less<PriorityQueueElement>> pq_;
	std::map<Cell_t,PriorityQueueElement> map_;
};

const Cell_t cell_direction_[9]{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1},{0,0}};
const Cell_t cell_direction_4[4]{{1,0},{-1,0},{0,1},{0,-1}};

//bool APathAlgorithm::generateShortestPath(GridMap &map, const Point_t &curr,const Point_t &target, const Dir_t &last_dir, Points &plan_path) {
//	Cell_t corner1 ,corner2;
//	auto path_cell = findShortestPath(map, curr.toCell(), target.toCell(),last_dir, false,false,corner1,corner2);
//
//	plan_path = *cells_to_points(make_unique<Cells>(path_cell));
//}

//Cells APathAlgorithm::findShortestPath(GridMap &map, const Cell_t &start, const Cell_t &target,
//										const Dir_t &last_dir, bool use_unknown,bool bound ,Cell_t min_corner,Cell_t max_corner)
//{
//	Cells path_{};
//	// limit cost map range or get the total map range.
//	int16_t x_min, x_max, y_min, y_max;
//	if(bound){
//		x_min = min_corner.x;
//		y_min = min_corner.y;
//		x_max = max_corner.x;
//		y_max = max_corner.y;
//	}
//	else
//		map.getMapRange(&x_min, &x_max, &y_min, &y_max);
//
//	// Reset the COST_MAP.
//	map.reset(COST_MAP);
//
//	// Mark obstacles in COST_MAP
//	for (int16_t i = x_min - 1; i <= x_max + 1; ++i) {
//		for (int16_t j = y_min - 1; j <= y_max + 1; ++j) {
//			CellState cs = map.getCost(i, j);
//			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
//				//for (m = ROBOT_RIGHT_OFFSET + 1; m <= ROBOT_LEFT_OFFSET - 1; m++)
//				for (int16_t m = ROBOT_RIGHT_OFFSET; m <= ROBOT_LEFT_OFFSET; m++) {
//					for (int16_t n = ROBOT_RIGHT_OFFSET; n <= ROBOT_LEFT_OFFSET; n++) {
//						map.setCost(COST_MAP, (i + m), (j + n), COST_HIGH);
//					}
//				}
//			}
//			else if(cs == UNCLEAN && !use_unknown)
//				map.setCost(COST_MAP, i, j, COST_HIGH);
//		}
//	}
//
//	// For protection, the target cell must be reachable.
//	if (map.getCost(COST_MAP, target.x, target.y) == COST_HIGH)
//	{
//		ROS_ERROR("%s %d: Target cell has high cost(%d)! This target should be filtered before calling this function.",
//				  __FUNCTION__, __LINE__, map.getCost(COST_MAP, target.x, target.y));
//		return path_;
//	}
//
//	// Set for target cell. For reverse algorithm, we will generate a-star map from target cell.
//	map.setCost(COST_MAP, target.x, target.y, COST_1);
//
//	// For protection, the start cell must be reachable.
//	if (map.getCost(COST_MAP, start.x, start.y) == COST_HIGH)
//	{
//		ROS_ERROR("%s %d: Start cell has high cost(%d)! It may cause bug, please check.",
//							__FUNCTION__, __LINE__, map.getCost(COST_MAP, start.x, start.y));
//		map.print(getPosition().toCell(), COST_MAP, Cells{target});
//		map.setCost(COST_MAP, start.x, start.y, COST_NO);
//	}
//
//	/*
//	 * Find the path to target from the start cell. Set the cell values
//	 * in COST_MAP to 1, 2, 3, 4 or 5. This is a method like A-Star, starting
//	 * from a start point, init the cells one level away, until we reach the target.
//	 */
//	int16_t offset = 0;
//	bool cost_updated = true;
//	int16_t cost_value = 1;
//	int16_t next_cost_value = 2;
//	while (map.getCost(COST_MAP, start.x, start.y) == COST_NO && cost_updated) {
//		offset++;
//		cost_updated = false;
//
//		/*
//		 * The following 2 for loops is for optimise the computational time.
//		 * Since there is not need to go through the whole COST_MAP for searching the
//		 * cell that have the next pass value.
//		 *
//		 * It can use the offset to limit the range of searching, since in each loop
//		 * the cell (x -/+ offset, y -/+ offset) would be set only. The cells far away
//		 * to the robot position won'trace_cell be set.
//		 */
//		for (auto i = target.x - offset; i <= target.x + offset; i++) {
//			if (i < x_min || i > x_max)
//				continue;
//
//			for (auto j = target.y - offset; j <= target.y + offset; j++) {
//				if (j < y_min || j > y_max)
//					continue;
//
//				/* Found a cell that has a pass value equal to the current pass value. */
//				if(map.getCost(COST_MAP, i, j) == cost_value) {
//					/* Set the lower cell of the cell which has the pass value equal to current pass value. */
//					for(auto index = 0;index<4; index++) {
//						auto neighbor = Cell_t(i, j) - cell_direction_[index];
//						if (map.getCost(COST_MAP, neighbor.x, neighbor.y) == COST_NO) {
//							map.setCost(COST_MAP, neighbor.x, neighbor.y, (CellState) next_cost_value);
//							cost_updated = true;
//						}
//					}
//				}
//			}
//		}
//
//		/* Update the pass value. */
//		cost_value = next_cost_value;
//		next_cost_value++;
//
//		/* Reset the pass value, pass value can only between 1 to 5. */
//		if(next_cost_value == COST_PATH)
//			next_cost_value = 1;
//	}
//
//	// If the start cell still have a cost of 0, it means target is not reachable.
//	CellState start_cell_state = map.getCost(COST_MAP, start.x, start.y);
//	if (start_cell_state == COST_NO || start_cell_state == COST_HIGH) {
//		ROS_WARN("%s, %d: Target (%d, %d) is not reachable for start cell(%d, %d)(%d), return empty path.",
//						 __FUNCTION__, __LINE__, target.x, target.y, start.x, start.y, start_cell_state);
//#if	DEBUG_COST_MAP
//		map.print(getPosition().toCell(),COST_MAP, Cells{target});
//#endif
//		// Now the path_ is empty.
//		return path_;
//	}
//
//	/*
//	 * Start from the start position, trace back the path by the cost level.
//	 * Value of cells on the path is set to 6. Stops when reach the target
//	 * position.
//	 *
//	 * The last robot direction is use, this is to avoid using the path that
//	 * have the same direction as previous action.
//	 */
//	Cell_t trace_cell;
//	int16_t trace_x, trace_y, trace_x_last, trace_y_last, total_cost = 0;
//	trace_cell.x = trace_x = trace_x_last = start.x;
//	trace_cell.y = trace_y = trace_y_last = start.y;
//	path_.push_back(trace_cell);
//
//	uint16_t next = 0;
//	auto trace_dir = (last_dir == MAP_POS_Y || last_dir == MAP_NEG_Y) ? 1: 0;
//	//ROS_INFO("%s %d: trace dir: %d", __FUNCTION__, __LINE__, trace_dir);
//	while (trace_x != target.x || trace_y != target.y) {
//		CellState cost_at_cell = map.getCost(COST_MAP, trace_x, trace_y);
//		auto target_cost = static_cast<CellState>(cost_at_cell - 1);
//
//		/* Reset target cost to 5, since cost only set from 1 to 5 in the COST_MAP. */
//		if (target_cost == 0)
//			target_cost = COST_5;
//
//		/* Set the cell value to 6 if the cells is on the path. */
//		map.setCost(COST_MAP, (int32_t) trace_x, (int32_t) trace_y, COST_PATH);
//
//#define COST_SOUTH	{											\
//				if (next == 0 && (map.getCost(COST_MAP, trace_x - 1, trace_y) == target_cost)) {	\
//					trace_x--;								\
//					next = 1;								\
//					trace_dir = 1;								\
//				}										\
//			}
//
//#define COST_WEST	{											\
//				if (next == 0 && (map.getCost(COST_MAP, trace_x, trace_y - 1) == target_cost)) {	\
//					trace_y--;								\
//					next = 1;								\
//					trace_dir = 0;								\
//				}										\
//			}
//
//#define COST_EAST	{											\
//				if (next == 0 && (map.getCost(COST_MAP, trace_x, trace_y + 1) == target_cost)) {	\
//					trace_y++;								\
//					next = 1;								\
//					trace_dir = 0;								\
//				}										\
//			}
//
//#define COST_NORTH	{											\
//				if (next == 0 && map.getCost(COST_MAP, trace_x + 1, trace_y) == target_cost) {	\
//					trace_x++;								\
//					next = 1;								\
//					trace_dir = 1;								\
//				}										\
//			}
//
//		next = 0;
//		if (trace_dir == 0) {
//			COST_WEST
//			COST_EAST
//			COST_SOUTH
//			COST_NORTH
//		} else {
//			COST_SOUTH
//			COST_NORTH
//			COST_WEST
//			COST_EAST
//		}
//
//#undef COST_EAST
//#undef COST_SOUTH
//#undef COST_WEST
//#undef COST_NORTH
//
//		total_cost++;
//		if (path_.back().x != trace_x && path_.back().y != trace_y) {
//			// Only turning cell will be pushed to path.
//			trace_cell.x = trace_x_last;
//			trace_cell.y = trace_y_last;
//			path_.push_back(trace_cell);
//		}
//		trace_x_last = trace_x;
//		trace_y_last = trace_y;
//	}
//
//	map.setCost(COST_MAP, (int32_t) target.x, (int32_t) target.y, COST_PATH);
//
//	trace_cell = target;
//	path_.push_back(trace_cell);
//
//	displayCellPath(path_);
//
//	return path_;
//}

inline uint16_t heuristic_estimate_of_distance(const Cell_t& curr,const Cell_t& goal){
	return static_cast<uint16_t>(std::abs(curr.x - goal.x) + std::abs(curr.y - goal.y));
};

std::unique_ptr<Cells> reconstruct_path(std::map<Cell_t,Cell_t>& cameFrom,Cell_t& current){
	auto total_path = make_unique<Cells>();
	total_path->emplace_front(current);
	while(std::any_of(cameFrom.begin(), cameFrom.end(),[&](std::pair<Cell_t,Cell_t> t_map){return t_map.first == current;}))
	{
		current = cameFrom[current];
		total_path->emplace_front(current);
	}
	return total_path;
}

uint16_t distween_between(const Cell_t &curr, const Cell_t &neighbor) {
	auto dir = get_dir(curr, neighbor);
//	printf("(%d),nei(%d,%d),curr(%d,%d)~~~~~~~~~~~~~~\n",dir,neighbor.x,neighbor.y,curr.x,curr.y);
	return static_cast<uint16_t>(get_dir(neighbor, curr) < 4 ? 10 : 14);
}
bool APathAlgorithm::AStarIsAccessible(const Cell_t &c_it, const BoundingBox2 &bound, GridMap &map) {
    return bound.Contains(c_it) && map.isBlockAccessible(c_it.x, c_it.y);
}
class Neighbors{
public:
	Neighbors(const Cell_t& curr,Dir_t dir) {
//		assert(dir >=0 && dir<4);
        if(!(dir >=0 && dir<4))
		{
//			dir = MAP_POS_X;
            ROS_WARN("dir(%d) is bad",dir);
		}
//        printf("%d", dir);
        if(dir == 0 || dir == MAP_ANY)
		{
			cells_.emplace_back(curr + cell_direction_[1]);
			cells_.emplace_back(curr + cell_direction_[2]);
			cells_.emplace_back(curr + cell_direction_[3]);
			cells_.emplace_back(curr + cell_direction_[0]);
		} else if(dir == 1){
			cells_.emplace_back(curr + cell_direction_[0]);
			cells_.emplace_back(curr + cell_direction_[2]);
			cells_.emplace_back(curr + cell_direction_[3]);
			cells_.emplace_back(curr + cell_direction_[1]);
		} else if(dir == 2){
			cells_.emplace_back(curr + cell_direction_[3]);
			cells_.emplace_back(curr + cell_direction_[0]);
			cells_.emplace_back(curr + cell_direction_[1]);
			cells_.emplace_back(curr + cell_direction_[2]);
		}else {
			cells_.emplace_back(curr + cell_direction_[2]);
			cells_.emplace_back(curr + cell_direction_[0]);
			cells_.emplace_back(curr + cell_direction_[1]);
			cells_.emplace_back(curr + cell_direction_[3]);
		}
	}
    Cells::iterator begin()
	{

		return cells_.begin();
	}
	Cells::iterator end()
	{
		return cells_.end();
	}


private:
	Cells cells_;
};
std::unique_ptr<Cells> APathAlgorithm::shortestPath(const Cell_t &start, const Cell_t& goal, const cmp_condition_t is_accessible, Dir_t dir)
{
    auto closedSet = std::multiset<Cell_t>();
	std::map<Cell_t,Cell_t> cameFrom{};
	std::map<Cell_t,uint16_t> g_score{{start, 0}};
	std::map<Cell_t,uint16_t> h_score{{start, heuristic_estimate_of_distance(start, goal)}};
	PriorityQueue openSet([&](const Cell_t& cell){ return g_score[cell] + h_score[cell];});
	openSet.push(start);
    while (!openSet.empty()){
        auto current = openSet.pop();
//        printf("curr(%d,%d)\n",current.x, current.y);
        if (goal == current)
		{
//			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!curr(%d,%d)\n",current.x, current.y);
			return reconstruct_path(cameFrom, current);
		}
        closedSet.insert(current);
		for (auto& neighbor : Neighbors(current, (current == start) ? dir : get_dir(current, cameFrom[current]))) {
			if (std::any_of(closedSet.begin(), closedSet.end(), CellEqual(neighbor)))
				continue;
			if(!is_accessible(neighbor))
				continue;

			auto tentative_gScore = g_score[current] + distween_between(current, neighbor);
			if (openSet.contains(neighbor))
			{
				if (tentative_gScore >= g_score[neighbor])
					continue;
				openSet.remove(neighbor);
			}
			cameFrom[neighbor] = current;
			g_score[neighbor] = tentative_gScore;
			h_score[neighbor] = heuristic_estimate_of_distance(neighbor, goal);
			openSet.push(neighbor);
		}
	}

    return {};
}

bool APathAlgorithm::checkTrappedUsingDijkstra(GridMap &map, const Cell_t &curr_cell)
{
	Cells targets;
	// Find if there is uncleaned area.
	ROS_INFO("%s %d: Check for uncleaned target", __FUNCTION__, __LINE__);
	auto expand_condition = [&](const Cell_t &cell, const Cell_t &neighbor_cell)
	{
		return map.isBlockAccessible(neighbor_cell.x, neighbor_cell.y);
	};

	map.print(curr_cell,targets);
	auto found_unclean_target = dijkstra(map, curr_cell, targets, true,
												 IsTarget(&map, map.genTargetRange()),
												 isAccessible(&map,expand_condition));

	if (found_unclean_target)
	{
		ROS_INFO("%s %d: Found uncleaned target", __FUNCTION__, __LINE__);
		return false;
	}

//	ROS_INFO("Use clean area proportion to judge if it is trapped.");
	auto dijkstra_cleaned_count = dijkstraCountCleanedArea(map, getPosition(), targets);

	auto map_cleand_count = map.getCleanedArea();
	double clean_proportion = static_cast<double>(dijkstra_cleaned_count) / static_cast<double>(map_cleand_count);
	ROS_INFO("%s %d: dijkstra_cleaned_count(%d), map_cleand_count(%d), clean_proportion(%f) ,when prop < 0,8 is trapped",
					 __FUNCTION__, __LINE__, dijkstra_cleaned_count, map_cleand_count, clean_proportion);
	return (clean_proportion < 0.8);
}

bool APathAlgorithm::isTargetReachable(GridMap map,Cell_t target)
{
	for (int16_t i = target.x - 1; i <= target.x + 1; ++i) {
		for (int16_t j = target.y- 1; j <= target.y + 1; ++j) {
			CellState cs = map.getCost(i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				return false;
			}
		}
	}
	return true;

}

std::unique_ptr<Cell_t> APathAlgorithm::find_target(const Cell_t &start, cmp_one is_target, cmp_one is_accessible,
                                                    const cmp_two &cmp_lambda) {

	std::priority_queue<Cell_t,std::vector<Cell_t>, decltype(cmp_lambda)> openSet(cmp_lambda);
    std::set<Cell_t> open_set;
	std::set<Cell_t> closedSet;
	openSet.push(start);
	open_set.insert(start);

	while (!openSet.empty()) {

		auto current = openSet.top();

        if (is_target(current)) {
			return make_unique<Cell_t>(current);
		}
		closedSet.insert(current);
		openSet.pop();
		open_set.erase(current);

		for (auto index = 0; index < 4; index++) {
				auto neighbor = current + cell_direction_[index];

			if (std::any_of(closedSet.begin(), closedSet.end(), CellEqual(neighbor)))
				continue;

            if (!is_accessible(neighbor))
				continue;

            if(open_set.find(neighbor) != open_set.end())
				continue;

			openSet.push(neighbor);
            open_set.insert(neighbor);
		}
	}
	return {};
}


void APathAlgorithm::findPath(GridMap &map, const Cell_t &start, const Cell_t &target,
																		 Cells &path,
																		 Dir_t last_i) {
	if(start == target)
	{
		path.push_front(target);
		return;
	}
	auto cost = map.getCost(target.x, target.y);
	auto iterator = target;
//	printf("findPath, start(%d,%d),target(%d,%d)\n",start.x, start.y, target.x, target.y);
//	map.print(getPosition().toCell(), COST_MAP, Cells{});
	for (; iterator != start;) {
		if(map.getCost(iterator.x, iterator.y) != cost)
		{
			printf("start(%d,%d) iterator(%d,%d),target(%d,%d)cost(%d,%d)\n",start.x, start.y, iterator.x, iterator.y,target.x, target.y, cost,
				   map.getCost(iterator.x, iterator.y) );
			map.print(getPosition().toCell(), Cells{target});
			map.print(getPosition().toCell(), Cells{});
			ROS_ASSERT(map.getCost(iterator.x, iterator.y) == cost);
		}
		cost -= 1;
		if(cost == 0)
			cost = 5;
		for (auto i = 0; i < 4; i++) {
			auto neighbor = iterator + cell_direction_[(last_i + i) % 4];
//			printf("iterator(%d,%d)cost(%d,%d)\n", iterator.x, iterator.y, cost,map.getCost(COST_MAP, iterator.x, iterator.y));
			if (map.cellIsOutOfTargetRange(neighbor))
				continue;

			if (map.getCost(neighbor.x, neighbor.y) == cost) {
//				printf("~~iterator(%d,%d)cost(%d,%d)\n", iterator.x, iterator.y, cost,map.getCost(COST_MAP, iterator.x, iterator.y));
				if (i != 0 || path.empty()) {
					last_i = (last_i + i) % 4;
					path.push_front(iterator);
				}
				iterator = neighbor;
				break;
			}
		}
	}
	if(path.empty())
		path.push_front(target);
	if (path.back() != target)
		path.push_back(target);
	path.push_front(start);
//	displayCellPath(path);
}


bool APathAlgorithm::shift_path(GridMap &map, const Cell_t &p1, Cell_t &p2, Cell_t &p3, int num,bool is_half, bool is_reveave, const func_compare_two_t& expand_condition) {
	auto dir_p23 = get_dir(p3, p2);
	auto dir_p12 = is_reveave ? get_dir(p1, p2) : get_dir(p2, p1);
//	ROS_INFO("dir_p12(%d), dir_p23(%d)", dir_p12, dir_p23);
	auto is_break = false;
	auto p12_it = p2;
	auto p12_it_tmp = p2;
	auto i = 1;
	for (; i <= num; i++) {
		p12_it_tmp += cell_direction_[dir_p12];
//		ROS_ERROR("p12_it_tmp,%d,%d", p12_it_tmp.x, p12_it_tmp.y);
		for (auto p23_it = p12_it_tmp; p23_it != p3 + cell_direction_[dir_p12] * i+cell_direction_[dir_p23]; p23_it += cell_direction_[dir_p23]) {
//			ROS_WARN("p23_it,%d,%d", p23_it.x, p23_it.y);
			if (!expand_condition(p23_it, p23_it)) {
				is_break = true;
				break;
			}
		}
		if(is_break)
			break;
		p12_it = p12_it_tmp;
	}
	if (i > 1) {
		auto shift = (p12_it - p2);
		if(is_half)
			shift /= 2;
		ROS_INFO("(shift(%d,%d),", shift.x, shift.y);
		p2 += shift;
		p3 += shift;
		return shift != Cell_t{0,0};
	}
	return false;
}


void APathAlgorithm::flood_fill(const Cell_t& curr)
{
//	if(is_target(curr))
//	{
//		printf("%s,%d\n",__FUNCTION__, __LINE__);
//		return true;
//	}
//	printf("%s,%d,curr(%d,%d)\n",__FUNCTION__, __LINE__,curr.x, curr.y);
//	for(auto i =0; i<4 ;i ++)
//	{
////		printf("%s,%d\n",__FUNCTION__, __LINE__);
//		if(!map_out_range())
//        	flood_fill(curr + cell_direction_[i]);
//	}
}

bool APathAlgorithm::dijkstra(GridMap &map, const Cell_t &curr_cell, Cells &targets, bool is_stop,
							  func_compare_t is_target, func_compare_two_t isAccessable) {
	typedef std::multimap<int16_t, Cell_t> Queue;
	typedef std::pair<int16_t, Cell_t> Entry;

	GridMap closeSet{};
	Queue queue;
	closeSet.setCost(curr_cell.x, curr_cell.y, 1);
	queue.emplace(1, curr_cell);

	while (!queue.empty()) {
//		 Get the nearest next from the queue
		if (queue.begin()->first == 5) {
			Queue tmp_queue;
			std::for_each(queue.begin(), queue.end(), [&](const Entry &iterators) {
				tmp_queue.emplace(0, iterators.second);
			});
			queue.swap(tmp_queue);
		}
		auto start = queue.begin();
		auto next = start->second;
		auto cost = start->first;
		queue.erase(start);
		if (is_target(next))
		{
			if(is_stop)
			{
				ROS_INFO("find target(%d,%d)",next.x, next.y);
				findPath(closeSet,curr_cell,next, targets,MAP_POS_X);
				closeSet.print(curr_cell,targets);
				return true;
			} else{
				targets.push_back(next);
			}
		}
		for (auto index = 0; index < 4; index++) {

			auto neighbor = next + cell_direction_[index];

			if (!isAccessable(next, neighbor)) // access
				continue;

			if (closeSet.getCost(neighbor.x, neighbor.y) != 0)//close set
				continue;

			queue.emplace(cost + 1, neighbor);
			closeSet.setCost(neighbor.x, neighbor.y, cost + 1);
		}
	}
//	closeSet.print(curr_cell,targets);
	return !targets.empty();
}

uint16_t APathAlgorithm::dijkstraCountCleanedArea(GridMap& map, Point_t curr, Cells &targets)
{
	std::set<Cell_t> c_cleans;
	auto count_condition = [&](const Cell_t cell){
		for (int16_t x = -2; x <= 2; x++)
		{
			for (int16_t y = -2; y <= 2; y++)
			{
				auto tmp = cell;
				tmp.x += x;
				tmp.y += y;
				if (map.getCost(tmp.x, tmp.y) == CLEANED)
					c_cleans.insert(tmp);
			}
		}
		return false;
	};

	auto expand_condition = [&](const Cell_t cell, const Cell_t neighbor_cell){
		return map.isBlockAccessible(neighbor_cell.x, neighbor_cell.y) &&
				map.getCost(neighbor_cell.x, neighbor_cell.y) == CLEANED;
	};

	// Count the cleaned cells using dijkstra algorithm.
	dijkstra(map, curr.toCell(), targets, false, count_condition, isAccessible(&map,expand_condition));

	return static_cast<uint16_t>(c_cleans.size());
}

void APathAlgorithm::optimizePath(GridMap &map, Cells &path, const Dir_t& priority_dir,const func_compare_two_t& expand_condition)
{
	displayCellPath(path);
	auto is_had_move_start_point=false;
	if (path.size() > 2)
	{
		if (is_opposite_dir(get_dir(path.begin() + 1, path.begin()), priority_dir) ||
			(path.begin()->y % 2 == 1 && isXAxis(priority_dir) &&
			 get_dir(path.begin() + 1, path.begin()) == (priority_dir)))
		{
			ROS_WARN("opposite dir");
			ROS_INFO("dir(%d,%d)", get_dir(path.begin() + 1, path.begin()), priority_dir);
//			beeper.debugBeep(INVALID);
			auto tmp = path.front();
			auto iterator = path.begin();
			if (shift_path(map, *(iterator + 2), *(iterator + 1), *(iterator + 0), 1, true, true, expand_condition))
			{
				if (*(iterator + 1) == *(iterator + 2))
					path.erase(path.begin() + 1);
				path.push_front(tmp);
				is_had_move_start_point = true;
				ROS_WARN("opposite dir move success");
			}
			path.erase(std::unique(path.begin(), path.end()), path.end());
		}
	}
	if (path.size() > 3)
	{
		ROS_INFO(" size_of_path > 3 Optimize path for adjusting it away from obstacles..");
		displayCellPath(path);

		auto path_tmp = path;
		std::vector<Cells::iterator> path_its{};

		auto iterator = path_tmp.begin();

		if(is_had_move_start_point)
			iterator++;


		for (; iterator != path_tmp.end() - 3; ++iterator)
		{
			auto diff_c = *(iterator+3) - *(iterator+2);
			auto dir_3_2 = get_dir(iterator+3, iterator + 2);
			auto limit_num = is_opposite_dir(get_dir(iterator+1, iterator), dir_3_2) ? 2 : (std::abs((isXAxis(dir_3_2) ?diff_c.x : diff_c.y) - 1));
			ROS_INFO("diff_c(%d,%d), limit_num(%d)", diff_c.x, diff_c.y, limit_num);
			if(shift_path(map, *iterator, *(iterator + 1), *(iterator + 2), limit_num, false, false,expand_condition))
				path_its.push_back((iterator + 1));
		}
		displayCellPath(path);
		displayCellPath(path_tmp);
		for(auto && curr_it : path_its) {
			auto next_it_i = std::distance(path_tmp.begin(), curr_it) + 1;
			ROS_INFO("dir %d,path[%d](%d,%d), path_tmp[%d](%d,%d)", get_dir(curr_it - 1, curr_it), next_it_i,
					 path[next_it_i].x, path[next_it_i].y, next_it_i, path_tmp[next_it_i].x, path_tmp[next_it_i].y);
			if (isXAxis(get_dir(curr_it - 1, curr_it)))
				path[next_it_i].x = curr_it->x;
			else
				path[next_it_i].y = curr_it->y;
		}
		displayCellPath(path_tmp);
	}
}

bool APathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell) {

	ROS_WARN("%s,%d",__FUNCTION__, __LINE__);
	auto expand_condition = [&](const Cell_t &cell, const Cell_t &neighbor_cell)
	{
		return map.isBlockAccessible(neighbor_cell.x, neighbor_cell.y) /*&&
				map.getCost(neighbor_cell.x, neighbor_cell.y) == CLEANED*/;
	};

	Cells cells{};
	return !dijkstra(map, getPosition().toCell(), cells, true, CellEqual(Cell_t{0,0}), isAccessible(&map, expand_condition));
}

bool TargetVal::operator()(const Cell_t &c_it) {
		return p_map_->getCost(c_it.x, c_it.y) == val_;
}

isAccessible::isAccessible(GridMap *p_map, func_compare_two_t external_condition,
						   const BoundingBox2& bound)
: bound_(bound), p_map_(p_map), external_condition_(external_condition) {
		if (bound_.GetMinimum() == Cell_t{INT16_MAX, INT16_MAX} && bound_.GetMaximum() == Cell_t{INT16_MIN, INT16_MIN})
			bound_ = p_map_->genTargetRange();
		if (external_condition_ == nullptr)
			external_condition_ = [](const Cell_t &next, const Cell_t &neighbor) { return true; };
//		ROS_INFO("target_bound_(%d,%d,%d,%d)",target_bound_.min.x,target_bound_.min.y,target_bound_.max.y,target_bound_.max.y );
	}

bool isAccessible::operator()(const Cell_t &next, const Cell_t &neighbor) {
		return bound_.Contains(neighbor) && !p_map_->cellIsOutOfTargetRange(neighbor) &&
			   external_condition_(next, neighbor);
}

bool IsTarget::operator()(const Cell_t &c_it) {
		return c_it.y % 2 == 0 && p_map_->getCost(c_it.x, c_it.y) == UNCLEAN &&
			   target_bound_.Contains(c_it);
}


GridMap* GoHomeWay_t::updateMap(GridMap &map,std::unique_ptr<GridMap>& p_temp_map, const Point_t &curr) {
	if(is_allow_update_map_) {
		p_temp_map->copy(map);
		p_temp_map->merge(slam_grid_map, false, false, true, false, false, false);
		ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
		p_temp_map->print(curr.toCell(), Cells{{0, 0}});
		ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
		return p_temp_map.get();
	}else
		return &map;
}

void GoHomeWay_t::clearBlock(GridMap &map) {
		map.merge(slam_grid_map, false, false, false, false, false, true);
}

bool GoHomeWay_t::displayName() {
	ROS_INFO("go home way %s",name_.c_str());
}


bool ThroughAccessibleAndCleaned::operator()(const Cell_t &next, const Cell_t &neighbor) {
	return p_map_->isBlockAccessible(neighbor.x, neighbor.y) && p_map_->getCost(neighbor.x, neighbor.y) == CLEANED;
}
bool ThroughBlockAccessible::operator()(const Cell_t &next, const Cell_t &neighbor) {
	return p_map_->isBlockAccessible(neighbor.x, neighbor.y);
}
