//
// Created by austin on 17-12-3.
//

#include "ros/ros.h"
#include "robot.hpp"
#include "path_algorithm.h"
ACleanMode* APathAlgorithm::p_cm_ = nullptr;

const Cell_t cell_direction_[9]{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1},{0,0}};
void APathAlgorithm::displayCellPath(const Cells &path)
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": Path size(" + std::to_string(path.size()) + "):";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "{" + std::to_string(it->x) + ", " + std::to_string(it->y) + "},";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void APathAlgorithm::displayTargetList(const Cells &target_list)
{
	std::string     msg = __FUNCTION__;
	msg += " " + std::to_string(__LINE__) + ": targets = {" + std::to_string(target_list.size()) + "}:";
	for (auto it = target_list.begin(); it != target_list.end(); ++it) {
		msg += "{" + std::to_string(it->x) + ", " + std::to_string(it->y) + ", " + "},";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void APathAlgorithm::displayPointPath(const Points &point_path)
{
	std::string     msg = __FUNCTION__;
	msg += " " + std::to_string(__LINE__) + ": Targers(" + std::to_string(point_path.size()) + "):";
	for (auto it = point_path.begin(); it != point_path.end(); ++it) {
		msg += "(" + std::to_string((it->toCell().x)) + ", " + std::to_string(it->toCell().y) + ", " + std::to_string(
						static_cast<int>(radian_to_degree(it->th))) + std::to_string(it->dir) + "),";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

Points APathAlgorithm::cells_generate_points(Cells &path)
{
//	displayCellPath(path);
	Points point_path{};
	if(!path.empty()){
		for(auto it = path.begin(); it < path.end(); ++it) {
			Point_t target {cellToCount((*it).x),cellToCount((*it).y),0};
			auto it_next = it+1;
			if (it->x == it_next->x)
			{
				target.dir = it->y > it_next->y ? MAP_NEG_Y : MAP_POS_Y;
				target.th = isPos(target.dir) ? PI/2 : -PI/2;
			}
			else
			{
				target.dir = it->x > it_next->x ? MAP_NEG_X : MAP_POS_X;
				target.th = isPos(target.dir) ? 0 : PI;
			}
			point_path.push_back(target);
		}
	//		ROS_INFO("path.back(%d,%d,%d)",path.back().n, path.back().y, path.back().TH);

		point_path.back().dir = (point_path.end()-2)->dir;
		point_path.back().th = (point_path.end()-2)->th;
	}
	return point_path;
}

bool APathAlgorithm::generateShortestPath(GridMap &map, const Point_t &curr,const Point_t &target, const Dir_t &last_dir, Points &plan_path) {
	Cell_t corner1 ,corner2;
	auto path_cell = findShortestPath(map, curr.toCell(), target.toCell(),last_dir, false,false,corner1,corner2);

	plan_path = cells_generate_points(path_cell);
}

Cells APathAlgorithm::findShortestPath(GridMap &map, const Cell_t &start, const Cell_t &target,
										const Dir_t &last_dir, bool use_unknown,bool bound ,Cell_t min_corner,Cell_t max_corner)
{
	Cells path_{};
	// limit cost map range or get the total map range.
	int16_t x_min, x_max, y_min, y_max;
	if(bound){
		x_min = min_corner.x;
		y_min = min_corner.y;	
		x_max = max_corner.x;
		y_max = max_corner.y;
	}
	else
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
			else if(cs == UNCLEAN && !use_unknown)
				map.setCell(COST_MAP, i, j, COST_HIGH);
		}
	}

	// For protection, the target cell must be reachable.
	if (map.getCell(COST_MAP, target.x, target.y) == COST_HIGH)
	{
		ROS_ERROR("%s %d: Target cell has high cost(%d)! This target should be filtered before calling this function.",
				  __FUNCTION__, __LINE__, map.getCell(COST_MAP, target.x, target.y));
		return path_;
	}

	// Set for target cell. For reverse algorithm, we will generate a-star map from target cell.
	map.setCell(COST_MAP, target.x, target.y, COST_1);

	// For protection, the start cell must be reachable.
	if (map.getCell(COST_MAP, start.x, start.y) == COST_HIGH)
	{
		ROS_ERROR("%s %d: Start cell has high cost(%d)! It may cause bug, please check.",
							__FUNCTION__, __LINE__, map.getCell(COST_MAP, start.x, start.y));
		map.print(getPosition().toCell(), COST_MAP, Cells{target});
		map.setCell(COST_MAP, start.x, start.y, COST_NO);
	}

	/*
	 * Find the path to target from the start cell. Set the cell values
	 * in COST_MAP to 1, 2, 3, 4 or 5. This is a method like A-Star, starting
	 * from a start point, init the cells one level away, until we reach the target.
	 */
	int16_t offset = 0;
	bool cost_updated = true;
	int16_t cost_value = 1;
	int16_t next_cost_value = 2;
	while (map.getCell(COST_MAP, start.x, start.y) == COST_NO && cost_updated) {
		offset++;
		cost_updated = false;

		/*
		 * The following 2 for loops is for optimise the computational time.
		 * Since there is not need to go through the whole COST_MAP for searching the
		 * cell that have the next pass value.
		 *
		 * It can use the offset to limit the range of searching, since in each loop
		 * the cell (x -/+ offset, y -/+ offset) would be set only. The cells far away
		 * to the robot position won'trace_cell be set.
		 */
		for (auto i = target.x - offset; i <= target.x + offset; i++) {
			if (i < x_min || i > x_max)
				continue;

			for (auto j = target.y - offset; j <= target.y + offset; j++) {
				if (j < y_min || j > y_max)
					continue;

				/* Found a cell that has a pass value equal to the current pass value. */
				if(map.getCell(COST_MAP, i, j) == cost_value) {
					/* Set the lower cell of the cell which has the pass value equal to current pass value. */
					for(auto index = 0;index<4; index++) {
						auto neighbor = Cell_t(i, j) - cell_direction_[index];
						if (map.getCell(COST_MAP, neighbor.x, neighbor.y) == COST_NO) {
							map.setCell(COST_MAP, neighbor.x, neighbor.y, (CellState) next_cost_value);
							cost_updated = true;
						}
					}
				}
			}
		}

		/* Update the pass value. */
		cost_value = next_cost_value;
		next_cost_value++;

		/* Reset the pass value, pass value can only between 1 to 5. */
		if(next_cost_value == COST_PATH)
			next_cost_value = 1;
	}

	// If the start cell still have a cost of 0, it means target is not reachable.
	CellState start_cell_state = map.getCell(COST_MAP, start.x, start.y);
	if (start_cell_state == COST_NO || start_cell_state == COST_HIGH) {
		ROS_WARN("%s, %d: Target (%d, %d) is not reachable for start cell(%d, %d)(%d), return empty path.",
						 __FUNCTION__, __LINE__, target.x, target.y, start.x, start.y, start_cell_state);
#if	DEBUG_COST_MAP
		map.print(getPosition().toCell(),COST_MAP, Cells{target});
#endif
		// Now the path_ is empty.
		return path_;
	}

	/*
	 * Start from the start position, trace back the path by the cost level.
	 * Value of cells on the path is set to 6. Stops when reach the target
	 * position.
	 *
	 * The last robot direction is use, this is to avoid using the path that
	 * have the same direction as previous action.
	 */
	Cell_t trace_cell;
	int16_t trace_x, trace_y, trace_x_last, trace_y_last, total_cost = 0;
	trace_cell.x = trace_x = trace_x_last = start.x;
	trace_cell.y = trace_y = trace_y_last = start.y;
	path_.push_back(trace_cell);

	uint16_t next = 0;
	auto trace_dir = (last_dir == MAP_POS_Y || last_dir == MAP_NEG_Y) ? 1: 0;
	//ROS_INFO("%s %d: trace dir: %d", __FUNCTION__, __LINE__, trace_dir);
	while (trace_x != target.x || trace_y != target.y) {
		CellState cost_at_cell = map.getCell(COST_MAP, trace_x, trace_y);
		auto target_cost = static_cast<CellState>(cost_at_cell - 1);

		/* Reset target cost to 5, since cost only set from 1 to 5 in the COST_MAP. */
		if (target_cost == 0)
			target_cost = COST_5;

		/* Set the cell value to 6 if the cells is on the path. */
		map.setCell(COST_MAP, (int32_t) trace_x, (int32_t) trace_y, COST_PATH);

#define COST_SOUTH	{											\
				if (next == 0 && (map.getCell(COST_MAP, trace_x - 1, trace_y) == target_cost)) {	\
					trace_x--;								\
					next = 1;								\
					trace_dir = 1;								\
				}										\
			}

#define COST_WEST	{											\
				if (next == 0 && (map.getCell(COST_MAP, trace_x, trace_y - 1) == target_cost)) {	\
					trace_y--;								\
					next = 1;								\
					trace_dir = 0;								\
				}										\
			}

#define COST_EAST	{											\
				if (next == 0 && (map.getCell(COST_MAP, trace_x, trace_y + 1) == target_cost)) {	\
					trace_y++;								\
					next = 1;								\
					trace_dir = 0;								\
				}										\
			}

#define COST_NORTH	{											\
				if (next == 0 && map.getCell(COST_MAP, trace_x + 1, trace_y) == target_cost) {	\
					trace_x++;								\
					next = 1;								\
					trace_dir = 1;								\
				}										\
			}

		next = 0;
		if (trace_dir == 0) {
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

		total_cost++;
		if (path_.back().x != trace_x && path_.back().y != trace_y) {
			// Only turning cell will be pushed to path.
			trace_cell.x = trace_x_last;
			trace_cell.y = trace_y_last;
			path_.push_back(trace_cell);
		}
		trace_x_last = trace_x;
		trace_y_last = trace_y;
	}

	map.setCell(COST_MAP, (int32_t) target.x, (int32_t) target.y, COST_PATH);

	trace_cell = target;
	path_.push_back(trace_cell);

	displayCellPath(path_);

	return path_;
}

bool APathAlgorithm::checkTrappedUsingDijkstra(GridMap &map, const Cell_t &curr_cell)
{
	Cells targets;
	// Use clean area proportion to judge if it is trapped.
    int dijkstra_cleaned_count{};
	auto is_trapped = map.count_if(curr_cell,[&](Cell_t c_it) {
		return (map.getCell(CLEAN_MAP, c_it.x, c_it.y) == CLEANED);
	},dijkstra_cleaned_count);
    if(!is_trapped)
		return false;

	auto map_cleand_count = map.getCleanedArea();
	double clean_proportion = static_cast<double>(dijkstra_cleaned_count) / static_cast<double>(map_cleand_count);
	ROS_ERROR("%s %d: !!!!!!!!!!!!!!!!!!!!!!!dijkstra_cleaned_count(%d), map_cleand_count(%d), clean_proportion(%f) ,when prop < 0,8 is trapped",
					 __FUNCTION__, __LINE__, dijkstra_cleaned_count, map_cleand_count, clean_proportion);
	return (clean_proportion < 0.8);
}

bool APathAlgorithm::isTargetReachable(GridMap map,Cell_t target)
{
	for (int16_t i = target.x - 1; i <= target.x + 1; ++i) {
		for (int16_t j = target.y- 1; j <= target.y + 1; ++j) {
			CellState cs = map.getCell(CLEAN_MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				return false;
			}
		}
	}
	return true;

}
