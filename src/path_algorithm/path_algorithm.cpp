//
// Created by austin on 17-12-3.
//

#include <pp.h>
#include "ros/ros.h"
#include "path_algorithm.h"

void APathAlgorithm::displayCellPath(const Cells &path)
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": Path size(" + std::to_string(path.size()) + "):";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ")->";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void APathAlgorithm::displayTargetList(const Cells &target_list)
{
	std::string     msg = __FUNCTION__;
	msg += " " + std::to_string(__LINE__) + ": Targers(" + std::to_string(target_list.size()) + "):";
	for (auto it = target_list.begin(); it != target_list.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ", " + "),";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void APathAlgorithm::displayPointPath(const Points &point_path)
{
	std::string     msg = __FUNCTION__;
	msg += " " + std::to_string(__LINE__) + ": Targers(" + std::to_string(point_path.size()) + "):";
	for (auto it = point_path.begin(); it != point_path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ", " + std::to_string(it->TH) + "),";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void APathAlgorithm::optimizePath(GridMap &map, Cells& path)
{
	// Optimize only if the path have more than 3 cells.
	if (path.size() > 3) {
		ROS_INFO("%s %d: Start optimizing Path",__FUNCTION__,__LINE__);
		auto it = path.begin();
		for (uint16_t i = 0; i < path.size() - 3; i++) {
			ROS_DEBUG("%s %d: i: %d, size: %ld.", __FUNCTION__, __LINE__, i, path.size());
			auto it_ptr1 = it;

			auto it_ptr2 = it_ptr1;
			it_ptr2++;

			auto it_ptr3 = it_ptr2;
			it_ptr3++;

			// Just for protection, in case the iterator overflow. It should not happen under (i < path_points.size() - 3).
			if (it_ptr3 == path.end())
				break;

			if (it_ptr2->X == it_ptr3->X) {		// X coordinates are the same for p1, p2, find a better Y coordinate.
				int16_t x_min, x_max;
				x_min = x_max = it_ptr2->X;

				int16_t	ei, ej, si, sj;
				sj = it_ptr1->X > it_ptr2->X ? it_ptr2->X : it_ptr1->X;
				ej = it_ptr1->X > it_ptr2->X ? it_ptr1->X : it_ptr2->X;
				si = it_ptr2->Y > it_ptr3->Y ? it_ptr3->Y : it_ptr2->Y;
				ei = it_ptr2->Y > it_ptr3->Y ? it_ptr2->Y : it_ptr3->Y;

				bool blocked_min, blocked_max;
				blocked_min = blocked_max = false;
				while (!blocked_min || !blocked_max) {
					for (int16_t j = si; j <= ei && (!blocked_min || !blocked_max); j++) {
						if (!blocked_min && (x_min - 1 < sj ||
								map.getCell(COST_MAP, x_min - 1, j) == COST_HIGH)) {
							blocked_min = true;
						}
						if (!blocked_max && (x_max + 1 > ej ||
								map.getCell(COST_MAP, x_max + 1, j) == COST_HIGH)) {
							blocked_max = true;
						}
					}
					if (!blocked_min) {
						x_min--;
					}
					if (!blocked_max) {
						x_max++;
					}
				}

				if (it_ptr3->X != (x_min + x_max) / 2) {
					it_ptr2->X = it_ptr3->X = (x_min + x_max) / 2;
				}
				//ROS_INFO("%s %d: Loop i:%d\tx_min_forward: %d\tx_max_forward: %d\tget x:%d", __FUNCTION__, __LINE__, i, x_min_forward, x_max_forward, (x_min_forward + x_max_forward) / 2);
			} else {
				int16_t y_min, y_max;
				y_min = y_max = it_ptr2->Y;

				int16_t	ei, ej, si, sj;
				sj = it_ptr1->Y > it_ptr2->Y ? it_ptr2->Y : it_ptr1->Y;
				ej = it_ptr1->Y > it_ptr2->Y ? it_ptr1->Y : it_ptr2->Y;
				si = it_ptr2->X > it_ptr3->X ? it_ptr3->X : it_ptr2->X;
				ei = it_ptr2->X > it_ptr3->X ? it_ptr2->X : it_ptr3->X;

				bool blocked_min, blocked_max;
				blocked_min = blocked_max = false;
				while (!blocked_min || !blocked_max) {
					for (int16_t j = si; j <= ei && (!blocked_min || !blocked_max); j++) {
						if (!blocked_min && (y_min - 1 < sj ||
								map.getCell(COST_MAP, j, y_min - 1) == COST_HIGH)) {
							blocked_min = true;
						}
						if (!blocked_max && (y_max + 1 > ej ||
								map.getCell(COST_MAP, j, y_max + 1) == COST_HIGH)) {
							blocked_max = true;
						}
					}
					if (!blocked_min) {
						y_min--;
					}
					if (!blocked_max) {
						y_max++;
					}
				}

				if (it_ptr3->Y != (y_min + y_max) / 2) {
					it_ptr2->Y = it_ptr3->Y = (y_min + y_max) / 2;
				}
				//ROS_INFO("%s %d: Loop i:%d\ty_min: %d\ty_max: %d\tget y:%d", __FUNCTION__, __LINE__, i, y_min, y_max, (y_min + y_max) / 2);
			}

			it++;
		}
		displayCellPath(path);
	} else
		ROS_INFO("%s %d:Path too short(size: %ld), optimization terminated.", __FUNCTION__, __LINE__, path.size());

}
Points APathAlgorithm::cells_generate_points(Cells &path)
{
//	displayCellPath(path);
	Points targets{};
	for(auto it = path.begin(); it < path.end(); ++it) {
		Point32_t target {cellToCount((*it).X),cellToCount((*it).Y),0};
		auto it_next = it+1;
		if (it->X == it_next->X)
			target.TH = it->Y > it_next->Y ? MAP_NEG_Y : MAP_POS_Y;
		else
			target.TH = it->X > it_next->X ? MAP_NEG_X : MAP_POS_X;
		targets.push_back(target);
	}
//		ROS_INFO("path.back(%d,%d,%d)",path.back().X, path.back().Y, path.back().TH);

	targets.back().TH = (targets.end()-2)->TH;
//	ROS_INFO("%s %d: path.back(%d,%d,%d), path.front(%d,%d,%d)", __FUNCTION__, __LINE__,
//					 path.back().X, path.back().Y, path.back().TH, path.front().X, path.front().Y, path.front().TH);
	return targets;
}

bool APathAlgorithm::generateShortestPath(GridMap &map, const Point32_t &curr,const Point32_t &target, const MapDirection &last_dir, Points &plan_path) {
	auto path_cell = findShortestPath(map, curr.toCell(), target.toCell(),last_dir, false);

	plan_path = cells_generate_points(path_cell);
}

Cells APathAlgorithm::findShortestPath(GridMap &map, const Cell_t &start, const Cell_t &target,
										const MapDirection &last_dir, bool use_unknown)
{
	Cells path_{};

	// Get the map range.
	int16_t x_min, x_max, y_min, y_max;
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

	// Set for target cell. For reverse algorithm, we will generate a-star map from target cell.
	map.setCell(COST_MAP, target.X, target.Y, COST_1);

	// For protection, the start cell must be reachable.
	if (map.getCell(COST_MAP, start.X, start.Y) == COST_HIGH)
	{
		ROS_ERROR("%s %d: Start cell has high cost(%d)! It may cause bug, please check.",
							__FUNCTION__, __LINE__, map.getCell(COST_MAP, start.X, start.Y));
		map.print(COST_MAP, target.X, target.Y);
		map.setCell(COST_MAP, start.X, start.Y, COST_NO);
	}

	/*
	 * Find the path to target from the start cell. Set the cell values
	 * in COST_MAP to 1, 2, 3, 4 or 5. This is a method like A-Star, starting
	 * from a start point, update the cells one level away, until we reach the target.
	 */
	int16_t offset = 0;
	bool cost_updated = true;
	int16_t cost_value = 1;
	int16_t next_cost_value = 2;
	while (map.getCell(COST_MAP, start.X, start.Y) == COST_NO && cost_updated) {
		offset++;
		cost_updated = false;

		/*
		 * The following 2 for loops is for optimise the computational time.
		 * Since there is not need to go through the whole COST_MAP for searching the
		 * cell that have the next pass value.
		 *
		 * It can use the offset to limit the range of searching, since in each loop
		 * the cell (X -/+ offset, Y -/+ offset) would be set only. The cells far away
		 * to the robot position won'trace_cell be set.
		 */
		for (auto i = target.X - offset; i <= target.X + offset; i++) {
			if (i < x_min || i > x_max)
				continue;

			for (auto j = target.Y - offset; j <= target.Y + offset; j++) {
				if (j < y_min || j > y_max)
					continue;

				/* Found a cell that has a pass value equal to the current pass value. */
				if(map.getCell(COST_MAP, i, j) == cost_value) {
					/* Set the lower cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i - 1, j) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) (i - 1), (int32_t) j, (CellState) next_cost_value);
						cost_updated = true;
					}

					/* Set the upper cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i + 1, j) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) (i + 1), (int32_t) j, (CellState) next_cost_value);
						cost_updated = true;
					}

					/* Set the cell on the right hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i, j - 1) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) i, (int32_t) (j - 1), (CellState) next_cost_value);
						cost_updated = true;
					}

					/* Set the cell on the left hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i, j + 1) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) i, (int32_t) (j + 1), (CellState) next_cost_value);
						cost_updated = true;
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
	CellState start_cell_state = map.getCell(COST_MAP, start.X, start.Y);
	if (start_cell_state == COST_NO || start_cell_state == COST_HIGH) {
		ROS_WARN("%s, %d: Target (%d, %d) is not reachable for start cell(%d, %d)(%d), return empty path.",
						 __FUNCTION__, __LINE__, target.X, target.Y, start.X, start.Y, start_cell_state);
#if	DEBUG_COST_MAP
		map.print(COST_MAP, target.X, target.Y);
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
	trace_cell.X = trace_x = trace_x_last = start.X;
	trace_cell.Y = trace_y = trace_y_last = start.Y;
	path_.push_back(trace_cell);

	uint16_t next = 0;
	auto trace_dir = (last_dir == MAP_POS_Y || last_dir == MAP_NEG_Y) ? 1: 0;
	//ROS_INFO("%s %d: trace dir: %d", __FUNCTION__, __LINE__, trace_dir);
	while (trace_x != target.X || trace_y != target.Y) {
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
		if (path_.back().X != trace_x && path_.back().Y != trace_y) {
			// Only turning cell will be pushed to path.
			trace_cell.X = trace_x_last;
			trace_cell.Y = trace_y_last;
			path_.push_back(trace_cell);
		}
		trace_x_last = trace_x;
		trace_y_last = trace_y;
	}

	map.setCell(COST_MAP, (int32_t) target.X, (int32_t) target.Y, COST_PATH);

	trace_cell.X = target.X;
	trace_cell.Y = target.Y;
	path_.push_back(trace_cell);

	displayCellPath(path_);

	return path_;
}

bool APathAlgorithm::findTargetUsingDijkstra(GridMap &map, const Cell_t& curr_cell, Cell_t& target, int& cleaned_count)
{
	typedef std::multimap<double, Cell_t> Queue;
	typedef std::pair<double, Cell_t> Entry;

	map.reset(COST_MAP);
	map.setCell(COST_MAP, curr_cell.X, curr_cell.Y, COST_1);

	Queue queue;
	Entry startPoint(0.0, curr_cell);
	cleaned_count = 1;
	queue.insert(startPoint);
	bool is_found = false;
//	map.print(CLEAN_MAP,curr.X, curr.Y);
	ROS_INFO("Do full search with weightless Dijkstra-Algorithm\n");
	while (!queue.empty())
	{
//		 Get the nearest next from the queue
		auto start = queue.begin();
		auto next = start->second;
		queue.erase(start);

//		ROS_WARN("adjacent cell(%d,%d)", next.X, next.Y);
		if (map.getCell(CLEAN_MAP, next.X, next.Y) == UNCLEAN && map.isBlockAccessible(next.X, next.Y))
		{
			ROS_WARN("We find the Unclean next(%d,%d)", next.X, next.Y);
			is_found = true;
			target = next;
			break;
		} else
		{
			for (auto it1 = 0; it1 < 4; it1++)
			{
				auto neighbor = next + cell_direction_index_[it1];
//				ROS_INFO("g_index[%d],next(%d,%d)", it1, neighbor.X,neighbor.Y);
				if (map.getCell(COST_MAP, neighbor.X, neighbor.Y) != COST_1) {
//					ROS_INFO("(%d,%d),", neighbor.X, neighbor.Y);

					for (auto it2 = 0; it2 < 9; it2++) {
						auto neighbor_ = neighbor + cell_direction_index_[it2];
						if (map.getCell(CLEAN_MAP, neighbor_.X, neighbor_.Y) == CLEANED &&
								map.getCell(COST_MAP, neighbor_.X, neighbor_.Y) == COST_NO)
						{
							cleaned_count++;
							map.setCell(COST_MAP, neighbor_.X, neighbor_.Y, COST_2);
//							ROS_INFO("(%d,%d, cleaned_count(%d)),", neighbor_.X, neighbor_.Y, cleaned_count);
						}
					}

					if (map.isBlockAccessible(neighbor.X, neighbor.Y)) {
//						ROS_WARN("add to Queue:(%d,%d)", neighbor.X, neighbor.Y);
						queue.insert(Entry(0, neighbor));
						map.setCell(COST_MAP, neighbor.X, neighbor.Y, COST_1);
					}
				}
			}
		}
	}
//	cleaned_count = roscost_map.get_cleaned_area(curr);
	return is_found;
}

bool APathAlgorithm::checkTrappedUsingDijkstra(GridMap &map, const Cell_t &curr_cell)
{
	int dijkstra_cleaned_count = 0;
	Cells path{{0,0}};
	Cell_t target;
	// Check if there is any reachable target.
	bool is_found = findTargetUsingDijkstra(map, curr_cell, target, dijkstra_cleaned_count);
	if(is_found)
		return false;

	// Use clean area proportion to judge if it is trapped.
	auto map_cleand_count = map.getCleanedArea();
	double clean_proportion = static_cast<double>(dijkstra_cleaned_count) / static_cast<double>(map_cleand_count);
	ROS_WARN("%s %d: dijkstra_cleaned_count(%d), map_cleand_count(%d), clean_proportion(%f) ,when prop < 0,8 is trapped",
					 __FUNCTION__, __LINE__, dijkstra_cleaned_count, map_cleand_count, clean_proportion);
	return (clean_proportion < 0.8);
}
