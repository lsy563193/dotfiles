//
// Created by austin on 17-12-3.
//

#include "ros/ros.h"
#include "path_algorithm/path_algorithm.h"

PathType PathAlgorithm::findShortestPath(CostMap &map, const Cell_t &start,
										 const Cell_t &target, const MapDirection &last_dir)
{
	PathType path_;
	path_.clear();

	// Get the map range.
	int16_t x_min, x_max, y_min, y_max;
	map.getMapRange(MAP, &x_min, &x_max, &y_min, &y_max);

	// Reset the SPMAP.
	map.reset(SPMAP);

	// Mark obstacles in SPMAP
	for (int16_t i = x_min - 1; i <= x_max + 1; ++i) {
		for (int16_t j = y_min - 1; j <= y_max + 1; ++j) {
			CellState cs = map.getCell(MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				//for (m = ROBOT_RIGHT_OFFSET + 1; m <= ROBOT_LEFT_OFFSET - 1; m++)
				for (int16_t m = ROBOT_RIGHT_OFFSET; m <= ROBOT_LEFT_OFFSET; m++) {
					for (int16_t n = ROBOT_RIGHT_OFFSET; n <= ROBOT_LEFT_OFFSET; n++) {
						map.setCell(SPMAP, (i + m), (j + n), COST_HIGH);
					}
				}
			}
			else if(cs == UNCLEAN)
				map.setCell(SPMAP, i, j, COST_HIGH);
		}
	}

	// Set for target cell. For reverse algorithm, we will generate a-star map from target cell.
	map.setCell(SPMAP, target.X, target.Y, COST_1);

	// For protection, the start cell must be reachable.
	if (map.getCell(MAP, start.X, start.Y) == COST_HIGH)
	{
		ROS_ERROR("%s %d: Start cell has high cost(%d)! It may cause bug, please check.",
				  __FUNCTION__, __LINE__, map.getCell(MAP, start.X, start.Y));
		map.print(SPMAP, target.X, target.Y);
		map.setCell(SPMAP, start.X, start.Y, COST_NO);
	}

	/*
	 * Find the path to target from the start cell. Set the cell values
	 * in SPMAP to 1, 2, 3, 4 or 5. This is a method like A-Star, starting
	 * from a start point, update the cells one level away, until we reach the target.
	 */
	int16_t offset = 0;
	bool cost_updated = true;
	int16_t cost_value = 1;
	int16_t next_cost_value = 2;
	while (map.getCell(SPMAP, start.X, start.Y) == COST_NO && cost_updated) {
		offset++;
		cost_updated = false;

		/*
		 * The following 2 for loops is for optimise the computational time.
		 * Since there is not need to go through the whole SPMAP for searching the
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
				if(map.getCell(SPMAP, i, j) == cost_value) {
					/* Set the lower cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(SPMAP, i - 1, j) == COST_NO) {
						map.setCell(SPMAP, (int32_t) (i - 1), (int32_t) j, (CellState) next_cost_value);
						cost_updated = true;
					}

					/* Set the upper cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(SPMAP, i + 1, j) == COST_NO) {
						map.setCell(SPMAP, (int32_t) (i + 1), (int32_t) j, (CellState) next_cost_value);
						cost_updated = true;
					}

					/* Set the cell on the right hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(SPMAP, i, j - 1) == COST_NO) {
						map.setCell(SPMAP, (int32_t) i, (int32_t) (j - 1), (CellState) next_cost_value);
						cost_updated = true;
					}

					/* Set the cell on the left hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(SPMAP, i, j + 1) == COST_NO) {
						map.setCell(SPMAP, (int32_t) i, (int32_t) (j + 1), (CellState) next_cost_value);
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
	CellState start_cell_state = map.getCell(SPMAP, start.X, start.Y);
	if (start_cell_state == COST_NO || start_cell_state == COST_HIGH) {
		ROS_WARN("%s, %d: Target (%d, %d) is not reachable for start cell(%d, %d)(%d), return empty path.",
				 __FUNCTION__, __LINE__, target.X, target.Y, start.X, start.Y, start_cell_state);
#if	DEBUG_SP_MAP
		map.print(SPMAP, target.X, target.Y);
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
	trace_cell.X = trace_x = trace_x_last = target.X;
	trace_cell.Y = trace_y = trace_y_last = target.Y;
	path_.push_back(trace_cell);

	uint16_t next = 0;
	auto trace_dir = (last_dir == MAP_POS_Y || last_dir == MAP_NEG_Y) ? 1: 0;
	//ROS_INFO("%s %d: trace dir: %d", __FUNCTION__, __LINE__, trace_dir);
	while (trace_x != target.X || trace_y != target.Y) {
		CellState cost_at_cell = map.getCell(SPMAP, trace_x, trace_y);
		auto target_cost = static_cast<CellState>(cost_at_cell - 1);

		/* Reset target cost to 5, since cost only set from 1 to 5 in the SPMAP. */
		if (target_cost == 0)
			target_cost = COST_5;

		/* Set the cell value to 6 if the cells is on the path. */
		map.setCell(SPMAP, (int32_t) trace_x, (int32_t) trace_y, COST_PATH);

#define COST_SOUTH	{											\
				if (next == 0 && (map.getCell(SPMAP, trace_x - 1, trace_y) == target_cost)) {	\
					trace_x--;								\
					next = 1;								\
					trace_dir = 1;								\
				}										\
			}

#define COST_WEST	{											\
				if (next == 0 && (map.getCell(SPMAP, trace_x, trace_y - 1) == target_cost)) {	\
					trace_y--;								\
					next = 1;								\
					trace_dir = 0;								\
				}										\
			}

#define COST_EAST	{											\
				if (next == 0 && (map.getCell(SPMAP, trace_x, trace_y + 1) == target_cost)) {	\
					trace_y++;								\
					next = 1;								\
					trace_dir = 0;								\
				}										\
			}

#define COST_NORTH	{											\
				if (next == 0 && map.getCell(SPMAP, trace_x + 1, trace_y) == target_cost) {	\
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

	map.setCell(SPMAP, (int32_t) target.X, (int32_t) target.Y, COST_PATH);

	trace_cell.X = target.X;
	trace_cell.Y = target.Y;
	path_.push_back(trace_cell);

	displayPath(path_);

	return path_;
}

void PathAlgorithm::displayPath(const PathType& path)
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": Path size(" + std::to_string(path.size()) + "):";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ", " + std::to_string(it->TH) + ")->";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void PathAlgorithm::optimizePath(CostMap &map, PathType& path)
{
	// Optimize only if the path have more than 3 cells.
	if (path.size() > 3) {
		ROS_INFO("%s %d: Start optimizing Path");
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
								map.getCell(SPMAP, x_min - 1, j) == COST_HIGH)) {
							blocked_min = true;
						}
						if (!blocked_max && (x_max + 1 > ej ||
								map.getCell(SPMAP, x_max + 1, j) == COST_HIGH)) {
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
				//ROS_INFO("%s %d: Loop i:%d\tx_min: %d\tx_max: %d\tget x:%d", __FUNCTION__, __LINE__, i, x_min, x_max, (x_min + x_max) / 2);
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
								map.getCell(SPMAP, j, y_min - 1) == COST_HIGH)) {
							blocked_min = true;
						}
						if (!blocked_max && (y_max + 1 > ej ||
								map.getCell(SPMAP, j, y_max + 1) == COST_HIGH)) {
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
		displayPath(path);
	} else
		ROS_INFO("%s %d:Path too short(size: %ld), optimization terminated.", __FUNCTION__, __LINE__, path.size());

}

void PathAlgorithm::fillPathWithDirection(PathType &path)
{
	for(auto it = path.begin(); it < path.end(); ++it) {
		auto it_next = it+1;
		if (it->X == it_next->X)
			it->TH = it->Y > it_next->Y ? MAP_NEG_Y : MAP_POS_Y;
		else
			it->TH = it->X > it_next->X ? MAP_NEG_X : MAP_POS_X;
	}
//		ROS_INFO("path.back(%d,%d,%d)",path.back().X, path.back().Y, path.back().TH);

	path.back().TH = (path.end()-2)->TH;
	ROS_INFO("%s %d: path.back(%d,%d,%d), path.front(%d,%d,%d)", __FUNCTION__, __LINE__,
			 path.back().X, path.back().Y, path.back().TH, path.front().X, path.front().Y, path.front().TH);
}

bool PathAlgorithm::sortPathsWithTargetYAscend(const PathType a, const PathType b)
{
	return a.back().Y < b.back().Y;
}

PathType NavCleanPathAlgorithm::generatePath(CostMap &map, const Cell_t &curr_cell, const MapDirection &last_dir)
{
	PathType path;
	path.clear();

	//Step 1: Find possible targets in same lane.
	path = findTargetInSameLane(map, curr_cell);
	if (!path.empty())
	{
		fillPathWithDirection(path);
		// Congratulation!! path is generated successfully!!
		return path;
	}

	//Step 2: Find all possible targets at the edge of cleaned area and filter targets in same lane.

	// Copy map to a BoundingBox2 type b_map.
	BoundingBox2 b_map;
	auto b_map_temp = map.generateBound();

	for (const auto &cell : b_map_temp) {
		if (map.getCell(MAP, cell.X, cell.Y) != UNCLEAN)
			b_map.Add(cell);
	}

	TargetList filtered_targets = filterAllPossibleTarget(map, curr_cell, b_map);

	//Step 3: Generate the SPMAP for map and filter targets that are unreachable.
	TargetList reachable_targets = getReachableTargets(map, curr_cell, filtered_targets);
	ROS_INFO("%s %d: After generating SPMAP, Get %lu reachable targets.", __FUNCTION__, __LINE__, reachable_targets.size());
	if (reachable_targets.size() != 0)
		displayTargets(reachable_targets);
	else
		// Now path is empty.
		return path;

	//Step 4: Trace back the path of these targets in SPMAP.
	PathList paths_for_reachable_targets = tracePathsToTargets(map, reachable_targets, curr_cell);

	//Step 5: Filter paths to get the best target.
	Cell_t best_target;
	if (!filterPathsToSelectTarget(map, paths_for_reachable_targets, curr_cell, best_target))
		// Now path is empty.
		return path;

	//Step 6: Find shortest path for this best target.
	PathType shortest_path = findShortestPath(map, curr_cell, best_target, last_dir);
	if (shortest_path.empty())
		// Now path is empty.
		return path;

	//Step 7: Optimize path for adjusting it away from obstacles..
	optimizePath(map, shortest_path);

	//Step 8: Fill path with direction.
	fillPathWithDirection(path);

	// Congratulation!! path is generated successfully!!
	path = shortest_path;

	return path;
}

PathType NavCleanPathAlgorithm::findTargetInSameLane(CostMap &map, const Cell_t &curr_cell)
{
	int8_t is_found = 0;
	Cell_t it[2]; // it[0] means the furthest cell of X positive direction, it[1] means the furthest cell of X negative direction.

//	map.print(MAP, 0, 0);
	for (auto i = 0; i < 2; i++) {
		it[i] = curr_cell;
		auto unclean_cells = 0;
		for (Cell_t neighbor = it[i] + cell_direction_index[i];
			 !map.cellIsOutOfRange(neighbor + cell_direction_index[i]) && !map.isBlocksAtY(neighbor.X, neighbor.Y);
			 neighbor += cell_direction_index[i])
		{
			unclean_cells += map.isUncleanAtY(neighbor.X, neighbor.Y);
			if (unclean_cells >= 3) {
				it[i] = neighbor;
				unclean_cells = 0;
//				ROS_INFO("%s %d: it[%d](%d,%d)", __FUNCTION__, __LINE__, i, it[i].X, it[i].Y);
			}
//			ROS_WARN("%s %d: it[%d](%d,%d)", __FUNCTION__, __LINE__, i, it[i].X, it[i].Y);
//			ROS_WARN("%s %d: nb(%d,%d)", __FUNCTION__, __LINE__, neighbor.X, neighbor.Y);
		}
	}

	Cell_t target = it[0];
	if (it[0].X != curr_cell.X)
	{
		is_found++;
	} else if (it[1].X != curr_cell.X)
	{
		target = it[1];
		is_found++;
	}
	if (is_found == 2)
	{
		// Select the nearest side.
		if (std::abs(curr_cell.X - it[0].X) < std::abs(curr_cell.X - it[0].X))
			target = it[0];

		//todo
//		ROS_WARN("%s %d: nag dir(%d)", __FUNCTION__, __LINE__, (Movement::s_target_p.Y<Movement::s_origin_p.Y));
//		if(mt.is_follow_wall() && cm_is_reach())
//		{
//			if(mt.is_left() ^ (Movement::s_target_p.Y<Movement::s_origin_p.Y))
//				target = it[1];
//		}
	}

	PathType path{};
	if (is_found)
	{
		path.push_front(target);
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target:(%d,%d)", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y, target.X, target.Y);
		map.print(MAP, target.X, target.Y);
	}
	else
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target not found.", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y);

	return path;
}

TargetList NavCleanPathAlgorithm::filterAllPossibleTarget(CostMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map)
{
	TargetList possible_target_list{};

	auto b_map_copy = b_map;
	b_map_copy.pos_ = b_map.min;

	// Check all boundarys between cleaned cells and unclean cells.
	for (const auto &cell : b_map_copy) {
		if (map.getCell(MAP, cell.X, cell.Y) != CLEANED /*|| std::abs(cell.Y % 2) == 1*/)
			continue;

		Cell_t neighbor;
		for (auto i = 0; i < 4; i++) {
			neighbor = cell + cell_direction_index[i];
			if (map.getCell(MAP, neighbor.X, neighbor.Y) == UNCLEAN && map.isCellAccessible(neighbor.X, neighbor.Y))
				possible_target_list.push_back(neighbor);
		}
	}

	std::sort(possible_target_list.begin(),possible_target_list.end(),[](Cell_t l,Cell_t r){
			return (l.Y < r.Y || (l.Y == r.Y && l.X < r.X));
	});

	displayTargets(possible_target_list);

	ROS_INFO("%s %d: Filter targets in the same line.", __FUNCTION__, __LINE__);
	TargetList filtered_targets{};
	/* Filter the targets. */
	for(;!possible_target_list.empty();) {
		auto y = possible_target_list.front().Y;
		TargetList tmp_list{};
		std::remove_if(possible_target_list.begin(), possible_target_list.end(), [&y, &tmp_list](Cell_t &it) {
			if (it.Y == y && (tmp_list.empty() || (it.X - tmp_list.back().X == 1))) {
				tmp_list.push_back(it);
				return true;
			}
			return false;
		});
		possible_target_list.resize(possible_target_list.size() - tmp_list.size());
		if (tmp_list.size() > 2) {
			tmp_list.erase(std::remove_if(tmp_list.begin() + 1, tmp_list.end() - 1, [&curr_cell](Cell_t &it) {
					return it.X != curr_cell.X;
			}), tmp_list.end() - 1);
		}
		for(const auto& it:tmp_list){
			filtered_targets.push_back(it);
		};
	}

	displayTargets(filtered_targets);

	return filtered_targets;
}

TargetList NavCleanPathAlgorithm::getReachableTargets(CostMap &map, const Cell_t &curr_cell, TargetList &possible_targets)
{
	map.generateSPMAP(curr_cell, possible_targets);
	TargetList reachable_targets{};
	for (auto it = possible_targets.begin(); it != possible_targets.end();)
	{
		CellState it_cost = map.getCell(SPMAP, it->X, it->Y);
		if (it_cost == COST_NO || it_cost == COST_HIGH)
			continue;
		else
		{
			reachable_targets.push_back(*it);
			it++;
		}
	}
	return reachable_targets;
}

PathList NavCleanPathAlgorithm::tracePathsToTargets(CostMap &map, const TargetList &target_list, const Cell_t& curr_cell)
{
	PathList paths{};
	int16_t trace_cost, x_min, x_max, y_min, y_max;
	map.getMapRange(SPMAP, &x_min, &x_max, &y_min, &y_max);
	for (auto& it : target_list) {
		auto trace = it;
		PathType path{};
		//Trace the path for this target 'it'.
		while (trace != curr_cell) {
			trace_cost = map.getCell(SPMAP, trace.X, trace.Y) - 1;

			if (trace_cost == 0) {
				trace_cost = COST_5;
			}

			path.push_front(trace);

			if ((trace.X - 1 >= x_min) && (map.getCell(SPMAP, trace.X - 1, trace.Y) == trace_cost)) {
				trace.X--;
				continue;
			}

			if ((trace.X + 1 <= x_max) && (map.getCell(SPMAP, trace.X + 1, trace.Y) == trace_cost)) {
				trace.X++;
				continue;
			}

			if ((trace.Y - 1 >= y_min) && (map.getCell(SPMAP, trace.X, trace.Y - 1) == trace_cost)) {
				trace.Y--;
				continue;
			}

			if ((trace.Y + 1 <= y_max) && (map.getCell(SPMAP, trace.X, trace.Y + 1) == trace_cost)) {
				trace.Y++;
				continue;
			}
		}
		path.push_front(trace);

		paths.push_back(path);
	}

	return paths;
}

bool NavCleanPathAlgorithm::filterPathsToSelectTarget(CostMap &map, const PathList &paths, const Cell_t &curr_cell, Cell_t &best_target)
{
	bool match_target_found = false, is_found = false, within_range=false;
	PathList filtered_paths{};
	uint16_t final_cost = 1000;
	ROS_INFO("%s %d: case 1, towards Y+ only", __FUNCTION__, __LINE__);
	for (auto it = paths.begin(); it != paths.end(); ++it) {
		if (map.getCell(MAP, it->back().X, it->back().Y - 1) != CLEANED) {
			// If the Y- cell of the target of this path is not cleaned, don't handle in this case.
			continue;
		}
		if (it->back().Y > curr_cell.Y + 1)
		{
			// Filter path with target in Y+ direction of current cell.
			filtered_paths.push_front(*it);
		}
	}
	// Sort paths with target Y ascend order.
	std::sort(filtered_paths.begin(), filtered_paths.end(), sortPathsWithTargetYAscend);

	int16_t y_max;
	for (auto it = filtered_paths.begin(); it != filtered_paths.end(); ++it)
	{
		if (match_target_found && best_target.Y < it->back().Y) // No need to find for further targets.
			break;

		if (it->size() > final_cost)
			continue;

		y_max = it->back().Y;
		within_range = true;
		for (auto i = it->begin(); within_range && i != it->end(); ++i) {
			if (i->Y < curr_cell.Y)
				// All turning cells should be in Y+ area, so quit it.
				within_range = false;
			if (i->Y > y_max)
				// Not allow path towards Y- direction.
				within_range = false;
			else
				y_max = i->Y;
		}
		if (within_range) {
			best_target = it->back();
			final_cost = static_cast<uint16_t>(it->size());
			match_target_found = true;
		}
	}

	if (!match_target_found) {
		ROS_INFO("%s %d: case 6, fallback to A-start the nearest target, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, match_target_found);
		for (auto it = paths.begin(); it != paths.end(); ++it) {
			if (it->size() < final_cost) {
				best_target = it->back();
				final_cost = static_cast<uint16_t>(it->size());
				match_target_found = true;
			}
		}
	}

	if (match_target_found)
		ROS_INFO("%s %d: Found best target(%d, %d)", __FUNCTION__, __LINE__, best_target.X, best_target.Y);
	else
		ROS_INFO("%s %d: No target selected.", __FUNCTION__, __LINE__);

	return match_target_found;
}
