//
// Created by lsy563193 on 12/13/17.
//

#include <pp.h>
#include "ros/ros.h"
#include "path_algorithm.h"

bool NavCleanPathAlgorithm::generatePath(GridMap &map, const Point32_t &curr, const MapDirection &last_dir, Points &plan_path)
{

	plan_path.clear();
	auto curr_cell = curr.toCell();
	//Step 1: Find possible plan_path in same lane.
	auto plan_path_cell = findTargetInSameLane(map, curr_cell);
	if (!plan_path_cell.empty())
	{
		plan_path = cells_generate_points(plan_path_cell);
		// Congratulation!! plan_path is generated successfully!!
		return true;
	}

	//Step 2: Find all possible plan_path at the edge of cleaned area and filter plan_path in same lane.

	// Copy map to a BoundingBox2 type b_map.
	BoundingBox2 b_map;
	auto b_map_temp = map.generateBound();

	for (const auto &cell : b_map_temp) {
		if (map.getCell(CLEAN_MAP, cell.X, cell.Y) != UNCLEAN)
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
	Cells shortest_path = findShortestPath(map, curr_cell, best_target, last_dir, true);
	if (shortest_path.empty())
		// Now plan_path is empty.
		return false;

	//Step 7: Optimize path for adjusting it away from obstacles..
	optimizePath(map, shortest_path);

	//Step 8: Fill path with direction.
	plan_path = cells_generate_points(shortest_path);

	// Congratulation!! plan_path is generated successfully!!
//	plan_path_cell = shortest_path;

	return true;
}

Cells NavCleanPathAlgorithm::findTargetInSameLane(GridMap &map, const Cell_t &curr_cell)
{
	int8_t is_found = 0;
	Cell_t it[2]; // it[0] means the furthest cell of X positive direction, it[1] means the furthest cell of X negative direction.

//	map.print(CLEAN_MAP, 0, 0);
	for (auto i = 0; i < 2; i++) {
		it[i] = curr_cell;
		auto unclean_cells = 0;
		for (Cell_t neighbor = it[i] + cell_direction_index_[i];
				 !map.cellIsOutOfRange(neighbor + cell_direction_index_[i]) && !map.isBlocksAtY(neighbor.X, neighbor.Y);
				 neighbor += cell_direction_index_[i])
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
//		ROS_WARN("%s %d: nag dir(%d)", __FUNCTION__, __LINE__, (Movement::cm_target_p_.Y<Movement::cm_origin_p_.Y));
//		if(mt.is_follow_wall() && cm_is_reach())
//		{
//			if(mt.is_left() ^ (Movement::cm_target_p_.Y<Movement::cm_origin_p_.Y))
//				target = it[1];
//		}
	}

	Cells path{};
	if (is_found)
	{
		path.push_front(target);
		path.push_front(getCurrCell());
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target:(%d,%d)", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y, target.X, target.Y);
//		map.print(CLEAN_MAP, target.X, target.Y);
	}
	else
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target not found.", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y);

	return path;
}

Cells NavCleanPathAlgorithm::filterAllPossibleTargets(GridMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map)
{
	Cells possible_target_list{};

	auto b_map_copy = b_map;
	b_map_copy.pos_ = b_map.min;

	// Check all boundarys between cleaned cells and unclean cells.
	for (const auto &cell : b_map_copy) {
		if (map.getCell(CLEAN_MAP, cell.X, cell.Y) != CLEANED /*|| std::abs(cell.Y % 2) == 1*/)
			continue;

		Cell_t neighbor;
		for (auto i = 0; i < 4; i++) {
			neighbor = cell + cell_direction_index_[i];
			if (map.getCell(CLEAN_MAP, neighbor.X, neighbor.Y) == UNCLEAN && map.isBlockAccessible(neighbor.X, neighbor.Y))
				possible_target_list.push_back(neighbor);
		}
	}

	std::sort(possible_target_list.begin(),possible_target_list.end(),[](Cell_t l,Cell_t r){
		return (l.Y < r.Y || (l.Y == r.Y && l.X < r.X));
	});

	displayTargetList(possible_target_list);

	ROS_INFO("%s %d: Filter targets in the same line.", __FUNCTION__, __LINE__);
	Cells filtered_targets{};
	/* Filter the targets. */
	for(;!possible_target_list.empty();) {
		auto y = possible_target_list.front().Y;
		Cells tmp_list{};
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
		CellState it_cost = map.getCell(COST_MAP, it->X, it->Y);
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
			trace_cost = map.getCell(COST_MAP, trace.X, trace.Y) - 1;

			if (trace_cost == 0) {
				trace_cost = COST_5;
			}

			path.push_front(trace);

			if ((trace.X - 1 >= x_min) && (map.getCell(COST_MAP, trace.X - 1, trace.Y) == trace_cost)) {
				trace.X--;
				continue;
			}

			if ((trace.X + 1 <= x_max) && (map.getCell(COST_MAP, trace.X + 1, trace.Y) == trace_cost)) {
				trace.X++;
				continue;
			}

			if ((trace.Y - 1 >= y_min) && (map.getCell(COST_MAP, trace.X, trace.Y - 1) == trace_cost)) {
				trace.Y--;
				continue;
			}

			if ((trace.Y + 1 <= y_max) && (map.getCell(COST_MAP, trace.X, trace.Y + 1) == trace_cost)) {
				trace.Y++;
				continue;
			}
		}
		path.push_front(trace);

		paths.push_back(path);
	}

	return paths;
}

bool NavCleanPathAlgorithm::filterPathsToSelectTarget(GridMap &map, const PathList &paths, const Cell_t &curr_cell, Cell_t &best_target)
{
	bool match_target_found = false, is_found = false, within_range=false;
	PathList filtered_paths{};
	uint16_t final_cost = 1000;
	ROS_INFO("%s %d: case 1, towards Y+ only", __FUNCTION__, __LINE__);
	for (auto it = paths.begin(); it != paths.end(); ++it) {
		if (map.getCell(CLEAN_MAP, it->back().X, it->back().Y - 1) != CLEANED) {
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
		{
			break;
		}

		if (it->size() > final_cost)
			continue;

		y_max = it->back().Y;
		within_range = true;
		for (auto i = it->rbegin(); within_range && i != it->rend(); ++i) {
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

bool NavCleanPathAlgorithm::sortPathsWithTargetYAscend(const Cells a, const Cells b)
{
	return a.back().Y < b.back().Y;
}

bool NavCleanPathAlgorithm::checkTrapped(GridMap &map, const Cell_t &curr_cell)
{
	return checkTrappedUsingDijkstra(map, curr_cell);
}
