//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <pp.h>
#include "arch.hpp"

CleanModeNav::CleanModeNav()
{
	register_events();
//	setMode(this);
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
	sp_action_.reset(new ActionOpenGyro());
	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	action_i_ = ac_open_gyro;
	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
//	sp_action_->registerMode(this);
//	ROS_INFO("%s %d:this(%d)", __FUNCTION__, __LINE__,this);
}

bool CleanModeNav::isFinish() {

	if (!Mode::isFinish())
		return false;

	if (action_i_ == ac_open_gyro) {
		if (charger.isOnStub()) {
			action_i_ = ac_back_form_charger;
			sp_action_.reset(new ActionBackFromCharger);
		}
		else {
			action_i_ = ac_open_lidar;
			sp_action_.reset(new ActionOpenLidar);
		}
	}
	else if (action_i_ == ac_back_form_charger) {
		action_i_ = ac_open_lidar;
		sp_action_.reset(new ActionOpenLidar);
	}
	else if (action_i_ == ac_open_lidar) {
		action_i_ = ac_align;
		sp_action_.reset(new ActionAlign);
	}
	else if (action_i_ == ac_align) {
		action_i_ = ac_open_slam;
		sp_action_.reset(new ActionOpenSlam);
	}
	else if (action_i_ == ac_open_slam || action_i_ == ac_movement_forward || action_i_ == ac_movement_turn ||
					 action_i_ == ac_movement_follow_wall || action_i_ == ac_movement_back) {
		if (!ACleanMode::isFinish()) {
			if (action_i_ == ac_movement_forward)
				sp_action_.reset(new MovementForward(GridMap::cellToPoint(plan_path_.back()), plan_path_));
			else if (action_i_ == ac_movement_follow_wall)
				sp_action_.reset(new MovementFollowWall(GridMap::getCurrPoint(), GridMap::cellToPoint(plan_path_.back())));
			else if (action_i_ == ac_movement_back)
				sp_action_.reset(new MovementBack());
			else if (action_i_ == ac_movement_turn)
				sp_action_.reset(new MovementTurn(plan_path_.back().TH));
		}
		sp_action_ == nullptr;
	}
	PP_INFO();
	ROS_INFO("action = %d", action_i_);
	return true;
}

State *CleanModeNav::getNextState() {
	if(state_i_ == st_null)
	{
		PP_INFO();
		plan_path_ = generatePath(nav_map, nav_map.getCurrCell(),g_old_dir);
		state_i_ = st_clean;
		displayPath(plan_path_);
		return new StateClean;
	}
	state_i_ = ac_null;
	return nullptr;
}

void CleanModeNav::register_events(void)
{
	event_manager_register_handler(this);
	event_manager_set_enable(true);
}

int CleanModeNav::getNextMovement() {
	if(movement_i_ == mv_null)
	{
		PP_INFO();
		movement_i_ = mv_turn;
		return ac_movement_turn;
	}
	else if(movement_i_ == mv_turn)
	{
		PP_INFO();
		movement_i_ = mv_forward;
		return ac_movement_forward;
	}

	else if(movement_i_ == mv_forward)
	{
		PP_INFO();

		if( ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered )
			movement_i_ = mv_back;
		return ac_movement_back;
	}
//	else if(movement_i_ == mv_turn2)
//	{
//		PP_INFO();
//		return ac_null;
//	}
	PP_INFO();
	return ac_null;
}

IMoveType *CleanModeNav::getNextMoveType(const Cell_t& start, MapDirection dir) {

//	if(move_type_i_ == mt_null) {
		PP_INFO();
		if (state_i_ == st_clean) {
			auto delta_y = plan_path_.back().Y - start.Y;
//		ROS_INFO( "%s,%d: path size(%u), dir(%d), g_check_path_in_advance(%d), bumper(%d), cliff(%d), lidar(%d), delta_y(%d)",
//						__FUNCTION__, __LINE__, path.size(), dir, g_check_path_in_advance, ev.bumper_triggered, ev.cliff_triggered,
//						ev.lidar_triggered, delta_y);

			if (!GridMap::isXDirection(dir) // If last movement is not x axis linear movement, should not follow wall.
					|| plan_path_.size() > 2 ||
					(!g_check_path_in_advance && !ev.bumper_triggered && !ev.cliff_triggered && !ev.lidar_triggered)
					|| delta_y == 0 || std::abs(delta_y) > 2) {
				move_type_i_ = mt_linear;
				PP_INFO();
				action_i_ = ac_movement_turn;
				return new MoveTypeLinear;
			}
			delta_y = plan_path_.back().Y - start.Y;
			bool is_left = ((GridMap::isPositiveDirection(g_old_dir) && GridMap::isXDirection(g_old_dir)) ^ delta_y > 0);
//		ROS_INFO("\033[31m""%s,%d: target:, 1_left_2_right(%d)""\033[0m", __FUNCTION__, __LINE__, get());
			move_type_i_ = mt_follow_wall;
			PP_INFO();
			return new MoveTypeFollowWall(is_left);
		}
//	}

	PP_INFO();
	return nullptr;
}


Path_t CleanModeNav::generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir)
{
	Path_t path;
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
		if (map.getCell(CLEAN_MAP, cell.X, cell.Y) != UNCLEAN)
			b_map.Add(cell);
	}

	TargetList filtered_targets = filterAllPossibleTargets(map, curr_cell, b_map);

	//Step 3: Generate the COST_MAP for map and filter targets that are unreachable.
	TargetList reachable_targets = getReachableTargets(map, curr_cell, filtered_targets);
	ROS_INFO("%s %d: After generating COST_MAP, Get %lu reachable targets.", __FUNCTION__, __LINE__, reachable_targets.size());
	if (reachable_targets.size() != 0)
		displayTargets(reachable_targets);
	else
		// Now path is empty.
		return path;

	//Step 4: Trace back the path of these targets in COST_MAP.
	PathList paths_for_reachable_targets = tracePathsToTargets(map, reachable_targets, curr_cell);

	//Step 5: Filter paths to get the best target.
	Cell_t best_target;
	if (!filterPathsToSelectTarget(map, paths_for_reachable_targets, curr_cell, best_target))
		// Now path is empty.
		return path;

	//Step 6: Find shortest path for this best target.
	Path_t shortest_path = findShortestPath(map, curr_cell, best_target, last_dir);
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

Path_t CleanModeNav::findTargetInSameLane(GridMap &map, const Cell_t &curr_cell)
{
	int8_t is_found = 0;
	Cell_t it[2]; // it[0] means the furthest cell of X positive direction, it[1] means the furthest cell of X negative direction.

//	map.print(CLEAN_MAP, 0, 0);
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

	Path_t path{};
	if (is_found)
	{
		path.push_front(target);
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target:(%d,%d)", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y, target.X, target.Y);
		map.print(CLEAN_MAP, target.X, target.Y);
	}
	else
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target not found.", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y);

	return path;
}

TargetList CleanModeNav::filterAllPossibleTargets(GridMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map)
{
	TargetList possible_target_list{};

	auto b_map_copy = b_map;
	b_map_copy.pos_ = b_map.min;

	// Check all boundarys between cleaned cells and unclean cells.
	for (const auto &cell : b_map_copy) {
		if (map.getCell(CLEAN_MAP, cell.X, cell.Y) != CLEANED /*|| std::abs(cell.Y % 2) == 1*/)
			continue;

		Cell_t neighbor;
		for (auto i = 0; i < 4; i++) {
			neighbor = cell + cell_direction_index[i];
			if (map.getCell(CLEAN_MAP, neighbor.X, neighbor.Y) == UNCLEAN && map.isBlockAccessible(neighbor.X, neighbor.Y))
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

TargetList CleanModeNav::getReachableTargets(GridMap &map, const Cell_t &curr_cell, TargetList &possible_targets)
{
	map.generateSPMAP(curr_cell, possible_targets);
	TargetList reachable_targets{};
	for (auto it = possible_targets.begin(); it != possible_targets.end();)
	{
		CellState it_cost = map.getCell(COST_MAP, it->X, it->Y);
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

PathList CleanModeNav::tracePathsToTargets(GridMap &map, const TargetList &target_list, const Cell_t& start)
{
	PathList paths{};
	int16_t trace_cost, x_min, x_max, y_min, y_max;
	map.getMapRange(COST_MAP, &x_min, &x_max, &y_min, &y_max);
	for (auto& it : target_list) {
		auto trace = it;
		Path_t path{};
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

bool CleanModeNav::filterPathsToSelectTarget(GridMap &map, const PathList &paths, const Cell_t &curr_cell, Cell_t &best_target)
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

