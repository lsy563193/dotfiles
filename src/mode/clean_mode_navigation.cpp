//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <pp.h>
#include "arch.hpp"

#define NAV_INFO() ROS_INFO("st(%d),mt(%d),ac(%d)", state_i_, move_type_i_, action_i_)

CleanModeNav::CleanModeNav()
{
	IMovement::sp_cm_ = this;
	if (g_resume_cleaning || cs_is_paused())
	{
		speaker.play(SPEAKER_CLEANING_CONTINUE);
	}
	else if(g_plan_activated)
	{
		g_plan_activated = false;
	}
	else{
		speaker.play(SPEAKER_CLEANING_START);
	}
}

CleanModeNav::~CleanModeNav()
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);


	if (ev.key_clean_pressed)
	{
		speaker.play(SPEAKER_CLEANING_FINISHED);
		ROS_WARN("%s %d: Key clean pressed. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else if (ev.cliff_all_triggered)
	{
		speaker.play(SPEAKER_ERROR_LIFT_UP, false);
		speaker.play(SPEAKER_CLEANING_STOP);
		ROS_WARN("%s %d: Cliff all triggered. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(SPEAKER_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}

	auto cleaned_count = nav_map.getCleanedArea();
	auto map_area = cleaned_count * (CELL_SIZE * 0.001) * (CELL_SIZE * 0.001);
	ROS_INFO("%s %d: Cleaned area = \033[32m%.2fm2\033[0m, cleaning time: \033[32m%d(s) %.2f(min)\033[0m, cleaning speed: \033[32m%.2f(m2/min)\033[0m.",
			 __FUNCTION__, __LINE__, map_area, robot_timer.getWorkTime(),
			 static_cast<float>(robot_timer.getWorkTime()) / 60, map_area / (static_cast<float>(robot_timer.getWorkTime()) / 60));
}

bool CleanModeNav::map_mark() {
	displayPath(passed_path_);
	if (move_type_i_ == mt_linear) {
		PP_INFO()
		nav_map.setCleaned(passed_path_);
	}

	if (state_i_ == st_trapped)
		nav_map.markRobot(CLEAN_MAP);

	nav_map.setBlocks();
	if (move_type_i_ == mt_follow_wall_left || move_type_i_ == mt_follow_wall_right)
		nav_map.setFollowWall();
	if (state_i_ == st_trapped)
		fw_map.setFollowWall();

	PP_INFO()
	nav_map.print(CLEAN_MAP, nav_map.getXCell(), nav_map.getYCell());

	passed_path_.clear();
	return false;
}
bool CleanModeNav::isExit()
{
	if (ev.key_clean_pressed || ev.cliff_all_triggered)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (ev.charge_detect)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	return false;
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
//		displayTargets(reachable_targets);
		displayPath(reachable_targets);
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
	fillPathWithDirection(shortest_path);

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
		path.push_front(nav_map.getCurrCell());
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target:(%d,%d)", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y, target.X, target.Y);
//		map.print(CLEAN_MAP, target.X, target.Y);
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

void CleanModeNav::key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);
	ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void CleanModeNav::cliff_all(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);

	ev.cliff_all_triggered = true;
}

bool CleanModeNav::setNextMoveType() {
	move_type_i_ = mt_linear;
	auto start = nav_map.getCurrCell();
	auto dir = old_dir_;
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
		}
		else {
			delta_y = plan_path_.back().Y - start.Y;
			bool is_left = ((GridMap::isPositiveDirection(old_dir_) && GridMap::isXDirection(old_dir_)) ^ delta_y > 0);
//		ROS_INFO("\033[31m""%s,%d: target:, 1_left_2_right(%d)""\033[0m", __FUNCTION__, __LINE__, get());
			move_type_i_ = is_left ? mt_follow_wall_left : mt_follow_wall_right;
		}
	}
	return ACleanMode::setNextMoveType();
}

bool CleanModeNav::MovementFollowWallisFinish() {
	return isNewLineReach();
}

bool CleanModeNav::isOverOriginLine()
{
	auto s_curr_p = nav_map.getCurrPoint();
	auto curr = nav_map.pointToCell(s_curr_p);
	if ((plan_path_.back().Y > s_origin_p.Y && (s_origin_p.Y - s_curr_p.Y) > 120)
		|| (plan_path_.back().Y < s_origin_p.Y && (s_curr_p.Y - s_origin_p.Y) > 120))
	{
		auto target_angle = (plan_path_.back().Y > s_origin_p.Y) ? -900 : 900;
		//ROS_INFO("%s %d: target_angel(%d),curr(%d)diff(%d)", __FUNCTION__, __LINE__, target_angel, robot::instance()->getPoseAngle(), target_angel - robot::instance()->getPoseAngle());
		if (std::abs(ranged_angle(robot::instance()->getPoseAngle() - target_angle)) < 50) // If robot is directly heading to the opposite side of target line, stop.
		{
			ROS_WARN("%s %d: Opposite to target angle. s_curr_p(%d, %d), plan_path_.back()(%d, %d), gyro(%d), target_angle(%d)",
					 __FUNCTION__, __LINE__, s_curr_p.X, s_curr_p.Y, plan_path_.back().X, plan_path_.back().Y, robot::instance()->getPoseAngle(), target_angle);
			return true;
		}
		else if (nav_map.isBlockCleaned(curr.X, curr.Y)) // If robot covers a big block, stop.
		{
			ROS_WARN("%s %d: Back to cleaned place, current(%d, %d), s_curr_p(%d, %d), plan_path_.back()(%d, %d).",
					 __FUNCTION__, __LINE__, curr.X, curr.Y, s_curr_p.X, s_curr_p.Y, plan_path_.back().X, plan_path_.back().Y);
			return true;
		}
		else{
			// Dynamic adjust the origin line and target line, so it can smoothly follow the wall to clean.
			plan_path_.back().Y += s_curr_p.Y - s_origin_p.Y;
			s_origin_p.Y = s_curr_p.Y;
		}
	}

	return false;
}

bool CleanModeNav::isNewLineReach()
{
	auto s_curr_p = nav_map.getCurrPoint();
	auto ret = false;
	auto is_pos_dir = plan_path_.back().Y - s_origin_p.Y > 0;
	// The limit is CELL_COUNT_MUL / 8 * 3 further than target line center.
	auto target_limit = plan_path_.back().Y + CELL_COUNT_MUL / 8 * 3 * is_pos_dir;
//	ROS_WARN("~~~~~~~~~~~~~~~~~%s %d: Reach the target limit, s_origin_p.Y(%d), target.Y(%d),curr_y(%d)",
//					 __FUNCTION__, __LINE__, nav_map.countToCell(s_origin_p.Y), nav_map.countToCell(plan_path_.back().Y),
//					 nav_map.countToCell(s_curr_p.Y));
	if (is_pos_dir ^ s_curr_p.Y < target_limit) // Robot has reached the target line limit.
	{
		ROS_WARN("%s %d: Reach the target limit, s_origin_p.Y(%d), target.Y(%d),curr_y(%d)",
				 __FUNCTION__, __LINE__, nav_map.countToCell(s_origin_p.Y), nav_map.countToCell(plan_path_.back().Y),
				 nav_map.countToCell(s_curr_p.Y));
		ret = true;
	}
	else if (is_pos_dir ^ s_curr_p.Y < plan_path_.back().Y)
	{
		// Robot has reached the target line center but still not reach target line limit.
		// Check if the wall side has blocks on the costmap.
		auto dx = (is_pos_dir ^ move_type_i_ == mt_follow_wall_left) ? +2 : -2;
		if (nav_map.isBlocksAtY(nav_map.countToCell(s_curr_p.X) + dx, nav_map.countToCell(s_curr_p.Y))) {
			ROS_WARN("%s %d: Already has block at the wall side, s_origin_p.Y(%d), target.Y(%d),curr_y(%d)",
					 __FUNCTION__, __LINE__, nav_map.countToCell(s_origin_p.Y), nav_map.countToCell(plan_path_.back().Y),
					 nav_map.countToCell(s_curr_p.Y));
			ret = true;
		}
	}

	return ret;
}