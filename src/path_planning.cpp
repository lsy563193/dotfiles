/**
******************************************************************************
* @file        Shortest Path
* @author      ILife Team Patrick Chau
* @version     Ver 20160118
* @date        18-Jan-2016
* @brief       Function to find the shorest path to target
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
******************************************************************************
*/

/*
 * Path Planning for robot movement is a ZigZag way. When cleaning process starts,
 * robot will try to clean its left hand side first, a new lane must be cleaned
 * in both ends before getting the new cleaning target. Targets will be added to
 * the target list as soon as the robot is moving.
 *
 * After the cleaning process is done, robot will back to its starting point and
 * finishes the cleaning.
 */
#include "pp.h"
#include "map.h"
#define FINAL_COST (1000)

#define TRAPPED 2
using namespace std;
const int ISOLATE_COUNT_LIMIT = 4;
int16_t g_new_dir;
int16_t g_old_dir;
bool g_check_path_in_advance = false;
bool g_allow_check_path_in_advance = true;
int g_wf_reach_count;
std::deque <PPTargetType> g_paths;

uint8_t g_direct_go = 0; /* Enable direct go when there is no obstcal in between current pos. & dest. */

// This list is for storing the position that robot sees the charger stub.

std::vector<Cell_t> g_homes;
std::vector<int> g_home_way_list;
std::vector<int>::iterator g_home_way_it;
Cell_t g_home_point;
//int8_t g_home_cnt = 0;// g_homes.size()*HOMEWAY_NUM-1 3/9, 2/4, 1/2
bool g_home_gen_rosmap = true;

/*
 *     ^x      4 0 5
 *     |       2 8 3
 * y<---       6 1 7
 */
Cell_t g_index[9]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1},{0,0}};

const int16_t g_home_x = 0, g_home_y = 0;
Cell_t g_zero_home = {0,0};
Cell_t g_virtual_target = {SHRT_MAX,SHRT_MAX};//for followall

Cell_t g_continue_cell;

uint32_t g_wf_start_timer;
uint32_t g_wf_diff_timer;

uint8_t g_trapped_cell_size = ESCAPE_TRAPPED_REF_CELL_SIZE;

bool sort_g_targets_y_ascend(PPTargetType a, PPTargetType b)
{
	return a.back().Y < b.back().Y;
}

static std::vector<int>::iterator _gen_home_ways(int size, std::vector<int> &go_home_way_list) {
	/*	ROS_INFO("%s,%d: go_home_way_list 1:                       2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 2: 5,      4,     3,     2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 3: 8,5,    7,4,   6,3,   2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 4: 11,8,5, 10,7,4,9,6,3, 2,1,0",__FUNCTION__, __LINE__);
	go_home_way_list.resize(size * HOMEWAY_NUM,0);
	if(size == 4) 			go_home_way_list = {11,8,5, 10,7,4,9,6,3, 2,1,0};
	else if(size == 3)	go_home_way_list = {8,5,    7,4,   6,3,   2,1,0};
	else if(size == 2)	go_home_way_list = {5,      4,     3,     2,1,0};
	else if(size == 1)	go_home_way_list = {                      2,1,0};

std::iota(go_home_way_list.begin(), go_home_way_list.end(),0);
	std::sort(go_home_way_list.begin(), go_home_way_list.end(), [](int x,int y){
			return (x >= 3 && y >= 3) && (x % 3) < (y % 3);
	});
	std::reverse(go_home_way_list.begin(),go_home_way_list.end());
	std::copy(go_home_way_list.begin(), go_home_way_list.end(),std::ostream_iterator<int>(std::cout, " "));*/

	ROS_INFO("%s,%d: go_home_way_list 1:                       2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 2: 5,      4,     3,     2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 3: 8,5,    7,4,   6,3,   2,1,0", __FUNCTION__, __LINE__);
	ROS_INFO("%s,%d: go_home_way_list 4: 11,8,5, 10,7,4,9,6,3, 2,1,0",__FUNCTION__, __LINE__);
	if(size == 4) 			go_home_way_list = {5,8,11, 4,7,10,3,6,9, 2,1,0};
	else if(size == 3)	go_home_way_list = {5,8,    4,7,   3,6,   2,1,0};
	else if(size == 2)	go_home_way_list = {5,      4,     3,     2,1,0};
	else if(size == 1)	go_home_way_list = {                      2,1,0};
	std::cout << std::endl;

	return go_home_way_list.begin();
}

bool cell_is_out_of_range(const Cell_t &cell)
{
	return std::abs(cell.X) > MAP_SIZE || std::abs(cell.Y) > MAP_SIZE;
}

void path_planning_initialize()
{
	g_direct_go = 0;
	cost_map.mark_robot(MAP);
}

uint16_t path_get_robot_direction()
{
	return g_plan_path.front().TH;
}


//int16_t path_lane_distance(bool is_min)
//{
//	int angle = robot::instance()->getPoseAngle();
//	if(is_min)
//		angle = uranged_angle(angle + 1800);
//	angle /= 10;
//	auto dis = MotionManage::s_laser->getLaserDistance(angle);
//	int16_t cell_dis = dis * 1000 * CELL_COUNT_MUL / CELL_SIZE;
//	if(is_min)
//		ROS_INFO("min cell_dis(%d)",count_to_cell(cell_dis) );
//	else
//		ROS_INFO("max cell_dis(%d)",count_to_cell(cell_dis) );
//
//	return cell_dis;
//}

bool path_lane_is_cleaned(const Cell_t& curr, PPTargetType& path)
{
	int16_t is_found=0;
	Cell_t it[2]; // it[0] means the furthest cell of X positive direction, it[1] means the furthest cell of X negative direction.

//	map.print(MAP, 0, 0);
	for (auto i = 0; i < 2; i++) {
		it[i] = curr;
		auto uc = 0;
		for (Cell_t neighbor = it[i] + g_index[i]; !cell_is_out_of_range(neighbor + g_index[i]) && !cost_map.is_block_blocked(
						neighbor.X, neighbor.Y); neighbor += g_index[i]) {
			uc += cost_map.is_block_unclean(neighbor.X, neighbor.Y);
			if (uc >= 3) {
				it[i] = neighbor;
				uc = 0;
//				ROS_INFO("%s %d: it[%d](%d,%d)", __FUNCTION__, __LINE__, i, it[i].X, it[i].Y);
			}
//			ROS_WARN("%s %d: it[%d](%d,%d)", __FUNCTION__, __LINE__, i, it[i].X, it[i].Y);
//			ROS_WARN("%s %d: nb(%d,%d)", __FUNCTION__, __LINE__, neighbor.X, neighbor.Y);
		}
	}

	Cell_t target = it[0];
	if (it[0].X != curr.X)
	{
		is_found++;
	} else if (it[1].X != curr.X)
	{
		target = it[1];
		is_found++;
	}
	if (is_found == 2)
	{
		target = it[0];
		//todo
//		ROS_WARN("%s %d: nag dir(%d)", __FUNCTION__, __LINE__, (Movement::s_target_p.Y<Movement::s_origin_p.Y));
//		if(mt.is_follow_wall() && cm_is_reach())
//		{
//			if(mt.is_left() ^ (Movement::s_target_p.Y<Movement::s_origin_p.Y))
//				target = it[1];
//		}
	}

	if (is_found)
	{
		path.push_front(target);
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target:(%d,%d)", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y, target.X, target.Y);
		cost_map.print(MAP, target.X, target.Y);
	}
	else
		ROS_INFO("%s %d: X pos:(%d,%d), X neg:(%d,%d), target not found.", __FUNCTION__, __LINE__, it[0].X, it[0].Y, it[1].X, it[1].Y);

	return is_found !=0;
}

bool is_axis_access(const Cell_t &start, int i, Cell_t &target)
{
	auto is_found = false;
	for (auto tmp = start; std::abs(tmp.X) < MAP_SIZE && std::abs(tmp.Y) < MAP_SIZE && std::abs(tmp.Y- start.Y)  <=1; tmp += g_index[i]) {
//		ROS_INFO("%s, %d:tmp(%d,%d)", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		if(cost_map.is_block_cleanable(tmp.X, tmp.Y) ) {
			if ((i == 0 || i == 1) || cost_map.is_block_accessible(tmp.X, tmp.Y)) {
				is_found = true;
				target = tmp;
				if (i == 2 || i == 3)
					target -= g_index[i];
			}
		}else {
			break;
		}
	}
	return is_found;
}


/*void path_find_target(const Cell_t& curr, PPTargetType& path,const Cell_t& target) {
	bool all_set;
	int16_t offset, passValue, nextPassValue, passSet, targetCost;
	CellState cs;

	cost_map.reset(SPMAP);

	for (auto x = g_x_min; x <= g_x_max; ++x) {
		for (auto y = g_y_min; y <= g_y_max; ++y) {
			cs = cost_map.get_cell(MAP, x, y);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				for (auto dx = ROBOT_RIGHT_OFFSET; dx <= ROBOT_LEFT_OFFSET; dx++) {
					for (auto dy = ROBOT_RIGHT_OFFSET; dy <= ROBOT_LEFT_OFFSET; dy++) {
						cost_map.set_cell(SPMAP, x + dx, y + dy, COST_HIGH);
					}
				}
			}
		}
	}
*//* Set the current robot position has the cost value of 1. *//*
	cost_map.set_cell(SPMAP, (int32_t) curr.X, (int32_t) curr.Y, COST_1);

	offset = 0;
	passSet = 1;
	passValue = 1;
	nextPassValue = 2;
	while (passSet == 1) {
		offset++;
		passSet = 0;
		Cell_t cell;
		for (cell.X = curr.X - offset; cell.X <= curr.X + offset; cell.X++) {
			for (cell.Y = curr.Y - offset; cell.Y <= curr.Y + offset; cell.Y++) {
				if (cost_map.get_cell(SPMAP, cell.X, cell.Y) == passValue) {
					for (auto i = 0; i < 4; i++) {
						auto neighbor = g_index[i] + cell;
						if (neighbor > MIN_CELL && neighbor < MAX_CELL)
							if (cost_map.get_cell(SPMAP, neighbor.X, neighbor.Y) == COST_NO) {
								cost_map.set_cell(SPMAP, neighbor.X, neighbor.Y, (CellState) nextPassValue);
								passSet = 1;
							}
					}
				}
			}
		}

		all_set = true;
		if (cost_map.get_cell(SPMAP, path.back().X, path.back().Y) == COST_NO) {
			all_set = false;
		}
		if (all_set) {
			ROS_INFO("%s %d: all possible target are checked & reachable.", __FUNCTION__, __LINE__);
			passSet = 0;
		}

		passValue = nextPassValue;
		nextPassValue++;
		if (nextPassValue == COST_PATH)
			nextPassValue = 1;
	}

#if DEBUG_SM_MAP
//	map.print(SPMAP, 0, 0);
#endif

	if (cost_map.get_cell(SPMAP, target.X, target.Y) == COST_NO ||
			cost_map.get_cell(SPMAP, target.X, target.Y) == COST_HIGH) {
		return;
	}

	Cell_t tmp;
	Cell_t trace = target;
	while (trace != curr) {
		targetCost = cost_map.get_cell(SPMAP, trace.X, trace.Y) - 1;

		if (targetCost == 0) {
			targetCost = COST_5;
		}

		tmp = trace;
    ROS_INFO("%s %d: tmp(%d,%d)", __FUNCTION__, __LINE__, tmp.X, tmp.Y);
		path.push_back(tmp);

		for (auto i = 0; i < 4; i++) {
			auto neighbor = trace + g_index[i];
			if (neighbor > MIN_CELL && neighbor < MAX_CELL)
				if (cost_map.get_cell(SPMAP, neighbor.X, neighbor.Y) == targetCost) {
					trace = neighbor;
					break;
				}
		}
		if (trace != tmp)
			continue;
	}
	if (trace.X == curr.X || trace.Y == curr.Y) {
		tmp = trace;
		path.push_back(tmp);
	}

	std::reverse(path.begin(), path.end());
}*/
/*

bool path_full(const Cell_t& curr, PPTargetType& path)
{
	auto is_found = false;
	auto target = curr;
	auto tmp = curr;
  */
/*if(curr.Y % 2 != 0) {
		if (mt.is_follow_wall()) {
			auto dir = path.target.Y - g_cell_history[1].Y;//+2,-2
			auto step = curr.Y - g_cell_history[1].Y;
			auto is_local = (dir > 0) ? step <= 0 : step >= 0;
			tmp.Y  = (is_local) ? g_cell_history[1].Y : path.target.Y;
			ROS_ERROR("%s %d:follow_wall dir(%d),step(%d),tmp(%d,%d),his1(%d,%d)", __FUNCTION__, __LINE__, dir, step, tmp.X,
								tmp.Y, g_cell_history[1].X, g_cell_history[1].Y);
		}
		if (mt.is_linear() && IS_X_AXIS(g_new_dir)) {
			auto dir = path.target.Y - g_cell_history[1].Y;//+2,-2
			tmp.Y = g_cell_history[1].Y;
			ROS_ERROR("%s %d:mt.is_linear tmp(%d,%d),his1(%d,%d),curr(%d,%d)", __FUNCTION__, __LINE__, tmp.X, tmp.Y,
								g_cell_history[1].X, g_cell_history[1].Y, curr.X, curr.Y);
		}
	}*//*

	ROS_INFO("%s %d: curr(%d,%d)", __FUNCTION__, __LINE__, curr.X, curr.Y);
	auto i=0;
	for(;i<4;i++)
	{
//		ROS_INFO("%s %d: i(%d)", __FUNCTION__, __LINE__, i);
		auto neighbor = tmp+g_index[i]+g_index[i];
		is_found = is_axis_access(neighbor, i, target);
		if(is_found) {
			path.clear();
			path.push_back(curr);
			path.push_back(target);//next
			break;
		}
	}
	ROS_INFO("%s %d: is_found(%d)", __FUNCTION__, __LINE__, is_found);
	if (! is_found) {
#if DEBUG_MAP
//		cost_map.print(MAP, 0, 0);
#endif
		int cleaned_count;
		is_found = path_dijkstra(curr, target, cleaned_count);
		if(is_found) {
			ROS_INFO("%s %d: is_found(%d), i(%d) target(%d,%d)", __FUNCTION__, __LINE__, is_found, i, target.X, target.Y);
//			pathFind(curr, target, path.cells);
			if( target.Y%2 != 0) {
				ROS_ERROR("%s %d: is_found(%d), i(%d) target(%d,%d)", __FUNCTION__, __LINE__, is_found, i, target.X, target.Y);
				if(curr.Y < target.Y*/
/* && is_block_accessible(target.X,target.Y+1)*//*
)
					target.Y = curr.Y+2;
				else if(curr.Y > target.Y*/
/* && is_block_accessible(target.X,target.Y+1))*//*
)
					target.Y = curr.Y-2;
				ROS_ERROR("%s %d: is_found(%d), i(%d) target(%d,%d)", __FUNCTION__, __LINE__, is_found, i, target.X, target.Y);
			}
			path.clear();
			path_find_target(curr, path,target);
//			for (const auto &cell : path.cells)
//				ROS_WARN("%s %d: cell(%d,%d)", __FUNCTION__, __LINE__, cell.X, cell.Y);
		}
	}
	return is_found;
}
*/

typedef int32_t(*Func_t)(void);

//bool for_each_neighbor(const Cell_t& cell,CellState cost, Func_t func) {
//	for (auto i = 0; i < 4; i++) {
//		Cell_t neighbor = g_index[i] + cell;
//		if (neighbor.Y >= g_y_min && neighbor.Y <= g_y_max && neighbor.X >= g_x_min && neighbor.X <= g_x_max)
//			if (cost_map.get_cell(SPMAP, neighbor.X, neighbor.Y) == cost) {
//				func();
//      }
//	}
//}

void path_find_all_targets(const Cell_t& curr, BoundingBox2& map) {
	auto map_tmp=  cost_map.generateBound();

	for (const auto &cell : map_tmp) {
		if (cost_map.get_cell(MAP, cell.X, cell.Y) != UNCLEAN)
			map.Add(cell);
	}

	map_tmp = map;
	map_tmp.pos_ = map.min;

	/* Clear the target list and path list. */
	for (auto &target : g_paths)
		target.clear();
	g_paths.clear();

	std::deque<Cell_t>cells{};

	cost_map.reset(SPMAP);
	for (const auto &cell : map_tmp) {
		if (cost_map.get_cell(MAP, cell.X, cell.Y) != CLEANED /*|| std::abs(cell.Y % 2) == 1*/)
			continue;

		Cell_t neighbor;
		for (auto i = 0; i < 4; i++) {
			neighbor = cell + g_index[i];
			if (cost_map.get_cell(MAP, neighbor.X, neighbor.Y) == UNCLEAN) {
				if (cost_map.is_block_accessible(neighbor.X, neighbor.Y) == 1 && cost_map.get_cell(SPMAP, neighbor.X, neighbor.Y) == UNCLEAN) {
					cost_map.set_cell(SPMAP, neighbor.X, neighbor.Y, CLEANED);
					cells.push_back(neighbor);
				}
			}
		}
	}
	std::sort(cells.begin(),cells.end(),[](Cell_t l,Cell_t r){
			return (l.Y < r.Y || (l.Y == r.Y && l.X < r.X));
	});
	path_display_targets(cells);

	ROS_INFO("%s %d: Filter targets in the same line.", __FUNCTION__, __LINE__);
	std::deque<Cell_t>filtered_cells{};
	/* Filter the targets. */
	for(;!cells.empty();) {
		auto y = cells.front().Y;
		std::deque<Cell_t> tmp{};
		std::remove_if(cells.begin(), cells.end(), [&y, &tmp](Cell_t &it) {
			if (it.Y == y && (tmp.empty() || (it.X - tmp.back().X == 1))) {
				tmp.push_back(it);
				return true;
			}
			return false;
		});
		cells.resize(cells.size() - tmp.size());
		if (tmp.size() > 2) {
			tmp.erase(std::remove_if(tmp.begin() + 1, tmp.end() - 1, [&curr](Cell_t &it) {
					return it.X != curr.X;
			}), tmp.end() - 1);
		}
		for(const auto& it:tmp){
			PPTargetType t = {{it}};
			g_paths.push_back(t);
			filtered_cells.push_back(it);
		};
	}

	path_display_targets(filtered_cells);
//#if DEBUG_MAP
	// Print for costmap that contains all targets.
//	cost_map.print(MAP, g_home_x, g_home_y);
//#endif

	// Restore the target cells in MAP to unclean.
//	for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
//		cost_map.set_cell(MAP, cell_to_count(it->back().X), cell_to_count(it->back().Y), UNCLEAN);
//	}
//	ROS_INFO("%s %d: Found %lu targets from MAP.", __FUNCTION__, __LINE__, g_paths.size());
}

bool get_reachable_targets(const Cell_t& curr, BoundingBox2& map)
{
	ROS_INFO("%s %d: Start getting reachable targets.", __FUNCTION__, __LINE__);
	path_find_all_targets(curr, map);
	cost_map.generate_SPMAP(curr,g_paths);
	PPTargetType reachable_targets{};
	for (auto it = g_paths.begin(); it != g_paths.end();) {
		if (cost_map.get_cell(SPMAP, it->back().X, it->back().Y) == COST_NO ||
						cost_map.get_cell(SPMAP, it->back().X, it->back().Y) == COST_HIGH) {
			// Drop the unreachable targets.
			it = g_paths.erase(it);
			continue;
		}
		else
		{
			reachable_targets.push_back(it->back());
			it++;
		}
	}

	ROS_INFO("%s %d: After generating SPMAP, Get %lu reachable targets.", __FUNCTION__, __LINE__, g_paths.size());
	if (!g_paths.empty())
		path_display_targets(reachable_targets);

	return !g_paths.empty();
}

void generate_path_to_targets(const Cell_t& curr)
{
	int16_t targetCost, x_min, x_max, y_min, y_max;
	cost_map.path_get_range(SPMAP, &x_min, &x_max, &y_min, &y_max);
	for (auto& it : g_paths) {
		auto trace = it.front();
		it.pop_front();
		while (trace != curr) {
			targetCost = cost_map.get_cell(SPMAP, trace.X, trace.Y) - 1;

			if (targetCost == 0) {
				targetCost = COST_5;
			}

			it.push_back(trace);

			if ((trace.X - 1 >= x_min) && (cost_map.get_cell(SPMAP, trace.X - 1, trace.Y) == targetCost)) {
				trace.X--;
				continue;
			}

			if ((trace.X + 1 <= x_max) && (cost_map.get_cell(SPMAP, trace.X + 1, trace.Y) == targetCost)) {
				trace.X++;
				continue;
			}

			if ((trace.Y - 1 >= y_min) && (cost_map.get_cell(SPMAP, trace.X, trace.Y - 1) == targetCost)) {
				trace.Y--;
				continue;
			}

			if ((trace.Y + 1 <= y_max) && (cost_map.get_cell(SPMAP, trace.X, trace.Y + 1) == targetCost)) {
				trace.Y++;
				continue;
			}
		}
		if (trace.X == curr.X || trace.Y == curr.Y) {
			it.push_back(trace);
		}
	}
}


bool path_target(const Cell_t& curr, PPTargetType& path)
{
	BoundingBox2 map;
	if (!get_reachable_targets(curr, map)) {
		ROS_WARN("%s %d: No more target to clean,check is_trapped()?!!", __FUNCTION__, __LINE__);
		return false;
	}

	generate_path_to_targets(curr);

	Cell_t temp_target;
	return (path_select_target(curr, temp_target, map) && path_next_shortest(curr, temp_target, path) == 1);
}

bool path_select_target(const Cell_t& curr, Cell_t& temp_target, const BoundingBox2& map)
{
	bool is_stop = false, is_found = false, within_range=false;
	int16_t y_max;
	deque <PPTargetType> temp_targets;
	temp_targets.clear();
	uint16_t final_cost = 1000;
	ROS_INFO("%s %d: case 1, towards Y+ only", __FUNCTION__, __LINE__);
	// Filter targets in Y+ direction of curr.
	for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
//		ROS_INFO("target(%d,%d)", it->front().X, it->front().Y);
//		path_display_path_points(*it);
		if (cost_map.get_cell(MAP, it->front().X, it->front().Y - 1) != CLEANED) {
			continue;
		}
		if (it->front().Y > curr.Y + 1)
		{
			temp_targets.push_front(*it);
		}
	}
	// Sort targets with Y ascend order.
	std::sort(temp_targets.begin(),temp_targets.end(),sort_g_targets_y_ascend);

	for (auto it = temp_targets.begin(); it != temp_targets.end(); ++it) {
		if (is_stop && temp_target.Y != it->front().Y)
			break;

		if (it->size() > final_cost)
			continue;

		y_max = it->front().Y;
		within_range = true;
		for (auto i = it->begin(); within_range && i != it->end(); ++i) {
			if (i->Y < curr.Y)
				// All turning cells should be in Y+ area, so quit it.
				within_range = false;
			if (i->Y > y_max)
				// Not allow path towards Y- direction.
				within_range = false;
			else
				y_max = i->Y;
		}
		if (within_range) {
			temp_target = it->front();
			final_cost = it->size();
			is_stop = true;
		}
	}

/*
	for (auto d = costmap.max.Y; d > curr.Y + 1; --d) {
		if (is_stop) {
			break;
		}
		for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
			if (cost_map.get_cell(MAP, it->target.X, it->target.Y - 1) != CLEANED) {
				continue;
			}
			if (it->target.Y == d) {
				if (it->cells.size() > final_cost) {
					continue;
				}

				y_max = it->cells.front().Y;
				within_range = true;
				for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
					if (i->Y < curr.Y || i->Y > d) {
						within_range = false;
					}
					if (i->Y > y_max) {
						within_range = false;
					} else {
						y_max = i->Y;
					}
				}
				if (within_range == true) {
					temp_target = it->target;
					final_cost = it->cells.size();
					is_stop = true;
				}
			}
		}
	}
*/

#if 0
	if (!is_stop) {
		ROS_INFO("%s %d: case 2, towards Y+, allow Y- shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto a = curr.Y; a >= costmap.min.Y && !is_stop; --a) {
			for (auto d = a; d <= costmap.max.Y && !is_stop; ++d) {
				for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
					if (it->target.Y == d) {
						if (it->cells.size() > final_cost) {
							continue;
						}

						within_range = true;
						y_max = it->cells.front().Y;
						bool turn = false;
						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
							if (i->Y < a || i->Y > (d > curr.Y ? d : curr.Y)) {
								within_range = false;
							}
							if (turn == false) {
								if (i->Y > y_max) {
									within_range = false;
								} else {
									y_max = i->Y;
								}
							} else {
								if (i->Y < y_max) {
									within_range = false;
								} else {
									y_max = i->Y;
								}
							}
							if (i->Y == a) {
								turn = true;
							}
						}
						if (within_range == true) {
							temp_target = it->target;
							final_cost = it->cells.size();
							is_stop = true;
						}
					}
				}
			}
		}
	}

	if (!is_stop) {
		ROS_INFO("%s %d: case 3, towards Y- only, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto d = costmap.min.Y; d >= curr.Y; ++d) {
			if (is_stop && d >= curr.Y - 1) {
				break;
			}

			for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
				if (cost_map.get_cell(MAP, it->target.X, it->target.Y + 1) != CLEANED) {
					continue;
				}

				if (it->target.Y == d) {
					if (it->cells.size() > final_cost) {
						continue;
					}

					y_max = it->cells.front().Y;
					within_range = true;
					for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
						if (i->Y > curr.Y || i->Y < d) {
							within_range = false;
						}
						if (i->Y < y_max) {
							within_range = false;
						} else {
							y_max = i->Y;
						}
					}
					if (within_range == true) {
						temp_target = it->target;
						final_cost = it->cells.size();
						is_stop = true;
					}
				}
			}
		}
	}

	if (!is_stop) {
		ROS_INFO("%s %d: case 4, towards Y-, allow Y+ shift, allow 1 turn, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto a = curr.Y; a <= costmap.max.Y && !is_stop; ++a) {
			for (auto d = a; d >= costmap.min.Y && !is_stop; --d) {
				for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
					if (it->target.Y == d) {
						if (it->cells.size() > final_cost) {
							continue;
						}

						within_range = true;
						y_max = it->cells.front().Y;
						bool turn = false;
						for (auto i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
							if (i->Y > a || i->Y < (d > curr.Y ? curr.Y : d)) {
								within_range = false;
							}
							if (turn == false) {
								if (i->Y < y_max) {
									within_range = false;
								} else {
									y_max = i->Y;
								}
							} else {
								if (i->Y > y_max) {
									within_range = false;
								} else {
									y_max = i->Y;
								}
							}
							if (i->Y == a) {
								turn = true;
							}
						}
						if (within_range == true) {
							temp_target = it->target;
							final_cost = it->cells.size();
							is_stop = true;
						}
					}
				}
			}
		}
	}
	if (!is_stop) {
		ROS_INFO("%s %d: case 5: towards Y+, allow Y- shift, allow turns, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto a = curr.Y; a <= costmap.max.Y  && is_stop == 0; ++a) {
			for (auto d = curr.Y; d <= a && is_stop == 0; ++d) {
				for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
					if (it->target.Y == d) {
						within_range = true;
						for (list<Cell_t>::iterator i = it->cells.begin(); within_range == true && i != it->cells.end(); ++i) {
							if (i->Y < curr.Y || i->Y > a) {
								within_range = false;
							}
						}
						if (within_range == true && it->cells.size() < final_cost) {
							temp_target = it->target;
							final_cost = it->cells.size();
							is_stop = 1;
						}
					}
				}
			}
		}
	}
#endif
	/* fallback to find unclean area */
	if (!is_stop) {
		ROS_INFO("%s %d: case 6, fallback to A-start the nearest target, cost: %d(%d)", __FUNCTION__, __LINE__, final_cost, is_stop);
		for (auto c = map.min.X; c <= map.max.X; ++c) {
			for (auto d = map.min.Y; d <= map.max.Y; ++d) {
				for (auto it = g_paths.begin(); it != g_paths.end(); ++it) {
					if (it->size() < final_cost) {
						temp_target = it->front();
						final_cost = it->size();
					}
				}
			}
		}
	}

	is_found = (final_cost != 1000) ? final_cost : 0 ;
	ROS_INFO("%s %d: is_found: %d target(%d, %d)\n", __FUNCTION__, __LINE__, is_found, temp_target.X, temp_target.Y);
#if DEBUG_MAP
	cost_map.print(MAP, temp_target.X, temp_target.Y);
#endif

	return is_found;
}

bool wf_is_isolate() {
//	path_update_cell_history();
	int16_t	val = 0;
	uint16_t i = 0;
	int16_t x_min, x_max, y_min, y_max;
	fw_map.path_get_range(MAP, &x_min, &x_max, &y_min, &y_max);
	Cell_t out_cell {int16_t(x_max + 1),int16_t(y_max + 1)};

	fw_map.mark_robot(MAP);//note: To clear the obstacle when check isolated, please don't remove it!
	auto curr = cost_map.point_to_cell(Movement::s_curr_p);
	fw_map.print(MAP, curr.X, curr.Y);
	ROS_WARN("%s %d: curr(%d,%d),out(%d,%d)", __FUNCTION__, __LINE__, curr.X, curr.Y,out_cell.X, out_cell.Y);

	if ( out_cell != g_zero_home){
			val = wf_path_find_shortest_path(curr.X, curr.Y, out_cell.X, out_cell.Y, 0);
			val = (val < 0 || val == SCHAR_MAX) ? 0 : 1;
	} else {
		if (cost_map.is_block_accessible(0, 0) == 1) {
			val = wf_path_find_shortest_path(curr.X, curr.Y, 0, 0, 0);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = wf_path_find_shortest_path(curr.X, curr.Y, 0, 0, 0);

				if (val < 0 || val == SCHAR_MAX) {
					val = 0;
				} else {
					val = 1;
				};
			} else {
				val = 1;
			}
		} else {
			val = wf_path_find_shortest_path(curr.X, curr.Y, 0, 0, 0);
			if (val < 0 || val == SCHAR_MAX)
				val = 0;
			else
				val = 1;
		}
	}
	return val != 0;
}

bool cm_is_reach()
{
	return (ev.bumper_triggered || ev.obs_triggered || ev.cliff_triggered || ev.rcon_triggered);
}

bool path_next_spot(const Cell_t &start, PPTargetType &path) {
	return SpotMovement::instance()->spotNextTarget(start, &path);
}

bool path_next_fw(const Cell_t &start) {
	ROS_INFO("%s,%d: path_next_fw",__FUNCTION__, __LINE__);
	if (mt.is_linear()) {
		ROS_INFO("%s,%d: path_next_fw",__FUNCTION__, __LINE__);
		if (cm_is_reach()) {
			ROS_INFO("%s,%d: path_next_fw",__FUNCTION__, __LINE__);
			mt.set(MT_FOLLOW_LEFT_WALL);
			g_plan_path.push_back(g_virtual_target);
			return true;
		}
	}
	else {
		ROS_INFO("%s,%d: path_next_fw",__FUNCTION__, __LINE__);
		if (g_wf_reach_count == 0 ||
				(g_wf_reach_count < ISOLATE_COUNT_LIMIT && !fw_is_time_up()/*get_work_time() < WALL_FOLLOW_TIME*/ &&
				 wf_is_isolate())) {
			fw_map.reset(MAP);
			auto angle = -900;
			if (g_wf_reach_count == 0) {
				angle = 0;
			}
			const float FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
			Cell_t cell;
			auto point = CostMap::get_curr_point();
			point.TH = ranged_angle(robot::instance()->getPoseAngle() + angle);
			cost_map.robot_to_cell(point, 0, FIND_WALL_DISTANCE * 1000, cell.X, cell.Y);
			g_plan_path.push_back(cell);
			mt.set(MT_LINEARMOVE);
			return true;
		}
	}
	return false;
}

bool path_next_nav(const Cell_t &start, PPTargetType &path)
{
	bool ret = true;
	if (g_resume_cleaning && !path_get_continue_target(start, path))
			g_resume_cleaning = false;

	if (!g_resume_cleaning) {
		if (!path_lane_is_cleaned(start, path)) {
			if (g_wf_reach_count > 0) {
				isolate_target(start, path);
			}
			else {
				ret = path_target(start, path);//-1 not target, 0 found
			}
		}
	}
	g_new_dir = path_full_angle(start, path);
	return ret;
}

bool path_next_nav_in_advance(int16_t &dir, const Cell_t &start, PPTargetType &path)
{
	bool ret = true;
	ret = path_lane_is_cleaned(start, path);
	if (!ret)
		ret = path_target(start, path);//-1 not target, 0 found
	if (ret)
		dir = path_full_angle(start, path);
	else
		dir = 0;
	return ret;
}

bool cm_turn_and_check_charger_signal(void)
{
	time_t delay_counter;
	bool eh_status_now, eh_status_last;

	ROS_INFO("%s, %d: wait about 1s to check if charger signal is received.", __FUNCTION__, __LINE__);
	delay_counter = time(NULL);
	while(time(NULL) - delay_counter < 2)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			usleep(100);
			continue;
		}

		if(c_rcon.get_status())
		{
			ROS_INFO("%s, %d: have seen charger signal, return and go home now.", __FUNCTION__, __LINE__);
			return true;
		}
		else if(ev.key_clean_pressed || ev.fatal_quit || ev.charge_detect)
		{
			ROS_INFO("%s, %d: event triggered, return now.", __FUNCTION__, __LINE__);
			return false;
		}
	}
	return false;
}

int16_t path_full_angle(const Cell_t& start, PPTargetType& path)
{
	path.push_front(start);
	for(auto it = path.begin(); it < path.end(); ++it) {
		auto it_next = it+1;
		if (it->X == it_next->X)
			it->TH = it->Y > it_next->Y ? NEG_Y : POS_Y;
		else
			it->TH = it->X > it_next->X ? NEG_X : POS_X;
	}
//		ROS_INFO("path.back(%d,%d,%d)",path.back().X, path.back().Y, path.back().TH);
	if(cs.is_going_home() && g_home_point == g_zero_home)
		path.back().TH = g_home_point.TH;
	else
		path.back().TH = (path.end()-2)->TH;
	ROS_INFO("%s %d: path.back(%d,%d,%d), path.front(%d,%d,%d)", __FUNCTION__, __LINE__,
			 path.back().X, path.back().Y, path.back().TH, path.front().X, path.front().Y, path.front().TH);
	int16_t dir = path.front().TH;
	path.pop_front();
	return dir;
}

bool path_next(const Cell_t& start, PPTargetType& path)
{
	if (cm_is_spot()) {
		return path_next_spot(start, path);
	}
	else if (cm_is_navigation()) {
		return path_next_nav(start, path);
	}
}

bool cs_path_next(const Cell_t& start, PPTargetType& path) {
/*	if (!cs.is_going_home()) {
		if ((ev.remote_home || ev.battery_home)) {//cs.is_switch_go_home()
			if(g_have_seen_charger)
				cs.set(CS_GO_HOME_POINT);
			else
				cs.set(CS_EXPLORATION);
		}
	}

	if (!cs.is_going_home()) {
		if (ev.remote_spot) {//cs.is_switch_tmp_spot()
			cs.set(CS_TMP_SPOT);
		}
	}*/

/*if (cs.is_trapped()) {
		if (!is_trapped(start, path))
			cs.set(CS_CLEAN);
	}

	if (cs.is_clean()) {
		if (!path_next(start, path))
		{
			ROS_INFO("%s%d:", __FUNCTION__, __LINE__);
			if (is_trapped(start, path))
			{
				cs.set(CS_TRAPPED);
				path.push_back(g_virtual_target);
			}
			else
			{
//				if(g_have_seen_charger)
					cs.set(CS_GO_HOME_POINT);
//				else
//					cs.set(CS_EXPLORATION);
			}
		}else
			mt.update(start,path);
	}
	else if (cs.is_tmp_spot()) {
		path_next_spot(start, path);
	}

	if (cs.is_go_home_point()) {
		if (start == g_home_point) {
			if (g_home_point != g_zero_home ||cm_turn_and_check_charger_signal())
					cs.set(CS_GO_CHANGER);
		}else

		if(path_get_home_point_target(start, path))
			cs.set(CS_EXPLORATION);
	}
	if (cs.is_exploration()) {
		if(g_have_seen_charger)
			cs.set(CS_GO_CHANGER);
		else
			if(path_next_nav(start, path))
				return false;
	}

	return true;*/

}

void path_fill_path(std::deque<Cell_t>& path)
{
	int16_t dir;
	Cell_t cell;
	auto saved_path = path;
	path.clear();
	//path_display_path_points(saved_path);

	//for (list<Cell_t>::iterator it = saved_path.begin(); it->X != saved_path.back().X || it->Y != saved_path.back().Y; it++)
	for (auto it = saved_path.begin(); it != saved_path.end(); it++)
	{
		auto next_it = it;
		if(++next_it == saved_path.end()){
			ROS_INFO("%s,%d,fill path to last interator",__FUNCTION__,__LINE__);
			break;
		}
		//ROS_DEBUG("%s %d: it(%d, %d), next it(%d, %d).", __FUNCTION__, __LINE__, it->X, it->Y, next_it->X, next_it->Y);
		if (next_it->X == it->X)
			dir = next_it->Y > it->Y ? POS_Y : NEG_Y;
		else
			dir = next_it->X > it->X ? POS_X : NEG_X;

		cell.X = it->X;
		cell.Y = it->Y;
		switch(dir)
		{
			case POS_X:
			{
				while (cell.X != next_it->X)
				{
					// Fill the path with the middle cells.
					path.push_back(cell);
					//ROS_DEBUG("%s %d: Push back(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
					cell.X++;
				}
				break;
			}
			case NEG_X:
			{
				while (cell.X != next_it->X)
				{
					// Fill the path with the middle cells.
					path.push_back(cell);
					//ROS_DEBUG("%s %d: Push back(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
					cell.X--;
				}
				break;
			}
			case POS_Y:
			{
				while (cell.Y != next_it->Y)
				{
					// Fill the path with the middle cells.
					path.push_back(cell);
					//ROS_DEBUG("%s %d: Push back(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
					cell.Y++;
				}
				break;
			}
			case NEG_Y:
			{
				while (cell.Y != next_it->Y)
				{
					// Fill the path with the middle cells.
					path.push_back(cell);
					//ROS_DEBUG("%s %d: Push back(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
					cell.Y--;
				}
				break;
			}
		}
	}
	// Push the target point to path.
	cell = saved_path.back();
	path.push_back(cell);
	//ROS_DEBUG("%s %d: End cell(%d, %d).", __FUNCTION__, __LINE__, cell.X, cell.Y);
	std::string msg = "Filled path:";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "->(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ")";
	}
	//ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());
}

void path_escape_set_trapped_cell( Cell_t *cell, uint8_t size )
{
	g_trapped_cell_size = size;
	for (auto i = 0; i < g_trapped_cell_size; ++i ) {
		g_homes[i] = cell[i];
		ROS_INFO("%s %d Set %d trapped reference cell: x: %d\ty:%d", __FUNCTION__, __LINE__, i, g_homes[i].X, g_homes[i].Y);
	}
}

//Cell_t *path_escape_get_trapped_cell()
//{
//	return g_homes;
//}

void path_set_home(const Cell_t& curr)
{
	bool is_found = false;

	for (const auto& it : g_homes) {
		ROS_INFO("%s %d: curr\033[33m(%d, %d)\033[0m home_it\033[33m(%d,%d)\033[0m.", __FUNCTION__, __LINE__, curr.X, curr.Y,it.X,it.Y);
		if (it == curr) {
			is_found = true;
			break;
		}
	}
	if (!is_found) {
		ROS_INFO("%s %d: Push new reachable home:\033[33m (%d, %d)\033[0m to home point list.", __FUNCTION__, __LINE__, curr.X, curr.Y);
		if(cm_get() != Clean_Mode_Spot)
			g_have_seen_charger = true;
		// If curr near (0, 0)
		if (abs(curr.X) >= 5 || abs(curr.Y) >= 5)
		{
			if(g_homes.size() >= ESCAPE_TRAPPED_REF_CELL_SIZE+1)//escape_count + zero_home = 3+1 = 4
			{
				std::copy(g_homes.begin() + 2, g_homes.end(), g_homes.begin()+1);//shift 1 but save zero_home
				g_homes.pop_back();
			}
			g_homes.push_back(curr);
		}
	}
	else if(curr == g_zero_home && cm_get() != Clean_Mode_Spot)
	{
		g_start_point_seen_charger = true;
		g_have_seen_charger = true;
	}
}

/* Get next point and home point.
 * return :NO_TARGET_LEFT (0)
 *        :TARGET_FOUND (1)
 */
bool path_get_home_point_target(const Cell_t &curr, PPTargetType &path) {
	if(g_home_way_list.empty()) {
		g_home_way_it = _gen_home_ways(g_homes.size(), g_home_way_list);
		auto map_tmp = cost_map.generateBound();
		for (const auto &cell : map_tmp) {
			if (cost_map.get_cell(MAP, cell.X, cell.Y) == BLOCKED_RCON)
				cost_map.set_cell(MAP, cost_map.cell_to_count(cell.X), cost_map.cell_to_count(cell.Y), UNCLEAN);
		}
	}
	for (; g_home_way_it != g_home_way_list.end(); ++g_home_way_it) {
		auto way = *g_home_way_it % HOMEWAY_NUM;
		auto cnt = *g_home_way_it / HOMEWAY_NUM;
		g_home_point = g_homes[cnt];
		ROS_INFO("\033[1;46;37m" "%s,%d:g_home_point(%d, %d, %d), way(%d), cnt(%d) " "\033[0m", __FUNCTION__, __LINE__, g_home_point.X, g_home_point.Y, g_home_point.TH, way, cnt);
		if (way == USE_ROS && g_home_gen_rosmap) {
			g_home_gen_rosmap = false;
			auto map_bound = cost_map.generateBound();
			ROS_INFO("\033[1;46;37m" "%s,%d:ros_cost_map.convert" "\033[0m", __FUNCTION__, __LINE__);
			ros_map.reset(MAP);
//			map.print(MAP, 0, 0);
			ros_map.ros_convert(MAP, false, true, false);
//			map.print(MAP, 0, 0);
			ROS_INFO("\033[1;46;37m" "%s,%d:ros_map" "\033[0m", __FUNCTION__, __LINE__);
//			map.print(ROSMAP, 0, 0);
			for (const auto &cell : map_bound){
				auto rm_status = ros_map.get_cell(MAP, cell.X, cell.Y);
				auto m_status = cost_map.get_cell(MAP, cell.X, cell.Y);
//				ROS_INFO("\033[1;46;37m" "%s,%d:cell_it(%d,%d), rms(%d),ms(%d)" "\033[0m", __FUNCTION__, __LINE__,cell.X, cell.Y, rm_status, m_status);
				if ((m_status == BLOCKED_BUMPER || m_status == BLOCKED_LASER) && rm_status == CLEANED){
					ROS_WARN("%s,%d:cell_it(%d,%d), rms(%d),ms(%d)", __FUNCTION__, __LINE__,cell.X, cell.Y, rm_status, m_status);
					cost_map.set_cell(MAP, cost_map.cell_to_count(cell.X), cost_map.cell_to_count(cell.Y), CLEANED);
				}
			}
		}

		if (path_next_shortest(curr, g_home_point, path) == 1) {
			return true;
		}
	}
	ROS_INFO("%s %d: No more home targets.", __FUNCTION__, __LINE__);
	return false;
}

int16_t path_get_home_x()
{
	return g_home_x;
}

int16_t path_get_home_y()
{
	return g_home_y;
}

void wf_path_planning_initialize()
{
	int16_t i;


	/* Initialize the default settings. */
//	preset_action_count = 0;

//	weight_enabled = 1;

#if (ROBOT_SIZE == 5)

//	g_weight_cnt_threshold = 4;

#else

//	g_weight_cnt_threshold = 3;

#endif

	g_direct_go = 0;

//	g_cell_history[0] = {0,0};

	/* Reset the poisition list. */
/*	for (i = 0; i < 3; i++) {
		g_cell_history[i]= {int16_t(i + 1), int16_t(i + 1)};
	}*/

	/* Initialize the shortest path. */
	path_position_init(g_direct_go);
}

void path_set_continue_cell(Cell_t cell)
{
	g_continue_cell = cell;
	ROS_INFO("%s %d: Set continue cell: (%d, %d).", __FUNCTION__, __LINE__, g_continue_cell.X, g_continue_cell.Y);
}

bool path_get_continue_target(const Cell_t& curr, PPTargetType& path)
{
	int8_t return_val;

	ROS_WARN("%s %d: Need to resume cleaning, continue cell(%d, %d).", __FUNCTION__, __LINE__, g_continue_cell.X, g_continue_cell.Y);
	if (curr == g_continue_cell)
	{
		return true;
	}

	if (cost_map.is_block_accessible(g_continue_cell.X, g_continue_cell.Y) == 0) {
		ROS_WARN("%s %d: target(%d, %d) is blocked, unblock the target.\n", __FUNCTION__, __LINE__, g_continue_cell.X, g_continue_cell.Y);
		cost_map.set_cells(ROBOT_SIZE, g_continue_cell.X, g_continue_cell.Y, CLEANED);
	}

	auto path_next_status = (int8_t) path_next_shortest(curr, g_continue_cell, path);
//	ROS_INFO("%s %d: Path Find: %d\tNext point: (%d, %d)\tNow: (%d, %d)", __FUNCTION__, __LINE__, path_next_status, path.cells.front().X, path.cells.front().Y, curr.X, curr.Y);
	if (path_next_status == 1/* && !cm_check_loop_back(next)*/)
		return_val = true;
	else
		return_val = false;

	return return_val;
}

int16_t isolate_target(const Cell_t& curr, PPTargetType& path) {
	int16_t ret;
	//extern int g_wf_reach_count;
	//extern bool ev.fatal_quit;
	//if (g_wf_reach_count <= 3) {
		auto angle = -900;
	Cell_t cell;
		const float	FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
	auto point = CostMap::get_curr_point();
	point.TH = ranged_angle(robot::instance()->getPoseAngle() + angle);
	cost_map.robot_to_cell(point, 0, FIND_WALL_DISTANCE * 1000, cell.X, cell.Y);
		path.clear();
	path.push_front(cell);
		path.push_front(curr);
		ret = 1;
		ROS_INFO("target.X = %d target.Y = %d", path.back().X, path.back().Y);
	//} else {
	//	ev.fatal_quit = true;
	//	ret = 0;
	//}
	return ret;
}
//int roscost_map.get_cleaned_area(const Cell_t& curr)
//{
//	typedef std::multimap<double, Cell_t> Queue;
//	typedef std::pair<double, Cell_t> Entry;
//	int cleaned_count = 0;
//	cost_map.set_cell(SPMAP,curr.X,curr.Y,COST_2);
//	Queue queue;
//	Entry startPoint(0.0, curr);
//	queue.insert(startPoint);
//	bool is_found = false;
////	debug_map(MAP,curr.X, curr.Y);
//	ROS_INFO("cleaned area: Do full search with weightless Dijkstra-Algorithm\n");
//	while (!queue.empty())
//	{
////		 Get the nearest next from the queue
//		auto start = queue.begin();
//		auto next = start->second;
//		queue.erase(start);
//
//		{
//			for (auto it = 0; it < 8; it++)
//			{
//				auto neighbor = next + g_index[it];
////				ROS_INFO("g_index[%d],next(%d,%d)", it, neighbor.X,neighbor.Y);
//				auto spcost = cost_map.get_cell(SPMAP, neighbor.X, neighbor.Y);
//				if (spcost == COST_1 || spcost == COST_NO) {
//					if (cost_map.get_cell(MAP, neighbor.X, neighbor.Y) == CLEANED) {
//						cleaned_count++;
//						ROS_WARN("cleaned_count(%d),neighbor(%d,%d)", cleaned_count, neighbor.X, neighbor.Y);
//					}
//					cost_map.set_cell(SPMAP, neighbor.X, neighbor.Y, COST_2);
//					if(spcost == COST_1)
//					{
//						ROS_WARN("add to Queue:(%d,%d)", neighbor.X, neighbor.Y);
//						queue.insert(Entry(0, neighbor));
//					}
//				}
//			}
//		}
//	}
//	return cleaned_count;
//}

bool path_dijkstra(const Cell_t& curr, Cell_t& target,int& cleaned_count)
{
	typedef std::multimap<double, Cell_t> Queue;
	typedef std::pair<double, Cell_t> Entry;

	cost_map.reset(SPMAP);

	cost_map.set_cell(SPMAP,curr.X,curr.Y,COST_1);
	Queue queue;
	Entry startPoint(0.0, curr);
	cleaned_count = 1;
	queue.insert(startPoint);
	bool is_found = false;
//	map.print(MAP,curr.X, curr.Y);
	ROS_INFO("Do full search with weightless Dijkstra-Algorithm\n");
	while (!queue.empty())
	{
//		 Get the nearest next from the queue
		auto start = queue.begin();
		auto next = start->second;
		queue.erase(start);

//		ROS_WARN("adjacent cell(%d,%d)", next.X, next.Y);
		if (cost_map.get_cell(MAP, next.X,next.Y) == UNCLEAN && !cost_map.is_block_cleaned_unblock(next.X, next.Y))
		{
			ROS_WARN("We find the Unclean next(%d,%d)", next.X, next.Y);
			is_found = true;
			target = next;
			break;
		} else
		{
			for (auto it = 0; it < 4; it++)
			{
				auto neighbor = next + g_index[it];
//				ROS_INFO("g_index[%d],next(%d,%d)", it, neighbor.X,neighbor.Y);
				if (cost_map.get_cell(SPMAP, neighbor.X, neighbor.Y) != COST_1) {
//					ROS_INFO("(%d,%d),", neighbor.X, neighbor.Y);

					for (auto it_ = 0; it_ < 9; it_++) {
						auto neighbor_ = neighbor + g_index[it_];
						if (cost_map.get_cell(MAP, neighbor_.X, neighbor_.Y) == CLEANED && \
						    cost_map.get_cell(SPMAP, neighbor_.X, neighbor_.Y) == COST_NO) {
							cleaned_count++;
							cost_map.set_cell(SPMAP, neighbor_.X, neighbor_.Y, COST_2);
//							ROS_INFO("(%d,%d, cleaned_count(%d)),", neighbor_.X, neighbor_.Y, cleaned_count);
						}
					}

					if (cost_map.is_block_accessible(neighbor.X, neighbor.Y)) {
//					ROS_WARN("add to Queue:(%d,%d)", neighbor.X, neighbor.Y);
						queue.insert(Entry(0, neighbor));
						cost_map.set_cell(SPMAP, neighbor.X, neighbor.Y, COST_1);
					}
				}
			}
		}
	}
//	cleaned_count = roscost_map.get_cleaned_area(curr);
	return is_found;
}

bool is_fobbit_free() {
	//NOTE: g_home_way_it should last of g_home_point,for g_homeway_list may empty.
	return (cs.is_going_home() && *g_home_way_it % HOMEWAY_NUM == USE_CLEANED);
}

bool fw_is_time_up()
{
	return ((uint32_t)difftime(time(NULL), g_wf_start_timer)) > g_wf_diff_timer;
}
