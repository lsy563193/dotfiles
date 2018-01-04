//
// Created by lsy563193 on 12/13/17.
//

#include <pp.h>
#include <map.h>
#include "ros/ros.h"
#include "path_algorithm.h"

//----------
GoHomePathAlgorithm::GoHomePathAlgorithm(GridMap &map, HomePoints home_points)
{
	// Save the home_points to local.
	switch (home_points.size())
	{
		case 0:
		{
			ROS_ERROR("%s,%d: input home_points are empty,at least it should has a home cell(0, 0)!! "
							  "Now set go_home_way_list_ as one home cell (0, 0).", __FUNCTION__, __LINE__);
			home_points_ = {{{0, 0, 0}, false}};
			go_home_way_list_ = {                                      9,10,11};
			ROS_INFO("%s,%d: go_home_way_list_ 1:                      9,10,11", __FUNCTION__, __LINE__);
			break;
		}
		case 1:
		{
			home_points_ = home_points;
			go_home_way_list_ = {                                      9,10,11};
			ROS_INFO("%s,%d: go_home_way_list_ 1:                      9,10,11", __FUNCTION__, __LINE__);
			break;
		}
		case 2:
		{
			home_points_ = home_points;
			go_home_way_list_ = {                 0,     1,     2,     9,10,11};
			ROS_INFO("%s,%d: go_home_way_list_ 2: 0,     1,     2,     9,10,11", __FUNCTION__, __LINE__);
			break;
		}
		case 3:
		{
			home_points_ = home_points;
			go_home_way_list_ = {                 0,3,   1,4,   2,5,   9,10,11};
			ROS_INFO("%s,%d: go_home_way_list_ 3: 0,3,   1,4,   2,5,   9,10,11", __FUNCTION__, __LINE__);
			break;
		}
		case 4:
		{
			home_points_ = home_points;
			go_home_way_list_ = {                 0,3,6, 1,4,7, 2,5,8, 9,10,11};
			ROS_INFO("%s,%d: go_home_way_list_ 4: 0,3,6, 1,4,7, 2,5,8, 9,10,11", __FUNCTION__, __LINE__);
			break;
		}
		default:
		{
			ROS_ERROR("%s,%d: input home_points more than 4, we just take the first 3 and add (0, 0).",
					  __FUNCTION__, __LINE__);
			auto it = home_points.begin();
			for (auto i = 0; i < 3; i++)
			{
				home_points_.push_back(*it);
				it++;
			}
			home_points_.push_back(home_points.back());
			go_home_way_list_ = {                 0,3,6, 1,4,7, 2,5,8, 9,10,11};
			ROS_INFO("%s,%d: go_home_way_list_ 4: 0,3,6, 1,4,7, 2,5,8, 9,10,11", __FUNCTION__, __LINE__);
			break;
		}
	}
	go_home_way_list_it_ = go_home_way_list_.begin();

	std::string msg = "Home_points_: ";
	for (auto it : home_points_)
		msg += "(" + std::to_string(it.home_point.toCell().x) + ", " + std::to_string(it.home_point.toCell().y) + "),";
	ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());

	// Clear the rcon blocks in map.
	auto map_tmp = map.generateBound();
	for (const auto &cell : map_tmp) {
		if (map.getCell(CLEAN_MAP, cell.x, cell.y) == BLOCKED_RCON)
			map.setCell(CLEAN_MAP,cell.x,cell.y, UNCLEAN);
	}

	// Copy the map data to local go_home_map_.
	go_home_map_.copy(map);
}

bool GoHomePathAlgorithm::generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &plan_path)
{
	auto curr_cell = curr.toCell();

	// Search path to home cells.
	for (; go_home_way_list_it_ != go_home_way_list_.end(); ++go_home_way_list_it_) {
		auto way = *go_home_way_list_it_ % GO_HOME_WAY_NUM;
		auto cnt = *go_home_way_list_it_ / GO_HOME_WAY_NUM;
		current_home_point_ = home_points_[cnt];
		ROS_INFO("\033[1;46;37m" "%s,%d:current_home_point_(%d, %d, %d), have_seen_charger?%d, way(%d), cnt(%d) " "\033[0m",
				 __FUNCTION__, __LINE__, current_home_point_.home_point.toCell().x, current_home_point_.home_point.toCell().y,
				 current_home_point_.home_point.th, current_home_point_.have_seen_charger, way, cnt);
		// Update go_home_map_.
		go_home_map_.copy(map);
		if (way == THROUGH_SLAM_MAP_REACHABLE_AREA) {
			// Using slam grid map to clear the bumper and laser blocks.
			go_home_map_.mergeFromSlamGridMap(slam_grid_map, false, false, false, false, false, true);
		}
		auto plan_path_cell = findShortestPath(go_home_map_, curr_cell, current_home_point_.home_point.toCell(), last_dir, true);

		if (!plan_path_cell.empty())
		{
			plan_path = cells_generate_points(plan_path_cell);
			go_home_map_.print(CLEAN_MAP, plan_path_cell.back().x, plan_path_cell.back().y);
			go_home_way_list_it_++;
			break;
		}
	}

	if (plan_path.empty())
	{
		ROS_INFO("%s %d: No more way to go home.", __FUNCTION__, __LINE__);
		return false;
	}

	return true;
}

HomePoint GoHomePathAlgorithm::getCurrentHomePoint()
{
	return current_home_point_;
}


//Cells GoHomePathAlgorithm::getRestHomeCells()
//{
//	// Return rest home cells.
//	return home_points_;
//}