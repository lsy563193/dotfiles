//
// Created by lsy563193 on 12/13/17.
//

#include <pp.h>
#include "ros/ros.h"
#include "path_algorithm.h"

//----------
GoHomePathAlgorithm::GoHomePathAlgorithm(GridMap &map, Points home_points)
{
	// Save the home_points to local.
	switch (home_points.size())
	{
		case 0:
		{
			ROS_ERROR("%s,%d: input home_points are empty,at least it should has a home cell(0, 0)!! "
							  "Now set go_home_way_list_ as one home cell (0, 0).", __FUNCTION__, __LINE__);
			home_points_ = {{0, 0,0}};
			go_home_way_list_ = {                                       2,1,0};
			ROS_INFO("%s,%d: go_home_way_list_ 1:                       2,1,0", __FUNCTION__, __LINE__);
			break;
		}
		case 1:
		{
			home_points_ = home_points;
			go_home_way_list_ = {                                       2,1,0};
			ROS_INFO("%s,%d: go_home_way_list_ 1:                       2,1,0", __FUNCTION__, __LINE__);
			break;
		}
		case 2:
		{
			home_points_ = home_points;
			go_home_way_list_ = {                 5,      4,     3,     2,1,0};
			ROS_INFO("%s,%d: go_home_way_list_ 2: 5,      4,     3,     2,1,0", __FUNCTION__, __LINE__);
			break;
		}
		case 3:
		{
			home_points_ = home_points;
			go_home_way_list_ = {                 5,8,    4,7,   3,6,   2,1,0};
			ROS_INFO("%s,%d: go_home_way_list_ 3: 5,8,    4,7,   3,6,   2,1,0", __FUNCTION__, __LINE__);
			break;
		}
		case 4:
		{
			home_points_ = home_points;
			go_home_way_list_ = {                 5,8,11, 4,7,10,3,6,9, 2,1,0};
			ROS_INFO("%s,%d: go_home_way_list_ 4: 5,8,11, 4,7,10,3,6,9, 2,1,0", __FUNCTION__, __LINE__);
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
			home_points_.push_back({0, 0, 0});
			go_home_way_list_ = {                 5,8,11, 4,7,10,3,6,9, 2,1,0};
			ROS_INFO("%s,%d: go_home_way_list_ 4: 5,8,11, 4,7,10,3,6,9, 2,1,0", __FUNCTION__, __LINE__);
			break;
		}
	}
	go_home_way_list_it_ = go_home_way_list_.begin();

	// Clear the rcon blocks in map.
	auto map_tmp = map.generateBound();
	for (const auto &cell : map_tmp) {
		if (map.getCell(CLEAN_MAP, cell.X, cell.Y) == BLOCKED_RCON)
			map.setCell(CLEAN_MAP,cell.X,cell.Y, UNCLEAN);
	}

	// Copy the map data to local go_home_map_.
	go_home_map_.copy(map);
}

bool GoHomePathAlgorithm::generatePath(GridMap &map, const Point32_t &curr, const MapDirection &last_dir, Points &plan_path)
{
	auto curr_cell = GridMap::pointToCell(curr);
	auto current_home_target_cell = GridMap::pointToCell(current_home_target_);
	ROS_INFO("%s %d: current_cell(%d, %d), Reach home cell(%d, %d)", __FUNCTION__ ,__LINE__,
			 curr_cell.X, curr_cell.Y, curr_cell, current_home_target_cell.X, current_home_target_cell.Y);
	if (curr_cell == current_home_target_cell/* && abs(curr_cell.TH -  current_home_target_.TH) < 50*/)
	{
		ROS_INFO("%s %d: Reach home cell(%d, %d)", __FUNCTION__ ,__LINE__, current_home_target_.X, current_home_target_.Y);
		// Congratulations! You have reached home.
		return true;
	}

	// Search path to home cells.
	for (; go_home_way_list_it_ != go_home_way_list_.end(); ++go_home_way_list_it_) {
		auto way = *go_home_way_list_it_ % GO_HOME_WAY_NUM;
		auto cnt = *go_home_way_list_it_ / GO_HOME_WAY_NUM;
		current_home_target_ = home_points_[cnt];
		ROS_INFO("\033[1;46;37m" "%s,%d:current_home_target_(%d, %d, %d), way(%d), cnt(%d) " "\033[0m",
				 __FUNCTION__, __LINE__, GridMap::countToCell(current_home_target_.X), GridMap::countToCell(current_home_target_.Y), current_home_target_.TH, way, cnt);
		// Update go_home_map_.
		go_home_map_.copy(map);
		if (way == THROUGH_SLAM_MAP_REACHABLE_AREA) {
			// Using slam grid map to clear the bumper and laser blocks.
			go_home_map_.mergeFromSlamGridMap(slam_grid_map, false, false, false, false, false, true);
		}

		auto plan_path_cell = findShortestPath(go_home_map_, curr_cell, GridMap::pointToCell(current_home_target_), last_dir, true);

		if (!plan_path_cell.empty())
		{
			ROS_INFO("%s %d", __FUNCTION__, __LINE__);
			plan_path = cells_generate_points(plan_path_cell);
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


//Cells GoHomePathAlgorithm::getRestHomeCells()
//{
//	// Return rest home cells.
//	return home_points_;
//}