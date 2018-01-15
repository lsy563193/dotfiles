//
// Created by lsy563193 on 12/13/17.
//

#include <pp.h>
#include <map.h>
#include "ros/ros.h"
#include "path_algorithm.h"

//----------
GoHomePathAlgorithm::GoHomePathAlgorithm(GridMap &map, Points &home_points, Point32_t start_point)
{
	// Save the home_points to local.
	home_points_ = home_points;

	if (home_points_.empty())
		ROS_INFO("%s %d: No home points, just go to start point.", __FUNCTION__, __LINE__);
	else
	{
		std::string msg = "Home_points_: ";
		for (auto it : home_points_)
			msg += "(" + std::to_string(it.toCell().x) + ", " + std::to_string(it.toCell().y) + "),";
		ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());
	}

	start_point_ = start_point;

	// Clear the rcon blocks in map.
	auto map_tmp = map.generateBound();
	for (const auto &cell : map_tmp) {
		if (map.getCell(CLEAN_MAP, cell.x, cell.y) == BLOCKED_RCON)
			map.setCell(CLEAN_MAP,cell.x,cell.y, UNCLEAN);
	}

	// For debug.
	std::string msg = "home point index: ";
	for (auto it : home_point_index_)
		msg += "(" + std::to_string(it) + "),";
	ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());
}

bool GoHomePathAlgorithm::generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &plan_path)
{
	bool generate_finish = false;
	while (!generate_finish && ros::ok())
	{
		switch (home_way_index_)
		{
			case THROUGH_SLAM_MAP_REACHABLE_AREA:
				generate_finish = generatePathThroughSlamMapReachableArea(map, curr, last_dir, plan_path);
				break;
			case THROUGH_UNKNOWN_AREA:
				generate_finish = generatePathThroughUnknownArea(map, curr, last_dir, plan_path);
				break;
			default: //case THROUGH_CLEANED_AREA:
				generate_finish = generatePathThroughCleanedArea(map, curr, last_dir, plan_path);
				break;
		}
	}

	return !plan_path.empty();

}

bool GoHomePathAlgorithm::generatePathThroughCleanedArea(GridMap &map, const Point32_t &curr, const int &last_dir,
														 Points &plan_path)
{
	Cells plan_path_cells{};
	auto &home_point_index = home_point_index_[THROUGH_CLEANED_AREA];

	if (!home_points_.empty())
	{
		if (home_point_index >= home_points_.size())
		{
			home_way_index_ = THROUGH_SLAM_MAP_REACHABLE_AREA;
			ROS_INFO("%s %d: Clear blocks with slam map.", __FUNCTION__, __LINE__);
			map.mergeFromSlamGridMap(slam_grid_map, false, false, false, false, false, true);
			map.print(CLEAN_MAP, curr.toCell().x, curr.toCell().y);
			return false;
		}

		current_home_point_ = home_points_[home_point_index];

		if (map.isBlockAccessible(current_home_point_.toCell().x, current_home_point_.toCell().y))
		{
			Cell_t min_corner, max_corner;
			plan_path_cells = findShortestPath(map, curr.toCell(), current_home_point_.toCell(),
											   last_dir, false, false, min_corner, max_corner);
		}

		if (!plan_path_cells.empty())
		{
			plan_path = cells_generate_points(plan_path_cells);
			ROS_INFO("\033[1;46;37m" "%s,%d: Index(%d), current_home_point_(%d, %d) reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, home_point_index,
					 current_home_point_.toCell().x, current_home_point_.toCell().y);
			map.print(CLEAN_MAP, plan_path_cells.back().x, plan_path_cells.back().y);
			return true;
		}
		else
		{
			// In this way, this home point is not reachable.
			ROS_INFO("\033[1;46;37m" "%s,%d: Index(%d), current_home_point_(%d, %d) NOT reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, home_point_index,
					 current_home_point_.toCell().x, current_home_point_.toCell().y);

			home_point_index++;
			return false;
		}
	}
	else // For going back to start point.
	{
		Cell_t min_corner, max_corner;
		plan_path_cells = findShortestPath(map, curr.toCell(), start_point_.toCell(),
										   last_dir, false, false, min_corner, max_corner);

		if (!plan_path_cells.empty())
		{
			plan_path = cells_generate_points(plan_path_cells);
			ROS_INFO("\033[1;46;37m" "%s,%d: start_point_(%d, %d) reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, start_point_.toCell().x, start_point_.toCell().y);
			map.print(CLEAN_MAP, plan_path_cells.back().x, plan_path_cells.back().y);
			return true;
		}
		else
		{
			// In this way, start point is not reachable.
			ROS_INFO("\033[1;46;37m" "%s,%d: start_point_(%d, %d) NOT reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, start_point_.toCell().x, start_point_.toCell().y);
			home_way_index_ = THROUGH_SLAM_MAP_REACHABLE_AREA;
			ROS_INFO("%s %d: Clear blocks with slam map.", __FUNCTION__, __LINE__);
			map.mergeFromSlamGridMap(slam_grid_map, false, false, false, false, false, true);
			map.print(CLEAN_MAP, curr.toCell().x, curr.toCell().y);
			return false;
		}
	}
}

bool
GoHomePathAlgorithm::generatePathThroughSlamMapReachableArea(GridMap &map, const Point32_t &curr, const int &last_dir,
															 Points &plan_path)
{
	Cells plan_path_cells{};
	auto &home_point_index = home_point_index_[THROUGH_CLEANED_AREA];

	if (!home_points_.empty())
	{
		if (home_point_index >= home_points_.size())
		{
			home_way_index_ = THROUGH_UNKNOWN_AREA;
			return false;
		}

		current_home_point_ = home_points_[home_point_index];

		if (map.isBlockAccessible(current_home_point_.toCell().x, current_home_point_.toCell().y))
		{
			Cell_t min_corner, max_corner;
			plan_path_cells = findShortestPath(map, curr.toCell(), current_home_point_.toCell(),
											   last_dir, false, false, min_corner, max_corner);
		}

		if (!plan_path_cells.empty())
		{
			plan_path = cells_generate_points(plan_path_cells);
			ROS_INFO("\033[1;46;37m" "%s,%d: Index(%d), current_home_point_(%d, %d) reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, home_point_index,
					 current_home_point_.toCell().x, current_home_point_.toCell().y);
			map.print(CLEAN_MAP, plan_path_cells.back().x, plan_path_cells.back().y);
			return true;
		}
		else
		{
			// In this way, this home point is not reachable.
			ROS_INFO("\033[1;46;37m" "%s,%d: Index(%d), current_home_point_(%d, %d) NOT reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, home_point_index,
					 current_home_point_.toCell().x, current_home_point_.toCell().y);

			home_point_index++;
			return false;
		}
	}
	else // For going back to start point.
	{
		Cell_t min_corner, max_corner;
		plan_path_cells = findShortestPath(map, curr.toCell(), start_point_.toCell(),
										   last_dir, false, false, min_corner, max_corner);

		if (!plan_path_cells.empty())
		{
			plan_path = cells_generate_points(plan_path_cells);
			ROS_INFO("\033[1;46;37m" "%s,%d: start_point_(%d, %d) reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, start_point_.toCell().x, start_point_.toCell().y);
			map.print(CLEAN_MAP, plan_path_cells.back().x, plan_path_cells.back().y);
			return true;
		}
		else
		{
			// In this way, start point is not reachable.
			ROS_INFO("\033[1;46;37m" "%s,%d: start_point_(%d, %d) NOT reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, start_point_.toCell().x, start_point_.toCell().y);
			home_way_index_ = THROUGH_UNKNOWN_AREA;
			return false;
		}
	}
}

bool GoHomePathAlgorithm::generatePathThroughUnknownArea(GridMap &map, const Point32_t &curr, const int &last_dir,
														 Points &plan_path)
{
	Cells plan_path_cells{};
	auto &home_point_index = home_point_index_[THROUGH_UNKNOWN_AREA];

	if (!home_points_.empty())
	{
		current_home_point_ = home_points_[home_point_index];

		if (map.isBlockAccessible(current_home_point_.toCell().x, current_home_point_.toCell().y))
		{
			Cell_t min_corner, max_corner;
			plan_path_cells = findShortestPath(map, curr.toCell(), current_home_point_.toCell(),
											   last_dir, true, false, min_corner, max_corner);
		}

		if (!plan_path_cells.empty())
		{
			plan_path = cells_generate_points(plan_path_cells);
			ROS_INFO("\033[1;46;37m" "%s,%d: Index(%d), current_home_point_(%d, %d) reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, home_point_index,
					 current_home_point_.toCell().x, current_home_point_.toCell().y);
			map.print(CLEAN_MAP, plan_path_cells.back().x, plan_path_cells.back().y);
			return true;
		}
		else
		{
			// In this way, this home point is not reachable.
			ROS_INFO("\033[1;46;37m" "%s,%d: Index(%d), current_home_point_(%d, %d) NOT reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, home_point_index,
					 current_home_point_.toCell().x, current_home_point_.toCell().y);

			// Drop this unreachable home point.
			eraseHomePoint(current_home_point_);
			home_point_index = 0;

			if (home_points_.empty()) // No more home points, go back to start point.
			{
				current_home_point_ = {CELL_COUNT_MUL * (MAP_SIZE + 1), CELL_COUNT_MUL * (MAP_SIZE + 1), 0};
				home_way_index_ = THROUGH_CLEANED_AREA;
				ROS_INFO("%s %d: No more reachable home points, just go back to start point.", __FUNCTION__, __LINE__);
			}

			return false;
		}
	}
	else // For going back to start point.
	{
		Cell_t min_corner, max_corner;
		plan_path_cells = findShortestPath(map, curr.toCell(), start_point_.toCell(),
										   last_dir, true, false, min_corner, max_corner);

		if (!plan_path_cells.empty())
		{
			plan_path = cells_generate_points(plan_path_cells);
			ROS_INFO("\033[1;46;37m" "%s,%d: start_point_(%d, %d) reachable in this way." "\033[0m",
					 __FUNCTION__, __LINE__, start_point_.toCell().x, start_point_.toCell().y);
			map.print(CLEAN_MAP, plan_path_cells.back().x, plan_path_cells.back().y);
			return true;
		}
		else
		{
			// In this way, start point is not reachable.
			ROS_INFO("\033[1;46;37m" "%s,%d: start_point_(%d, %d) NOT reachable in this way, exit state go home point." "\033[0m",
					 __FUNCTION__, __LINE__, start_point_.toCell().x, start_point_.toCell().y);
			return true;
		}
	}
}

bool GoHomePathAlgorithm::reachTarget(bool &should_go_to_charger)
{
	bool ret = false;

	if (!home_points_.empty())
	{
		if (getPosition().toCell() == getCurrentHomePoint().toCell())
		{
			ROS_INFO("%s %d: Reach home point(%d, %d).",
					 __FUNCTION__, __LINE__, getCurrentHomePoint().toCell().x, getCurrentHomePoint().toCell().y);
			// Erase home point but do NOT change the index.
			eraseHomePoint(getCurrentHomePoint());
			// Reset current home point.
			current_home_point_ = {CELL_COUNT_MUL * (MAP_SIZE + 1), CELL_COUNT_MUL * (MAP_SIZE + 1), 0};

			std::string msg = "Home_points_: ";
			for (auto it : home_points_)
				msg += "(" + std::to_string(it.toCell().x) + ", " + std::to_string(it.toCell().y) + "),";
			ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());

			ret = true;
			should_go_to_charger = true;
		}
	}
	else
	{
		if (getPosition().toCell() == start_point_.toCell())
		{
			ROS_INFO("%s %d: Reach start point(%d, %d).",
					 __FUNCTION__, __LINE__, start_point_.toCell().x, start_point_.toCell().y);
			ret = true;
			should_go_to_charger = false;
		}
	}

	return ret;
}

Point32_t GoHomePathAlgorithm::getCurrentHomePoint()
{
	return current_home_point_;
}

Points GoHomePathAlgorithm::getRestHomePoints()
{
	return home_points_;
}

bool GoHomePathAlgorithm::eraseHomePoint(Point32_t target_home_point)
{
	bool ret = false;
	Points::iterator home_point_it = home_points_.begin();
	for (;home_point_it != home_points_.end(); home_point_it++)
	{
		if (home_point_it->toCell() == target_home_point.toCell())
		{
			ROS_INFO("%s %d: Erase this home point(%d, %d).",
					 __FUNCTION__, __LINE__, home_point_it->toCell().x, home_point_it->toCell().y);
			home_points_.erase(home_point_it);
			ret = true;
			break;
		}
	}

	return ret;
}
