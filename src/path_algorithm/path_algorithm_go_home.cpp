//
// Created by lsy563193 on 12/13/17.
//

#include <map.h>
#include "ros/ros.h"
#include "path_algorithm.h"

bool GoHomePathAlgorithm::generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path)
{
	bool generate_finish = false;
	plan_path.clear();
	Cells plan_path_cells{};
	map.print(curr.toCell(),CLEAN_MAP,Cells{});
	while (!generate_finish && ros::ok())
	{
		switch (home_way_index_)
		{
			case GoHomeWay_t::THROUGH_CLEANED_AREA:
				generate_finish = generatePathThroughCleanedArea(map, curr, last_dir, plan_path_cells);
				plan_path = handleResult(generate_finish, plan_path_cells, curr, map);

				if (!generate_finish)
				{
					home_way_index_ = GoHomeWay_t::SLAM_MAP_CLEAR_BLOCKS;
					ROS_INFO("%s %d: Clear c_blocks with slam map.", __FUNCTION__, __LINE__);
					slam_grid_map.print(curr.toCell(), CLEAN_MAP, Cells{{0, 0}});
					map.merge(slam_grid_map, false, false, false, false, false, true);
					map.print(curr.toCell(), CLEAN_MAP, Cells{curr.toCell()});
				}
				break;
			case GoHomeWay_t::SLAM_MAP_CLEAR_BLOCKS:
				generate_finish = generatePathWithSlamMapClearBlocks(map, curr, last_dir, plan_path_cells);
				plan_path = handleResult(generate_finish, plan_path_cells, curr, map);

				if (!generate_finish)
				{
					home_way_index_ = GoHomeWay_t::THROUGH_SLAM_MAP_REACHABLE_AREA;
					ROS_INFO("%s %d: Use slam map reachable area.", __FUNCTION__, __LINE__);
				}
				break;
			case GoHomeWay_t::THROUGH_SLAM_MAP_REACHABLE_AREA:
				generate_finish = generatePathThroughSlamMapReachableArea(map, curr, last_dir, plan_path_cells);
				plan_path = handleResult(generate_finish, plan_path_cells, curr, map);

				if (!generate_finish)
				{
					home_way_index_ = GoHomeWay_t::THROUGH_UNKNOWN_AREA;
					ROS_INFO("%s %d: Use unknown area.", __FUNCTION__, __LINE__);
				}
				break;
			case GoHomeWay_t::THROUGH_UNKNOWN_AREA:
				generate_finish = generatePathThroughUnknownArea(map, curr, last_dir, plan_path_cells);
				plan_path = handleResult(generate_finish, plan_path_cells, curr, map);

				if (!generate_finish)
					home_way_index_ = GoHomeWay_t::GO_HOME_WAY_NUM;
				break;
			default: //GO_HOME_WAY_NUM
			{
				generate_finish = switchHomePoint();
				break;
			}
		}
	}
	ROS_INFO("%s %d:",__FUNCTION__, __LINE__);
	return !plan_path.empty();
}

bool GoHomePathAlgorithm::generatePathThroughCleanedArea(GridMap &map, const Point_t &curr, Dir_t last_dir,
														 Cells &plan_path)
{
	if (map.isBlockAccessible(current_home_point_.toCell().x, current_home_point_.toCell().y))
	{
//		Cell_t min_corner, max_corner;
//		plan_path = findShortestPath(map, curr.toCell(), current_home_point_.toCell(),
//										   last_dir, false, false, min_corner, max_corner);
		Cells cells{};
		auto is_found = map.dijstra(curr.toCell(), cells,[&](const Cell_t& c_it){return c_it == current_home_point_.toCell();},true);
		if(is_found)
			findPath(map, curr.toCell(),current_home_point_.toCell(),plan_path,last_dir);
	}

	ROS_WARN(
			"\033[1;46;37m" "%s,%d: Current_home_point_(%d, %d) %sreachable in this way, total %d home points." "\033[0m",
			__FUNCTION__, __LINE__, current_home_point_.toCell().x, current_home_point_.toCell().y,
			!plan_path.empty() ? "" : "NOT ", home_points_.size());
	return !plan_path.empty();
}

bool GoHomePathAlgorithm::generatePathWithSlamMapClearBlocks(GridMap &map, const Point_t &curr, Dir_t last_dir,
														Cells &plan_path)
{
	if (map.isBlockAccessible(current_home_point_.toCell().x, current_home_point_.toCell().y))
	{
		Cell_t min_corner, max_corner;
		Cells cells{};
		auto is_found = map.dijstra(curr.toCell(), cells,[&](const Cell_t& c_it){return c_it == current_home_point_.toCell();},true);
		if(is_found)
			findPath(map, curr.toCell(),current_home_point_.toCell(),plan_path,last_dir);
	}

	ROS_WARN(
			"\033[1;46;37m" "%s,%d: Current_home_point_(%d, %d) %sreachable in this way, total %d home points." "\033[0m",
			__FUNCTION__, __LINE__, current_home_point_.toCell().x, current_home_point_.toCell().y,
			!plan_path.empty() ? "" : "NOT ", home_points_.size());
	return !plan_path.empty();
}

bool GoHomePathAlgorithm::generatePathThroughSlamMapReachableArea(GridMap &map, const Point_t &curr,
																  Dir_t last_dir, Cells &plan_path)
{
	if (map.isBlockAccessible(current_home_point_.toCell().x, current_home_point_.toCell().y))
	{
		Cell_t min_corner, max_corner;
		GridMap temp_map;
		temp_map.copy(map);
		temp_map.merge(slam_grid_map, false, false, true, false, false, false);
		temp_map.print(curr.toCell(), CLEAN_MAP, Cells{{0, 0}});
		Cells cells{};
		auto is_found = map.dijstra(curr.toCell(), cells,[&](const Cell_t& c_it){return c_it == current_home_point_.toCell();},true);
		if(is_found)
			findPath(map, curr.toCell(),current_home_point_.toCell(),plan_path,last_dir);
	}

	ROS_WARN(
			"\033[1;46;37m" "%s,%d: Current_home_point_(%d, %d) %sreachable in this way, total %d home points." "\033[0m",
			__FUNCTION__, __LINE__, current_home_point_.toCell().x, current_home_point_.toCell().y,
			!plan_path.empty() ? "" : "NOT ", home_points_.size());
	return !plan_path.empty();
}

bool GoHomePathAlgorithm::generatePathThroughUnknownArea(GridMap &map, const Point_t &curr, Dir_t last_dir,
														 Cells &plan_path)
{
	if (map.isBlockAccessible(current_home_point_.toCell().x, current_home_point_.toCell().y))
	{
		Cell_t min_corner, max_corner;
		Cells cells{};
		auto is_found = map.dijstra(curr.toCell(), cells,[&](const Cell_t& c_it){return c_it == current_home_point_.toCell();},true);
		if(is_found)
			findPath(map, curr.toCell(),current_home_point_.toCell(),plan_path,last_dir);
	}

	ROS_WARN(
			"\033[1;46;37m" "%s,%d: Current_home_point_(%d, %d) %sreachable in this way, total %d home points." "\033[0m",
			__FUNCTION__, __LINE__, current_home_point_.toCell().x, current_home_point_.toCell().y,
			!plan_path.empty() ? "" : "NOT ", home_points_.size());
	return !plan_path.empty();
}

bool GoHomePathAlgorithm::reachTarget(bool &should_go_to_charger, Point_t curr)
{
	bool ret = false;

	if (!home_points_.empty())
	{
//		ROS_ERROR("!!!!!!!!!!!!!home_points_(%d)", home_points_.size());
		if (curr.toCell() == getCurrentHomePoint().toCell())
		{
			ROS_INFO("%s %d: Reach home point(%d, %d).",
					 __FUNCTION__, __LINE__, getCurrentHomePoint().toCell().x, getCurrentHomePoint().toCell().y);
			// Erase home point but do NOT change the index.
			eraseCurrentHomePoint();
			// Reset current home point.

			if (!home_points_.empty())
			{
				current_home_point_ = home_points_.front();
				ROS_WARN("%s %d: Current_home_point_ set to home point(%d, %d), total %d home points.",
						 __FUNCTION__, __LINE__, current_home_point_.toCell().x, current_home_point_.toCell().y,
						 home_points_.size());
			} else
			{
				current_home_point_ = start_point_;
				ROS_WARN("%s %d: Current_home_point_ set to start point(%d, %d), total %d home points.",
						 __FUNCTION__, __LINE__, current_home_point_.toCell().x, current_home_point_.toCell().y,
						 home_points_.size());
			}

			std::string msg = "Home_points_: ";
			for (auto it : home_points_)
				msg += "(" + std::to_string(it.toCell().x) + ", " + std::to_string(it.toCell().y) + "),";
			ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());

			ret = true;
			should_go_to_charger = true;
		}
	}
	else {
		ROS_INFO("%s %d: home_points_ empty curr (%d,%d,%f), start(%d,%d,%f)", __FUNCTION__, __LINE__, curr.toCell().x,
							curr.toCell().y, radian_to_degree(curr.th), start_point_.toCell().x,
							start_point_.toCell().y, radian_to_degree(start_point_.th));
		if (curr.isCellAndAngleEqual(start_point_)) {
			ROS_INFO("%s %d: Reach start point with angle.(%d, %d, %f).",
							 __FUNCTION__, __LINE__, start_point_.toCell().x, start_point_.toCell().y, start_point_.th);
			ret = true;
			should_go_to_charger = false;
		}
	}

	return ret;
}

Point_t GoHomePathAlgorithm::getCurrentHomePoint()
{
	return current_home_point_;
}

Points GoHomePathAlgorithm::getRestHomePoints()
{
	return home_points_;
}

bool GoHomePathAlgorithm::eraseCurrentHomePoint()
{
	if (!home_points_.empty())
	{
		ROS_WARN("%s %d: Erase this home point(%d, %d).",
				 __FUNCTION__, __LINE__, home_points_.front().toCell().x, home_points_.front().toCell().y);
		home_points_.pop_front();
	} else
		ROS_ERROR("%s %d: CAN NOT ERASE ANYTHING..", __FUNCTION__, __LINE__);

	return true;
}

void GoHomePathAlgorithm::setHomePoint(Point_t current_point)
{
	// Set home cell.
	Points::iterator home_point_it = home_points_.begin();
	for (;home_point_it != home_points_.end(); home_point_it++)
	{
		if (home_point_it->toCell() == current_point.toCell())
		{
			ROS_INFO("%s %d: Home point(%d, %d) exists.",
					 __FUNCTION__, __LINE__, home_point_it->toCell().x, home_point_it->toCell().y);

			return;
		}
	}

	while(ros::ok() && home_points_.size() >= (uint32_t)HOME_POINTS_SIZE && (home_points_.size() >= 1))
		// Drop the oldest home point to keep the home_points_.size() is within HOME_POINTS_SIZE.
		home_points_.pop_back();

	home_points_.push_front(current_point);
	std::string msg = "Update Home_points_: ";
	for (auto it : home_points_)
		msg += "(" + std::to_string(it.toCell().x) + ", " + std::to_string(it.toCell().y) + "),";
	ROS_WARN("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());
}

void GoHomePathAlgorithm::updateStartPointRadian(double radian)
{
	start_point_.th = radian;
	ROS_WARN("%s %d: Start point radian update to %f(%f in degree).", __FUNCTION__, __LINE__, start_point_.th,
			 radian_to_degree(start_point_.th));
}

void GoHomePathAlgorithm::initForGoHomePoint(GridMap &map)
{
	if (home_points_.empty())
		ROS_WARN("%s %d: No home points, just go to start point.", __FUNCTION__, __LINE__);
	else
	{
		std::string msg = "Home_points_: ";
		for (auto it : home_points_)
		{
			msg += "(" + std::to_string(it.toCell().x) + ", " + std::to_string(it.toCell().y) + "),";
			// Clear 8 cells around home points.
			map.setArea(it.toCell(), CLEANED, 1, 1);
		}
		ROS_WARN("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());
	}

	// Clear 8 cells around start point.
	map.setArea(start_point_.toCell(), CLEANED, 1, 1);

	// set the rcon c_blocks to cleaned
	auto map_tmp = map.generateBound();
	for (const auto &cell : map_tmp) {
		if(map.getCell(CLEAN_MAP,cell.x,cell.y) == BLOCKED_TMP_RCON
					|| map.getCell(CLEAN_MAP, cell.x, cell.y) == BLOCKED_RCON)
			map.setCell(CLEAN_MAP,cell.x,cell.y, UNCLEAN);
	}

	home_way_index_ = GO_HOME_WAY_NUM;
	current_home_point_ = invalid_point_;
	back_to_start_point_ = false;
}

bool GoHomePathAlgorithm::switchHomePoint()
{
	bool generate_finish{false};
	if (!home_points_.empty() && current_home_point_ != invalid_point_)
		eraseCurrentHomePoint();

	if (!home_points_.empty())
	{
		current_home_point_ = home_points_.front();
		ROS_WARN("%s %d: Current_home_point_ set to home point(%d, %d), total %d home points.",
				 __FUNCTION__, __LINE__, current_home_point_.toCell().x, current_home_point_.toCell().y,
				 home_points_.size());
	} else
	{
		current_home_point_ = start_point_;
		ROS_WARN("%s %d: Current_home_point_ set to start point(%d, %d), total %d home points.",
				 __FUNCTION__, __LINE__, current_home_point_.toCell().x, current_home_point_.toCell().y,
				 home_points_.size());
	}

	home_way_index_ = GoHomeWay_t::THROUGH_CLEANED_AREA;

	if (current_home_point_ == start_point_)
	{
		if (!back_to_start_point_)
		{
			ROS_WARN("%s %d: Start going back to start point.", __FUNCTION__, __LINE__);
			back_to_start_point_ = true;
		} else
		{
			// Now it means robot can not go back to start point after trying all those methods.
			ROS_WARN("%s %d: CAN NOT go back to start point.", __FUNCTION__, __LINE__);
			generate_finish = true;
		}
	}

	return generate_finish;
}

Points GoHomePathAlgorithm::handleResult(bool generate_finish, Cells plan_path_cells, Point_t curr, GridMap &map)
{
	Points plan_path{};

	if (generate_finish)
	{
		plan_path = *cells_generate_points(make_unique<Cells>(plan_path_cells));
		map.print(curr.toCell(), CLEAN_MAP, plan_path_cells);
	}

	return plan_path;
}
