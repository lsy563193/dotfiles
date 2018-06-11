//
// Created by lsy563193 on 12/13/17.
//

#include <map.h>
#include "ros/ros.h"
#include "path_algorithm.h"

GoHomePathAlgorithm::GoHomePathAlgorithm(GridMap& map, HomePoints_t* p_home_points, HomePoints_t* p_start_points)
{
//	p_home_points_ = p_home_points;
//	p_start_points_ = p_start_points;
	ROS_INFO("%s,%d: Clear 8 cells around start and home point.",__FUNCTION__,__LINE__);
	if(!p_home_points->empty())
	{
		home_points_set.push_back(p_home_points);
		for (auto &&pointsSet : home_points_set) {
			ROS_INFO("pointsSet(%d,%d)",pointsSet->begin()->x,pointsSet->begin()->y);
		}
	}

	home_points_set.push_back(p_start_points);

	for(auto && p_home_points_it : home_points_set)
	{
//		ROS_INFO("!!!!%s,%d: Clear 8 cells around start and home point.",__FUNCTION__,__LINE__);
		for(auto &&it = p_home_points_it->begin();it != p_home_points_it->end(); ++it)
		{
//			ROS_INFO("set Area Clean,%d,%d",it->toCell().x, it->toCell().y);
			map.setArea(it->toCell(), CLEANED, 1, 1);
		}
	}
//	ROS_INFO("%s,%d: set the rcon c_blocks to cleaned.",__FUNCTION__,__LINE__);
	auto map_tmp = map.generateBound();
	for (const auto &cell : map_tmp) {
		if(map.getCell(CLEAN_MAP,cell.x,cell.y) == BLOCKED_TMP_RCON
					|| map.getCell(CLEAN_MAP, cell.x, cell.y) == BLOCKED_RCON)
			map.setCell(CLEAN_MAP,cell.x,cell.y, UNCLEAN);
	}
	ROS_INFO("%s,%d: push back home way.",__FUNCTION__,__LINE__);
	home_ways.push_back(make_unique<GoHomeWay_t>("MapThroughAccessableAndCleaned", ThroughAccessableAndCleaned(&map)));
	home_ways.push_back(make_unique<GoHomeWay_t>("MapCleanBlockThroughAccessableAndCleaned", ThroughAccessableAndCleaned(&map),false, true));
	home_ways.push_back(make_unique<GoHomeWay_t>("SlamMapThroughAccessable", ThroughBlockAccessable(&map),true));
	home_ways.push_back(make_unique<GoHomeWay_t>("MapThroughAccessable", ThroughBlockAccessable(&map)));
	curr_way_ = home_ways.begin();
	curr_home_points = home_points_set.begin();
	curr_home_point_ = (*curr_home_points)->begin();
	ROS_INFO("%s,%d: init finish.",__FUNCTION__,__LINE__);
}

bool GoHomePathAlgorithm::generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path)
{
	plan_path.clear();
	Cells plan_path_cells{};
	map.print(curr.toCell(),CLEAN_MAP,Cells{});
	for(;curr_home_points != home_points_set.end(); ++curr_home_points) {
		ROS_INFO("%s,%d:go home point start home or rcon point" ,__FUNCTION__, __LINE__);
		if(curr_way_ == home_ways.end())
			curr_way_ = home_ways.begin();

		for (; curr_way_ != home_ways.end(); ++curr_way_) {

			curr_way_->get()->displayName();
			if(curr_way_->get()->isClearBlock())
				curr_way_->get()->clearBlock(map);

			if(curr_home_point_ == (*curr_home_points)->end())
				curr_home_point_ = (*curr_home_points)->begin();

			for (; curr_home_point_ != (*curr_home_points)->end(); ++curr_home_point_) {
				ROS_INFO("curr_home_point_(%d,%d)", curr_home_point_->toCell().x, curr_home_point_->toCell().y);
				auto tmp_map = curr_way_->get()->updateMap(map,curr);
				if (dijkstra(*tmp_map, curr.toCell(), plan_path_cells, true, CellEqual(curr_home_point_->toCell()),
							 isAccessable(tmp_map.get(), curr_way_->get()->expand_condition))) {
					optimizePath(map, plan_path_cells, last_dir);
					plan_path = *cells_to_points(plan_path_cells);
					ROS_ERROR("5555curr_home_points");
					tmp_map->print(curr.toCell(), COST_MAP, plan_path_cells);
					tmp_map->print(curr.toCell(), CLEAN_MAP, plan_path_cells);
					return true;
				}
				tmp_map->print(curr.toCell(), COST_MAP, plan_path_cells);
				tmp_map->print(curr.toCell(), CLEAN_MAP, plan_path_cells);
			}
		}
	}

	return !plan_path.empty();
}

//void FollowWallModeGoHomePathAlgorithm::getNextWay(GridMap &map, const Point_t &curr)
//{
//	switch (home_way_index_)
//	{
//		case GoHomeWay_t::THROUGH_SLAM_MAP_REACHABLE_AREA:
//			home_way_index_ = GoHomeWay_t::SLAM_MAP_CLEAR_BLOCKS;
//			slam_grid_map.print(curr.toCell(), CLEAN_MAP, Cells{{0, 0}});
//			map.merge(slam_grid_map, false, false, false, false, false, true);
//			map.print(curr.toCell(), CLEAN_MAP, Cells{cur.toCell()});
//			ROS_INFO("%s %d: Clear c_blocks with slam map.", __FUNCTION__, __LINE__);
//			break;
//		case GoHomeWay_t::SLAM_MAP_CLEAR_BLOCKS:
//			home_way_index_ = GoHomeWay_t::GO_HOME_WAY_NUM;
//			break;
//		default: // GoHomeWay_t::GO_HOME_WAY_NUM
//			home_way_index_ = GoHomeWay_t::THROUGH_SLAM_MAP_REACHABLE_AREA;
//			ROS_INFO("%s %d: Use slam map reachable area.", __FUNCTION__, __LINE__);
//			break;
//	}
//}
