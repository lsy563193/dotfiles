//
// Created by lsy563193 on 12/13/17.
//

#include <map.h>
#include "ros/ros.h"
#include "path_algorithm.h"

GoHomePathAlgorithm::GoHomePathAlgorithm(GridMap& map, HomePoints_t* p_home_points, HomePoints_t* p_start_points,bool is_follow_wall)
{
//	p_home_points_ = p_home_points;
//	p_start_points_ = p_start_points;
	if(!p_home_points->empty())
	{
		home_points_set.push_back(p_home_points);
//		for (auto &&pointsSet : *p_home_points) {
//			ROS_INFO("pointsSet(%d,%d)",pointsSet.toCell().x,pointsSet.toCell().y);
//		}
	}

	home_points_set.push_back(p_start_points);

	for(auto && p_home_points_it : home_points_set)
	{
		ROS_INFO("!!!!%s,%d: Clear 8 cells around start and home point.",__FUNCTION__,__LINE__);
		for(auto &&it = p_home_points_it->begin();it != p_home_points_it->end(); ++it)
		{
			ROS_WARN("set Area Clean,%d,%d",it->toCell().x, it->toCell().y);
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
	temp_map = make_unique<GridMap>();
	if(!is_follow_wall)
	{
		home_ways.push_back(make_unique<GoHomeWay_t>("MapThroughAccessableAndCleaned", ThroughAccessableAndCleaned(&map)));
		home_ways.push_back(make_unique<GoHomeWay_t>("MapCleanBlockThroughAccessableAndCleaned", ThroughAccessableAndCleaned(&map),false, true));
		home_ways.push_back(make_unique<GoHomeWay_t>("SlamMapThroughAccessable", ThroughBlockAccessable(temp_map.get()),true));
		home_ways.push_back(make_unique<GoHomeWay_t>("MapThroughAccessable", ThroughBlockAccessable(&map)));
	}else{
		home_ways.push_back(make_unique<GoHomeWay_t>("SlamMapThroughAccessable", ThroughBlockAccessable(temp_map.get()),true));
		home_ways.push_back(make_unique<GoHomeWay_t>("MapCleanBlockThroughAccessableAndCleaned", ThroughAccessableAndCleaned(&map),false, true));
	}
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
				auto p_tmp_map_ = curr_way_->get()->updateMap(map,temp_map, curr);

				if (dijkstra(*p_tmp_map_, curr.toCell(), plan_path_cells, true, CellEqual(curr_home_point_->toCell()),
							 isAccessable(p_tmp_map_, curr_way_->get()->expand_condition))) {

					optimizePath(*p_tmp_map_, plan_path_cells, last_dir, curr_way_->get()->expand_condition);
					plan_path = *cells_to_points(plan_path_cells);
					p_tmp_map_->print(curr.toCell(), COST_MAP, plan_path_cells);
					p_tmp_map_->print(curr.toCell(), CLEAN_MAP, plan_path_cells);
					return true;
				}
//				p_tmp_map_->print(curr.toCell(), COST_MAP, plan_path_cells);
//				p_tmp_map_->print(curr.toCell(), CLEAN_MAP, plan_path_cells);
			}
		}
	}

	return !plan_path.empty();
}
