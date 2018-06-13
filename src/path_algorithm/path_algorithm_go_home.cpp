//
// Created by lsy563193 on 12/13/17.
//

#include <map.h>
#include "ros/ros.h"
#include "path_algorithm.h"

GoHomePathAlgorithm::GoHomePathAlgorithm(GridMap& map, HomePoints_t* p_home_points, HomePoints_t* p_start_points, HomePoints_t::iterator* p_home_point_it, bool is_follow_wall)
{
	if(!p_home_points->empty())
	{
		home_points_set.push_back(p_home_points);
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
	way_it = home_ways.begin();
	home_points_it = home_points_set.begin();
	p_home_point_it_ = p_home_point_it;
	*p_home_point_it_ = (*home_points_it)->begin();
	ROS_INFO("%s,%d: init finish.",__FUNCTION__,__LINE__);
}

bool GoHomePathAlgorithm::generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path)
{
	plan_path.clear();
	Cells plan_path_cells{};
	map.print(curr.toCell(),CLEAN_MAP,Cells{});
	for(;home_points_it != home_points_set.end(); ++home_points_it) {
		ROS_INFO("%s,%d:go home point start home or rcon point" ,__FUNCTION__, __LINE__);
		if(way_it == home_ways.end())
		{
			way_it = home_ways.begin();
			*p_home_point_it_ = (*home_points_it)->begin();
		}

		for (; way_it != home_ways.end(); ++way_it) {

			way_it->get()->displayName();
			if(way_it->get()->isClearBlock()&& !has_clean_)
			{
				has_clean_ = true;
				way_it->get()->clearBlock(map);
			}

			if(*p_home_point_it_ == (*home_points_it)->end())
				*p_home_point_it_ = (*home_points_it)->begin();

			for (; *p_home_point_it_ != (*home_points_it)->end(); ++(*p_home_point_it_)) {
				ROS_INFO("p_home_point_it_(%d,%d)", (*p_home_point_it_)->toCell().x, (*p_home_point_it_)->toCell().y);
				auto p_tmp_map_ = way_it->get()->updateMap(map,temp_map, curr);

				if (dijkstra(*p_tmp_map_, curr.toCell(), plan_path_cells, true, CellEqual((*p_home_point_it_)->toCell()),
							 isAccessable(p_tmp_map_, way_it->get()->expand_condition))) {

					optimizePath(*p_tmp_map_, plan_path_cells, last_dir, way_it->get()->expand_condition);
					plan_path = *cells_to_points(plan_path_cells);
					p_tmp_map_->print(curr.toCell(), COST_MAP, plan_path_cells);
					p_tmp_map_->print(curr.toCell(), CLEAN_MAP, plan_path_cells);
					return true;
				}
			}
		}
	}

	return !plan_path.empty();
}
