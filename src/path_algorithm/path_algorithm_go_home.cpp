//
// Created by lsy563193 on 12/13/17.
//

#include <map.h>
#include "ros/ros.h"
#include "path_algorithm.h"

GoHomePathAlgorithm::GoHomePathAlgorithm(GridMap& map, HomePointsManager *p_home_points_manage, bool is_follow_wall)
{
	p_home_points_manage_ = p_home_points_manage;
	p_home_points_manage->for_each([&](const Point_t& it){
		ROS_WARN("set Area Clean,%d,%d",it.toCell().x, it.toCell().y);
		map.setArea(it.toCell(), CLEANED, 1, 1);
	});

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
	ROS_INFO("%s,%d: init finish.",__FUNCTION__,__LINE__);
}

bool GoHomePathAlgorithm::generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path)
{
	plan_path.clear();
	Cells plan_path_cells{};
	map.print(curr.toCell(),CLEAN_MAP,Cells{});
	auto& hps_it = p_home_points_manage_->home_points_it();
	auto& hp_it = p_home_points_manage_->home_point_it();
	const auto& hps_list = p_home_points_manage_->home_points_list();
	for(;hps_it != hps_list.end(); ++hps_it) {
		ROS_INFO("%s,%d:go home point start home or rcon point" ,__FUNCTION__, __LINE__);
		if(hps_it->empty())
			break;

		if(way_it == home_ways.end())
		{
			way_it = home_ways.begin();
			hp_it = hps_it->begin();
		}

		for (; way_it != home_ways.end(); ++way_it) {

			way_it->get()->displayName();
			if(way_it->get()->isClearBlock()&& !has_clean_)
			{
				has_clean_ = true;
				way_it->get()->clearBlock(map);
			}

			if(hp_it == hps_it->end())
			{
				ROS_INFO("hp_it is empty !!!!(%d,%d)");
				hp_it = hps_it->begin();
			}

			for (; hp_it != hps_it->end(); ++(hp_it)) {
				ROS_INFO("hp_it(%d,%d)", hp_it->toCell().x, hp_it->toCell().y);
				auto p_tmp_map_ = way_it->get()->updateMap(map,temp_map, curr);

				if (dijkstra(*p_tmp_map_, curr.toCell(), plan_path_cells, true, CellEqual(hp_it->toCell()),
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
