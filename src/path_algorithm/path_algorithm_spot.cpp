#include <ros/ros.h>
#include "robot.hpp"

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm() {

}

bool SpotCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr,  bool is_linear, Points &plan_path, Points::iterator& points_it, bool &is_close)
{
    if(!spot_running_)
	{
		Cells targets_cells{};
    	for(auto &&it : Cells{
			{0, 0},
			{2, 0},{2, 2},{-2, 2},{-2, -2},
			{3, -2},{3, 3},{-3, 3},{-3, -3},
			{4, -3},{4, 4},{-4, 4},{-4, -4},{4, -4},{4, 3},
			{-3, 3},{-3, -3},{3, -3},{3, 2},
			{-2, 2},{-2, -2},{2, -2},{2,0},
				{0,0}
		}) {
			targets_cells.emplace_back(it + curr.toCell());
		}
		plan_path = *cells_to_points(targets_cells);
    	spot_running_ = true;
        return true;
	} else {
//		ROS_WARN("points_it(%d,%d)",points_it->toCell().x, points_it->toCell().y);
		if(is_close)
		{
			is_close =false;
			ROS_ERROR_COND(points_it != plan_path.end(), "is isolate: points_it(%d,%d)",points_it->toCell().x, points_it->toCell().y);
//			auto d_cell = cell_direction_[points_it->dir];
			auto cell = points_it->toCell();
			plan_path.pop_front();
			points_it = std::find_if(plan_path.begin(), plan_path.end(),[&](const Point_t& p_it){// for the case {2,2},{3,3},{4,4} can't find other some cell ,so find instead of {3,2},{4,3}
				if(cell == Cell_t{3,3} || cell == Cell_t{4,4})
					return p_it.toCell() == points_it->toCell() -Cell_t{0,1};
				if(cell == Cell_t{4,-3} || cell == Cell_t{3,-2})
					return p_it.toCell() == points_it->toCell() - Cell_t{1,0};
				else if(cell == Cell_t{2,2})
					return p_it.toCell() == Cell_t{2,0};
				else{
					return p_it.toCell() == cell;
				}
			});

			ROS_WARN_COND(points_it != plan_path.end(), "is isolate: new points_it(%d,%d)",points_it->toCell().x, points_it->toCell().y);
		}
		plan_path.erase(plan_path.begin(), points_it);
        if(plan_path.size()>=5)
		{
            Cells cells{};
			for(auto&&it = plan_path.begin(); it!=plan_path.begin()+5; ++it)
			{
				cells.emplace_back(it->toCell());
			}
            map.printInRange(curr.toCell(),CLEAN_MAP,cells,true, BoundingBox2{{-10,-10},{10,10}});
		}
		if(is_linear)
		{
			ROS_INFO_COND(plan_path.size()<=4,"Try go Cell_(0,0) with dijstra");
			return plan_path.size() > 4;
		}
        else
			return !plan_path.empty();
	}
}

