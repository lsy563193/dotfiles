#include <ros/ros.h>
#include "robot.hpp"

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm() {

}

bool SpotCleanPathAlgorithm::generatePath(GridMap &map, const Point_t &curr,  bool is_linear, Points &plan_path, const Points::iterator& points_it)
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
		plan_path = *cells_generate_points(make_unique<Cells>(targets_cells));
    	spot_running_ = true;
        return true;
	} else{
//		ROS_WARN("points_it(%d,%d)",points_it->toCell().x, points_it->toCell().y);
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
			return plan_path.size() > 4;
        else
			return !plan_path.empty();
	}
}

