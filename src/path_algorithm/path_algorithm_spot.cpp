#include <ros/ros.h>
#include "robot.hpp"

SpotCleanPathAlgorithm::SpotCleanPathAlgorithm() {

}
bool find_between(const Point_t& curr, const Points::iterator& it, const Points::iterator& next_it)
{
	ROS_INFO("p_it(%d,%d)",it->toCell().x, it->toCell().y);
	if(it->dir == MAP_POS_X)
		return curr.x > next_it->x;
	else if(it->dir == MAP_NEG_X)
		return curr.x < next_it->x;
	else if(it->dir == MAP_POS_Y)
		return curr.y > next_it->y;
	else if(it->dir == MAP_NEG_Y)
		return curr.y < next_it->y;
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
		ROS_WARN("points_it(%d,%d)",points_it->toCell().x, points_it->toCell().y);
		Points::iterator p_it = points_it;
		if(!is_linear) {
			for (; p_it != plan_path.end(); ++p_it) {
				if (find_between(curr, p_it, std::next(p_it)))
				{
                    p_it++;
					break;
				}
			}
		}
		plan_path.erase(plan_path.begin(), p_it);
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
			return plan_path.size() > 3;
        else
			return !plan_path.empty();
	}
}

