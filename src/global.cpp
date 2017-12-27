//
// Created by lsy563193 on 12/14/17.
//
#include "ros/ros.h"
#include <mathematics.h>
#include <deque>
#include <pp.h>
bool line_is_found;
int g_wf_reach_count;
bool g_check_path_in_advance;
Points g_homes;

Cells points_generate_cells(Points &targets)
{
//	displayCellPath(targets);
	Cells path{};
	for(const Point32_t& point : targets) {
		path.push_back(GridMap::pointToCell(point));
	}
	return path;
}

Points cells_generate_points(Cells &path)
{
//	displayCellPath(path);
	Points targets{};
	for(auto it = path.begin(); it < path.end(); ++it) {
		Point32_t target {GridMap::cellToCount((*it).X),GridMap::cellToCount((*it).Y),0};
		auto it_next = it+1;
		if (it->X == it_next->X)
			target.TH = it->Y > it_next->Y ? MAP_NEG_Y : MAP_POS_Y;
		else
			target.TH = it->X > it_next->X ? MAP_NEG_X : MAP_POS_X;
		targets.push_back(target);
	}
//		ROS_INFO("path.back(%d,%d,%d)",path.back().X, path.back().Y, path.back().TH);

	targets.back().TH = (targets.end()-2)->TH;
//	ROS_INFO("%s %d: path.back(%d,%d,%d), path.front(%d,%d,%d)", __FUNCTION__, __LINE__,
//					 path.back().X, path.back().Y, path.back().TH, path.front().X, path.front().Y, path.front().TH);
	return targets;
}

