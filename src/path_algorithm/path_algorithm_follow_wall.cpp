//
// Created by lsy563193 on 12/13/17.
//

#include "ros/ros.h"
#include "path_algorithm.h"
#include "pp.h"

bool WFCleanPathAlgorithm::generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir, Path_t &plan_path)
{
	if (plan_path.empty()) {//fw ->linear
		auto curr = curr_cell;
		curr.TH = 0;
		plan_path.push_back(curr);
//		if (g_wf_reach_count == 0 ||
//				(g_wf_reach_count < ISOLATE_COUNT_LIMIT && !fw_is_time_up()/*get_work_time() < WALL_FOLLOW_TIME*/ &&
//				 wf_is_isolate())) {
			fw_map.reset(CLEAN_MAP);
			auto angle = 0;
//			if (g_wf_reach_count == -900) {
//				angle = -900;
//			}
			const float FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
			auto point = nav_map.getCurrPoint();
			point.TH = ranged_angle(robot::instance()->getPoseAngle() + angle);
			nav_map.robotToCell(point, 0, FIND_WALL_DISTANCE * 1000, curr.X, curr.Y);
			plan_path.push_back(curr);
		ROS_INFO("%s,%d: path_next_linear(%d)", __FUNCTION__, __LINE__,point.TH);
//		}
		return true;
	}
	else//linear->fw
	{
		plan_path.empty();
		ROS_INFO("%s,%d: path_next_fw", __FUNCTION__, __LINE__);
		plan_path.push_back(curr_cell);
		return true;
	}
	return false;
}
