//
// Created by lsy563193 on 12/13/17.
//

#include "ros/ros.h"
#include "path_algorithm.h"

Path_t WFCleanPathAlgorithm::generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir)
{
	Path_t path{};
//	if (move_type_i_ == mt_linear) {
//		ROS_INFO("%s,%d: path_next_fw", __FUNCTION__, __LINE__);
//		path.push_back(curr_cell);
//		path.push_back(g_virtual_target);
//	}
//	else {
//		path.push_back(curr_cell);
//		if (g_wf_reach_count == 0 ||
//				(g_wf_reach_count < ISOLATE_COUNT_LIMIT && !fw_is_time_up()/*get_work_time() < WALL_FOLLOW_TIME*/ &&
//				 wf_is_isolate())) {
//			fw_map.reset(CLEAN_MAP);
//			auto angle = 0;
//			if (g_wf_reach_count == -900) {
//				angle = 0;
//			}
//			const float FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
//			Cell_t cell;
//			auto point = nav_map.getCurrPoint();
//			point.TH = ranged_angle(robot::instance()->getPoseAngle() + angle);
//			nav_map.robotToCell(point, 0, FIND_WALL_DISTANCE * 1000, cell.X, cell.Y);
//			path.push_back(cell);
//		}
//	}
	return path;
}
