//
// Created by lsy563193 on 12/13/17.
//

#include "ros/ros.h"
#include "path_algorithm.h"
#include "pp.h"

bool WFCleanPathAlgorithm::generatePath(GridMap &map, const Point32_t &curr, const MapDirection &last_dir, Points &targets)
{
	if (targets.empty()) {//fw ->linear
		auto curr = getPosition();
		curr.TH = 0;
		targets.push_back(curr);
//		if (g_wf_reach_count == 0 ||
//				(g_wf_reach_count < ISOLATE_COUNT_LIMIT && !fw_is_time_up()/*get_work_time() < WALL_FOLLOW_TIME*/ &&
//				 wf_is_isolate())) {
			fw_map.reset(CLEAN_MAP);
			auto angle = 0;
//			if (g_wf_reach_count == -900) {
//				angle = -900;
//			}
			const float FIND_WALL_DISTANCE = 8;//8 means 8 metres, it is the distance limit when the robot move straight to find wall
			auto point = getPosition();
			point.TH = ranged_angle(robot::instance()->getWorldPoseAngle() + angle);
			curr = point.getRelative(FIND_WALL_DISTANCE * 1000, 0);
			targets.push_back(curr);
		ROS_INFO("%s,%d: path_next_linear(%d)", __FUNCTION__, __LINE__,point.TH);
//		}
		return true;
	}
	else//linear->fw
	{
		targets.empty();
		ROS_INFO("%s,%d: path_next_fw", __FUNCTION__, __LINE__);
		targets.push_back(getPosition());
		return true;
	}
}
